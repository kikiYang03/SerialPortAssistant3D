#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <set>
#include <mutex>
#include <optional>
#include <fcntl.h>
#include <unistd.h>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <cstring>
#include <cerrno>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <filesystem>
#include <fstream>
#include <cmath>
#include <yaml-cpp/yaml.h>
#include <nlohmann/json.hpp>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

// ==== 引入 tf2_ros 相关头文件 ====
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>

#include "geometry_msgs/msg/pose_stamped.hpp"           
#include "visualization_msgs/msg/marker.hpp"            
#include "quadrotor_msgs/msg/position_command.hpp"      

using json = nlohmann::json;
using namespace std::chrono_literals;
namespace fs = std::filesystem;

/* -------------------- base64 -------------------- */
static const char kB64Table[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

static std::string base64Encode(const uint8_t* data, size_t len)
{
    std::string out;
    out.reserve(((len + 2) / 3) * 4);
    size_t i = 0;
    while (i + 3 <= len) {
        uint32_t v = (data[i] << 16) | (data[i+1] << 8) | data[i+2];
        out.push_back(kB64Table[(v >> 18) & 0x3F]);
        out.push_back(kB64Table[(v >> 12) & 0x3F]);
        out.push_back(kB64Table[(v >>  6) & 0x3F]);
        out.push_back(kB64Table[(v >>  0) & 0x3F]);
        i += 3;
    }
    const size_t rem = len - i;
    if (rem == 1) {
        uint32_t v = (data[i] << 16);
        out.push_back(kB64Table[(v >> 18) & 0x3F]);
        out.push_back(kB64Table[(v >> 12) & 0x3F]);
        out.push_back('='); out.push_back('=');
    } else if (rem == 2) {
        uint32_t v = (data[i] << 16) | (data[i+1] << 8);
        out.push_back(kB64Table[(v >> 18) & 0x3F]);
        out.push_back(kB64Table[(v >> 12) & 0x3F]);
        out.push_back(kB64Table[(v >>  6) & 0x3F]);
        out.push_back('=');
    }
    return out;
}

/* ======================== 节点 ======================== */
class TcpSender : public rclcpp::Node
{
public:
    TcpSender() : Node("tcp_sender"), listen_sock_(-1)
    {
        package_share_dir_ = ament_index_cpp::get_package_share_directory("upper");

        loadParamsFromYaml();

        this->declare_parameter<std::string>("tcp_ip", "0.0.0.0");
        this->declare_parameter<int>("tcp_port", 6666);
        tcp_ip_ = this->get_parameter("tcp_ip").as_string();
        tcp_port_ = this->get_parameter("tcp_port").as_int();

        startTcpServer();

        // ==== 初始化 TF Buffer 和 Listener ====
        // Listener 会在后台自动订阅 /tf 和 /tf_static 话题，并维护一棵完整的坐标树
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        /* 根据模块类型动态订阅数据 (0/1=3D, 2/3=2D) */
        if (param_cache_.module_type < 2) {
            RCLCPP_INFO(this->get_logger(), "当前为 3D 模式，订阅点云话题");
            cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                "/cloud_registered", rclcpp::QoS(10),
                [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                    std::lock_guard<std::mutex> lk(cloud_mtx_);
                    latest_cloud_ = msg;
                });
            cloud_10hz_timer_ = this->create_wall_timer(100ms, [this](){ sendCloudLoop(); });
        } else {
            RCLCPP_INFO(this->get_logger(), "当前为 2D 模式，订阅 LaserScan 和 Map 话题");
            scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
                "/scan", rclcpp::QoS(10),
                [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
                    std::lock_guard<std::mutex> lk(scan_mtx_);
                    latest_scan_ = msg;
                });
            map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
                "/map", rclcpp::QoS(1),
                [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
                    std::lock_guard<std::mutex> lk(map_mtx_);
                    latest_map_ = msg;
                });
            scan_10hz_timer_ = this->create_wall_timer(100ms, [this](){ sendScanLoop(); });
            map_1hz_timer_   = this->create_wall_timer(1000ms, [this](){ sendMapLoop(); });
        }

        /* Ego-planner 相关订阅发布 */
        goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/move_base_simple/goal", 10);
        pos_cmd_sub_ = this->create_subscription<quadrotor_msgs::msg::PositionCommand>(
            "/drone_0_planning/pos_cmd", rclcpp::QoS(10),
            [this](const quadrotor_msgs::msg::PositionCommand::SharedPtr msg) {
                std::lock_guard<std::mutex> lk(pos_cmd_mtx_);
                latest_pos_cmd_ = msg;
            });
        optimal_list_sub_ = this->create_subscription<visualization_msgs::msg::Marker>(
            "/drone_0_plan_vis/optimal_list", rclcpp::QoS(10),
            [this](const visualization_msgs::msg::Marker::SharedPtr msg) {
                std::lock_guard<std::mutex> lk(optimal_list_mtx_);
                latest_optimal_list_ = msg;
            });

        /* 系统定时器 */
        tcp_accept_timer_ = this->create_wall_timer(200ms, std::bind(&TcpSender::acceptPendingClients, this));
        tcp_recv_timer_   = this->create_wall_timer(100ms, std::bind(&TcpSender::checkTcpRecv, this));
        rate_timer_       = this->create_wall_timer(10s, std::bind(&TcpSender::logRates, this));
        
        // TF发送定时器，以10Hz主动去查询所需的位姿
        tf_10hz_timer_    = this->create_wall_timer(100ms, [this](){ sendTfLoop(); });
        ego_10hz_timer_   = this->create_wall_timer(100ms, [this](){ sendEgoPlannerLoop(); });
    }

    ~TcpSender()
    {
        if (listen_sock_ != -1) close(listen_sock_);
        for (int c : clients_) close(c);
    }

private:
    /* ---------- 参数缓存 ---------- */
    struct ParamCache {
        bool dirty = false;
        int16_t module_type = 0;     // 0x99: 0=3D定位, 1=3D导航, 2=2D定位, 3=2D导航
        int16_t lidar_name = 0;      // 0x00: 0=mid360, 10=N10, 11=N10_P
        int16_t x = 0, y = 0, z = 0;
        int16_t roll = 0, pitch = 0, yaw = 0;
        int16_t px4_flag = 1;
        int16_t uart_flag = 1;
        int16_t max_vel = 100;
        int16_t max_acc = 100;
        int16_t virtual_ceil_height = 200;
        int16_t obstacles_inflation = 30;
        int16_t dist0 = 40;
        int16_t depth_filter_mindist = 30;
    } param_cache_;
    std::mutex param_cache_mtx_;

    /* -------------------- 从 YAML 加载参数 -------------------- */
    void loadParamsFromYaml()
    {
        std::string file_path = package_share_dir_ + "/config/params.yaml";
        if (!fs::exists(file_path)) {
            RCLCPP_WARN(this->get_logger(), "参数文件不存在，使用默认参数。");
            return;
        }

        try {
            YAML::Node cfg = YAML::LoadFile(file_path);
            std::lock_guard<std::mutex> lk(param_cache_mtx_);

            if (cfg["module_type"]) param_cache_.module_type = cfg["module_type"].as<int16_t>();
            if (cfg["lidar_name"])  param_cache_.lidar_name  = cfg["lidar_name"].as<int16_t>();
            
            if (cfg["x"]) param_cache_.x = cfg["x"].as<int16_t>();
            if (cfg["y"]) param_cache_.y = cfg["y"].as<int16_t>();
            if (cfg["z"]) param_cache_.z = cfg["z"].as<int16_t>();
            if (cfg["roll"])  param_cache_.roll  = cfg["roll"].as<int16_t>();
            if (cfg["pitch"]) param_cache_.pitch = cfg["pitch"].as<int16_t>();
            if (cfg["yaw"])   param_cache_.yaw   = cfg["yaw"].as<int16_t>();
            if (cfg["px4_flag"])  param_cache_.px4_flag  = cfg["px4_flag"].as<int16_t>();
            if (cfg["uart_flag"]) param_cache_.uart_flag = cfg["uart_flag"].as<int16_t>();

            if (cfg["max_vel"]) param_cache_.max_vel = static_cast<int16_t>(cfg["max_vel"].as<float>() * 100);
            if (cfg["max_acc"]) param_cache_.max_acc = static_cast<int16_t>(cfg["max_acc"].as<float>() * 100);
            if (cfg["virtual_ceil_height"]) param_cache_.virtual_ceil_height = static_cast<int16_t>(cfg["virtual_ceil_height"].as<float>() * 100);
            if (cfg["obstacles_inflation"]) param_cache_.obstacles_inflation = static_cast<int16_t>(cfg["obstacles_inflation"].as<float>() * 100);
            if (cfg["dist0"]) param_cache_.dist0 = static_cast<int16_t>(cfg["dist0"].as<float>() * 100);
            if (cfg["depth_filter_mindist"]) param_cache_.depth_filter_mindist = static_cast<int16_t>(cfg["depth_filter_mindist"].as<float>() * 100);

            RCLCPP_INFO(this->get_logger(), "已加载参数，当前模块类型: %d, 雷达型号: %d", 
                        param_cache_.module_type, param_cache_.lidar_name);
        } catch (const YAML::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "解析 YAML 文件失败: %s", e.what());
        }
    }

    /* -------------------- TCP 服务器 -------------------- */
    void startTcpServer()
    {
        listen_sock_ = socket(AF_INET, SOCK_STREAM, 0);
        if (listen_sock_ < 0) return;
        int opt = 1;
        setsockopt(listen_sock_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port   = htons(tcp_port_);
        addr.sin_addr.s_addr = inet_addr(tcp_ip_.c_str());
        if (bind(listen_sock_, (struct sockaddr *)&addr, sizeof(addr)) < 0) return;
        listen(listen_sock_, 5);
        fcntl(listen_sock_, F_SETFL, O_NONBLOCK);
        RCLCPP_INFO(this->get_logger(), "TCP 监听 %s:%d", tcp_ip_.c_str(), tcp_port_);
    }

    void acceptPendingClients()
    {
        sockaddr_in caddr{};
        socklen_t clen = sizeof(caddr);
        int fd = accept(listen_sock_, (struct sockaddr *)&caddr, &clen);
        if (fd < 0) return;
        fcntl(fd, F_SETFL, O_NONBLOCK);
        clients_.insert(fd);
    }

    /* -------------------- 发送 TF 数据 (基于 tf2_ros) -------------------- */
    void sendTfLoop()
    {
        // 1. 获取并发送机器人位姿 (map -> base_link)
        try {
            // lookupTransform 参数: 目标系, 源系, 时间(TimePointZero表示获取最新的可用TF)
            geometry_msgs::msg::TransformStamped t_robot = tf_buffer_->lookupTransform(
                "map", "base_link", tf2::TimePointZero);
            sendTfJson(t_robot, 0x01, "tf_robot");
        } catch (const tf2::TransformException & ex) {
            // 如果 TF 还没建立完成，这里会捕获异常，防止节点崩溃。
            // 屏蔽报错日志防止刷屏，取消注释可用于调试：
            // RCLCPP_DEBUG(this->get_logger(), "获取机器人位姿失败: %s", ex.what());
        }

        // 2. 获取并发送雷达位姿 (3D: map -> body, 2D: map -> laser_link)
        try {
            std::string lidar_frame = (param_cache_.module_type < 2) ? "body" : "laser_link";
            geometry_msgs::msg::TransformStamped t_lidar = tf_buffer_->lookupTransform(
                "map", lidar_frame, tf2::TimePointZero);
            sendTfJson(t_lidar, 0x02, "tf_lidar");
        } catch (const tf2::TransformException & ex) {
            // 同上
        }
    }

    void sendTfJson(const geometry_msgs::msg::TransformStamped &t, uint8_t topic_id, const std::string& topic_name)
    {
        json j;
        j["topic"]          = topic_name;
        j["frame_id"]       = t.header.frame_id;
        j["child_frame_id"] = t.child_frame_id;
        j["x"]  = t.transform.translation.x;
        j["y"]  = t.transform.translation.y;
        j["z"]  = t.transform.translation.z;
        j["qx"] = t.transform.rotation.x;
        j["qy"] = t.transform.rotation.y;
        j["qz"] = t.transform.rotation.z;
        j["qw"] = t.transform.rotation.w;
        sendJsonPacket(topic_id, j.dump());
    }

    /* -------------------- 发送 3D/2D 传感器数据 -------------------- */
    void sendCloudLoop()
    {
        std::lock_guard<std::mutex> lk(cloud_mtx_);
        if (!latest_cloud_) return;
        json j;
        j["topic"]      = "cloud_registered";
        j["frame_id"]   = latest_cloud_->header.frame_id;
        j["width"]      = latest_cloud_->width;
        j["height"]     = latest_cloud_->height;
        j["point_step"] = latest_cloud_->point_step;
        j["row_step"]   = latest_cloud_->row_step;
        j["is_dense"]   = latest_cloud_->is_dense;
        j["data"]       = base64Encode(latest_cloud_->data.data(), latest_cloud_->data.size());
        sendJsonPacket(0x03, j.dump());
    }

    void sendScanLoop()
    {
        std::lock_guard<std::mutex> lk(scan_mtx_);
        if (!latest_scan_ || latest_scan_->ranges.empty()) return;
        json j;
        j["topic"] = "scan";
        j["angle_min"] = latest_scan_->angle_min;
        j["angle_max"] = latest_scan_->angle_max;
        j["angle_increment"] = latest_scan_->angle_increment;
        j["range_count"] = latest_scan_->ranges.size();
        j["ranges"] = latest_scan_->ranges;
        sendJsonPacket(0x05, j.dump());
    }

    void sendMapLoop()
    {
        std::lock_guard<std::mutex> lk(map_mtx_);
        if (!latest_map_ || latest_map_->data.empty()) return;
        json j;
        j["topic"] = "map";
        j["width"] = latest_map_->info.width;
        j["height"] = latest_map_->info.height;
        j["resolution"] = latest_map_->info.resolution;
        j["origin_x"] = latest_map_->info.origin.position.x;
        j["origin_y"] = latest_map_->info.origin.position.y;

        std::vector<std::pair<int, int>> rle;
        const auto &data = latest_map_->data;
        int current = data[0];
        int count = 1;
        for (size_t i = 1; i < data.size(); ++i) {
            if (data[i] == current && count < 255) count++;
            else {
                rle.emplace_back(current, count);
                current = data[i];
                count = 1;
            }
        }
        rle.emplace_back(current, count);

        json rle_json = json::array();
        for (auto &p : rle) rle_json.push_back({p.first, p.second});
        j["rle"] = rle_json;

        sendJsonPacket(0x06, j.dump());
    }

    void sendEgoPlannerLoop()
    {
        {
            std::lock_guard<std::mutex> lk(pos_cmd_mtx_);
            if (latest_pos_cmd_) {
                json j;
                j["topic"] = "pos_cmd";
                j["x"]     = latest_pos_cmd_->position.x;
                j["y"]     = latest_pos_cmd_->position.y;
                j["z"]     = latest_pos_cmd_->position.z;
                j["yaw"]   = latest_pos_cmd_->yaw;
                sendJsonPacket(0x08, j.dump()); // 统一协议: 0x08 pos_cmd
            }
        }
        {
            std::lock_guard<std::mutex> lk(optimal_list_mtx_);
            if (latest_optimal_list_) {
                json j;
                j["topic"] = "optimal_list";
                json points = json::array();
                for (const auto& p : latest_optimal_list_->points) {
                    points.push_back({{"x", p.x}, {"y", p.y}, {"z", p.z}});
                }
                j["points"] = points;
                sendJsonPacket(0x09, j.dump()); // 统一协议: 0x09 optimal_list
            }
        }
    }

    /* -------------------- 接收与指令解析 -------------------- */
    void checkTcpRecv()
    {
        uint8_t buf[8192];
        for (auto it = clients_.begin(); it != clients_.end();)
        {
            ssize_t n = recv(*it, buf, sizeof(buf), 0);
            if (n <= 0){ ++it; continue; }
            bytes_recv_tcp_ += n;

            bool save_pending = false;
            for (ssize_t i = 0; i < n - 3; ++i) {
                // 测试: AA 00 01 0A
                if (buf[i] == 0xAA && buf[i+1] == 0x00 && buf[i+2] == 0x01 && buf[i+3] == 0x0A) {
                    uint8_t reply[4] = {0xAA, 0x00, 0x01, 0x0A};
                    send(*it, reply, 4, 0);
                }
                // 保存地图: AA 00 02 0A
                if (buf[i] == 0xAA && buf[i+1] == 0x00 && buf[i+2] == 0x02 && buf[i+3] == 0x0A) {
                    handleMapSaveCommand();
                }
                // 读取参数: AA 00 03 0A
                if (buf[i] == 0xAA && buf[i+1] == 0x00 && buf[i+2] == 0x03 && buf[i+3] == 0x0A) {
                    handleSendParamsCommand();
                }
                // 保存参数: AA 00 04 0A
                if (buf[i] == 0xAA && buf[i+1] == 0x00 && buf[i+2] == 0x04 && buf[i+3] == 0x0A) {
                    save_pending = true;
                }
            }

            // 修改参数: AA 10 ID H L 0A
            for (ssize_t i = 0; i < n - 5; ++i) {
                if (buf[i] == 0xAA && buf[i+1] == 0x10 && buf[i+5] == 0x0A) {
                    uint8_t param_id = buf[i+2];
                    int16_t value    = (buf[i+3] << 8) | buf[i+4];
                    handleParamCommand(param_id, value);
                }
            }

            if (save_pending) handleSaveParamsCommand();

            // 目标点解析: 统一协议 0x07 (AA 07 [JSON] 0A)
            for (ssize_t i = 0; i < n - 1; ++i) {
                if (buf[i] == 0xAA && buf[i+1] == 0x07) {
                    ssize_t end_idx = -1;
                    for (ssize_t j = i + 2; j < n; ++j) {
                        if (buf[j] == 0x0A) { end_idx = j; break; }
                    }
                    if (end_idx != -1) {
                        std::string json_str(reinterpret_cast<char*>(&buf[i+2]), end_idx - (i+2));
                        handleGoalJson(json_str);
                        i = end_idx; 
                    }
                }
            }
            ++it;
        }
    }

    void handleGoalJson(const std::string& json_str)
    {
        try {
            auto j = json::parse(json_str);
            geometry_msgs::msg::PoseStamped goal;
            goal.header.stamp = this->now();
            goal.header.frame_id = "map"; 
            goal.pose.position.x = j.value("x", 0.0);
            goal.pose.position.y = j.value("y", 0.0);
            goal.pose.position.z = j.value("z", 0.0);
            tf2::Quaternion q;
            q.setRPY(0, 0, j.value("yaw", 0.0));
            goal.pose.orientation.x = q.x(); goal.pose.orientation.y = q.y();
            goal.pose.orientation.z = q.z(); goal.pose.orientation.w = q.w();
            goal_pub_->publish(goal);
        } catch (...) {}
    }

    /* -------------------- 2D 地图保存 -------------------- */
    void handleMapSaveCommand()
    {
        if (param_cache_.module_type < 2) return; // 仅 2D 支持
        std::lock_guard<std::mutex> lk(map_mtx_);
        if (!latest_map_) return;

        std::string dir_path = package_share_dir_ + "/map/";
        if (!fs::exists(dir_path)) fs::create_directories(dir_path);

        std::string yaml_path = dir_path + "my_map.yaml";
        std::string pgm_path  = dir_path + "my_map.pgm";

        std::ofstream ofs(pgm_path, std::ios::binary);
        if (ofs.is_open()) {
            int width = latest_map_->info.width;
            int height = latest_map_->info.height;
            ofs << "P5\n" << width << " " << height << "\n255\n";
            for (int y = 0; y < height; ++y) {
                for (int x = 0; x < width; ++x) {
                    int8_t val = latest_map_->data[y * width + x];
                    uint8_t pgm_val = (val == -1) ? 205 : static_cast<uint8_t>((100 - val) * 254 / 100);
                    ofs << pgm_val;
                }
            }
            ofs.close();
        }

        std::ofstream yfs(yaml_path);
        if (yfs.is_open()) {
            yfs << "image: my_map.pgm\n"
                << "resolution: " << latest_map_->info.resolution << "\n"
                << "origin: [" << latest_map_->info.origin.position.x << ", "
                               << latest_map_->info.origin.position.y << ", "
                               << latest_map_->info.origin.position.z << "]\n"
                << "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n";
            yfs.close();
        }
        RCLCPP_INFO(this->get_logger(), "2D Map 保存成功");
    }

    /* -------------------- 参数管理 -------------------- */
    void handleSendParamsCommand()
    {
        std::lock_guard<std::mutex> lk(param_cache_mtx_);
        sendParamToClients(0x00, param_cache_.lidar_name);
        sendParamToClients(0x01, param_cache_.x);
        sendParamToClients(0x02, param_cache_.y);
        sendParamToClients(0x03, param_cache_.z);
        sendParamToClients(0x04, param_cache_.roll);
        sendParamToClients(0x05, param_cache_.pitch);
        sendParamToClients(0x06, param_cache_.yaw);
        sendParamToClients(0x11, param_cache_.px4_flag);
        sendParamToClients(0x12, param_cache_.uart_flag);
        sendParamToClients(0x21, param_cache_.max_vel);
        sendParamToClients(0x22, param_cache_.max_acc);
        sendParamToClients(0x23, param_cache_.virtual_ceil_height);
        sendParamToClients(0x24, param_cache_.obstacles_inflation);
        sendParamToClients(0x25, param_cache_.dist0);
        sendParamToClients(0x26, param_cache_.depth_filter_mindist);
        sendParamToClients(0x99, param_cache_.module_type); // 发送模块类型
    }

    void handleParamCommand(uint8_t param_id, int16_t value)
    {
        std::lock_guard<std::mutex> lk(param_cache_mtx_);
        switch (param_id) {
            case 0x00: param_cache_.lidar_name = value; break;
            case 0x01: param_cache_.x = value; break;
            case 0x02: param_cache_.y = value; break;
            case 0x03: param_cache_.z = value; break;
            case 0x04: param_cache_.roll = value; break;
            case 0x05: param_cache_.pitch = value; break;
            case 0x06: param_cache_.yaw = value; break;
            case 0x11: param_cache_.px4_flag = value; break;
            case 0x12: param_cache_.uart_flag = value; break;
            case 0x21: param_cache_.max_vel = value; break;
            case 0x22: param_cache_.max_acc = value; break;
            case 0x23: param_cache_.virtual_ceil_height = value; break;
            case 0x24: param_cache_.obstacles_inflation = value; break;
            case 0x25: param_cache_.dist0 = value; break;
            case 0x26: param_cache_.depth_filter_mindist = value; break;
            case 0x99: param_cache_.module_type = value; break;
            default: return;
        }
        param_cache_.dirty = true;
    }

    void handleSaveParamsCommand()
    {
        std::lock_guard<std::mutex> lk(param_cache_mtx_);
        if (!param_cache_.dirty) return;

        const std::string file_path = fs::path(package_share_dir_) / "config" / "params.yaml";
        std::ofstream fout(file_path);
        fout << std::fixed << std::setprecision(2); 
        
        fout << "# 模块类型: 0=3D定位, 1=3D导航, 2=2D定位, 3=2D导航\n"
             << "module_type: " << param_cache_.module_type << "\n\n"
             << "# 雷达型号: 0=mid360, 10=N10, 11=N10_P\n"
             << "lidar_name: " << param_cache_.lidar_name << "\n\n"
             << "# 外参偏移 (单位cm和度)\n"
             << "x: " << param_cache_.x << "\ny: " << param_cache_.y << "\nz: " << param_cache_.z << "\n"
             << "roll: " << param_cache_.roll << "\npitch: " << param_cache_.pitch << "\nyaw: " << param_cache_.yaw << "\n\n"
             << "# 通信使能标志\n"
             << "px4_flag: " << param_cache_.px4_flag << "\nuart_flag: " << param_cache_.uart_flag << "\n\n"
             << "# egoplanner 规划参数\n"
             << "max_vel: " << (param_cache_.max_vel / 100.0f) << "\n"
             << "max_acc: " << (param_cache_.max_acc / 100.0f) << "\n"
             << "virtual_ceil_height: " << (param_cache_.virtual_ceil_height / 100.0f) << "\n"
             << "obstacles_inflation: " << (param_cache_.obstacles_inflation / 100.0f) << "\n"
             << "dist0: " << (param_cache_.dist0 / 100.0f) << "\n"
             << "depth_filter_mindist: " << (param_cache_.depth_filter_mindist / 100.0f) << "\n";
        fout.close();
        
        param_cache_.dirty = false;
        sync();
        system("poweroff");
    }

    /* -------------------- 发送工具 -------------------- */
    void sendParamToClients(uint8_t param_id, int16_t value) {
        std::vector<uint8_t> pkt = {0xAA, 0x10, param_id, static_cast<uint8_t>((value >> 8) & 0xFF), static_cast<uint8_t>(value & 0xFF), 0x0A};
        sendPacket(pkt);
    }
    void sendJsonPacket(uint8_t topic_id, const std::string &json_str) {
        std::vector<uint8_t> pkt = {0xAA, topic_id};
        pkt.insert(pkt.end(), json_str.begin(), json_str.end());
        pkt.push_back(0x0A);
        sendPacket(pkt);
    }
    void sendPacket(const std::vector<uint8_t> &pkt) {
        for (auto it = clients_.begin(); it != clients_.end();) {
            if (send(*it, pkt.data(), pkt.size(), 0) <= 0) { close(*it); it = clients_.erase(it); }
            else { bytes_sent_tcp_ += pkt.size(); ++it; }
        }
    }
    void logRates() { RCLCPP_INFO(this->get_logger(), "TCP send: %lu B, recv: %lu B", bytes_sent_tcp_, bytes_recv_tcp_); }

    /* ======================== 成员 ======================== */
    std::string tcp_ip_; int tcp_port_; int listen_sock_; std::set<int> clients_;
    std::string package_share_dir_;
    size_t bytes_sent_tcp_ = 0, bytes_recv_tcp_ = 0;

    // ==== TF 监听与缓存 ====
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    std::mutex cloud_mtx_, scan_mtx_, map_mtx_, pos_cmd_mtx_, optimal_list_mtx_;
    sensor_msgs::msg::PointCloud2::SharedPtr latest_cloud_;
    sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
    nav_msgs::msg::OccupancyGrid::SharedPtr latest_map_;
    quadrotor_msgs::msg::PositionCommand::SharedPtr latest_pos_cmd_;
    visualization_msgs::msg::Marker::SharedPtr latest_optimal_list_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
    rclcpp::Subscription<quadrotor_msgs::msg::PositionCommand>::SharedPtr pos_cmd_sub_;
    rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr optimal_list_sub_;

    rclcpp::TimerBase::SharedPtr tcp_accept_timer_, tcp_recv_timer_, rate_timer_;
    rclcpp::TimerBase::SharedPtr tf_10hz_timer_, cloud_10hz_timer_, scan_10hz_timer_, map_1hz_timer_, ego_10hz_timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TcpSender>());
    rclcpp::shutdown();
    return 0;
}