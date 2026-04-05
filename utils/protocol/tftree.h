#ifndef TFTREE_H
#define TFTREE_H

#include <QString>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <optional>
#include <mutex>

using Vec3 = Eigen::Vector3d;
using Quat = Eigen::Quaterniond;
using Mat4 = Eigen::Matrix4d;

class TfTree
{
public:
    // 设置变换矩阵
    void setMapToBaseLink(const Mat4& T);
    void setMapToLaser(const Mat4& T);

    // 获取变换矩阵
    std::optional<Mat4> getMapToBaseLink() const;
    std::optional<Mat4> getMapToLaser() const;

    // 检查是否有有效变换
    bool hasMapToBaseLink() const;
    bool hasMapToLaser() const;

    // 清空所有变换
    void clear();

private:
    mutable std::mutex m_;
    std::optional<Mat4> T_map_base_link_;  // map → base_link (机器人位姿)
    std::optional<Mat4> T_map_laser_;       // map → laser (雷达位姿)
};
#endif // TFTREE_H