#ifndef GLWIDGET_H
#define GLWIDGET_H

#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QOpenGLBuffer>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLShaderProgram>

#include <QMutex>
#include <atomic>
#include <deque>

#include "tftree.h"
#include "protocol_msg.h"
#include "adminmode.h"
#include <unordered_set>   // ← 让 std::unordered_set 可见
#include <cstdint>         // ← uint64_t
// 轨迹缓存
struct Trail {
    std::deque<Eigen::Vector3f> points;
    Eigen::Matrix4d             latestTransform;
    bool                        hasValidTransform = false;
};

class GLWidget : public QOpenGLWidget, protected QOpenGLFunctions
{
    Q_OBJECT
public:
    explicit GLWidget(QWidget *parent = nullptr);
    ~GLWidget();

    // 换成法线配色
    QVector3D normalToColor(const Eigen::Vector3f& n);
    // 实时切换方案
    enum ColorMode { Height, Normal };
    void setColorMode(ColorMode m) { colorMode_ = m; update(); }

    void setShowRealtimeCloud(bool show);   // 实时点云开关
    void setShowMapCloud(bool show);        // 地图点云开关
    void setShowScan2D(bool show);          // 2D激光雷达开关
    void setShowMap2D(bool show);           // 2D地图开关
    void setMap2DPointSize(float size);     // 设置地图点大小
    void setMap2DStep(int step);            // 设置地图采样步长

    void setZFilterRange(float minZ, float maxZ);
    void setZFilterEnabled(bool enabled);

    void setNavMode(bool enable);


signals:
    void tfInfoChanged(double x, double y, double z,
                       int yaw_deg, int pitch_deg, int roll_deg);   //显示TF相关数据
    // 添加消息显示信号
    void appendMessage(const QString &message);

    void navGoalSet(double x, double y, double z, double yaw);
    void navModeChanged(bool isNavMode);

    // 通知UI更新目标点数值
    void navTargetUpdated(double x, double y, double z, double yaw_deg);

public slots:
    void onRobotPose(const RobotPoseMsg &);  // 机器人位姿 (map → base_link)
    void onLidarPose(const LidarPoseMsg &);  // 雷达位姿 (map → laser)
    void onCloud(const CloudMsg &);
    void onScan2D(const Scan2DMsg &);        // 2D激光雷达数据 (0x05)
    void onMap2D(const Map2DMsg &);          // 2D地图数据 (0x06)

    void clearMap();        // 清理点云地图
    void clearTrail();  // 新增
    void clearCloud();  // 新增
    void setCamera();     // 初始化相机位置
    void saveMapToFile();   // 保存地图


    void addYaw  (int degrees);   // 正数右转，负数左转
    void addPitch(int degrees);   // 正数下俯，负数上仰

    // 接收UI微调框传来的目标点数值
    void updateNavTargetFromUI(double x, double y, double z, double yaw_deg);

    void onGoalPath(const PathMsg &m);

private slots:   // 新增
    void doUploadCloud();   // 在主线程里把 cloudCpu_ 塞进 vboCloud_
    void doUploadMap();     // 在主线程里把 mapInterleavedCpu_ 塞进 vboMap_
    void doUploadScan2D();  // 在主线程里把 scan2DCpu_ 塞进 vboScan2D_
    void doUploadMap2D();   // 在主线程里把 map2DCpu_ 塞进 vboMap2D_

protected:
    void initializeGL() override;
    void resizeGL(int w, int h) override;
    void paintGL() override;

    void mouseReleaseEvent(QMouseEvent *e) override;

private:

    // =========== 新协议TF相关 ===========
    Eigen::Matrix4d T_map_base_link_;    // map → base_link (机器人位姿)
    Eigen::Matrix4d T_map_laser_;        // map → laser (雷达位姿)
    bool hasRobotPose_ = false;          // 是否收到机器人位姿
    bool hasLidarPose_ = false;          // 是否收到雷达位姿

    // 修改这行：
    Eigen::Vector3d screenToWorld(const QPoint& pos, double targetZ);

    bool isNavMode_ = false;
    bool hasNavTarget_ = false;  // 是否已经生成了箭头
    bool isNavGoalSet_ = false;  // 是否已经最终确认了目标（退出指点模式后）

    // 👇 新增：记录是否正在按住左键拖动设定朝向
    bool isDraggingNavGoal_ = false;

    Eigen::Vector3d navTarget3D_{0, 0, 1}; // 目标坐标，Z默认1m
    double navYaw_ = 0.0;

    ColorMode colorMode_ = Height;
    TfTree tf_;

    // 点云大小
    float cloudPtSize_ = 3.0f;
    float mapPtSize_   = 3.0f;

    // ---------------- CPU 缓存 + dirty 标志 ----------------
    QMutex dataMtx_;

    std::vector<Eigen::Vector3f> cloudCpu_;
    std::vector<Eigen::Vector3f> mapInterleavedCpu_;

    std::atomic_bool cloudDirty_{false};
    std::atomic_bool mapDirty_{false};

    // GPU 对象
    QOpenGLBuffer vboCloud_{QOpenGLBuffer::VertexBuffer};
    QOpenGLBuffer vboMap_  {QOpenGLBuffer::VertexBuffer};
    QOpenGLVertexArrayObject vaoCloud_, vaoMap_;
    int cloudPts_ = 0, mapPts_ = 0;

    // 2D数据 GPU对象
    std::vector<Eigen::Vector3f> scan2DCpu_;
    std::vector<Eigen::Vector3f> map2DCpu_;       // 位置+颜色交错存储
    QOpenGLBuffer vboScan2D_{QOpenGLBuffer::VertexBuffer};
    QOpenGLBuffer vboMap2D_{QOpenGLBuffer::VertexBuffer};
    QOpenGLVertexArrayObject vaoScan2D_, vaoMap2D_;
    int scan2DPts_ = 0, map2DPts_ = 0;
    std::atomic_bool scan2DDirty_{false};
    std::atomic_bool map2DDirty_{false};

    QOpenGLShaderProgram progSimple_;
    QOpenGLShaderProgram progColorCloud_;

    Eigen::Matrix4d proj_, view_;

    // 相机控制
    float distance_ = 30.0f;
    float pitch_ = -45.0f;
    float yaw_ = 180.0f;
    QVector3D center_{0, 0, 0};
    QPoint lastMousePos_;

    Trail trail_;
    static constexpr size_t kMaxTrail = 5000;

    // 地图高度范围（用于颜色）
    float mapMinZ_ = 0.0f;
    float mapMaxZ_ = 0.0f;

    /* 去重用：空间哈希表 */
    std::unordered_set<uint64_t> mapVoxelSet_;

    // 工具函数
    static QMatrix4x4 toQMatrix(const Eigen::Matrix4d &m);
    void drawAxis(const Eigen::Matrix4d &T, float len = 5.f);
    void drawGrid(const Eigen::Matrix4d &T, int cells = 20, float step = 1.f);
    void drawSolidArrow(const Eigen::Matrix4d &T, float len, float radius = 0.08f);
    QVector3D heightToColor(float z, float minZ, float maxZ);

    // 鼠标事件
    void mousePressEvent(QMouseEvent *e) override;
    void mouseMoveEvent(QMouseEvent *e) override;
    void wheelEvent(QWheelEvent *e) override;

    QOpenGLBuffer vboAxis_;
    QOpenGLVertexArrayObject vaoAxis_;

    QOpenGLBuffer vboGrid_;
    QOpenGLVertexArrayObject vaoGrid_;

    QOpenGLBuffer vboTrail_;
    QOpenGLVertexArrayObject vaoTrail_;

    QOpenGLBuffer vboArrow_;
    QOpenGLVertexArrayObject vaoArrow_;

    void createArrowGeometry();
    void createTrailVAO();
    void createAxisGeometry();
    void createGridGeometry();

    bool glReady_ = false;

    bool showRealtimeCloud_ = true;         // 默认显示
    bool showMapCloud_      = true;

    // 2D数据显示控制
    bool showScan2D_ = true;
    bool showMap2D_  = true;
    float map2DPointSize_ = 6.0f;  // 2D地图点大小
    int map2DStep_ = 1;             // 2D地图采样步长 (1=全部, 2=隔1取1, 3=隔2取1)


    // Z轴范围控制
    float zMinFilter_ = -10.0f;   // 默认最小Z值
    float zMaxFilter_ =  10.0f;   // 默认最大Z值
    bool  enableZFilter_ = false; // 是否启用Z轴过滤

    // 添加新的过滤点云渲染方法
    void drawFilteredPoints(QOpenGLShaderProgram &program,
                            QOpenGLVertexArrayObject &vao,
                            QOpenGLBuffer &vbo,
                            const std::vector<Eigen::Vector3f>& points,
                            int pointCount,
                            float pointSize,
                            const QVector3D& color = QVector3D(1,1,1),
                            bool useColorProgram = false);

    std::vector<Eigen::Vector3f> optimalPathPts_; // 存储规划路径点
    QOpenGLBuffer vboOptimalPath_;
    QOpenGLVertexArrayObject vaoOptimalPath_;
    bool hasOptimalPath_ = false;

    // 2D模式标志（通过接收2D激光雷达数据判断）
    bool is2DMode_ = false;
};

#endif // GLWIDGET_H
