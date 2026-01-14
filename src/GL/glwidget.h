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

signals:
    void tfInfoChanged(double x, double y, double z,
                       double yaw_rad, double pitch_rad, double roll_rad);   //显示TF相关数据

public slots:
    void onTf(const TFMsg &);
    void onCloud(const CloudMsg &);
    void onMap(const MapCloudMsg &);

    void clearMap();        // 清理点云地图
    void clearTrail();  // 新增
    void clearCloud();  // 新增
    void resetCamera();     // 初始化相机位置
    void saveMapToFile();   // 保存地图

private slots:   // 新增
    void doUploadCloud();   // 在主线程里把 cloudCpu_ 塞进 vboCloud_
    void doUploadMap();     // 在主线程里把 mapInterleavedCpu_ 塞进 vboMap_

protected:
    void initializeGL() override;
    void resizeGL(int w, int h) override;
    void paintGL() override;

private:

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

    // TF关系链
    Eigen::Matrix4d T_map_ci__latest_      = Eigen::Matrix4d::Identity(); // map→camera_init
    Eigen::Matrix4d T_body_baselink_latest_= Eigen::Matrix4d::Identity(); // body→base_link


    // 放在 private 段
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
};

#endif // GLWIDGET_H
