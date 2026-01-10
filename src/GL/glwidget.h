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

public slots:
    void onTf(const TFMsg &);
    void onCloud(const CloudMsg &);
    void onMap(const MapCloudMsg &);

protected:
    void initializeGL() override;
    void resizeGL(int w, int h) override;
    void paintGL() override;

private:
    TfTree tf_;

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
};

#endif // GLWIDGET_H
