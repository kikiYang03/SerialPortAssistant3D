#ifndef GLWIDGET_H
#define GLWIDGET_H
#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QOpenGLBuffer>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLShaderProgram>
#include <QOpenGLTexture>
#include "tftree.h"
#include "protocol_msg.h"

// TF 轨迹缓存（map 坐标系）
struct TfTrail {
    std::vector<Eigen::Vector3f> body;      // body 原点序列
    std::vector<Eigen::Vector3f> camera_init;
    std::vector<Eigen::Vector3f> base_link;
};

struct Trail {
    std::vector<Eigen::Vector3f> points;    // 只存储轨迹点（用于画线）
    Eigen::Matrix4d latestTransform;        // 只存储最新变换（用于画箭头）
    bool hasValidTransform = false;         // 是否有有效的变换
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
    void resizeGL(int w,int h) override;
    void paintGL() override;

private:
    TfTree tf_;
    bool   map_T_camera_init_ready_ = false;

    // GPU
    QOpenGLBuffer vboCloud_{QOpenGLBuffer::VertexBuffer};
    QOpenGLBuffer vboMap_  {QOpenGLBuffer::VertexBuffer};
    QOpenGLVertexArrayObject vaoCloud_, vaoMap_;
    int cloudPts_ = 0, mapPts_ = 0;

    QOpenGLShaderProgram progSimple_;

    Eigen::Matrix4d proj_, view_;

    static QMatrix4x4 toQMatrix(const Eigen::Matrix4d &m);
    void drawAxis(const Eigen::Matrix4d &T, float len = 5.f);
    void drawGrid(const Eigen::Matrix4d &T, int cells = 20, float step = 1.f);
    void drawArrow(const Eigen::Matrix4d &T, float len = 2.f);
    std::vector<Eigen::Matrix4d> trajectory_;
    void drawPointCloud(QOpenGLVertexArrayObject &vao,
                                  QOpenGLBuffer &vbo,
                                  int pointCount,
                                  const Eigen::Matrix4d &modelMatrix,
                        const QVector3D &color);

    // 添加相机参数
    float distance_ = 30.0f;
    float pitch_ = -45.0f;    // 俯仰角
    float yaw_ = 180.0f;      // 偏航角
    QVector3D center_ = QVector3D(0, 0, 0);
    QPoint lastMousePos_;

    // 添加鼠标事件处理
    void mousePressEvent(QMouseEvent* e) override;
    void mouseMoveEvent(QMouseEvent* e) override;
    void wheelEvent(QWheelEvent* e) override;



    // 测试立方体
    // QOpenGLVertexArrayObject vaoCube_;
    // QOpenGLBuffer vboCube_;
    // int cubePts_ = 0;
    // void drawCube(const Eigen::Matrix4d &T, float size);

    // 测试立方体变换矩阵
    // Eigen::Matrix4d cubeTransform_;

    Trail  trail_;
    static constexpr size_t kMaxTrail = 5000;   // 轨迹最多存 5000 点

    void drawSolidArrow(const Eigen::Matrix4d &T, float len, float radius = 0.08f);

    QOpenGLShaderProgram progColorCloud_;  // 新增彩色着色器
    std::vector<Eigen::Vector3f> mapVertices_;  // 存储地图顶点数据（位置+颜色）
    float mapMinZ_ = 0.0f;  // 地图点云最小高度
    float mapMaxZ_ = 0.0f;  // 地图点云最大高度

    // 根据高度生成颜色（类似RViz的HeightMap）
    QVector3D heightToColor(float z, float minZ, float maxZ);

};
#endif // GLWIDGET_H
