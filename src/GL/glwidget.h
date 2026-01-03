#ifndef GLWIDGET_H
#define GLWIDGET_H
#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QOpenGLBuffer>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLShaderProgram>
#include "tftree.h"
#include "protocol_msg.h"

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
};
inline QDebug operator<<(QDebug dbg, const Eigen::Matrix4d& m)
{
    std::stringstream ss;
    ss << m;
    dbg << QString::fromStdString(ss.str());
    return dbg;
}
#endif // GLWIDGET_H
