#ifndef GLSCENE_H
#define GLSCENE_H

#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QVector>
#include <QVector3D>
#include <QQuaternion>
#include <QMouseEvent>
#include <QWheelEvent>

struct Point3D {
    float x, y, z;
};

struct Transform {
    QString frame;
    QString child;
    QVector3D t;
    QQuaternion q;
};

class GLScene : public QOpenGLWidget, protected QOpenGLFunctions
{
    Q_OBJECT
public:
    explicit GLScene(QWidget* parent = nullptr);

    void setPointCloud(const QVector<Point3D>& pts);
    void setTFs(const QVector<Transform>& tfs);

protected:
    void initializeGL() override;
    void resizeGL(int w, int h) override;
    void paintGL() override;

    void mousePressEvent(QMouseEvent*) override;
    void mouseMoveEvent(QMouseEvent*) override;
    void wheelEvent(QWheelEvent*) override;

private:
    void drawAxes(float scale = 1.0f);
    void drawPointCloud();
    void drawTFs();
    void applyCamera();

private:
    QVector<Point3D> pointCloud_;
    QVector<Transform> tfs_;

    // 相机参数（RViz 风格）
    float distance_ = 10.0f;
    float yaw_ = -45.0f;
    float pitch_ = 45.0f;
    QVector3D center_ = QVector3D(0, 0, 0);

    QPoint lastMouse_;
};

#endif // CONFIG_H

