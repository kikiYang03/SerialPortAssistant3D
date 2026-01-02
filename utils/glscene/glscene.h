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
    void setMapPoints(const QVector<Point3D>& pts);
    void setCloudPoints(const QVector<Point3D>& pts);

    void setWorldFrame(const QString& f) { worldFrame_ = f; }


    // 轨迹追加点（点已经是在本视图 world 坐标系下）
    void addTrajectoryPoint(const QVector3D& p);

    void setCloudPoseInWorld(const QVector3D& t, const QQuaternion& q);

protected:
    void initializeGL() override;
    void resizeGL(int w, int h) override;
    void paintGL() override;

    void mousePressEvent(QMouseEvent*) override;
    void mouseMoveEvent(QMouseEvent*) override;
    void wheelEvent(QWheelEvent*) override;

private:
    void drawAxes(float scale = 1.0f);
    void drawPointClouds();
    void drawTFs();
    void applyCamera();
    void drawTrajectory();

private:
    QString worldFrame_;
    //
    QVector3D    cloud_t_ = QVector3D(0,0,0);            // T_world_cloudFrame
    QQuaternion  cloud_q_ = QQuaternion(1,0,0,0);
    //
    QVector<Point3D> pointCloud_;
    QVector<Transform> tfs_;

    QVector<QVector3D> trajectory_;   // 绿色轨迹点
    int maxTrajectoryPoints_ = 5000;  // 防止无限增长

    QVector<Point3D> mapPoints_;    // 存储地图点
    QVector<Point3D> cloudPoints_;  // 存储最新传感器点


    // 相机参数（RViz 风格）
    float distance_ = 10.0f;
    float yaw_ = -45.0f;
    float pitch_ = 45.0f;
    QVector3D center_ = QVector3D(0, 0, 0);

    QPoint lastMouse_;
};

#endif // CONFIG_H

