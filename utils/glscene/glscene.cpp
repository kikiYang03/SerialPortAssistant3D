#include "glscene.h"
#include <cmath>
#include <QMatrix4x4>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

GLScene::GLScene(QWidget* parent)
    : QOpenGLWidget(parent)
{
    setFocusPolicy(Qt::StrongFocus);

    distance_ = 20.0f;
    pitch_    = -45.0f;   // 关键：向下看
    yaw_ = 180.0f;


    center_ = QVector3D(0, 0, 0);
}

static void applyQuaternion(const QQuaternion& q)
{
    QMatrix4x4 m;
    m.rotate(q);                 // quaternion → rotation matrix
    glMultMatrixf(m.constData()); // 乘到当前 MODELVIEW
}

// 绘制机头箭头
void draw3DArrow(float length = 1.0f, float shaftRadius = 0.05f, float headLength = 0.2f, float headRadius = 0.1f, int slices = 16)
{
    if (length <= 0) return;
    if (headLength >= length) headLength = length * 0.3f;

    float shaftLength = length - headLength;

    // --------------------
    // 1. 绘制圆柱杆 (shaft)
    // --------------------
    glColor3f(1.0f, 0.0f, 0.0f); // 橙色
    glBegin(GL_QUAD_STRIP);
    for (int i = 0; i <= slices; ++i) {
        float theta = (2.0f * M_PI * i) / slices;
        float x = 0.0f;
        float y = cos(theta) * shaftRadius;
        float z = sin(theta) * shaftRadius;

        glVertex3f(x, y, z);
        glVertex3f(shaftLength, y, z);
    }
    glEnd();

    // --------------------
    // 2. 绘制圆锥头 (head)
    // --------------------
    glBegin(GL_TRIANGLES);
    for (int i = 0; i < slices; ++i) {
        float theta0 = (2.0f * M_PI * i) / slices;
        float theta1 = (2.0f * M_PI * (i + 1)) / slices;

        float y0 = cos(theta0) * headRadius;
        float z0 = sin(theta0) * headRadius;

        float y1 = cos(theta1) * headRadius;
        float z1 = sin(theta1) * headRadius;

        // 顶点：尖端 + 两底边
        glVertex3f(length, 0, 0);              // 尖端
        glVertex3f(shaftLength, y1, z1);       // 底边1
        glVertex3f(shaftLength, y0, z0);       // 底边0
    }
    glEnd();
}


// 画 XY 平面栅格，halfSize 单方向范围，step 格子间距
void drawGrid(float halfSize = 20.0f, float step = 1.0f)
{
    glColor3f(0.35f, 0.35f, 0.35f);          // 栅格线颜色 #595959
    glBegin(GL_LINES);
    for (float x = -halfSize; x <= halfSize + 1e-4f; x += step)
    {
        glVertex3f(x, -halfSize, 0.0f);
        glVertex3f(x,  halfSize, 0.0f);
    }
    for (float y = -halfSize; y <= halfSize + 1e-4f; y += step)
    {
        glVertex3f(-halfSize, y, 0.0f);
        glVertex3f( halfSize, y, 0.0f);
    }
    glEnd();
}

void GLScene::initializeGL()
{
    initializeOpenGLFunctions();
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);               // 可选
    // RViz 默认深灰
    glClearColor(0.19f, 0.19f, 0.19f, 1.0f);   // #303030
}

void GLScene::resizeGL(int w, int h)
{
    if (h == 0) h = 1;
    glViewport(0, 0, w, h);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    float aspect = float(w) / float(h);
    float nearPlane = 0.01f;
    float farPlane  = 1000.0f;
    float fov = 60.0f;

    float top = nearPlane * tanf(fov * 0.5f * M_PI / 180.0f);
    float right = top * aspect;

    glFrustum(-right, right, -top, top, nearPlane, farPlane);
}

void GLScene::paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    applyCamera();

    drawGrid();
    drawTrajectory();     // 轨迹
    drawTFs();            // 只画跟随 TF 的箭头
    drawPointClouds();
}



// 绘制轨迹
void GLScene::drawTrajectory()
{
    if (trajectory_.size() < 2)
        return;

    glLineWidth(2.0f);
    glColor3f(0.0f, 1.0f, 0.0f);   // 绿色

    glBegin(GL_LINE_STRIP);
    for (const auto& p : trajectory_) {
        glVertex3f(p.x(), p.y(), p.z());
    }
    glEnd();
}

// 相机操控
void GLScene::applyCamera()
{
    // 1. 原来的轨道球：眼睛在 map 系
    glTranslatef(0, 0, -distance_);
    glRotatef(pitch_, 1, 0, 0);
    glRotatef(-yaw_, 0, 0, 1);
    glTranslatef(-center_.x(), -center_.y(), -center_.z());

    // 2. 再乘一次  map->camera_init 的逆，于是 OpenGL 当前矩阵变成
    //    “camera_init 系 -> map 眼”
    QMatrix4x4 m;
    QQuaternion invQ = camInit_q_.inverted();
    m.translate(-camInit_t_);
    m.rotate(invQ);
    glMultMatrixf(m.constData());
}

// 鼠标控制
void GLScene::mousePressEvent(QMouseEvent* e)
{
    lastMouse_ = e->pos();
}

void GLScene::mouseMoveEvent(QMouseEvent* e)
{
    QPoint delta = e->pos() - lastMouse_;
    lastMouse_ = e->pos();

    if (e->buttons() & Qt::LeftButton) {
        yaw_   += delta.x() * 0.5f;
        pitch_ += delta.y() * 0.5f;
    }
    else if (e->buttons() & Qt::RightButton) {
        center_.setX(center_.x() + delta.x() * 0.01f);
        center_.setY(center_.y() - delta.y() * 0.01f);
    }
    update();
}

void GLScene::wheelEvent(QWheelEvent* e)
{
    distance_ *= (e->angleDelta().y() > 0) ? 0.9f : 1.1f;
    update();
}

// TF 点云控制
void GLScene::drawAxes(float s)
{
    glLineWidth(2.0f);
    glBegin(GL_LINES);

    glColor3f(1,0,0); glVertex3f(0,0,0); glVertex3f(s,0,0);
    glColor3f(0,1,0); glVertex3f(0,0,0); glVertex3f(0,s,0);
    glColor3f(0,0,1); glVertex3f(0,0,0); glVertex3f(0,0,s);

    glEnd();
}

void GLScene::drawTFs()
{
    if (tfs_.isEmpty())
        return;

    const auto& tf = tfs_.last();   // 只画最新 TF

    glPushMatrix();

    // 1) 平移到 TF 原点
    glTranslatef(tf.t.x(), tf.t.y(), tf.t.z());

    // 2) 应用姿态
    applyQuaternion(tf.q);

    // 3) 画机体朝向（X 轴 = 机头）
    // draw3DArrow(1.0f, 0.05f, 0.2f, 0.1f, 16);
    draw3DArrow(3.0f, 0.15f, 0.6f, 0.25f, 24);

    glPopMatrix();
}


void GLScene::drawPointClouds()
{
    // 1. 地图点云（本来就在 map，所以保持原样）
    glPointSize(2.0f);
    glBegin(GL_POINTS);
    glColor3f(0.5f, 0.5f, 0.5f);
    for (const auto& p : mapPoints_)
        glVertex3f(p.x, p.y, p.z);
    glEnd();

    // 2. 当前激光点云——**直接画，不再乘 cloud_t_/cloud_q_**
    glPointSize(4.0f);
    glBegin(GL_POINTS);
    glColor3f(0.0f, 0.0f, 1.0f);
    for (const auto& p : cloudPoints_)
        glVertex3f(p.x, p.y, p.z);   // 纯 camera_init 坐标
    glEnd();

}

void GLScene::setMapPoints(const QVector<Point3D>& pts)
{
    mapPoints_ = pts;
    update();
}

void GLScene::setCloudPoints(const QVector<Point3D>& pts)
{
    cloudPoints_ = pts;
    update();
}


void GLScene::setPointCloud(const QVector<Point3D>& pts)
{
    pointCloud_ = pts;
    update();
}
void GLScene::setTFs(const QVector<Transform>& tfs)
{
    tfs_ = tfs;

    if (!tfs_.isEmpty()) {
        trajectory_.push_back(tfs_.last().t);   // 关键：使用最新 TF

        if (trajectory_.size() > maxTrajectoryPoints_)
            trajectory_.pop_front();
    }

    update();
}


void GLScene::addTrajectoryPoint(const QVector3D& p)
{
    trajectory_.push_back(p);
    if (trajectory_.size() > maxTrajectoryPoints_)
        trajectory_.pop_front();
    update();
}

void GLScene::setCloudPoseInWorld(const QVector3D& t, const QQuaternion& q)
{
    // cloud_t_ = t;
    // cloud_q_ = q;
    camInit_t_ = t;
    camInit_q_ = q;
    update();
}
