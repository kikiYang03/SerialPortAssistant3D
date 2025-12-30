#include "glscene.h"
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

GLScene::GLScene(QWidget* parent)
    : QOpenGLWidget(parent)
{
    setFocusPolicy(Qt::StrongFocus);
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

    // 1. 栅格
    drawGrid();

    // 2. 其余对象
    drawAxes(1.0f);
    drawTFs();
    drawPointCloud();
}

// 相机操控
void GLScene::applyCamera()
{
    glTranslatef(0, 0, -distance_);
    glRotatef(pitch_, 1, 0, 0);
    glRotatef(yaw_, 0, 0, 1);
    glTranslatef(-center_.x(), -center_.y(), -center_.z());
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
        center_.setX(center_.x() - delta.x() * 0.01f);
        center_.setY(center_.y() + delta.y() * 0.01f);
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
    for (auto& tf : tfs_) {
        glPushMatrix();
        glTranslatef(tf.t.x(), tf.t.y(), tf.t.z());
        drawAxes(0.5f);
        glPopMatrix();
    }
}

void GLScene::drawPointCloud()
{
    glPointSize(3.0f);
    glBegin(GL_POINTS);
    glColor3f(0.2f, 0.2f, 0.2f);

    for (auto& p : pointCloud_)
        glVertex3f(p.x, p.y, p.z);

    glEnd();
}
void GLScene::setPointCloud(const QVector<Point3D>& pts)
{
    pointCloud_ = pts;
    update();
}
void GLScene::setTFs(const QVector<Transform>& tfs)
{
    tfs_ = tfs;
    update();
}

