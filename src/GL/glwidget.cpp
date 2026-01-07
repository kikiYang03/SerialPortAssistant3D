#include "glwidget.h"
#include <Eigen/Dense>
#include <QMouseEvent>
#include <QDebug>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

GLWidget::GLWidget(QWidget *parent)
    : QOpenGLWidget(parent)
    , distance_(10.0)   // 原来可能是 1.0 或 3.0，改到 10 以上
    , yaw_(0)
    , pitch_(0)
    , center_(0,0,0)
{
}
GLWidget::~GLWidget() {}

void GLWidget::initializeGL()
{
    initializeOpenGLFunctions();
    glClearColor(.1f,.1f,.15f,1.f);
    glEnable(GL_DEPTH_TEST);

    progSimple_.addShaderFromSourceCode(QOpenGLShader::Vertex,
                                        "#version 330 core\n"
                                        "layout(location=0) in vec3 aPos;\n"
                                        "uniform mat4 mvp;\n"
                                        "void main(){ gl_Position = mvp * vec4(aPos,1); }");
    progSimple_.addShaderFromSourceCode(QOpenGLShader::Fragment,
                                        "#version 330 core\n"
                                        "uniform vec3 col;\n"
                                        "out vec4 fragCol;\n"
                                        "void main(){ fragCol = vec4(col,1); }");
    progSimple_.link();

    vaoCloud_.create();  vaoCloud_.bind();
    vboCloud_.create();  vboCloud_.setUsagePattern(QOpenGLBuffer::DynamicDraw);
    vboCloud_.bind();
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0,3,GL_FLOAT,GL_FALSE,0,nullptr);
    vaoCloud_.release();

    vaoMap_.create();  vaoMap_.bind();
    vboMap_.create();  vboMap_.setUsagePattern(QOpenGLBuffer::DynamicDraw);
    vboMap_.bind();
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0,3,GL_FLOAT,GL_FALSE,0,nullptr);
    vaoMap_.release();

    // 静态 TF
    // tf_.setTransform("map","camera_init", Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity());
    tf_.setTransform("body","base_link", Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity());

}

void GLWidget::resizeGL(int w,int h)
{
    double aspect = double(w)/std::max(1,h);
    proj_.setIdentity();
    double fovy = 45.0 * M_PI / 180.0;
    double zNear = 0.1;
    double zFar  = 1000.0;
    double tanHalfFovy = std::tan(fovy / 2.0);
    double f = 1.0 / tanHalfFovy;
    proj_(0,0) = f / aspect;
    proj_(1,1) = f;
    proj_(2,2) = (zFar + zNear) / (zNear - zFar);
    proj_(3,2) = -1.0;
    proj_(2,3) = (2.0 * zFar * zNear) / (zNear - zFar);
    proj_(3,3) = 0.0;
}

QMatrix4x4 GLWidget::toQMatrix(const Eigen::Matrix4d &m)
{
    QMatrix4x4 q;
    for(int i=0;i<4;i++)
        for(int j=0;j<4;j++)
            q(i,j)=float(m(i,j));
    return q;
}

void GLWidget::drawAxis(const Eigen::Matrix4d &T, float len)
{
    static std::vector<Eigen::Vector3f> pts = {
        {0,0,0},{len,0,0}, {0,0,0},{0,len,0}, {0,0,0},{0,0,len}
    };
    static QOpenGLBuffer vbo; static QOpenGLVertexArrayObject vao;
    if(!vbo.isCreated()){ vbo.create(); vao.create(); vao.bind(); vbo.bind();
        vbo.allocate(pts.data(), pts.size()*sizeof(Eigen::Vector3f));
        glEnableVertexAttribArray(0); glVertexAttribPointer(0,3,GL_FLOAT,GL_FALSE,0,0); vao.release(); }
    progSimple_.bind();
    progSimple_.setUniformValue("mvp", toQMatrix(proj_*view_*T));
    vao.bind();
    progSimple_.setUniformValue("col", QVector3D(1,0,0)); glDrawArrays(GL_LINES,0,2);
    progSimple_.setUniformValue("col", QVector3D(0,1,0)); glDrawArrays(GL_LINES,2,2);
    progSimple_.setUniformValue("col", QVector3D(0,0,1)); glDrawArrays(GL_LINES,4,2);
    vao.release(); progSimple_.release();
}

void GLWidget::drawGrid(const Eigen::Matrix4d &T, int cells, float step)
{
    std::vector<Eigen::Vector3f> lines;
    float ext = cells*step*.5f;
    for(int i=0;i<=cells;i++){
        float x = -ext + i*step;
        lines.emplace_back(x,-ext,0); lines.emplace_back(x,ext,0);
        lines.emplace_back(-ext,x,0); lines.emplace_back(ext,x,0);
    }
    static QOpenGLBuffer vbo; static QOpenGLVertexArrayObject vao;
    if(!vbo.isCreated()){ vbo.create(); vao.create(); vao.bind(); vbo.bind();
        glEnableVertexAttribArray(0); glVertexAttribPointer(0,3,GL_FLOAT,GL_FALSE,0,0); vao.release(); }
    vbo.allocate(lines.data(), lines.size()*sizeof(Eigen::Vector3f));
    progSimple_.bind();
    progSimple_.setUniformValue("mvp", toQMatrix(proj_*view_*T));
    progSimple_.setUniformValue("col", QVector3D(.4,.4,.4));
    vao.bind(); glDrawArrays(GL_LINES, 0, lines.size()); vao.release();
    progSimple_.release();
}

void GLWidget::drawArrow(const Eigen::Matrix4d &T, float len)
{
    Eigen::Vector3f from = T.block<3,1>(0,3).cast<float>();
    Eigen::Vector3f dir  = (T.block<3,3>(0,0) * Eigen::Vector3d::UnitX()).cast<float>();
    std::vector<Eigen::Vector3f> pts = { from, from+len*dir };
    static QOpenGLBuffer vbo; static QOpenGLVertexArrayObject vao;
    if(!vbo.isCreated()){ vbo.create(); vao.create(); vao.bind(); vbo.bind();
        glEnableVertexAttribArray(0); glVertexAttribPointer(0,3,GL_FLOAT,GL_FALSE,0,0); vao.release(); }
    vbo.allocate(pts.data(), pts.size()*sizeof(Eigen::Vector3f));
    progSimple_.bind();
    progSimple_.setUniformValue("mvp", toQMatrix(proj_*view_));
    progSimple_.setUniformValue("col", QVector3D(1,.5,0));
    vao.bind(); glDrawArrays(GL_LINES, 0, 2); vao.release();
    progSimple_.release();
}

void GLWidget::paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);

    /* ---------------- 1. 轨道球视图矩阵（camera_init 系） ---------------- */
    Eigen::Matrix4d view = Eigen::Matrix4d::Identity();

    const double ry = yaw_   * M_PI / 180.0;
    const double rp = pitch_ * M_PI / 180.0;

    Eigen::Matrix4d transBack = Eigen::Matrix4d::Identity();
    transBack(2,3) = -distance_;

    Eigen::Matrix4d rotYaw = Eigen::Matrix4d::Identity();
    rotYaw(0,0) =  cos(ry); rotYaw(0,1) = -sin(ry);
    rotYaw(1,0) =  sin(ry); rotYaw(1,1) =  cos(ry);

    Eigen::Matrix4d rotPitch = Eigen::Matrix4d::Identity();
    rotPitch(1,1) =  cos(rp); rotPitch(1,2) = -sin(rp);
    rotPitch(2,1) =  sin(rp); rotPitch(2,2) =  cos(rp);

    Eigen::Matrix4d lookCenter = Eigen::Matrix4d::Identity();
    lookCenter(0,3) = -center_.x();
    lookCenter(1,3) = -center_.y();
    lookCenter(2,3) = -center_.z();

    view = transBack * rotYaw * rotPitch * lookCenter;
    view_ = view;

    /* ---------------- 2. 基础元素（camera_init 系） ---------------- */
    Eigen::Matrix4d world = Eigen::Matrix4d::Identity(); // 就是 camera_init
    drawAxis(world, 10.0f);
    drawGrid(world, 40, 1.0f);

    /* ---------------- 3. 轨迹线（绿色） ---------------- */
    /* ---------------- 轨迹线（camera_init 系） ---------------- */
    if (trail_.points.size() > 1)
    {
        progSimple_.bind();
        progSimple_.setUniformValue("mvp", toQMatrix(proj_ * view_));
        progSimple_.setUniformValue("col", QVector3D(0,1,0));

        static QOpenGLVertexArrayObject vao;
        static QOpenGLBuffer vbo;
        if (!vao.isCreated()) { vao.create(); vbo.create(); }

        vao.bind();
        vbo.bind();
        vbo.allocate(trail_.points.data(),
                     trail_.points.size() * sizeof(Eigen::Vector3f));
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, nullptr);
        glLineWidth(3.0f);
        glDrawArrays(GL_LINE_STRIP, 0, trail_.points.size());
        vao.release();
        progSimple_.release();
    }

    /* ---------------- 最新箭头（camera_init 系） ---------------- */
    if (trail_.hasValidTransform)
    {
        progSimple_.bind();
        drawSolidArrow(trail_.latestTransform, 1.0f, 0.15f);
        progSimple_.release();
    }

    /* ---------------- 5. 点云（蓝色当前 / 灰色地图） ---------------- */
    progSimple_.bind();
    const Eigen::Matrix4d mvp = proj_ * view_;
    progSimple_.setUniformValue("mvp", toQMatrix(mvp));

    // 当前帧点云
    progSimple_.setUniformValue("col", QVector3D(0,0,1));
    vaoCloud_.bind();
    if (cloudPts_ > 0) { glPointSize(3.0f); glDrawArrays(GL_POINTS, 0, cloudPts_); }
    vaoCloud_.release();

    // 地图点云
    progSimple_.setUniformValue("col", QVector3D(0.5f,0.5f,0.5f));
    vaoMap_.bind();
    if (mapPts_ > 0) { glPointSize(2.0f); glDrawArrays(GL_POINTS, 0, mapPts_); }
    vaoMap_.release();

    progSimple_.release();
}

// 槽函数
// void GLWidget::onTf(const TFMsg &m)
// {
//     // 1. 存储TF到树中
//     tf_.setTransform(m.frame_id, m.child_frame_id,
//                      Eigen::Vector3d(m.t.x(),m.t.y(),m.t.z()),
//                      Eigen::Quaterniond(m.q.scalar(),m.q.x(),m.q.y(),m.q.z()));

//     // 2. 检查map->camera_init是否就绪
//     if(m.frame_id == "map" && m.child_frame_id == "camera_init") {
//         map_T_camera_init_ready_ = true;
//     }

//     // 3. 获取最新TF变换
//     auto T_map_ci = tf_.lookupMapCameraInit();
//     auto T_ci_b   = tf_.lookupCameraInitBody();

//     if (T_map_ci && T_ci_b) {
//         // 计算map->body的变换
//         Eigen::Matrix4d T_map_body = (*T_map_ci) * (*T_ci_b);

//         // 3.1 存储轨迹点（只存位置）
//         Eigen::Vector3f position = T_map_body.block<3,1>(0,3).cast<float>();
//         trail_.points.push_back(position);

//         // 限制轨迹点数量
//         const size_t kMaxTrailPoints = 1000;  // 可以根据需要调整
//         if(trail_.points.size() > kMaxTrailPoints) {
//             trail_.points.erase(trail_.points.begin());
//         }

//         // 3.2 更新最新变换（用于绘制箭头）
//         trail_.latestTransform = T_map_body;
//         trail_.hasValidTransform = true;
//     }

//     update();   // 触发重绘
// }
void GLWidget::onTf(const TFMsg &m)
{
    /* 1. 照旧存树（方便别处用） */
    tf_.setTransform(m.frame_id, m.child_frame_id,
                     Eigen::Vector3d(m.t.x(),m.t.y(),m.t.z()),
                     Eigen::Quaterniond(m.q.scalar(),m.q.x(),m.q.y(),m.q.z()));

    /* 2. 只要收到 camera_init→body 就记录轨迹 */
    if (m.frame_id == "camera_init" && m.child_frame_id == "body")
    {
        Eigen::Matrix4d T_ci_b = Eigen::Matrix4d::Identity();
        T_ci_b.block<3,3>(0,0) = Eigen::Quaterniond(m.q.scalar(),m.q.x(),m.q.y(),m.q.z()).matrix();
        T_ci_b.block<3,1>(0,3) = Eigen::Vector3d(m.t.x(),m.t.y(),m.t.z());

        Eigen::Vector3f pt = T_ci_b.block<3,1>(0,3).cast<float>();

        trail_.points.push_back(pt);
        if (trail_.points.size() > kMaxTrail) trail_.points.erase(trail_.points.begin());

        trail_.latestTransform   = T_ci_b;   // 箭头也放在 camera_init 系
        trail_.hasValidTransform = true;

        update();          // 请求主线程重绘
    }
}

void GLWidget::onCloud(const CloudMsg &m)
{
    std::vector<Eigen::Vector3f> tmp;
    tmp.reserve(m.points.size());
    // auto T_map_ci = tf_.lookup("map","camera_init");
    // Eigen::Matrix4d T = T_map_ci ? (*T_map_ci) : Eigen::Matrix4d::Identity();

    for(const auto &p : m.points){
        // Eigen::Vector4d pi(p.x(),p.y(),p.z(),1.0);
        // Eigen::Vector4d pw = T * pi;
        // tmp.emplace_back(pw.head<3>().cast<float>());
        tmp.emplace_back(p.x(), p.y(), p.z());   // 直接拷
    }
    vboCloud_.bind();
    vboCloud_.allocate(tmp.data(), tmp.size()*sizeof(Eigen::Vector3f));
    cloudPts_ = tmp.size();
    update();
}

void GLWidget::onMap(const MapCloudMsg &m)
{
    std::vector<Eigen::Vector3f> tmp;
    tmp.reserve(m.points.size());
    // auto T_map_ci = tf_.lookup("map","camera_init");
    // Eigen::Matrix4d T = T_map_ci ? (*T_map_ci) : Eigen::Matrix4d::Identity();

    for(const auto &p : m.points){
        // Eigen::Vector4d pi(p.x(),p.y(),p.z(),1.0);
        // Eigen::Vector4d pw = T * pi;
        // tmp.emplace_back(pw.head<3>().cast<float>());
        tmp.emplace_back(p.x(), p.y(), p.z());   // 直接拷
    }
    vboMap_.bind();
    vboMap_.allocate(tmp.data(), tmp.size()*sizeof(Eigen::Vector3f));
    mapPts_ = tmp.size();
    update();
}


void GLWidget::mousePressEvent(QMouseEvent* e)
{
    lastMousePos_ = e->pos();
}

void GLWidget::mouseMoveEvent(QMouseEvent* e)
{
    QPoint delta = e->pos() - lastMousePos_;
    lastMousePos_ = e->pos();

    if (e->buttons() & Qt::LeftButton) {
        // 旋转视角
        yaw_ += delta.x() * 0.5f;
        pitch_ += delta.y() * 0.5f;
        pitch_ = std::max(-89.0f, std::min(89.0f, pitch_));  // 限制范围
    }
    else if (e->buttons() & Qt::RightButton) {
        // 平移视角中心
        float sensitivity = 0.01f * distance_;
        center_.setX(center_.x() + delta.x() * sensitivity);
        center_.setY(center_.y() - delta.y() * sensitivity);
    }

    update();
}

void GLWidget::wheelEvent(QWheelEvent* e)
{
    // 缩放
    distance_ *= (e->angleDelta().y() > 0) ? 0.9f : 1.1f;
    distance_ = std::max(1.0f, std::min(500.0f, distance_));  // 限制范围
    update();
}

void GLWidget::drawSolidArrow(const Eigen::Matrix4d &T, float len, float radius)
{
    // 1. 创建箭头几何数据
    const int seg = 16;
    const float shaftLen = len * 0.65f;
    const float headLen = len - shaftLen;
    const float headRad = radius * 2.0f;

    std::vector<Eigen::Vector3f> vertices;

    // 2. 圆柱杆（四边形带）
    for (int i = 0; i <= seg; ++i) {
        float a = 2.0f * M_PI * i / seg;
        float x = cosf(a) * radius;
        float y = sinf(a) * radius;
        vertices.emplace_back(x, y, 0);            // 起点
        vertices.emplace_back(x, y, shaftLen);     // 终点
    }

    // 3. 圆锥头（三角形）
    for (int i = 0; i < seg; ++i) {
        float a0 = 2.0f * M_PI * i / seg;
        float a1 = 2.0f * M_PI * (i+1) / seg;
        float x0 = cosf(a0) * headRad;
        float y0 = sinf(a0) * headRad;
        float x1 = cosf(a1) * headRad;
        float y1 = sinf(a1) * headRad;

        // 三个顶点组成一个三角形
        vertices.emplace_back(0, 0, shaftLen + headLen);  // 尖
        vertices.emplace_back(x1, y1, shaftLen);          // 底边1
        vertices.emplace_back(x0, y0, shaftLen);          // 底边0
    }

    // 4. 使用着色器绘制
    progSimple_.bind();

    // 关键：设置正确的MVP矩阵
    Eigen::Matrix4d mvp = proj_ * view_ * T;
    progSimple_.setUniformValue("mvp", toQMatrix(mvp));

    // 设置箭头颜色
    progSimple_.setUniformValue("col", QVector3D(1.0f, 0.5f, 0.0f));  // 橙色

    // 创建VAO/VBO
    static QOpenGLVertexArrayObject arrowVao;
    static QOpenGLBuffer arrowVbo;

    if (!arrowVao.isCreated()) {
        arrowVao.create();
        arrowVbo.create();
    }

    arrowVao.bind();
    arrowVbo.bind();

    // 上传顶点数据
    arrowVbo.allocate(vertices.data(), vertices.size() * sizeof(Eigen::Vector3f));

    // 设置顶点属性
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, nullptr);

    // 绘制圆柱杆（GL_QUAD_STRIP）
    int cylinderVertices = (seg + 1) * 2;
    glDrawArrays(GL_QUAD_STRIP, 0, cylinderVertices);

    // 绘制圆锥头（GL_TRIANGLES）
    int coneVertices = vertices.size() - cylinderVertices;
    glDrawArrays(GL_TRIANGLES, cylinderVertices, coneVertices);

    arrowVao.release();
}

