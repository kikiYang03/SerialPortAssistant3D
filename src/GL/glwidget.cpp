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

    // 初始化立方体VAO/VBO
    vaoCube_.create();
    vboCube_.create();

    // 静态 TF
    // tf_.setTransform("map","camera_init", Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity());
    tf_.setTransform("body","base_link", Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity());
    // 初始化立方体变换矩阵（放置在原点）
    cubeTransform_ = Eigen::Matrix4d::Identity();
    cubeTransform_(2,3) = -10.0;   // 往“相机前方”推 10 米
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
    glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);

    static bool firstTime = true;
    if (firstTime) {
        qDebug() << "Camera parameters:";
        qDebug() << "  Distance:" << distance_;
        qDebug() << "  Yaw:" << yaw_;
        qDebug() << "  Pitch:" << pitch_;
        qDebug() << "  Center:" << center_.x() << center_.y() << center_.z();
        firstTime = false;
    }

    // ========== 1. 计算正确的视图矩阵 ==========
    // 使用类似 GLScene 的轨道球相机
    Eigen::Matrix4d view = Eigen::Matrix4d::Identity();

    // 轨道球相机矩阵构建顺序（与GLScene一致）：
    // 1. 相机从中心点向后移动
    // 2. 应用俯仰和偏航旋转
    // 3. 看向中心点

    // 将相机向后移动
    Eigen::Matrix4d translateBack = Eigen::Matrix4d::Identity();
    translateBack(2,3) = -distance_;  // 沿Z轴向后

    // 旋转矩阵：先偏航（绕Z轴），再俯仰（绕X轴）
    double radYaw = yaw_ * M_PI / 180.0;
    double radPitch = pitch_ * M_PI / 180.0;

    // 偏航矩阵（绕Z轴）
    Eigen::Matrix4d yawRot = Eigen::Matrix4d::Identity();
    yawRot(0,0) = cos(radYaw);   yawRot(0,1) = -sin(radYaw);
    yawRot(1,0) = sin(radYaw);   yawRot(1,1) = cos(radYaw);

    // 俯仰矩阵（绕X轴）
    Eigen::Matrix4d pitchRot = Eigen::Matrix4d::Identity();
    pitchRot(1,1) = cos(radPitch);   pitchRot(1,2) = -sin(radPitch);
    pitchRot(2,1) = sin(radPitch);   pitchRot(2,2) = cos(radPitch);

    // 看向中心点
    Eigen::Matrix4d lookAtCenter = Eigen::Matrix4d::Identity();
    lookAtCenter(0,3) = -center_.x();
    lookAtCenter(1,3) = -center_.y();
    lookAtCenter(2,3) = -center_.z();

    // 组合视图矩阵：先向后平移，再旋转，最后看向中心
    // 注意：矩阵乘法顺序与OpenGL的glTranslate/glRotate调用顺序相反
    view = translateBack * yawRot * pitchRot * lookAtCenter;

    // ========== 2. 应用TF变换（map->camera_init） ==========
    // 为了让点云在map坐标系正确显示，需要将camera_init坐标系的点变换到map坐标系
    // 或者等效地，将相机从map坐标系变换到camera_init坐标系
    if(map_T_camera_init_ready_){
        if(auto T_map_cam = tf_.lookup("map","camera_init")){
            // 方将相机从map系变换到camera_init系
            Eigen::Matrix4d T_cam_map = T_map_cam->inverse();  // camera_init -> map
            view = view * T_cam_map;  // 先变换相机到camera_init系
        }
    }

    view_ = view;

    // ========== 3. 绘制 ==========

    // 1. 绘制map坐标系下的内容
    // 1. 绘制map坐标系下的内容
    if(map_T_camera_init_ready_){
        Eigen::Matrix4d id = Eigen::Matrix4d::Identity();

        // 绘制map坐标轴（红色X，绿色Y，蓝色Z）
        drawAxis(id, 10.f);

        // 绘制栅格
        drawGrid(id, 40, 1.f);

        // 绘制测试立方体（在map坐标系原点）
        drawCube(cubeTransform_, 2.0f); // 边长为2的立方体

        // 绘制相机初始位姿（map->camera_init）
        if(auto T = tf_.lookup("map","camera_init")){
            drawAxis(*T, 3.f);

            // 在camera_init坐标系原点也绘制一个小立方体
            Eigen::Matrix4d cubeAtCameraInit = *T; // 立方体跟随camera_init坐标系
            drawCube(cubeAtCameraInit, 1.0f); // 边长为1的小立方体

            // 绘制从camera_init到body的箭头
            if(auto T_body = tf_.lookup("camera_init", "body")){
                Eigen::Matrix4d T_map_body = (*T) * (*T_body);
                drawArrow(T_map_body, 2.f);
            }
        }
    } else {
        // 如果没有TF变换，也在原点绘制一个立方体
        drawCube(Eigen::Matrix4d::Identity(), 2.0f);
    }

    // 2. 绘制点云（现在点云在camera_init坐标系，相机也在camera_init坐标系）
    progSimple_.bind();

    // 为点云设置MVP矩阵
    Eigen::Matrix4d mvp = proj_ * view_;
    progSimple_.setUniformValue("mvp", toQMatrix(mvp));

    // 绘制当前点云（蓝色）
    progSimple_.setUniformValue("col", QVector3D(0, 0, 1));
    vaoCloud_.bind();
    if(cloudPts_ > 0) {
        glPointSize(3.0f);  // 设置点大小
        glDrawArrays(GL_POINTS, 0, cloudPts_);
    }
    vaoCloud_.release();

    // 绘制地图点云（灰色）
    progSimple_.setUniformValue("col", QVector3D(0.5, 0.5, 0.5));
    vaoMap_.bind();
    if(mapPts_ > 0) {
        glPointSize(2.0f);  // 设置点大小
        glDrawArrays(GL_POINTS, 0, mapPts_);
    }

    vaoMap_.release();

    // ========== 4. 画 TF 轨迹（map1 空间，但数据已在 map 系） ==========
    if(trail_.body.size() > 1)    // 至少两个点才能看出方向
    {
        progSimple_.bind();
        progSimple_.setUniformValue("mvp", toQMatrix(proj_*view_)); // 已在世界空间
        progSimple_.setUniformValue("col", QVector3D(0,1,0));       // 纯绿轨迹

        static QOpenGLBuffer trailVbo; static QOpenGLVertexArrayObject trailVao;
        if(!trailVbo.isCreated()){ trailVbo.create(); trailVao.create(); }
        trailVao.bind(); trailVbo.bind();
        trailVbo.allocate(trail_.body.data(),
                          trail_.body.size()*sizeof(Eigen::Vector3f));
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0,3,GL_FLOAT,GL_FALSE,0,nullptr);

        // 1. 绿色线
        glLineWidth(3.0f);
        glDrawArrays(GL_LINE_STRIP, 0, trail_.body.size());

        // 2. 每段终点画绿色箭头
        progSimple_.setUniformValue("col", QVector3D(0,0.8,0));   // 稍深绿
        for(size_t i = 1; i < trail_.body.size(); ++i)
        {
            Eigen::Vector3f from = trail_.body[i-1];
            Eigen::Vector3f to   = trail_.body[i];
            Eigen::Vector3f dir  = (to - from).normalized();
            float len = 0.8f;                 // 箭头长度，可调

            // 构造一个“Z 轴沿 dir”的 4×4 矩阵
            Eigen::Vector3f up(0,0,1);
            Eigen::Vector3f x = dir.cross(up).normalized();
            Eigen::Vector3f y = dir.cross(x).normalized();
            Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
            T.block<3,1>(0,0) = x.cast<double>();
            T.block<3,1>(0,1) = y.cast<double>();
            T.block<3,1>(0,2) = dir.cast<double>();
            T.block<3,1>(0,3) = to.cast<double>();   // 平移到线段终点

            drawArrow(T, len);   // 复用你已有的 drawArrow
        }

        trailVao.release();
        progSimple_.release();
    }

    // progSimple_.release();
}

// 槽函数
void GLWidget::onTf(const TFMsg &m)
{
    // 1. 照旧存 TF 树
    tf_.setTransform(m.frame_id, m.child_frame_id,
                     Eigen::Vector3d(m.t.x(),m.t.y(),m.t.z()),
                     Eigen::Quaterniond(m.q.scalar(),m.q.x(),m.q.y(),m.q.z()));
    // 无论哪条 TF，只要能让链完整就更新轨迹
    // auto T_map_ci = tf_.lookup("map","camera_init");
    // auto T_ci_b   = tf_.lookup("camera_init","body");
    auto T_map_ci = tf_.lookupMapCameraInit();
    auto T_ci_b   = tf_.lookupCameraInitBody();
    // qDebug() << "lookup map->camera_init =" << (T_map_ci.has_value() ? "OK" : "FAIL")
    //          << "  lookup camera_init->body =" << (T_ci_b.has_value() ? "OK" : "FAIL");
    if (T_map_ci && T_ci_b) {
        Eigen::Matrix4d T_map_body = (*T_map_ci) * (*T_ci_b);
        Eigen::Vector3f pt = T_map_body.block<3,1>(0,3).cast<float>();

        trail_.body.push_back(pt);
        if(trail_.body.size() > kMaxTrail) trail_.body.erase(trail_.body.begin());
    }

    // 照旧开门
    if(m.frame_id == "map" && m.child_frame_id == "camera_init")
        map_T_camera_init_ready_ = true;

    update();   // 重绘
}

void GLWidget::onCloud(const CloudMsg &m)
{
    std::vector<Eigen::Vector3f> tmp; tmp.reserve(m.points.size());
    auto T_map_ci = tf_.lookup("map","camera_init");
    Eigen::Matrix4d T = T_map_ci ? (*T_map_ci) : Eigen::Matrix4d::Identity();

    for(const auto &p : m.points){
        Eigen::Vector4d pi(p.x(),p.y(),p.z(),1.0);
        Eigen::Vector4d pw = T * pi;
        tmp.emplace_back(pw.head<3>().cast<float>());
    }
    vboCloud_.bind();
    vboCloud_.allocate(tmp.data(), tmp.size()*sizeof(Eigen::Vector3f));
    cloudPts_ = tmp.size();
    update();
}

void GLWidget::onMap(const MapCloudMsg &m)
{
    std::vector<Eigen::Vector3f> tmp; tmp.reserve(m.points.size());
    auto T_map_ci = tf_.lookup("map","camera_init");
    Eigen::Matrix4d T = T_map_ci ? (*T_map_ci) : Eigen::Matrix4d::Identity();

    for(const auto &p : m.points){
        Eigen::Vector4d pi(p.x(),p.y(),p.z(),1.0);
        Eigen::Vector4d pw = T * pi;
        tmp.emplace_back(pw.head<3>().cast<float>());
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

void GLWidget::drawCube(const Eigen::Matrix4d &T, float size)
{
    // 创建立方体的8个顶点
    float half = size * 0.5f;
    std::vector<Eigen::Vector3f> vertices = {
        // 前平面
        {-half, -half, half},  // 0: 左下前
        {half, -half, half},   // 1: 右下前
        {half, half, half},    // 2: 右上前
        {-half, half, half},   // 3: 左上前

        // 后平面
        {-half, -half, -half}, // 4: 左下后
        {half, -half, -half},  // 5: 右下后
        {half, half, -half},   // 6: 右后上
        {-half, half, -half}   // 7: 左后上
    };

    // 绘制线框的索引（12条边）
    std::vector<unsigned int> indices = {
        // 前平面的边
        0,1, 1,2, 2,3, 3,0,
        // 后平面的边
        4,5, 5,6, 6,7, 7,4,
        // 连接前后平面的边
        0,4, 1,5, 2,6, 3,7
    };

    // 创建线框顶点数据
    std::vector<Eigen::Vector3f> lines;
    for (size_t i = 0; i < indices.size(); i += 2) {
        lines.push_back(vertices[indices[i]]);
        lines.push_back(vertices[indices[i+1]]);
    }

    // 绘制线框立方体
    progSimple_.bind();
    progSimple_.setUniformValue("mvp", toQMatrix(proj_ * view_ * T));
    progSimple_.setUniformValue("col", QVector3D(1.0f, 1.0f, 0.0f)); // 黄色

    // 创建临时VAO/VBO
    static QOpenGLVertexArrayObject tempVao;
    static QOpenGLBuffer tempVbo;

    if (!tempVao.isCreated()) {
        tempVao.create();
        tempVbo.create();
    }

    tempVao.bind();
    tempVbo.bind();

    // 上传数据
    tempVbo.allocate(lines.data(), lines.size() * sizeof(Eigen::Vector3f));

    // 设置顶点属性
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, nullptr);

    // 绘制线框
    glLineWidth(2.0f);  // 设置线宽
    glDrawArrays(GL_LINES, 0, lines.size());

    tempVao.release();
    progSimple_.release();
}

