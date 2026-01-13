#include "glwidget.h"
#include <Eigen/Dense>
#include <QMouseEvent>
#include <QDebug>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static inline void rotToYPR_ZYX(const Eigen::Matrix3d& R,
                                double& yaw, double& pitch, double& roll)
{
    // ZYX: R = Rz(yaw)*Ry(pitch)*Rx(roll)
    // yaw   = atan2(R10, R00)
    // pitch = asin(-R20)   (注意数值夹紧)
    // roll  = atan2(R21, R22)

    yaw = std::atan2(R(1,0), R(0,0));

    double sp = -R(2,0);
    if (sp <= -1.0) pitch = -M_PI/2.0;
    else if (sp >= 1.0) pitch =  M_PI/2.0;
    else pitch = std::asin(sp);

    roll = std::atan2(R(2,1), R(2,2));
}


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
    glDepthFunc(GL_LESS);

    // RViz 点云观感关键：开启混合（软边/透明叠加）
    glDepthMask(GL_TRUE);
    glDisable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // 建议开启多重采样（需要你的 QSurfaceFormat 也设置 samples）
    glEnable(GL_MULTISAMPLE);
    glEnable(GL_PROGRAM_POINT_SIZE);
    glEnable(GL_POINT_SPRITE);


    // （可选）如果你后面用了 gl_PointCoord 的圆点shader，不需要 GL_POINT_SMOOTH
    // glDisable(GL_POINT_SMOOTH);


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
    // ===== 新增：彩色点云着色器 =====
    progColorCloud_.addShaderFromSourceCode(QOpenGLShader::Vertex,
                                            "#version 330 core\n"
                                            "layout(location=0) in vec3 aPos;\n"
                                            "layout(location=1) in vec3 aColor;\n"
                                            "out vec3 vColor;\n"
                                            "uniform mat4 mvp;\n"
                                            "uniform float uPointSize;\n"
                                            "void main() {\n"
                                            "    vColor = aColor;\n"
                                            "    gl_Position = mvp * vec4(aPos,1.0);\n"
                                            "    gl_PointSize = uPointSize;\n"
                                            "}\n");


    progColorCloud_.addShaderFromSourceCode(QOpenGLShader::Fragment,
                                            "#version 330 core\n"
                                            "in vec3 vColor;\n"
                                            "out vec4 fragCol;\n"
                                            "uniform float uAlpha;\n"
                                            "uniform float uSoftEdge;\n" // 0.0~0.5，越大越柔
                                            "void main() {\n"
                                            "    vec2 p = gl_PointCoord * 2.0 - 1.0;\n"
                                            "    float r2 = dot(p,p);\n"
                                            "    if (r2 > 1.0) discard;\n"
                                            "    float a = 1.0;\n"
                                            "    if (uSoftEdge > 0.0) {\n"
                                            "        // r2: 0中心 -> 1边缘，soft edge 做衰减\n"
                                            "        float edge0 = 1.0;\n"
                                            "        float edge1 = 1.0 - uSoftEdge;\n"
                                            "        a = smoothstep(edge0, edge1, r2);\n"
                                            "    }\n"
                                            "    fragCol = vec4(vColor, uAlpha * a);\n"
                                            "}\n");

    progColorCloud_.link();
    // ===== 结束新增 =====

    vaoCloud_.create();  vaoCloud_.bind();
    vboCloud_.create();  vboCloud_.setUsagePattern(QOpenGLBuffer::DynamicDraw);
    vboCloud_.bind();
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0,3,GL_FLOAT,GL_FALSE,0,nullptr);
    vaoCloud_.release();

    // ===== 修改：为地图点云创建带颜色的VAO =====
    vaoMap_.create();
    vaoMap_.bind();
    vboMap_.create();
    vboMap_.setUsagePattern(QOpenGLBuffer::DynamicDraw);
    vboMap_.bind();
    glEnableVertexAttribArray(0);  // 位置属性
    glVertexAttribPointer(0,3,GL_FLOAT,GL_FALSE,6*sizeof(float),(void*)0);
    glEnableVertexAttribArray(1);  // 颜色属性（新增）
    glVertexAttribPointer(1,3,GL_FLOAT,GL_FALSE,6*sizeof(float),(void*)(3*sizeof(float)));
    vaoMap_.release();

    // 静态 TF
    // tf_.setTransform("map","camera_init", Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity());
    tf_.setTransform("body","base_link", Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity());

    // 初始化静态几何体 VAO/VBO
    createAxisGeometry();
    createGridGeometry();
    createTrailVAO();
    createArrowGeometry();

    glReady_ = true;

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

/* ---------- 坐标轴 ---------- */
void GLWidget::createAxisGeometry()
{
    static const std::array<Eigen::Vector3f,6> pts{{
        {0,0,0},{1,0,0}, {0,0,0},{0,1,0}, {0,0,0},{0,0,1}
    }};

    vboAxis_.create();  vaoAxis_.create();
    vaoAxis_.bind();
    vboAxis_.setUsagePattern(QOpenGLBuffer::StaticDraw);
    vboAxis_.bind();
    vboAxis_.allocate(pts.data(), pts.size()*sizeof(Eigen::Vector3f));
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0,3,GL_FLOAT,GL_FALSE,0,nullptr);
    vaoAxis_.release();
}

void GLWidget::drawAxis(const Eigen::Matrix4d &T, float len)
{
    progSimple_.bind();
    progSimple_.setUniformValue("mvp", toQMatrix(proj_*view_*T));
    vaoAxis_.bind();
    progSimple_.setUniformValue("col", QVector3D(1,0,0)); glDrawArrays(GL_LINES,0,2);
    progSimple_.setUniformValue("col", QVector3D(0,1,0)); glDrawArrays(GL_LINES,2,2);
    progSimple_.setUniformValue("col", QVector3D(0,0,1)); glDrawArrays(GL_LINES,4,2);
    vaoAxis_.release();
    progSimple_.release();
}

/* ---------- 网格 ---------- */
void GLWidget::createGridGeometry()
{
    constexpr int cells = 40;
    constexpr float step = 1.0f;
    std::vector<Eigen::Vector3f> lines;
    lines.reserve(cells*4*2);
    float ext = cells*step*0.5f;
    for(int i=0;i<=cells;++i){
        float x = -ext + i*step;
        lines.emplace_back(x,-ext,0); lines.emplace_back(x, ext,0);
        lines.emplace_back(-ext,x,0); lines.emplace_back( ext,x,0);
    }

    vboGrid_.create();  vaoGrid_.create();
    vaoGrid_.bind();
    vboGrid_.setUsagePattern(QOpenGLBuffer::StaticDraw);
    vboGrid_.bind();
    vboGrid_.allocate(lines.data(), lines.size()*sizeof(Eigen::Vector3f));
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0,3,GL_FLOAT,GL_FALSE,0,nullptr);
    vaoGrid_.release();
}

void GLWidget::drawGrid(const Eigen::Matrix4d &T, int /*cells*/, float /*step*/)
{
    progSimple_.bind();
    progSimple_.setUniformValue("mvp", toQMatrix(proj_*view_*T));
    progSimple_.setUniformValue("col", QVector3D(.4,.4,.4));
    vaoGrid_.bind();
    glDrawArrays(GL_LINES, 0, 40*4*2);   // 固定 40*4*2 条线段
    vaoGrid_.release();
    progSimple_.release();
}

/* ---------- 轨迹 ---------- */
void GLWidget::createTrailVAO()
{
    vboTrail_.create();  vaoTrail_.create();
    vaoTrail_.bind();
    vboTrail_.setUsagePattern(QOpenGLBuffer::DynamicDraw);
    vboTrail_.bind();
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0,3,GL_FLOAT,GL_FALSE,0,nullptr);
    vaoTrail_.release();
}

/* ---------- 箭头 ---------- */
void GLWidget::createArrowGeometry()
{
    constexpr int seg = 16;
    constexpr float len   = 1.0f;
    constexpr float radius= 0.08f;
    const float shaftLen = len*0.65f, headLen=len-shaftLen, headRad=radius*2.0f;

    std::vector<Eigen::Vector3f> vtx;
    /* 圆柱杆 */
    for(int i=0;i<=seg;++i){
        float a = 2.0f*M_PI*i/seg;
        float x=cosf(a)*radius, y=sinf(a)*radius;
        vtx.emplace_back(x,y,0); vtx.emplace_back(x,y,shaftLen);
    }
    /* 圆锥头 */
    for(int i=0;i<seg;++i){
        float a0=2.0f*M_PI*i/seg, a1=2.0f*M_PI*(i+1)/seg;
        float x0=cosf(a0)*headRad, y0=sinf(a0)*headRad;
        float x1=cosf(a1)*headRad, y1=sinf(a1)*headRad;
        vtx.emplace_back(0,0,shaftLen+headLen);  // 尖
        vtx.emplace_back(x1,y1,shaftLen);
        vtx.emplace_back(x0,y0,shaftLen);
    }

    vboArrow_.create();  vaoArrow_.create();
    vaoArrow_.bind();
    vboArrow_.setUsagePattern(QOpenGLBuffer::StaticDraw);
    vboArrow_.bind();
    vboArrow_.allocate(vtx.data(), vtx.size()*sizeof(Eigen::Vector3f));
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0,3,GL_FLOAT,GL_FALSE,0,nullptr);
    vaoArrow_.release();
}

void GLWidget::drawSolidArrow(const Eigen::Matrix4d &T, float len, float radius)
{
    progSimple_.bind();
    Eigen::Matrix4d Rfix = Eigen::Matrix4d::Identity();
    double a = M_PI/2.0;
    Rfix(0,0)=cos(a); Rfix(0,2)= sin(a);
    Rfix(2,0)=-sin(a); Rfix(2,2)=cos(a);
    Eigen::Matrix4d mvp = proj_*view_*T*Rfix;
    progSimple_.setUniformValue("mvp", toQMatrix(mvp));
    progSimple_.setUniformValue("col", QVector3D(1.0f,0.5f,0.0f));

    vaoArrow_.bind();
    int cylinderVerts = (16+1)*2;
    glDrawArrays(GL_QUAD_STRIP, 0, cylinderVerts);
    glDrawArrays(GL_TRIANGLES, cylinderVerts, 16*3);
    vaoArrow_.release();
    progSimple_.release();
}

void GLWidget::paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);

    // ---------- 0. 上传 VBO（保证 OpenGL context current） ----------
    {
        QMutexLocker lk(&dataMtx_);
        if (cloudDirty_.exchange(false, std::memory_order_acq_rel)) {
            vboCloud_.bind();
            vboCloud_.allocate(cloudCpu_.data(),
                               static_cast<int>(cloudCpu_.size() * sizeof(Eigen::Vector3f)));
            vboCloud_.release();
        }
    }

    // ---------- 1. 计算轨道球 view 矩阵（camera_init 系） ----------
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

    // ---------- 2. 世界系（camera_init）基础元素 ----------
    Eigen::Matrix4d world = Eigen::Matrix4d::Identity();

    // ---------- 3. 把 map 当刚体：用最新 map→camera_init 的逆 ----------
    Eigen::Matrix4d T_ci_map = T_map_ci__latest_.inverse();
    drawAxis(T_ci_map, 10.0f);
    drawGrid(T_ci_map, 40, 1.0f);

    // ---------- 4. 轨迹线（绿色）—— base_link 原点在 camera_init 系 ----------
    if (trail_.points.size() > 1)
    {
        /* deque -> vector 拿到连续内存 */
        std::vector<Eigen::Vector3f> tmp(trail_.points.begin(),
                                         trail_.points.end());

        progSimple_.bind();
        progSimple_.setUniformValue("mvp", toQMatrix(proj_ * view_));
        progSimple_.setUniformValue("col", QVector3D(0,1,0));

        vaoTrail_.bind();
        vboTrail_.bind();
        vboTrail_.allocate(tmp.data(),
                           tmp.size() * sizeof(Eigen::Vector3f));
        glLineWidth(3.0f);
        glDrawArrays(GL_LINE_STRIP, 0, tmp.size());
        vaoTrail_.release();
        progSimple_.release();
    }

    // ---------- 5. 最新箭头（camera_init 系） ----------
    if (trail_.hasValidTransform)
    {
        progSimple_.bind();
        drawSolidArrow(trail_.latestTransform, 1.0f, 0.15f);
        progSimple_.release();
    }

    // ---------- 6. 当前帧点云（红色） ----------
    {
        progSimple_.bind();
        const Eigen::Matrix4d mvp = proj_ * view_;
        progSimple_.setUniformValue("mvp", toQMatrix(mvp));
        progSimple_.setUniformValue("col", QVector3D(0.35f, 0.85f, 0.95f));


        vaoCloud_.bind();
        if (cloudPts_ > 0) {
            glPointSize(cloudPtSize_);
            glDrawArrays(GL_POINTS, 0, cloudPts_);
        }
        vaoCloud_.release();
        progSimple_.release();
    }

    // ---------- 7. 地图点云（彩色，高度着色） ----------
    if (mapPts_ > 0)
    {
        // progColorCloud_.bind();
        // progColorCloud_.setUniformValue("mvp", toQMatrix(proj_ * view_));

        // vaoMap_.bind();
        // glPointSize(mapPtSize_);
        // glDrawArrays(GL_POINTS, 0, mapPts_);
        // vaoMap_.release();
        progColorCloud_.bind();
        progColorCloud_.setUniformValue("mvp", toQMatrix(proj_ * view_));
        progColorCloud_.setUniformValue("uPointSize", mapPtSize_); // 建议 3~6
        progColorCloud_.setUniformValue("uAlpha", 1.0f);                  // 可试 0.8
        progColorCloud_.setUniformValue("uSoftEdge", 0.35f);              // 0.25~0.45

        vaoMap_.bind();
        glDrawArrays(GL_POINTS, 0, mapPts_);
        vaoMap_.release();

        progColorCloud_.release();


        progColorCloud_.release();
    }
}

// =========== 数据处理

/* -------------- 1. 收 TF：只存最新 -------------- */
void GLWidget::onTf(const TFMsg &m)
{
    // 我们关心的两条边直接存成矩阵，方便后面直接乘
    if (m.frame_id == "map" && m.child_frame_id == "camera_init")
    {
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        T.block<3,3>(0,0) = Eigen::Quaterniond(m.q.scalar(),m.q.x(),m.q.y(),m.q.z()).matrix();
        T.block<3,1>(0,3) = Eigen::Vector3d(m.t.x(),m.t.y(),m.t.z());
        T_map_ci__latest_ = T;                 // map→camera_init

        // 更新 TF 树（保留，万一别处要用）
        tf_.setTransform(m.frame_id, m.child_frame_id,
                         Eigen::Vector3d(m.t.x(),m.t.y(),m.t.z()),
                         Eigen::Quaterniond(m.q.scalar(),m.q.x(),m.q.y(),m.q.z()));
    }
    else if (m.frame_id == "body" && m.child_frame_id == "base_link")
    {
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        T.block<3,3>(0,0) = Eigen::Quaterniond(m.q.scalar(),m.q.x(),m.q.y(),m.q.z()).matrix();
        T.block<3,1>(0,3) = Eigen::Vector3d(m.t.x(),m.t.y(),m.t.z());
        T_body_baselink_latest_ = T;           // body→base_link

        // 更新 TF 树（保留，万一别处要用）
        tf_.setTransform(m.frame_id, m.child_frame_id,
                         Eigen::Vector3d(m.t.x(),m.t.y(),m.t.z()),
                         Eigen::Quaterniond(m.q.scalar(),m.q.x(),m.q.y(),m.q.z()));
    }
    else if (m.frame_id == "camera_init" && m.child_frame_id == "body")
    {
        // 轨迹只跟 camera_init→body 有关
        Eigen::Matrix4d T_ci_b = Eigen::Matrix4d::Identity();
        T_ci_b.block<3,3>(0,0) = Eigen::Quaterniond(m.q.scalar(),m.q.x(),m.q.y(),m.q.z()).matrix();
        T_ci_b.block<3,1>(0,3) = Eigen::Vector3d(m.t.x(),m.t.y(),m.t.z());
        // 如果需要关系转化，则把下面的T_ci_b改为T_ci_bl
        // Eigen::Matrix4d T_ci_bl = T_ci_b * T_body_baselink_latest_;
        Eigen::Vector3f pt = T_ci_b.block<3,1>(0,3).cast<float>();
        trail_.points.push_back(pt);
        while (trail_.points.size() > kMaxTrail) trail_.points.pop_front();

        trail_.latestTransform   = T_ci_b;
        trail_.hasValidTransform = true;


        // 计算 YPR
        const Eigen::Matrix3d R = T_ci_b.block<3,3>(0,0);
        double yaw, pitch, roll;
        rotToYPR_ZYX(R, yaw, pitch, roll);

        emit tfInfoChanged(T_ci_b(0,3), T_ci_b(1,3), T_ci_b(2,3),
                           yaw, pitch, roll);
    }
}


void GLWidget::onCloud(const CloudMsg &m)
{
    if (!glReady_) return;
    QMutexLocker lk(&dataMtx_);
    cloudCpu_.clear();
    cloudCpu_.reserve(m.points.size());
    for (const auto &p : m.points)
        cloudCpu_.emplace_back(p.x(), p.y(), p.z());
    cloudPts_ = static_cast<int>(cloudCpu_.size());
    cloudDirty_.store(true, std::memory_order_release);

    QMetaObject::invokeMethod(this, &GLWidget::doUploadCloud, Qt::QueuedConnection);
}


/* ---------- 1. 收到 MapCloudMsg 后只做 CPU 侧缓存 + 抛上传任务 ---------- */
void GLWidget::onMap(const MapCloudMsg &m)
{
    if (!glReady_) return;
    if (m.points.empty()) return;

    QMutexLocker lk(&dataMtx_);

    /* 1) 统计 Z 范围 */
    mapMinZ_ = std::numeric_limits<float>::max();
    mapMaxZ_ = std::numeric_limits<float>::lowest();
    for (const auto &p : m.points) {
        mapMinZ_ = std::min(mapMinZ_, static_cast<float>(p.z()));
        mapMaxZ_ = std::max(mapMaxZ_, static_cast<float>(p.z()));
    }
    if (std::abs(mapMaxZ_ - mapMinZ_) < 1e-6f) mapMaxZ_ = mapMinZ_ + 1.0f;

    /* 2) 组装 interleaved 数据：pos + color */
    mapInterleavedCpu_.clear();
    mapInterleavedCpu_.reserve(m.points.size() * 2);
    for (const auto &p : m.points) {
        mapInterleavedCpu_.emplace_back(p.x(), p.y(), p.z());   // pos
        QVector3D c = heightToColor(static_cast<float>(p.z()), mapMinZ_, mapMaxZ_);
        mapInterleavedCpu_.emplace_back(c.x(), c.y(), c.z());   // color
    }

    mapPts_ = static_cast<int>(m.points.size());
    mapDirty_.store(true, std::memory_order_release);

    /* 3) 立即在主线程上传（OpenGL context 当前） */
    QMetaObject::invokeMethod(this, &GLWidget::doUploadMap, Qt::QueuedConnection);
}

/* ---------- 2. 真正的 GPU 上传（主线程执行） ---------- */
void GLWidget::doUploadCloud()
{
    QMutexLocker lk(&dataMtx_);
    if (!cloudDirty_.load(std::memory_order_acquire)) return;
    vboCloud_.bind();
    vboCloud_.allocate(cloudCpu_.data(),
                       static_cast<int>(cloudCpu_.size() * sizeof(Eigen::Vector3f)));
    vboCloud_.release();
    cloudDirty_.store(false, std::memory_order_release);
    update();          // 通知重绘
}
void GLWidget::doUploadMap()
{
    QMutexLocker lk(&dataMtx_);
    if (!mapDirty_.load(std::memory_order_acquire)) return;

    vboMap_.bind();
    vboMap_.allocate(mapInterleavedCpu_.data(),
                     static_cast<int>(mapInterleavedCpu_.size() * sizeof(Eigen::Vector3f)));
    vboMap_.release();
    mapDirty_.store(false, std::memory_order_release);

    update();   // 通知 Qt 立即重绘
}


// ===========交互处理
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

static inline float clamp01(float v) { return std::max(0.0f, std::min(1.0f, v)); }
QVector3D GLWidget::heightToColor(float z, float minZ, float maxZ)
{
    // // 紫色-绿色
    // float n = (z - minZ) / (maxZ - minZ + 1e-6f);
    // n = std::clamp(n, 0.0f, 1.0f);

    // // 非线性，增强层次
    // n = std::pow(n, 0.7f);

    // // 低->高：黄绿青 -> 蓝紫
    // float hue = 80.0f + n * (300.0f - 80.0f);

    // // 饱和度略低
    // float sat = 0.50f;

    // // 亮度随高度变化（关键）
    // float val = 0.70f + (1.0f - n) * 0.20f;

    // QColor c;
    // c.setHsvF(hue / 360.0f, sat, val);

    // return QVector3D(c.redF(), c.greenF(), c.blueF());

    float n = (z - minZ) / (maxZ - minZ + 1e-6f);
    n = std::clamp(n, 0.0f, 1.0f);

    // 先不要 pow，避免压暗
    float hue = 80.0f + n * (300.0f - 80.0f);

    float sat = 1.0f;
    float val = 1.0f;

    QColor c;
    c.setHsvF(hue / 360.0f, sat, val);
    return QVector3D(c.redF(), c.greenF(), c.blueF());

    // 蓝色-黄色
    // float t = (z - minZ) / (maxZ - minZ + 1e-6f);
    // t = std::pow(t, 0.5f);          // 中间调提亮
    // QColor c;
    // c.setHsvF(0.60f - t*0.55f,     // 0.60→0.05  蓝→黄
    //           0.9f,                // 饱和度高
    //           0.25f + t*0.75f);    // 亮度 0.25→1.0
    // return QVector3D(c.redF(), c.greenF(), c.blueF());

    //灰色
    // float t = (z - minZ) / (maxZ - minZ + 1e-6f);
    // t = std::pow(t, 0.35f);              // 伽马<1，暗部提亮
    // int steps = 8;
    // float v = std::floor(t * steps) / steps;  // 量化成 8 级台阶
    // return QVector3D(v, v, v);

    // 黑白红
    // float t = (z - minZ) / (maxZ - minZ + 1e-6f);
    // t = std::pow(t, 0.4f);
    // t = std::round(t * 5) / 5;   // 只留 6 个离散值
    // if (t < 0.2f) return {0,0,0};          // 黑
    // if (t < 0.4f) return {0.6f,0,0.8f};    // 深洋红
    // if (t < 0.6f) return {1,0.2f,1};       // 亮洋红
    // if (t < 0.8f) return {1,0.8f,1};       // 淡粉
    // return {1,1,1};                        // 白

    // float n = (z - minZ) / (maxZ - minZ + 1e-6f);
    // n = clamp01(n);

    // n = std::pow(n, 0.7f);          // 低区拉伸，高区压缩

    // float hue = 240.f - n * 240.f;  // 240°蓝 → 0°红
    // float sat = 0.55f + 0.25f * std::sin(n * M_PI);  // 两端 0.55，中间 0.8
    // float val = 0.65f + 0.25f * std::sin(n * M_PI);  // 两端 0.65，中间 0.9

    // QColor c = QColor::fromHsvF(hue / 360.f, sat, val);
    // return QVector3D(c.redF(), c.greenF(), c.blueF());
}

QVector3D GLWidget::normalToColor(const Eigen::Vector3f& n)
{
    // 把 nx/ny/nz 直接当 r/g/b，[-1,1]→[0,1]
    auto c = (n.normalized().array() + 1.f) * 0.5f;
    return QVector3D(c.x(), c.y(), c.z());
}

// ========操作栏===============
void GLWidget::clearMap()
{
    {
        QMutexLocker lk(&dataMtx_);
        mapInterleavedCpu_.clear();
        mapPts_ = 0;
        mapDirty_.store(false, std::memory_order_release);
    }

    // 清空 GPU buffer（主线程 / 当前 context）
    QMetaObject::invokeMethod(this, [this](){
        vboMap_.bind();
        vboMap_.allocate(nullptr, 0);
        vboMap_.release();
        update();
    }, Qt::QueuedConnection);
}

void GLWidget::resetCamera()
{
    yaw_ = 0.0;
    pitch_ = 0.0;
    distance_ = 10.0;
    center_ = QVector3D(0,0,0);
    update();
}

void GLWidget::saveMapToFile()
{
    qDebug() << "保存地图...";
}
