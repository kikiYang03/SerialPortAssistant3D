#include "glwidget.h"
#include <Eigen/Dense>
#include <QMouseEvent>
#include <QDebug>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ============= 新增：去重工具 ============= */
namespace {
constexpr float VOXEL_RES = 0.1f;          // 10 cm 体素

inline std::tuple<int32_t,int32_t,int32_t> posToVoxel(const Eigen::Vector3f& p)
{
    return { int32_t(p.x() / VOXEL_RES),
            int32_t(p.y() / VOXEL_RES),
            int32_t(p.z() / VOXEL_RES) };
}

inline uint64_t voxelHash(int32_t x, int32_t y, int32_t z)
{
    auto pair = [](uint64_t a, uint64_t b){
        return (a + b) * (a + b + 1) / 2 + b;
    };
    return pair(pair(uint64_t(x), uint64_t(y)), uint64_t(z));
}
}
/* ======================================== */

static inline void rotToYPR_ZYX(const Eigen::Matrix3d& R,
                                double& yaw, double& pitch, double& roll)
{

    yaw = std::atan2(R(1,0), R(0,0));

    double sp = -R(2,0);
    if (sp <= -1.0) pitch = -M_PI/2.0;
    else if (sp >= 1.0) pitch =  M_PI/2.0;
    else pitch = std::asin(sp);

    roll = std::atan2(R(2,1), R(2,2));
}

// =========== 新增：坐标转换辅助函数 ===========
Eigen::Vector3d GLWidget::transformPointToMap(const Eigen::Vector3d& pt_in_camera_init)
{
    if (!hasReceivedMapToCameraInitTf_) {
        return pt_in_camera_init;  // 未收到TF时原样返回
    }

    // camera_init → map 变换
    Eigen::Vector4d homog(pt_in_camera_init.x(),
                          pt_in_camera_init.y(),
                          pt_in_camera_init.z(), 1.0);
    Eigen::Vector4d transformed = T_map_ci_ * homog;
    return Eigen::Vector3d(transformed.x(), transformed.y(), transformed.z());
}


GLWidget::GLWidget(QWidget *parent)
    : QOpenGLWidget(parent)
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


    // 初始化静态几何体 VAO/VBO
    createAxisGeometry();
    createGridGeometry();
    createTrailVAO();
    createArrowGeometry();
    setCamera();

    mapVoxelSet_.clear();   // <-- 新增
    // 初始化TF相关标志
    glReady_ = true;

    hasReceivedMapToCameraInitTf_ = false;
    hasReceivedBodyToBaseLinkTf_ = false;   // 已存在，确保为false
    T_map_ci_.setIdentity();
    T_ci_map_.setIdentity();
    T_body_baselink_.setIdentity();         // 新增：初始化静态TF矩阵
    T_map_baselink_.setIdentity();

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
    Eigen::Matrix4d S = Eigen::Matrix4d::Identity();
    S(0, 0) = S(1, 1) = S(2, 2) = len;   // 统一缩放
    Eigen::Matrix4d mvp = proj_ * view_ * T * S;
    progSimple_.setUniformValue("mvp", toQMatrix(mvp));

    vaoAxis_.bind();

    // 加粗坐标轴线
    glLineWidth(3.0f);  // 设置线条宽度为3像素

    progSimple_.setUniformValue("col", QVector3D(1, 0, 0)); glDrawArrays(GL_LINES, 0, 2);
    progSimple_.setUniformValue("col", QVector3D(0, 1, 0)); glDrawArrays(GL_LINES, 2, 2);
    progSimple_.setUniformValue("col", QVector3D(0, 0, 1)); glDrawArrays(GL_LINES, 4, 2);

    glLineWidth(1.0f);  // 恢复默认线条宽度

    vaoAxis_.release();
    progSimple_.release();
}

/* ---------- 网格 ---------- */
void GLWidget::createGridGeometry()
{
    constexpr int cells = 80;
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

void GLWidget::drawGrid(const Eigen::Matrix4d &T, int cells, float step)
{
    progSimple_.bind();
    progSimple_.setUniformValue("mvp", toQMatrix(proj_ * view_ * T));
    progSimple_.setUniformValue("col", QVector3D(.4, .4, .4));
    vaoGrid_.bind();
    glDrawArrays(GL_LINES, 0, cells * 4 * 2);   // 固定 40*4*2 条线段
    vaoGrid_.release();
    progSimple_.release();
}

/* ---------- 轨迹 ---------- */
void GLWidget::createTrailVAO()
{
    vboTrail_.create();
    vaoTrail_.create();
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
    // progSimple_.setUniformValue("col", QVector3D(1.0f,0.5f,0.0f));
    progSimple_.setUniformValue("col", QVector3D(1.0f,0.0f,0.0f));

    vaoArrow_.bind();
    int cylinderVerts = (16+1)*2;
    glDrawArrays(GL_QUAD_STRIP, 0, cylinderVerts);
    glDrawArrays(GL_TRIANGLES, cylinderVerts, 16*3);
    vaoArrow_.release();
    progSimple_.release();
}

void GLWidget::paintGL()
{
    doUploadCloud();  // 上传实时点云
    doUploadMap();    // 上传地图点云

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);

    // ---------- 1. 计算相机的 view 矩阵（基于 map 坐标系）----------
    Eigen::Matrix4d view = Eigen::Matrix4d::Identity();

    const double ry = yaw_ * M_PI / 180.0;  // 偏航角（yaw）转换为弧度
    const double rp = pitch_ * M_PI / 180.0;  // 俯仰角（pitch）转换为弧度

    Eigen::Matrix4d transBack = Eigen::Matrix4d::Identity();
    transBack(2, 3) = -distance_;  // 沿 Z 轴后移

    Eigen::Matrix4d rotYaw = Eigen::Matrix4d::Identity();
    rotYaw(0, 0) = cos(ry); rotYaw(0, 1) = -sin(ry);
    rotYaw(1, 0) = sin(ry); rotYaw(1, 1) = cos(ry);

    Eigen::Matrix4d rotPitch = Eigen::Matrix4d::Identity();
    rotPitch(1, 1) = cos(rp); rotPitch(1, 2) = -sin(rp);
    rotPitch(2, 1) = sin(rp); rotPitch(2, 2) = cos(rp);

    Eigen::Matrix4d lookCenter = Eigen::Matrix4d::Identity();
    lookCenter(0, 3) = -center_.x();
    lookCenter(1, 3) = -center_.y();
    lookCenter(2, 3) = -center_.z();

    view = transBack * rotPitch * rotYaw * lookCenter;
    view_ = view;

    // ---------- 2. 渲染栅格和坐标轴（直接在 map 坐标系下）----------
    drawAxis(Eigen::Matrix4d::Identity(), 3.0f);  // 直接在 map 坐标系下渲染坐标轴
    drawGrid(Eigen::Matrix4d::Identity(), 40, 1.0f);  // 直接在 map 坐标系下渲染栅格

    // ---------- 3. 渲染其他元素（如轨迹、点云等）----------
    // 渲染轨迹线（白色）—— 现在直接在 map 坐标系下
    if (trail_.points.size() > 1)
    {
        std::vector<Eigen::Vector3f> tmp(trail_.points.begin(), trail_.points.end());

        progSimple_.bind();
        // 关键点：直接使用 proj * view，因为点已经在 map 系下
        progSimple_.setUniformValue("mvp", toQMatrix(proj_ * view_));
        progSimple_.setUniformValue("col", QVector3D(1.0f, 1.0f, 1.0f));

        vaoTrail_.bind();
        vboTrail_.bind();
        vboTrail_.allocate(tmp.data(), tmp.size() * sizeof(Eigen::Vector3f));
        glLineWidth(3.0f);
        glDrawArrays(GL_LINE_STRIP, 0, tmp.size());
        vaoTrail_.release();
        progSimple_.release();
    }

    // 渲染当前帧点云
    if (showRealtimeCloud_ && cloudPts_ > 0) {
        progSimple_.bind();
        const Eigen::Matrix4d mvp = proj_ * view_;
        progSimple_.setUniformValue("mvp", toQMatrix(mvp));
        progSimple_.setUniformValue("col", QVector3D(0.35f, 0.85f, 0.95f));
        vaoCloud_.bind();
        glPointSize(cloudPtSize_);
        glDrawArrays(GL_POINTS, 0, cloudPts_);
        vaoCloud_.release();
        progSimple_.release();
    }

    // 渲染地图点云（彩色，高度着色）
    if (showMapCloud_ && mapPts_ > 0) {
        progColorCloud_.bind();
        progColorCloud_.setUniformValue("mvp", toQMatrix(proj_ * view_));
        progColorCloud_.setUniformValue("uPointSize", mapPtSize_);
        progColorCloud_.setUniformValue("uAlpha", 1.0f);
        progColorCloud_.setUniformValue("uSoftEdge", 0.35f);
        vaoMap_.bind();
        glDrawArrays(GL_POINTS, 0, mapPts_);
        vaoMap_.release();
        progColorCloud_.release();
    }

    // 渲染箭头（map → base_link）
    if (trail_.hasValidTransform &&
        hasReceivedMapToCameraInitTf_ &&
        hasReceivedBodyToBaseLinkTf_)
    {
        progSimple_.bind();
        drawSolidArrow(T_map_baselink_, 1.0f, 0.15f);  // 使用 map → base_link 的变换矩阵
        progSimple_.release();
    }
    // 渲染完成后打印TF状态
    static int frame_cnt = 0;
    if (++frame_cnt % 100 == 0) {
        qDebug() << "TF接收状态:" << hasReceivedMapToCameraInitTf_;
        if (hasReceivedMapToCameraInitTf_) {
            qDebug() << "camera_init在map中的位置:"
                     << T_map_ci_(0,3) << T_map_ci_(1,3) << T_map_ci_(2,3);
        }
    }
}

// =========== 数据处理

/* -------------- 1. 收 TF:俩静态只接收一次，动态一直更新 -------------- */
void GLWidget::onTf(const TFMsg &m)
{
    qDebug() << "收到TF消息:" << m.frame_id << "->" << m.child_frame_id;

    /* ============= 静态TF1: map → camera_init ============= */
    if (m.frame_id == "map" && m.child_frame_id == "camera_init")
    {
        if (hasReceivedMapToCameraInitTf_)
        {
            qDebug() << "  忽略：map→camera_init 静态TF已接收过";
            return;
        }

        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        T.block<3,3>(0,0) = Eigen::Quaterniond(m.q.scalar(), m.q.x(), m.q.y(), m.q.z()).matrix();
        T.block<3,1>(0,3) = Eigen::Vector3d(m.t.x(), m.t.y(), m.t.z());

        T_map_ci_ = T;           // map → camera_init
        T_ci_map_ = T.inverse(); // camera_init → map

        hasReceivedMapToCameraInitTf_ = true;
        qDebug() << "✓ 静态TF已接收: map→camera_init";
    }

    /* ============= 静态TF2: body → base_link ============= */
    else if (m.frame_id == "body" && m.child_frame_id == "base_link")
    {
        if (hasReceivedBodyToBaseLinkTf_)
        {
            qDebug() << "  忽略：body→base_link 静态TF已接收过";
            return;
        }

        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        T.block<3,3>(0,0) = Eigen::Quaterniond(m.q.scalar(), m.q.x(), m.q.y(), m.q.z()).matrix();
        T.block<3,1>(0,3) = Eigen::Vector3d(m.t.x(), m.t.y(), m.t.z());

        T_body_baselink_ = T;  // 存储静态变换

        hasReceivedBodyToBaseLinkTf_ = true;
        qDebug() << "✓ 静态TF已接收: body→base_link";
    }

    /* ============= 动态TF: camera_init → body ============= */
    else if (m.frame_id == "camera_init" && m.child_frame_id == "body")
    {
        // 必须等待两个静态TF都接收完成
        if (!hasReceivedMapToCameraInitTf_ || !hasReceivedBodyToBaseLinkTf_)
        {
            qDebug() << "  等待静态TF："
                     << "map→camera_init=" << hasReceivedMapToCameraInitTf_
                     << " body→base_link=" << hasReceivedBodyToBaseLinkTf_;
            return;
        }

        // 1. 构建 camera_init→body 矩阵
        Eigen::Matrix4d T_ci_b = Eigen::Matrix4d::Identity();
        T_ci_b.block<3,3>(0,0) = Eigen::Quaterniond(m.q.scalar(), m.q.x(), m.q.y(), m.q.z()).matrix();
        T_ci_b.block<3,1>(0,3) = Eigen::Vector3d(m.t.x(), m.t.y(), m.t.z());

        // 2. 计算完整路径：map → base_link
        Eigen::Matrix4d T_map_baselink = T_map_ci_ * T_ci_b * T_body_baselink_;

        // 3. 存储map系下的轨迹点
        Eigen::Vector3f pt = T_map_baselink.block<3,1>(0,3).cast<float>();
        trail_.points.push_back(pt);
        while (trail_.points.size() > kMaxTrail) trail_.points.pop_front();

        // 4. 更新状态
        T_map_baselink_ = T_map_baselink;
        trail_.latestTransform = T_map_baselink;
        trail_.hasValidTransform = true;

        // 5. 发出位姿信号（使用 camera_init 系下的原始值）
        const Eigen::Matrix3d R = T_ci_b.block<3,3>(0,0);
        double yaw, pitch, roll;
        rotToYPR_ZYX(R, yaw, pitch, roll);
        emit tfInfoChanged(T_ci_b(0,3), T_ci_b(1,3), T_ci_b(2,3), yaw, pitch, roll);
    }
}

void GLWidget::onCloud(const CloudMsg &m)
{
    qDebug() << "cloudmsg: "<< m;
    if (m.points.empty()) return;

    for (int i = 0; i < std::min(10, (int)m.points.size()); ++i) {
        qDebug() << "点" << i << ": ("
                 << m.points[i].x() << ", "
                 << m.points[i].y() << ", "
                 << m.points[i].z() << ")";
    }

    QMutexLocker lk(&dataMtx_);

    // 0) 先统计本帧在 map 系下的高度范围（用于归一化）
    float localMinZ_map = std::numeric_limits<float>::max();
    float localMaxZ_map = std::numeric_limits<float>::lowest();

    // 这里直接复用 transformPointToMap，未收到TF时会原样返回（等价用 camera_init.z）
    for (const auto &pt : m.points) {
        Eigen::Vector3d p_ci(pt.x(), pt.y(), pt.z());
        Eigen::Vector3d p_map = transformPointToMap(p_ci);
        float z_map = static_cast<float>(p_map.z());
        localMinZ_map = std::min(localMinZ_map, z_map);
        localMaxZ_map = std::max(localMaxZ_map, z_map);
    }
    if (localMaxZ_map - localMinZ_map < 1e-6f) localMaxZ_map = localMinZ_map + 1.0f;

    // 更新全局范围（map系）
    mapMinZ_ = std::min(mapMinZ_, localMinZ_map);
    mapMaxZ_ = std::max(mapMaxZ_, localMaxZ_map);

    // 1) 转换实时帧到 map 系
    cloudCpu_.clear();
    cloudCpu_.reserve(m.points.size());

    for (const auto &pt : m.points) {
        Eigen::Vector3d p_ci(pt.x(), pt.y(), pt.z());
        Eigen::Vector3d p_map = transformPointToMap(p_ci);
        cloudCpu_.emplace_back(p_map.cast<float>());
    }
    cloudPts_ = static_cast<int>(cloudCpu_.size());
    cloudDirty_.store(true, std::memory_order_release);

    // 2) 累加到地图点云（带体素去重）
    // 去重仍在 camera_init 系下做（保持你的原设计）
    if (hasReceivedMapToCameraInitTf_) {
        qDebug() << "要绘制点云529";
        for (const auto &pt : m.points) {
            Eigen::Vector3d p_ci(pt.x(), pt.y(), pt.z());

            auto [vx, vy, vz] = posToVoxel(p_ci.cast<float>());
            uint64_t h = voxelHash(vx, vy, vz);

            if (mapVoxelSet_.insert(h).second) {
                Eigen::Vector3d p_map = transformPointToMap(p_ci);
                mapInterleavedCpu_.emplace_back(p_map.cast<float>());

                // 颜色：基于 map.z + map系 min/max
                QVector3D c = heightToColor(static_cast<float>(p_map.z()), mapMinZ_, mapMaxZ_);
                mapInterleavedCpu_.emplace_back(c.x(), c.y(), c.z());
            }
        }
    } else {
        // 未收到TF时：transformPointToMap 原样返回，所以 map==camera_init（仍自洽）
        for (const auto &pt : m.points) {
            Eigen::Vector3d p_ci(pt.x(), pt.y(), pt.z());

            auto [vx, vy, vz] = posToVoxel(p_ci.cast<float>());
            uint64_t h = voxelHash(vx, vy, vz);

            if (mapVoxelSet_.insert(h).second) {
                Eigen::Vector3d p_map = transformPointToMap(p_ci);
                mapInterleavedCpu_.emplace_back(p_map.cast<float>());

                QVector3D c = heightToColor(static_cast<float>(p_map.z()), mapMinZ_, mapMaxZ_);
                mapInterleavedCpu_.emplace_back(c.x(), c.y(), c.z());
            }
        }
    }

    // 3) 维护地图点数与上传标志
    mapPts_ = static_cast<int>(mapInterleavedCpu_.size() / 2);
    mapDirty_.store(true, std::memory_order_release);

    if (glReady_) update();
}




/* ---------- 1. 收到 MapCloudMsg 后只做 CPU 侧缓存 + 抛上传任务 ---------- */
void GLWidget::onMap(const MapCloudMsg &m)
{
    if (m.points.empty()) return;
    QMutexLocker lk(&dataMtx_);

    // 1) 统计 map 系 Z 范围（归一化范围）
    mapMinZ_ = std::numeric_limits<float>::max();
    mapMaxZ_ = std::numeric_limits<float>::lowest();

    // 先缓存 camera_init 点（可选；保留你原结构）
    std::vector<Eigen::Vector3d> points_ci;
    points_ci.reserve(m.points.size());

    for (const auto &p : m.points) {
        Eigen::Vector3d p_ci(p.x(), p.y(), p.z());
        points_ci.push_back(p_ci);

        Eigen::Vector3d p_map = transformPointToMap(p_ci);
        float z_map = static_cast<float>(p_map.z());
        mapMinZ_ = std::min(mapMinZ_, z_map);
        mapMaxZ_ = std::max(mapMaxZ_, z_map);
    }
    if (std::abs(mapMaxZ_ - mapMinZ_) < 1e-6f) mapMaxZ_ = mapMinZ_ + 1.0f;

    // 2) 转换到 map 系并组装 interleaved 数据（pos+color）
    mapInterleavedCpu_.clear();
    mapInterleavedCpu_.reserve(m.points.size() * 2);

    for (const Eigen::Vector3d& p_ci : points_ci) {
        Eigen::Vector3d p_map = transformPointToMap(p_ci);

        // 位置
        mapInterleavedCpu_.emplace_back(p_map.cast<float>());

        // 颜色：基于 map.z + map系 min/max
        QVector3D c = heightToColor(static_cast<float>(p_map.z()), mapMinZ_, mapMaxZ_);
        mapInterleavedCpu_.emplace_back(c.x(), c.y(), c.z());
    }

    mapPts_ = static_cast<int>(m.points.size());
    mapDirty_.store(true, std::memory_order_release);
    if (glReady_) update();
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

// 右键鼠标移动
void GLWidget::mouseMoveEvent(QMouseEvent* e)
{
    QPoint delta = e->pos() - lastMousePos_;
    lastMousePos_ = e->pos();

    if (e->buttons() & Qt::LeftButton) {
        pitch_ += delta.y() * 0.5f;   // 鼠标上下 → 俯仰
        yaw_   += delta.x() * 0.5f;   // 鼠标左右 → 偏航
        pitch_ = qBound(-89.0f, pitch_, 89.0f);
    }
    else if (e->buttons() & Qt::RightButton) {
        // 平移视角中心 - 基于当前相机视角方向
        float sensitivity = 0.02f; // 降低基础灵敏度，可配合distance_使用：0.005f * distance_

        // 从当前的yaw和pitch计算相机坐标系的基向量在世界坐标系中的表示
        double yawRad = yaw_ * M_PI / 180.0;
        double pitchRad = pitch_ * M_PI / 180.0;

        double cosYaw = cos(yawRad);
        double sinYaw = sin(yawRad);
        double cosPitch = cos(pitchRad);
        double sinPitch = sin(pitchRad);

        // 相机右向量（世界坐标系）
        Eigen::Vector3d right(cosYaw, -sinYaw, 0);

        // 相机上向量（世界坐标系）
        Eigen::Vector3d up(sinYaw * cosPitch, cosYaw * cosPitch, -sinPitch);

        // 转换鼠标delta到世界坐标系的移动
        // 注意：delta.y()需要取负号，因为Qt的y坐标向下增长，而相机坐标系向上为正
        Eigen::Vector3d deltaWorld =
            right * (delta.x() * sensitivity) -
            up * (delta.y() * sensitivity);

        // 更新相机中心
        center_.setX(center_.x() - deltaWorld.x());
        center_.setY(center_.y() - deltaWorld.y());
        center_.setZ(center_.z() - deltaWorld.z());
    }

    update();
}

void GLWidget::addYaw(int degrees)
{
    yaw_ += degrees;
    update();
}
void GLWidget::addPitch(int degrees)
{
    pitch_ += degrees;
    pitch_ = qBound(-89.0f, pitch_, 89.0f);
    update();
}

void GLWidget::setShowRealtimeCloud(bool show)
{
    if (showRealtimeCloud_ == show) return;
    showRealtimeCloud_ = show;
    update();
}

void GLWidget::setShowMapCloud(bool show)
{
    if (showMapCloud_ == show) return;
    showMapCloud_ = show;
    update();
}

void GLWidget::wheelEvent(QWheelEvent* e)
{
    // 缩放
    distance_ *= (e->angleDelta().y() > 0) ? 0.9f : 1.1f;
    distance_ = std::max(1.0f, std::min(500.0f, distance_));  // 限制范围
    update();
}

QVector3D GLWidget::heightToColor(float z, float minZ, float maxZ)
{

    float n = (z - minZ) / (maxZ - minZ + 1e-6f);
    n = std::clamp(n, 0.0f, 1.0f);

    // 先不要 pow，避免压暗
    float hue = 80.0f + n * (300.0f - 80.0f);

    float sat = 1.0f;
    float val = 1.0f;

    QColor c;
    c.setHsvF(hue / 360.0f, sat, val);
    return QVector3D(c.redF(), c.greenF(), c.blueF());

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
    // 清理地图点云
    {
        QMutexLocker lk(&dataMtx_);
        mapInterleavedCpu_.clear();
        mapPts_ = 0;
        mapDirty_.store(false, std::memory_order_release);
        mapVoxelSet_.clear();   // <-- 关键：把去重哈希也清掉
    }

    // 清理轨迹
    clearTrail();

    // 清理实时点云
    clearCloud();

    // 清空GPU buffer（主线程 / 当前 context）
    QMetaObject::invokeMethod(this, [this](){
        // 清空地图buffer
        vboMap_.bind();
        vboMap_.allocate(nullptr, 0);
        vboMap_.release();

        // 清空点云buffer
        vboCloud_.bind();
        vboCloud_.allocate(nullptr, 0);
        vboCloud_.release();

        update();
    }, Qt::QueuedConnection);
}

// 在GLWidget类中添加以下方法：
void GLWidget::clearTrail()
{
    trail_.points.clear();
    trail_.hasValidTransform = false;
    update();
}

void GLWidget::clearCloud()
{
    QMutexLocker lk(&dataMtx_);
    cloudCpu_.clear();
    cloudPts_ = 0;
    cloudDirty_.store(false, std::memory_order_release);

    // 清空GPU buffer
    QMetaObject::invokeMethod(this, [this](){
        vboCloud_.bind();
        vboCloud_.allocate(nullptr, 0);
        vboCloud_.release();
        update();
    }, Qt::QueuedConnection);
}

void GLWidget::setCamera()
{
    yaw_ = 90.0;
    pitch_ = 0.0;
    distance_ = 15.0;
    center_ = QVector3D(0,0,0);
    update();
}

void GLWidget::saveMapToFile()
{
    qDebug() << "保存地图...";
}
