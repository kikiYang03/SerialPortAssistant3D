#include "glwidget.h"
#include <Eigen/Dense>
#include <QMouseEvent>
#include <QDebug>
#include <QDateTime>
#include <cmath>

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

    hasRobotPose_ = false;
    hasLidarPose_ = false;
    T_map_base_link_.setIdentity();
    T_map_laser_.setIdentity();


    vboOptimalPath_.create();
    vaoOptimalPath_.create();
    vaoOptimalPath_.bind();
    vboOptimalPath_.setUsagePattern(QOpenGLBuffer::DynamicDraw);
    vboOptimalPath_.bind();
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, nullptr);
    vaoOptimalPath_.release();

    // 初始化2D激光雷达VAO/VBO
    vaoScan2D_.create();
    vaoScan2D_.bind();
    vboScan2D_.create();
    vboScan2D_.setUsagePattern(QOpenGLBuffer::DynamicDraw);
    vboScan2D_.bind();
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, nullptr);
    vaoScan2D_.release();

    // 初始化2D地图VAO/VBO (支持颜色)
    vaoMap2D_.create();
    vaoMap2D_.bind();
    vboMap2D_.create();
    vboMap2D_.setUsagePattern(QOpenGLBuffer::DynamicDraw);
    vboMap2D_.bind();
    glEnableVertexAttribArray(0);  // 位置属性
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (void*)0);
    glEnableVertexAttribArray(1);  // 颜色属性
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (void*)(3*sizeof(float)));
    vaoMap2D_.release();

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
    doUploadCloud();   // 上传实时点云
    doUploadMap();     // 上传地图点云
    doUploadScan2D();  // 上传2D激光数据
    doUploadMap2D();   // 上传2D地图数据

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

    // ---------- 3. 渲染点云（先地图，后实时，这样实时点云在上层）----------
    if (hasLidarPose_){
        // 先渲染地图点云（Z=0，在下层）
        if (showMapCloud_ && mapPts_ > 0) {
            if (enableZFilter_) {
                // 过滤地图点云
                progColorCloud_.bind();
                progColorCloud_.setUniformValue("mvp", toQMatrix(proj_ * view_));
                progColorCloud_.setUniformValue("uPointSize", mapPtSize_);
                progColorCloud_.setUniformValue("uAlpha", 1.0f);
                progColorCloud_.setUniformValue("uSoftEdge", 0.35f);

                // 过滤点云数据
                std::vector<Eigen::Vector3f> filteredMap;
                filteredMap.reserve(mapInterleavedCpu_.size());

                // 注意：mapInterleavedCpu_ 是交错的 [位置, 颜色, 位置, 颜色, ...]
                for (size_t i = 0; i < mapInterleavedCpu_.size(); i += 2) {
                    const auto& position = mapInterleavedCpu_[i];
                    const auto& color = mapInterleavedCpu_[i + 1];

                    if (position.z() >= zMinFilter_ && position.z() <= zMaxFilter_) {
                        filteredMap.push_back(position);
                        filteredMap.push_back(color);
                    }
                }

                if (!filteredMap.empty()) {
                    vaoMap_.bind();
                    vboMap_.bind();
                    vboMap_.allocate(filteredMap.data(),
                                     static_cast<int>(filteredMap.size() * sizeof(Eigen::Vector3f)));
                    glDrawArrays(GL_POINTS, 0, static_cast<int>(filteredMap.size() / 2));
                    vboMap_.release();
                    vaoMap_.release();
                }
                progColorCloud_.release();
            } else {
                // 原始渲染逻辑
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
        }

        // 再渲染实时点云（Z=2.0，在上层）
        if (showRealtimeCloud_ && cloudPts_ > 0) {
            if (enableZFilter_) {
                // 使用过滤后的点云
                progSimple_.bind();
                const Eigen::Matrix4d mvp = proj_ * view_;
                progSimple_.setUniformValue("mvp", toQMatrix(mvp));
                progSimple_.setUniformValue("col", QVector3D(0.35f, 0.85f, 0.95f));

                // 过滤点云
                std::vector<Eigen::Vector3f> filteredCloud;
                filteredCloud.reserve(cloudCpu_.size());
                for (const auto& pt : cloudCpu_) {
                    if (pt.z() >= zMinFilter_ && pt.z() <= zMaxFilter_) {
                        filteredCloud.push_back(pt);
                    }
                }

                if (!filteredCloud.empty()) {
                    vaoCloud_.bind();
                    vboCloud_.bind();
                    vboCloud_.allocate(filteredCloud.data(),
                                       static_cast<int>(filteredCloud.size() * sizeof(Eigen::Vector3f)));
                    glPointSize(cloudPtSize_);
                    glDrawArrays(GL_POINTS, 0, static_cast<int>(filteredCloud.size()));
                    vboCloud_.release();
                    vaoCloud_.release();
                }
                progSimple_.release();
            } else {
                // 原始渲染逻辑
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
        }

    }
    else
    {
        // 可以在这里添加一些提示信息或调试输出
        static int frameCnt = 0;
        if (++frameCnt % 60 == 0) {  // 每秒显示一次（假设60fps）
            qDebug() << "等待map->camera_init TF变换，暂不绘制点云...";
        }
    }

    // 绘制去到目标点的轨迹路线
    if (hasOptimalPath_ && !optimalPathPts_.empty())
    {
        progSimple_.bind();
        // 同样直接使用 proj * view，因为点已在 map 系下
        progSimple_.setUniformValue("mvp", toQMatrix(proj_ * view_));
        // 设置为红色以区别于机器人历史轨迹
        progSimple_.setUniformValue("col", QVector3D(1.0f, 0.0f, 0.0f));

        vaoOptimalPath_.bind();
        vboOptimalPath_.bind();
        vboOptimalPath_.allocate(optimalPathPts_.data(),
                                 static_cast<int>(optimalPathPts_.size() * sizeof(Eigen::Vector3f)));

        glLineWidth(4.0f); // 规划路径稍微加粗
        glDrawArrays(GL_LINE_STRIP, 0, static_cast<int>(optimalPathPts_.size()));

        // 绘制路径点（可选，让路径更清晰）
        glPointSize(6.0f);
        glDrawArrays(GL_POINTS, 0, static_cast<int>(optimalPathPts_.size()));

        vaoOptimalPath_.release();
        progSimple_.release();
    }

    // 渲染2D地图数据 (使用颜色着色器) - 先渲染地图(Z=0，下层)
    // 关联到地图点云开关，保持一致的显隐控制
    if (showMapCloud_ && showMap2D_ && map2DPts_ > 0) {
        progColorCloud_.bind();
        progColorCloud_.setUniformValue("mvp", toQMatrix(proj_ * view_));
        progColorCloud_.setUniformValue("uPointSize", map2DPointSize_);
        progColorCloud_.setUniformValue("uAlpha", 1.0f);
        progColorCloud_.setUniformValue("uSoftEdge", 0.0f);
        vaoMap2D_.bind();
        glDrawArrays(GL_POINTS, 0, map2DPts_);
        vaoMap2D_.release();
        progColorCloud_.release();
    }

    // 渲染2D激光雷达数据 (红色) - 后渲染实时点云(Z=0.1，上层)
    // 关联到实时点云开关，保持一致的显隐控制
    if (showRealtimeCloud_ && showScan2D_ && scan2DPts_ > 0 && hasLidarPose_) {
        progSimple_.bind();
        progSimple_.setUniformValue("mvp", toQMatrix(proj_ * view_));
        progSimple_.setUniformValue("col", QVector3D(1.0f, 0.3f, 0.3f)); // 浅红色
        vaoScan2D_.bind();
        glPointSize(4.0f);
        glDrawArrays(GL_POINTS, 0, scan2DPts_);
        vaoScan2D_.release();
        progSimple_.release();
    }

    // ---------- 渲染轨迹线（绿色）- 放在点云之后确保可见 ----------
    if (trail_.points.size() > 1)
    {
        std::vector<Eigen::Vector3f> tmp(trail_.points.begin(), trail_.points.end());

        // 将轨迹点z轴提高0.1，避免被点云遮挡
        for (auto& pt : tmp) {
            pt.z() += 0.1f;
        }

        progSimple_.bind();
        progSimple_.setUniformValue("mvp", toQMatrix(proj_ * view_));
        progSimple_.setUniformValue("col", QVector3D(0.0f, 1.0f, 0.0f)); // 绿色

        vaoTrail_.bind();
        vboTrail_.bind();
        vboTrail_.allocate(tmp.data(), tmp.size() * sizeof(Eigen::Vector3f));
        glLineWidth(5.0f);  // 增加线宽，确保在点云上方可见
        glDrawArrays(GL_LINE_STRIP, 0, tmp.size());
        vaoTrail_.release();
        progSimple_.release();
    }

    // 添加箭头渲染的Z轴过滤
    if (trail_.hasValidTransform && hasRobotPose_)
    {
        // 检查箭头位置是否在Z轴范围内
        Eigen::Vector3d arrowPos = T_map_base_link_.block<3,1>(0,3);
        if (!enableZFilter_ || (arrowPos.z() >= zMinFilter_ && arrowPos.z() <= zMaxFilter_)) {
            progSimple_.bind();
            drawSolidArrow(T_map_base_link_, 1.0f, 0.15f);
            progSimple_.release();
        }
    }

    // 绘制指点模式的箭头（未完成：绿色，已完成：黄色）
    if (hasNavTarget_ || isNavGoalSet_) {
        Eigen::Matrix4d T_nav = Eigen::Matrix4d::Identity();
        T_nav.block<3,1>(0,3) = navTarget3D_;
        T_nav.block<3,3>(0,0) = Eigen::AngleAxisd(navYaw_, Eigen::Vector3d::UnitZ()).matrix();

        Eigen::Matrix4d Rfix = Eigen::Matrix4d::Identity();
        double a = M_PI/2.0;
        Rfix(0,0)=cos(a);  Rfix(0,2)=sin(a);
        Rfix(2,0)=-sin(a); Rfix(2,2)=cos(a);

        progSimple_.bind();
        Eigen::Matrix4d mvp = proj_ * view_ * T_nav * Rfix;
        progSimple_.setUniformValue("mvp", toQMatrix(mvp));

        // 判断状态给颜色
        if (isNavMode_) {
            progSimple_.setUniformValue("col", QVector3D(0.0f, 1.0f, 0.0f)); // 绿色：正在指点
        } else {
            progSimple_.setUniformValue("col", QVector3D(1.0f, 1.0f, 0.0f)); // 黄色：已确认目标
        }

        vaoArrow_.bind();
        int cylinderVerts = (16+1)*2;
        glDrawArrays(GL_QUAD_STRIP, 0, cylinderVerts);
        glDrawArrays(GL_TRIANGLES, cylinderVerts, 16*3);
        vaoArrow_.release();
        progSimple_.release();
    }
    // 渲染完成后打印TF状态
    // static int frame_cnt = 0;
    // if (++frame_cnt % 100 == 0) {
    //     qDebug() << "TF接收状态:" << hasReceivedMapToCameraInitTf_;
    //     if (hasReceivedMapToCameraInitTf_) {
    //         qDebug() << "camera_init在map中的位置:"
    //                  << T_map_ci_(0,3) << T_map_ci_(1,3) << T_map_ci_(2,3);
    //     }
    // }
}

// 添加公共方法用于设置Z轴范围：
void GLWidget::setZFilterRange(float minZ, float maxZ)
{
    zMinFilter_ = minZ;
    zMaxFilter_ = maxZ;
    update();
}

void GLWidget::setZFilterEnabled(bool enabled)
{
    enableZFilter_ = enabled;
    update();
}

// =========== 数据处理

/* -------------- 新协议：机器人位姿处理 -------------- */
void GLWidget::onRobotPose(const RobotPoseMsg &m)
{
    // 构建变换矩阵 map → base_link
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3,3>(0,0) = Eigen::Quaterniond(m.qw, m.qx, m.qy, m.qz).matrix();
    T.block<3,1>(0,3) = Eigen::Vector3d(m.x, m.y, m.z);

    T_map_base_link_ = T;
    hasRobotPose_ = true;

    // 更新轨迹
    Eigen::Vector3f pt = T.block<3,1>(0,3).cast<float>();
    trail_.points.push_back(pt);
    while (trail_.points.size() > kMaxTrail) trail_.points.pop_front();

    trail_.latestTransform = T;
    trail_.hasValidTransform = true;

    // 发出位姿信号
    const Eigen::Matrix3d &R = T.block<3,3>(0,0);
    double ypr_yaw, ypr_pitch, ypr_roll;
    rotToYPR_ZYX(R, ypr_yaw, ypr_pitch, ypr_roll);

    int deg_yaw   = static_cast<int>(std::lround(ypr_yaw   * 180.0 / M_PI));
    int deg_pitch = static_cast<int>(std::lround(ypr_pitch * 180.0 / M_PI));
    int deg_roll  = static_cast<int>(std::lround(ypr_roll  * 180.0 / M_PI));

    emit tfInfoChanged(m.x, m.y, m.z, deg_yaw, deg_pitch, deg_roll);
}

/* -------------- 新协议：雷达位姿处理 -------------- */
void GLWidget::onLidarPose(const LidarPoseMsg &m)
{
    // 构建变换矩阵 map → laser
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3,3>(0,0) = Eigen::Quaterniond(m.qw, m.qx, m.qy, m.qz).matrix();
    T.block<3,1>(0,3) = Eigen::Vector3d(m.x, m.y, m.z);

    T_map_laser_ = T;
    hasLidarPose_ = true;
}

void GLWidget::onCloud(const CloudMsg &m)
{
    if (m.points.empty()) return;

    // 必须等待雷达位姿才能处理点云
    if (!hasLidarPose_) {
        return;
    }

    QMutexLocker lk(&dataMtx_);

    // 统计本帧在 map 系下的高度范围
    float localMinZ_map = std::numeric_limits<float>::max();
    float localMaxZ_map = std::numeric_limits<float>::lowest();

    for (const auto &pt : m.points) {
        // 点云在 laser 坐标系下，变换到 map 系
        Eigen::Vector4d p_laser(pt.x(), pt.y(), pt.z(), 1.0);
        Eigen::Vector4d p_map = T_map_laser_ * p_laser;
        float z_map = static_cast<float>(p_map.z());
        localMinZ_map = std::min(localMinZ_map, z_map);
        localMaxZ_map = std::max(localMaxZ_map, z_map);
    }
    if (localMaxZ_map - localMinZ_map < 1e-6f) localMaxZ_map = localMinZ_map + 1.0f;

    // 更新全局范围（map系）
    mapMinZ_ = std::min(mapMinZ_, localMinZ_map);
    mapMaxZ_ = std::max(mapMaxZ_, localMaxZ_map);

    // 转换实时帧到 map 系
    cloudCpu_.clear();
    cloudCpu_.reserve(m.points.size());

    for (const auto &pt : m.points) {
        Eigen::Vector4d p_laser(pt.x(), pt.y(), pt.z(), 1.0);
        Eigen::Vector4d p_map = T_map_laser_ * p_laser;
        Eigen::Vector3f p_result = p_map.head<3>().cast<float>();

        // 2D模式下实时点云Z轴统一设为0.5（在地图点云之上）
        if (is2DMode_) {
            p_result.z() = 2.0f;
        }

        cloudCpu_.push_back(p_result);
    }
    cloudPts_ = static_cast<int>(cloudCpu_.size());
    cloudDirty_.store(true, std::memory_order_release);

    // 累加到地图点云（带体素去重）
    for (const auto &pt : m.points) {
        // 在 laser 系下做去重
        auto [vx, vy, vz] = posToVoxel(Eigen::Vector3f(pt.x(), pt.y(), pt.z()));
        uint64_t h = voxelHash(vx, vy, vz);

        if (mapVoxelSet_.insert(h).second) {
            // 变换到 map 系
            Eigen::Vector4d p_laser(pt.x(), pt.y(), pt.z(), 1.0);
            Eigen::Vector4d p_map = T_map_laser_ * p_laser;
            Eigen::Vector3f p_result = p_map.head<3>().cast<float>();

            // 2D模式下地图点云Z轴统一设为0
            if (is2DMode_) {
                p_result.z() = 0.0f;
            }

            mapInterleavedCpu_.push_back(p_result);

            // 颜色：基于 map.z（2D模式下使用固定Z值计算颜色）
            float z_for_color = is2DMode_ ? 0.0f : static_cast<float>(p_map.z());
            QVector3D c = heightToColor(z_for_color, mapMinZ_, mapMaxZ_);
            mapInterleavedCpu_.emplace_back(c.x(), c.y(), c.z());
        }
    }

    // 维护地图点数与上传标志
    mapPts_ = static_cast<int>(mapInterleavedCpu_.size() / 2);
    mapDirty_.store(true, std::memory_order_release);

    if (glReady_) update();
}




/* ---------- onMap 已移除，新协议中不再使用 ---------- */

void GLWidget::onGoalPath(const PathMsg &m)
{
    if (m.points.empty()) return;

    QMutexLocker lk(&dataMtx_);
    optimalPathPts_.clear();
    optimalPathPts_.reserve(m.points.size());

    for (const auto &pt : m.points) {
        // 转换到 map 坐标系（与点云和机器人位置对齐）
        Eigen::Vector3d p_ci(pt.x(), pt.y(), pt.z());

        // 2D模式下，将Z轴值统一设为0.1
        if (is2DMode_) {
            p_ci.z() = 0.1;
        }

        // Eigen::Vector3d p_map = transformPointToMap(p_ci);
        optimalPathPts_.emplace_back(p_ci.cast<float>());
    }

    hasOptimalPath_ = true;
    update(); // 触发重绘
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

void GLWidget::doUploadScan2D()
{
    QMutexLocker lk(&dataMtx_);
    if (!scan2DDirty_.load(std::memory_order_acquire)) return;

    if (scan2DPts_ > 0) {
        vaoScan2D_.bind();
        vboScan2D_.bind();
        vboScan2D_.allocate(scan2DCpu_.data(),
                            static_cast<int>(scan2DCpu_.size() * sizeof(Eigen::Vector3f)));
        vaoScan2D_.release();
    }
    scan2DDirty_.store(false, std::memory_order_release);
}

void GLWidget::doUploadMap2D()
{
    QMutexLocker lk(&dataMtx_);
    if (!map2DDirty_.load(std::memory_order_acquire)) return;

    if (map2DPts_ > 0) {
        vaoMap2D_.bind();
        vboMap2D_.bind();
        vboMap2D_.allocate(map2DCpu_.data(),
                           static_cast<int>(map2DCpu_.size() * sizeof(Eigen::Vector3f)));
        vaoMap2D_.release();
    }
    map2DDirty_.store(false, std::memory_order_release);
}


// ===========交互处理
void GLWidget::mousePressEvent(QMouseEvent* e)
{
    lastMousePos_ = e->pos();

    // 拦截：如果处于指点模式，并且按下左键
    if (isNavMode_ && e->button() == Qt::LeftButton) {
        // 1. 按下瞬间：确定起点位置 (X, Y)，保持 Z 不变
        navTarget3D_ = screenToWorld(e->pos(), navTarget3D_.z());
        hasNavTarget_ = true;
        isDraggingNavGoal_ = true; // 开始拖动状态

        // 发送更新信号给UI
        emit navTargetUpdated(navTarget3D_.x(), navTarget3D_.y(), navTarget3D_.z(), navYaw_ * 180.0 / M_PI);
        update();
        return; // 拦截事件，不再往下走
    }
}

// 右键鼠标移动
void GLWidget::mouseMoveEvent(QMouseEvent* e)
{
    QPoint delta = e->pos() - lastMousePos_;
    lastMousePos_ = e->pos();

    // 2. 拖动过程：如果是指标模式且正在拖动，计算 Yaw 角度
    if (isNavMode_ && hasNavTarget_ && isDraggingNavGoal_) {
        // 获取当前鼠标在 Z=targetZ 平面上的世界坐标
        Eigen::Vector3d dragPos = screenToWorld(e->pos(), navTarget3D_.z());

        // 计算起点 (navTarget3D_) 到当前拖动点 (dragPos) 的向量
        double dx = dragPos.x() - navTarget3D_.x();
        double dy = dragPos.y() - navTarget3D_.y();

        // 避免原地抖动：当拖动距离大于一小段阈值时，才更新角度
        if (std::hypot(dx, dy) > 0.05) {
            navYaw_ = std::atan2(dy, dx);

            // 实时更新UI和画面
            emit navTargetUpdated(navTarget3D_.x(), navTarget3D_.y(), navTarget3D_.z(), navYaw_ * 180.0 / M_PI);
            update();
        }
        return; // 拦截事件，防止触发视角平移/旋转
    }

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
    // 清理TF数据
    hasRobotPose_ = false;
    hasLidarPose_ = false;
    T_map_base_link_.setIdentity();
    T_map_laser_.setIdentity();
    // 清理地图点云
    {
        QMutexLocker lk(&dataMtx_);
        mapInterleavedCpu_.clear();
        mapPts_ = 0;
        mapDirty_.store(false, std::memory_order_release);
        mapVoxelSet_.clear();   // <-- 关键：把去重哈希也清掉

        // 清空Z值范围
        mapMinZ_ = std::numeric_limits<float>::max();
        mapMaxZ_ = std::numeric_limits<float>::lowest();

        // 重置Z轴范围
        // zMinFilter_ = -5.0f;   // 修改：默认最小Z值-5m
        // zMaxFilter_ =  20.0f;  // 修改：默认最大Z值20m
        // enableZFilter_ = false; // 是否启用Z轴过滤
    }


    // 清理轨迹
    clearTrail();

    // 清理实时点云
    clearCloud();

    // 清理2D数据
    {
        QMutexLocker lk(&dataMtx_);
        scan2DCpu_.clear();
        scan2DPts_ = 0;
        scan2DDirty_.store(false, std::memory_order_release);

        map2DCpu_.clear();
        map2DPts_ = 0;
        map2DDirty_.store(false, std::memory_order_release);
    }

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

        // 清空2D数据buffer
        vboScan2D_.bind();
        vboScan2D_.allocate(nullptr, 0);
        vboScan2D_.release();

        vboMap2D_.bind();
        vboMap2D_.allocate(nullptr, 0);
        vboMap2D_.release();

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


void GLWidget::setNavMode(bool enable)
{
    isNavMode_ = enable;
    if (enable) {
        // 进入模式：如果已经有目标点了，变为绿色(未确认状态)，保持位置不变
        isNavGoalSet_ = false;
    } else {
        // 退出模式：锁定为黄色并发送目标信号
        if (hasNavTarget_) {
            isNavGoalSet_ = true;
            emit navGoalSet(navTarget3D_.x(), navTarget3D_.y(), navTarget3D_.z(), navYaw_);
        }
    }
    update();
}

Eigen::Vector3d GLWidget::screenToWorld(const QPoint& pos, double targetZ)
{
    // 1. 转 NDC 坐标 [-1, 1]
    double x_ndc = (2.0 * pos.x()) / width() - 1.0;
    double y_ndc = 1.0 - (2.0 * pos.y()) / height();

    Eigen::Vector4d ray_clip(x_ndc, y_ndc, -1.0, 1.0);

    // 2. 转相机坐标系
    Eigen::Vector4d ray_eye = proj_.inverse() * ray_clip;
    ray_eye.z() = -1.0;
    ray_eye.w() = 0.0;

    // 3. 转世界坐标系 (方向向量)
    Eigen::Vector4d ray_wor_4d = view_.inverse() * ray_eye;
    Eigen::Vector3d ray_wor(ray_wor_4d.x(), ray_wor_4d.y(), ray_wor_4d.z());
    ray_wor.normalize();

    // 4. 获取相机世界坐标
    Eigen::Vector4d cam_pos_4d = view_.inverse() * Eigen::Vector4d(0, 0, 0, 1);
    Eigen::Vector3d cam_pos(cam_pos_4d.x(), cam_pos_4d.y(), cam_pos_4d.z());

    // 5. 与 Z = targetZ 平面求交 (修改这里)
    if (std::abs(ray_wor.z()) > 1e-6) {
        double t = (targetZ - cam_pos.z()) / ray_wor.z();
        if (t > 0) {
            return cam_pos + t * ray_wor;
        }
    }
    return Eigen::Vector3d(0, 0, targetZ); // 兜底返回当前Z高度
}

void GLWidget::mouseReleaseEvent(QMouseEvent* e)
{
    // 3. 松开鼠标：结束拖动状态，箭头成型
    if (e->button() == Qt::LeftButton && isDraggingNavGoal_) {
        isDraggingNavGoal_ = false;
        // 如果你需要在这里触发“确认”操作，可以发信号，
        // 不过你目前的设计是按“退出指点模式”按钮才算最终确认，所以这里只改状态即可。
    }
}

void GLWidget::updateNavTargetFromUI(double x, double y, double z, double yaw_deg)
{
    navTarget3D_ = Eigen::Vector3d(x, y, z);
    navYaw_ = yaw_deg * M_PI / 180.0;
    hasNavTarget_ = true; // 用户在界面输入也算作生成了目标
    update();
}

/* -------------- 2D激光雷达数据处理 -------------- */
void GLWidget::onScan2D(const Scan2DMsg &msg)
{
    // 首次进入2D模式时，清空已有的3D点云地图数据
    if (!is2DMode_) {
        is2DMode_ = true;
        // 清空已有的地图点云，让它重新累积（Z值会统一设为0）
        QMutexLocker lk(&dataMtx_);
        mapInterleavedCpu_.clear();
        mapPts_ = 0;
        mapDirty_.store(false, std::memory_order_release);
        mapVoxelSet_.clear();
        cloudCpu_.clear();
        cloudPts_ = 0;
        cloudDirty_.store(false, std::memory_order_release);
    }

    // 必须等待雷达位姿才能处理点云
    if (!hasLidarPose_) {
        return;
    }

    QMutexLocker lk(&dataMtx_);
    scan2DCpu_.clear();
    scan2DCpu_.reserve(msg.range_count);

    for (int i = 0; i < msg.range_count; ++i) {
        float r = msg.ranges[i];
        if (r <= 0 || std::isinf(r) || std::isnan(r)) continue;

        double angle = msg.angle_min + i * msg.angle_increment;

        // 在 laser_link 坐标系下的点
        float x = r * static_cast<float>(cos(angle));
        float y = r * static_cast<float>(sin(angle));
        float z = 0;  // 2D激光雷达，z=0

        // 变换到 map 坐标系
        Eigen::Vector4d p_laser(x, y, z, 1.0);
        Eigen::Vector4d p_map = T_map_laser_ * p_laser;
        Eigen::Vector3f p_result = p_map.head<3>().cast<float>();

        // 2D实时点云Z轴统一设为0.1（在地图点云之上）
        p_result.z() = 0.1f;

        scan2DCpu_.push_back(p_result);
    }

    scan2DPts_ = static_cast<int>(scan2DCpu_.size());
    scan2DDirty_.store(true, std::memory_order_release);

    if (glReady_) update();  // 触发 paintGL 在 GL 线程上传数据
}

/* -------------- 2D地图数据处理 -------------- */
void GLWidget::onMap2D(const Map2DMsg &msg)
{
    if (msg.data.isEmpty()) return;

    QMutexLocker lk(&dataMtx_);
    map2DCpu_.clear();

    int step = map2DStep_;  // 采样步长
    int estPts = (msg.width / step) * (msg.height / step);
    map2DCpu_.reserve(estPts * 2);  // 位置+颜色

    // 地图直接生成，不做坐标变化
    for (int y = 0; y < msg.height; y += step) {
        for (int x = 0; x < msg.width; x += step) {
            int idx = y * msg.width + x;
            if (idx >= msg.data.size()) break;

            int8_t val = msg.data[idx];

            // 计算位置
            float px = static_cast<float>(msg.origin_x + x * msg.resolution);
            float py = static_cast<float>(msg.origin_y + y * msg.resolution);
            float pz = 0;  // 2D地图在Z=0平面

            // 计算颜色: ROS occupancy: 0=自由, 100=障碍, -1=未知
            float gray;
            if (val < 0) {
                // 未知区域显示为深灰色
                gray = 0.3f;
            } else {
                // val=0(自由)→白色, val=100(障碍)→黑色
                gray = 1.0f - (val / 100.0f);
                gray = std::clamp(gray, 0.0f, 1.0f);
            }

            map2DCpu_.emplace_back(px, py, pz);       // 位置
            map2DCpu_.emplace_back(gray, gray, gray); // 颜色 (灰度)
        }
    }

    map2DPts_ = static_cast<int>(map2DCpu_.size() / 2);
    map2DDirty_.store(true, std::memory_order_release);

    if (glReady_) update();  // 触发 paintGL 在 GL 线程上传数据
}

void GLWidget::setShowScan2D(bool show)
{
    if (showScan2D_ == show) return;
    showScan2D_ = show;
    update();
}

void GLWidget::setShowMap2D(bool show)
{
    if (showMap2D_ == show) return;
    showMap2D_ = show;
    update();
}

void GLWidget::setMap2DPointSize(float size)
{
    map2DPointSize_ = size;
    update();
}

void GLWidget::setMap2DStep(int step)
{
    map2DStep_ = std::max(1, step);
    // 注意：改变步长需要重新处理地图数据，这里只更新参数
    // 实际效果需要等下一次收到地图数据时生效
}
