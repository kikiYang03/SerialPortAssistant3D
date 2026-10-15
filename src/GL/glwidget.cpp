#include "glwidget.h"
#include <Eigen/Dense>
#include <QMouseEvent>
#include <QDebug>
#include <QDateTime>
#include <QTimer>
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

// 「定位状态」显示位（工单 08）的判定节拍。**判定本身不是这个数**——「已定位 / 未定位」
// 取的是模块里那个 kFreshWindow（恰好 500 ms 仍算新鲜），这个节拍只决定「多久去看一眼」：
// 翻牌的延迟最多一个节拍，取窗口的 1/5（100 ms），不自己另立一个超时值。
// 反过来，节拍**不能**靠「帧到达」来驱动：0x02 停发之后恰恰不再有事件，
// 而 0x01 那条同频的 10 Hz 照旧在发，拿它当节拍会得到一个永远停在「已定位」的标签。
constexpr int kLocalizationTickMs = 100;

GLWidget::GLWidget(QWidget *parent)
    : QOpenGLWidget(parent)
{
    // 只问不记：到点就去问模块「被 0x02 喂的那条边还新不新鲜」，文案变了才发信号。
    localizationTimer_ = new QTimer(this);
    localizationTimer_->setInterval(kLocalizationTickMs);
    connect(localizationTimer_, &QTimer::timeout,
            this, &GLWidget::refreshLocalizationStatus);
    localizationTimer_->start();
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

    // 位姿不放这儿初始化：tf_ 默认就是「两条边都没到过」，
    // 而「没到过」与「到了但等于恒等」在模块里是两个可分辨的状态，别拿恒等兜底。

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

void GLWidget::drawSolidArrow(const Eigen::Matrix4d &T, float len, float radius,
                              const QVector3D &color)
{
    progSimple_.bind();
    Eigen::Matrix4d Rfix = Eigen::Matrix4d::Identity();
    double a = M_PI/2.0;
    Rfix(0,0)=cos(a); Rfix(0,2)= sin(a);
    Rfix(2,0)=-sin(a); Rfix(2,2)=cos(a);
    Eigen::Matrix4d mvp = proj_*view_*T*Rfix;
    progSimple_.setUniformValue("mvp", toQMatrix(mvp));
    // 颜色由调用方给（默认红）：同一个几何体也用来画开机点标记（蓝，工单 09）——
    // 两个箭头同时在图上时，光看形状分不出哪个是机器人。
    progSimple_.setUniformValue("col", color);

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
    //
    // ⚠️ 这里**没有**「收到过位姿才画」这道门，那是工单 06 特意摘掉的：`0x03` 的载荷本来就
    // 在图的原点系里，一上来（还没收到任何位姿时）就该画得出来——操作员打开界面就能核对地图。
    // 点的坐标在 onCloud 里**原样**存下（不做任何变换），这里也不再碰位姿。
    // 别把门加回来：加上它，未定位时点云会**整条不画**，界面上看起来就像卡住了。
    // （2D 扫描那一路仍然要位姿，它的门在下面留着，见工单 07。）
    //
    // 三条路径（实时点云 / 累积点云 / Z 范围过滤）都吃同一份 CPU 缓存
    // （cloudCpu_ 与 mapInterleavedCpu_），坐标口径只由 onCloud 一处决定，
    // 所以三者不可能各画各的——加变换要加在 onCloud，别加在这里。
    //
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
    if (showRealtimeCloud_ && showScan2D_ && scan2DPts_ > 0
        && tf_.edgeTransform(TfTree::Edge::BaseLink).has_value()) {
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

    // 箭头渲染 - 不受Z轴过滤影响，始终显示
    // 矩阵从模块现取（不判新鲜度：机器人「最后一帧停在哪儿」照旧要画得出来）。
    // 清地图时 tf_.clear() 会把这条边一起清掉，箭头随之消失。
    if (const auto T_base_link = tf_.edgeTransform(TfTree::Edge::BaseLink))
    {
        progSimple_.bind();
        drawSolidArrow(*T_base_link, 1.0f, 0.15f);
        progSimple_.release();
    }

    // 开机点标记（工单 09）：**仅定位模式**、且 `0x02` 到过时才画。
    //
    // 与机器人箭头是**同一个几何体，只换颜色**：两个箭头会同时在图上，形状分不出谁是谁，
    // 颜色是这一票唯一的区分手段。取蓝色是因为一圈颜色都用掉了——机器人箭头红、实时点云青、
    // 地图点云按高度配色、2D 扫描浅红、轨迹绿、规划路径红、指点箭头绿/黄。
    //
    // 位置与朝向都取模块那条静态边的**现值**，**不判新鲜度**：开机点是一趟里不变的事实，
    // 不会因为 `0x02` 不再重发就不成立（判那条边新不新鲜是工单 08 那个显示位的事）。
    // 建图模式下压根不画（见 originState()：那时它恒等于图原点，压在图原点坐标轴上只是噪声）。
    if (originState() == OriginState::Ready) {
        if (const auto T_origin = tf_.edgeTransform(TfTree::Edge::BaseInit))
            drawSolidArrow(*T_origin, 1.0f, 0.15f, QVector3D(0.25f, 0.55f, 1.0f));
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

/* -------------- 新协议：机器人位姿处理 (0x01 → base_init_map → base_link) -------------- */
void GLWidget::onRobotPose(const RobotPoseMsg &m)
{
    // 位姿→矩阵这一折只在模块里做一次。Eigen 的四元数构造是 **w 在前**，线上是 qx,qy,qz,qw。
    const Vec3 t(m.x, m.y, m.z);
    const Quat q(m.qw, m.qx, m.qy, m.qz);
    tf_.setPose(TfTree::Edge::BaseLink, t, q, TfTree::Clock::now());

    // 更新轨迹（数值同改动前：直接用载荷里的平移）
    trail_.points.push_back(t.cast<float>());
    while (trail_.points.size() > kMaxTrail) trail_.points.pop_front();

    // 发出位姿信号。取角用的矩阵就是模块摆好的那一份，不在调用点另建一个。
    const auto T_base_link = tf_.edgeTransform(TfTree::Edge::BaseLink);
    if (!T_base_link) return;

    double ypr_yaw, ypr_pitch, ypr_roll;
    rotToYPR_ZYX(T_base_link->block<3,3>(0,0), ypr_yaw, ypr_pitch, ypr_roll);

    int deg_yaw   = static_cast<int>(std::lround(ypr_yaw   * 180.0 / M_PI));
    int deg_pitch = static_cast<int>(std::lround(ypr_pitch * 180.0 / M_PI));
    int deg_roll  = static_cast<int>(std::lround(ypr_roll  * 180.0 / M_PI));

    emit tfInfoChanged(m.x, m.y, m.z, deg_yaw, deg_pitch, deg_roll);
}

/* -------------- 新协议：重定位结果处理 (0x02 → base_init_map → base_init) -------------- */
void GLWidget::onRelocalizationPose(const RelocalizationPoseMsg &m)
{
    // 收下这条边就够了，**到此为止**：v2.8 的 `0x02` 是「这一趟的开机点在图里哪儿」，
    // 它不参与任何坐标变换——以前拿它当雷达位姿折点云，等于把所有激光数据整体
    // 平移一个开机点偏移（安装位姿 T 全 0 时看不出来，正是要消灭的那个静默错）。
    // 显示开机点（工单 09）与「这条边到没到」的判据（工单 08）都从模块里取。
    const Vec3 t(m.x, m.y, m.z);
    const Quat q(m.qw, m.qx, m.qy, m.qz);
    tf_.setPose(TfTree::Edge::BaseInit, t, q, TfTree::Clock::now());

    // 「定位状态」立刻翻成「已定位」（工单 08）：不等下一个节拍，收帧这一下就是最新事实。
    // 反方向（超时）没有事件可等，只能靠定时器到点去看，见 refreshLocalizationStatus()。
    refreshLocalizationStatus();

    // 开机点也立刻跟上（工单 09）：新的一帧 `0x02` = 新的一次重定位，开机点可能挪了地方。
    // 标记的位置、界面上那三个数，都是从这里推出去的（数值原样取自刚喂进去的载荷）。
    publishOriginMarkerState();
}

// 「定位状态」显示位（工单 08）的**唯一**入口：帧到达、清图、定时器到点三处都走它。
//
// 判据**唯一来源**是最后一条 `0x02` 的到达时刻——问的就是模块里被 `0x02` 喂的那条边
// （`Edge::BaseInit`）。**不拿 `0x0B` 状态帧里的 `relocalization` 位顶替**：那是模块的
// **内部状态自述**（进行中 / 成功 / 失败 / 未响应），而界面缺的是「`base_init_map → base_init`
// 这条边发出来了没有」——二者可以不一致（例如丢包），不能互相顶替。`0x0B` 那一路照旧
// 喂「重定位 / 规划」两个标签（在 Ros3DPage 里），与这一位各占各的显示位。
//
// 三种文案都是**当前状态**（不是事件留痕），所以恢复正常时自己会变回来：
//   已定位                  —— 最近一条 `0x02` 还在 kFreshWindow 内（**恰好 500 ms 仍算新鲜**）
//   未定位（0x02 超时）      —— 收到过 `0x02`，但这一条之后已经超过 kFreshWindow 没有新的
//   未定位（还没收到过 0x02）—— 本会话一条都没到过（含模块重启之后）
//
// 「超时」与「还没收到过」分开是本仓既有的规矩（同 Ros3DPage 里「未收到规划状态」与模块那句
// 「未响应」必须显示得不一样）：**链路/模块没发**与**发过但断了**不是一个事实，
// 一上来就说「超时」等于断言一条从没到过的帧超时了。这里也不弹窗、不写日志——
// 位姿在 500 ms 边界上来回抖时它会跟着反复变，那是状态、不是事件（留痕归帧名校验那个红状态位）。
void GLWidget::refreshLocalizationStatus()
{
    QString text;
    if (tf_.isFresh(TfTree::Edge::BaseInit, TfTree::Clock::now())) {
        text = tr("已定位");
    } else if (tf_.edgeTransform(TfTree::Edge::BaseInit).has_value()) {
        // 「这条边到过」问模块即可（有值 = 喂过），不另存一个「0x02 收到过没有」的标志：
        // 多存一个标志就多一处能跟模块对不上的地方。
        text = tr("未定位（0x02 超时）");
    } else {
        text = tr("未定位（还没收到过 0x02）");
    }

    // 节拍 100 ms，文案不变时发信号毫无意义 —— 与工单 05 的红状态位、工单 07 的扫描说明
    // 同一条规矩：只在变化时动一次。
    if (text == localizationStatus_) return;
    localizationStatus_ = text;
    emit localizationStatusChanged(text);
}

/* -------------- 开机点标记（工单 09） -------------- */

// 画不画开机点、界面上那三个数显示什么、不画时说明哪一句 —— **判据只有这一处**。
// （paintGL 画不画、publishOriginMarkerState 发什么，都从它推出来。分开各写一遍，
//  就会出现「图上画着、界面却说没画」这种自相矛盾的状态。）
//
// 判据取参数 `0x13`「模式」，**不从 `0x02` 的数值去猜**：恒等不等于建图，将来 `T ≠ 0` 就会猜错。
// 这里**只认原始上报值**：`0x13` 是模块的**现场写入值**，协议另有一说（`0x14 = 1` 且模块类型
// ∈ {1, 3} 时会被**强制**成定位模式，生效值 ≠ 现场值），要不要按生效值判由本票之外的改动定；
// 本票按票面「模式由 `0x13` 判定」办，只送/只用这一份原始值。
GLWidget::OriginState GLWidget::originState() const
{
    // 只认「明说 1」。`-1`（没读到过）与任何不认识的取值一律按「不知道」算 ——
    // 不知道就不画：按建图处理会多出一个说不清来源的标记，按定位处理则会画一个可能错位的
    // （未读到过是常见态：`0x13` 不是周期帧，用户没点过「读取参数」的整段会话里它都是 -1）。
    if (moduleMode_ == 0) return OriginState::Mapping;
    if (moduleMode_ != 1) return OriginState::UnknownMode;

    // 定位模式：还得 `0x02` 到过。这条边是静态的、只发一次，到过就一直有；
    // **不判新鲜度** —— 开机点是这一趟里不变的事实，`0x02` 不再重发不代表它不成立。
    return tf_.edgeTransform(TfTree::Edge::BaseInit).has_value()
               ? OriginState::Ready
               : OriginState::NoReloc;
}

// 把当前状态推给界面：**一条信号带齐**「画不画 / 为什么」与「数值是多少」。
// 四个调用点：`0x02` 到达、`0x13` 变化、清图（这条边被抹掉）、以及界面构造完之后主动推的第一次。
void GLWidget::publishOriginMarkerState()
{
    // 数值也从模块那条静态边取。它就是 `0x02` 载荷本身：平移是载荷的 x/y/z（原样存进去的），
    // 旋转是载荷的四元数 —— 所以显示出来的数与载荷逐位一致，图形层不另存一份副本。
    const auto T_origin = tf_.edgeTransform(TfTree::Edge::BaseInit);

    QString note;
    double x = 0.0, y = 0.0, yawDeg = 0.0;

    switch (originState()) {
    case OriginState::UnknownMode:
        note = tr("开机点未显示：还没读到过模式参数（0x13）");
        break;
    case OriginState::Mapping:
        // 这句话是**必须**有的：建图模式下这个标记一整场都不画，不说出来
        // 就分不清「本来就不画」和「界面卡住了」（本次三处「可能不画」的第三处）。
        note = tr("开机点未显示：建图模式下开机点就是图原点");
        break;
    case OriginState::NoReloc:
        note = tr("开机点未显示：定位模式下还没收到过重定位结果（0x02）");
        break;
    case OriginState::Ready:
        // Ready 就是「这条边有值」判出来的（见 originState()），这里必然取得到。
        x = (*T_origin)(0, 3);
        y = (*T_origin)(1, 3);
        {
            double yawRad = 0.0, pitch = 0.0, roll = 0.0;
            rotToYPR_ZYX(T_origin->block<3, 3>(0, 0), yawRad, pitch, roll);
            // 取整到度：与「机器人坐标」那一组同一个口径（那边也是整数度）。
            yawDeg = static_cast<double>(std::lround(yawRad * 180.0 / M_PI));
        }
        break;
    }

    // 只在真变了才发（`0x02` 是 10 Hz 重发的）：文案或三个数任一不同才算变。
    // 第一次一定发（originMarkerPublished_）—— 初值是「还没读到过模式」，而它不会再变，
    // 不推这一次，界面上那句说明就永远出不来（信号只在变化时发）。
    if (originMarkerPublished_ && note == originMarkerNote_
        && x == originMarkerX_ && y == originMarkerY_ && yawDeg == originMarkerYawDeg_)
        return;

    originMarkerPublished_ = true;
    originMarkerNote_      = note;
    originMarkerX_         = x;
    originMarkerY_         = y;
    originMarkerYawDeg_    = yawDeg;

    emit originMarkerChanged(note, x, y, yawDeg);
}

void GLWidget::onCloud(const CloudMsg &m)
{
    if (m.points.empty()) return;

    // ⚠️ 这一段是本次（工单 06）新引入的**主动不做事**，两件事都要保住，别被后续改动抹掉：
    //
    //   1. **一行变换都不做**：v2.8 的 `0x03` 载荷已经在图的原点系（`base_init_map`）里，
    //      载荷坐标**就是**图上坐标。v2.6 那套「点云在 laser_link 系、靠位姿嫁接过来」
    //      到此为止（工单 04 先把嫁接依据换成 `0x01` 的 base_link 边，本票把嫁接本身摘掉）。
    //   2. **没有「必须先收到位姿」这道门**：点云本来就不需要位姿。只摘嫁接、忘了摘门，
    //      会造成未定位时点云**整条不画**——界面上看起来就是卡住了，又一处静默错误，
    //      正是工单 06 特意点出的一处。
    //
    // 于是这里既不调 placeInMap()、也不折安装位姿 T：
    //   * placeInMap() 自带「边没喂过 / 超时就不摆放」，对 `0x03` **不适用**——
    //     调它等于把上面那道门又装回去；
    //   * 折 T 等于把整片点云平移一个开机点偏移（实机上 T 全 0 时看不出来，正是要消灭的静默错）。
    //
    // 摘了门之后的版本安全感由**帧名校验**提供（工单 05）：模块若是 v2.6，它发的 `0x03`
    // 父系不是 `base_init_map`，在解析层就被丢了，根本到不了这里。
    // 谁要给 `0x03` 加回变换，先把上面这两条读一遍。
    QMutexLocker lk(&dataMtx_);

    // 首次进入3D模式时，清空已有的2D点云数据
    if (is2DMode_) {
        is2DMode_ = false;
        // 清空已有的2D地图和实时点云数据
        mapInterleavedCpu_.clear();
        mapPts_ = 0;
        mapDirty_.store(false, std::memory_order_release);
        mapVoxelSet_.clear();
        cloudCpu_.clear();
        cloudPts_ = 0;
        cloudDirty_.store(false, std::memory_order_release);

        // 清空2D数据
        scan2DCpu_.clear();
        scan2DPts_ = 0;
        scan2DDirty_.store(false, std::memory_order_release);
        map2DCpu_.clear();
        map2DPts_ = 0;
        map2DDirty_.store(false, std::memory_order_release);
    }

    // 统计本帧的高度范围。图系 = 载荷系，所以直接读载荷的 z，不经过任何变换。
    float localMinZ_map = std::numeric_limits<float>::max();
    float localMaxZ_map = std::numeric_limits<float>::lowest();

    for (const auto &pt : m.points) {
        const float z_map = pt.z();
        localMinZ_map = std::min(localMinZ_map, z_map);
        localMaxZ_map = std::max(localMaxZ_map, z_map);
    }
    if (localMaxZ_map - localMinZ_map < 1e-6f) localMaxZ_map = localMinZ_map + 1.0f;

    // 更新全局范围（map系）
    mapMinZ_ = std::min(mapMinZ_, localMinZ_map);
    mapMaxZ_ = std::max(mapMaxZ_, localMaxZ_map);

    // 实时帧：载荷坐标**原样**存下（图系），不做任何变换——上面那段说的「主动不做事」在这里落成代码
    cloudCpu_.clear();
    cloudCpu_.reserve(m.points.size());

    for (const auto &pt : m.points) {
        Eigen::Vector3f p_result(pt.x(), pt.y(), pt.z());

        // 2D模式下实时点云Z轴统一设为2.0（在地图点云之上；渲染那边也是这个值）
        if (is2DMode_) {
            p_result.z() = 2.0f;
        }

        cloudCpu_.push_back(p_result);
    }
    cloudPts_ = static_cast<int>(cloudCpu_.size());
    cloudDirty_.store(true, std::memory_order_release);

    // 累加到地图点云（带体素去重）
    //
    // 去重用的栅格现在**就是图系里的栅格**：载荷坐标即图上坐标，一个点落在哪一格只由它自己的
    // 坐标决定。以前去重键取的是「子帧」下的坐标（那时点云还要被摆一次，栅格跟着位姿飘），
    // 现在两者合一：机器人动了，同一处障碍物照旧落进同一格，累积地图才在图上站得住。
    for (const auto &pt : m.points) {
        auto [vx, vy, vz] = posToVoxel(Eigen::Vector3f(pt.x(), pt.y(), pt.z()));
        uint64_t h = voxelHash(vx, vy, vz);

        if (mapVoxelSet_.insert(h).second) {
            Eigen::Vector3f p_result(pt.x(), pt.y(), pt.z());

            // 2D模式下地图点云Z轴统一设为0
            if (is2DMode_) {
                p_result.z() = 0.0f;
            }

            mapInterleavedCpu_.push_back(p_result);

            // 颜色：基于 map.z（2D模式下使用固定Z值计算颜色）
            float z_for_color = is2DMode_ ? 0.0f : pt.z();
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
    // 清理位姿：两条边一起清（位姿与到达时刻都清），机器人箭头随之不再绘制
    tf_.clear();

    // 「扫描为什么不画」的那句说明也跟着收掉（工单 07）：它是**由状态推出来的**，
    // 状态刚被整个复位，留着旧句子就是在说一件已经不存在的事。
    // 位姿若仍然陈旧，下一帧扫描到了会重新推出来（setScan2DNote 只在变化时发信号）。
    setScan2DNote(QString());

    // 「定位状态」同一条道理（工单 08）：`0x02` 那条边刚被 clear() 抹掉，显示位立刻回到
    // 「还没收到过」——不等下一个节拍，否则那 100 ms 里它还在说「已定位」，而那位姿已经无处可查。
    // （这条边下一次收帧就会重新点亮它，模块的 `0x02` 是 10 Hz 重发的。）
    refreshLocalizationStatus();

    // 开机点同理（工单 09）：标记画不画、界面上那三个数，都是由这条边推出来的，
    // 边刚被抹掉就得跟着走 —— 留着就是拿上一趟开机点当这一趟的。
    // 定位模式下它会立刻变成「还没收到过重定位结果（0x02）」，那三个数随之从界面上消失。
    publishOriginMarkerState();

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
        clearScan2DCacheLocked();   // 同一个入口，见它上面的说明（工单 07）

        map2DCpu_.clear();
        map2DPts_ = 0;
        map2DDirty_.store(false, std::memory_order_release);
    }

    // 清理目标轨迹（规划路径）
    {
        QMutexLocker lk(&dataMtx_);
        optimalPathPts_.clear();
        hasOptimalPath_ = false;
    }

    // 清理指点模式目标点状态
    hasNavTarget_ = false;
    isNavGoalSet_ = false;
    navTarget3D_ = Eigen::Vector3d(0, 0, is2DMode_ ? 0.0 : 1.0);  // 根据模式设置初始Z值
    navYaw_ = 0.0;

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

        // 清空规划路径buffer
        vboOptimalPath_.bind();
        vboOptimalPath_.allocate(nullptr, 0);
        vboOptimalPath_.release();

        update();
    }, Qt::QueuedConnection);
}

// 参数 0x13「模式」：串口层收到 AA 10 时就地缓存，变化时下发到这里（工单 03）。
// 取值 0 = 建图、1 = 定位、-1 = 本会话还没读到过（模块只在被读取参数时才下发该参数，
// 模块重启或重新连接后要重新读到才算数）。取值一变，开机点标记的画法与那句说明跟着改
// （工单 09）：重绘一次，并把新状态推给界面 —— 建图 ⇄ 定位 切换时两边必须一起变，
// 只重绘不推、或只推不重绘，都会留下「图上画着、界面说没画」这种不一致。
void GLWidget::onModuleModeChanged(int mode)
{
    if (mode == moduleMode_) return;

    moduleMode_ = mode;
    publishOriginMarkerState();
    update();
}

// 在GLWidget类中添加以下方法：
void GLWidget::clearTrail()
{
    // 只清轨迹点：位姿归模块管，这里的按钮不该顺手把机器人箭头也抹掉
    trail_.points.clear();
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
        // 如果还没有目标点，根据当前模式设置初始Z值
        if (!hasNavTarget_) {
            navTarget3D_.z() = is2DMode_ ? 0.0 : 1.0;
        }
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

// 2D 扫描缓存的唯一清理入口（工单 07）：不摆放一帧时要用它把**上一帧**也抹掉。
// **必须在持有 dataMtx_ 时调用**（两个调用点都在锁内）。只清 CPU 缓存与点数：
// 渲染那道门看的是 scan2DPts_，它归零就不会再画；VBO 里剩的旧顶点画不出来，
// 等下一帧有数据时会被整块重新 allocate。
void GLWidget::clearScan2DCacheLocked()
{
    scan2DCpu_.clear();
    scan2DPts_ = 0;
    scan2DDirty_.store(false, std::memory_order_release);
}

// 「2D 扫描为什么不画」的说明（工单 07）：**只在文案变化时**发一次信号。
// 扫描是按 10 Hz 重发的，同一个原因每帧都发一遍毫无意义（与工单 05 的红状态位同一条规矩）。
// 空串 = 正常，界面收到后把标签收起来。**不写日志**：位姿在 500 ms 边界上来回抖时
// 这句话会跟着反复出现，逐次记日志会把接收窗口淹掉——它描述的是**当前状态**，不是事件。
void GLWidget::setScan2DNote(const QString &note)
{
    if (note == scan2DNote_) return;

    scan2DNote_ = note;
    emit scan2DNoteChanged(note);
}

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

    // 摆放依据：最近一帧 `0x01` 的 base_init_map → base_link（工单 04 起）。
    // `0x05` 载荷里既没有帧名也没有位姿，而且**已经**在 base_link 系（模块侧的 /scan
    // 折过安装位姿 T 了），所以这里**不折 T**——再折一次就是用 T 罚两遍。
    // 摆放依据只能借这条边，取**最近一帧**，**不插值**：`0x01` 与 `0x05` 的载荷里都没有时间戳
    // （两份协议文档里都没有），插值没有依据；接受「扫描跟着上一帧位姿」的错位。
    // （帧名同理：`0x05` 载荷里**没有**帧名字段，模块也不发，只能**硬约定**本帧在
    //  `base_link` 系——工单 02 的对照表里它记作「无字段、不可校」。）
    const TfTree::Stamp now = TfTree::Clock::now();
    if (!tf_.isFresh(TfTree::Edge::BaseLink, now)) {
        // 工单 07 新引入的「主动不做事」，两件事必须一起做：
        //   ① 这一帧扫描**不摆放**（placeInMap() 自己也会拒绝，这里提前挡掉，免得白算一场）；
        //   ② **并且清空扫描缓存** —— 只挡住新帧是不够的：上一帧摆好的点还在 scan2DCpu_ 里，
        //      下面的 2D 渲染照画不误，看起来就是扫描**冻在旧位置**上。那比不画更糟：
        //      界面上分不出「扫描没动」和「机器人没动」，又是一个静默错。
        {
            QMutexLocker lk(&dataMtx_);
            clearScan2DCacheLocked();
        }
        // 界面上留一句话：不画**是有原因的**，否则会被当成卡顿。
        // 门槛值从模块里取（kFreshWindow），不在界面上另写一个 500 ms。
        setScan2DNote(tf_.edgeTransform(TfTree::Edge::BaseLink).has_value()
                          ? tr("2D 扫描未绘制：机器人位姿（0x01）已超过 %1 ms 未更新")
                                .arg(TfTree::kFreshWindow.count())
                          : tr("2D 扫描未绘制：还没收到过机器人位姿（0x01）"));
        if (glReady_) update();   // 数据已清，重画一次让屏幕上的旧扫描消失
        return;
    }

    // 位姿是新的：把上面那句「不画」的说明收掉（恢复正常时不留在屏幕上）。
    setScan2DNote(QString());

    QMutexLocker lk(&dataMtx_);
    scan2DCpu_.clear();
    scan2DCpu_.reserve(msg.range_count);

    for (int i = 0; i < msg.range_count; ++i) {
        float r = msg.ranges[i];
        if (r <= 0 || std::isinf(r) || std::isnan(r)) continue;

        double angle = msg.angle_min + i * msg.angle_increment;

        // 在 base_link 系（机器人中心）下的点
        float x = r * static_cast<float>(cos(angle));
        float y = r * static_cast<float>(sin(angle));
        float z = 0;  // 2D激光雷达，z=0

        // 摆进图的原点系
        const auto p_map = tf_.placeInMap(TfTree::Edge::BaseLink,
                                          Vec3(x, y, z), now);
        if (!p_map) continue;
        Eigen::Vector3f p_result = p_map->cast<float>();

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
