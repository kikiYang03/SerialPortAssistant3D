#include "glwidget.h"
#include <Eigen/Dense>
#include <QDebug>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

GLWidget::GLWidget(QWidget *parent) : QOpenGLWidget(parent) {}
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
{ QMatrix4x4 q; for(int i=0;i<4;i++)for(int j=0;j<4;j++)q(i,j)=float(m(j,i)); return q; }

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
    auto eye   = Eigen::Vector3d(0, -30, 10);
    auto center= Eigen::Vector3d(0, 0, 0);
    auto up    = Eigen::Vector3d::UnitZ();

    Eigen::Vector3d f = (center - eye).normalized();
    Eigen::Vector3d s = f.cross(up).normalized();
    Eigen::Vector3d u = s.cross(f);

    Eigen::Matrix4d view = Eigen::Matrix4d::Identity();
    view.block<1,3>(0,0) = s.transpose();
    view.block<1,3>(1,0) = u.transpose();
    view.block<1,3>(2,0) = (-f).transpose();
    view(0,3) = -s.dot(eye);
    view(1,3) = -u.dot(eye);
    view(2,3) =  f.dot(eye);
    view_ = view;


    // 1. map 系下画轴、栅格、相机初始位姿、所有 TF
    if(map_T_camera_init_ready_){
        Eigen::Matrix4d id = Eigen::Matrix4d::Identity();
        drawAxis(id, 10.f);                  // map 自身轴
        drawGrid(id, 40, 1.f);               // 栅格

        if(auto T = tf_.lookup("map","camera_init")){
            drawAxis(*T, 3.f);               // 相机初始位姿
            drawArrow(*T, 2.f);
        }
        // 其余动态 TF 箭头
        for(const auto &[edge, Tdirect] : tf_.allEdges()){
            const auto &[par,chi] = edge;
            if(par == "map") continue;
            auto Tmap = tf_.lookup("map", chi);
            if(Tmap) drawArrow(*Tmap, 2.f);
        }
    }

    // 2. camera_init 系下画点云
    progSimple_.bind();
    progSimple_.setUniformValue("col", QVector3D(0,1,0));
    vaoCloud_.bind(); glDrawArrays(GL_POINTS, 0, cloudPts_); vaoCloud_.release();

    progSimple_.setUniformValue("col", QVector3D(.5,.5,1));
    vaoMap_.bind();   glDrawArrays(GL_POINTS, 0, mapPts_);   vaoMap_.release();
    progSimple_.release();
}

// 槽函数
void GLWidget::onTf(const TFMsg &m)
{
    // 1. 存进树
    tf_.setTransform(m.frame_id, m.child_frame_id,
                     Eigen::Vector3d(m.t.x(),m.t.y(),m.t.z()),
                     Eigen::Quaterniond(m.q.scalar(),m.q.x(),m.q.y(),m.q.z()));

    // 2. 如果是 map→camera_init，则打开开关
    if(m.frame_id == "map" && m.child_frame_id == "camera_init")
        map_T_camera_init_ready_ = true;

    update();   // 重绘
}

void GLWidget::onCloud(const CloudMsg &m)
{
    auto T = tf_.lookup("camera_init", m.frame_id);
    if(!T) return;
    std::vector<Eigen::Vector3f> tmp; tmp.reserve(m.points.size());
    for(const auto &p : m.points){
        Eigen::Vector4d vh(p.x(), p.y(), p.z(), 1.0);
        vh = (*T) * vh;
        tmp.emplace_back(vh.head<3>().cast<float>());
    }
    vboCloud_.bind();
    vboCloud_.allocate(tmp.data(), tmp.size()*sizeof(Eigen::Vector3f));
    cloudPts_ = tmp.size();
    update();
}

void GLWidget::onMap(const MapCloudMsg &m)
{
    auto T = tf_.lookup("camera_init", m.frame_id);
    if(!T) return;
    std::vector<Eigen::Vector3f> tmp; tmp.reserve(m.points.size());
    for(const auto &p : m.points){
        Eigen::Vector4d vh(p.x(), p.y(), p.z(), 1.0);
        vh = (*T) * vh;
        tmp.emplace_back(vh.head<3>().cast<float>());
    }
    vboMap_.bind();
    vboMap_.allocate(tmp.data(), tmp.size()*sizeof(Eigen::Vector3f));
    mapPts_ = tmp.size();
    update();
}
