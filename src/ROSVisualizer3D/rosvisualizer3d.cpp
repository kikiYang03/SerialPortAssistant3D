#include "rosvisualizer3d.h"
#include "ui_rosvisualizer3d.h"

#include <QVBoxLayout>
#include <QJsonArray>
#include <cmath>

ROSVisualizer3D::ROSVisualizer3D(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::ROSVisualizer3D)
{
    ui->setupUi(this);

    glScene_ = new GLScene(this);


    glSceneMap_     = new GLScene(this);

    // 设置“视图坐标系”
    glSceneMap_->setWorldFrame("map");

    // Map 视角默认参数
    // glSceneMap_->setDefaultThirdPersonView(/*distance=*/30.f, /*pitch=*/-60.f, /*yaw=*/180.f);
    auto* layout = new QVBoxLayout(this);
    layout->setMargin(0);
    layout->addWidget(glScene_);
    setLayout(layout);
}

ROSVisualizer3D::~ROSVisualizer3D()
{
    delete ui;
}


bool resolveTFChain(const QString& target,
                    const QString& source,
                    const QMap<QString, Transform>& tfMap,
                    QVector3D& out_t,
                    QQuaternion& out_q)
{
    out_t = QVector3D(0,0,0);
    out_q = QQuaternion(1,0,0,0);

    QString cur = source;

    while (cur != target) {
        if (!tfMap.contains(cur))
            return false;

        const Transform& tf = tfMap[cur];

        // 旋转累积：先已有，再叠加
        out_t = out_t + out_q.rotatedVector(tf.t);
        out_q = out_q * tf.q;

        cur = tf.frame;
    }
    return true;
}

QVector3D calcTF(const QString& target,
                 const QString& source,
                 const QMap<QString, Transform>& map)
{
    QVector3D result(0,0,0);
    QString cur = source;

    while (map.contains(cur)) {
        const Transform& tf = map[cur];
        result += tf.t;
        cur = tf.frame;
        if (cur == target) break;
    }
    return result;
}

void ROSVisualizer3D::onTFUpdated(const TFMsg& msg)
{
    Transform tf;
    tf.frame = msg.frame_id;        // parent
    tf.child = msg.child_frame_id;  // child
    tf.t     = msg.t;
    tf.q     = msg.q;

    tfMap_[tf.child] = tf;

    // 1) body in map：用于显示机器人姿态与轨迹（用户视角是 map）
    QVector3D t_map_body;
    QQuaternion q_map_body;
    if (resolveTFChain("map", "body", tfMap_, t_map_body, q_map_body)) {
        // glScene_->setBodyPoseInWorld(t_map_body, q_map_body);
        glScene_->addTrajectoryPoint(t_map_body);
    }

    // 2) camera_init in map：用于点云一次性模型变换（点云顶点仍是 camera_init 坐标）
    QVector3D t_map_cam;
    QQuaternion q_map_cam;
    if (resolveTFChain("map", "camera_init", tfMap_, t_map_cam, q_map_cam)) {
        glScene_->setCloudPoseInWorld(t_map_cam, q_map_cam);
    }
}


void ROSVisualizer3D::onCloudUpdated(const CloudMsg& msg)
{
    // 点云数据接收是在 camera_init 下
    if (msg.frame_id != "camera_init")
        return;

    QVector<Point3D> pts_cam;
    pts_cam.reserve(msg.points.size());
    for (const auto& p : msg.points) {
        pts_cam.push_back({p.x(), p.y(), p.z()});
    }

    // 关键：点云点保持 camera_init 坐标
    glScene_->setCloudPoints(pts_cam);
}


// ================= Map =================
void ROSVisualizer3D::onMapUpdated(const MapMsg& msg)
{
    QVector<Point3D> mapPts;
    mapPts.reserve(msg.cells.size());

    for (int i = 0; i < msg.cells.size(); ++i) {
        if (msg.cells[i] != 100) continue;

        int x = i % msg.width;
        int y = i / msg.width;

        mapPts.push_back({
            msg.origin_x + x * msg.resolution,
            msg.origin_y + y * msg.resolution,
            0.f
        });
    }

    glScene_->setMapPoints(mapPts);
}

void ROSVisualizer3D::onMapCloudUpdated(const MapCloudMsg& msg)
{
    if (msg.frame_id != "camera_init" && msg.frame_id != "map") {
        // 需要 TF 到 map；这里略
    }

    QVector<Point3D> pts;
    pts.reserve(msg.points.size());
    for (const auto& p : msg.points)
        pts.push_back({p.x(), p.y(), p.z()});

    glScene_->setMapPoints(pts);
}


