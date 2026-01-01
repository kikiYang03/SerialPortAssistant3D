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
    tf.frame = msg.frame_id;
    tf.child = msg.child_frame_id;
    tf.t     = msg.t;
    tf.q     = msg.q;

    tfMap_[tf.child] = tf;
    glScene_->setTFs(tfMap_.values().toVector());
}



void ROSVisualizer3D::onCloudUpdated(const CloudMsg& msg)
{
    QVector3D t_map_base;
    QQuaternion q_map_base;

    if (!resolveTFChain("map", msg.frame_id, tfMap_, t_map_base, q_map_base))
        return;

    QVector<Point3D> pts;
    pts.reserve(msg.points.size());
    for (const QVector3D& p_local : msg.points) {
        QVector3D p_map = t_map_base + q_map_base.rotatedVector(p_local);
        pts.push_back({p_map.x(), p_map.y(), p_map.z()});
    }

    glScene_->setCloudPoints(pts);
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

