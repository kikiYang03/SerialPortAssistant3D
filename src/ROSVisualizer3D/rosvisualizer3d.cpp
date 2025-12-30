#include "rosvisualizer3d.h"
#include "ui_rosvisualizer3d.h"

#include <QVBoxLayout>
#include <QJsonArray>
#include <QMatrix4x4>
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

void ROSVisualizer3D::onProtocolJson(int cmd, const QJsonObject& obj)
{
    switch (cmd) {
    case 0x01: handleTF(obj);   break;
    case 0x02: handleScan(obj); break;
    case 0x03: handleMap(obj);  break;
    default:
        break;
    }
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

void ROSVisualizer3D::handleTF(const QJsonObject& obj)
{
    Transform tf;
    tf.frame = obj["frame_id"].toString();        // parent
    tf.child = obj["child_frame_id"].toString();  // child

    tf.t = QVector3D(
        obj["x"].toDouble(),
        obj["y"].toDouble(),
        obj["z"].toDouble()
        );

    tf.q = QQuaternion(
        obj["qw"].toDouble(),
        obj["qx"].toDouble(),
        obj["qy"].toDouble(),
        obj["qz"].toDouble()
        );

    tfMap_[tf.child] = tf;

    // 可视化整个 TF 树
    glScene_->setTFs(tfMap_.values().toVector());
}


void ROSVisualizer3D::handleScan(const QJsonObject& obj)
{
    QVector<Point3D> pts;

    QVector3D base_t;
    QQuaternion base_q;

    if (!resolveTFChain("map", "base_link", tfMap_, base_t, base_q))
        return;

    double angle = obj["angle_min"].toDouble();
    double inc   = obj["angle_increment"].toDouble();
    QJsonArray ranges = obj["ranges"].toArray();

    for (auto v : ranges) {
        double r = v.toDouble();
        if (r < 0.05) { angle += inc; continue; }

        QVector3D p_local(
            r * cos(angle),
            r * sin(angle),
            0.0
            );

        QVector3D p_map = base_t + base_q.rotatedVector(p_local);

        pts.push_back({ p_map.x(), p_map.y(), p_map.z() });
        angle += inc;
    }

    glScene_->setPointCloud(pts);
}


void ROSVisualizer3D::handleMap(const QJsonObject& obj)
{
    QVector<Point3D> mapPts;

    int width  = obj["width"].toInt();
    int height = obj["height"].toInt();
    double res = obj["resolution"].toDouble();
    double ox  = obj["origin_x"].toDouble();
    double oy  = obj["origin_y"].toDouble();

    QJsonArray rle = obj["rle"].toArray();

    int idx = 0;
    for (auto v : rle) {
        QJsonArray pair = v.toArray();
        int val   = pair[0].toInt();
        int count = pair[1].toInt();

        for (int i = 0; i < count; ++i) {
            if (val == 100) {
                int x = idx % width;
                int y = idx / width;

                Point3D p;
                p.x = ox + x * res;
                p.y = oy + y * res;
                p.z = 0.0f;

                mapPts.push_back(p);
            }
            idx++;
        }
    }

    glScene_->setPointCloud(mapPts);
}


