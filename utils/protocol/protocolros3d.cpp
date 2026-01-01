#include "protocolros3d.h"
#include <QJsonDocument>
#include <QJsonValue>
#include <QDebug>
#include <QtEndian>

static const uint8_t CMD_TF   = 0x01;
static const uint8_t CMD_CLOUD = 0x02;
static const uint8_t CMD_MAP  = 0x03;

ProtocolRos3D::ProtocolRos3D(QObject* parent)
    : QObject(parent)
{
}

void ProtocolRos3D::onRawBytes(const QByteArray& data)
{
    buffer_.append(data);
    tryParseFrame();
}

/* 简单 CRC8 (poly=0x07) */
quint8 ProtocolRos3D::crc8(const QByteArray& data)
{
    quint8 crc = 0;
    for (char c : data) {
        crc ^= static_cast<quint8>(c);
        for (int i = 0; i < 8; ++i)
            crc = (crc & 0x80) ? (crc << 1) ^ 0x07 : crc << 1;
    }
    return crc;
}

void ProtocolRos3D::tryParseFrame()
{
    while (buffer_.contains('\n')) {
        int eol = buffer_.indexOf('\n');          // 0x0A
        QByteArray line = buffer_.left(eol);      // 不含 0x0A
        buffer_.remove(0, eol + 1);               // 把这行扔掉

        if (line.size() < 4) continue;

        /* 只验头 */
        if (!line.startsWith(QByteArray("\xAA", 1))) {
            qWarning() << "头标记异常，丢弃行:" << line.toHex(' ');
            continue;
        }
        /* 取命令字 */
        uint8_t cmd = static_cast<uint8_t>(line.at(1));
        /* 取 JSON 部分 */
        QByteArray jsonBytes = line.mid(2);
        QJsonParseError err;
        QJsonDocument doc = QJsonDocument::fromJson(jsonBytes, &err);
        if (err.error != QJsonParseError::NoError) {
            qWarning() << "JSON 解析失败:" << err.errorString();
            continue;
        }
        parseJsonFrame(cmd, doc.object());
    }
}

/* 根据 cmd 分发 */
void ProtocolRos3D::parseJsonFrame(uint8_t cmd, const QJsonObject& obj)
{
    switch (cmd) {
    case CMD_TF:   parseTF(obj);   break;
    case CMD_CLOUD: parseCloud(obj); break;
    case CMD_MAP:  parseMap(obj);  break;
    default: qWarning() << "unknown cmd" << cmd;
    }
}

/* TF 解析 */
void ProtocolRos3D::parseTF(const QJsonObject& obj)
{
    qDebug() << "TF数据解析前："<< obj;
    TFMsg m;
    m.frame_id       = obj["frame_id"].toString();
    m.child_frame_id = obj["child_frame_id"].toString();
    m.t              = QVector3D(obj["x"].toDouble(),
                    obj["y"].toDouble(),
                    obj["z"].toDouble());
    m.q              = QQuaternion(obj["qw"].toDouble(),
                      obj["qx"].toDouble(),
                      obj["qy"].toDouble(),
                      obj["qz"].toDouble());
    // qDebug() << "TF数据解析成TFMsg："<< m;
    emit tfUpdated(m);
}

/* 点云解析 */
void ProtocolRos3D::parseCloud(const QJsonObject& obj)
{
    CloudMsg m;
    m.frame_id = obj["frame_id"].toString();
    m.points   = extractXYZFromPointCloud2(
        obj["data"].toString().toLatin1(),
        obj["width"].toInt(),
        obj["height"].toInt(),
        obj["point_step"].toInt(),
        obj["is_dense"].toBool());
    qDebug() << "CloudMsg: " << m;
    emit cloudUpdated(m);
}

/* 地图解析 */
void ProtocolRos3D::parseMap(const QJsonObject& obj)
{
    MapMsg m;
    m.width      = obj["width"].toInt();
    m.height     = obj["height"].toInt();
    m.resolution = obj["resolution"].toDouble();
    m.origin_x   = obj["origin_x"].toDouble();
    m.origin_y   = obj["origin_y"].toDouble();

    /* 假设收到的 cells 是 RLE 压缩后的 base64-int8 数组 */
    QByteArray rleBa = QByteArray::fromBase64(obj["cells"].toString().toLatin1());
    QVector<int8_t> rle;
    rle.resize(rleBa.size());
    memcpy(rle.data(), rleBa.constData(), rleBa.size());
    m.cells = decompressRLE(rle);
    qDebug() << "MapMsg: " << m;
    emit mapUpdated(m);
}

/* -------------- 工具实现 -------------- */

QVector<QVector3D> ProtocolRos3D::extractXYZFromPointCloud2(
    const QByteArray& base64data,
    quint32 width, quint32 height, quint32 point_step,
    bool is_dense)
{
    QByteArray blob = QByteArray::fromBase64(base64data);
    const char* ptr = blob.constData();
    QVector<QVector3D> pts;
    pts.reserve(width * height);

    for (uint i = 0; i < width * height; ++i) {
        const char* off = ptr + i * point_step;
        float x, y, z;
        memcpy(&x, off + 0, 4);
        memcpy(&y, off + 4, 4);
        memcpy(&z, off + 8, 4);
        if (!is_dense) {
            /* 简单跳过 NaN/Inf */
            if (qIsNaN(x) || qIsNaN(y) || qIsNaN(z) ||
                qIsInf(x) || qIsInf(y) || qIsInf(z)) continue;
        }
        pts.append(QVector3D(x, y, z));
    }
    return pts;
}

QVector<int8_t> ProtocolRos3D::decompressRLE(const QVector<int8_t>& rle)
{
    QVector<int8_t> out;
    for (int i = 0; i + 1 < rle.size(); i += 2) {
        int8_t cnt = rle[i];
        int8_t val = rle[i + 1];
        for (int k = 0; k < cnt; ++k) out.append(val);
    }
    return out;
}
