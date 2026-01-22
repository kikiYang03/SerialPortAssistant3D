#include "protocolros3d.h"
#include <QJsonDocument>
#include <QJsonValue>
#include <QDebug>
#include <QtEndian>
#include <QJsonArray>
#include <QJsonObject>
#include <QTimer>


static const uint8_t CMD_TF   = 0x01;
static const uint8_t CMD_CLOUD = 0x02;
static const uint8_t CMD_MAP  = 0x03;

ProtocolRos3D::ProtocolRos3D(QObject* parent)
    : QObject(parent)
{
    m_hzTimer = new QTimer(this);
    m_hzTimer->setInterval(10000);   // 1 s
    connect(m_hzTimer, &QTimer::timeout, this, &ProtocolRos3D::calcHz);
    m_hzTimer->start();
}

void ProtocolRos3D::onRawBytes(quint8 cmd, const QByteArray& data)
{
    // data 已经是纯 JSON 字节串，不带 AA/0A 头尾
    QJsonParseError err;
    QJsonDocument doc = QJsonDocument::fromJson(data, &err);
    if (err.error != QJsonParseError::NoError) {
        qWarning() << "JSON 解析失败:" << err.errorString();
        return;
    }
    if (!doc.isObject()) {
        qWarning() << "JSON 不是对象";
        return;
    }
    QJsonObject obj = doc.object();
    // int cmd = obj.value("cmd").toInt(-1);
    if (cmd < 0) {
        qWarning() << "JSON 里缺少 cmd 字段";
        return;
    }
    parseJsonFrame(static_cast<uint8_t>(cmd), obj);
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

/* 全局节流时间戳，单位 ms */
static qint64 g_lastTfLogMS   = 0;
static qint64 g_lastCloudLogMS = 0;

/* TF 解析 */
void ProtocolRos3D::parseTF(const QJsonObject& obj)
{
    /* ---- 节流打印 ---- */
    qint64 nowMS = QDateTime::currentMSecsSinceEpoch();
    if (nowMS - g_lastTfLogMS >= 1000) {      // 距离上次 ≥1 s 才打印
        g_lastTfLogMS = nowMS;
        QString msg = QStringLiteral("接收到TF: %1")
                          .arg(QString::fromUtf8(QJsonDocument(obj).toJson(QJsonDocument::Compact)));
        AdminMode::appendMessage(msg);
    }

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

    if (m.frame_id == "camera_init" && m.child_frame_id == "body"){
        ++m_tfCnt;          // 计数
    }
    emit tfUpdated(m);
}


/* 点云解析 */
void ProtocolRos3D::parseCloud(const QJsonObject& obj)
{
    qint64 nowMS = QDateTime::currentMSecsSinceEpoch();
    if (nowMS - g_lastCloudLogMS >= 1000) {
        g_lastCloudLogMS = nowMS;
        AdminMode::appendMessage(QStringLiteral("接收到Cloud: 不做打印"));
    }

    CloudMsg m;
    m.frame_id = obj["frame_id"].toString();

    const int width      = obj["width"].toInt();
    const int height     = obj["height"].toInt();
    const int point_step = obj["point_step"].toInt();
    const bool is_dense  = obj["is_dense"].toBool();
    const int row_step = obj["row_step"].toInt();


    QByteArray raw;

    // 兼容两种格式：1) data 是 object: {bytes:[...]}  2) data 是 base64 string
    if (obj["data"].isObject()) {
        QJsonObject dataObj = obj["data"].toObject();
        QJsonArray bytesArr = dataObj["bytes"].toArray();

        raw.resize(bytesArr.size());
        for (int i = 0; i < bytesArr.size(); ++i) {
            raw[i] = static_cast<char>(bytesArr[i].toInt()); // 0..255
        }
    } else {
        // 旧格式：base64 string
        QByteArray b64 = obj["data"].toString().toLatin1();
        raw = QByteArray::fromBase64(b64);
    }

    const int expected = width * height * point_step;
    if (raw.size() < expected) {
        qWarning() << "PointCloud raw bytes too small:"
                   << "raw=" << raw.size() << "expected>=" << expected
                   << "width=" << width << "height=" << height << "point_step=" << point_step;
        return;
    }

    m.points = extractXYZFromPointCloud2Raw(raw, width, height, point_step, row_step, is_dense);
    ++m_cloudCnt;
    emit cloudUpdated(m);
}


/* 地图解析 */
void ProtocolRos3D::parseMap(const QJsonObject& obj)
{
    // 新格式：PointCloud2
    MapCloudMsg m;
    m.frame_id = obj["frame_id"].toString();

    const int width      = obj["width"].toInt();
    const int height     = obj["height"].toInt();
    const int point_step = obj["point_step"].toInt();
    const int row_step   = obj["row_step"].toInt();
    const bool is_dense  = obj["is_dense"].toBool();

    QByteArray raw;
    if (obj["data"].isObject()) {
        QJsonObject dataObj = obj["data"].toObject();
        QJsonArray bytesArr = dataObj["bytes"].toArray();
        raw.resize(bytesArr.size());
        for (int i = 0; i < bytesArr.size(); ++i)
            raw[i] = static_cast<char>(bytesArr[i].toInt());
    } else {
        raw = QByteArray::fromBase64(obj["data"].toString().toLatin1());
    }

    const int expectedMin = height * row_step; // 更稳妥
    if (raw.size() < expectedMin) {
        qWarning() << "Map PointCloud raw bytes too small:"
                   << "raw=" << raw.size() << "expected>=" << expectedMin
                   << "width=" << width << "height=" << height
                   << "point_step=" << point_step << "row_step=" << row_step;
        return;
    }

    m.points = extractXYZFromPointCloud2Raw(raw, width, height, point_step, row_step, is_dense);
    // qDebug() << "MapCloudMsg: " << m.points.size();
    // ++m_mapCnt;
    emit mapCloudUpdated(m);
}


/* -------------- 工具实现 -------------- */

QVector<QVector3D> ProtocolRos3D::extractXYZFromPointCloud2Raw(
    const QByteArray& raw,
    quint32 width, quint32 height, quint32 point_step, quint32 row_step,
    bool is_dense)
{
    const char* base = raw.constData();

    QVector<QVector3D> pts;
    pts.reserve(static_cast<int>(width * height));

    for (quint32 r = 0; r < height; ++r) {
        const char* row = base + r * row_step;
        for (quint32 c = 0; c < width; ++c) {
            const char* off = row + c * point_step;

            float x, y, z;
            memcpy(&x, off + 0, 4);
            memcpy(&y, off + 4, 4);
            memcpy(&z, off + 8, 4);

            if (qIsNaN(x) || qIsNaN(y) || qIsNaN(z) ||
                qIsInf(x) || qIsInf(y) || qIsInf(z)) {
                continue;
            }
            pts.append(QVector3D(x, y, z));
        }
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

void ProtocolRos3D::calcHz()
{
    if (!m_linkAlive) return;
    constexpr double WIN = 10.0;
    double tfHz   = (m_tfCnt   - m_tfLast)   / WIN;
    double scanHz = (m_cloudCnt - m_cloudLast) / WIN;
    // double mapHz  = (m_mapCnt  - m_mapLast)  / WIN;

    m_tfLast    = m_tfCnt;
    m_cloudLast = m_cloudCnt;
    // m_mapLast   = m_mapCnt;

    QString msg = QStringLiteral("%1 >> 话题统计:  /tf = %2 Hz, /cloud = %3 Hz")
                      .arg(QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss"))
                      .arg(tfHz,   0, 'f', 2)
                      .arg(scanHz, 0, 'f', 2);

    emit appendMessage(msg);
}
