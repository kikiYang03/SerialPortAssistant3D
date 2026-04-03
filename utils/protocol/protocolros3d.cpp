#include "protocolros3d.h"
#include <QJsonDocument>
#include <QJsonValue>
#include <QDebug>
#include <QtEndian>
#include <QJsonArray>
#include <QJsonObject>
#include <QTimer>


// 新协议命令ID定义
static const uint8_t CMD_ROBOT_POSE  = 0x01;  // 机器人位姿 (map → base_link)
static const uint8_t CMD_LIDAR_POSE  = 0x02;  // 雷达安装位姿 (map → laser)
static const uint8_t CMD_CLOUD       = 0x03;  // 3D点云数据
static const uint8_t CMD_SCAN_2D     = 0x05;  // 2D激光雷达数据
static const uint8_t CMD_MAP_2D      = 0x06;  // 2D地图数据
static const uint8_t CMD_NAV_GOAL    = 0x07;  // 目标点指令
static const uint8_t CMD_GOAL_PATH   = 0x09;  // 最优轨迹线

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
    case CMD_ROBOT_POSE:  parseRobotPose(obj);  break;
    case CMD_LIDAR_POSE:  parseLidarPose(obj);  break;
    case CMD_CLOUD:       parseCloud(obj);      break;
    case CMD_SCAN_2D:     parseScan2D(obj);     break;
    case CMD_MAP_2D:      parseMap2D(obj);      break;
    case CMD_GOAL_PATH:   parseGoalPath(obj);   break;
    default: qWarning() << "unknown cmd" << cmd;
    }
}

/* 全局节流时间戳，单位 ms */
static qint64 g_lastTfLogMS   = 0;
static qint64 g_lastCloudLogMS = 0;

/* 机器人位姿解析 (map → base_link) */
void ProtocolRos3D::parseRobotPose(const QJsonObject& obj)
{
    qint64 nowMS = QDateTime::currentMSecsSinceEpoch();
    if (nowMS - g_lastTfLogMS >= 1000) {
        g_lastTfLogMS = nowMS;
        QString msg = QStringLiteral("接收到机器人位姿: x=%1, y=%2, z=%3")
                          .arg(obj["x"].toDouble(), 0, 'f', 2)
                          .arg(obj["y"].toDouble(), 0, 'f', 2)
                          .arg(obj["z"].toDouble(), 0, 'f', 2);
        AdminMode::appendMessage(msg);
    }

    RobotPoseMsg m;
    m.frame_id       = obj["frame_id"].toString();
    m.child_frame_id = obj["child_frame_id"].toString();
    m.x              = obj["x"].toDouble();
    m.y              = obj["y"].toDouble();
    m.z              = obj["z"].toDouble();
    m.qx             = obj["qx"].toDouble();
    m.qy             = obj["qy"].toDouble();
    m.qz             = obj["qz"].toDouble();
    m.qw             = obj["qw"].toDouble();

    ++m_poseCnt;
    emit robotPoseUpdated(m);
}

/* 雷达位姿解析 (map → laser) */
void ProtocolRos3D::parseLidarPose(const QJsonObject& obj)
{
    LidarPoseMsg m;
    m.frame_id       = obj["frame_id"].toString();
    m.child_frame_id = obj["child_frame_id"].toString();
    m.x              = obj["x"].toDouble();
    m.y              = obj["y"].toDouble();
    m.z              = obj["z"].toDouble();
    m.qx             = obj["qx"].toDouble();
    m.qy             = obj["qy"].toDouble();
    m.qz             = obj["qz"].toDouble();
    m.qw             = obj["qw"].toDouble();

    QString msg = QStringLiteral("接收到雷达位姿: x=%1, y=%2, z=%3")
                      .arg(m.x, 0, 'f', 2).arg(m.y, 0, 'f', 2).arg(m.z, 0, 'f', 2);
    AdminMode::appendMessage(msg);

    emit lidarPoseUpdated(m);
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
    ++m_cloudCnt;  // 点云话题 (0x03+0x05)
    ++m_mapCnt;    // 地图话题 (0x03+0x06)
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

/* 目标轨迹解析 */
void ProtocolRos3D::parseGoalPath(const QJsonObject& obj)
{
    PathMsg m;

    QJsonArray ptsArr = obj["points"].toArray();
    m.points.reserve(ptsArr.size());

    for (int i = 0; i < ptsArr.size(); ++i) {
        QJsonObject pObj = ptsArr[i].toObject();
        m.points.append(QVector3D(
            pObj["x"].toDouble(),
            pObj["y"].toDouble(),
            pObj["z"].toDouble()
            ));
    }

    emit goalPathUpdated(m);
    ++m_pathCnt;
}

/* 2D激光雷达数据解析 (0x05) */
void ProtocolRos3D::parseScan2D(const QJsonObject& obj)
{
    Scan2DMsg m;
    m.frame_id = obj["frame_id"].toString();
    m.angle_min = obj["angle_min"].toDouble();
    m.angle_max = obj["angle_max"].toDouble();
    m.angle_increment = obj["angle_increment"].toDouble();
    m.range_count = obj["range_count"].toInt();

    QJsonArray rangesArr = obj["ranges"].toArray();
    m.ranges.reserve(rangesArr.size());
    for (int i = 0; i < rangesArr.size(); ++i) {
        m.ranges.append(static_cast<float>(rangesArr[i].toDouble()));
    }

    QString msg = QStringLiteral("接收到2D激光数据: %1个点, 角度范围[%2, %3]")
                      .arg(m.range_count)
                      .arg(m.angle_min, 0, 'f', 2)
                      .arg(m.angle_max, 0, 'f', 2);
    AdminMode::appendMessage(msg);

    ++m_cloudCnt;  // 2D激光数据也计入cloud频率
    emit scan2DUpdated(m);
}

/* 2D地图数据解析 (0x06) */
void ProtocolRos3D::parseMap2D(const QJsonObject& obj)
{
    Map2DMsg m;
    m.frame_id = obj["frame_id"].toString();
    m.width = obj["width"].toInt();
    m.height = obj["height"].toInt();
    m.resolution = obj["resolution"].toDouble();
    m.origin_x = obj["origin_x"].toDouble();
    m.origin_y = obj["origin_y"].toDouble();

    // 解析RLE压缩数据
    QJsonArray rleArr = obj["rle"].toArray();
    m.data.clear();
    m.data.reserve(m.width * m.height);

    for (int i = 0; i < rleArr.size(); ++i) {
        QJsonArray pair = rleArr[i].toArray();
        if (pair.size() >= 2) {
            int8_t value = static_cast<int8_t>(pair[0].toInt());
            int count = pair[1].toInt();
            for (int j = 0; j < count; ++j) {
                m.data.append(value);
            }
        }
    }

    QString msg = QStringLiteral("接收到2D地图: %1x%2, 分辨率%3")
                      .arg(m.width).arg(m.height).arg(m.resolution, 0, 'f', 3);
    AdminMode::appendMessage(msg);

    ++m_mapCnt;
    emit map2DUpdated(m);
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

    // 先计算本周期帧数
    quint32 poseFrames  = m_poseCnt  - m_poseLast;
    quint32 cloudFrames = m_cloudCnt - m_cloudLast;
    quint32 mapFrames   = m_mapCnt   - m_mapLast;
    quint32 pathFrames  = m_pathCnt  - m_pathLast;

    // 计算频率
    double poseHz  = poseFrames  / WIN;
    double cloudHz = cloudFrames / WIN;
    double mapHz   = mapFrames   / WIN;
    double pathHz  = pathFrames  / WIN;

    // 更新上周期计数
    m_poseLast   = m_poseCnt;
    m_cloudLast  = m_cloudCnt;
    m_mapLast    = m_mapCnt;
    m_pathLast   = m_pathCnt;

    QString ts = QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss");
    emit appendMessage(QStringLiteral("%1 >> ====== 数据接收频率统计 ======").arg(ts));
    emit appendMessage(QStringLiteral("%1 >> 位姿话题：%2Hz").arg(ts).arg(poseHz, 0, 'f', 2));
    emit appendMessage(QStringLiteral("%1 >> 点云话题：%2Hz").arg(ts).arg(cloudHz, 0, 'f', 2));
    emit appendMessage(QStringLiteral("%1 >> 地图话题：%2Hz").arg(ts).arg(mapHz, 0, 'f', 2));
    emit appendMessage(QStringLiteral("%1 >> 轨迹话题：%2Hz").arg(ts).arg(pathHz, 0, 'f', 2));
    emit appendMessage(QStringLiteral("%1 >> ==============================").arg(ts));
}

// 设置目标点
QByteArray ProtocolRos3D::buildNavGoalFrame(double x, double y, double z, double yaw)
{
    // 1. 构建 JSON 对象
    QJsonObject obj;
    obj["x"] = x;
    obj["y"] = y;
    obj["z"] = z;
    obj["yaw"] = yaw;

    // 2. 转换为紧凑的 JSON 字节串
    QJsonDocument doc(obj);
    QByteArray jsonData = doc.toJson(QJsonDocument::Compact);

    // 3. 组装完整数据帧: AA 07 [JSON] 0A (导航目标点)
    QByteArray frame;
    frame.append(static_cast<char>(0xAA));
    frame.append(static_cast<char>(CMD_NAV_GOAL));
    frame.append(jsonData);
    frame.append(static_cast<char>(0x0A));

    return frame;
}
