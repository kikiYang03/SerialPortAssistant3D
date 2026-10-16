#include "protocolros3d.h"
#include "tftree.h"          // frameNamesAccepted()：帧名期望表**唯一**的所在（工单 02）
#include <QJsonDocument>
#include <QJsonValue>
#include <QDebug>
#include <QtEndian>
#include <QJsonArray>
#include <QJsonObject>
#include <QTimer>


// 新协议命令ID定义
static const uint8_t CMD_ROBOT_POSE  = 0x01;  // 机器人位姿 (base_init → base_link)
static const uint8_t CMD_RELOCALIZATION_POSE = 0x02;  // 重定位结果 (base_init_map → base_init)
static const uint8_t CMD_CLOUD       = 0x03;  // 3D点云数据
static const uint8_t CMD_SCAN_2D     = 0x05;  // 2D激光雷达数据
static const uint8_t CMD_MAP_2D      = 0x06;  // 2D地图数据
static const uint8_t CMD_NAV_GOAL    = 0x07;  // 目标点指令
static const uint8_t CMD_POS_CMD     = 0x08;  // 位置控制指令
static const uint8_t CMD_GOAL_PATH   = 0x09;  // 最优轨迹线
static const uint8_t CMD_NAV_STATUS  = 0x0B;  // 规划状态
static const uint8_t CMD_MAP_ARCHIVE = 0x0C;  // 地图下传分块

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
    // ---- 帧名校验（工单 05）：帧名从「读了不用」变成「读了就校验」----
    //
    // 校验放在这里、只放在这里：本层是「拿到 JSON、知道帧号」的那一层，也是唯一一处
    // 能一网打尽所有帧的地方（下面每个 parseXxx 各查一遍就会漏、就会散）。
    //
    // ⚠️ 这是本次新引入、最容易被后续改动抹掉的**主动拒绝**，别当成多余的防御删掉：
    // 0x01 的父系从 `world` 改成 `base_init_map` 时**载荷字段一个都没变**，
    // 不读帧名的话这种改名就是一条完全静默的通道（改对了看不出来、改错了也看不出来），
    // 而 0x02 的载荷与 v2.6 逐字段相同、只能靠帧名分辨新旧。删掉这次校验，
    // 下一步就是「照折旧位姿」——正是本次要消灭的那个静默错误。
    //
    // 期望值**只有一份**，在 utils/protocol/tftree.cpp 的 kFrameNameExpectations 那张表里。
    // 这里只把载荷里读到的两个名字**原样**交过去判，「期望是什么」本层一概不知道，
    // 也不许另抄一份——那正是那张表要收掉的东西。
    //
    // 判拒绝 ⇒ 丢帧：本函数到此为止，该帧不解析、不渲染，只发一个信号让界面留痕。
    // 0x05 / 0x06（载荷里没有帧名字段）与表外帧号恒判接受——那是「不可校验」，
    // 不是「校验通过」，详见 tftree.h 里那段说明。
    const QString frameId      = obj["frame_id"].toString();
    const QString childFrameId = obj["child_frame_id"].toString();
    if (!frameNamesAccepted(cmd, frameId.toStdString(), childFrameId.toStdString())) {
        emit frameNameRejected(cmd, frameId, childFrameId);
        return;
    }

    switch (cmd) {
    case CMD_ROBOT_POSE:  parseRobotPose(obj);  break;
    case CMD_RELOCALIZATION_POSE: parseRelocalizationPose(obj);  break;
    case CMD_CLOUD:       parseCloud(obj);      break;
    case CMD_SCAN_2D:     parseScan2D(obj);     break;
    case CMD_MAP_2D:      parseMap2D(obj);      break;
    case CMD_POS_CMD:     parsePosCmd(obj);     break;
    case CMD_GOAL_PATH:   parseGoalPath(obj);   break;
    case CMD_NAV_STATUS:  parseNavStatus(obj);  break;
    case CMD_MAP_ARCHIVE: parseMapChunk(obj);   break;
    default: qWarning() << "unknown cmd" << cmd;
    }
}

/* 全局节流时间戳，单位 ms */
static qint64 g_lastTfLogMS   = 0;
static qint64 g_lastCloudLogMS = 0;
static qint64 g_lastPosCmdLogMS = 0;

/* 机器人位姿解析 (base_init → base_link) */
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

/* 重定位结果解析 (base_init_map → base_init) */
// 字段与 v2.6 的「雷达位姿」一模一样（x,y,z + 四元数 + 两个帧名），照旧解析不会报错——
// 分辨新旧只能靠帧名。本层只翻字段：这条边不参与任何坐标变换（工单 04），
// 只喂给图形层做「开机点在图里哪儿」的显示与「这条边到了没」的判据。
void ProtocolRos3D::parseRelocalizationPose(const QJsonObject& obj)
{
    RelocalizationPoseMsg m;
    m.frame_id       = obj["frame_id"].toString();
    m.child_frame_id = obj["child_frame_id"].toString();
    m.x              = obj["x"].toDouble();
    m.y              = obj["y"].toDouble();
    m.z              = obj["z"].toDouble();
    m.qx             = obj["qx"].toDouble();
    m.qy             = obj["qy"].toDouble();
    m.qz             = obj["qz"].toDouble();
    m.qw             = obj["qw"].toDouble();

    QString msg = QStringLiteral("接收到重定位结果: x=%1, y=%2, z=%3")
                      .arg(m.x, 0, 'f', 2).arg(m.y, 0, 'f', 2).arg(m.z, 0, 'f', 2);
    AdminMode::appendMessage(msg);

    emit relocalizationPoseUpdated(m);
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

/* 位置控制指令解析 (0x08) */
// 100 Hz 逐帧期望位置：值每帧都在动，日志按每秒最多一条节流，否则会淹掉接收窗口
void ProtocolRos3D::parsePosCmd(const QJsonObject& obj)
{
    PosCmdMsg m;
    m.topic = obj["topic"].toString();
    m.x     = obj["x"].toDouble();
    m.y     = obj["y"].toDouble();
    m.z     = obj["z"].toDouble();
    m.yaw   = obj["yaw"].toDouble();

    qint64 nowMS = QDateTime::currentMSecsSinceEpoch();
    if (nowMS - g_lastPosCmdLogMS >= 1000) {
        g_lastPosCmdLogMS = nowMS;
        AdminMode::appendMessage(QStringLiteral("接收到位置控制指令: x=%1, y=%2, z=%3, yaw=%4")
                                     .arg(m.x, 0, 'f', 2)
                                     .arg(m.y, 0, 'f', 2)
                                     .arg(m.z, 0, 'f', 2)
                                     .arg(m.yaw, 0, 'f', 2));
    }

    emit posCmdUpdated(m);
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
    // 不读 obj["frame_id"]：0x05 的载荷里**没有**帧名字段，读了只会得到一个空串。
    // 它的系 base_link 是硬约定、**不可校验**（见 tftree.cpp 的对照表与工单 05）。
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
    // 同 0x05：载荷里没有帧名字段，base_init_map 是硬约定、不可校验（工单 05）。
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


/* 状态帧解析 (0x0B)：飞控 + 重定位 + 路径规划 */
void ProtocolRos3D::parseNavStatus(const QJsonObject& obj)
{
    NavStatusMsg m;
    m.topic = obj["topic"].toString();
    m.fcState = obj["fc_state"].toInt();
    m.relocalization = obj["relocalization"].toInt();
    m.planning = obj["planning"].toInt();
    // 线上另有恒等于 `planning` 的 `state` 键，是给只认单值键的旧上位机看的；
    // 这里刻意不解析它——`NavStatusMsg::state()` 直接返回 `planning`。

    // 三件都是「当前值」不是「事件」：模块 10 Hz 重发同一个值，
    // 这里不写日志（日志只在值跳变时由展示端写一次）。
    ++m_statusCnt;
    emit navStatusUpdated(m);
}

/* 地图下传分块解析 (0x0C) */
// 一包一块，逐块到达；本层只翻字段，不攒块、不校验（见 maparchive.h）。
// 刻意不写日志：一次下载几十块、每块几十 KB，逐块记会把接收窗口刷掉；
// 进度由下载流程的进度框呈现（工单 10）。
void ProtocolRos3D::parseMapChunk(const QJsonObject& obj)
{
    MapChunkMsg m;
    m.topic = obj["topic"].toString();          // 恒为 "map_archive"
    m.name  = obj["name"].toString();           // map_3d.zip / map_2d.zip
    m.seq   = obj["seq"].toInt();               // 从 1 起
    m.total = obj["total"].toInt();
    m.bytes = obj["bytes"].toInt();             // 整包字节数
    m.crc32 = obj["crc32"].toString();          // 整包 CRC-32，形如 "0x1234abcd"
    // 本块字节：把线上 base64 解开。载荷是 base64 这点也保证了帧里不会出现 0x0A，
    // 所以「扫到第一个 0x0A 定帧长」的分帧走法在分块帧上依然成立。
    m.data = QByteArray::fromBase64(obj["data"].toString().toLatin1());

    emit mapChunkUpdated(m);
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
    quint32 statusFrames = m_statusCnt - m_statusLast;

    // 计算频率
    double poseHz  = poseFrames  / WIN;
    double cloudHz = cloudFrames / WIN;
    double mapHz   = mapFrames   / WIN;
    double pathHz  = pathFrames  / WIN;
    double statusHz = statusFrames / WIN;

    // 更新上周期计数
    m_poseLast   = m_poseCnt;
    m_cloudLast  = m_cloudCnt;
    m_mapLast    = m_mapCnt;
    m_pathLast   = m_pathCnt;
    m_statusLast = m_statusCnt;

    QString ts = QDateTime::currentDateTime().toString("HH:mm:ss");
    emit appendMessage(QStringLiteral("%1 >> ====== TCP话题接收频率统计 ======").arg(ts));
    emit appendMessage(QStringLiteral("%1 >> 位姿：%2Hz").arg(ts).arg(poseHz, 0, 'f', 2));
    emit appendMessage(QStringLiteral("%1 >> 点云：%2Hz").arg(ts).arg(cloudHz, 0, 'f', 2));
    emit appendMessage(QStringLiteral("%1 >> 地图：%2Hz").arg(ts).arg(mapHz, 0, 'f', 2));
    emit appendMessage(QStringLiteral("%1 >> 规划轨迹：%2Hz").arg(ts).arg(pathHz, 0, 'f', 2));
    emit appendMessage(QStringLiteral("%1 >> 状态：%2Hz").arg(ts).arg(statusHz, 0, 'f', 2));
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
