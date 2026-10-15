#ifndef PROTOCOLROS3D_H
#define PROTOCOLROS3D_H
#include <QObject>
#include <QByteArray>
#include <QJsonObject>
#include "protocol_msg.h"
#include <adminmode.h>

class ProtocolRos3D : public QObject {
    Q_OBJECT
public:
    explicit ProtocolRos3D(QObject* parent = nullptr);

    // 【新增】构建目标点下发帧: AA 07 [JSON] 0A
    static QByteArray buildNavGoalFrame(double x, double y, double z, double yaw);

public slots:
    void onRawBytes(quint8 cmd,const QByteArray& data);
    void setLinkAlive(bool alive) { m_linkAlive = alive; }

signals:
    void robotPoseUpdated(const RobotPoseMsg& msg);  // 机器人位姿 (base_init → base_link)
    // 重定位结果 (base_init_map → base_init)：静态边，**不参与任何坐标变换**，只用于显示开机点。
    void relocalizationPoseUpdated(const RelocalizationPoseMsg& msg);
    void cloudUpdated(const CloudMsg& msg);
    void mapCloudUpdated(const MapCloudMsg& m);

    // 2D数据信号
    void scan2DUpdated(const Scan2DMsg& msg);        // 2D激光雷达数据 (0x05)
    void map2DUpdated(const Map2DMsg& msg);          // 2D地图数据 (0x06)

    // 添加消息显示信号
    void appendMessage(const QString &message);

    void goalPathUpdated(const PathMsg& msg);

    // 位置控制指令 (0x08)
    void posCmdUpdated(const PosCmdMsg& msg);

    // 状态帧 (0x0B)：飞控 + 重定位 + 路径规划
    void navStatusUpdated(const NavStatusMsg& msg);

    // 地图下传分块 (0x0C)：一包一块，逐块到达。
    // 拼装与校验不在这里做——本层只把线上字节翻成字段，攒块与校验归
    // MapArchive（utils/protocol/maparchive.h）。下载流程（工单 10）接这个信号攒块。
    void mapChunkUpdated(const MapChunkMsg& msg);

    // 帧名不符 ⇒ 该帧**已被丢弃**（不解析、不渲染，见 parseJsonFrame 顶部那段说明）。
    // 本信号只做一件事：让界面留痕（把状态位变红），**不弹窗、不阻塞**。
    // 参数是**载荷里读到的**那两个名字，原样带出来给界面显示用——界面因此不必
    // （也不许）知道期望值是什么：期望值只在 tftree.cpp 那张对照表里有一份，
    // 界面上写出「应该是某某」就等于把它抄成了第二份。
    // frameNo 是协议帧号（`AA NN …` 的 NN），不是串口那套 0x01/0x04/0x05 的编号。
    void frameNameRejected(quint8 frameNo, const QString& frameId, const QString& childFrameId);

private:
    void parseJsonFrame(uint8_t cmd, const QJsonObject& obj);

    void parseRobotPose(const QJsonObject& obj);  // 解析机器人位姿 (0x01)
    void parseRelocalizationPose(const QJsonObject& obj);  // 解析重定位结果 (0x02)
    void parseCloud(const QJsonObject& obj);      // 解析3D点云 (0x03)
    void parseMap(const QJsonObject& obj);
    void parseGoalPath(const QJsonObject& obj);   // 解析最优轨迹 (0x09)
    void parsePosCmd(const QJsonObject& obj);     // 解析位置控制指令 (0x08)
    void parseScan2D(const QJsonObject& obj);     // 解析2D激光雷达 (0x05)
    void parseMap2D(const QJsonObject& obj);      // 解析2D地图 (0x06)
    void parseNavStatus(const QJsonObject& obj);  // 解析状态帧 (0x0B)
    void parseMapChunk(const QJsonObject& obj);   // 解析地图下传分块 (0x0C)

    /* 工具 */
    static quint8 crc8(const QByteArray& data);
    static QVector<QVector3D> extractXYZFromPointCloud2Raw(
        const QByteArray& raw,
        quint32 width, quint32 height, quint32 point_step, quint32 row_step,
        bool is_dense);

    static QVector<int8_t> decompressRLE(const QVector<int8_t>& rle);
    // 计数器
    quint32 m_poseCnt   = 0;  // 位姿话题 (0x01)
    quint32 m_cloudCnt  = 0;  // 点云话题 (0x03 + 0x05)
    quint32 m_mapCnt    = 0;  // 地图话题 (0x06)
    quint32 m_pathCnt   = 0;  // 轨迹话题 (0x09)
    quint32 m_statusCnt = 0;  // 状态帧 (0x0B)

    // 上一周期计数
    quint32 m_poseLast  = 0;
    quint32 m_cloudLast = 0;
    quint32 m_mapLast   = 0;
    quint32 m_pathLast  = 0;
    quint32 m_statusLast = 0;

    QTimer *m_hzTimer = nullptr;
    bool m_linkAlive = false;

private slots:
    void calcHz();          // 1 s 定时算频率

};

#endif // PROTOCOLROS3D_H
