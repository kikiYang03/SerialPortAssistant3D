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

    // 【新增】构建目标点下发帧: AA 04 [JSON] 0A
    static QByteArray buildNavGoalFrame(double x, double y, double z, double yaw);

public slots:
    void onRawBytes(quint8 cmd,const QByteArray& data);
    void setLinkAlive(bool alive) { m_linkAlive = alive; }

signals:
    void robotPoseUpdated(const RobotPoseMsg& msg);  // 机器人位姿 (map → base_link)
    void lidarPoseUpdated(const LidarPoseMsg& msg);  // 雷达位姿 (map → laser)
    void cloudUpdated(const CloudMsg& msg);
    void mapCloudUpdated(const MapCloudMsg& m);

    // 2D数据信号
    void scan2DUpdated(const Scan2DMsg& msg);        // 2D激光雷达数据 (0x05)
    void map2DUpdated(const Map2DMsg& msg);          // 2D地图数据 (0x06)

    // 添加消息显示信号
    void appendMessage(const QString &message);

    void goalPathUpdated(const PathMsg& msg);

private:
    void parseJsonFrame(uint8_t cmd, const QJsonObject& obj);

    void parseRobotPose(const QJsonObject& obj);  // 解析机器人位姿 (0x01)
    void parseLidarPose(const QJsonObject& obj);  // 解析雷达位姿 (0x02)
    void parseCloud(const QJsonObject& obj);      // 解析3D点云 (0x03)
    void parseMap(const QJsonObject& obj);
    void parseGoalPath(const QJsonObject& obj);   // 解析最优轨迹 (0x09)
    void parseScan2D(const QJsonObject& obj);     // 解析2D激光雷达 (0x05)
    void parseMap2D(const QJsonObject& obj);      // 解析2D地图 (0x06)

    /* 工具 */
    static quint8 crc8(const QByteArray& data);
    static QVector<QVector3D> extractXYZFromPointCloud2Raw(
        const QByteArray& raw,
        quint32 width, quint32 height, quint32 point_step, quint32 row_step,
        bool is_dense);

    static QVector<int8_t> decompressRLE(const QVector<int8_t>& rle);
    // 计数器
    quint32 m_tfCnt    = 0;
    quint32 m_cloudCnt = 0;  // 统一统计3D点云和2D激光数据
    quint32 m_mapCnt   = 0;

    // 上一周期计数
    quint32 m_tfLast     = 0;
    quint32 m_cloudLast  = 0;
    quint32 m_mapLast    = 0;

    QTimer *m_hzTimer = nullptr;
    bool m_linkAlive = false;

private slots:
    void calcHz();          // 1 s 定时算频率

};

#endif // PROTOCOLROS3D_H
