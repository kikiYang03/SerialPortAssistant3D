#ifndef PROTOCOLROS3D_H
#define PROTOCOLROS3D_H
#include <QObject>
#include <QByteArray>
#include <QJsonObject>
#include "protocol_msg.h"

class ProtocolRos3D : public QObject {
    Q_OBJECT
public:
    explicit ProtocolRos3D(QObject* parent = nullptr);

public slots:
    void onRawBytes(quint8 cmd,const QByteArray& data);
    void setLinkAlive(bool alive) { m_linkAlive = alive; }

signals:
    void tfUpdated(const TFMsg& msg);
    void cloudUpdated(const CloudMsg& msg);
    void mapCloudUpdated(const MapCloudMsg& m);

    // 添加消息显示信号
    void appendMessage(const QString &message);

private:
    void parseJsonFrame(uint8_t cmd, const QJsonObject& obj);

    void parseTF(const QJsonObject& obj);
    void parseCloud(const QJsonObject& obj);   // cloud_registered / Laser_map 共用
    void parseMap(const QJsonObject& obj);

    /* 工具 */
    static quint8 crc8(const QByteArray& data);
    static QVector<QVector3D> extractXYZFromPointCloud2Raw(
        const QByteArray& raw,
        quint32 width, quint32 height, quint32 point_step, quint32 row_step,
        bool is_dense);

    static QVector<int8_t> decompressRLE(const QVector<int8_t>& rle);
    // 计数器
    quint32 m_tfCnt   = 0;
    quint32 m_cloudCnt= 0;
    quint32 m_mapCnt  = 0;

    // 上一周期计数
    quint32 m_tfLast   = 0;
    quint32 m_cloudLast= 0;
    quint32 m_mapLast  = 0;

    QTimer *m_hzTimer = nullptr;
    bool m_linkAlive = false;

private slots:
    void calcHz();          // 1 s 定时算频率

};

#endif // PROTOCOLROS3D_H
