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

signals:
    void tfUpdated(const TFMsg& msg);
    void cloudUpdated(const CloudMsg& msg);
    void mapCloudUpdated(const MapCloudMsg& m);

private:

    void tryParseFrame();
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
};

#endif // PROTOCOLROS3D_H
