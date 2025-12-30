#ifndef PROTOCOLDISPATCHER_H
#define PROTOCOLDISPATCHER_H

#include <QObject>
#include <QByteArray>
#include <QJsonObject>

enum {
    CMD_TF = 0x01,
    CMD_CLOUD_REGISTERED = 0x02,
    CMD_LASER_MAP = 0x03
};


class ProtocolDispatcher : public QObject {
    Q_OBJECT
public:
    explicit ProtocolDispatcher(QObject* parent = nullptr);

public slots:
    void onRawBytes(const QByteArray& data);

signals:
    void rosJsonReceived(quint8 command, const QJsonObject& obj);

private:
    QByteArray buffer_;
};
#endif // PROTOCOLDISPATCHER_H
