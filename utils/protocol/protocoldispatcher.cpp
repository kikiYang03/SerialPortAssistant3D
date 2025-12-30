#include "protocoldispatcher.h"
#include "protocolhandler.h"

ProtocolDispatcher::ProtocolDispatcher(QObject* parent)
    : QObject(parent)
{
}

void ProtocolDispatcher::onRawBytes(const QByteArray& data)
{
    buffer_.append(data);

    QList<QByteArray> frames =
        ProtocolHandler::extractFramesFromBuffer(buffer_);

    for (const QByteArray& frame : frames) {
        if (!ProtocolHandler::validateFrame(frame))
            continue;

        quint8 cmd = ProtocolHandler::getCommandType(frame);
        QByteArray payload = ProtocolHandler::extractPayload(frame);

        QJsonObject obj = ProtocolHandler::parseRosData(payload);
        if (!obj.isEmpty()) {
            emit rosJsonReceived(cmd, obj);
        }
    }
}
