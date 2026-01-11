#include "protocolhandler.h"

ProtocolHandler::ProtocolHandler(QObject *parent) : QObject(parent)
{
}

// 构建通信测试帧
QByteArray ProtocolHandler::buildTestFrame()
{
    QByteArray frame;
    frame.append(static_cast<char>(FRAME_HEADER));
    frame.append(static_cast<char>(ProtocolCommand::TEST));
    frame.append(static_cast<char>(0x01)); // 固定数据
    frame.append(static_cast<char>(FRAME_TAIL));
    return frame;
}

// 构建参数配置帧
// 读取参数-AA 00 03 0A
QByteArray ProtocolHandler::buildCurParameterFrame()
{
    QByteArray frame;
    frame.append(static_cast<char>(FRAME_HEADER));
    frame.append(static_cast<char>(ProtocolCommand::TEST));
    frame.append(static_cast<char>(0x03)); // 固定数据
    frame.append(static_cast<char>(FRAME_TAIL));
    return frame;
}
// 写入参数
QByteArray ProtocolHandler::buildParameterFrame(quint8 command, quint8 paramId, qint16  value)
{
    QByteArray frame;
    frame.append(static_cast<char>(FRAME_HEADER));
    frame.append(static_cast<char>(command));
    frame.append(static_cast<char>(paramId));

    if (command == ProtocolCommand::PARAM_WRITE) {
        // 保证值在 16-bit 有符号范围内
        value = qBound<int>(-32768, value, 32767);
        frame.append(static_cast<char>((value >> 8) & 0xFF)); // 高
        frame.append(static_cast<char>(value & 0xFF));        // 低
    }
    frame.append(static_cast<char>(FRAME_TAIL));
    return frame;
}

// 构建ROS数据帧
QByteArray ProtocolHandler::buildRosFrame(quint8 topicId, const QJsonObject &data)
{
    QByteArray frame;
    QJsonDocument doc(data);
    QByteArray jsonData = doc.toJson(QJsonDocument::Compact);

    frame.append(static_cast<char>(FRAME_HEADER));
    frame.append(static_cast<char>(topicId));
    frame.append(jsonData);
    frame.append(static_cast<char>(FRAME_TAIL));

    return frame;
}

// 保存地图
QByteArray ProtocolHandler::buildSaveMapFrame()
{
    QByteArray frame;
    frame.append(static_cast<char>(FRAME_HEADER));
    frame.append(static_cast<char>(ProtocolCommand::TEST));
    frame.append(static_cast<char>(0x02)); // 固定数据
    frame.append(static_cast<char>(FRAME_TAIL));
    return frame;
}

// 验证帧格式
bool ProtocolHandler::validateFrame(const QByteArray &frame)
{
    if (frame.size() < 4) return false;

    quint8 header = static_cast<quint8>(frame.at(0));
    quint8 tail = static_cast<quint8>(frame.at(frame.size() - 1));

    return (header == FRAME_HEADER && tail == FRAME_TAIL);
}

// 获取命令类型
quint8 ProtocolHandler::getCommandType(const QByteArray &frame)
{
    if (frame.size() < 2) return 0xFF;
    return static_cast<quint8>(frame.at(1));
}

// 提取载荷数据
QByteArray ProtocolHandler::extractPayload(const QByteArray &frame)
{
    if (frame.size() < 3) return QByteArray();
    return frame.mid(2, frame.size() - 3);
}

// 解析测试响应
bool ProtocolHandler::parseTestResponse(const QByteArray &frame)
{
    if (!validateFrame(frame)) return false;

    quint8 command = getCommandType(frame);
    if (command != ProtocolCommand::TEST) return false;

    // 检查是否为正确的测试回复数据: AA 00 01 0A
    return (frame.size() == 4 &&
            static_cast<quint8>(frame.at(2)) == 0x01);
}

// 解析参数响应
bool ProtocolHandler::parseParameterResponse(const QByteArray &frame,
                                             quint8 &paramId,
                                             qint16 &value)   // 匹配头文件
{
    if (!validateFrame(frame) || frame.size() != 6) return false;
    if (getCommandType(frame) != ProtocolCommand::PARAM_READ) return false;

    paramId = static_cast<quint8>(frame.at(2));
    // 16-bit 有符号拼接
    value = static_cast<qint16>((static_cast<quint8>(frame.at(3)) << 8)
                                | static_cast<quint8>(frame.at(4)));
    return true;
}

// 解析ROS数据
QJsonObject ProtocolHandler::parseRosData(const QByteArray &payload)
{
    QJsonParseError error;
    QJsonDocument doc = QJsonDocument::fromJson(payload, &error);

    if (error.error != QJsonParseError::NoError) {
        qWarning() << "JSON解析错误:" << error.errorString();
        return QJsonObject();
    }

    if (!doc.isObject()) {
        qWarning() << "JSON数据不是对象格式";
        return QJsonObject();
    }

    return doc.object();
}

// 从缓冲区提取完整帧
QList<QByteArray> ProtocolHandler::extractFramesFromBuffer(QByteArray &buffer)
{
    QList<QByteArray> frames;

    while (!buffer.isEmpty()) {
        // 查找帧头
        int headIndex = buffer.indexOf(static_cast<char>(FRAME_HEADER));
        if (headIndex == -1) {
            buffer.clear();
            break;
        }

        // 丢弃帧头前的数据
        if (headIndex > 0) {
            buffer.remove(0, headIndex);
        }

        if (buffer.size() < 3) break; // 等待更多数据

        quint8 command = static_cast<quint8>(buffer.at(1));
        int frameLength = getFrameLength(command, buffer);

        if (frameLength == -1) {
            // 可变长度帧，查找帧尾
            int tailIndex = buffer.indexOf(static_cast<char>(FRAME_TAIL), 2);
            if (tailIndex == -1) break; // 等待更多数据
            frameLength = tailIndex + 1;
        }

        if (buffer.size() < frameLength) break; // 等待更多数据

        // 提取完整帧
        QByteArray frame = buffer.left(frameLength);
        buffer.remove(0, frameLength);

        if (validateFrame(frame)) {
            frames.append(frame);
        }
    }

    return frames;
}

// 获取帧长度
int ProtocolHandler::getFrameLength(quint8 command, const QByteArray &data)
{
    switch (command) {
    case ProtocolCommand::TEST:
        return 4; // AA 00 01 0A
    case ProtocolCommand::PARAM_READ:
        return 6; // AA 10 [参数ID] [值高字节] [值低字节] 0A
    default:
        return -1;
    }
}

