#include "protocolrouter.h"
#include <QDebug>
#include <QDateTime>
#include <QtEndian>

ProtocolRouter* ProtocolRouter::m_instance = nullptr;

ProtocolRouter::ProtocolRouter(QObject *parent)
    : QObject(parent)
{
    // 初始化默认处理器
    initDefaultHandlers();
}

ProtocolRouter::~ProtocolRouter()
{
    handlers.clear();
}

ProtocolRouter* ProtocolRouter::instance()
{
    if (!m_instance) {
        m_instance = new ProtocolRouter();
    }
    return m_instance;
}

void ProtocolRouter::initDefaultHandlers()
{
    // 注册默认的内部处理器（如果不使用信号槽方式）
    registerHandler(0x00, [this](const QByteArray &frame) {
        handleControlFrame(frame);
    });

    registerHandler(0x10, [this](const QByteArray &frame) {
        handleParameterFrame(frame);
    });

    registerHandler(0x01, [this](const QByteArray &frame) {
        handleRos3dData(0x01, frame);
    });

    registerHandler(0x02, [this](const QByteArray &frame) {
        handleRos3dData(0x02, frame);
    });

    registerHandler(0x03, [this](const QByteArray &frame) {
        handleRos3dData(0x03, frame);
    });
}

void ProtocolRouter::registerHandler(quint8 commandType, std::function<void(const QByteArray&)> handler)
{
    handlers[commandType] = handler;
}

void ProtocolRouter::processDataStream(QByteArray buffer, bool isSerialPortMode)
{
    if (isSerialPortMode) {
        // 串口模式：处理10字节定长UART帧
        processUartFrames(buffer);
    } else {
        // WiFi模式：处理协议帧
        processProtocolFrames(buffer);
    }
}

void ProtocolRouter::processUartFrames(QByteArray &buffer)
{
    while (buffer.size() >= 10) {
        int head = buffer.indexOf(char(0xAA));
        if (head < 0 || buffer.size() - head < 10) break;

        QByteArray frame = buffer.mid(head, 10);

        // 验证帧格式
        if (quint8(frame[0]) != 0xAA || quint8(frame[9]) != 0x0A) {
            buffer.remove(head, 1);
            continue;
        }

        buffer.remove(head, 10);
        processUartFrame(frame);
    }
}

void ProtocolRouter::processUartFrame(const QByteArray &frame)
{
    if (frame.size() != 10) return;

    // 解析10字节UART帧：AA x y z yaw 0A
    qint16 x = qFromBigEndian<qint16>(
        reinterpret_cast<const uchar*>(frame.constData() + 1));
    qint16 y = qFromBigEndian<qint16>(
        reinterpret_cast<const uchar*>(frame.constData() + 3));
    qint16 z = qFromBigEndian<qint16>(
        reinterpret_cast<const uchar*>(frame.constData() + 5));
    qint16 yaw = qFromBigEndian<qint16>(
        reinterpret_cast<const uchar*>(frame.constData() + 7));

    emit uartFrameReceived(x, y, z, yaw);
}

// 直接替换掉原来的 processProtocolFrames 实现
void ProtocolRouter::processProtocolFrames(QByteArray &buffer)
{
    m_parseBuf.append(buffer);          // 1. 把新字节喂进来
    buffer.clear();                     // 2. 外部 buffer 清掉，避免重复处理

    while (true) {
        switch (m_parseState) {
        case ParseState::WaitHead:
        {
            int pos = m_parseBuf.indexOf(char(0xAA));
            if (pos < 0) {              // 一整包都没找到头
                m_parseBuf.clear();     // 全扔掉
                return;
            }
            m_parseBuf.remove(0, pos);  // 把 0xAA 前面的废数据清掉
            m_parseState = ParseState::WaitPayload;
            break;
        }

        case ParseState::WaitPayload:
        {
            int tail = m_parseBuf.indexOf(char(0x0A), 1); // 从第2字节找尾
            if (tail < 0) return;        // 还没收全，继续等

            QByteArray frame = m_parseBuf.left(tail + 1); // 包含头尾的完整帧
            m_parseBuf.remove(0, tail + 1);              // 把这帧清掉
            dispatchFrame(frame);        // 立刻分发（老逻辑不变）
            m_parseState = ParseState::WaitHead;
            break;
        }
        }
    }
}

QList<QByteArray> ProtocolRouter::extractProtocolFrames(QByteArray &buffer)
{
    QList<QByteArray> frames;

    while (!buffer.isEmpty()) {
        // 查找帧头 0xAA
        int headIndex = buffer.indexOf(char(0xAA));
        if (headIndex == -1) {
            buffer.clear();
            break;
        }

        if (headIndex > 0) {
            // 丢弃帧头前的无效数据
            buffer.remove(0, headIndex);
        }

        // 查找帧尾 0x0A
        int tailIndex = buffer.indexOf(char(0x0A), 1); // 从第二个字节开始找
        if (tailIndex == -1) {
            // 没有找到完整帧，保留数据等待下次接收
            break;
        }

        // 提取完整帧（包含头尾）
        int frameLength = tailIndex + 1;
        QByteArray frame = buffer.left(frameLength);
        buffer.remove(0, frameLength);

        frames.append(frame);
    }

    return frames;
}

void ProtocolRouter::dispatchFrame(const QByteArray &frame)
{
    if (frame.size() < 3) { // 最小帧：AA CMD 0A
        qWarning() << "帧太短：" << frame.size() << "字节";
        return;
    }

    // 验证帧头帧尾
    if (quint8(frame[0]) != 0xAA || quint8(frame[frame.size()-1]) != 0x0A) {
        qWarning() << "帧格式错误：" << frame.toHex();
        return;
    }

    quint8 command = static_cast<quint8>(frame.at(1));

    // 先尝试调用注册的处理器
    if (handlers.contains(command)) {
        handlers[command](frame);
    } else {
        // 使用信号分发
        dispatchFrameBySignal(frame, command);
    }
}

void ProtocolRouter::dispatchFrameBySignal(const QByteArray &frame, quint8 command)
{
    switch (command) {
    case 0x00: // 控制指令
        handleControlFrame(frame);
        break;

    case 0x10: // 参数配置
        handleParameterFrame(frame);
        break;

    case 0x01: // TF数据
    case 0x02: // 点云数据
    case 0x03: // 地图数据
        handleRos3dData(command, frame);
        break;

    default:
        qWarning() << "未知命令：" << QString::number(command, 16)
                   << "帧数据：" << frame.toHex();
        break;
    }
}

void ProtocolRouter::handleControlFrame(const QByteArray &frame)
{
    if (frame.size() < 4) return;

    quint8 subCmd = static_cast<quint8>(frame.at(2));

    switch (subCmd) {
    case 0x01: // 测试指令
        if (frame.size() == 4) {
            // AA 00 01 0A - 完整的测试帧
            bool isResponse = true; // 假设这是响应
            emit testFrameReceived(frame, isResponse);
        }
        break;

    case 0x02: // 保存地图指令
        qDebug() << "保存地图指令";
        emit saveMapCommandReceived();
        break;

    case 0x03: // 读取参数指令
        qDebug() << "读取参数指令";
        emit readParamCommandReceived();
        break;

    default:
        qWarning() << "未知控制子命令：" << subCmd;
        break;
    }
}

void ProtocolRouter::handleParameterFrame(const QByteArray &frame)
{
    if (frame.size() != 6) { // AA 10 ID VALUE_H VALUE_L 0A
        qWarning() << "参数帧长度错误：" << frame.size() << "应为6";
        return;
    }

    quint8 paramId = static_cast<quint8>(frame.at(2));
    qint16 value = (static_cast<quint8>(frame.at(3)) << 8) |
                   static_cast<quint8>(frame.at(4));

    emit parameterFrameReceived(paramId, value);
}

// 处理ROS数据
void ProtocolRouter::handleRos3dData(quint8 cmd,const QByteArray &frame)
{
    if (frame.size() <= 3) return;

    // 提取JSON数据（去掉AA 01和0A）
    QByteArray jsonData = frame.mid(2, frame.size() - 3);
    // qDebug() << "解析ROS数据";
    emit ros3dDataReceived(cmd, jsonData);
}

QByteArray ProtocolRouter::buildFrame(quint8 command, const QVariantMap &params)
{
    QByteArray frame;
    frame.append(static_cast<char>(0xAA)); // 帧头

    switch (command) {
    case 0x00: { // 控制指令集
        frame.append(static_cast<char>(0x00)); // 主命令

        quint8 subCmd = params.value("sub_cmd", 0x01).toUInt();
        frame.append(static_cast<char>(subCmd)); // 子命令

        // 如果是参数读取请求，可以添加更多数据
        if (subCmd == 0x03 && params.contains("param_id")) {
            quint8 paramId = params.value("param_id").toUInt();
            frame.append(static_cast<char>(paramId));
        }

        break;
    }

    case 0x10: { // 参数写入
        frame.append(static_cast<char>(0x10)); // 主命令

        if (!params.contains("param_id") || !params.contains("value")) {
            qWarning() << "构建参数帧缺少必要参数";
            return QByteArray();
        }

        quint8 paramId = params.value("param_id").toUInt();
        qint16 value = params.value("value").toInt();

        // 限制值范围
        if (value > 32767) value = 32767;
        if (value < -32768) value = -32768;

        frame.append(static_cast<char>(paramId));
        frame.append(static_cast<char>((value >> 8) & 0xFF)); // 高字节
        frame.append(static_cast<char>(value & 0xFF));        // 低字节

        break;
    }
    default:
        qWarning() << "无法构建未知命令的帧：" << command;
        return QByteArray();
    }

    frame.append(static_cast<char>(0x0A)); // 帧尾

    qDebug() << "构建帧：" << frame.toHex(' ')
             << "长度：" << frame.size() << "字节";

    return frame;
}

QByteArray ProtocolRouter::buildTestFrame(bool isResponse)
{
    QVariantMap params;
    params["sub_cmd"] = 0x01;
    params["is_request"] = !isResponse;

    return buildFrame(0x00, params);
}

QByteArray ProtocolRouter::buildSaveMapFrame()
{
    QVariantMap params;
    params["sub_cmd"] = 0x02;

    return buildFrame(0x00, params);
}

QByteArray ProtocolRouter::buildReadParamFrame(quint8 paramId)
{
    QVariantMap params;
    params["sub_cmd"] = 0x03;
    if (paramId != 0xFF) { // 0xFF表示读取所有参数
        params["param_id"] = paramId;
    }

    return buildFrame(0x00, params);
}

QByteArray ProtocolRouter::buildWriteParamFrame(quint8 paramId, qint16 value)
{
    QVariantMap params;
    params["param_id"] = paramId;
    params["value"] = value;

    return buildFrame(0x10, params);
}

QByteArray ProtocolRouter::buildRosFrame(quint8 topicId, const QJsonObject &data)
{
    QVariantMap params;
    params["data_object"] = QJsonValue(data);

    return buildFrame(topicId, params);
}

bool ProtocolRouter::validateFrame(const QByteArray &frame)
{
    if (frame.size() < 3) return false;

    quint8 header = static_cast<quint8>(frame.at(0));
    quint8 tail = static_cast<quint8>(frame.at(frame.size() - 1));

    return (header == 0xAA && tail == 0x0A);
}

quint8 ProtocolRouter::getFrameCommand(const QByteArray &frame)
{
    if (frame.size() < 2) return 0xFF;
    return static_cast<quint8>(frame.at(1));
}

QByteArray ProtocolRouter::extractFramePayload(const QByteArray &frame)
{
    if (frame.size() < 3) return QByteArray();
    return frame.mid(2, frame.size() - 3);
}

void ProtocolRouter::clearBuffer(QByteArray &buffer)
{
    buffer.clear();
}

QString ProtocolRouter::frameToHexString(const QByteArray &frame)
{
    return frame.toHex(' ').toUpper();
}
