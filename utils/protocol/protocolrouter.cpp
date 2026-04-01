#include "protocolrouter.h"
#include <QDebug>
#include <QDateTime>
#include <QtEndian>

static int frameType(const QByteArray &buf)
{
    if (buf.size() < 3) return -1;
    if (quint8(buf[0]) != 0xAA) return -1;
    // 将 14 改为 15，索引 13 改为 14
    if (buf.size() >= 15 && quint8(buf[14]) == 0x0A) return 0;   // 串口
    if (quint8(buf.back()) == 0x0A)        return 1;             // 网络
    return -1;
}

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

    // 新协议命令ID
    registerHandler(0x01, [this](const QByteArray &frame) {  // 机器人位姿
        handleRos3dData(0x01, frame);
    });

    registerHandler(0x02, [this](const QByteArray &frame) {  // 雷达安装位姿
        handleRos3dData(0x02, frame);
    });

    registerHandler(0x03, [this](const QByteArray &frame) {  // 3D点云
        handleRos3dData(0x03, frame);
    });

    registerHandler(0x05, [this](const QByteArray &frame) {  // 2D激光雷达数据
        handleRos3dData(0x05, frame);
    });

    registerHandler(0x06, [this](const QByteArray &frame) {  // 2D地图数据
        handleRos3dData(0x06, frame);
    });

    registerHandler(0x09, [this](const QByteArray &frame) {  // 最优轨迹线
        handleRos3dData(0x09, frame);
    });
}

void ProtocolRouter::registerHandler(quint8 commandType, std::function<void(const QByteArray&)> handler)
{
    handlers[commandType] = handler;
}

void ProtocolRouter::processDataStream(QByteArray buffer, bool isSerialPortMode)
{
    if (isSerialPortMode) {
        // 将新数据追加到累积缓冲区
        m_uartBuffer.append(buffer);
        processUartFrames();
    } else {
        // WiFi模式：处理协议帧
        processProtocolFrames(buffer);
    }
}

void ProtocolRouter::processUartFrames()
{
    // 将所有的 14 改为 15，索引 13 改为 14
    while (m_uartBuffer.size() >= 15) {
        int head = m_uartBuffer.indexOf(char(0xAA));
        if (head < 0 || m_uartBuffer.size() - head < 15) {
            break; // 剩余数据不足 15 字节，等待
        }

        // 提取 15 字节候选帧
        QByteArray frame = m_uartBuffer.mid(head, 15);

        // 验证帧头帧尾
        if (quint8(frame[0]) != 0xAA || quint8(frame[14]) != 0x0A) {
            m_uartBuffer.remove(head, 1);
            continue;
        }

        // 解析成功
        processUart15BFrame(frame);
        m_uartBuffer.remove(head, 15);
    }

    if (m_uartBuffer.size() > 1024 && !m_uartBuffer.contains(0xAA)) {
        qWarning() << "串口缓冲区无有效帧头，清空:" << m_uartBuffer.size() << "字节";
        m_uartBuffer.clear();
    }
}

void ProtocolRouter::processProtocolFrames(QByteArray &buffer)
{
    m_parseBuf.append(buffer);          // 把新字节追加到缓冲区
    buffer.clear();                     // 外部 buffer 清掉，避免重复处理

    while (true) {
        // 1. 找帧头 0xAA
        int headPos = m_parseBuf.indexOf(char(0xAA));
        if (headPos < 0) {
            m_parseBuf.clear();         // 没有帧头，清空
            return;
        }
        if (headPos > 0) {
            m_parseBuf.remove(0, headPos);  // 清理帧头前的废数据
        }

        // 2. 检查是否有足够数据读取CMD（至少需要2字节：AA + CMD）
        if (m_parseBuf.size() < 2) {
            return;                     // 等待更多数据
        }

        quint8 cmd = static_cast<quint8>(m_parseBuf[1]);

        // 3. 根据CMD判断期望帧长度
        int expectedLen = -1;           // -1表示变长帧

        if (cmd == 0x00) {
            // 控制指令：固定4字节 AA 00 子命令 0A
            expectedLen = 4;
        } else if (cmd == 0x10) {
            // 参数配置：固定6字节 AA 10 [ID] [值H] [值L] 0A
            expectedLen = 6;
        } else {
            // ROS数据(0x01~0x09)：变长JSON帧，需要找帧尾
            // 帧格式：AA CMD [JSON数据] 0A
            for (int i = 2; i < m_parseBuf.size(); ++i) {
                if (quint8(m_parseBuf[i]) == 0x0A) {
                    expectedLen = i + 1;
                    break;
                }
            }
        }

        // 4. 检查是否已接收完整帧
        if (expectedLen < 0) {
            return;                     // 变长帧还没找到帧尾，等待更多数据
        }
        if (m_parseBuf.size() < expectedLen) {
            return;                     // 数据不足，等待更多数据
        }

        // 5. 提取完整帧并处理
        QByteArray frame = m_parseBuf.left(expectedLen);
        m_parseBuf.remove(0, expectedLen);
        dispatchFrame(frame);

        // 继续循环，处理可能存在的后续帧
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

    case 0x01: // 机器人位姿 (map → base_link)
    case 0x02: // 雷达安装位姿 (map → laser)
    case 0x03: // 3D点云数据
    case 0x05: // 2D激光雷达数据
    case 0x06: // 2D地图数据
    case 0x09: // 最优轨迹线
        handleRos3dData(command, frame);
        break;

    default:
        // qWarning() << "未知命令：" << QString::number(command, 16)
        //            << "帧数据：" << frame.toHex();
        break;
    }
}

void ProtocolRouter::handleControlFrame(const QByteArray &frame)
{
    // 控制帧格式：AA 00 子命令 0A（固定4字节）
    if (frame.size() != 4) return;

    quint8 subCmd = static_cast<quint8>(frame.at(2));

    switch (subCmd) {
    case 0x01: // 通信测试指令
        emit testFrameReceived(frame, true);
        break;
    case 0x02: // 保存地图指令
        emit saveMapCommandReceived();
        break;
    case 0x03: // 读取参数指令
        emit readParamCommandReceived();
        break;
    case 0x04: // 保存参数指令
        qDebug() << "收到保存参数指令响应";
        break;
    default:
        qWarning() << "未知控制子命令：" << subCmd;
        break;
    }
}

// 参数配置
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

// 构建指令
QByteArray ProtocolRouter::buildFrame(quint8 command, const QVariantMap &params)
{
    QByteArray frame;
    frame.append(static_cast<char>(0xAA)); // 帧头

    switch (command) {
    case 0x00: { // 控制指令集
        frame.append(static_cast<char>(0x00)); // 主命令

        quint8 subCmd = params.value("sub_cmd", 0x01).toUInt();
        frame.append(static_cast<char>(subCmd)); // 子命令

        break;
    }
    case 0x10: { // 参数写入
        frame.append(static_cast<char>(0x10)); // 主命令

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

// 新增解析串口数据函数
// void ProtocolRouter::processUart14BFrame(const QByteArray &fr)
// {
//     if (fr.size() != 14 || quint8(fr[0]) != 0xAA || quint8(fr[13]) != 0x0A)
//         return ;

//     auto i16 = [&](int off){ return qFromBigEndian<qint16>(
//                                   reinterpret_cast<const uchar*>(fr.constData()+off)); };
//     qint16 x    = i16(1);
//     qint16 y    = i16(3);
//     qint16 z    = i16(5);
//     qint16 roll = i16(7);
//     qint16 pitch= i16(9);
//     qint16 yaw  = i16(11);
//     emit uart14BFrameReceived(x,y,z,roll,pitch,yaw);
// }


void ProtocolRouter::processUart15BFrame(const QByteArray &fr)
{
    if (fr.size() != 15 || quint8(fr[0]) != 0xAA || quint8(fr[14]) != 0x0A)
        return ;

    quint8 msgId = quint8(fr[1]); // 提取 MsgID

    // 偏移量统一 +1 (因为中间插入了 MsgID)
    auto i16 = [&](int off){ return qFromBigEndian<qint16>(
                                  reinterpret_cast<const uchar*>(fr.constData()+off)); };

    // 解析改为小端
    // auto i16 = [&](int off){ return qFromLittleEndian<qint16>(
    //                               reinterpret_cast<const uchar*>(fr.constData()+off)); };

    qint16 x    = i16(2);
    qint16 y    = i16(4);
    qint16 z    = i16(6);
    qint16 roll = i16(8);
    qint16 pitch= i16(10);
    qint16 yaw  = i16(12);

    // 0x01 (当前位姿) 和 0x03 (位置控制指令) 是从模块发给上位机的
    // 0x02 (导航目标点) 上位机发送后，模块会完整返回
    if (msgId == 0x01 || msgId == 0x03) {
        emit uartPoseReceived(msgId, x, y, z, roll, pitch, yaw);
    } else if (msgId == 0x02) {
        // 0x02目标点回传，发送完整原始帧
        emit navGoalEchoReceived(fr);
    } else {
        qWarning() << "收到预期外的串口 MsgID:" << msgId;
    }
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

QByteArray ProtocolRouter::buildNavGoalFrame(qint16 x, qint16 y, qint16 z,
                                             qint16 roll, qint16 pitch, qint16 yaw)
{
    QByteArray frame;
    frame.resize(15);
    frame[0] = static_cast<char>(0xAA);
    frame[1] = static_cast<char>(0x02); // 消息ID: 0x02 (导航目标点)

    // // 使用 qToBigEndian 快速写入大端数据
    qToBigEndian<qint16>(x, frame.data() + 2);
    qToBigEndian<qint16>(y, frame.data() + 4);
    qToBigEndian<qint16>(z, frame.data() + 6);
    qToBigEndian<qint16>(roll, frame.data() + 8);
    qToBigEndian<qint16>(pitch, frame.data() + 10);
    qToBigEndian<qint16>(yaw, frame.data() + 12);

    // 小端序
    // qToLittleEndian<qint16>(x, frame.data() + 2);
    // qToLittleEndian<qint16>(y, frame.data() + 4);
    // qToLittleEndian<qint16>(z, frame.data() + 6);
    // qToLittleEndian<qint16>(roll, frame.data() + 8);
    // qToLittleEndian<qint16>(pitch, frame.data() + 10);
    // qToLittleEndian<qint16>(yaw, frame.data() + 12);


    frame[14] = static_cast<char>(0x0A);

    qDebug() << "构建 0x02 导航帧：" << frame.toHex(' ').toUpper();
    return frame;
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
