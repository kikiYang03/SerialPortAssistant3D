#ifndef PROTOCOLHANDLER_H
#define PROTOCOLHANDLER_H

#include <QObject>
#include <QByteArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QDebug>
#include <QtEndian>

// 协议命令定义
namespace ProtocolCommand {
    const quint8 TEST = 0x00;        // 通信测试
    const quint8 PARAM_READ = 0x10;  // 参数读取
    const quint8 PARAM_WRITE = 0x10; // 参数写入
    const quint8 COORD_UART = 0x20;   // 串口坐标帧
}

class ProtocolHandler : public QObject
{
    Q_OBJECT

public:
    explicit ProtocolHandler(QObject *parent = nullptr);

    // 帧构建方法
    static QByteArray buildTestFrame();                                   // 通信测试帧
    static QByteArray buildCurParameterFrame();                                   // 读取参数帧
    static QByteArray buildParameterFrame(quint8 command, quint8 paramId, qint16  value = 0); // 参数配置帧
    static QByteArray buildRosFrame(quint8 topicId, const QJsonObject &data); // ROS数据帧
    static QByteArray buildSaveMapFrame();        //保存地图帧

    // 帧解析方法
    static bool validateFrame(const QByteArray &frame);                  // 帧验证
    static quint8 getCommandType(const QByteArray &frame);               // 获取命令类型
    static QByteArray extractPayload(const QByteArray &frame);           // 提取载荷数据

    // 特定帧解析
    static bool parseTestResponse(const QByteArray &frame);              // 解析测试响应
    static bool parseParameterResponse(const QByteArray &frame, quint8 &paramId, qint16 &value); // 解析参数响应
    static QJsonObject parseRosData(const QByteArray &payload);          // 解析ROS数据

    // 数据流处理
    static QList<QByteArray> extractFramesFromBuffer(QByteArray &buffer); // 从缓冲区提取完整帧


private:
    static const quint8 FRAME_HEADER = 0xAA;
    static const quint8 FRAME_TAIL = 0x0A;

    static int getFrameLength(quint8 command, const QByteArray &data = QByteArray());
};

#endif // PROTOCOLHANDLER_H
