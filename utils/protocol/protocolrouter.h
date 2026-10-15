#ifndef PROTOCOLROUTER_H
#define PROTOCOLROUTER_H
#include <QObject>
#include <QByteArray>
#include <QMap>
#include <QVariantMap>
#include <QJsonObject>
#include <functional>

class ProtocolRouter : public QObject
{
    Q_OBJECT

public:
    static ProtocolRouter* instance();

    // 注册处理器（可选，如果不使用信号槽）
    void registerHandler(quint8 commandType,
                         std::function<void(const QByteArray&)> handler);

    // 处理原始数据流
    void processDataStream(QByteArray buffer, bool isSerialPortMode);

    // 构建指令帧
    static QByteArray buildFrame(quint8 command,
                          const QVariantMap &params = QVariantMap());

    // 便捷方法
    static QByteArray buildTestFrame(bool isResponse = false);
    QByteArray buildSaveMapFrame();
    static QByteArray buildReadParamFrame(quint8 paramId = 0xFF); // 0xFF=读取所有
    static QByteArray buildWriteParamFrame(quint8 paramId, qint16 value);
    // 下传地图指令 AA 00 05 0A：请求模块把盘上那份地图打包分块发来。
    // 应答是 N 帧 0x0C（见 MapArchive）；模块失败时一帧都不发，上位机按超时判失败。
    QByteArray buildDownloadMapFrame();
    // 重启模块指令 AA 00 06 0A：模块**先原样回发本帧、再执行关机**，所以「收到回发」是
    // 上位机判成功的唯一依据（回显到达与连接断开是先后两件事，本功能不去推断断线时机）。
    QByteArray buildRestartFrame();
    QByteArray buildRosFrame(quint8 topicId, const QJsonObject &data);

    // 工具方法
    static bool validateFrame(const QByteArray &frame);
    static quint8 getFrameCommand(const QByteArray &frame);
    static QByteArray extractFramePayload(const QByteArray &frame);
    static void clearBuffer(QByteArray &buffer);
    static QString frameToHexString(const QByteArray &frame);

    // 构建 0x02 导航目标点帧 (上位机 -> 模块)
    static QByteArray buildNavGoalFrame(qint16 x, qint16 y, qint16 z,
                                        qint16 roll, qint16 pitch, qint16 yaw);

signals:
    // 不同类型数据的信号
    void testFrameReceived(const QByteArray &frame, bool isResponse);
    // 入站的 AA 00 02 0A = 模块存图成功且产物核对通过（只回成功，失败一个字节都不回）
    void saveMapAckReceived();

    // 入站的 AA 00 04 0A = 保存终结指令回显。模块把参数落盘、从盘上重读一次之后，
    // 先发完全表的 21 条 AA 10，再原样回发本帧——所以「收到它」= 回读已发完，可以下结论了。
    // 与存图不同：这里失败也是沉默的，只能按超时判（见工单 06）。
    void saveParamsAckReceived();

    // 入站的 AA 00 06 0A = 重启指令回显。模块先回发、再断电重启，所以「收到它」= 模块已受理
    // 重启。⚠️ 这里的 0x06 是**控制组子命令**；帧号位置上的那个 0x06（AA 06 …）是「2D 地图
    // 数据」，两者位置不同、含义不同，读代码时极易看错。
    void restartAckReceived();

    void readParamCommandReceived();

    void parameterFrameReceived(quint8 paramId, qint16 value);

    // ros3D数据
    void ros3dDataReceived(quint8 cmd,const QByteArray &jsonData);

    void uartFrameReceived(qint16 x, qint16 y, qint16 z, qint16 yaw);

    // 串口接收信息信号
    // void uart14BFrameReceived(qint16 x, qint16 y, qint16 z,
    //                           qint16 roll, qint16 pitch, qint16 yaw);

    // 串口接收信息信号 (带 msgId 分发)
    void uartPoseReceived(quint8 msgId, qint16 x, qint16 y, qint16 z,
                          qint16 roll, qint16 pitch, qint16 yaw);

    // 0x02目标点回传信号（下位机返回完整帧）
    void navGoalEchoReceived(const QByteArray &frame);

    // 0x04 实时速度帧信号（100Hz）：vx/vy/vz 单位 cm/s，yawDot 单位度/s
    void uartSpeedReceived(qint16 vx, qint16 vy, qint16 vz, qint16 yawDot);

    // 0x05 状态帧信号（1Hz）：三件状态各占一个字节，顺排在 Byte 2 / Byte 3 / Byte 4。
    // fcState 是飞控枚举（0x00~0x04，超出按未知处理），relocalization / planning 取值
    // 同 TCP 的 AA 0B（0 进行中 / 1 成功 / 2 失败 / 3 未响应），两边同义同值。
    // Byte 5~13 是恒零预留区，不参与解析。
    void uartFlightStatusReceived(quint8 fcState, quint8 relocalization, quint8 planning);

    // 打印原来数据
    void printFrame(const QByteArray &frame);

private:
    explicit ProtocolRouter(QObject *parent = nullptr);
    ~ProtocolRouter();

    void initDefaultHandlers();
    void processUartFrames();
    void processProtocolFrames(QByteArray &buffer);
    QList<QByteArray> extractProtocolFrames(QByteArray &buffer);
    void dispatchFrame(const QByteArray &frame);
    void dispatchFrameBySignal(const QByteArray &frame, quint8 command);

    // 内部处理方法
    void processUartFrame(const QByteArray &frame);
    void handleControlFrame(const QByteArray &frame);
    void handleParameterFrame(const QByteArray &frame);

    // 处理ros数据
    void handleRos3dData(quint8 cmd,const QByteArray &frame);
    // void handleTFData(const QByteArray &frame);
    // void handleCloudData(const QByteArray &frame);
    // void handleMapData(const QByteArray &frame);

    // 串口解析数据
    // void processUart14BFrame(const QByteArray &fr);
    void processUart15BFrame(const QByteArray &fr);

    static ProtocolRouter* m_instance;
    QMap<quint8, std::function<void(const QByteArray&)>> handlers;


    // protocolrouter.h 里 private 区增加：
    enum class ParseState { WaitHead, WaitPayload };
    ParseState m_parseState = ParseState::WaitHead;
    QByteArray m_parseBuf;   // 状态机内部缓冲
    QByteArray m_uartBuffer;    // 【新增】串口模式累积缓冲区

    static const quint8 FRAME_HEADER = 0xAA;
    static const quint8 FRAME_TAIL = 0x0A;
};

#endif // PROTOCOLROUTER_H
