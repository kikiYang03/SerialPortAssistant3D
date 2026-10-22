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

    // 便捷方法 —— **以下产出的都是 TCP / UDP 的短帧形状**（4 / 6 字节）；
    // 串口要的 15 字节定长帧用下面的 buildUart* 两个。
    static QByteArray buildTestFrame(bool isResponse = false);
    QByteArray buildSaveMapFrame();
    static QByteArray buildReadParamFrame(quint8 paramId = 0xFF); // 0xFF=读取所有
    static QByteArray buildWriteParamFrame(quint8 paramId, qint16 value);

    // 串口线上的命令帧 / 参数帧（15 字节定长）///////////////////////////////
    // 串口与 TCP 是**两套帧**，但命令面的子命令号逐个相同（协议「命令帧 / 参数帧格式」一节）：
    //   TCP   AA 00 <sub> 0A                                 （4 字节）
    //   串口  AA 00 <sub> 00 ×11 0A                           （15 字节，载荷空、补零到 14）
    //   TCP   AA 10 <id> <vH> <vL> 0A                         （6 字节）
    //   串口  AA 10 <id> <vH> <vL> 00 ×9 0A                    （15 字节）
    // 两个形状的第 2 字节（id 位 / 子命令位）**位置相同**，所以解析侧可以共用一套
    // （handleControlFrame / handleParameterFrame 同时认两种长度），只有构建要分链路 —— 见下。
    static QByteArray buildUartCommandFrame(quint8 subCmd);
    static QByteArray buildUartParamFrame(quint8 paramId, qint16 value);

    // 串口 0x05 状态帧 Byte 5/6（CPU / 运存）的**未知哨兵**：upper 没起、或超 2 秒没收到
    // /resource_usage 采样时发的是它，不是百分比。本地拿它区分「0%」与「没人报」——
    // 与三件状态各占一格的 `3`（未响应）是同一条规矩：不能让上位机把「没人报」读成 0。
    static constexpr quint8 kUartPercentUnknown = 0xFF;
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

    // 0x04 规划速度帧信号（100Hz）：vx/vy/vz 单位 cm/s，yawDot 单位度/s。
    // 「规划速度」= pos_cmd 里下发的速度指令，**不是实测实时速度**，文案/日志别写成实时速度。
    void uartSpeedReceived(qint16 vx, qint16 vy, qint16 vz, qint16 yawDot);

    // 0x05 状态帧信号（1Hz）：**五件**状态，顺排在 Byte 2 ~ Byte 6。
    // fcState 是飞控枚举（0x00~0x04，超出按未知处理），relocalization / planning 取值
    // 同 TCP 的 AA 0B（0 进行中 / 1 成功 / 2 失败 / 3 未响应），两边同义同值。
    // cpuPercent / memPercent 是模块自己的资源占用（0~100），**0xFF = 未知**
    // （upper 没起 / 超 2 秒没采样），由接收方按 kUartPercentUnknown 判，不要直接除以 100。
    // Byte 7~13 是恒零预留区，不参与解析。
    void uartFlightStatusReceived(quint8 fcState, quint8 relocalization, quint8 planning,
                                  quint8 cpuPercent, quint8 memPercent);

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
