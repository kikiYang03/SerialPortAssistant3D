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
    QByteArray buildFrame(quint8 command,
                          const QVariantMap &params = QVariantMap());

    // 便捷方法
    QByteArray buildTestFrame(bool isResponse = false);
    QByteArray buildSaveMapFrame();
    QByteArray buildReadParamFrame(quint8 paramId = 0xFF); // 0xFF=读取所有
    QByteArray buildWriteParamFrame(quint8 paramId, qint16 value);
    QByteArray buildRosFrame(quint8 topicId, const QJsonObject &data);

    // 工具方法
    static bool validateFrame(const QByteArray &frame);
    static quint8 getFrameCommand(const QByteArray &frame);
    static QByteArray extractFramePayload(const QByteArray &frame);
    static void clearBuffer(QByteArray &buffer);
    static QString frameToHexString(const QByteArray &frame);

signals:
    // 不同类型数据的信号
    void testFrameReceived(const QByteArray &frame, bool isResponse);
    void saveMapCommandReceived();
    void readParamCommandReceived();

    void parameterFrameReceived(quint8 paramId, qint16 value);

    // ros3D数据
    void ros3dDataReceived(quint8 cmd,const QByteArray &jsonData);

    // void tfDataReceived(const QByteArray &jsonData);
    // void cloudDataReceived(const QByteArray &jsonData);
    // void mapDataReceived(const QByteArray &jsonData);

    void uartFrameReceived(qint16 x, qint16 y, qint16 z, qint16 yaw);

    // 统计信号
    // void frameCountUpdated(quint8 type, int count);

private:
    explicit ProtocolRouter(QObject *parent = nullptr);
    ~ProtocolRouter();

    void initDefaultHandlers();
    void processUartFrames(QByteArray &buffer);
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

    static ProtocolRouter* m_instance;
    QMap<quint8, std::function<void(const QByteArray&)>> handlers;


    // protocolrouter.h 里 private 区增加：
    enum class ParseState { WaitHead, WaitPayload };
    ParseState m_parseState = ParseState::WaitHead;
    QByteArray m_parseBuf;   // 状态机内部缓冲
};

#endif // PROTOCOLROUTER_H
