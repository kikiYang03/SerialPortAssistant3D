#ifndef TCPCLIENT_H
#define TCPCLIENT_H

#include <QObject>
#include <QTcpSocket>
#include <QTimer>
#include <QMutex>
#include <QNetworkProxy>

class TcpClient : public QObject
{
    Q_OBJECT

public:
    static TcpClient* getInstance();

    // 连接管理
    bool connectToHost(const QString &ip, quint16 port, int timeoutMs = 5000);
    void disconnectFromHost();
    bool isConnected() const;

    // 数据发送
    qint64 sendData(const QByteArray &data);

    // 连接信息
    QString getServerIp() const { return m_serverIp; }
    quint16 getServerPort() const { return m_serverPort; }

    // 设置/获取连接状态
    void setConnectionStatus(bool connected);
    bool getConnectionStatus() const { return m_isConnected; }

    void stopAutoReconnect();   // 完全停止重连

    // 自动重连开关（工单 03 / 04）：置假之后，断线不再武装重连定时器、也不发 reconnectTimeout()，
    // 只发一次 autoReconnectSuppressed() 让上层把这次断开解释清楚。默认启用 —— 只有「重启模块」
    // 这条路径会临时关掉它，一直关到用户自己点「打开连接」（工单 03），或手动关连接那一刻（工单 04）。
    // 与 stopAutoReconnect() 不是一回事：那个停的是「正在进行的一次重连」，不改变是否允许重连。
    void setAutoReconnectEnabled(bool enabled);
    bool isAutoReconnectEnabled() const { return m_autoReconnect; }

signals:
    // 连接状态信号
    void connected();
    void disconnected();
    void connectionError(const QString &errorString);

    // 数据接收信号
    void dataReceived(QByteArray &data);

    // 状态变化信号（供UI更新）
    void connectionStatusChanged(bool connected);
    // 重连超时，通知 UI 层
    void reconnectTimeout();

    // 这次断开被压制（自动重连已被禁用）：它不是无来由的掉线，而是一次有解释的、预期内的断开。
    // 每次禁用期间只发一次 —— Qt 可能经 errorOccurred 与 disconnected 两条路先后把同一次断开
    // 报进来，上层据此写日志 / 复位按钮，写两行就是重复。
    void autoReconnectSuppressed();

public slots:
    void onReconnect();

private slots:
    void onConnected();
    void onDisconnected();
    void onReadyRead();
    void onErrorOccurred(QAbstractSocket::SocketError error);
    void onConnectionTimeout();

private:
    explicit TcpClient(QObject *parent = nullptr);
    ~TcpClient();

    // 断线处理入口（onDisconnected / onErrorOccurred）在武装重连定时器之前先过这一关：
    // 返回真 = 自动重连已禁用，本次断开不重连也不发 reconnectTimeout()，调用方立即 return。
    bool reconnectSuppressed();

    static TcpClient* m_instance;
    static QMutex m_mutex;

    QTcpSocket* m_tcpSocket;
    QTimer* m_connectionTimer;
    QTimer* m_reconnectTimer;

    QString m_serverIp;
    quint16 m_serverPort;
    bool m_isConnected;
    bool m_autoReconnect;               // 见 setAutoReconnectEnabled()
    bool m_suppressedNotified;          // 本次压制是否已上报（见 reconnectSuppressed()）

    // // 记录本次“断开”后累计重连耗时
    // int m_reconnectElapsedMs;
    int m_reconnectAttempts;  // 重连尝试次数
    bool m_reconnecting ;   // 正在重连过程中
};

#endif // TCPCLIENT_H
