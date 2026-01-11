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

    static TcpClient* m_instance;
    static QMutex m_mutex;

    QTcpSocket* m_tcpSocket;
    QTimer* m_connectionTimer;
    QTimer* m_reconnectTimer;

    QString m_serverIp;
    quint16 m_serverPort;
    bool m_isConnected;
    bool m_autoReconnect;

    // // 记录本次“断开”后累计重连耗时
    // int m_reconnectElapsedMs;
    int m_reconnectAttempts;  // 重连尝试次数
    bool m_reconnecting ;   // 正在重连过程中
};

#endif // TCPCLIENT_H
