#include "tcpclient.h"
#include <QHostAddress>
#include <QDateTime>

TcpClient* TcpClient::m_instance = nullptr;
QMutex TcpClient::m_mutex;

TcpClient::TcpClient(QObject* parent)
    : QObject(parent)
    , m_isConnected(false)
    , m_reconnectAttempts(0)
    , m_reconnecting(false)
{
    m_tcpSocket     = new QTcpSocket(this);
    m_connectionTimer = new QTimer(this);
    m_reconnectTimer  = new QTimer(this);

    m_connectionTimer->setSingleShot(true);
    m_reconnectTimer ->setSingleShot(true);   // 每次只触发一次，由槽函数决定下一次

    m_tcpSocket->setProxy(QNetworkProxy::NoProxy);

    connect(m_tcpSocket, &QTcpSocket::connected,    this, &TcpClient::onConnected);
    connect(m_tcpSocket, &QTcpSocket::disconnected, this, &TcpClient::onDisconnected);
    connect(m_tcpSocket, &QTcpSocket::readyRead,    this, &TcpClient::onReadyRead);
    connect(m_tcpSocket, QOverload<QAbstractSocket::SocketError>::of(&QTcpSocket::errorOccurred),
            this, &TcpClient::onErrorOccurred);

    connect(m_connectionTimer, &QTimer::timeout, this, &TcpClient::onConnectionTimeout);
    connect(m_reconnectTimer,  &QTimer::timeout, this, &TcpClient::onReconnect);
}

TcpClient::~TcpClient()
{
    if (m_tcpSocket->state() == QAbstractSocket::ConnectedState)
    {
        m_tcpSocket->disconnectFromHost();
        if (m_tcpSocket->state() == QAbstractSocket::ConnectedState)
            m_tcpSocket->waitForDisconnected(1000);
    }
}

TcpClient* TcpClient::getInstance()
{
    QMutexLocker locker(&m_mutex);
    if (!m_instance) m_instance = new TcpClient;
    return m_instance;
}

/* 外部接口：发起一次新的连接，同时清零重连计数器 */
bool TcpClient::connectToHost(const QString& ip, quint16 port, int timeoutMs)
{
    if (m_isConnected) disconnectFromHost();

    if (m_tcpSocket->state() != QAbstractSocket::UnconnectedState)
        m_tcpSocket->abort();

    m_serverIp   = ip;
    m_serverPort = port;

    /* 新一轮连接，重连状态全部归零 */
    m_reconnectAttempts = 0;
    m_reconnecting      = false;
    m_reconnectTimer->stop();

    qDebug() << "开始连接TCP服务器..." << ip << ":" << port << "timeoutMs:" << timeoutMs;
    qDebug() << "[CONNECT]" << QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss")
             << "connectToHost" << ip << port << "attempt=0";
    m_connectionTimer->start(1000);
    m_tcpSocket->connectToHost(ip, port);
    return true;
}

void TcpClient::disconnectFromHost()
{
    m_connectionTimer->stop();
    m_reconnectTimer ->stop();
    m_reconnecting   = false;

    if (m_tcpSocket->state() == QAbstractSocket::ConnectedState)
        m_tcpSocket->disconnectFromHost();
}

bool TcpClient::isConnected() const
{
    return m_isConnected && (m_tcpSocket->state() == QAbstractSocket::ConnectedState);
}

qint64 TcpClient::sendData(const QByteArray& data)
{
    if (!isConnected())
    {
        qWarning() << "TCP未连接，无法发送数据";
        return -1;
    }
    qint64 ret = m_tcpSocket->write(data);
    if (ret == -1)
        qWarning() << "TCP数据发送失败:" << m_tcpSocket->errorString();
    return ret;
}

/* =============================================================
 * 私有槽函数
 * ============================================================= */
void TcpClient::onConnected()
{
    m_connectionTimer->stop();
    setConnectionStatus(true);

    m_reconnectAttempts = 0;
    m_reconnecting      = false;
    m_reconnectTimer->stop();

    qDebug() << "TCP连接成功！";
    emit connected();
}

void TcpClient::onDisconnected()
{
    qDebug() << "[DISCONN]" << QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss");
    setConnectionStatus(false);
    qDebug() << "TCP连接断开";

    /* 防止重复启动 */
    if (m_reconnecting) return;

    if (m_reconnectAttempts < 1)
    {
        m_reconnecting = true;
        ++m_reconnectAttempts;
        qDebug() << "安排第" << m_reconnectAttempts << "次重连定时器";
        m_reconnectTimer->start(1000);
    }
    else
    {
        qDebug() << "重连1次均失败，停止重连";
        emit reconnectTimeout();
    }
}

void TcpClient::onErrorOccurred(QAbstractSocket::SocketError)
{
    m_connectionTimer->stop();
    setConnectionStatus(false);

    QString err = m_tcpSocket->errorString();
    qWarning() << "TCP连接错误:" << err;
    emit connectionError(err);

    if (m_reconnectAttempts >= 1)
    {
        qDebug() << "重连1次均失败，停止重连";
        stopAutoReconnect();
        emit reconnectTimeout();
        return;
    }

    if (m_reconnecting)
    {
        ++m_reconnectAttempts;
        qDebug() << "安排第" << m_reconnectAttempts << "次重连定时器";
        m_reconnectTimer->start(1000);
    }
    else
    {
        m_reconnecting = true;
        ++m_reconnectAttempts;
        qDebug() << "连接错误，开始第" << m_reconnectAttempts << "次重连尝试";
        m_reconnectTimer->start(1000);
    }
}

void TcpClient::onConnectionTimeout()
{
    if (m_isConnected) return;

    qDebug() << "TCP连接超时(1s)，主动abort";
    qDebug() << "[TIMEOUT]" << QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss");
    m_tcpSocket->abort();
    emit connectionError("连接超时，请检查网络和服务器");

    if (m_reconnectAttempts < 1)
    {
        m_reconnecting = true;
        ++m_reconnectAttempts;
        qDebug() << "连接超时，安排第" << m_reconnectAttempts << "次重连定时器";
        m_reconnectTimer->start(1000);
    }
    else
    {
        qDebug() << "连接超时，重连1次均失败，停止重连";
        stopAutoReconnect();
        emit reconnectTimeout();
    }
}

void TcpClient::onReconnect()
{
    /* 定时器 singleShot，每次只进来一次 */
    if (m_tcpSocket->state() != QAbstractSocket::UnconnectedState)
        m_tcpSocket->abort();

    qDebug() << "重连尝试：" << m_reconnectAttempts << "/1, 连接状态："
             << m_isConnected << ", 正在重连：" << m_reconnecting;

    m_tcpSocket->connectToHost(m_serverIp, m_serverPort);
}

void TcpClient::setConnectionStatus(bool connected)
{
    if (m_isConnected == connected) return;
    m_isConnected = connected;
    qDebug() << "更新连接状态：" << (connected ? "已连接" : "未连接");
    emit connectionStatusChanged(connected);
}

void TcpClient::stopAutoReconnect()
{
    m_reconnecting      = false;
    m_reconnectAttempts = 0;
    m_reconnectTimer->stop();
}

void TcpClient::onReadyRead()
{
    QByteArray data = m_tcpSocket->readAll();
    if (!data.isEmpty())
        emit dataReceived(data);
}
