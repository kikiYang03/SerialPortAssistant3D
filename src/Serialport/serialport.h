#ifndef SERIALPORT_H
#define SERIALPORT_H

#include <QMainWindow>
#include <QSerialPort>
#include <QString>
#include <QSerialPortInfo>
#include <QMessageBox>
#include <QTimer>
#include <QPainter>
#include <QDebug>
#include <QStatusBar>
#include <QDateTime>
#include <QTextCodec>
#include <QByteArray>
#include <QTcpSocket>
#include <QUdpSocket>
#include <QTcpServer>
#include <QNetworkInterface>
#include <QNetworkProxy>
#include <QDataStream>
#include <QtEndian>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QElapsedTimer>
#include <protocolhandler.h>
#include <tcpclient.h>
#include <protocolrouter.h>

namespace Ui {
class SerialPort;
}

class SerialPort : public QWidget
{
    Q_OBJECT

public:
    explicit SerialPort(QWidget *parent = nullptr);
    ~SerialPort();

    QSerialPort *serialPort;

    // 添加公共访问方法以便Params类可以访问连接状态
    bool getIsTcpConnected() const { return isTcpConnected; }
    bool getIsUdpBound() const { return isUdpBound; }
    bool getIsSerialPortConnected() const { return isSerialPortConnected; }

    // 添加公共数据发送方法
    void sendData(const QByteArray &data);  // 将sendData改为public

    // 添加公共方法供其他界面调用
    void appendMessage(const QString &message) {
        emit appendToDisplay(message);
    }

protected:
    void findFreePorts();  //查找可用串口
    bool initSerialPort(); //初始化串口连接

public slots:
    void handleTestFrame(const QByteArray &frame, bool isResponse);

private slots:
    void onTestTimeout();
    // 串口通信相关函数
    void on_portSearchBt_clicked();
    void on_portOpenBt_clicked();
    void on_clearRecvBt_clicked();
    void on_sendBt_clicked();
    void on_btnClearSend_clicked();

    // WiFi连接
    void on_wifiConnectBt_clicked();

    void on_protocolComboBox_currentIndexChanged(int index);

private:
    Ui::SerialPort *ui;

    // 发送、接收字节计数
    long sendNum, recvNum;

    // 接收数据缓冲区
    QByteArray recvBuffer;

    // 测试相关变量
    bool testFlag;
    QTimer *testTimer;

    // 协议数据解析
    void parseProtocolData(const QByteArray &data);
    void processProtocolFrame(const QByteArray &frame);
    // 数据处理
    void processReceivedData(QByteArray &recBuf);

    // 串口连接状态
    bool isSerialPortConnected;

    // WiFi通信相关
    QUdpSocket *udpSocket;
    bool isTcpConnected;
    bool isUdpBound;

    // 添加TCP客户端单例访问
    TcpClient* getTcpClient() { return TcpClient::getInstance(); }

    // 统计相关变量
    int tfCount = 0;
    int scanCount = 0;
    int mapCount = 0;
    QTimer* statsTimer;
    QElapsedTimer elapsedTimer;
    void onStatsTimeout();

    bool m_reconnectWarningShown = false;
    void tryExtractUartFrame();          // 从 recvBuffer 拆 10 B 定长帧
    void handleUartFrame(const QByteArray& frame); // 解析测试/参数命令
    void setupConnections();

signals:
    void rawBytesArrived(QByteArray data, bool isSerialPortMode);

    void coordinatesUpdated(qint16 x, qint16 y, qint16 z, qint16 yaw);

     // 新增：向显示窗口添加信息
    void appendToDisplay(const QString &message);

    void parameterResponseReceived(const QByteArray &data);  // 参数响应信号
    // 清理绘图视图
    void requestClearVisualization();



};


#endif // SERIALPORT_H
