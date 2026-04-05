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
#include <tcpclient.h>
#include <protocolrouter.h>
#include "glwidget.h"
#include "adminmode.h"


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
    // 初始化Glwiget
    void setGLWidget(GLWidget* w);

protected:
    void findFreePorts();  //查找可用串口
    bool initSerialPort(); //初始化串口连接

public slots:
    void handleTestFrame(const QByteArray &frame, bool isResponse);
    void onSendNavGoalRequested(double x, double y, double z, double yaw_deg);

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

    // 断连重试定时器
    QTimer* statsTimer;
    QElapsedTimer elapsedTimer;

    bool m_reconnectWarningShown = false;
    void tryExtractUartFrame();          // 从 recvBuffer 拆 10 B 定长帧
    void handleUartFrame(const QByteArray& frame); // 解析测试/参数命令
    void setupConnections();

    GLWidget* glWidget_;   // 新增

    // 数据打印和统计相关
    QTimer* printTimer;              // 每1秒打印定时器
    QTimer* statsPrintTimer;         // 每10秒统计打印定时器

    // 缓存最新的位姿数据
    struct PoseData {
        qint16 x = 0, y = 0, z = 0, roll = 0, pitch = 0, yaw = 0;
        bool valid = false;
    };
    PoseData latestPose01;    // 0x01 当前位姿
    PoseData latestPose03;    // 0x03 位置控制指令
    QByteArray latestFrame02; // 0x02 目标点回传原始数据

    // 接收计数器（用于频率统计）
    int count01 = 0;  // 0x01计数
    int count02 = 0;  // 0x02计数
    int count03 = 0;  // 0x03计数
    int statsIntervalSeconds = 10;  // 统计间隔（秒）

    // 打印数据槽函数
    void onPrintTimerTimeout();
    void onStatsPrintTimerTimeout();

signals:
    void rawBytesArrived(QByteArray data, bool isSerialPortMode);

    void coordinatesUpdated(qint16 x, qint16 y, qint16 z, qint16 yaw);

     // 新增：向显示窗口添加信息
    void appendToDisplay(const QString &message);

    void parameterResponseReceived(const QByteArray &data);  // 参数响应信号

    void tcpConnectionChanged(bool connected);

};


#endif // SERIALPORT_H
