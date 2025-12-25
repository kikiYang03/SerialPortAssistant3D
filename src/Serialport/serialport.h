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


// 简化ROS消息struct（自定义，非ROS原生类型）
struct OccupancyGrid {
    int width, height;
    float resolution;
    QVector<float> data;  // 占用值 [-1:未知, 0:空, 100:占用]
    float origin_x, origin_y;
};

struct LaserScan {
    float angle_min, angle_max, angle_increment;
    QVector<float> ranges;  // 距离数组
    float range_min, range_max;
};

// 修改后的 TFMessage 结构体
struct TFMessage {
    struct Header {
        QString frame_id;  // 父坐标系，通常是 "map"
    };

    struct Transform {
        Header header;           // 添加 header 字段
        QString child_frame_id;  // 添加 child_frame_id 字段
        float x, y, z, yaw;      // 位姿
    };

    QVector<Transform> transforms;  // 多变换
};

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

    // 解析ROS数据帧
    // void parseRosFrame(quint8 topicId, const QByteArray &payload);
    void parseRosFrame(quint8 topicId, const QJsonObject &jsonData);
    double quaternionToYaw(double x, double y, double z, double w);

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
    void sendMsg(const QString &msg); //发送消息（可保留原有功能）

private slots:
    void onTestTimeout();
    void parseTestData(const QByteArray &frame);
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
    void processProtocolFrame(const QByteArray &frame);
    void parseProtocolData(const QByteArray &data);
    // ROS数据解析
    void parseRosData(const QByteArray& recBuf);
    // 数据处理
    void processReceivedData(const QByteArray &recBuf);

    // 串口连接状态
    bool isSerialPortConnected;

    // WiFi通信相关
    // QTcpSocket *tcpSocket;
    QUdpSocket *udpSocket;
    bool isTcpConnected;
    bool isUdpBound;

    // 添加TCP客户端单例访问
    TcpClient* getTcpClient() { return TcpClient::getInstance(); }

    // 删除私有的sendData声明，因为已经移到public
    // void sendData(const QByteArray &data);

    // TF缓存
    QMap<QString, TFMessage::Transform> tfCache;

    // 统计相关变量
    int tfCount = 0;
    int scanCount = 0;
    int mapCount = 0;
    QTimer* statsTimer;
    QElapsedTimer elapsedTimer;
    void onStatsTimeout();

    bool m_reconnectWarningShown = false;

signals:
    void coordinatesUpdated(qint16 x, qint16 y, qint16 z, qint16 yaw);

     // 新增：向显示窗口添加信息
    void appendToDisplay(const QString &message);

    // ROS数据更新信号
    void rosMapUpdated(const OccupancyGrid& map);
    void rosScanUpdated(const LaserScan& scan);
    void rosTfUpdated(const TFMessage& tf);

    void parameterResponseReceived(const QByteArray &data);  // 参数响应信号
    // 清理绘图视图
    void requestClearVisualization();

};


#endif // SERIALPORT_H
