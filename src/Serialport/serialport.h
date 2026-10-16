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
#include <QMap>          // 保存参数回读：按参数号匹配（顺序不可信）
#include <tcpclient.h>
#include <protocolrouter.h>
#include "glwidget.h"
#include "adminmode.h"
#include "waitdialog.h"
#include "params.h"     // ParamWrite：参数页交来的待写入参数表
#include "protocol_msg.h"   // MapChunkMsg：地图下传分块（0x0C）


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

    // 保存地图：发 AA 00 02 0A，等模块回包，超时判失败
    void onSaveMapRequested();
    void onSaveMapAckReceived();

    // 写入参数：参数页只收集值，由这里逐条发 AA 10 <参数号> <值>，
    // 再补一条保存终结指令 AA 00 04 0A；发完就地等回读、比对、下结论（工单 06）
    void onWriteParamsRequested(const QVector<ParamWrite> &params);
    void onSaveParamsAckReceived();   // 收到终结指令 AA 00 04 0A：回读已发完，可以比对了

    // 获取参数：参数页只报出「表里有多少条」，本层发 AA 00 03 0A、逐条收 AA 10、
    // 收满条数才下结论。0x03 的应答**没有终结帧**，所以判据只能是条数（见 .cpp）
    void onReadParamsRequested(int expectedCount);

    // 下载地图：发 AA 00 05 0A，逐块收 0x0C，收齐后校验整包、再让用户选路径、落盘（工单 10）
    void onDownloadMapRequested();
    void onMapChunkReceived(const MapChunkMsg &msg);   // 模块下传的每一块（0x0C）

    // 重启算法（工单 01）：发 AA 00 06 0A，等模块原样回发。模块先回发、再断电重启 ——
    // 回显是判成功的唯一依据，本功能不推断断线时机。
    // 判定成功才清理界面上的模块运行态镜像（工单 02）并弹那句回执；
    // 超时路径既不清理也不弹框，只在日志里写一行「重启算法失败」。
    void onRestartAckReceived();

    // 控制链路（TCP 已连接或 UDP 已绑定）门禁：切换本页「重启算法」的可用性。
    // 与 3D 页 / 参数页的同名方法接的是同一个信号，本页只是追加了一路接收方。
    void setLinkConnected(bool connected);

private slots:
    void onTestTimeout();
    void onSaveMapTimeout();
    void onSaveParamsTimeout();
    void onReadParamsTimeout();      // 读取参数超时：没在窗口内收齐参数表条数
    void onDownloadChunkTimeout();   // 块间静默超时：两块之间的空档（含首块之前）
    void onDownloadTotalTimeout();   // 整包总超时：兜底，不让整包无限期挂着
    void onDownloadCancelled();      // 进度框上的「取消」：丢弃已收数据，不落盘、不报错、不写日志
    void onRestartTimeout();         // 回发等待窗口超时：按钮恢复可用 + 一行「重启算法失败」，数据一个都不动
    // 模块下发的每一条 AA 10：既缓存 0x99 模块类型与 0x13 模式，也在保存等待期喂给比对器
    void onModuleParameterFrame(quint8 paramId, qint16 value);
    // 串口通信相关函数
    void on_portSearchBt_clicked();
    void on_portOpenBt_clicked();
    void on_clearRecvBt_clicked();
    void on_sendBt_clicked();
    void on_btnClearSend_clicked();
    void on_restartBt_clicked();     // 「重启算法」（Designer 表单里的按钮，靠命名自动连接；按下即发，不二次确认）

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

    // 保存地图相关变量
    QTimer *saveMapTimer;            // 单次超时定时器（不复用 testTimer：后者带着 3 秒的通信测试超时）
    bool saveMapProbePending;        // 存图超时后补发的通信测试尚未出结果
    int latestModuleType = -1;       // 参数 0x99，未读到过为 -1（读不到就按 3D 的宽超时兜底）
    int latestModuleMode = -1;       // 参数 0x13「模式」，未读到过为 -1（见 setModuleMode）

    // 参数 0x13「模式」的落地处：缓存 + 只在取值变化时下发给图形层（工单 03）。
    // 与 0x99 同一条路径 —— 同一个 AA 10 槽缓存、同一处清理，不另起一套。
    void setModuleMode(int mode);

    // 保存地图等待框：可复用组件（工单 06 直接复用、工单 10 在其上扩展），
    // 实例反复用，父子关系管生命周期。回包或超时时先关框，再由调用方弹结果。
    WaitDialog *saveMapWait_ = nullptr;
    void showSaveMapWait();
    void closeSaveMapWait();

    int saveMapTimeoutMs() const;    // 3D 30 秒 / 2D 10 秒，走 config.ini

    // 保存参数等待闭环（工单 06）////////////////////////////////////////////
    // 与存图那套同形，但多了「回读比对」这一步：模块先把全表 21 条 AA 10 发回来，
    // 再回终结指令 AA 00 04 0A，收到它才算回读发完、可以下结论。
    QTimer *saveParamsTimer = nullptr;      // 单独一个单次超时定时器
    WaitDialog *saveParamsWait_ = nullptr;  // 复用 04 的组件，实例反复用
    bool saveParamsPending = false;

    // 比对基准 = 「点击那一刻」界面上的全部参数值。它由参数页在点击时收集、随信号一起交来，
    // 等待期用户再改界面也动不到它。
    QVector<ParamWrite> saveParamsSnapshot;
    // 等待期收到的回读值，按参数号存：模块的回发顺序与发送顺序不同，按号匹配才不会误报
    QMap<quint8, qint16> saveParamsReadback;

    void showSaveParamsWait();
    void closeSaveParamsWait();
    void settleSaveParams(const QString &logLine, const QString &summary,
                          const QString &title, const QString &text,
                          const QString &detail, QMessageBox::Icon icon);
    int saveParamsTimeoutMs() const;        // 走 config.ini，默认 10 秒

    // 读取参数等待闭环 /////////////////////////////////////////////////////
    // 与保存参数同形（等待框 + 单次超时 + 收尾），差在完成判据：那边等的是终结指令
    // AA 00 04 0A，这边**等不到任何终结帧**——模块回完 21 条 AA 10 就没了，
    // 所以只能按「收满参数表的条数」判齐（与下载地图按块数判齐同一条路子）。
    QTimer *readParamsTimer = nullptr;      // 单个单次超时定时器（语义与保存参数不同，不复用）
    WaitDialog *readParamsWait_ = nullptr;  // 04 的组件 + setProgress，实例反复用
    bool readParamsPending = false;         // 正在读取；为假时到的 AA 10 一律不进这次计数

    // 期望条数由参数页在点击那一刻报来（本层不持有参数表，不该替它数）
    int readParamsExpected = 0;
    // 等待期收到的参数值，按参数号存：**重复帧不算进度**，收满「不同参数号」的条数才算齐
    QMap<quint8, qint16> readParamsReceived;

    void showReadParamsWait();
    void closeReadParamsWait();
    void settleReadParamsSuccess();         // 收齐了：关框 → 记日志 → 弹「接收成功」
    void settleReadParams(const QString &logLine, const QString &summary,
                          const QString &title, const QString &text,
                          const QString &detail, QMessageBox::Icon icon);
    int readParamsTimeoutMs() const;        // 走 config.ini，默认 10 秒

    // 地图下载等待闭环（工单 10）////////////////////////////////////////////
    // 与存图 / 存参数同形，但形态不同：带进度、可取消，且**两级超时**。
    // 完成判据是「收齐了总块数」——模块发完没有任何终结帧；失败时一帧都不发，
    // 所以块间静默（卡在第 3 块不该和第 28 块等一样久）与整包总超时（兜底）两条都在。
    QTimer *downloadChunkTimer = nullptr;   // 块间静默超时（每收一块重置）
    QTimer *downloadTotalTimer = nullptr;   // 整包总超时
    WaitDialog *downloadWait_ = nullptr;    // 04 的组件 + setProgress / setCancelEnabled
    bool downloadPending = false;           // 正在下载；为假时到的分块一律丢弃（取消/超时后模块还会发完）
    QVector<MapChunkMsg> downloadChunks;    // 已收分块，收齐后整包交给 MapArchive 拼装校验
    QString downloadPkgName;                // 模块自报的包名，预填文件对话框
    int downloadTotalChunks = 0;            // 模块自报的总块数（0 = 还没听它说过）

    void showDownloadWait();
    void closeDownloadWait();
    void onDownloadComplete();              // 收齐：校验 → 选路径 → 落盘
    // 失败收尾：停表、丢块、关框、记日志、弹结果框。取消不走这里（取消不报错）
    void abortDownload(const QString &logLine, const QString &text, const QString &detail);
    int downloadChunkTimeoutMs() const;     // 走 config.ini，默认 5 秒
    int downloadTotalTimeoutMs() const;     // 走 config.ini，默认 120 秒
    QString downloadDefaultDir() const;     // 走 config.ini，默认取可执行文件旁边的目录

    // 重启算法等待闭环（工单 01）////////////////////////////////////////////
    // 与存图 / 存参数同形（单独一个单次定时器 + 一个等待标志），但**没有失败清理、也没有失败提示**：
    // 超时意味着模块很可能根本没重启，界面上那些运行态镜像依然是有效的，顺手清掉就是毁掉还在用
    // 的东西；而提示那一侧，本功能全程只弹一次框（收到回发时的回执），超时静默。
    QTimer *restartTimer = nullptr;         // 3 秒单次超时（RESTART_ACK_TIMEOUT_MS，见 .cpp）
    bool restartPending = false;            // 正在等回发：等待窗口内按钮置灰，挡掉第二条重启指令
    bool controlLinkConnected_ = false;     // 「TCP 已连接或 UDP 已绑定」的最近一次取值

    // 「重启算法」的可用性只由这一处决定（含等待窗口与串口模式两项）
    void updateRestartButton();

    // 重启判定成功后清掉本页缓存的运行态镜像（工单 02）：位姿两路（0x01 / 0x03）、
    // 模块类型（0x99）、模式（0x13）。
    // 不清的话每秒一次的打印定时器会继续把重启前的旧位姿当当下值吐到接收窗口。
    void clearModuleRuntimeCaches();

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

    // 接收计数器（用于频率统计）。0x04 / 0x05 不再有每秒打印，计数只喂下面那次统计
    int count01 = 0;  // 0x01计数
    int count02 = 0;  // 0x02计数
    int count03 = 0;  // 0x03计数
    int count04 = 0;  // 0x04计数（实时速度帧，100Hz）
    int count05 = 0;  // 0x05计数（飞控状态帧，1Hz）
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

    // 控制指令链路（TCP / UDP）状态：给可视化页的「保存地图」、参数页的「写入参数」、
    // 本页的「重启算法」开门 —— 三个接收方，同一条门禁。
    // 串口不算 —— 15 字节帧里没有控制指令这一类消息（0x00 系列），
    // 与测试按钮在串口模式下被禁用是同一条规矩。
    void controlLinkStateChanged(bool connected);

    // 保存参数等待期间：为真时参数页不拿模块回读值刷界面（回读只喂本页的比对器）
    void paramEchoSuppressedChanged(bool suppressed);

    // 参数 0x13「模式」的取值（工单 03）：0 = 建图、1 = 定位、-1 = 本会话还没读到过。
    // 图形层据此判要不要画开机点标记（工单 09）。**只在取值变化时发**，且「没读到过」
    // 会单独发一次 -1 —— 默认按 0（建图）处理会让图形层把「不知道」当成「在建图」。
    void moduleModeChanged(int mode);

    // 保存参数闭环的结论摘要（全部一致 / 有出入 N 项 / 超时未集齐），给参数页的状态行
    void saveParamsFinished(const QString &summary);

    // 读取参数闭环的结论摘要（收齐 N 条 / 超时只收到 N 条），给参数页的状态行
    void readParamsFinished(const QString &summary);

    // 重启指令判定成功（窗口内收到模块回发）：3D 页面据此清掉 TF 数值标签、规划 / 重定位
    // 状态标签与跳变锁存（工单 02）。**只在成功时发** —— 超时失败路径不发，那时模块
    // 很可能根本没重启，界面上的数据依然有效，清掉就是毁掉还在用的东西。
    void restartSucceeded();

};


#endif // SERIALPORT_H
