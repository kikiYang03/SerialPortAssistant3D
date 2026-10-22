#include "serialport.h"
#include "ui_serialport.h"
#include "utils/config/config.h"
#include "protocolros3d.h"
#include "protocolrouter.h"
#include "maparchive.h"     // 地图下传分块的拼装 + CRC-32 校验（工单 09）
#include "Theme.h"         // 状态色：setState 只写属性，颜色在全局 QSS 里定（工单 03）

#include <QDataStream>
#include <QTimer>
#include <QMessageBox>
#include <QStringList>
#include <QCoreApplication>
#include <QDir>
#include <QFileDialog>
#include <QSaveFile>
#include <qmath.h>


// 重启算法（工单 01）的回发等待窗口。模块收到指令即回发（毫秒级的事），3 秒足够容忍链路
// 延迟，又不至于让人干等。定义在这一处，以后要调只改这里。
static const int RESTART_ACK_TIMEOUT_MS = 3000;

namespace {

// 串口链路的文本格式只在这里定（工单 07）：端口 · 波特率 · 状态。
// 串口页那行标签、标题栏胶囊的 tooltip、状态栏第一段吃的都是这**一条**字符串
// （同一条 portStateChanged），所以格式必须只有一处 —— 三处各拼一遍迟早不同步。
// 端口未知（还没选过口）时用占位符，不留空、也不留半截的「· · 已关闭」。
QString portStateText(const QString &port, qint32 baud, const QString &state)
{
    const QString name = port.trimmed().isEmpty() ? QStringLiteral("—") : port;
    return QStringLiteral("%1 · %2 · %3").arg(name).arg(baud).arg(state);
}

// 0x05 状态帧那三件枚举的中文（只有这一处，日志与将来的界面共用同一份说法）：
// 飞控五档见协议「附：0x05 飞控状态枚举值定义」，重定位 / 规划四档见
// 「Byte 3 重定位 / Byte 4 规划 取值」。表外取值不猜，原样报出数字。
QString fcStateText(quint8 v)
{
    switch (v) {
    case 0:  return QStringLiteral("未连接");
    case 1:  return QStringLiteral("未解锁");
    case 2:  return QStringLiteral("已解锁·地面");
    case 3:  return QStringLiteral("空中悬停");
    case 4:  return QStringLiteral("执行航点");
    default: return QStringLiteral("未知(%1)").arg(v);
    }
}

QString progressStateText(quint8 v)
{
    switch (v) {
    case 0:  return QStringLiteral("进行中");
    case 1:  return QStringLiteral("成功");
    case 2:  return QStringLiteral("失败");
    case 3:  return QStringLiteral("未响应");
    default: return QStringLiteral("未知(%1)").arg(v);
    }
}

// 百分比：`0xFF` 是「未知」哨兵，不是读数（upper 没起 / 超 2 秒没采样）。
// 折成 0% 或钳到 100% 都是在编造一条模块没报过的数。
QString percentText(quint8 v)
{
    return v == ProtocolRouter::kUartPercentUnknown
               ? QStringLiteral("—")
               : QStringLiteral("%1%").arg(v);
}

} // namespace



// 初始化ui界面
SerialPort::SerialPort(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::SerialPort)
    , m_reconnectWarningShown(false)
    , glWidget_(nullptr)
{
    ui->setupUi(this);

    // 连接显示信号
    connect(this, &SerialPort::appendToDisplay, this, [this](const QString &message) {
        ui->recvEdit->append(message);
    });

    // 串口链路状态只有一个数据源（工单 05）：本页的标签是 portStateChanged 的订阅者，
    // 标题栏的胶囊也是。所以下面各处只 emit、不再自己 setText / setState。
    connect(this, &SerialPort::portStateChanged, this, &SerialPort::applyPortState);
    // 控制链路（TCP / UDP）的显示同理（工单 07）：本页标签与状态栏第二段都订阅这一条，
    // 本页不再直接改 ui->lblWifiState —— 双写迟早出现「页面说 TCP、状态栏说 UDP」。
    connect(this, &SerialPort::controlLinkTextChanged, this, &SerialPort::applyControlLinkState);

    // 初始化串口
    findFreePorts();
    serialPort = new QSerialPort(this);
    connect(serialPort, &QSerialPort::readyRead, this, [this]() {
        QByteArray recBuf = serialPort->readAll();
        processReceivedData(recBuf);
    });
    isSerialPortConnected = false;

    // 初始化TCP客户端连接
    // 协议选择
    ui->protocolComboBox->addItem("TCP");
    ui->protocolComboBox->addItem("UDP");

    QString ipText = Config::instance().value("Network/tcp_ip", "127.0.0.1").toString();
    QString port = Config::instance().value("Network/tcp_port", "8088").toString();
    // ui->ipInput->setEnabled(false);
    AdminMode::instance().install(ui->ipInput, true, false);
    ui->ipInput->setText(ipText);
    // ui->portInput->setEnabled(false);
    AdminMode::instance().install(ui->portInput, true, false);
    ui->portInput->setText(port);
    ui->protocolComboBox->setEnabled(false);
    TcpClient* tcpClient = TcpClient::getInstance();
    connect(tcpClient, &TcpClient::dataReceived, this, &SerialPort::processReceivedData);
    connect(tcpClient, &TcpClient::connectionStatusChanged, this, [this](bool connected) {
        if (connected) {
            emit controlLinkTextChanged(QStringLiteral("TCP 已连接"), true);
            ui->wifiConnectBt->setText("关闭连接");

            // TCP连接成功时启动统计定时器
            if (!statsTimer->isActive()) {
                statsTimer->start();
                elapsedTimer.restart();
            }

        } else {
            // TCP 掉了：UDP 还绑着的话链路其实没断，文案就得说 UDP —— 判据与门禁
            // 那条 canSendControlCommands()（`connected || isUdpBound || 串口`）是同一个，
            // 不在这里另立一套「算不算断了」的规则。
            // 注意这一段管的是**网络**那条文案（TCP / UDP），串口那段另有 portStateChanged，
            // 所以串口开着而 TCP 掉了时，这里照样说「尝试自动重连」。
            if (isUdpBound)
                emit controlLinkTextChanged(QStringLiteral("UDP 已连接"), true);
            else
                emit controlLinkTextChanged(QStringLiteral("尝试自动重连"), false);

            // 停止统计定时器
            if (statsTimer->isActive()) {
                statsTimer->stop();
            }
        }
        emit tcpConnectionChanged(connected);
        // 两条门禁都交给唯一判定处：TCP 掉了但 UDP 绑着 / 串口开着，指令照样发得出去
        emitCommandLinkGates();
    });

    // 初始化WiFi UDP
    udpSocket = new QUdpSocket(this);
    connect(udpSocket, &QUdpSocket::readyRead, this, [this]() {
        while (udpSocket->hasPendingDatagrams()) {
            QByteArray datagram;
            datagram.resize(udpSocket->pendingDatagramSize());
            QHostAddress sender;
            quint16 senderPort;
            udpSocket->readDatagram(datagram.data(), datagram.size(), &sender, &senderPort);
            processReceivedData(datagram);
        }
    });
    isUdpBound = false;

    // 初始化测试相关变量
    testFlag = false;
    testTimer = new QTimer(this);
    testTimer->setSingleShot(true);  // 单次定时器
    connect(testTimer, &QTimer::timeout, this, &SerialPort::onTestTimeout);

    // 保存地图超时：单独一个单次定时器，不复用 testTimer
    // （testTimer 的 3 秒是通信测试的语义，两者会互相踩）
    saveMapTimer = new QTimer(this);
    saveMapTimer->setSingleShot(true);
    connect(saveMapTimer, &QTimer::timeout, this, &SerialPort::onSaveMapTimeout);
    saveMapProbePending = false;

    // 保存参数超时：同样单独一个单次定时器（语义与通信测试、存图都不同，不复用）
    saveParamsTimer = new QTimer(this);
    saveParamsTimer->setSingleShot(true);
    connect(saveParamsTimer, &QTimer::timeout, this, &SerialPort::onSaveParamsTimeout);

    // 读取参数超时：同理单独一个（等的是「收满条数」，与等终结帧的保存参数不是一回事）
    readParamsTimer = new QTimer(this);
    readParamsTimer->setSingleShot(true);
    connect(readParamsTimer, &QTimer::timeout, this, &SerialPort::onReadParamsTimeout);

    // 地图下载的两级超时：两条各一个单次定时器。
    // 块间静默那条每收一块就重置（卡在第 3 块不该和第 28 块等一样久），
    // 整包那条从头跑到尾（兜底，不让整包无限期挂着）。
    downloadChunkTimer = new QTimer(this);
    downloadChunkTimer->setSingleShot(true);
    connect(downloadChunkTimer, &QTimer::timeout, this, &SerialPort::onDownloadChunkTimeout);

    downloadTotalTimer = new QTimer(this);
    downloadTotalTimer->setSingleShot(true);
    connect(downloadTotalTimer, &QTimer::timeout, this, &SerialPort::onDownloadTotalTimeout);

    // 重启算法（工单 01）的回发等待窗口：同样单独一个单次定时器（3 秒与通信测试/存图/存参数
    // 的语义都不同，不复用）。等待期按钮由 updateRestartButton() 置灰。
    restartTimer = new QTimer(this);
    restartTimer->setSingleShot(true);
    connect(restartTimer, &QTimer::timeout, this, &SerialPort::onRestartTimeout);

    // 本页「串口待连接」的红字由构造末尾那次 portStateChanged 初值落地（工单 05），
    // 不在这里直接写标签 —— 否则又是一份不受信号管辖的状态。

    // 控制链路的初值（工单 07）：还没连过 TCP / UDP。与串口那条一样走信号，
    // 本页标签与状态栏第二段一起落地，别在这里再写一遍标签。
    // 初值统一放到构造末尾那一次 QTimer::singleShot(0) 里发（见那里）——此刻本页
    // 之外的订阅者（标题栏胶囊、状态栏）还没接上信号。

    // ui->serialBox->setEnabled(false);

    // 发送、接收计数清零
    sendNum = 0;
    recvNum = 0;

    // 初始化接收缓冲区
    recvBuffer.clear();

    // 初始化统计定时器
    statsTimer = new QTimer(this);
    statsTimer->setInterval(10000); // 10秒

    // 初始化打印定时器（每1秒打印0x01和0x03数据）
    printTimer = new QTimer(this);
    printTimer->setInterval(1000); // 1秒
    connect(printTimer, &QTimer::timeout, this, &SerialPort::onPrintTimerTimeout);

    // 初始化统计打印定时器（每10秒统计频率）
    statsPrintTimer = new QTimer(this);
    statsPrintTimer->setInterval(10000); // 10秒
    connect(statsPrintTimer, &QTimer::timeout, this, &SerialPort::onStatsPrintTimerTimeout);

    connect(tcpClient, &TcpClient::reconnectTimeout, this, [this](){
        if (!m_reconnectWarningShown) {
            QMessageBox::warning(this, "网络中断",
                                 "信号较差，未能连接成功，\n"
                                 "已停止自动重连，请检查网络后手动'打开连接'。");
            m_reconnectWarningShown = true;

            // 界面恢复成"断开"状态
            ui->wifiConnectBt->setText("打开连接");
            emit controlLinkTextChanged(QStringLiteral("未连接"), false);
            ui->serialBox->setEnabled(true);
        }
    });

    // 断线被压制（工单 03）：这次断开是预期内的（重启算法），不是无来由掉线。
    // 「重连失败」弹窗不会出现 —— TcpClient 在开关关闭时根本不发 reconnectTimeout()，
    // 不需要在这里挡那个信号。这里只把界面说清楚：日志写一行，连接按钮复位成
    // 「打开连接」，让操作员知道下一步该由他手动重连。
    // 手动「关闭连接」（工单 04）复用同一把开关、同一个信号，走的就是这一行日志。
    connect(tcpClient, &TcpClient::autoReconnectSuppressed, this, [this]() {
        ui->recvEdit->append(QDateTime::currentDateTime().toString("HH:mm:ss >> ") +
                             QStringLiteral("连接已断开"));

        ui->wifiConnectBt->setText("打开连接");
        // 不写「尝试自动重连」—— 这一次确实一次都不会尝试（那是上面那个 lambda 的话）
        emit controlLinkTextChanged(QStringLiteral("未连接"), false);
        // serialBox 不动：恢复办法是手动重连 TCP / UDP，不是切回串口
    });

    elapsedTimer.start();
    // 串口数据解析 - 缓存0x01和0x03数据，仅在串口模式下计数
    connect(ProtocolRouter::instance(), &ProtocolRouter::uartPoseReceived,
            this, [this](quint8 msgId, qint16 x, qint16 y, qint16 z, qint16 r, qint16 p, qint16 yaw){
                // 仅在串口模式下处理
                if (!isSerialPortConnected) return;

                if (msgId == 0x01) {
                    // 缓存0x01数据
                    latestPose01.x = x;
                    latestPose01.y = y;
                    latestPose01.z = z;
                    latestPose01.roll = r;
                    latestPose01.pitch = p;
                    latestPose01.yaw = yaw;
                    latestPose01.valid = true;
                    count01++;
                } else if (msgId == 0x03) {
                    // 缓存0x03数据
                    latestPose03.x = x;
                    latestPose03.y = y;
                    latestPose03.z = z;
                    latestPose03.roll = r;
                    latestPose03.pitch = p;
                    latestPose03.yaw = yaw;
                    latestPose03.valid = true;
                    count03++;
                }
            });

    // 串口数据解析 - 0x04 规划速度帧（pos_cmd 的速度指令，非实测速度）：只计数，供每 10 秒的频率统计读数
    connect(ProtocolRouter::instance(), &ProtocolRouter::uartSpeedReceived,
            this, [this](qint16, qint16, qint16, qint16){
                // 仅在串口模式下处理
                if (!isSerialPortConnected) return;

                count04++;
            });

    // 串口数据解析 - 0x05 状态帧（1Hz，五件）：飞控 / 重定位 / 规划 + CPU / 运存百分比。
    // 前三个字节是枚举，后两格是百分比或 0xFF（未知）；只缓存 + 计数，打印由每秒那一次定时器统一做
    // （5 Hz 的帧不该有 5 行日志，1 Hz 也照样是一秒一行）。
    connect(ProtocolRouter::instance(), &ProtocolRouter::uartFlightStatusReceived,
            this, [this](quint8 fcState, quint8 relocalization, quint8 planning,
                         quint8 cpuPercent, quint8 memPercent){
                // 仅在串口模式下处理
                if (!isSerialPortConnected) return;

                latestStatus05.fcState = fcState;
                latestStatus05.relocalization = relocalization;
                latestStatus05.planning = planning;
                latestStatus05.cpuPercent = cpuPercent;
                latestStatus05.memPercent = memPercent;
                latestStatus05.valid = true;
                count05++;
            });

    // 处理0x02目标点回传数据，仅在串口模式下
    connect(ProtocolRouter::instance(), &ProtocolRouter::navGoalEchoReceived,
            this, [this](const QByteArray &frame){
                // 仅在串口模式下处理
                if (!isSerialPortConnected) return;

                count02++;
                // 解析帧数据并打印
                if (frame.size() == 15 && quint8(frame[0]) == 0xAA && quint8(frame[14]) == 0x0A) {
                    // 大端序解析
                    qint16 x = qFromBigEndian<qint16>(reinterpret_cast<const uchar*>(frame.constData() + 2));
                    qint16 y = qFromBigEndian<qint16>(reinterpret_cast<const uchar*>(frame.constData() + 4));
                    qint16 z = qFromBigEndian<qint16>(reinterpret_cast<const uchar*>(frame.constData() + 6));
                    qint16 yaw = qFromBigEndian<qint16>(reinterpret_cast<const uchar*>(frame.constData() + 12));

                    QString t = QDateTime::currentDateTime().toString("HH:mm:ss >> ");
                    ui->recvEdit->append(
                        QString("%1模块回传目标点：x=%2m y=%3m z=%4m yaw=%5°")
                        .arg(t)
                        .arg(x / 100.0, 0, 'f', 2)
                        .arg(y / 100.0, 0, 'f', 2)
                        .arg(z / 100.0, 0, 'f', 2)
                        .arg(yaw));
                }
            });

    // 入站的 AA 00 02 0A = 存图成功（模块只在产物核对通过之后原样回发，失败一个字节都不回）
    connect(ProtocolRouter::instance(), &ProtocolRouter::saveMapAckReceived,
            this, &SerialPort::onSaveMapAckReceived);

    // 入站的 AA 00 04 0A = 保存参数的终结指令回显：模块发完全表 21 条 AA 10 之后才发它，
    // 所以收到它就意味着「回读已发完」，等待可以结束了（工单 06）。
    connect(ProtocolRouter::instance(), &ProtocolRouter::saveParamsAckReceived,
            this, &SerialPort::onSaveParamsAckReceived);

    // 模块下发的每条 AA 10：既缓存 0x99 模块类型（用来挑存图超时，2/3 = 2D。模块只在被读取
    // 参数时才下发，读不到就停在 -1，按 3D 的宽超时兜底），也在保存等待期喂给比对器。
    connect(ProtocolRouter::instance(), &ProtocolRouter::parameterFrameReceived,
            this, &SerialPort::onModuleParameterFrame);

    // 入站的 AA 00 06 0A = 重启指令原样回发。模块先回发、再断电重启，所以「收到它」是本功能
    // 判成功的唯一依据（回显到达与连接断开是先后两件事，不去推断断线时机）。
    connect(ProtocolRouter::instance(), &ProtocolRouter::restartAckReceived,
            this, &SerialPort::onRestartAckReceived);

    // 初始不可点：还没连上任何控制链路（.ui 里也是 false，这里再按三条规则算一次）
    updateRestartButton();

    connect(ProtocolRouter::instance(), &ProtocolRouter::printFrame,
            this, [this](const QByteArray &frame){
                QString t = QDateTime::currentDateTime()
                .toString(Qt::ISODateWithMs)
                    .replace('T', ' ');
                QString hex = frame.toHex(' ');          // 先转成 QString
                ui->recvEdit->append(
                    QString("%1  原始数据: %2").arg(t, hex));   // C++14 起支持多参数 arg
            });

    // 串口链路状态的初值（工单 05/07）：启动到第一次连接之间，串口页的标签、标题栏的
    // 胶囊、状态栏的第一段都该有话说。文字走与打开/关闭同一条 portStateText()，
    // 所以三段格式一致（端口 · 波特率 · 状态）—— 状态栏第一段在没连过时也能看出
    // 即将用哪个口、哪个波特率，而不是只剩一个「未连接」。
    //
    // 用 singleShot(0) 推到事件循环第一轮再发，而不是在构造函数里直接 emit：
    // 此刻 MainWindow 还没执行到 connect(serialPort, ...) —— 标题栏胶囊与状态栏
    // 都还没订阅，直接 emit 会让它们永远停在自己的默认值上（「状态栏说未连接、
    // 串口页说 COM4 · 115200 · 未连接」）。这条初值与后来的每次状态变化走的是同一条
    // 信号、同一份文字，没有第二份状态。
    QTimer::singleShot(0, this, [this]() {
        emit portStateChanged(
            portStateText(ui->portNames->currentText(), QSerialPort::Baud115200,
                          QStringLiteral("未连接")),
            false);
        emit controlLinkTextChanged(QStringLiteral("未连接"), false);
        // 两道指令门禁也发一次初值：此刻三个按钮（存图 / 下载 / 写参数）都还是构造时的
        // 不可点状态，发一遍让「谁在决定它们」始终只有 emitCommandLinkGates() 这一处
        emitCommandLinkGates();
    });
}

// 本页状态标签的唯一写入点：文字 + 红/绿都跟着 portStateChanged 走。
void SerialPort::applyPortState(const QString &text, bool ok)
{
    if (!text.isEmpty())
        ui->lblPortState->setText(text);
    // 颜色只由 Theme::setState 表达（"ok" 绿 / "err" 红），不写色值（工单 03）
    Theme::setState(ui->lblPortState, ok ? "ok" : "err");
}

// 本页「WiFi 状态」标签的唯一写入点（工单 07）：与状态栏第二段同一条信号，
// 文案已经区分 TCP / UDP，这里只负责落地文字与红/绿。
void SerialPort::applyControlLinkState(const QString &text, bool ok)
{
    if (!text.isEmpty())
        ui->lblWifiState->setText(text);
    Theme::setState(ui->lblWifiState, ok ? "ok" : "err");
}

// 析构函数
SerialPort::~SerialPort()
{
    if(serialPort->isOpen()){
        serialPort->close();
    }

    TcpClient* tcpClient = TcpClient::getInstance();
    if (tcpClient->isConnected()) {
        tcpClient->disconnectFromHost();
    }

    if (udpSocket->state() == QAbstractSocket::BoundState) {
        udpSocket->close();
    }

    if (statsTimer && statsTimer->isActive()) {
        statsTimer->stop();
    }
    if (printTimer && printTimer->isActive()) {
        printTimer->stop();
    }
    if (statsPrintTimer && statsPrintTimer->isActive()) {
        statsPrintTimer->stop();
    }
    delete ui;
}

//
void SerialPort::setGLWidget(GLWidget* w)
{
    glWidget_ = w;
}
// 数据发送函数
void SerialPort::sendData(const QByteArray &data)
{
    int bytesSent = 0;
    TcpClient* tcpClient = TcpClient::getInstance();

    if(isSerialPortConnected){
        if (serialPort->isOpen()) {
            bytesSent = serialPort->write(data);
        } else {
            QMessageBox::warning(this, "警告", "串口未打开");
            return;
        }
    }else if (tcpClient->isConnected()) {
        qDebug() << "使用TCP发送测试数据...";
        bytesSent = tcpClient->sendData(data);
    } else if (isUdpBound) {
        qDebug() << "使用UDP发送测试数据...";
        QString ip = Config::instance().value("Network/tcp_ip", "127.0.0.1").toString();
        bool ok = false;
        quint16 port = Config::instance().value("Network/tcp_port", "6666").toString().toUShort(&ok);
        if (!ok) {
            // 处理转换失败，例如使用默认值
            port = 6666;
        }

        if (ip.isEmpty() || port == 0) {
            QMessageBox::warning(this, "警告", "请输入有效的目标IP和端口");
            return;
        }

        bytesSent = udpSocket->writeDatagram(data, QHostAddress(ip), port);
    } else{
        QMessageBox::warning(this, "警告", "请连接模块！");
        return;
    }

    // 发送字节计数并显示
    if(bytesSent > 0) {
        sendNum += bytesSent;
        QString sm = "发送字节数量： %1";
        QString revText = sm.arg(sendNum);
        ui->sendNum->setText(revText);
    }
}

// 发送按钮点击槽函数
void SerialPort::on_sendBt_clicked()
{
    // 使用协议工具类构建测试帧：串口上是 15 字节命令帧（AA 00 01 00×11 0A），
    // TCP / UDP 上是 4 字节（AA 00 01 0A）—— 帧形不同，子命令号都是 01。
    QByteArray testFrame = buildControlFrame(0x01);
    qDebug() << "发送测试数据中...."<< testFrame;
    sendData(testFrame);
    QString timestamp = QDateTime::currentDateTime().toString("HH:mm:ss >> 测试操作: ");
    ui->recvEdit->append(timestamp + "已发送测试，等待模块回复...");
    testFlag = false;       // 重置测试标志并启动定时器
    testTimer->start(3000); // 3秒超时
}

void SerialPort::onSendNavGoalRequested(double x, double y, double z, double yaw_deg)
{
    TcpClient* tcpClient = TcpClient::getInstance();
    QByteArray frame;

    if (isSerialPortConnected) {
        // --- 串口模式：调用 ProtocolRouter 构建二进制帧 (0x02) ---
        // 注意：由于 ProtocolRouter 接收的是 qint16，通常需要将米(m)和弧度(rad)转换为毫米(mm)和毫弧度(mrad)或相关单位。
        // 这里默认乘 100 转换为cm，请根据你的下位机实际通信协议修改比例系数！
        qint16 val_x = static_cast<qint16>(x * 100);
        qint16 val_y = static_cast<qint16>(y * 100);
        qint16 val_z = static_cast<qint16>(z * 100);
        qint16 val_yaw = static_cast<qint16>(yaw_deg);

        frame = ProtocolRouter::instance()->buildNavGoalFrame(val_x, val_y, val_z, 0, 0, val_yaw);

    } else if (tcpClient->isConnected()) {
        // --- TCP 模式：调用 ProtocolRos3D 构建 JSON 帧 (0x07) ---
        // yaw 发送整数度数
        int yaw_int = static_cast<int>(std::round(yaw_deg));
        frame = ProtocolRos3D::buildNavGoalFrame(x, y, z, yaw_int);

    } else {
        QMessageBox::warning(this, "错误", "请先建立 TCP 联网或打开串口！");
        return;
    }

    // 复用 SerialPort 现成的 sendData 进行发送
    sendData(frame);
    qDebug() << "成功发送导航目标点数据:" << frame.toHex(' ').toUpper();
}

// 保存地图 /////////////////////////////////////////////////////////////////
// 回包规矩（upper/config/通讯协议.md 消息表）：模块在产物核对通过之后才原样回发 AA 00 02 0A，
// 且只回成功 —— 核对不过 / 服务不可用 / 定位模式下被拒，都是一个字节都不回。
// 所以「没收到回包」= 失败，不等于「链路断了」；两者的区别只能靠 AA 00 01 0A 另判。
int SerialPort::saveMapTimeoutMs() const
{
    // 3D 30 秒 / 2D 10 秒（回显在产物核对之后才发，不能按「立刻回」判）。
    // 0x99 还没读到过时用 3D 那个更宽的。
    const bool is2D = (latestModuleType == 2 || latestModuleType == 3);
    const int defaultMs = is2D ? 10000 : 30000;
    const QString key = is2D ? "Protocol/save_map_timeout_2d_ms"
                             : "Protocol/save_map_timeout_3d_ms";
    bool ok = false;
    const int ms = Config::instance().value(key, defaultMs).toInt(&ok);
    return (ok && ms > 0) ? ms : defaultMs;
}

// 等待框：一个实例反复用（父子关系管生命周期），每次都从「等待中」的形态出现
void SerialPort::showSaveMapWait()
{
    if (!saveMapWait_) {
        // 挂在主窗口上：串口页只是栈里的一页（而且此刻多半藏在可视化页后面），
        // 按它居中会偏。
        saveMapWait_ = new WaitDialog(QStringLiteral("保存地图"),
                                      QStringLiteral("正在保存地图，等待模块回包…"),
                                      window());
    }
    saveMapWait_->show();
}

void SerialPort::closeSaveMapWait()
{
    // 用 accept() 而不是 close()：close() 会走 closeEvent，而那个框本来就挡着关闭按钮
    if (saveMapWait_) saveMapWait_->accept();
}

void SerialPort::onSaveMapRequested()
{
    // 与按钮的可用条件一致（同一条 controlLinkStateChanged）：TCP / UDP / 串口三条链路任一即可。
    // 串口命令面里 `02` 就是保存地图，行为与 TCP 那一份由同一段实现保证。
    if (!canSendControlCommands()) {
        QMessageBox::warning(this, "警告", "请先连接模块！");
        return;
    }

    const int timeoutMs = saveMapTimeoutMs();

    // 点下去就出现模态等待框，不给取消：模块侧已经在落盘了，「取消」在这里没有真实语义。
    // 关框与结果提示由 onSaveMapAckReceived / onSaveMapTimeout 负责。
    showSaveMapWait();

    sendData(buildControlFrame(0x02));

    // 上一轮补发的通信测试若还没出结果，这一轮重新计时
    saveMapProbePending = false;
    testTimer->stop();
    saveMapTimer->start(timeoutMs);
}

void SerialPort::onSaveMapAckReceived()
{
    const bool wasPending = saveMapTimer->isActive() || saveMapProbePending;

    saveMapTimer->stop();
    if (saveMapProbePending) {
        // 补发的通信测试还在等，回包先到 —— 既然后到的是成功，测试的计时作废
        saveMapProbePending = false;
        testTimer->stop();
        testFlag = true;
    }

    // 先关等待框再弹结果：两个都是模态的，叠在一起会先关掉下面那个
    closeSaveMapWait();

    if (wasPending) {
        ui->recvEdit->append(QDateTime::currentDateTime().toString("HH:mm:ss >> ") +
                             QStringLiteral("保存地图成功"));

        QMessageBox box(QMessageBox::Information, QStringLiteral("保存地图成功"),
                        QStringLiteral("模块已回包：产物核对通过。"),
                        QMessageBox::Ok, window());
        box.exec();
    }
}

void SerialPort::onSaveMapTimeout()
{
    closeSaveMapWait();

    // 补发一次通信测试：复用 buildControlFrame(0x01) 与 testFrameReceived → handleTestFrame
    // 这条链路（帧形跟链路走，串口上是 15 字节），结果由 handler 里的 saveMapProbePending 分流。
    // 这里不再单独写一行「超时」日志——这次超时的结论只有一句「保存地图失败」，
    // 由补发的那次测试收尾时写出。
    saveMapProbePending = true;
    testFlag = false;
    sendData(buildControlFrame(0x01));
    testTimer->start(3000);

    // 弹窗只给不承诺原因的结论：模块失败时是沉默的（定位模式下必然被拒也不回显），
    // 上位机分不出「存图失败」与「链路已断」。一个自信但可能错误的结论比没有结论更糟。
    QMessageBox box(QMessageBox::Warning, QStringLiteral("保存地图"),
                    QStringLiteral("未收到模块回包，保存结果未知。"),
                    QMessageBox::Ok, window());
    box.exec();
}

// 写入参数 /////////////////////////////////////////////////////////////////
// 参数页把「要写入的参数表」交过来，由链路层统一发送：逐条 AA 10 <参数号> <值>，
// 最后补一条保存终结指令 AA 00 04 0A。发送放在这里（而不是参数页里），
// 是为了让发送方与等待方同处一地——超时、断链、门禁的判断只有一处。
//
// 回包规矩（upper/config/通讯协议.md 消息表）：模块收到 AA 00 04 0A 后写入参数文件、
// **从盘上重读一次**，然后把**全表 21 条 AA 10** 连同本帧**原样回发**，**不重启算法**。
// 所以「保存成功没有」的唯一依据是：回读值逐条等于下发值。模块对表外参数号整条静默丢弃、
// 对已知号的越界值也不回错——不比对的话，填错值、发错号都只会得到一句「成功」。
void SerialPort::onWriteParamsRequested(const QVector<ParamWrite> &params)
{
    // 与按钮的可用条件一致：TCP / UDP / 串口任一即可 —— 串口上有参数帧 0x10 与保存子命令 04
    if (!canSendControlCommands()) {
        QMessageBox::warning(this, "警告", "请先连接模块！");
        return;
    }

    // 上一次的等待还挂着（模态框挡着，正常到不了这里）：先收干净，别让两次等待搅在一起
    if (saveParamsPending) {
        saveParamsTimer->stop();
        saveParamsPending = false;
        closeSaveParamsWait();
        emit paramEchoSuppressedChanged(false);
    }

    // 读等待若也还挂着（同上，正常到不了这里）：一并收干净。
    // 保存的回读同样是 AA 10，留着它会把回读帧算进那一次读取的进度里，
    // 于是「收齐 21 条」提前达成、弹出一个来路不明的「接收成功」。
    if (readParamsPending) {
        readParamsTimer->stop();
        readParamsPending = false;
        closeReadParamsWait();
    }

    // 比对基准就是这份列表本身：参数页在「点击那一刻」从界面收集，等待期用户再改界面也
    // 动不到它（见 ParamWrite 的注释）。
    saveParamsSnapshot = params;
    saveParamsReadback.clear();
    saveParamsPending = true;

    // 等待期模块回读的值只喂比对器，不刷界面：否则用户会在结果弹出前就以为保存成功了，
    // 而那正是这一步要抓的场景。
    emit paramEchoSuppressedChanged(true);

    // 点下去就出现模态等待框，不给取消：模块侧已经在落盘了，「取消」在这里没有真实语义。
    // 关框与结果提示由 onSaveParamsAckReceived / onSaveParamsTimeout 负责。
    showSaveParamsWait();

    for (const ParamWrite &p : params) {
        sendData(buildParamFrame(p.id, p.value));
    }

    // 保存终结指令：模块收到它才落盘（并回发全表参数值，下面等这个）
    sendData(buildControlFrame(0x04));

    const int timeoutMs = saveParamsTimeoutMs();
    saveParamsTimer->start(timeoutMs);
}

// 模块下发的每条 AA 10：0x99 模块类型与 0x13「模式」照常缓存；保存等待期把它按参数号存进比对器；
// 读取等待期按参数号计进度、收满条数就收尾。
void SerialPort::onModuleParameterFrame(quint8 paramId, qint16 value)
{
    if (paramId == 0x99) latestModuleType = value;
    if (paramId == 0x13) setModuleMode(value);   // 图形层判建图 / 定位（工单 03）

    if (saveParamsPending) {
        saveParamsReadback.insert(paramId, value);
    }

    // 读取参数：模块对 AA 00 03 0A 的应答是「按表序发全表 AA 10，没有终结帧」，
    // 所以进度的分子就是**收到过多少个不同的参数号**——不按到达帧数算：
    // 同一条参数回两遍会让进度虚高，停在 21/21 上等一个永远不会再来的第 22 条。
    if (readParamsPending) {
        if (!readParamsReceived.contains(paramId)) {
            readParamsReceived.insert(paramId, value);
            if (readParamsWait_) {
                readParamsWait_->setProgress(readParamsReceived.size(), readParamsExpected);
                readParamsWait_->setMessage(
                    QStringLiteral("正在接收参数：已收 %1 / %2 条…")
                        .arg(readParamsReceived.size()).arg(readParamsExpected));
            }
        }

        if (readParamsReceived.size() >= readParamsExpected) {
            settleReadParamsSuccess();
        }
    }
}

// 参数 0x13「模式」的落地处（工单 03）：缓存 + 只在变化时下发图形层。
// 判据取这个参数本身，**不**从 0x02 的数值去猜 —— 恒等不等于建图，将来 T ≠ 0 就会猜错。
// -1 = 本会话还没读到过任何一次 0x13（模块只在被读取参数时才下发 AA 10）；它必须与
// 「建图（0）」分开：默认按 0 处理，图形层会把「不知道」当成「模块正在建图」。
// 这里只送值，怎么用（要不要画开机点标记）由图形层的工单 09 决定。
void SerialPort::setModuleMode(int mode)
{
    if (mode == latestModuleMode) return;

    latestModuleMode = mode;
    emit moduleModeChanged(latestModuleMode);
}

// 终结指令到 = 回读已发完。这里才下结论，且必须是「按参数号」逐条比：
// 模块的回发顺序与发送顺序不同，按到达顺序比会满屏误报。
// 注意此刻所有回读帧都已到达——终结指令是模块发完 21 条 AA 10 之后才发的，
// 同一条 TCP 流、同一个线程，顺序不会反。
void SerialPort::onSaveParamsAckReceived()
{
    if (!saveParamsPending) {
        // 没有未确认的保存请求（没人点过按钮）：不弹框、也不写日志
        return;
    }

    saveParamsTimer->stop();
    saveParamsPending = false;
    emit paramEchoSuppressedChanged(false);

    // 先关等待框再弹结果：两个都是模态的，叠在一起会先关掉下面那个
    closeSaveParamsWait();

    QStringList details;          // 不一致明细（弹窗与日志共用同一份）
    for (const ParamWrite &p : saveParamsSnapshot) {
        const QString name = p.name.isEmpty()
                ? QStringLiteral("(无名称)") : p.name;
        // 0x 前缀与十六进制都按协议文档的写法保持小写（arg 的 base 16 本就出小写，
        // 这里不要 toUpper：那会把 "0x" 变成 "0X"，与文档和参数表的写法对不上）
        const QString head = QString("0x%1 %2")
                .arg(p.id, 2, 16, QLatin1Char('0')).arg(name);

        if (!saveParamsReadback.contains(p.id)) {
            // 表内参数号模块一条都没回：这份值没进模块（表外号整条静默丢弃，就是这个样子）
            details << QString("%1：上位机 %2 → 模块未返回").arg(head).arg(p.value);
            continue;
        }
        const qint16 got = saveParamsReadback.value(p.id);
        if (got != p.value) {
            details << QString("%1：上位机 %2 → 模块 %3").arg(head).arg(p.value).arg(got);
        }
    }

    if (details.isEmpty()) {
        settleSaveParams(
            QStringLiteral("参数保存成功"),
            QString("参数已保存（%1 项一致）").arg(saveParamsSnapshot.size()),
            QStringLiteral("保存参数"),
            QStringLiteral("参数已保存，请重启算法使参数生效。"),
            QString(), QMessageBox::Information);
        return;
    }

    // 逐条明细只进弹窗：日志窗口按「一行一个结论」的排版走，明细留在弹窗里给用户当场看
    settleSaveParams(
        QStringLiteral("保存参数失败"),
        QString("参数保存有出入（%1 项）").arg(details.size()),
        QStringLiteral("保存参数"),
        QStringLiteral("参数保存结果与下发值不一致，共 %1 项。"
                       "下列参数在模块里的值不是你下发的值：").arg(details.size()),
        details.join("\n"), QMessageBox::Warning);
}

// 超时：回读没集齐、终结指令也没来。模块侧是本地文件操作，超时即意味着链路上出了事——
// 但上位机分不出是哪种，所以只报事实，不承诺原因（与存图同一条规矩）。
void SerialPort::onSaveParamsTimeout()
{
    saveParamsPending = false;
    emit paramEchoSuppressedChanged(false);
    closeSaveParamsWait();

    const int timeoutMs = saveParamsTimeoutMs();

    settleSaveParams(
        QStringLiteral("保存参数失败"),
        QString("参数保存超时：%1 秒内未收到终结指令").arg(timeoutMs / 1000.0, 0, 'f', 1),
        QStringLiteral("保存参数"),
        QStringLiteral("未在 %1 秒内收到模块回读全表参数与保存终结指令 AA 00 04 0A，"
                       "本次保存结果未知。").arg(timeoutMs / 1000.0, 0, 'f', 1),
        QString("已收到 %1/%2 条参数回读。")
            .arg(saveParamsReadback.size()).arg(saveParamsSnapshot.size()),
        QMessageBox::Critical);
}

// 等待框：一个实例反复用（父子关系管生命周期），每次都从「等待中」的形态出现
void SerialPort::showSaveParamsWait()
{
    if (!saveParamsWait_) {
        // 挂在主窗口上：参数页只是栈里的一页（此刻多半藏在可视化页后面），按它居中会偏
        saveParamsWait_ = new WaitDialog(QStringLiteral("保存参数"),
                                         QStringLiteral("正在保存参数，等待模块回读全表参数…"),
                                         window());
    }
    saveParamsWait_->show();
}

void SerialPort::closeSaveParamsWait()
{
    // 用 accept() 而不是 close()：close() 会走 closeEvent，而那个框本来就挡着关闭按钮
    if (saveParamsWait_) saveParamsWait_->accept();
}

int SerialPort::saveParamsTimeoutMs() const
{
    // 模块侧只是本地文件操作，超过这个数还没回完就是链路上出事了。默认 10 秒。
    const int defaultMs = 10000;
    bool ok = false;
    const int ms = Config::instance().value("Protocol/save_params_timeout_ms", defaultMs)
                       .toInt(&ok);
    return (ok && ms > 0) ? ms : defaultMs;
}

// 收尾：摘要落在日志与参数页的状态行上，再弹一个结果框。
// 三态（一致 / 有出入 / 超时）共用这一段，免得「关框—记日志—弹窗」三处的顺序各写一遍。
// 日志只写一句结论（参数保存成功 / 保存参数失败），明细与耗时说明留在弹窗里。
void SerialPort::settleSaveParams(const QString &logLine, const QString &summary,
                                  const QString &title, const QString &text,
                                  const QString &detail, QMessageBox::Icon icon)
{
    const QString timestamp = QDateTime::currentDateTime().toString("HH:mm:ss >> ");
    ui->recvEdit->append(timestamp + logLine);

    emit saveParamsFinished(summary);

    QMessageBox box(icon, title, text, QMessageBox::Ok, window());
    if (!detail.isEmpty()) box.setInformativeText(detail);
    box.exec();
}

// 获取参数 /////////////////////////////////////////////////////////////////
// 参数页点了「获取参数」：由这里发 AA 00 03 0A、逐条收 AA 10、收满条数才下结论。
// 发送放在这里（而不是参数页里）与「写入参数」同一条理由：发送方与等待方同处一地，
// 超时、断链、门禁的判断只有一处；参数页只报出「它的表里有多少条」这一个数。
//
// 回包规矩（upper/config/通讯协议.md 消息表）：模块收到 AA 00 03 0A 后**按表序**把全表
// 21 条 AA 10 发回来，**发完没有任何终结帧**（这一点与保存参数的 AA 00 04 0A 不同）。
// 所以完成判据只能是「按参数号收满了多少条」，分母取参数页点击那一刻报来的条数
// （它的表 21 条，模块的表也是 21 条）。模块不回、或只回一半时是沉默的，
// 上位机分不出「读取失败」与「链路已断」，只按超时判、并写明收到了几条。
void SerialPort::onReadParamsRequested(int expectedCount)
{
    // 与按钮的可用条件一致：TCP / UDP / 串口任一即可 —— 串口命令面 `03` 就是读参数，
    // 应答是 22 条参数帧 0x10（TCP 侧形状是 6 字节，串口侧是 15 字节，解析同一段）
    if (!canSendControlCommands()) {
        QMessageBox::warning(this, "警告", "请先连接模块！");
        return;
    }

    // 上一轮还挂着（模态框挡着，正常到不了这里）：先收干净，别让两次读取搅在一起
    if (readParamsPending) {
        readParamsTimer->stop();
        readParamsPending = false;
        closeReadParamsWait();
    }

    // 分母取参数页报来的条数。为 0（表没建起来）时兜到 1：一个「0 条也算收齐」的读等待
    // 会当场弹成功，把「没读到任何东西」说成「接收成功」。
    readParamsExpected = qMax(1, expectedCount);
    readParamsReceived.clear();
    readParamsPending = true;

    // 点下去就出现模态进度框，不给取消（同保存参数那个框的取舍）：读参数没有副作用，
    // 但也没有「取消」的真实语义——模块那边已经在一条条发了，且回读值已经刷进参数页的界面，
    // 关掉框只会让这次读取变成一条没人认领的半截数据。
    showReadParamsWait();

    sendData(buildControlFrame(0x03));

    const int timeoutMs = readParamsTimeoutMs();
    readParamsTimer->start(timeoutMs);
}

// 收齐了：停表 → 关框 → 弹结果。
// 关框必须在弹结果之前：两个都是模态的，叠在一起会先关掉下面那个（同存图 / 存参数的次序）。
// 这条路径不写日志：读取参数只有「失败」会进日志窗口，成功由参数页的界面刷新和那个结果框交代。
void SerialPort::settleReadParamsSuccess()
{
    readParamsTimer->stop();
    readParamsPending = false;
    closeReadParamsWait();

    settleReadParams(
        QString(),
        QString("参数接收成功（%1 项）").arg(readParamsReceived.size()),
        QStringLiteral("获取参数"),
        QStringLiteral("参数接收成功。"),
        QString(), QMessageBox::Information);
}

// 超时：窗口内没收齐。模块失败时是沉默的，只报事实、不承诺原因（与存图 / 存参数同一条规矩）。
void SerialPort::onReadParamsTimeout()
{
    readParamsPending = false;
    closeReadParamsWait();

    const int got = readParamsReceived.size();
    const int timeoutMs = readParamsTimeoutMs();

    settleReadParams(
        QStringLiteral("读取参数失败"),
        QString("参数接收超时（%1/%2 条）").arg(got).arg(readParamsExpected),
        QStringLiteral("获取参数"),
        QStringLiteral("未在 %1 秒内收齐全部参数，本次只收到 %2/%3 条。")
            .arg(timeoutMs / 1000.0, 0, 'f', 1).arg(got).arg(readParamsExpected),
        QStringLiteral("收到的那 %1 条已经刷新到界面上了，其余各行保持原样。").arg(got),
        QMessageBox::Warning);
}

// 等待框：复用 04 的组件 + setProgress（与下载地图同一种「带进度」形态），实例反复用。
// 差在**不给取消**：下载可以丢掉已收分块从头再来，这里收到的 AA 10 已经逐条刷进界面了。
void SerialPort::showReadParamsWait()
{
    if (!readParamsWait_) {
        // 挂在主窗口上：参数页只是栈里的一页（此刻多半藏在可视化页后面），按它居中会偏
        readParamsWait_ = new WaitDialog(QStringLiteral("获取参数"),
                                         QStringLiteral("正在接收参数…"),
                                         window());
    }

    readParamsWait_->setProgress(0, readParamsExpected);
    readParamsWait_->setMessage(QStringLiteral("正在接收参数：已收 0 / %1 条…")
                                    .arg(readParamsExpected));
    readParamsWait_->show();
}

void SerialPort::closeReadParamsWait()
{
    // 用 accept() 而不是 close()：close() 会走 closeEvent，而那个框本来就挡着关闭按钮
    if (readParamsWait_) readParamsWait_->accept();
}

int SerialPort::readParamsTimeoutMs() const
{
    // 模块侧只是把内存里的全表读出来发一遍（不做落盘、不做重读），比保存参数还轻。
    // 默认 10 秒与保存参数同值：这么久还没收齐就是链路上出事了。走 config.ini。
    const int defaultMs = 10000;
    bool ok = false;
    const int ms = Config::instance().value("Protocol/read_params_timeout_ms", defaultMs)
                       .toInt(&ok);
    return (ok && ms > 0) ? ms : defaultMs;
}

// 收尾：摘要落在日志与参数页的状态行上，再弹一个结果框。
// 两态（收齐 / 超时）共用这一段，免得「关框—记日志—弹窗」三处的顺序各写一遍。
void SerialPort::settleReadParams(const QString &logLine, const QString &summary,
                                  const QString &title, const QString &text,
                                  const QString &detail, QMessageBox::Icon icon)
{
    // 成功那条路的 logLine 是空的（读取参数不记成功行），别在窗口里留一行只剩时间戳的空行
    if (!logLine.isEmpty()) {
        const QString timestamp = QDateTime::currentDateTime().toString("HH:mm:ss >> ");
        ui->recvEdit->append(timestamp + logLine);
    }

    emit readParamsFinished(summary);

    QMessageBox box(icon, title, text, QMessageBox::Ok, window());
    if (!detail.isEmpty()) box.setInformativeText(detail);
    box.exec();
}

// 下载地图 /////////////////////////////////////////////////////////////////
// 回包规矩（upper/config/通讯协议.md）：上位机发 AA 00 05 0A，模块把盘上那份地图打包、
// 按固定字节数切块，回 N 帧 AA 0C；**发完没有任何终结帧**，所以完成判据只能是
// 「收齐了 total 块」。模块打不出包 / 缺件时**一帧都不发**，与存图、存参数同一条
// 「失败即沉默」的规矩——上位机分不出「下传失败」与「链路已断」，文案不承诺原因。
//
// 与存图那套的区别只有两点：等待框带进度、可取消；超时分两级。
int SerialPort::downloadChunkTimeoutMs() const
{
    // 块间静默默认 5 秒：卡在第 3 块不该和第 28 块等一样久
    const int defaultMs = 5000;
    bool ok = false;
    const int ms = Config::instance().value("Protocol/download_map_chunk_timeout_ms", defaultMs)
                       .toInt(&ok);
    return (ok && ms > 0) ? ms : defaultMs;
}

int SerialPort::downloadTotalTimeoutMs() const
{
    // 整包总超时默认 120 秒：兜底，不让整包无限期挂着
    const int defaultMs = 120000;
    bool ok = false;
    const int ms = Config::instance().value("Protocol/download_map_total_timeout_ms", defaultMs)
                       .toInt(&ok);
    return (ok && ms > 0) ? ms : defaultMs;
}

QString SerialPort::downloadDefaultDir() const
{
    const QString configured =
        Config::instance().value("Protocol/download_map_dir").toString().trimmed();
    if (!configured.isEmpty()) return configured;

    // 没配就取可执行文件旁边的目录：现场最省事，落盘位置也可预期
    return QCoreApplication::applicationDirPath();
}

// 等待框：复用 04 的组件，扩出「带进度、可取消」的形态。
// 取消按钮点了就是「取消」（下载没有副作用），所以 cancelled 接进来就地丢弃。
void SerialPort::showDownloadWait()
{
    if (!downloadWait_) {
        // 挂在主窗口上：可视化页只是栈里的一页，按它居中会偏
        downloadWait_ = new WaitDialog(QStringLiteral("下载地图"),
                                       QStringLiteral("正在等待模块下发的分块…"),
                                       window());
        downloadWait_->setCancelEnabled(true);
        connect(downloadWait_, &WaitDialog::cancelled, this, &SerialPort::onDownloadCancelled);
    }

    // 总块数还没听模块说过，先回到不确定形态；收到第一块之后才有「已收 / 共」可显示
    downloadWait_->setMessage(QStringLiteral("正在等待模块下发的分块…"));
    downloadWait_->setProgress(-1, 0);
    downloadWait_->show();
}

void SerialPort::closeDownloadWait()
{
    // 用 accept() 而不是 close()：close() 会走 closeEvent，那条路是给「取消」用的
    if (downloadWait_) downloadWait_->accept();
}

void SerialPort::onDownloadMapRequested()
{
    // 这一条**只认 TCP / UDP**：串口的命令面是五条子命令（01/02/03/04/06），**没有 05**。
    // 主控照 TCP 的样子发 `AA 00 05` 会被模块静默丢弃，上位机只会白等一个超时 ——
    // 所以宁可当场说清楚，也不要让它发出去。
    if (!canDownloadMap()) {
        QMessageBox::warning(
            this, "警告",
            isSerialPortConnected
                ? "串口不支持下载地图（整包约 1.9 MB，115200 下要 165 秒起）。\n"
                  "请改用 TCP / UDP 连接。"
                : "请先连接模块！");
        return;
    }

    // 上一轮还挂着（模态框挡着，正常到不了这里）：先收干净，别让两次下载搅在一起
    if (downloadPending) {
        downloadChunkTimer->stop();
        downloadTotalTimer->stop();
        downloadPending = false;
        closeDownloadWait();
    }

    downloadChunks.clear();
    downloadPkgName.clear();
    downloadTotalChunks = 0;
    downloadPending = true;

    // 点下去就出现模态进度框，可以取消（下载没有副作用，取消就是取消）
    showDownloadWait();

    sendData(ProtocolRouter::instance()->buildDownloadMapFrame());

    const int chunkMs = downloadChunkTimeoutMs();
    const int totalMs = downloadTotalTimeoutMs();
    downloadChunkTimer->start(chunkMs);
    downloadTotalTimer->start(totalMs);

    ui->recvEdit->append(QDateTime::currentDateTime().toString("HH:mm:ss >> 用户操作: ") +
                         QStringLiteral("下载地图"));
}

// 模块下传的每一块（0x0C）。攒块与校验不在这里做——本函数只管收、报进度、判「收齐了」；
// 拼装与校验整包交给 MapArchive（纯函数模块，工单 09）。
void SerialPort::onMapChunkReceived(const MapChunkMsg &msg)
{
    // 没在下载时到的分块一律丢弃：取消或超时之后，模块往往还会把剩下的块发完
    if (!downloadPending) return;

    downloadChunks.append(msg);
    if (msg.total > 0) downloadTotalChunks = msg.total;
    if (!msg.name.isEmpty()) downloadPkgName = msg.name;

    const int total = downloadTotalChunks;
    if (downloadWait_) {
        downloadWait_->setProgress(downloadChunks.size(), total);
        downloadWait_->setMessage(
            total > 0
                ? QStringLiteral("正在接收地图分块：已收 %1 / %2 块…")
                      .arg(downloadChunks.size()).arg(total)
                : QStringLiteral("正在接收地图分块：已收 %1 块…").arg(downloadChunks.size()));
    }

    // 每收到一块就重置静默计时：量的是「这一块之后又静了多久」
    downloadChunkTimer->start(downloadChunkTimeoutMs());

    // 完成判据是「收齐了总块数」，不是等某个收尾指令——模块发完没有任何终结帧。
    // 块数够不等于块都对（可能有重复块顶替了缺块），那一类交给下面的校验去挡。
    if (total > 0 && downloadChunks.size() >= total) {
        onDownloadComplete();
    }
}

// 收齐了：先校验，通过才问路径，最后落盘。
void SerialPort::onDownloadComplete()
{
    downloadChunkTimer->stop();
    downloadTotalTimer->stop();
    downloadPending = false;

    // 先关等待框：后面还有两个模态框（文件对话框、结果提示），叠在一起会先关掉下面那个
    closeDownloadWait();

    const MapArchive::Result result = MapArchive::assembleMapArchive(downloadChunks);
    downloadChunks.clear();   // 无论成败都不再留着：坏包（result.package 为空）尤其不能

    const QString timestamp = QDateTime::currentDateTime().toString("HH:mm:ss >> ");

    // 校验失败：长度或 CRC 不符。坏包不能伪装成好包躺在用户的目录里，所以不落盘、直接报错。
    // 日志只留一句结论，是哪种不符（长度 / CRC / 缺块…）在弹窗里说。
    if (!result.ok()) {
        ui->recvEdit->append(timestamp + QStringLiteral("地图下载失败"));

        QMessageBox box(QMessageBox::Critical, QStringLiteral("地图下载失败"),
                        QStringLiteral("收到的地图包校验未通过，已丢弃，未写入任何文件。"),
                        QMessageBox::Ok, window());
        box.setInformativeText(result.detail);
        box.exec();
        return;
    }

    // 到这里整包已通过长度与 CRC-32 双重校验，路径选择才发生——
    // 用户此刻拿到的预填文件名是模块自报的真实包名，不用去猜。
    const QString suggested =
        QDir(downloadDefaultDir())
            .filePath(downloadPkgName.isEmpty() ? QStringLiteral("map_archive.zip")
                                                : downloadPkgName);

    const QString path = QFileDialog::getSaveFileName(
        window(), QStringLiteral("保存地图包"), suggested,
        QStringLiteral("压缩包 (*.zip);;所有文件 (*)"));

    if (path.isEmpty()) {
        // 取消文件对话框：不落盘、也不报错，已下载数据丢弃。这不是失败，是用户主动放弃，
        // 所以一条日志都不写（失败那一行会把它说成一次下传失败）。
        return;
    }

    // 压缩包原样保存，不自动解压：包内文件名的语义归模块侧，上位机不解释。
    // 用 QSaveFile 而不是 QFile：写到一半失败不会在用户目录里留个半截包
    // （与「坏包不落盘」同一条规矩）。
    QSaveFile file(path);
    const bool written = file.open(QIODevice::WriteOnly)
                         && file.write(result.package) == result.package.size()
                         && file.commit();
    if (!written) {
        ui->recvEdit->append(timestamp + QStringLiteral("地图下载失败"));

        QMessageBox box(QMessageBox::Critical, QStringLiteral("地图下载失败"),
                        QStringLiteral("地图包已下载并通过校验，但写入文件失败，未落盘。"),
                        QMessageBox::Ok, window());
        box.setInformativeText(QStringLiteral("%1\n%2")
                                   .arg(QDir::toNativeSeparators(path), file.errorString()));
        box.exec();
        return;
    }

    const QString nativePath = QDir::toNativeSeparators(path);
    ui->recvEdit->append(timestamp +
                         QStringLiteral("地图下载成功：已保存到 %1").arg(nativePath));

    QMessageBox box(QMessageBox::Information, QStringLiteral("地图下载成功"),
                    QStringLiteral("地图包已保存到：\n%1").arg(nativePath),
                    QMessageBox::Ok, window());
    box.exec();
}

// 取消：下载没有副作用，取消就是取消——不落盘、不报错（所以不走 abortDownload），
// 也不写日志：用户自己按的取消，不是一次失败。
void SerialPort::onDownloadCancelled()
{
    downloadChunkTimer->stop();
    downloadTotalTimer->stop();
    downloadPending = false;

    downloadChunks.clear();

    // 关框不在这里做：cancelled() 是 WaitDialog 自己在关闭路径上发出来的，它随后就把自己关掉
}

// 失败收尾：停表、丢块、关框、记一行日志、弹一个结果框。
// 两条超时路径共用这一段，免得「关框—记日志—弹窗」的顺序各写一遍。
// 日志统一是那一句「地图下载失败」；是哪一种超时、卡在第几块，都由弹窗交代。
void SerialPort::abortDownload(const QString &logLine, const QString &text, const QString &detail)
{
    downloadChunkTimer->stop();
    downloadTotalTimer->stop();
    downloadPending = false;
    downloadChunks.clear();

    closeDownloadWait();

    const QString timestamp = QDateTime::currentDateTime().toString("HH:mm:ss >> ");
    ui->recvEdit->append(timestamp + logLine);

    QMessageBox box(QMessageBox::Warning, QStringLiteral("地图下载失败"), text,
                    QMessageBox::Ok, window());
    if (!detail.isEmpty()) box.setInformativeText(detail);
    box.exec();
}

// 块间静默超时：两块之间静了太久（含首块之前那段）。这样卡在第 3 块不会和第 28 块等一样久。
void SerialPort::onDownloadChunkTimeout()
{
    const int got = downloadChunks.size();
    const int total = downloadTotalChunks;
    const QString secs = QString::number(downloadChunkTimeoutMs() / 1000.0, 'f', 1);

    // 一块都没收到是最像「模块下传失败」的那一种（模块失败时一帧都不发），
    // 但链路断了在这条链上同样一帧都没有，所以文案不承诺原因。
    const QString text =
        got == 0
            ? QStringLiteral("未收到模块的任何分块，地图下载失败。")
            : QStringLiteral("分块传输中断，地图下载失败。");

    const QString detail =
        got == 0
            ? QStringLiteral("块间静默超时 %1 秒内一块都没收到。「模块下传失败」（打不出包 / 缺件）"
                             "时一帧都不发，但链路已断在这条链上同样一帧都没有，上位机无法区分。")
                  .arg(secs)
            : QStringLiteral("已收 %1%2 块。块间静默超时 %3 秒；"
                             "「模块下传失败」与「链路已断」上位机区分不了。")
                  .arg(got)
                  .arg(total > 0 ? QStringLiteral("/%1").arg(total) : QString())
                  .arg(secs);

    abortDownload(QStringLiteral("地图下载失败"), text, detail);
}

// 整包总超时：分块一直在来，但整包迟迟收不齐（模块那边卡住 / 链路极慢）。
void SerialPort::onDownloadTotalTimeout()
{
    const int got = downloadChunks.size();
    const int total = downloadTotalChunks;
    const QString secs = QString::number(downloadTotalTimeoutMs() / 1000.0, 'f', 0);

    abortDownload(
        QStringLiteral("地图下载失败"),
        QStringLiteral("未在总超时内收齐地图分块，地图下载失败。"),
        QStringLiteral("已收 %1%2 块。整包总超时 %3 秒；"
                       "「模块下传失败」与「链路已断」上位机区分不了。")
            .arg(got)
            .arg(total > 0 ? QStringLiteral("/%1").arg(total) : QString())
            .arg(secs));
}

// 重启算法 /////////////////////////////////////////////////////////////////
// 线上是一条控制帧 —— TCP 是 `AA 00 06 0A`（4 字节），串口是 `AA 00 06 00×11 0A`（15 字节，
// 子命令号在同一个位置）。帧内没有长度字段、没有校验字节、没有版本字节。
// 回包规矩：模块**先原样回发本帧、再执行关机**。所以「收到回发」= 模块已受理重启，是判成功的
// 唯一依据；而「连接断开」是另一件、后发生的事，本功能不去推断它（那会依赖模块是否发出 TCP
// FIN / 串口是否掉线，属模块侧实现细节，不可依赖）。超时 = 未收到回发，结果未知 —— 不编造原因。
//
// 门禁：TCP / UDP / **串口**三条链路都行（串口命令面的五条子命令里就有 `06`，
// 协议原文点名「回显由串口侧的 pose_send 就地发出」），与「保存地图」「读/写参数」同一条门禁。
// 不设管理权限门禁，对所有用户开放。
//
// 提示策略：**全程只有一次弹窗** —— 收到回发后那句「模块已收到重启命令」的回执。
// 按下不二次确认（按下即发）；超时不弹框，只在后台把按钮恢复可用。
// 收/发过程在接收窗口留两行：按下时的「用户操作: 重启算法」，收尾时的成功或失败。
void SerialPort::setLinkConnected(bool connected)
{
    controlLinkConnected_ = connected;
    updateRestartButton();
}

// 控制指令门禁的**唯一判定处**（构建侧）。三条链路任一在，控制指令就发得出去：
// TCP 已连接 / UDP 已绑定 / 串口已打开。
bool SerialPort::canSendControlCommands() const
{
    TcpClient* tcpClient = TcpClient::getInstance();
    return isSerialPortConnected || tcpClient->isConnected() || isUdpBound;
}

// 下传地图只认 TCP / UDP：串口的命令面是**五条**子命令（01/02/03/04/06），**没有 05**。
// 主控若照 TCP 的样子发 `AA 00 05`，模块静默丢弃（不回帧、不打日志），上位机只会白等超时。
bool SerialPort::canDownloadMap() const
{
    TcpClient* tcpClient = TcpClient::getInstance();
    return tcpClient->isConnected() || isUdpBound;
}

// 两道门禁的唯一发射点：任何会改变链路状态的地方（TCP 连上/断开、UDP 绑定/关闭、
// 串口打开/关闭）都调它，别在别处再拼一遍布尔值 —— 三条链路 × 两种判断，
// 散着写迟早出现「按钮亮了但发不出去」。
void SerialPort::emitCommandLinkGates()
{
    emit controlLinkStateChanged(canSendControlCommands());
    emit mapDownloadLinkChanged(canDownloadMap());
}

// 控制指令帧：串口 15 字节定长帧，TCP / UDP 4 字节短帧。**只在构建时分链路** ——
// 解析侧（ProtocolRouter::handleControlFrame）两种长度都认，子命令位置相同。
QByteArray SerialPort::buildControlFrame(quint8 subCmd) const
{
    if (isSerialPortConnected) return ProtocolRouter::buildUartCommandFrame(subCmd);

    QVariantMap params;
    params["sub_cmd"] = QVariant(uint(subCmd));
    return ProtocolRouter::buildFrame(0x00, params);
}

// 参数帧：串口 15 字节（`AA 10 <id> <vH> <vL> 00×9 0A`），TCP / UDP 6 字节。
QByteArray SerialPort::buildParamFrame(quint8 paramId, qint16 value) const
{
    return isSerialPortConnected ? ProtocolRouter::buildUartParamFrame(paramId, value)
                                 : ProtocolRouter::buildWriteParamFrame(paramId, value);
}

// 「重启算法」可用性的唯一判定处，两条规则都写在这里：
//   1. 指令链路在（TCP 已连接 / UDP 已绑定 / **串口已打开**）—— 由 controlLinkStateChanged 喂进来；
//   2. 不在等待回发的窗口内 —— 模块已经准备重启了，不该再发第二条重启指令。
// （串口早年不承载控制指令时这里还有一条「不在串口模式」，协议 v2.9 起串口有命令帧 0x00，
// 那一条已经不成立了，别再往回加。）
void SerialPort::updateRestartButton()
{
    ui->restartBt->setEnabled(controlLinkConnected_ && !restartPending);
}

void SerialPort::on_restartBt_clicked()
{
    // 与按钮的可用条件一致：TCP / UDP / 串口任意一条在就行（串口命令面第五六条子命令）
    if (!canSendControlCommands()) {
        QMessageBox::warning(this, "警告", "请先连接模块！");
        return;
    }

    const bool viaSerial = isSerialPortConnected;
    TcpClient* tcpClient = TcpClient::getInstance();

    // 按下即发，**没有二次确认**：本功能的弹窗只有一次，就是收到回发后那句回执（见
    // onRestartAckReceived）。指令确实发出去了这件事由下面第一行日志交代。

    // 1. 先让操作员在日志里看到「指令确实发出去了」——这是三个可分辨事件里的第一个
    QString timestamp = QDateTime::currentDateTime().toString("HH:mm:ss >> 用户操作: ");
    ui->recvEdit->append(timestamp + QStringLiteral("重启算法"));

    // 2. 发出重启帧。走 TCP 时发帧之前先关掉自动重连（工单 03）：模块收到即断电，断线随时会来，
    //    开关必须先于那一帧在位。**成功回调里不解除** —— 回显早于断电到达，图省事在成功回调里
    //    解除的话，断电那一刻自动重连会照常触发（必然失败），本功能就静默失效了。
    //    一直压到用户自己点「打开连接」为止：万一模块重启得慢、或者回发丢了但其实断电了，
    //    那一刻也依然被正确压制。
    //    串口这条链没有「自动重连」这回事（口开着自己就在），所以只在非串口时动那把开关。
    if (!viaSerial)
        tcpClient->setAutoReconnectEnabled(false);
    sendData(buildControlFrame(0x06));

    // 3. 进入等待窗口：按钮置灰挡住第二条重启指令（等待期断开链路也由门禁接住）
    restartPending = true;
    updateRestartButton();
    restartTimer->start(RESTART_ACK_TIMEOUT_MS);
}

// 窗口内收到模块原样回发 → 成功。除了反馈，还要清掉界面上那些「模块运行态的镜像」（工单 02）。
void SerialPort::onRestartAckReceived()
{
    const bool wasPending = restartPending;

    restartPending = false;
    restartTimer->stop();
    updateRestartButton();

    // 没有未确认的请求（没人点过按钮）时不弹、也不写日志：那时用户并没有在等，
    // 这帧回显是「无源之水」，那次重启并不成立。
    // 清理同样挂在这个条件上 —— 顺手清掉界面就是在毁掉用户正在看的东西。
    if (wasPending) {
        ui->recvEdit->append(QDateTime::currentDateTime().toString("HH:mm:ss >> ") +
                             QStringLiteral("重启算法成功"));

        // 判定成功 = 模块即将变成一个「什么都没见过」的新模块，界面上那些运行态镜像
        // 描述的是重启前那个已经不复存在的模块，此刻一并清掉（工单 02）。
        // 失败路径（onRestartTimeout）一条都不清 —— 超时意味着模块很可能根本没重启。
        clearModuleRuntimeCaches();     // 本页：位姿 / 模块类型 / 模式
        emit restartSucceeded();        // 3D 页面：TF 标签、状态标签与锁存（含 3D 视图本身）

        QMessageBox box(QMessageBox::Information, QStringLiteral("重启算法"),
                        QStringLiteral("模块已收到重启命令。"),
                        QMessageBox::Ok, window());
        box.exec();
    }
}

// 重启判定成功后清掉本页缓存的运行态镜像（工单 02）。
// 这些值描述的是重启前那个模块：不清的话，每秒一次的打印定时器（0x01/0x03）
// 会继续把旧位姿当「当下值」吐到接收窗口，看起来像模块还活着。
// 只清本页的缓存；参数页界面上那批值一律不动 —— 那是用户给模块的配置，不是模块的运行输出。
void SerialPort::clearModuleRuntimeCaches()
{
    // 位姿两路：0x01 当前位姿 / 0x03 位置控制指令。valid 归假，打印定时器随之闭嘴
    latestPose01 = PoseData();
    latestPose03 = PoseData();

    // 五件状态（0x05）同理：verdict 回初值并置 valid=false，重启回来之前不再打那一行，
    // 免得把重启前的飞控 / 资源读数当成「现在」的值继续报
    latestStatus05 = FlightStatus();

    // 模块类型（参数 0x99）：-1 = 没读到过。它决定存图超时取 3D 的宽档还是 2D 的窄档，
    // 重启后模块还没自报类型之前，按「不知道」宽档兜底才是安全的
    latestModuleType = -1;

    // 模式（参数 0x13）：同样归 -1 = 没读到过，并把这次「忘掉」下发给图形层。
    // 重启前那个值描述的是已经不复存在的旧模块，而重启后模式完全可能被改过；
    // 图形层手上那份副本要跟着回到「不知道」，否则它会拿着旧模式继续画标记（工单 09）
    setModuleMode(-1);
}

// 窗口超时：把等待状态收回来（按钮恢复可用），记一行「重启算法失败」，**不弹框** ——
// 本功能全程只有一次弹窗，就是收到回发时那句回执；超时不编造原因，模块是否真的断了、
// 链路是否还在，上位机在那一刻区分不了。
// **不执行任何清理**：超时意味着模块很可能根本没重启，界面上那些运行态镜像依然有效，
// 顺手清掉就是毁掉还在用的东西。
void SerialPort::onRestartTimeout()
{
    restartPending = false;
    updateRestartButton();

    ui->recvEdit->append(QDateTime::currentDateTime().toString("HH:mm:ss >> ") +
                         QStringLiteral("重启算法失败"));
}

// 数据处理/////////////////////////////////////////////////////////////////
// 数据处理
void SerialPort::processReceivedData(QByteArray &recBuf)
{

    recvNum += recBuf.size();
    ui->recvNum->setText(QString("接收字节数量： %1").arg(recvNum));

    if (isSerialPortConnected) {
        emit rawBytesArrived(recBuf,true);
    } else {
       emit rawBytesArrived(recBuf,false);
    }
}


void SerialPort::setupConnections()
{
    // 连接ProtocolRouter的测试帧信号
    connect(ProtocolRouter::instance(), &ProtocolRouter::testFrameReceived,
            this, &SerialPort::handleTestFrame);
}

void SerialPort::handleTestFrame(const QByteArray &frame, bool isResponse)
{
    if (isResponse) {
        // 测试响应
        testFlag = true;
        testTimer->stop();  // 停止超时计时器

        QString timestamp = QDateTime::currentDateTime().toString("HH:mm:ss >> ");

        // 这次测试是存图超时后自动补发的：回包只说明存图那一步失败了（链接是好的）。
        // 这次补发的探测不对应用户点过的那次「发送测试数据」，所以只收尾、不写测试那两行。
        if (saveMapProbePending) {
            saveMapProbePending = false;
            ui->recvEdit->append(timestamp + QStringLiteral("保存地图失败"));
            return;
        }

        ui->recvEdit->append(timestamp + "测试成功！收到模块响应");

        // 显示原始响应数据
        QString hexStr = frame.toHex(' ').toUpper();
    } else {
        // 这是其他设备发来的测试请求（不是对我们请求的响应）
        qDebug() << "收到测试请求，自动回复响应";

        // 自动构建测试响应帧：帧形跟链路走（串口 15 字节 / 网络 4 字节），
        // 子命令都是 01。别在这里另拼一份 —— 帧形的唯一出处是 buildControlFrame()。
        QByteArray response = buildControlFrame(0x01);

        // 回复对方
        sendData(response);
    }
}

// 超时处理
void SerialPort::onTestTimeout()
{
    if (!testFlag) {
        QString timestamp = QDateTime::currentDateTime().toString("HH:mm:ss >> ");

        // 存图超时后自动补发的测试也超时：结论一样是失败。
        // 这条是机器补发的探测，不弹模态框 —— 那个「测试超时」框会挡住日志窗口。
        if (saveMapProbePending) {
            saveMapProbePending = false;
            testFlag = false;
            ui->recvEdit->append(timestamp + QStringLiteral("保存地图失败"));
            return;
        }

        ui->recvEdit->append(timestamp + "测试失败！！！未收到模块响应");
        testFlag = false;

        // 可以在这里触发重试或其他错误处理
        QMessageBox::warning(this, "测试超时",
                             "未在3秒内收到模块响应，请检查：\n"
                             "1. 模块是否开机\n"
                             "2. 连接是否正常\n"
                             "3. 模块端协议是否正确");
    }
}


// 打开网络通信UDP/TCP//////////////////////////////////////////////////////////
// 打开网络通信UDP/TCP
void SerialPort::on_wifiConnectBt_clicked()
{
    // 根据选择的协议启用相应控件
    QString protocol = ui->protocolComboBox->currentText();
    TcpClient* tcpClient = TcpClient::getInstance();

    if(ui->wifiConnectBt->text() == "打开连接"){
        /* ---- 新增：清掉上一轮的绘制数据 ---- */
        if(glWidget_) glWidget_->clearMap();   // 与“清理地图”按钮同一套逻辑
        // 新会话：上一轮读到的参数 0x13「模式」同样作废 —— 模块可能已经被改成另一种模式，
        // 本轮读到 0x13 之前一律按「不知道」处理（与重启成功那条路同一个归位动作）
        setModuleMode(-1);
        // 清空输出窗口
        on_clearRecvBt_clicked();
        // 先停止可能的重连
        tcpClient->stopAutoReconnect();
        // 这里是「重启算法」那次压制的唯一解除处（工单 03）：用户手动点了「打开连接」，
        // 重启那一段到此结束 —— 从这里起再断线，照旧走原有的自动重连逻辑。
        // 判定成功的那条路不解除，这是一次性压制能否成立的分水岭。
        tcpClient->setAutoReconnectEnabled(true);
        // 重置'重连'警告标志
        m_reconnectWarningShown = false;
        // 禁用串口通信方式
        ui->serialBox->setEnabled(false);
        if (protocol == "TCP"){
            if (!tcpClient->isConnected()) {
                // --- 修改开始：获取输入框中的 IP 和端口 ---
                QString ipText = ui->ipInput->text().trimmed();
                bool ok = false;
                quint16 port = ui->portInput->text().toUShort(&ok);

                // 如果输入框为空（比如初次启动），则回退到配置文件
                if (ipText.isEmpty()) {
                    ipText = Config::instance().value("Network/tcp_ip", "127.0.0.1").toString();
                }
                if (!ok || port == 0) {
                    port = Config::instance().value("Network/tcp_port", "6666").toUInt();
                }

                qDebug() << "开始连接TCP..." << ipText << ":" << port;

                // 使用单例TCP客户端连接
                if (tcpClient->connectToHost(ipText, port)) {
                    // 连接成功，等待连接建立信号
                    // 不要在这里立即修改按钮文本，等待连接成功的信号
                    qDebug() << "TCP连接请求已发送，等待连接结果...";
                } else {
                    // 连接立即失败
                    QMessageBox::warning(this, "TCP错误", "连接失败！请检查ip/端口号！");
                    // 不修改按钮文本，保持"打开连接"
                }
            } else {
                QMessageBox::warning(this, "警告", "TCP连接已打开");
                // 启用统计定时器
                if (!statsTimer->isActive()) {
                    statsTimer->start();
                }
            }

        }else if(protocol == "UDP"){
            if (!isUdpBound) {
                // todo 待设置
                quint16 port = 12312;

                if (port == 0) {
                    QMessageBox::warning(this, "警告", "请输入有效的端口号");
                    return;
                }

                if (udpSocket->bind(port)) {
                    isUdpBound = true;
                    emitCommandLinkGates();
                    ui->wifiConnectBt->setText("关闭连接");
                    emit controlLinkTextChanged(QStringLiteral("UDP 已连接"), true);

                    // UDP连接成功时启动统计定时器
                    if (!statsTimer->isActive()) {
                        statsTimer->start();
                        elapsedTimer.restart();
                    }
                } else {
                    QMessageBox::warning(this, "错误", "UDP绑定失败");
                    // UDP绑定失败，不修改按钮文本
                }
            } else {
                QMessageBox::warning(this, "警告", "UDP连接已打开");
                // 启用统计定时器
                if (!statsTimer->isActive()) {
                    statsTimer->start();
                }
            }
        }
    }else if(ui->wifiConnectBt->text() == "关闭连接"){
        // 停止统计定时器
        if (statsTimer->isActive()) {
            statsTimer->stop();
        }
        ui->wifiConnectBt->setText("打开连接");
        ui->serialBox->setEnabled(true);
        if (protocol == "TCP"){
            // 手动关掉就是关掉（工单 04）：**先置位禁用、再断开**，顺序不能反。
            // 反过来（先断开、后禁用）在断线信号不是同步送达时会白做 —— 断开那一刻断线处理入口
            // 已经把重连定时器武装起来了，后来的禁用管不了它，连接于是自己连回来。
            // 复用 03 的同一把开关与设置接口，不引入第二套并行逻辑；解除处仍是「打开连接」那一支。
            tcpClient->setAutoReconnectEnabled(false);

            // 上面那把开关只管「断线之后」；若断开之前正处在重连窗口里（定时器已武装、还没连上），
            // 撤掉这一次的是 stopAutoReconnect() —— 所以它放在 isConnected() 判断**之外**：
            // 那种时刻还没连上，可按钮正写着「关闭连接」，用户点它就是要停掉这一次。
            tcpClient->stopAutoReconnect();

            if (tcpClient->isConnected()){
                tcpClient->disconnectFromHost();
                // 断开连接后按钮文本已经是"打开连接"
            } else {
                // 没连上就不会有断开信号，autoReconnectSuppressed 也就不来，标签得在这里归位：
                // 断线时 connectionStatusChanged(false) 那一支把它写成了「尝试自动重连」，而这一次
                // 一次都不会尝试 —— 留着就是在骗操作员（与工单 03 同一条规矩）。
                emit controlLinkTextChanged(QStringLiteral("未连接"), false);
            }
        }else if(protocol == "UDP"){
            if (isUdpBound) {
                udpSocket->close();
                isUdpBound = false;
                emitCommandLinkGates();

                ui->wifiConnectBt->setText("打开连接");
                emit controlLinkTextChanged(QStringLiteral("未连接"), false);
            }
        }

        // ui->serialBox->setEnabled(true);
    }
}



// 不重要功能和未使用功能///////////////////////////////////////////////////////
//寻找空闲状态串口
void SerialPort::findFreePorts(){
    QList<QSerialPortInfo> ports = QSerialPortInfo::availablePorts();
    for (const auto &port : QSerialPortInfo::availablePorts()) {
        if (!port.isBusy()) {
            ui->portNames->addItem(port.portName());
        }
    };
}

// 检测串口
void SerialPort::on_portSearchBt_clicked()
{
    ui->portNames->clear();
    QList<QSerialPortInfo> ports = QSerialPortInfo::availablePorts();
    for (const auto &port : QSerialPortInfo::availablePorts()) {
        if (!port.isBusy()) {
            ui->portNames->addItem(port.portName());
        }
    };
    if (!ports.size()){
        QMessageBox::warning(NULL,"提示",QStringLiteral("没有找到空闲的串口！"));
        return;
    };
}

// 打开串口
void SerialPort::on_portOpenBt_clicked()
{
    // qint32 baudRate = 921600;
    QSerialPort::DataBits dataBits;
    QSerialPort::StopBits stopBits;
    QSerialPort::Parity checkBits;

    // 设置默认波特率115200
    qint32 baudRate=QSerialPort::Baud115200;
    // 设置默认数据位
    dataBits=QSerialPort::Data8;
    // 设置默认停止位
    stopBits=QSerialPort::OneStop;
    // 设置默认校验位
    checkBits = QSerialPort::NoParity;
    // 初始化串口属性，设置 端口号、波特率、数据位、停止位、奇偶校验位数
    // qDebug() << "portNames: " << ui->portNames->currentText();
    serialPort->setPortName(ui->portNames->currentText());
    serialPort->setBaudRate(baudRate);
    serialPort->setDataBits(dataBits);
    serialPort->setStopBits(stopBits);
    serialPort->setParity(checkBits);

    // 如果打开成功，反转打开按钮显示和功能。打开失败，无变化，并且弹出错误对话框。
    // 分支判据是 isSerialPortConnected 而不是按钮上的文字（工单 06）：命令栏那颗主按钮
    // 与页内这颗按钮是同一个动作的两个入口，靠「读按钮文案」推链路状态迟早会不同步。
    if (!isSerialPortConnected) {
        if(serialPort->open(QIODevice::ReadWrite) == true){
            // 设置WifiBox不可选（两条链路互斥：串口开着时不再接网络）
            ui->wifiBox->setEnabled(false);
            // 「发送」分组**不再置灰**：协议 v2.9 起串口上有命令帧 0x00（字节 2 = 子命令），
            // 「通信测试 / 保存地图 / 读参数 / 保存参数 / 重启模块算法」五件事在串口上都能做，
            // 只有「下传地图」那一条不在其中（见 onDownloadMapRequested）。
            // 不支持检测串口
            ui->portSearchBt->setEnabled(false);
            isSerialPortConnected=true;
            ui->portOpenBt->setText("关闭串口");
            // 让端口号下拉框不可选，避免误操作（选择功能不可用，控件背景为灰色）
            ui->portNames->setEnabled(false);
            // 状态标签 / 标题栏胶囊 / 状态栏第一段共用这一条信号（工单 05、07），
            // 文字格式只在 portStateText() 里定：端口 · 波特率 · 状态
            emit portStateChanged(
                portStateText(serialPort->portName(), baudRate, QStringLiteral("已连接")), true);

            // 门禁跟着链路走：串口一开，「保存地图 / 读参数 / 写参数 / 重启算法」就都该亮
            // （「下载地图」仍不亮 —— 串口没有 05 这条子命令）
            emitCommandLinkGates();

            // 日志窗口留一行交代这次连接能做什么。**不弹模态框**：原先那句
            // 「仅用于测试串口输出坐标」在协议加了命令帧之后已经是错的，而且它会挡住日志。
            ui->recvEdit->append(
                QDateTime::currentDateTime().toString("HH:mm:ss >> ") +
                QStringLiteral("串口已打开：%1 · %2 · 8N1；可用命令与 TCP 同号"
                               "（通信测试 / 保存地图 / 读参数 / 保存参数 / 重启算法），"
                               "下载地图仍需走 TCP。")
                    .arg(serialPort->portName()).arg(baudRate));

            // 串口连接成功，启动打印定时器
            if (!printTimer->isActive()) {
                printTimer->start();
            }
            if (!statsPrintTimer->isActive()) {
                statsPrintTimer->start();
                count01 = 0;
                count02 = 0;
                count03 = 0;
                count04 = 0;
                count05 = 0;
            }
        }else{
            QMessageBox::critical(this, "错误", "串口打开失败，请检查串口是否被占用");
            emit portStateChanged(
                portStateText(serialPort->portName(), baudRate, QStringLiteral("打开失败")), false);
        };


    }else{
        serialPort->close();
        isSerialPortConnected=false;
        ui->portOpenBt->setText("打开串口");
        // 端口号下拉框恢复可选，避免误操作
        ui->portNames->setEnabled(true);
        // 关闭时端口号还留着，所以三段仍是「COM3 · 115200 · 已关闭」—— 看得出刚才用的是哪个口
        emit portStateChanged(
            portStateText(serialPort->portName(), baudRate, QStringLiteral("已关闭")), false);
        // 设置WifiBox可选
        ui->wifiBox->setEnabled(true);
        // 支持检测串口
        ui->portSearchBt->setEnabled(true);

        // 串口一关，控制指令的门禁要跟着收回（否则按钮还亮着、点下去只能弹一句「请先连接模块」）
        emitCommandLinkGates();

        // 串口关闭，停止打印定时器
        if (printTimer->isActive()) {
            printTimer->stop();
        }
        if (statsPrintTimer->isActive()) {
            statsPrintTimer->stop();
        }
    };
}

// 命令栏「连接设备 / 断开设备」（工单 06）：一条电话线，不复制流程。
// 开还是关由上面那一段按真实连接状态自己判，这里不传方向、也不缓存状态 ——
// 主按钮的外观另有数据源（portStateChanged 回到 CommandBar::setPortConnected）。
void SerialPort::togglePortConnection()
{
    on_portOpenBt_clicked();
}

// 接收字节数统计归零
void SerialPort::on_clearRecvBt_clicked()
{
    ui->recvEdit->clear();
    // 清除发送、接收字节计数
    recvNum = 0;
    QString sm = "接收字节数量： 0";
    ui->recvNum->setText(sm);
}

// 发送字节数统计归零
void SerialPort::on_btnClearSend_clicked()
{
    // 清除发送、接收字节计数
    sendNum = 0;
    QString sm = "发送字节数量： 0";
    ui->sendNum->setText(sm);
}

void SerialPort::on_protocolComboBox_currentIndexChanged(int index)
{
    QString protocol = ui->protocolComboBox->currentText();

    if (protocol == "UDP") {
        QMessageBox::warning(this, "功能未开放", "UDP功能未开放");
    }
}

// 每1秒打印0x01和0x03数据（仅串口模式）
void SerialPort::onPrintTimerTimeout()
{
    // 仅在串口模式下打印
    if (!isSerialPortConnected) return;

    QString timestamp = QDateTime::currentDateTime().toString("HH:mm:ss");

    // 打印0x01当前位姿数据
    if (latestPose01.valid) {
        ui->recvEdit->append(timestamp + " >> 当前位姿: x=" + QString::number(latestPose01.x) +
                             "cm y=" + QString::number(latestPose01.y) + "cm z=" +
                             QString::number(latestPose01.z) + "cm roll=" +
                             QString::number(latestPose01.roll) + " pitch=" +
                             QString::number(latestPose01.pitch) + " yaw=" +
                             QString::number(latestPose01.yaw) + "°");
    }

    // 打印0x03位置控制指令数据
    if (latestPose03.valid) {
        ui->recvEdit->append(timestamp + " >> 位置指令: x=" + QString::number(latestPose03.x) +
                             "cm y=" + QString::number(latestPose03.y) + "cm z=" +
                             QString::number(latestPose03.z) + "cm roll=" +
                             QString::number(latestPose03.roll) + " pitch=" +
                             QString::number(latestPose03.pitch) + " yaw=" +
                             QString::number(latestPose03.yaw) + "°");
    }

    // 打印0x05状态帧（五件）：飞控 / 重定位 / 规划 + CPU / 运存。
    // 模块那边 1 Hz 发，这里也 1 Hz 打一行 —— 与前面两行同频，不额外刷屏；
    // 收到过至少一帧才打（valid），否则开串口的那一秒就会先出现一行全初值的假状态。
    // 两格百分比为 0xFF 时显示「—」：那是「没人报」，不是 0%。
    if (latestStatus05.valid) {
        ui->recvEdit->append(
            timestamp + " >> 模块状态: 飞控=" + fcStateText(latestStatus05.fcState) +
            " 重定位=" + progressStateText(latestStatus05.relocalization) +
            " 规划=" + progressStateText(latestStatus05.planning) +
            " CPU=" + percentText(latestStatus05.cpuPercent) +
            " 运存=" + percentText(latestStatus05.memPercent));
    }
}

// 每10秒统计频率（仅串口模式）
void SerialPort::onStatsPrintTimerTimeout()
{
    // 仅在串口模式下打印统计
    if (!isSerialPortConnected) return;

    QString timestamp = QDateTime::currentDateTime().toString("HH:mm:ss");

    double freq01 = count01 / (double)statsIntervalSeconds;
    double freq02 = count02 / (double)statsIntervalSeconds;
    double freq03 = count03 / (double)statsIntervalSeconds;
    double freq04 = count04 / (double)statsIntervalSeconds;
    double freq05 = count05 / (double)statsIntervalSeconds;

    ui->recvEdit->append(timestamp + " >> ====== 话题接收频率统计 ======");
    ui->recvEdit->append(timestamp + " >> 位姿：" + QString::number(freq01, 'f', 2) + "Hz");
    ui->recvEdit->append(timestamp + " >> 导航目标点：" + QString::number(freq02, 'f', 2) + "Hz");
    ui->recvEdit->append(timestamp + " >> 导航位置指令：" + QString::number(freq03, 'f', 2) + "Hz");
    // 0x04 是 pos_cmd 转发下来的**规划速度**（规划器给出的速度指令），不是实测实时速度，
    // 文案照这个叫法写，别写成「导航速度指令」或「实时速度」。
    ui->recvEdit->append(timestamp + " >> 规划速度：" + QString::number(freq04, 'f', 2) + "Hz");
    ui->recvEdit->append(timestamp + " >> 状态：" + QString::number(freq05, 'f', 2) + "Hz");
    ui->recvEdit->append(timestamp + " >> ==============================");

    // 重置计数器
    count01 = 0;
    count02 = 0;
    count03 = 0;
    count04 = 0;
    count05 = 0;
}
