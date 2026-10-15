#include "serialport.h"
#include "ui_serialport.h"
#include "utils/config/config.h"
#include "protocolros3d.h"
#include "protocolrouter.h"
#include "maparchive.h"     // 地图下传分块的拼装 + CRC-32 校验（工单 09）

#include <QDataStream>
#include <QTimer>
#include <QMessageBox>
#include <QStringList>
#include <QCoreApplication>
#include <QDir>
#include <QFileDialog>
#include <QSaveFile>
#include <qmath.h>


// 重启模块（工单 01）的回发等待窗口。模块收到指令即回发（毫秒级的事），3 秒足够容忍链路
// 延迟，又不至于让人干等。定义在这一处：发出指令与超时文案读的是同一个值，以后要调只改这里。
static const int RESTART_ACK_TIMEOUT_MS = 3000;

// 0x05 Byte 2 飞控状态枚举（对应协议文档那张枚举表），非法值返回"未知"
static QString fcStatusName(quint8 state)
{
    switch (state) {
    case 0x00: return "未连接/通信丢失";
    case 0x01: return "未解锁 (Unarmed)";
    case 0x02: return "已解锁，地面待命";
    case 0x03: return "空中悬停 (Hovering)";
    case 0x04: return "执行航点 (Offboard)";
    default:
        return QString("未知(0x%1)")
                .arg(QString("%1").arg(state, 2, 16, QChar('0')).toUpper());
    }
}

// 0x05 Byte 3 重定位四档 → 中文，与 TCP 侧 AA 0B 的四档同义同值（文案也保持一致）。
// 3 是模块的一句明确说法「本链上尚无该状态」，不是「未知」也不等于「进行中」。
// 「一包都没收到」（由 lastRelocState_ < 0 表示）才是不显示，两者不能混为一谈。
static QString relocStateName(quint8 state)
{
    switch (state) {
    case 0: return "进行中";
    case 1: return "成功";
    case 2: return "失败";
    case 3: return "未响应";
    default: return QString("未知(%1)").arg(state);
    }
}

// 0x05 Byte 4 规划四档 → 中文，取值与文案同重定位那四档（也与 TCP 侧 AA 0B 一致）。
// 旧实现读的是 Byte 8~9（恒零的预留区），于是规划状态永远显示「规划中」而不报错。
static QString planStateName(quint8 state)
{
    switch (state) {
    case 0: return "规划中";
    case 1: return "规划成功";
    case 2: return "规划出错";
    case 3: return "未响应";
    default: return QString("未知(%1)").arg(state);
    }
}


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
            ui->lblWifiState->setText("TCP已连接");
            ui->lblWifiState->setStyleSheet("color:green");
            ui->wifiConnectBt->setText("关闭连接");

            // TCP连接成功时启动统计定时器
            if (!statsTimer->isActive()) {
                statsTimer->start();
                elapsedTimer.restart();
            }

        } else {
            ui->lblWifiState->setText("尝试自动重连");
            ui->lblWifiState->setStyleSheet("color:red");

            // 停止统计定时器
            if (statsTimer->isActive()) {
                statsTimer->stop();
            }
        }
        emit tcpConnectionChanged(connected);
        emit controlLinkStateChanged(connected || isUdpBound);
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

    // 地图下载的两级超时：两条各一个单次定时器。
    // 块间静默那条每收一块就重置（卡在第 3 块不该和第 28 块等一样久），
    // 整包那条从头跑到尾（兜底，不让整包无限期挂着）。
    downloadChunkTimer = new QTimer(this);
    downloadChunkTimer->setSingleShot(true);
    connect(downloadChunkTimer, &QTimer::timeout, this, &SerialPort::onDownloadChunkTimeout);

    downloadTotalTimer = new QTimer(this);
    downloadTotalTimer->setSingleShot(true);
    connect(downloadTotalTimer, &QTimer::timeout, this, &SerialPort::onDownloadTotalTimeout);

    // 重启模块（工单 01）的回发等待窗口：同样单独一个单次定时器（3 秒与通信测试/存图/存参数
    // 的语义都不同，不复用）。等待期按钮由 updateRestartButton() 置灰。
    restartTimer = new QTimer(this);
    restartTimer->setSingleShot(true);
    connect(restartTimer, &QTimer::timeout, this, &SerialPort::onRestartTimeout);

    //设置状态标签为红色 表示等待连接状态
    ui->lblPortState->setStyleSheet("color:red");

    // ui->serialBox->setEnabled(false);

    ui->lblWifiState->setStyleSheet("color:red");

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
            ui->lblWifiState->setText("未连接");
            ui->lblWifiState->setStyleSheet("color:red");
            ui->serialBox->setEnabled(true);
        }
    });

    // 断线被压制（工单 03）：这次断开是预期内的（重启模块），不是无来由掉线。
    // 「重连失败」弹窗不会出现 —— TcpClient 在开关关闭时根本不发 reconnectTimeout()，
    // 不需要在这里挡那个信号。这里只把界面说清楚：日志写一行解释，连接按钮复位成
    // 「打开连接」，让操作员知道下一步该由他手动重连。
    connect(tcpClient, &TcpClient::autoReconnectSuppressed, this, [this]() {
        QString t = QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss >> ");
        ui->recvEdit->append(t + (restartDisconnectExpected_
            ? QStringLiteral("连接已断开（与重启一致）")
            // 同一把开关在工单 04 也被手动「关闭连接」用上，那条路别说是重启
            : QStringLiteral("连接已断开（已关闭自动重连，需手动打开连接）")));

        ui->wifiConnectBt->setText("打开连接");
        // 不写「尝试自动重连」—— 这一次确实一次都不会尝试（那是上面那个 lambda 的话）
        ui->lblWifiState->setText("未连接");
        ui->lblWifiState->setStyleSheet("color:red");
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

    // 串口数据解析 - 缓存0x04实时速度帧，仅在串口模式下计数
    connect(ProtocolRouter::instance(), &ProtocolRouter::uartSpeedReceived,
            this, [this](qint16 vx, qint16 vy, qint16 vz, qint16 yawDot){
                // 仅在串口模式下处理
                if (!isSerialPortConnected) return;

                latestSpeed.vx = vx;
                latestSpeed.vy = vy;
                latestSpeed.vz = vz;
                latestSpeed.yawDot = yawDot;
                latestSpeed.valid = true;
                count04++;
            });

    // 串口数据解析 - 缓存0x05状态帧（Byte 2 飞控 / Byte 3 重定位 / Byte 4 规划）
    connect(ProtocolRouter::instance(), &ProtocolRouter::uartFlightStatusReceived,
            this, [this](quint8 fcState, quint8 relocalization, quint8 planning){
                // 仅在串口模式下处理
                if (!isSerialPortConnected) return;

                latestFcStatus.state = fcState;
                latestFcStatus.relocalization = relocalization;
                latestFcStatus.planning = planning;
                latestFcStatus.valid = true;
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

                    QString t = QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss");
                    ui->recvEdit->append(
                        QString("%1  模块回传目标点：x=%2m y=%3m z=%4m yaw=%5°")
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
    // 使用协议工具类构建测试帧
    QByteArray testFrame = ProtocolRouter::buildTestFrame();
    qDebug() << "发送测试数据中...."<< testFrame;
    sendData(testFrame);
    QString timestamp = QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss >> 测试操作: ");
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
QString SerialPort::saveMapModuleDesc() const
{
    if (latestModuleType == 2 || latestModuleType == 3) return QStringLiteral("2D");
    if (latestModuleType == 0 || latestModuleType == 1) return QStringLiteral("3D");
    return QStringLiteral("模块类型未读到，按 3D");
}

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
    // 与按钮的可用条件一致：只有 TCP / UDP 能承载控制指令，串口不在此列
    TcpClient* tcpClient = TcpClient::getInstance();
    if (!tcpClient->isConnected() && !isUdpBound) {
        QMessageBox::warning(this, "警告", "请先连接模块！");
        return;
    }

    const int timeoutMs = saveMapTimeoutMs();

    // 点下去就出现模态等待框，不给取消：模块侧已经在落盘了，「取消」在这里没有真实语义。
    // 关框与结果提示由 onSaveMapAckReceived / onSaveMapTimeout 负责。
    showSaveMapWait();

    sendData(ProtocolRouter::instance()->buildSaveMapFrame());

    // 上一轮补发的通信测试若还没出结果，这一轮重新计时
    saveMapProbePending = false;
    testTimer->stop();
    saveMapTimer->start(timeoutMs);

    QString timestamp = QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss >> ");
    ui->recvEdit->append(timestamp + QString("已下发保存地图指令 AA 00 02 0A（%1），等待模块回包，%2 秒后判失败")
                         .arg(saveMapModuleDesc())
                         .arg(timeoutMs / 1000.0, 0, 'f', 1));
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

    QString timestamp = QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss >> ");
    ui->recvEdit->append(timestamp + (wasPending
        ? QString("保存地图成功：模块已回包 AA 00 02 0A（产物核对通过）")
        : QString("收到模块的保存地图回包 AA 00 02 0A（当前没有未确认的保存请求）")));

    // 回包就是「产物核对通过」——这正是模块回包的真实含义，成功文案必须点明它。
    // 没有未确认的请求（没人点过按钮）时不弹：那时用户并没有在等。
    if (wasPending) {
        QMessageBox box(QMessageBox::Information, QStringLiteral("保存地图成功"),
                        QStringLiteral("模块已回包：产物核对通过。"),
                        QMessageBox::Ok, window());
        box.exec();
    }
}

void SerialPort::onSaveMapTimeout()
{
    closeSaveMapWait();

    QString timestamp = QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss >> ");
    ui->recvEdit->append(timestamp + QString("保存地图超时：%1 秒内未收到 AA 00 02 0A（失败不回显）。"
                                             "正在补发通信测试 AA 00 01 0A，以区分「存图失败」与「链接已断」...")
                         .arg(saveMapTimeoutMs() / 1000.0, 0, 'f', 1));

    // 补发一次通信测试：复用 buildTestFrame 与 testFrameReceived → handleTestFrame 这条链路，
    // 结果由 handler 里的 saveMapProbePending 分流（成功 = 存图失败，超时 = 链接已断）
    saveMapProbePending = true;
    testFlag = false;
    sendData(ProtocolRouter::buildTestFrame());
    testTimer->start(3000);

    // 弹窗只给不承诺原因的结论：模块失败时是沉默的（定位模式下必然被拒也不回显），
    // 上位机分不出「存图失败」与「链路已断」。一个自信但可能错误的结论比没有结论更糟，
    // 精确结论留给上面那句日志和刚补发的通信测试。
    QMessageBox box(QMessageBox::Warning, QStringLiteral("保存地图"),
                    QStringLiteral("未收到模块回包，保存结果未知。"),
                    QMessageBox::Ok, window());
    box.setInformativeText(QStringLiteral("「存图失败」与「链路已断」的区分见日志窗口。"));
    box.exec();
}

// 写入参数 /////////////////////////////////////////////////////////////////
// 参数页把「要写入的参数表」交过来，由链路层统一发送：逐条 AA 10 <参数号> <值>，
// 最后补一条保存终结指令 AA 00 04 0A。发送放在这里（而不是参数页里），
// 是为了让发送方与等待方同处一地——超时、断链、门禁的判断只有一处。
//
// 回包规矩（upper/config/通讯协议.md 消息表）：模块收到 AA 00 04 0A 后写入参数文件、
// **从盘上重读一次**，然后把**全表 21 条 AA 10** 连同本帧**原样回发**，**不重启模块**。
// 所以「保存成功没有」的唯一依据是：回读值逐条等于下发值。模块对表外参数号整条静默丢弃、
// 对已知号的越界值也不回错——不比对的话，填错值、发错号都只会得到一句「成功」。
void SerialPort::onWriteParamsRequested(const QVector<ParamWrite> &params)
{
    // 与按钮的可用条件一致：控制指令只走 TCP / UDP，串口链只承载定长状态帧族
    TcpClient* tcpClient = TcpClient::getInstance();
    if (!tcpClient->isConnected() && !isUdpBound) {
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
        sendData(ProtocolRouter::buildWriteParamFrame(p.id, p.value));
    }

    // 保存终结指令：模块收到它才落盘（并回发全表参数值，下面等这个）
    sendData(ProtocolRouter::buildFrame(0x00, QVariantMap{{"sub_cmd", 0x04}}));

    const int timeoutMs = saveParamsTimeoutMs();
    saveParamsTimer->start(timeoutMs);

    QString timestamp = QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss >> ");
    ui->recvEdit->append(timestamp + QString("已下发 %1 条 AA 10 与保存终结指令 AA 00 04 0A，"
                                             "等待模块回读全表参数，%2 秒后判失败")
                         .arg(params.size())
                         .arg(timeoutMs / 1000.0, 0, 'f', 1));
}

// 模块下发的每条 AA 10：0x99 模块类型与 0x13「模式」照常缓存；保存等待期把它按参数号存进比对器。
void SerialPort::onModuleParameterFrame(quint8 paramId, qint16 value)
{
    if (paramId == 0x99) latestModuleType = value;
    if (paramId == 0x13) setModuleMode(value);   // 图形层判建图 / 定位（工单 03）

    if (saveParamsPending) {
        saveParamsReadback.insert(paramId, value);
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
    const QString timestamp = QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss >> ");

    if (!saveParamsPending) {
        // 没有未确认的保存请求（没人点过按钮）：不弹框，只记一笔
        ui->recvEdit->append(timestamp +
            QString("收到模块的保存终结指令 AA 00 04 0A（当前没有未确认的保存请求）"));
        return;
    }

    saveParamsTimer->stop();
    saveParamsPending = false;
    emit paramEchoSuppressedChanged(false);

    // 先关等待框再弹结果：两个都是模态的，叠在一起会先关掉下面那个
    closeSaveParamsWait();

    QStringList details;          // 不一致明细（弹窗与日志共用同一份）
    QStringList unexpected;       // 模块回了、但快照里没有的参数号：上位机的表该更新了
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

    // 反向看一眼：模块回读了快照里没有的号（两边参数表不一致），写日志供事后对账
    QMap<quint8, qint16>::const_iterator it = saveParamsReadback.constBegin();
    for (; it != saveParamsReadback.constEnd(); ++it) {
        bool inSnapshot = false;
        for (const ParamWrite &p : saveParamsSnapshot) {
            if (p.id == it.key()) { inSnapshot = true; break; }
        }
        if (!inSnapshot) {
            unexpected << QString("0x%1=%2")
                    .arg(it.key(), 2, 16, QLatin1Char('0')).arg(it.value());
        }
    }

    ui->recvEdit->append(timestamp +
        QString("保存参数回读完成：收到 %1 条 AA 10，与快照比对，不一致 %2 项")
        .arg(saveParamsReadback.size()).arg(details.size()));
    if (!unexpected.isEmpty()) {
        ui->recvEdit->append(timestamp +
            QString("模块回读了快照之外的参数号（两边参数表不一致，仅供参考）：%1")
            .arg(unexpected.join("、")));
    }

    if (details.isEmpty()) {
        settleSaveParams(
            QString("参数保存成功：%1 条回读全部与下发值一致（模块落盘后从盘上重读）")
                .arg(saveParamsSnapshot.size()),
            QString("参数已保存（%1 项一致）").arg(saveParamsSnapshot.size()),
            QStringLiteral("保存参数"),
            QStringLiteral("参数已保存，请重启模块使参数生效。"),
            QString(), QMessageBox::Information);
        return;
    }

    // 明细同时写进日志：事后要拿去和模块侧对账，弹窗关掉就没了
    ui->recvEdit->append(timestamp + QString("参数保存不一致明细（上位机期望 → 模块返回）："));
    for (const QString &line : details) {
        ui->recvEdit->append("    " + line);
    }

    settleSaveParams(
        QString("参数保存不一致：%1 项与下发值不同（仅日志中已逐条列出）").arg(details.size()),
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
    QString timestamp = QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss >> ");
    ui->recvEdit->append(timestamp +
        QString("保存参数超时：%1 秒内未收到终结指令 AA 00 04 0A（已收到 %2/%3 条参数回读）。"
                "模块失败时是沉默的，本次保存结果未知。")
        .arg(timeoutMs / 1000.0, 0, 'f', 1)
        .arg(saveParamsReadback.size())
        .arg(saveParamsSnapshot.size()));

    settleSaveParams(
        QString("参数保存超时：%1 秒内未收到终结指令").arg(timeoutMs / 1000.0, 0, 'f', 1),
        QString("参数保存超时"),
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
void SerialPort::settleSaveParams(const QString &logLine, const QString &summary,
                                  const QString &title, const QString &text,
                                  const QString &detail, QMessageBox::Icon icon)
{
    const QString timestamp = QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss >> ");
    ui->recvEdit->append(timestamp + logLine);

    emit saveParamsFinished(summary);

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
    // 与按钮的可用条件一致：控制指令只走 TCP / UDP，串口链不承载这一类消息
    TcpClient* tcpClient = TcpClient::getInstance();
    if (!tcpClient->isConnected() && !isUdpBound) {
        QMessageBox::warning(this, "警告", "请先连接模块！");
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

    QString timestamp = QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss >> ");
    ui->recvEdit->append(timestamp +
        QString("已下发下载地图指令 AA 00 05 0A，等待模块分块（块间静默 %1 秒 / 整包 %2 秒）")
            .arg(chunkMs / 1000.0, 0, 'f', 1)
            .arg(totalMs / 1000.0, 0, 'f', 1));
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

    const QString timestamp = QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss >> ");

    // 校验失败：长度或 CRC 不符。坏包不能伪装成好包躺在用户的目录里，所以不落盘、直接报错。
    if (!result.ok()) {
        ui->recvEdit->append(timestamp +
            QString("地图下载校验失败（%1）：%2。未落盘。")
                .arg(QString::fromLatin1(MapArchive::statusText(result.status)), result.detail));

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
        // 取消文件对话框：不落盘、也不报错，已下载数据丢弃。
        // 这不是失败，是一条操作日志——用户主动放弃了这次保存。
        ui->recvEdit->append(timestamp + QStringLiteral("已取消保存，下载到的地图包已丢弃（未落盘）。"));
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
        ui->recvEdit->append(timestamp +
            QString("地图下载成功但落盘失败：%1（%2）。")
                .arg(QDir::toNativeSeparators(path), file.errorString()));

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
        QString("地图下载成功：%1（%2 字节）已保存到 %3")
            .arg(downloadPkgName.isEmpty() ? QStringLiteral("地图包") : downloadPkgName)
            .arg(result.package.size())
            .arg(nativePath));

    QMessageBox box(QMessageBox::Information, QStringLiteral("地图下载成功"),
                    QStringLiteral("地图包已保存到：\n%1").arg(nativePath),
                    QMessageBox::Ok, window());
    box.exec();
}

// 取消：下载没有副作用，取消就是取消——不落盘、也不报错（所以不走 abortDownload）。
void SerialPort::onDownloadCancelled()
{
    downloadChunkTimer->stop();
    downloadTotalTimer->stop();
    downloadPending = false;

    const int got = downloadChunks.size();
    const int total = downloadTotalChunks;
    downloadChunks.clear();

    // 关框不在这里做：cancelled() 是 WaitDialog 自己在关闭路径上发出来的，它随后就把自己关掉
    const QString timestamp = QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss >> ");
    ui->recvEdit->append(timestamp +
        QString("已取消地图下载（已收 %1%2 块），下载数据已丢弃，未落盘。")
            .arg(got)
            .arg(total > 0 ? QStringLiteral("/%1").arg(total) : QString()));
}

// 失败收尾：停表、丢块、关框、记一行日志、弹一个结果框。
// 两条超时路径共用这一段，免得「关框—记日志—弹窗」的顺序各写一遍。
void SerialPort::abortDownload(const QString &logLine, const QString &text, const QString &detail)
{
    downloadChunkTimer->stop();
    downloadTotalTimer->stop();
    downloadPending = false;
    downloadChunks.clear();

    closeDownloadWait();

    const QString timestamp = QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss >> ");
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
    const QString logLine =
        got == 0
            ? QString("地图下载失败：%1 秒内一块都没收到（模块下传失败时一帧都不发，"
                      "与链路已断在此无法区分）。已丢弃，未落盘。").arg(secs)
            : QString("地图下载失败：%1 秒内未收到新的分块（已收 %2%3 块）。已丢弃，未落盘。")
                  .arg(secs)
                  .arg(got)
                  .arg(total > 0 ? QStringLiteral("/%1").arg(total) : QString());

    const QString text =
        got == 0
            ? QStringLiteral("未收到模块的任何分块，地图下载失败。")
            : QStringLiteral("分块传输中断，地图下载失败。");

    const QString detail =
        got == 0
            ? QStringLiteral("块间静默超时 %1 秒内一块都没收到。「模块下传失败」（打不出包 / 缺件）"
                             "时一帧都不发，但链路已断在这条链上同样一帧都没有，上位机无法区分——"
                             "精确结论见日志窗口。").arg(secs)
            : QStringLiteral("已收 %1%2 块。块间静默超时 %3 秒；"
                             "「模块下传失败」与「链路已断」的区分见日志窗口。")
                  .arg(got)
                  .arg(total > 0 ? QStringLiteral("/%1").arg(total) : QString())
                  .arg(secs);

    abortDownload(logLine, text, detail);
}

// 整包总超时：分块一直在来，但整包迟迟收不齐（模块那边卡住 / 链路极慢）。
void SerialPort::onDownloadTotalTimeout()
{
    const int got = downloadChunks.size();
    const int total = downloadTotalChunks;
    const QString secs = QString::number(downloadTotalTimeoutMs() / 1000.0, 'f', 0);

    abortDownload(
        QString("地图下载失败：整包总超时 %1 秒内未收齐分块（已收 %2%3 块）。已丢弃，未落盘。")
            .arg(secs)
            .arg(got)
            .arg(total > 0 ? QStringLiteral("/%1").arg(total) : QString()),
        QStringLiteral("未在总超时内收齐地图分块，地图下载失败。"),
        QStringLiteral("已收 %1%2 块。整包总超时 %3 秒；"
                       "「模块下传失败」与「链路已断」的区分见日志窗口。")
            .arg(got)
            .arg(total > 0 ? QStringLiteral("/%1").arg(total) : QString())
            .arg(secs));
}

// 重启模块 /////////////////////////////////////////////////////////////////
// 线上是一条控制帧 AA 00 06 0A（帧头 + 帧号 0x00 控制组 + 子命令 0x06 + 帧尾），帧内没有长度
// 字段、没有校验字节、没有版本字节，与既有 TCP 帧一致。
// 回包规矩：模块**先原样回发本帧、再执行关机**。所以「收到回发」= 模块已受理重启，是判成功的
// 唯一依据；而「连接断开」是另一件、后发生的事，本功能不去推断它（那会依赖模块是否发出 TCP
// FIN，属模块侧实现细节，不可依赖）。超时 = 未收到回发，结果未知 —— 不编造原因。
//
// 门禁：只走 TCP / UDP（串口链是另一套固定 15 字节帧族，4 字节的 AA 00 06 0A 在它那里根本不成帧），
// 与「保存地图」「写入参数」同一条控制链路门禁。不设管理权限门禁，对所有用户开放。
void SerialPort::setLinkConnected(bool connected)
{
    controlLinkConnected_ = connected;
    updateRestartButton();
}

// 「重启模块」可用性的唯一判定处，三条规则都写在这里：
//   1. 控制链路在（TCP 已连接或 UDP 已绑定）—— 由 controlLinkStateChanged 喂进来；
//   2. 不在串口模式 —— 同上，串口链不承载这条指令（控件树上这一条也已经成立：按钮所在的
//      「发送」分组在打开串口时被整体置灰，与「发送测试数据」是同一条规矩，这里再写一遍
//      是为了三条规则在同一处可读、可查）；
//   3. 不在等待回发的窗口内 —— 模块已经准备关机了，不该再发第二条重启指令。
void SerialPort::updateRestartButton()
{
    ui->restartBt->setEnabled(controlLinkConnected_ && !isSerialPortConnected && !restartPending);
}

void SerialPort::on_restartBt_clicked()
{
    // 与按钮的可用条件一致：这条指令只走 TCP / UDP。串口挨着当前串口打开时 sendData 会走串口，
    // 那是这条指令明确不支持的一条链，所以这里连串口模式一并挡住。
    TcpClient* tcpClient = TcpClient::getInstance();
    if (isSerialPortConnected || (!tcpClient->isConnected() && !isUdpBound)) {
        QMessageBox::warning(this, "警告", "请先连接模块！");
        return;
    }

    // 二次确认：这是一次会中断通信的破坏性操作，先说清代价。
    // 选「取消」什么都不发生 —— 不发帧、不起定时器、不打日志（下面的代码一行都到不了）。
    QMessageBox confirm(QMessageBox::Warning, QStringLiteral("重启模块"),
                        QStringLiteral("将重启模块（断电重启）。"),
                        QMessageBox::Ok | QMessageBox::Cancel, window());
    confirm.setInformativeText(QStringLiteral("通信会中断；重启期间模块不在线，所有指令均无效。"));
    // 按钮文字自己给（QMessageBox 的标准按钮没装翻译，默认是英文 OK / Cancel）；
    // 破坏性动作的确认按钮不叫「OK」，默认键给「取消」——回车不该把模块重启掉。
    confirm.setButtonText(QMessageBox::Ok, QStringLiteral("重启"));
    confirm.setButtonText(QMessageBox::Cancel, QStringLiteral("取消"));
    confirm.setDefaultButton(QMessageBox::Cancel);
    if (confirm.exec() != QMessageBox::Ok) return;

    // 1. 先让操作员在日志里看到「指令确实发出去了」——这是三个可分辨事件里的第一个
    QString timestamp = QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss >> ");
    ui->recvEdit->append(timestamp + QStringLiteral("已发送重启模块指令，通信即将中断（AA 00 06 0A）"));

    // 2. 发出 AA 00 06 0A。发帧之前先关掉自动重连（工单 03）：模块收到即断电，断线随时会来，
    //    开关必须先于那一帧在位。**成功回调里不解除** —— 回显早于断电到达，图省事在成功回调里
    //    解除的话，断电那一刻自动重连会照常触发（必然失败），本功能就静默失效了。
    //    一直压到用户自己点「打开连接」为止：万一模块重启得慢、或者回发丢了但其实断电了，
    //    那一刻也依然被正确压制。
    restartDisconnectExpected_ = true;
    tcpClient->setAutoReconnectEnabled(false);
    sendData(ProtocolRouter::instance()->buildRestartFrame());

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

    QString timestamp = QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss >> ");
    ui->recvEdit->append(timestamp + (wasPending
        ? QStringLiteral("重启模块成功：模块已回发 AA 00 06 0A（已受理，即将断电重启，通信将中断）")
        : QStringLiteral("收到模块的重启指令回发 AA 00 06 0A（当前没有未确认的重启请求）")));

    // 没有未确认的请求（没人点过按钮）时不弹：那时用户并没有在等。
    // 清理同样挂在这个条件上 —— 没人点过按钮时的回显是「无源之水」，那次重启并不成立，
    // 顺手清掉界面就是在毁掉用户正在看的东西。
    if (wasPending) {
        // 判定成功 = 模块即将变成一个「什么都没见过」的新模块，界面上那些运行态镜像
        // 描述的是重启前那个已经不复存在的模块，此刻一并清掉（工单 02）。
        // 失败路径（onRestartTimeout）一条都不清 —— 超时意味着模块很可能根本没重启。
        clearModuleRuntimeCaches();     // 本页：位姿 / 速度 / 飞控状态 / 三个锁存 / 模块类型 / 模式
        emit restartSucceeded();        // 3D 页面：TF 标签、状态标签与锁存（含 3D 视图本身）
        ui->recvEdit->append(timestamp +
            QStringLiteral("已清理界面上的模块运行态数据：点云 / 地图 / 位姿 / 状态标签"));

        QMessageBox box(QMessageBox::Information, QStringLiteral("重启模块"),
                        QStringLiteral("模块已回发重启指令回显，重启指令已被受理。"),
                        QMessageBox::Ok, window());
        box.setInformativeText(QStringLiteral("模块即将断电重启，通信会中断；"
                                              "重启期间所有指令均无效。"));
        box.exec();
    }
}

// 重启判定成功后清掉本页缓存的运行态镜像（工单 02）。
// 这些值描述的是重启前那个模块：不清的话，每秒一次的打印定时器（0x01/0x03/0x04/0x05）
// 会继续把旧位姿、旧速度、旧状态当「当下值」吐到接收窗口，看起来像模块还活着。
// 只清本页的缓存；参数页界面上那批值一律不动 —— 那是用户给模块的配置，不是模块的运行输出。
void SerialPort::clearModuleRuntimeCaches()
{
    // 位姿两路：0x01 当前位姿 / 0x03 位置控制指令。valid 归假，打印定时器随之闭嘴
    latestPose01 = PoseData();
    latestPose03 = PoseData();

    // 实时速度（0x04）
    latestSpeed = SpeedData();

    // 飞控状态帧（0x05，三件各占一字节）
    latestFcStatus = FcStatus();

    // 三个跳变锁存归 -1（= 「这一路还没收到过任何一包」）：模块重启后重新上报的第一帧
    // 要能再写一行「跳变 / 首次收到」，而不是因为锁存里还留着重启前的旧值被静默掉
    lastFcState_    = -1;
    lastRelocState_ = -1;
    lastPlanState_  = -1;

    // 模块类型（参数 0x99）：-1 = 没读到过。它决定存图超时取 3D 的宽档还是 2D 的窄档，
    // 重启后模块还没自报类型之前，按「不知道」宽档兜底才是安全的
    latestModuleType = -1;

    // 模式（参数 0x13）：同样归 -1 = 没读到过，并把这次「忘掉」下发给图形层。
    // 重启前那个值描述的是已经不复存在的旧模块，而重启后模式完全可能被改过；
    // 图形层手上那份副本要跟着回到「不知道」，否则它会拿着旧模式继续画标记（工单 09）
    setModuleMode(-1);
}

// 窗口超时 → 失败。**不执行任何清理**：超时意味着模块很可能根本没重启，界面上那些运行态
// 镜像依然有效，顺手清掉就是毁掉还在用的东西。文案也不编造原因 —— 模块是否真的断了、链路
// 是否还在，上位机在那一刻区分不了。
void SerialPort::onRestartTimeout()
{
    restartPending = false;
    updateRestartButton();

    QString timestamp = QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss >> ");
    ui->recvEdit->append(timestamp +
        QStringLiteral("重启模块超时：%1 秒内未收到模块回发 AA 00 06 0A，重启结果未知。")
            .arg(RESTART_ACK_TIMEOUT_MS / 1000.0, 0, 'f', 1));

    QMessageBox box(QMessageBox::Warning, QStringLiteral("重启模块"),
                    QStringLiteral("未收到模块回发，重启结果未知。"),
                    QMessageBox::Ok, window());
    box.setInformativeText(QStringLiteral("模块是否已断电重启、链路是否还在，上位机此刻区分不了。"));
    box.exec();
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

        QString timestamp = QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss >> ");

        // 这次测试是存图超时后自动补发的：回包只能证明「链接正常」——存图已经判失败。
        // 两种失败必须分开写，否则现场会把「存图失败」当成「网络抖了」去重启模块。
        if (saveMapProbePending) {
            saveMapProbePending = false;
            ui->recvEdit->append(timestamp +
                QString("保存地图失败：通信测试正常（链接正常），但模块未回 AA 00 02 0A → 存图失败。"
                        "若当前为定位模式（参数 0x13 = 1），本操作必然失败，"
                        "请切到建图模式或重新上电后再试，不要重试。"));
            return;
        }

        ui->recvEdit->append(timestamp + "测试成功！收到模块响应");

        // 显示原始响应数据
        QString hexStr = frame.toHex(' ').toUpper();
    } else {
        // 这是其他设备发来的测试请求（不是对我们请求的响应）
        qDebug() << "收到测试请求，自动回复响应";

        // 自动构建测试响应帧
        QByteArray response = ProtocolRouter::instance()->buildFrame(
            0x00, {
                {"sub_cmd", 0x01},
                {"is_request", false}  // 标记为响应
            }
            );

        // 回复对方
        sendData(response);
    }
}

// 超时处理
void SerialPort::onTestTimeout()
{
    if (!testFlag) {
        QString timestamp = QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss >> ");

        // 存图超时后自动补发的测试也超时 = 链路断了，存图结果无从判断。
        // 这条是机器补发的探测，不弹模态框（不然日志里那两句区分文案没法读）
        if (saveMapProbePending) {
            saveMapProbePending = false;
            testFlag = false;
            ui->recvEdit->append(timestamp +
                QString("保存地图失败：通信测试同样超时 → 链接已断，无法判断存图结果。"
                        "请先恢复连接再重试；若连接正常后仍失败，且当前为定位模式（参数 0x13 = 1），"
                        "本操作必然失败，请切到建图模式或重新上电后再试，不要重试。"));
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
        // 这里是「重启模块」那次压制的唯一解除处（工单 03）：用户手动点了「打开连接」，
        // 重启那一段到此结束 —— 从这里起再断线，照旧走原有的自动重连逻辑。
        // 判定成功的那条路不解除，这是一次性压制能否成立的分水岭。
        restartDisconnectExpected_ = false;
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
                    emit controlLinkStateChanged(tcpClient->isConnected() || isUdpBound);
                    ui->wifiConnectBt->setText("关闭连接");
                    ui->lblWifiState->setText("UDP已连接");
                    ui->lblWifiState->setStyleSheet("color:green");

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
            restartDisconnectExpected_ = false;   // 这条路别在日志里说成「与重启一致」
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
                ui->lblWifiState->setText("未连接");
                ui->lblWifiState->setStyleSheet("color:red");
            }
        }else if(protocol == "UDP"){
            if (isUdpBound) {
                udpSocket->close();
                isUdpBound = false;
                emit controlLinkStateChanged(tcpClient->isConnected() || isUdpBound);

                ui->wifiConnectBt->setText("打开连接");
                ui->lblWifiState->setText("未连接");
                ui->lblWifiState->setStyleSheet("color:red");
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
    if(ui->portOpenBt->text() == "打开串口"){
        if(serialPort->open(QIODevice::ReadWrite) == true){
            // 设置WifiBox不可选
            ui->wifiBox->setEnabled(false);
            // 不支持发送数据
            ui->groupBox_4->setEnabled(false);
            // 不支持检测串口
            ui->portSearchBt->setEnabled(false);
            isSerialPortConnected=true;
            ui->portOpenBt->setText("关闭串口");
            // 让端口号下拉框不可选，避免误操作（选择功能不可用，控件背景为灰色）
            ui->portNames->setEnabled(false);
            QString sm = "%1 串口已打开";
            QString status = sm.arg(serialPort->portName());
            ui->lblPortState->setText(status);
            ui->lblPortState->setStyleSheet("color:green");
            QMessageBox::warning(this, "提示", "仅用于测试串口输出坐标");

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
                // 清掉上一轮遗留的速度与状态帧，避免陈旧值停留在界面上；
                // 三件取值的 -1 一并归位，重连后第一包按「首次收到」重新写一行
                latestSpeed.valid = false;
                latestFcStatus.valid = false;
                lastFcState_ = -1;
                lastRelocState_ = -1;
                lastPlanState_ = -1;
            }
        }else{
            QMessageBox::critical(this, "错误", "串口打开失败，请检查串口是否被占用");
            QString sm = "%1 串口不可用";
            QString status = sm.arg(serialPort->portName());
            ui->lblPortState->setText(status);
            ui->lblPortState->setStyleSheet("color:red");
        };


    }else{
        serialPort->close();
        isSerialPortConnected=false;
        ui->portOpenBt->setText("打开串口");
        // 端口号下拉框恢复可选，避免误操作
        ui->portNames->setEnabled(true);
        //statusBar 状态栏显示端口状态
        QString sm = "%1 串口已关闭";
        QString status = sm.arg(serialPort->portName());
        ui->lblPortState->setText(status);
        ui->lblPortState->setStyleSheet("color:red");
        // 设置WifiBox可选
        ui->wifiBox->setEnabled(true);
        // 支持发送数据
        ui->groupBox_4->setEnabled(true);
        // 支持检测串口
        ui->portSearchBt->setEnabled(true);

        // 串口关闭，停止打印定时器
        if (printTimer->isActive()) {
            printTimer->stop();
        }
        if (statsPrintTimer->isActive()) {
            statsPrintTimer->stop();
        }
    };
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

    QString timestamp = QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss");

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

    // 打印0x04实时速度数据（100Hz：vx/vy/vz 单位 cm/s，yaw_dot 单位 度/s）
    if (latestSpeed.valid) {
        ui->recvEdit->append(timestamp + " >> 实时速度: vx=" + QString::number(latestSpeed.vx) +
                             "cm/s vy=" + QString::number(latestSpeed.vy) + "cm/s vz=" +
                             QString::number(latestSpeed.vz) + "cm/s yaw_dot=" +
                             QString::number(latestSpeed.yawDot) + "°/s");
    }

    // 打印0x05状态帧（1Hz）：飞控 / 重定位 / 规划三格，只在任一跳变时整行重写一遍。
    // 本帧是「当前值」不是「事件」，周期重发同一组值；逐帧写会把接收窗口刷屏，
    // 所以三件各自与上一行记下的值比对，全都没变就什么都不写。
    if (latestFcStatus.valid &&
        (int(latestFcStatus.state)          != lastFcState_  ||
         int(latestFcStatus.relocalization) != lastRelocState_ ||
         int(latestFcStatus.planning)       != lastPlanState_)) {
        lastFcState_   = latestFcStatus.state;
        lastRelocState_ = latestFcStatus.relocalization;
        lastPlanState_ = latestFcStatus.planning;

        ui->recvEdit->append(timestamp + " >> 飞控状态: " + fcStatusName(latestFcStatus.state) +
                             " 重定位状态: " + relocStateName(latestFcStatus.relocalization) +
                             " 规划状态: " + planStateName(latestFcStatus.planning));
    }
}

// 每10秒统计频率（仅串口模式）
void SerialPort::onStatsPrintTimerTimeout()
{
    // 仅在串口模式下打印统计
    if (!isSerialPortConnected) return;

    QString timestamp = QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss");

    double freq01 = count01 / (double)statsIntervalSeconds;
    double freq02 = count02 / (double)statsIntervalSeconds;
    double freq03 = count03 / (double)statsIntervalSeconds;
    double freq04 = count04 / (double)statsIntervalSeconds;
    double freq05 = count05 / (double)statsIntervalSeconds;

    ui->recvEdit->append(timestamp + " >> ====== 数据接收频率统计 ======");
    ui->recvEdit->append(timestamp + " >> 位姿话题：" + QString::number(freq01, 'f', 2) + "Hz");
    ui->recvEdit->append(timestamp + " >> 目标点话题：" + QString::number(freq02, 'f', 2) + "Hz");
    ui->recvEdit->append(timestamp + " >> 位置指令话题：" + QString::number(freq03, 'f', 2) + "Hz");
    ui->recvEdit->append(timestamp + " >> 实时速度：" + QString::number(freq04, 'f', 2) + "Hz");
    ui->recvEdit->append(timestamp + " >> 飞控状态：" + QString::number(freq05, 'f', 2) + "Hz");
    ui->recvEdit->append(timestamp + " >> ==============================");

    // 重置计数器
    count01 = 0;
    count02 = 0;
    count03 = 0;
    count04 = 0;
    count05 = 0;
}
