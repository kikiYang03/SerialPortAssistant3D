#include "serialport.h"
#include "ui_serialport.h"
#include "utils/config/config.h"
#include <QDataStream>
#include <QTimer>

// 10 字节帧 → 打印字符串
static QString uartFrameToText(const QByteArray &fr)
{
    if (fr.size() != 10 || quint8(fr[0]) != 0xAA || quint8(fr[9]) != 0x0A)
        return {};

    auto i16 = [&](int off) -> qint16 {
        return qFromBigEndian<qint16>(reinterpret_cast<const uchar*>(fr.constData() + off));
    };
    qint16 x  = i16(1);
    qint16 y  = i16(3);
    qint16 z  = i16(5);
    qint16 yaw= i16(7);

    QString ts = QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss >> 串口接收数据: ");
    return ts + QStringLiteral("x=%1cm  y=%2cm  z=%3cm  yaw=%4°")
                    .arg(x).arg(y).arg(z).arg(double(yaw), 0, 'f', 1);
}

// 初始化ui界面
SerialPort::SerialPort(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::SerialPort)
    , m_reconnectWarningShown(false)
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
    ui->ipInput->setEnabled(false);
    ui->ipInput->setText(ipText);
    ui->portInput->setEnabled(false);
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
                // 重置计数器
                tfCount = 0;
                scanCount = 0;
                mapCount = 0;
            }
        } else {
            ui->lblWifiState->setText("尝试自动重连");
            ui->lblWifiState->setStyleSheet("color:red");

            // 停止统计定时器
            if (statsTimer->isActive()) {
                statsTimer->stop();
            }
        }
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
    connect(statsTimer, &QTimer::timeout, this, &SerialPort::onStatsTimeout);

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

    elapsedTimer.start();
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
    delete ui;
}

// 数据发送函数
void SerialPort::sendData(const QByteArray &data)
{
    int bytesSent = 0;
    TcpClient* tcpClient = TcpClient::getInstance();

    if(isSerialPortConnected){
        qDebug() << "使用串口发送测试数据...";
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
        QString ip = "192.168.200.1";
        quint16 port = 12312;

        if (ip.isEmpty() || port == 0) {
            QMessageBox::warning(this, "警告", "请输入有效的目标IP和端口");
            return;
        }

        bytesSent = udpSocket->writeDatagram(data, QHostAddress(ip), port);
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
    QByteArray testFrame = ProtocolHandler::buildTestFrame();
    qDebug() << "发送测试数据中...."<< testFrame;
    sendData(testFrame);
    QString timestamp = QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss >> 测试操作: ");
    ui->recvEdit->append(timestamp + "已发送测试，等待模块回复...");
    testFlag = false;       // 重置测试标志并启动定时器
    testTimer->start(3000); // 3秒超时
}

// 测试超时处理函数
void SerialPort::onTestTimeout()
{
    if (!testFlag) {
        QString timestamp = QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss >> 测试操作: ");
        ui->recvEdit->append(timestamp + " 测试失败!!!");
        testFlag = false;
    }
}

// 数据处理/////////////////////////////////////////////////////////////////
// 数据处理
void SerialPort::processReceivedData(const QByteArray &recBuf)
{
    recvNum += recBuf.size();
    ui->recvNum->setText(QString("接收字节数量： %1").arg(recvNum));

    if (isSerialPortConnected) {
        recvBuffer.append(recBuf);
        while (recvBuffer.size() >= 10) {
            int head = recvBuffer.indexOf(char(0xAA));
            if (head < 0 || recvBuffer.size() - head < 10) break;
            QByteArray frame = recvBuffer.mid(head, 10);
            if (quint8(frame[0]) != 0xAA || quint8(frame[9]) != 0x0A) {
                recvBuffer.remove(head, 1);
                continue;
            }
            recvBuffer.remove(head, 10);
            QString line = uartFrameToText(frame);
            if (!line.isEmpty()) ui->recvEdit->append(line);
        }
    } else {
        /* ---------- 网口链路：保持原 ROS-JSON 逻辑 ---------- */
        recvBuffer.append(recBuf);
        QList<QByteArray> frames = ProtocolHandler::extractFramesFromBuffer(recvBuffer);
        for (const QByteArray &fr : frames)
            processProtocolFrame(fr);
    }
}

void SerialPort::processProtocolFrame(const QByteArray &frame)
{
    if (!ProtocolHandler::validateFrame(frame)) {
        qWarning() << "无效帧:" << frame.toHex(' ');
        return;
    }

    quint8 command = ProtocolHandler::getCommandType(frame);
    QByteArray payload = ProtocolHandler::extractPayload(frame);

    switch (command) {
    case ProtocolCommand::TEST:
        if (ProtocolHandler::parseTestResponse(frame)) {
            testFlag = true;
            testTimer->stop();
            QString timestamp = QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss >> 测试操作: ");
            ui->recvEdit->append(timestamp + " 测试成功，模块通讯正常!!");
        }
        break;

    case ProtocolCommand::PARAM_READ:{
        quint8 paramId;
        quint16 value;
        if (ProtocolHandler::parseParameterResponse(frame, paramId, value)) {
            emit parameterResponseReceived(frame);
        }
        break;
    }

    case ProtocolCommand::TF_DATA: {
        tfCount++;  // 增加TF计数
        QJsonObject jsonData = ProtocolHandler::parseRosData(payload);
        if (!jsonData.isEmpty()) {
            parseRosFrame(command, jsonData);
        }
        break;
    }

    case ProtocolCommand::SCAN_DATA: {
        scanCount++;  // 增加SCAN计数
        QJsonObject jsonData = ProtocolHandler::parseRosData(payload);
        if (!jsonData.isEmpty()) {
            parseRosFrame(command, jsonData);
        }
        break;
    }

    case ProtocolCommand::MAP_DATA: {
        mapCount++;  // 增加MAP计数
        QJsonObject jsonData = ProtocolHandler::parseRosData(payload);
        if (!jsonData.isEmpty()) {
            parseRosFrame(command, jsonData);
        }
        break;
    }

    default:
        qWarning() << "未知命令类型:" << command;
    }
}

// 测试数据解析函数
void SerialPort::parseTestData(const QByteArray &frame)
{
    // 检查是否为正确的测试回复数据: 0xAA 0x00 0x01 0x0A
    if (frame.size() == 4 &&
        static_cast<quint8>(frame.at(0)) == 0xAA &&
        static_cast<quint8>(frame.at(1)) == 0x00 &&
        static_cast<quint8>(frame.at(2)) == 0x01 &&
        static_cast<quint8>(frame.at(3)) == 0x0A) {

        // 设置测试标志
        testFlag = true;

        // 停止定时器
        testTimer->stop();

        // 在接收框显示成功信息
        ui->recvEdit->append("测试成功，模块通讯正常!!");

        // 重置标志位
        testFlag = false;
    }
}

// 定时任务
void SerialPort::onStatsTimeout()
{
    // 固定10秒时间窗口
    static const double TIME_WINDOW = 10.0;

    // 计算频率
    double tfFreq = tfCount / TIME_WINDOW;
    double scanFreq = scanCount / TIME_WINDOW;
    double mapFreq = mapCount / TIME_WINDOW;

    // 获取当前时间
    QString timestamp = QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss >> 话题统计: ");

    QString stats = QString("%1 /tf = %2 Hz, /scan = %3 Hz   /map = %4 Hz")
                        .arg(timestamp)
                        .arg(tfFreq, 0, 'f', 2)
                        .arg(scanFreq, 0, 'f', 2)
                        .arg(mapFreq, 0, 'f', 2);

    ui->recvEdit->append(stats);

    // 重置计数器
    tfCount = 0;
    scanCount = 0;
    mapCount = 0;
}

// 解析ros话题数据
void SerialPort::parseRosFrame(quint8 topicId, const QJsonObject &obj)
{
    switch (topicId)
    {

    case ProtocolCommand::TF_DATA: { // TF
        TFMessage::Transform t;
        t.header.frame_id = obj.value("frame_id").toString();
        t.child_frame_id = obj.value("child_frame_id").toString();
        t.x = obj.value("x").toDouble();
        t.y = obj.value("y").toDouble();
        t.z = obj.value("z").toDouble();

        if (obj.contains("qx")) {
            double qx = obj["qx"].toDouble();
            double qy = obj["qy"].toDouble();
            double qz = obj["qz"].toDouble();
            double qw = obj["qw"].toDouble();
            t.yaw = quaternionToYaw(qx, qy, qz, qw);
        }

        // 缓存每个变换
        tfCache[t.header.frame_id + "->" + t.child_frame_id] = t;

        // 当 map->odom 和 odom->base_link 都存在时，合成 map->base_link
        if (tfCache.contains("map->odom") && tfCache.contains("odom->base_link")) {
            auto mapOdom = tfCache["map->odom"];
            auto odomBase = tfCache["odom->base_link"];

            // yaw 角合成（简单近似）
            double yaw = mapOdom.yaw + odomBase.yaw;
            double cosY = cos(mapOdom.yaw);
            double sinY = sin(mapOdom.yaw);

            // 坐标合成： map→base_link = map→odom + (odom→base_link 在 map 坐标下的平移)
            double x = mapOdom.x + cosY * odomBase.x - sinY * odomBase.y;
            double y = mapOdom.y + sinY * odomBase.x + cosY * odomBase.y;
            double z = mapOdom.z + odomBase.z;   // ✅ z 直接线性叠加（通常odom/base_link只相差很小）

            TFMessage tf;
            TFMessage::Transform result;
            result.header.frame_id = "map";
            result.child_frame_id = "base_link";
            result.x = x;
            result.y = y;
            result.z = z;
            result.yaw = yaw;
            tf.transforms.append(result);

            emit rosTfUpdated(tf);
            // qDebug() << "TF合成为: map->base_link, x=" << x << " y=" << y << " z=" << z << " yaw=" << yaw;
        }

        break;
    }

    case ProtocolCommand::SCAN_DATA: { // Scan
        LaserScan scan;
        scan.angle_min = obj.value("angle_min").toDouble();
        scan.angle_max = obj.value("angle_max").toDouble();
        scan.angle_increment = obj.value("angle_increment").toDouble();
        scan.range_min = 0.05;
        scan.range_max = 30.0;
        QJsonArray arr = obj.value("ranges").toArray();
        for (const auto &v : arr)
            scan.ranges.append(v.toDouble());
        emit rosScanUpdated(scan);
        break;
    }

    case ProtocolCommand::MAP_DATA: { // Map

        OccupancyGrid map;
        map.width = obj.value("width").toInt();
        map.height = obj.value("height").toInt();
        map.resolution = obj.value("resolution").toDouble();
        map.origin_x = obj.value("origin_x").toDouble();
        map.origin_y = obj.value("origin_y").toDouble();
        // qDebug() << "绘制地图:" << obj;

        QJsonArray rle = obj.value("rle").toArray();
        map.data.reserve(map.width * map.height);
        for (auto entry : rle) {
            QJsonArray pair = entry.toArray();
            int value = pair.at(0).toInt();
            int count = pair.at(1).toInt();
            for (int i = 0; i < count; ++i)
                map.data.append(value);
        }

        emit rosMapUpdated(map);
        break;
    }

    default:
        qWarning() << "未知topicId:" << topicId;
    }
}

// 四元数转yaw，在解析ros话题数据中使用
double SerialPort::quaternionToYaw(double x, double y, double z, double w)
{
    // 四元数转偏航角 (yaw) - 绕Z轴旋转
    double siny_cosp = 2.0 * (w * z + x * y);
    double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    return std::atan2(siny_cosp, cosy_cosp);
}



// 打开网络通信UDP/TCP//////////////////////////////////////////////////////////
// 打开网络通信UDP/TCP
void SerialPort::on_wifiConnectBt_clicked()
{
    // 根据选择的协议启用相应控件
    QString protocol = ui->protocolComboBox->currentText();
    TcpClient* tcpClient = TcpClient::getInstance();

    if(ui->wifiConnectBt->text() == "打开连接"){
        // 清空输出窗口
        on_clearRecvBt_clicked();
        // 清空地图
        emit requestClearVisualization();
        // 先停止可能的重连
        tcpClient->stopAutoReconnect();
        // 重置'重连'警告标志
        m_reconnectWarningShown = false;
        // 禁用串口通信方式
        ui->serialBox->setEnabled(false);
        if (protocol == "TCP"){
            if (!tcpClient->isConnected()) {
                // QString ip = ui->ipInput->text();
                // quint16 port = ui->portInput->text().toUShort(&ok);
                // 固定模块IP 端口
                QString ipText = Config::instance().value("Network/tcp_ip", "127.0.0.1").toString();
                bool ok = false;
                quint16 port = Config::instance().value("Network/tcp_port", "6666").toString().toUShort(&ok);
                if (!ok) {
                    // 处理转换失败，例如使用默认值
                    port = 6666;
                }
                // QString ip = "10.42.0.1";
                // QString ip = "172.27.191.1";
                // quint16 port = 6666;


                if (!ok || port == 0) {
                    // 转换失败或端口号为0的处理
                    QMessageBox::warning(this, "错误", "请输入有效的端口号(1-65535)");
                    return;
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
                    ui->wifiConnectBt->setText("关闭连接");
                    ui->lblWifiState->setText("UDP已连接");
                    ui->lblWifiState->setStyleSheet("color:green");

                    // UDP连接成功时启动统计定时器
                    if (!statsTimer->isActive()) {
                        statsTimer->start();
                        elapsedTimer.restart();
                        // 重置计数器
                        tfCount = 0;
                        scanCount = 0;
                        mapCount = 0;
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

        // 重置统计计数器
        tfCount = 0;
        scanCount = 0;
        mapCount = 0;
        ui->wifiConnectBt->setText("打开连接");
        ui->serialBox->setEnabled(true);
        if (protocol == "TCP"){
            if (tcpClient->isConnected()){
                tcpClient->disconnectFromHost();
                tcpClient->stopAutoReconnect();
                // 断开连接后按钮文本已经是"打开连接"
            }
        }else if(protocol == "UDP"){
            if (isUdpBound) {
                udpSocket->close();
                isUdpBound = false;

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
