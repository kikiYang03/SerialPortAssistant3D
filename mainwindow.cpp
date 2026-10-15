#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QThread>
#include <QInputDialog>
#include <QLineEdit>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    // 实例化页面
    serialPort = new SerialPort;
    params = new Params;
    ros3dPage = new Ros3DPage;
    serialPort->setGLWidget(ros3dPage->glWidget());


    // 添加子页面
    ui->stackedWidget->addWidget(serialPort);
    ui->stackedWidget->addWidget(params);
    ui->stackedWidget->addWidget(ros3dPage);

    // 设置当前页面
    ui->stackedWidget->setCurrentWidget(serialPort);

    // 隐藏菜单栏上的右击菜单
    this->setContextMenuPolicy(Qt::NoContextMenu);

    // 创建基础顶部菜单并让其隐藏
    QMenuBar *bar = menuBar();
    this->setMenuBar(bar);
    QMenu * fileMenu = bar->addMenu("Ptr");

    // 隐藏菜单
    bar->setVisible(false);

    // 添加子菜单
    QAction *NewAction = fileMenu->addAction("连接设置");
    QAction *ReadAction = fileMenu->addAction("参数设置");
    QAction *Ros3DAction = fileMenu->addAction("可视化");

    // 创建工具栏
    QToolBar *toolBar = new QToolBar(this);
    addToolBar(Qt::TopToolBarArea,toolBar);

    // 将菜单项依次添加到工具栏
    toolBar->addAction(NewAction);
    toolBar->addAction(ReadAction);
    toolBar->addAction(Ros3DAction);

    // 设置禁止移动属性,工具栏默认贴在上方
    toolBar->setFloatable(false);
    toolBar->setMovable(false);
    toolBar->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);

    // mainwindow.cpp 构造函数追加
    QAction *adminLogin = new QAction("管理员登录/退出");
    connect(adminLogin, &QAction::triggered, this, [](){
        if (AdminMode::instance().isAdmin()) {
            AdminMode::instance().logout();
        } else {
            bool ok;
            QString pwd = QInputDialog::getText(nullptr, "管理员登录", "请输入密码：",
                                                QLineEdit::Password, "", &ok);
            if (ok) AdminMode::instance().login(pwd);
        }
    });
    connect(&AdminMode::instance(), &AdminMode::logAdminMessage,
            serialPort, &SerialPort::appendMessage);

    // 管理者模式，不需要则注释掉
    toolBar->addAction(adminLogin);

    //==========================================================
    // 1) 创建协议处理线程（ProtocolRouter + ProtocolRos3D 都在此线程）
    QThread* protoThread = new QThread(this);

    // 2) ProtocolRouter 单例移到工作线程
    ProtocolRouter* router = ProtocolRouter::instance();
    router->moveToThread(protoThread);

    // 3) ProtocolRos3D 也移到同一工作线程
    protocolHandler = new ProtocolRos3D;
    protocolHandler->moveToThread(protoThread);

    // 4) 线程退出时清理
    connect(protoThread, &QThread::finished, protocolHandler, &QObject::deleteLater);
    protoThread->start();

    //==========================================================
    // 数据流：SerialPort(主线程) -> ProtocolRouter(工作线程) -> ProtocolRos3D(工作线程) -> GLWidget(主线程)

    // SerialPort只负责原始数据收发，直接发给工作线程处理
    connect(serialPort, &SerialPort::rawBytesArrived,
            router, &ProtocolRouter::processDataStream,
            Qt::QueuedConnection);  // 跨线程，用队列连接

    // 测试指令处理（回到主线程）
    connect(router, &ProtocolRouter::testFrameReceived,
            serialPort, &SerialPort::handleTestFrame,
            Qt::QueuedConnection);

    // ROS数据交给ProtocolRos3D处理（都在同一工作线程，直接连接）
    connect(router, &ProtocolRouter::ros3dDataReceived,
            protocolHandler, &ProtocolRos3D::onRawBytes,
            Qt::DirectConnection);  // 同一线程，直接调用更高效

    // 参数数据（回到主线程）
    connect(router, &ProtocolRouter::parameterFrameReceived,
            params, &Params::updateParameter,
            Qt::QueuedConnection);

    //==========================================================
    // 解析结果 -> 渲染（回到主线程，QueuedConnection）
    connect(protocolHandler, &ProtocolRos3D::robotPoseUpdated,
            ros3dPage->glWidget(), &GLWidget::onRobotPose, Qt::QueuedConnection);

    connect(protocolHandler, &ProtocolRos3D::relocalizationPoseUpdated,
            ros3dPage->glWidget(), &GLWidget::onRelocalizationPose, Qt::QueuedConnection);

    connect(protocolHandler, &ProtocolRos3D::cloudUpdated,
            ros3dPage->glWidget(), &GLWidget::onCloud, Qt::QueuedConnection);

    // === 连接目标轨迹信号 ===
    connect(protocolHandler, &ProtocolRos3D::goalPathUpdated,
            ros3dPage->glWidget(), &GLWidget::onGoalPath, Qt::QueuedConnection);

    // === 连接2D数据信号 ===
    connect(protocolHandler, &ProtocolRos3D::scan2DUpdated,
            ros3dPage->glWidget(), &GLWidget::onScan2D, Qt::QueuedConnection);

    connect(protocolHandler, &ProtocolRos3D::map2DUpdated,
            ros3dPage->glWidget(), &GLWidget::onMap2D, Qt::QueuedConnection);

    // === 位置控制指令 (0x08)：不丢帧、可观测（不在 3D 视图里画） ===
    connect(protocolHandler, &ProtocolRos3D::posCmdUpdated,
            ros3dPage, &Ros3DPage::onPosCmdUpdated, Qt::QueuedConnection);

    // === 参数 0x13「模式」(建图 / 定位，未读到过为 -1)：串口层在收到 AA 10 时就地缓存
    // （与 0x99 同一条路径），变化时下发给图形层，由工单 09 判要不要画开机点标记 ===
    connect(serialPort, &SerialPort::moduleModeChanged,
            ros3dPage->glWidget(), &GLWidget::onModuleModeChanged);

    // === 状态帧 (0x0B)：重定位 / 规划上标签，飞控只写日志，三者都只在跳变时动 ===
    connect(protocolHandler, &ProtocolRos3D::navStatusUpdated,
            ros3dPage, &Ros3DPage::onNavStatusUpdated, Qt::QueuedConnection);

    // === 帧名校验 (工单 05)：不符的帧在解析层已被丢弃，这里只把红状态位点亮 + 写一行日志。
    // 刻意不弹窗、不阻塞：联调时两端不同版，操作员该继续能操作，看见红字就知道对不上 ===
    connect(protocolHandler, &ProtocolRos3D::frameNameRejected,
            ros3dPage, &Ros3DPage::onFrameNameRejected, Qt::QueuedConnection);

    // ----------------------------------------------------------
    // 绑定槽函数——显示页面
    // ----------------------------------------------------------
    connect(NewAction,&QAction::triggered,this,[=](){
        ui->stackedWidget->setCurrentIndex(0);
    });
    connect(ReadAction,&QAction::triggered,this,[=](){
        ui->stackedWidget->setCurrentIndex(1);
    });
    connect(Ros3DAction,&QAction::triggered,this,[=](){
        ui->stackedWidget->setCurrentIndex(2);
    });

    connect(serialPort, &SerialPort::tcpConnectionChanged,
            protocolHandler, &ProtocolRos3D::setLinkAlive);

    // 连接Params的消息信号到SerialPort的显示槽
    connect(params, &Params::appendMessage, serialPort, &SerialPort::appendMessage);
    // 连接解析数据的频率到SerialPort
    connect(protocolHandler, &ProtocolRos3D::appendMessage, serialPort, &SerialPort::appendMessage);
    // 连接 GLWidget 的消息信号到 SerialPort 的显示槽
    connect(ros3dPage->glWidget(), &GLWidget::appendMessage,
            serialPort, &SerialPort::appendMessage);

    // 连接可视化页的消息信号（规划状态跳变日志）到 SerialPort 的显示槽
    connect(ros3dPage, &Ros3DPage::appendMessage,
            serialPort, &SerialPort::appendMessage);

    connect(ros3dPage, &Ros3DPage::sendNavGoalRequested,
            serialPort, &SerialPort::onSendNavGoalRequested);

    // 「保存地图」按钮：请求交给 SerialPort 发 AA 00 02 0A
    connect(ros3dPage, &Ros3DPage::saveMapRequested,
            serialPort, &SerialPort::onSaveMapRequested);

    // 「下载地图」按钮：同样只发请求，帧、超时、进度框、校验与落盘都在 SerialPort 里
    connect(ros3dPage, &Ros3DPage::downloadMapRequested,
            serialPort, &SerialPort::onDownloadMapRequested);

    // 地图下传分块 (0x0C)：解析在工作线程（ProtocolRos3D），攒块与校验在主线程的
    // SerialPort 里。跨线程，用队列连接——下载流程要碰界面（进度框、文件对话框）。
    connect(protocolHandler, &ProtocolRos3D::mapChunkUpdated,
            serialPort, &SerialPort::onMapChunkReceived, Qt::QueuedConnection);

    // 未连接时「保存地图」不可点（控制指令链路 = TCP / UDP，串口不算）
    connect(serialPort, &SerialPort::controlLinkStateChanged,
            ros3dPage, &Ros3DPage::setLinkConnected);

    // 「写入参数」：参数页只收集值，帧由 SerialPort 统一发送（发送方与等待方同处一地）
    connect(params, &Params::writeParamsRequested,
            serialPort, &SerialPort::onWriteParamsRequested);

    // 未连接时「写入参数」也不可点，门禁与「保存地图」同一个信号源
    connect(serialPort, &SerialPort::controlLinkStateChanged,
            params, &Params::setLinkConnected);

    // 串口页的「重启模块」同走这道门禁（第三个接收方，信号源不变）
    connect(serialPort, &SerialPort::controlLinkStateChanged,
            serialPort, &SerialPort::setLinkConnected);

    // 重启模块判定成功：3D 页面把「模块运行态的镜像」清掉 —— 3D 视图（点云 / 地图 / 位姿 /
    // 拖尾 / 指点目标点几何）连同 TF 数值标签、规划与重定位状态标签、两者的跳变锁存。
    // 串口页自己的缓存由 SerialPort 就地清（同一个成功分支里），不走信号。
    // 失败路径不发这个信号：超时意味着模块很可能根本没重启，清掉就是毁掉还在用的数据。
    connect(serialPort, &SerialPort::restartSucceeded,
            ros3dPage, &Ros3DPage::clearModuleState);

    // 保存参数等待期：模块回读值只喂链路层的比对器，不刷参数页的界面
    // （否则用户会在结果弹出前就以为保存成功了，而那正是要抓的场景）
    connect(serialPort, &SerialPort::paramEchoSuppressedChanged,
            params, &Params::setEchoSuppressed);

    // 保存闭环的结论摘要落在参数页的状态行上（结果本身由链路层弹窗给出）
    connect(serialPort, &SerialPort::saveParamsFinished,
            params, &Params::setSaveSummary);



}

MainWindow::~MainWindow()
{
    delete ui;
}
