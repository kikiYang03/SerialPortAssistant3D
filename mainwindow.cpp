#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QThread>
#include <QInputDialog>
#include <QLineEdit>
#include <QMenu>       // 标题栏溢出菜单（管理员入口的宿主，工单 06）

MainWindow::MainWindow(QWidget *parent)
    : FramelessWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    // 无边框外壳（工单 04）：标题栏挂在 QMainWindow::setMenuWidget() 上，
    // centralWidget 布局与状态栏都不动。标题栏内容归 ui/shell/TitleBar（工单 05）：
    // 品牌 + 版本 + 串口状态胶囊 + 三键 + 一个留给 06 的空溢出菜单。
    titleBar = new TitleBar(this);
    setTitleBar(titleBar);

    // 底部状态栏（工单 07）：.ui 里那条空 QStatusBar 换成自绘两段版
    // （串口 · 控制链路；TF 帧名那一段已按现场要求删除）。两段的数据源各一处 ——
    // 串口来自 portStateChanged、控制链路来自 controlLinkTextChanged；本窗口只接线，
    // 不在这里再算一遍任何一段的状态。
    statusBarEx = new StatusBarEx(this);
    setStatusBar(statusBarEx);

    // 实例化页面
    serialPort = new SerialPort;
    params = new Params;
    ros3dPage = new Ros3DPage;
    serialPort->setGLWidget(ros3dPage->glWidget());

    // 标题栏胶囊订阅串口链路状态：与串口页自己的标签同一个信号源，
    // 不会出现「页面说已连接、标题栏说未连接」（工单 05）。
    connect(serialPort, &SerialPort::portStateChanged,
            titleBar, &TitleBar::setPortState);

    // 状态栏第一段同理：同一个信号、同一段文字（端口 · 波特率 · 状态）。
    // 第二段订阅控制链路的那条文案信号，文案本身已经区分 TCP / UDP。
    connect(serialPort, &SerialPort::portStateChanged,
            statusBarEx, &StatusBarEx::setPortState);
    connect(serialPort, &SerialPort::controlLinkTextChanged,
            statusBarEx, &StatusBarEx::setControlLinkState);


    // 添加子页面
    ui->stackedWidget->addWidget(serialPort);
    ui->stackedWidget->addWidget(params);
    ui->stackedWidget->addWidget(ros3dPage);

    // 设置当前页面
    ui->stackedWidget->setCurrentWidget(serialPort);

    // 标题栏上不要系统右击菜单（外壳自己就是标题栏）
    this->setContextMenuPolicy(Qt::NoContextMenu);

    // 命令栏（工单 06）：替掉菜单栏 + 工具栏。原来挂在工具栏上的三页切换变成左段
    // 分段控件；右侧多了「读取参数」与「连接 / 断开设备」。命令栏只发请求，
    // 堆栈、参数页、串口各自仍是自己那份逻辑的持有者。
    CommandBar *cmdBar = ui->commandBar;

    // 页切换：命令栏点段 → 堆栈换页；反过来堆栈是权威，换到哪一页命令栏就显示哪一段。
    // 两条连线的方向是单向的（activate → 堆栈 → currentChanged → 命令栏），不会绕成环。
    connect(cmdBar, &CommandBar::pageSelected,
            ui->stackedWidget, &QStackedWidget::setCurrentIndex);
    connect(ui->stackedWidget, &QStackedWidget::currentChanged,
            cmdBar, &CommandBar::setCurrentPage);
    cmdBar->setCurrentPage(ui->stackedWidget->currentIndex());

    // 「读取参数」：直接复用参数页既有的读取入口（它自己数条数、自己写状态行），
    // 命令栏不重复实现一遍 —— 也不必先切到参数页。
    connect(cmdBar, &CommandBar::readParamsRequested,
            params, &Params::requestRead);

    // 主按钮：开关串口的逻辑只有 SerialPort 那一份，命令栏只发一条「toggle」请求
    connect(cmdBar, &CommandBar::toggleConnectionRequested,
            serialPort, &SerialPort::togglePortConnection);

    // 主按钮长什么样只由串口的真实状态决定（不在这里记一个本地布尔）
    connect(serialPort, &SerialPort::portStateChanged, cmdBar,
            [cmdBar](const QString &, bool connected) {
                cmdBar->setPortConnected(connected);
            });

    // 管理员入口迁移：原来挂在 QToolBar 上，现在挂进标题栏右上角的溢出菜单
    // （05 预留的空菜单）。config.ini 的 Admin/enabled 判断与消息转发逻辑原样保留。
    QAction *adminLogin = new QAction("管理员登录/退出", this);
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

    // 管理者模式入口：由 config.ini 的 Admin/enabled 控制，false 时整条入口不显示
    // （菜单里空着时标题栏那个溢出按钮也不显示，见 TitleBar::syncOverflowEntry）
    if (AdminMode::instance().isEnabled())
        titleBar->overflowMenu()->addAction(adminLogin);

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
    // 页面切换的连线已在上面交给命令栏（工单 06）
    // ----------------------------------------------------------

    connect(serialPort, &SerialPort::tcpConnectionChanged,
            protocolHandler, &ProtocolRos3D::setLinkAlive);

    // 连接Params的消息信号到SerialPort的显示槽
    connect(params, &Params::appendMessage, serialPort, &SerialPort::appendMessage);
    // 连接解析数据的频率到SerialPort
    connect(protocolHandler, &ProtocolRos3D::appendMessage, serialPort, &SerialPort::appendMessage);
    // 连接 GLWidget 的消息信号到 SerialPort 的显示槽
    connect(ros3dPage->glWidget(), &GLWidget::appendMessage,
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

    // 「获取参数」：参数页只报出「它的表里有多少条」，帧的发送与「收齐了没」的判定都在链路层
    // （0x03 的应答没有终结帧，只能按条数收齐——参数页自己数不了到达的帧）
    connect(params, &Params::readParamsRequested,
            serialPort, &SerialPort::onReadParamsRequested);

    // 读取闭环的结论摘要同样落在参数页的状态行上
    connect(serialPort, &SerialPort::readParamsFinished,
            params, &Params::setReadSummary);



}

MainWindow::~MainWindow()
{
    delete ui;
}
