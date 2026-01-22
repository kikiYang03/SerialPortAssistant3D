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

    ProtocolRouter* router = ProtocolRouter::instance();

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
    QAction *Ros3DAction = fileMenu->addAction("3D可视化");

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

    toolBar->addAction(adminLogin);   // 或者加到菜单栏
    // 1) 创建线程
    QThread* protoThread = nullptr;
    protoThread = new QThread(this);

    protocolHandler = new ProtocolRos3D;
    protocolHandler->moveToThread(protoThread);

    // 2) 线程退出时清理
    connect(protoThread, &QThread::finished, protocolHandler, &QObject::deleteLater);
    protoThread->start();

    //==========================================================
    // SerialPort只负责原始数据收发
    connect(serialPort, &SerialPort::rawBytesArrived,
            router, &ProtocolRouter::processDataStream,
            Qt::QueuedConnection);

    // 测试指令处理
    connect(router, &ProtocolRouter::testFrameReceived,
            serialPort, &SerialPort::handleTestFrame);

    // ROS数据交给ProtocolRos3D处理
    connect(router, &ProtocolRouter::ros3dDataReceived,
            protocolHandler, &ProtocolRos3D::onRawBytes);

    // 参数数据
    connect(router, &ProtocolRouter::parameterFrameReceived,
            params, &Params::updateParameter);

    // 4) 解析结果 -> 渲染（回主线程 queued）
    connect(protocolHandler, &ProtocolRos3D::tfUpdated,
            ros3dPage->glWidget(), &GLWidget::onTf, Qt::QueuedConnection);

    connect(protocolHandler, &ProtocolRos3D::cloudUpdated,
            ros3dPage->glWidget(), &GLWidget::onCloud, Qt::QueuedConnection);

    connect(protocolHandler, &ProtocolRos3D::mapCloudUpdated,
            ros3dPage->glWidget(), &GLWidget::onMap, Qt::QueuedConnection);

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


}

MainWindow::~MainWindow()
{
    delete ui;
}
