#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    // 实例化页面
    serialPort = new SerialPort;
    params = new Params;
    visualizer3d = new ROSVisualizer3D;
    protocolHandler = new ProtocolRos3D;


    // 添加子页面
    ui->stackedWidget->addWidget(serialPort);
    ui->stackedWidget->addWidget(params);
    ui->stackedWidget->addWidget(visualizer3d);

    // 设置当前页面
    ui->stackedWidget->setCurrentWidget(serialPort);

    // 隐藏菜单栏上的右击菜单
    this->setContextMenuPolicy(Qt::NoContextMenu);


    // ----------------------------------------------------------
    // 创建menuBar组件
    // ----------------------------------------------------------
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



    // 连接信号
    connect(serialPort, &SerialPort::rawBytesArrived,
            protocolHandler, &ProtocolRos3D::onRawBytes);

    connect(protocolHandler, &ProtocolRos3D::tfUpdated,
            visualizer3d, &ROSVisualizer3D::onTFUpdated);

    connect(protocolHandler, &ProtocolRos3D::cloudUpdated,
            visualizer3d, &ROSVisualizer3D::onCloudUpdated);

    connect(protocolHandler, &ProtocolRos3D::mapUpdated,
            visualizer3d, &ROSVisualizer3D::onMapUpdated);

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

    // 连接Params的消息信号到SerialPort的显示槽
    connect(params, &Params::appendMessage, serialPort, &SerialPort::appendMessage);

    // // 连接参数响应信号
    // connect(serialPort, &SerialPort::parameterResponseReceived,
    //         params, &Params::onParameterResponseReceived);
}

MainWindow::~MainWindow()
{
    delete ui;
}
