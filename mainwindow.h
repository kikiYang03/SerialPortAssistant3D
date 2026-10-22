#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QAction>
#include <QMessageBox>
#include <QDebug>

// 无边框窗口基座（工单 04）：不设 FramelessWindowHint，只在 WM_NCCALCSIZE 里
// 把客户区撑满整个窗口，系统投影 / 贴边吸附 / Win+方向键照旧。
#include "FramelessWindow.h"
// 自绘标题栏（工单 05）：品牌 + 版本 + 串口状态胶囊 + 三键 + 溢出菜单
#include "TitleBar.h"
// 命令栏（工单 06）：三段切换 + 读取参数 + 连接/断开
#include "CommandBar.h"
// 两段状态栏（工单 07）：串口 · 控制链路
#include "StatusBarEx.h"

// 引入子页面
#include <coordinate.h>
#include <serialport.h>
#include <params.h>
#include <protocolros3d.h>
#include <glwidget.h>
#include <ros3dpage.h>
#include <protocolrouter.h>
#include "adminmode.h"

QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public FramelessWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    SerialPort * serialPort;
    Coordinate * coordinate;
    Params * params;
    ProtocolRos3D * protocolHandler;
    Ros3DPage *ros3dPage;
    TitleBar *titleBar;     // 外壳的标题栏（工单 05）
    StatusBarEx *statusBarEx; // 外壳的状态栏（工单 07）

private:
    Ui::MainWindow *ui;


};
#endif // MAINWINDOW_H
