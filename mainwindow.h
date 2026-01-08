#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QMenuBar>
#include <QToolBar>
#include <QMessageBox>
#include <QDebug>


// 引入子页面
#include <coordinate.h>
#include <serialport.h>
#include <params.h>
#include <protocolros3d.h>
#include <glwidget.h>

QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    SerialPort * serialPort;
    Coordinate * coordinate;
    Params * params;
    ProtocolRos3D * protocolHandler;
    GLWidget *glWidget;

private:
    Ui::MainWindow *ui;


};
#endif // MAINWINDOW_H
