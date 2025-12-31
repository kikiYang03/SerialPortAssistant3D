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
#include <rosvisualizer3d.h>
#include <protocolros3d.h>

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
    ROSVisualizer3D * visualizer3d;
    ProtocolRos3D * protocolHandler;

private:
    Ui::MainWindow *ui;


};
#endif // MAINWINDOW_H
