#ifndef COMMANDBAR_H
#define COMMANDBAR_H

#include <QWidget>

class IconButton;
class SegmentedControl;

// 命令栏（工单 06）：替掉「菜单栏 + 工具栏」，把三页切换与两个最常用操作放到同一行。
//
// 左段 = 自绘分段控件（连接设置 / 参数设置 / 可视化），右段 = 「读取参数」+ 主按钮
// 「连接设备 ⇄ 断开设备」。
//
// 本类**不查连接、不发帧、不碰串口**，只发三条请求：
//   * pageSelected(index)            —— 由 mainwindow 送去中央堆栈；
//   * readParamsRequested()          —— 接参数页既有的读取入口，不必先切页；
//   * toggleConnectionRequested()    —— 开关逻辑仍是 SerialPort 里那一份，这里不重写。
//
// 主按钮长什么样**只由串口的真实状态决定**（setPortConnected 由 SerialPort::portStateChanged
// 驱动），本类不记「上次是不是连上了」这种本地布尔 —— 本地布尔迟早与链路真实状态脱节，
// 于是出现「按钮写着断开、其实早就掉了」。
class CommandBar : public QWidget
{
    Q_OBJECT

public:
    explicit CommandBar(QWidget *parent = nullptr);

    SegmentedControl *navigation() const { return seg_; }

public slots:
    // 当前页：由中央堆栈反向驱动（堆栈才是权威，命令栏只跟着显示自己的选中态）
    void setCurrentPage(int index);

    // 串口链路状态：决定主按钮的文字与外观
    void setPortConnected(bool connected);

signals:
    void pageSelected(int index);
    void readParamsRequested();
    void toggleConnectionRequested();

protected:
    void paintEvent(QPaintEvent *e) override;

private:
    SegmentedControl *seg_ = nullptr;
    IconButton *readButton_ = nullptr;
    IconButton *connectButton_ = nullptr;
};

#endif // COMMANDBAR_H
