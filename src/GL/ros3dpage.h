#ifndef ROS3DPAGE_H
#define ROS3DPAGE_H
#pragma once
#include <QWidget>
#include <QCheckBox>
#include <QSlider>
#include <QDoubleSpinBox>
#include "dualrangeslider.h"
#include "glwidget.h"
#include "protocol_msg.h"

class GLWidget;
class QLabel;
class QPushButton;

class Ros3DPage : public QWidget
{
    Q_OBJECT
public:
    explicit Ros3DPage(QWidget* parent = nullptr);

    GLWidget* glWidget() const { return gl_; }

signals:
    void sendNavGoalRequested(double x, double y, double z, double yaw_deg);

    // 「保存地图」：发给 SerialPort，由它发 AA 00 02 0A 并等回包
    void saveMapRequested();

    // 「下载地图」：发给 SerialPort，由它发 AA 00 05 0A、收 0x0C 分块、校验后落盘
    void downloadMapRequested();

    // 需要补一行日志（如规划状态跳变），由 MainWindow 接到日志窗口
    void appendMessage(const QString &message);

public slots:
    // 状态帧 (0x0B)：重定位与规划上标签，飞控只写日志（串口链已有飞控状态显示，
    // 两条链的飞控状态如何合并显示是独立话题）
    void onNavStatusUpdated(const NavStatusMsg& msg);
    void onPosCmdUpdated(const PosCmdMsg& msg);        // 位置控制指令 (0x08)

    // 链路状态：未连接时「保存地图」与「下载地图」都不可点（两个按钮共用这一条门禁）
    void setLinkConnected(bool connected);

    // 模块重启判定成功后：清掉「模块运行态的镜像」（工单 02）—— 3D 视图那一片走既有的
    // gl_->clearMap()，加上 TF 数值标签、规划 / 重定位状态标签、以及两者的内部跳变锁存。
    // 只由 SerialPort 的 restartSucceeded 触发；超时失败路径不调用（那时数据依然有效）。
    void clearModuleState();

private:
    GLWidget* gl_ = nullptr;

    // TF 显示
    QLabel* labX_   = nullptr;
    QLabel* labY_   = nullptr;
    QLabel* labZ_   = nullptr;
    QLabel* labYaw_ = nullptr;
    QLabel* labPitch_ = nullptr;
    QLabel* labRoll_  = nullptr;

    // Z轴范围控制
    QCheckBox* ckZFilter_ = nullptr;
    DualRangeSlider* dualSlider_ = nullptr;  // 双滑块

    // 目标点控制组件
    QPushButton* btnNavGoal_ = nullptr;
    QDoubleSpinBox* spinNavX_ = nullptr;
    QDoubleSpinBox* spinNavY_ = nullptr;
    QDoubleSpinBox* spinNavZ_ = nullptr;
    QDoubleSpinBox* spinNavYaw_ = nullptr;

    QPushButton* btnSendNavGoal_ = nullptr;

    // 保存地图（模块侧落盘，见 AA 00 02 0A）
    QPushButton* btnSave_ = nullptr;

    // 下载地图（模块侧打包分块下传，见 AA 00 05 0A）：紧邻「保存地图」，
    // 共用同一条连接门禁（控制指令只走 TCP / UDP）
    QPushButton* btnDownload_ = nullptr;

    // 状态帧 (0x0B)
    QLabel* labNavStatus_   = nullptr;  // 规划状态
    QLabel* labRelocStatus_ = nullptr;  // 重定位状态，与规划状态并列

    // 上一包的值；-1 表示还没收到过任何一包。
    // 「一包都没收到」≠「模块明说未响应（3）」：模块听到过任一上游自述之前一包都不发，
    // 所以三位都取 -1 时标签停在「未收到…」。飞控那件不占格子，只用来判跳变写日志。
    int lastNavState_   = -1;
    int lastRelocState_ = -1;
    int lastFcState_    = -1;

    // 位置控制指令 (0x08)：100 Hz 逐帧期望位置，数值日志由解析端节流，
    // 这里只在首次收到时写一行，确认这一路确实接上了
    bool posCmdSeen_ = false;

private slots:
    void onDualRangeChanged(double lower, double upper);  // 双滑块的取值

    void onNavTargetUpdated(double x, double y, double z, double yaw_deg);
    void onNavSpinBoxChanged();

    void onSendNavGoalClicked();

private:
    static QString navStatusText(int state);    // 规划四档 → 中文，未知值单独显示
    static QString relocStatusText(int state);  // 重定位四档 → 中文，未知值单独显示
    static QString fcStatusText(int state);     // 飞控五档 → 中文（只进日志）
};

#endif // ROS3DPAGE_H
