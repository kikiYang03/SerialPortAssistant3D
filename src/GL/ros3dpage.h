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
class QGroupBox;

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

public slots:
    // 状态帧 (0x0B)：重定位与规划上标签（串口链已有飞控状态显示，
    // 两条链的飞控状态如何合并显示是独立话题）
    void onNavStatusUpdated(const NavStatusMsg& msg);

    // 帧名不符 (工单 05)：该帧已在解析层被丢弃，这里只把红状态位点亮、写一行日志。
    // **不弹窗、不阻塞**：联调时两端不同版，操作员该继续能操作，只是知道对不上。
    // 三个参数是**载荷里读到的**帧号与帧名，界面照原样显示，不写期望值
    // （期望值只有一份，在 tftree.cpp 那张对照表里，见 protocolros3d.h 的信号声明）。
    void onFrameNameRejected(quint8 frameNo, const QString& frameId, const QString& childFrameId);

    // 2D 扫描「现在画不出来」的说明（工单 07）：**空串 = 正常**，把标签收起来。
    // 与上面那个红状态位不同，它说的是**当前状态**（`0x01` 陈旧 ⇒ 扫描摆不进图里），
    // 不是一次事件的留痕：位姿回来、状态解除，标签就该消失。
    void onScan2DNoteChanged(const QString &note);

    // 开机点（工单 09）：`shown` = 画着，把 `x` / `y` / `yawDeg` 三个数摆出来；
    // 不画时 `note` 就是「为什么不画」那句话（建图模式 / 0x02 还没到），**也可能是空串**
    // —— 那是「本会话还没读到过模式参数（0x13）」，界面上什么都不显。
    // 与上面那位一样，画不画与文案都由 GLWidget 按模块的判据算好，这里只负责摆 ——
    // 而这个槽还要多做一件事：不画的时候把整块数值**收起来**（见 .cpp 里的说明）。
    void onOriginMarkerChanged(bool shown, const QString &note, double x, double y, double yawDeg);

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

    // 帧名校验 (工单 05)：不符时的**红状态位**，就在上面那两块状态的同一片显示区里。
    // 平时整块藏起来（正常状态不需要占一行），只有真的不符才显形 —— 它是一次
    // 「留痕」，不是常驻读数：看见它变红就说明两端帧名口径对不上了。
    // 亮起来后**一直挂着**（这是留痕该有的样子：走开一会儿回来还看得见），
    // 只在模块重启判定成功时随 clearModuleState() 一起清掉——那一刻旧模块的结论作废。
    QLabel* labFrameNameTitle_  = nullptr;
    QLabel* labFrameNameStatus_ = nullptr;

    // 2D 扫描「为什么不画」（工单 07）：挂在「点云显示」那一块下面——它管的正是
    // 「图上画不画」这件事。平时 hide()：正常时扫描画着，这话没有说出来的必要。
    QLabel* labScan2DNote_ = nullptr;

    // 开机点（工单 09）：本次开机那一刻机器人中心在图里的位置。与「机器人坐标」是同一类
    // **读数**（都是位姿），所以整块紧跟在它下面；而「为什么不显示」那句话挂在
    // 「点云显示」那一块（与 07 的扫描说明同一个位置，两处都是「图上画不画」）——
    // 但**不是每个「不显示」都有话说**：还没读到过模式参数（0x13）那种，两处都空着。
    //
    // 整块平时收起来：只有定位模式且 `0x02` 到过才有得显示。不画时**连数值一起收**，
    // 不留一行占位 —— 「没有残留数值」这条因此是结构上成立的，不靠谁记得去清。
    QGroupBox* originBox_ = nullptr;
    QLabel* labOriginX_   = nullptr;
    QLabel* labOriginY_   = nullptr;
    QLabel* labOriginYaw_ = nullptr;
    QLabel* labOriginNote_ = nullptr;

    // 上一包的值；-1 表示还没收到过任何一包。
    // 「一包都没收到」≠「模块明说未响应（3）」：模块听到过任一上游自述之前一包都不发，
    // 所以三位都取 -1 时标签停在「未收到…」。飞控那件不占格子，锁存只用来在重启后归位。
    int lastNavState_   = -1;
    int lastRelocState_ = -1;
    int lastFcState_    = -1;

    // 上一次「帧名不符」留在侧边栏上的文案。不符的帧是按 10 Hz 以上重发的，同一个不符
    // 每帧都刷一遍标签都是白做 —— 只在**变化时**动一次。
    // 空 = 本次会话还没见过不符（也可能见过、后被 clearModuleState() 复位）。
    QString lastFrameNameMismatch_;

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
