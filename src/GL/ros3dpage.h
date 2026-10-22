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
class CollapsibleSection;   // 右面板折叠分组（工单 08）
class PanelHost;            // 右面板两态容器：展开 = 滚动区 / 收起 = 26px 窄条（工单 09）
class QEvent;
class QResizeEvent;
class QSplitter;
class QTimer;
class FloatingBar;
class PointsCapsule;

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

    // 开机点（工单 09）：`shown` = 画着，把 `x` / `y` / `yawDeg` 三个数摆出来；
    // 不画时 `note` 就是「为什么不画」那句话（建图模式 / 0x02 还没到），**也可能是空串**
    // —— 那是「本会话还没读到过模式参数（0x13）」，界面上什么都不显。
    // 与上面那位一样，画不画与文案都由 GLWidget 按模块的判据算好，这里只负责摆 ——
    // 而这个槽还要多做一件事：不画的时候把整块数值**收起来**（见 .cpp 里的说明）。
    void onOriginMarkerChanged(bool shown, const QString &note, double x, double y, double yawDeg);

    // 指令链路状态（TCP / UDP / 串口任一）：未连接时「保存地图」不可点。
    // 串口自协议 v2.9 起也能存图（命令帧 0x00 的子命令 02），所以这里只判「有没有链路」。
    void setLinkConnected(bool connected);

    // 「下载地图」的专属门禁：**只认 TCP / UDP**。串口的命令面没有 `05`（下传地图），
    // 发过去模块会静默丢弃，所以这颗键不能跟「保存地图」共用同一条判据。
    void setDownloadAvailable(bool available);

    // 模块重启判定成功后：清掉「模块运行态的镜像」（工单 02）—— 3D 视图那一片走既有的
    // gl_->clearMap()，加上 TF 数值标签、规划 / 重定位状态标签、以及两者的内部跳变锁存。
    // 只由 SerialPort 的 restartSucceeded 触发；超时失败路径不调用（那时数据依然有效）。
    void clearModuleState();

protected:
    // 视口浮层（工单 10）的定位时机之一：本页大小变了，视口跟着变。
    // 另一个时机（拖分隔条只改视口、不改本页大小）走 gl_ 的 resize 事件过滤器，
    // 见 .cpp 里 relayoutOverlays() 的说明。
    void resizeEvent(QResizeEvent *e) override;

    // 只盯 gl_ 的 QEvent::Resize：其余事件原样放行（不改 glwidget.cpp 的一个替代做法）。
    bool eventFilter(QObject *watched, QEvent *e) override;

private:
    // 把两个浮层挪到该在的位置。纯几何换算，不碰绘制、不碰渲染。
    void relayoutOverlays();

    // 收起/展开之后重排分裂器：把手宽度、两段宽度都在这儿定（纯换算在 PanelWidth 里）。
    // 收起来时把宽度还给视口，展开时把宽度还给面板 —— 两件事都只走这一个入口。
    void applyPanelLayout();

    GLWidget* gl_ = nullptr;

    // ------------------------------------------------------------- 右面板两态（工单 09）
    // 分裂器要从构造期活到之后（收起/展开时得重排两段宽度），所以是成员不是局部变量。
    QSplitter* splitter_ = nullptr;
    PanelHost* panelHost_ = nullptr;

    // 「首次布局后钉一次宽度」只做一次：钉完归用户（拖过就记在 PanelHost 里）。
    bool panelPinned_ = false;

    // ------------------------------------------------------------- 视口浮层（工单 10）
    // 两者都是 **gl_ 的子控件**（不是本页的），所以坐标就是视口坐标，换算不用绕。
    // 「按内容固定尺寸」由各自构造时 setFixedSize 保证 —— 若哪天有人把它们改成
    // 铺满视口的容器，画布就再也拖不动了（见 FloatingBar.h 里那三条硬约束）。
    FloatingBar*   bar_     = nullptr;
    PointsCapsule* capsule_ = nullptr;

    // 点数读数没有信号可订阅（glwidget 不发帧事件，也不做计时埋点），所以按固定间隔
    // 取一次快照。500ms 足够看出「地图还在长」，又不值得为它做任何推送机制。
    QTimer* pointsTimer_ = nullptr;

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

    // 「Z = 当前机器人高度」：勾上之后 Z 微调框置灰，值由位姿帧驱动，
    // 画布取点的平面也换成机器人高度（判据在 GLWidget::setNavZFollowRobot 里，
    // 本页只负责勾选状态与框里的显示）。
    QCheckBox* ckNavZFollow_ = nullptr;
    double robotZ_     = 0.0;    // 最近一帧位姿的 Z（图系，米）
    bool   hasRobotZ_  = false;  // false = 这一趟还没收到过 `0x01`，没有「当前高度」可言

    QPushButton* btnSendNavGoal_ = nullptr;

    // 保存地图（模块侧落盘，见 AA 00 02 0A）
    QPushButton* btnSave_ = nullptr;

    // 下载地图（模块侧打包分块下传，见 AA 00 05 0A）：紧邻「保存地图」，
    // 但**门禁不同**——它只认 TCP / UDP（串口命令面没有 05），见 setDownloadAvailable()
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

    // 2D 扫描「为什么不画」那条说明标签在工单 08 里**整条删掉了**：不再创建、不再显示，
    // 它的 connect 也一并去掉。GLWidget 侧的判断与 `scan2DNoteChanged` 信号保持不变，
    // 只是没有界面接收者 —— 那句话说的是内部自动判定的结果，摆出来只会像「界面坏了」。

    // 开机点（工单 09）：本次开机那一刻机器人中心在图里的位置。与「机器人坐标」是同一类
    // **读数**（都是位姿），所以整块紧跟在它下面；而「为什么不显示」那句话挂在
    // 「点云显示」那一块（两处都是「图上画不画」）——
    // 但**不是每个「不显示」都有话说**：还没读到过模式参数（0x13）那种，两处都空着。
    //
    // 整块平时收起来：只有定位模式且 `0x02` 到过才有得显示。不画时**连数值一起收**，
    // 不留一行占位 —— 「没有残留数值」这条因此是结构上成立的，不靠谁记得去清。
    CollapsibleSection* originSection_ = nullptr;
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
