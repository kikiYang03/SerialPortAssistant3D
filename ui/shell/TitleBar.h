#ifndef TITLEBAR_H
#define TITLEBAR_H

#include <QWidget>

class QEvent;
class QFrame;
class QLabel;
class QMenu;
class QMouseEvent;
class QPaintEvent;
class CaptionButton;

// 自绘标题栏（工单 05）：左上角程序图标 + 品牌名 + 版本号，其后是**串口链路**状态胶囊；
// 右上角最小化 / 最大化 / 关闭三键，三键左边留一个默认空着的溢出入口（工单 06 往里加
// 「管理员登录 / 退出」）。
//
// 命中测试的两条约束（04 的 FramelessHelper 在这里找答案）：
//   * 三键与胶囊必须返回 HTCLIENT —— 三键是 QAbstractButton（helper 对按钮一律给
//     HTCLIENT），胶囊与它内部的文字/圆点靠**胶囊自己**打的 framelessHit="client"
//     兜住（helper 从最深的子控件往上爬，沿途看谁要点击）。
//   * 其余地方返回 HTCAPTION：拖动窗口、双击最大化都仍归系统，本类不自己 move()。
//
// 颜色一律从 Theme 取：胶囊的绿/红走 Theme::setState(w, "ok"/"err")，图标走
// IconTint 的三态；这个模块里没有任何色值字面量。
class TitleBar : public QWidget
{
    Q_OBJECT

public:
    // window 就是被拖动的那个顶层窗口（拖动/最小化/最大化/关闭都发给它）。
    explicit TitleBar(QWidget *window);

    // 溢出菜单：对象一直在，初始是空的；**空着时入口不显示、也不占位**。
    // 06 只要往 menu 里 addAction，入口自动出现（本类用事件过滤器盯 ActionAdded/
    // ActionRemoved），清空后又消失 —— 调用方不需要通知标题栏。
    QMenu *overflowMenu() const;

    // 串口链路状态。detail 是串口页那行文字（"COM3 串口已打开"），只用来做 tooltip；
    // connected 决定胶囊的「已连接 / 未连接」与绿/红。**这是胶囊唯一的数据入口**，
    // 与串口页的标签共用同一个信号源（SerialPort::portStateChanged）。
    void setPortState(const QString &detail, bool connected);

    // 品牌名与版本号。不调用时：名字取窗口标题（跟着 windowTitle 变），版本取
    // QCoreApplication::applicationVersion() —— 那一份由 main.cpp 从 config.ini 的
    // `[App] version` 读入，所以改配置文件、重启程序即生效，不用重新编译。
    void setBrand(const QString &name, const QString &version = QString());

protected:
    void paintEvent(QPaintEvent *e) override;
    void mouseDoubleClickEvent(QMouseEvent *e) override;
    bool eventFilter(QObject *watched, QEvent *e) override;
    bool event(QEvent *e) override;

private:
    void toggleMaximize();
    void syncOverflowEntry();
    void updateLogo();
    // 图标是按 dpr 烘出来的位图，换屏（缩放档变了）就得重出一张，否则一直用旧 dpr 的那张。
    // 三键图标不在此列：它们每帧都按当前 dpr 现取，本来就跟着走。
    void refreshDprArtifacts();

    QWidget *window_ = nullptr;

    QLabel *logo_ = nullptr;
    QLabel *brand_ = nullptr;
    QLabel *version_ = nullptr;

    QFrame *pill_ = nullptr;
    QLabel *pillDot_ = nullptr;
    QLabel *pillText_ = nullptr;

    QMenu *overflow_ = nullptr;
    CaptionButton *overflowBtn_ = nullptr;
};

#endif // TITLEBAR_H
