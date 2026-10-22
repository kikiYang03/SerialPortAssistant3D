#ifndef STATUSBAREX_H
#define STATUSBAREX_H

#include <QStatusBar>

class QLabel;
class ElideLabel;

// 底部两段状态栏（工单 07）：串口 · 控制链路。
// （原第三段「TF 帧名」已按现场要求整段删除，右下角不再显示任何 TF 内容。）
//
// 两段各有**一个**数据源，本类不推断、不另存一份状态：
//   第一段 setPortState(text, ok)         ← SerialPort::portStateChanged
//                                            （串口页那行标签、标题栏胶囊是同一个信号）
//   第二段 setControlLinkState(text, ok)  ← SerialPort::controlLinkTextChanged
//
// 断线**只染断的那一段**：只有该段的圆点与值文字各调一次 Theme::setState，另一段
// 一个像素都不动 —— 一眼就能看出是两段里的哪一段掉了。颜色全走 QSS 的
// `[state="ok"|"err"]`，本模块没有色值字面量。
//
// 宽度：两段的值标签是会自动省略的 ElideLabel，按 stretch 1:1 分整条宽度；窗口拉到
// 最窄时两段仍被一条 1px 竖线分开，也不会撑出横向滚动。
class StatusBarEx : public QStatusBar
{
    Q_OBJECT

public:
    explicit StatusBarEx(QWidget *parent = nullptr);

public slots:
    // 第一段。text 与串口页那行标签是同一个字符串（含端口与波特率），ok 决定绿/红；
    // 空文本表示「还没连过」，显示占位符而不是一条红字。
    void setPortState(const QString &text, bool ok);

    // 第二段。文案本身区分 TCP / UDP（"TCP 已连接" / "UDP 已连接" / "尝试自动重连"），
    // 本类不再按协议另算一遍。
    void setControlLinkState(const QString &text, bool ok);

private:
    QLabel *portDot_ = nullptr;
    ElideLabel *portValue_ = nullptr;
    QLabel *linkDot_ = nullptr;
    ElideLabel *linkValue_ = nullptr;
};

#endif // STATUSBAREX_H
