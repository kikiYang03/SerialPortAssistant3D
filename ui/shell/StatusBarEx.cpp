#include "StatusBarEx.h"

#include "Theme.h"

#include <QFrame>
#include <QHBoxLayout>
#include <QLabel>
#include <QResizeEvent>

namespace {

// 尺寸取自 docs/prototype-v2.html 的 .status：24px 高、5px 圆点、1×11 的竖线。
// 一律在代码里定 —— QSS 的 min-height 会落到 setMinimumHeight，跟 setFixedHeight 打架（04 踩过）。
constexpr int kBarHeight = 24;
// 圆点取 6 而不是原型标的 5：圆角最多写短边的一半，5px 只能配 2px 圆角，
// 渲染出来是「小方块带一点倒角」；6px + 3px 才是标题栏胶囊上那颗的样子。
constexpr int kDotPx = 6;
constexpr int kSepW  = 1;
constexpr int kSepH  = 11;
// 一段内部「圆点 / 名字 / 值」之间的间距（原型 .status .s 的 gap: 6px）。
constexpr int kSegGap = 6;

// 原先这里是第三段「TF 帧名」（静态 base_link）。按现场要求整段摘掉了：
// 状态栏只留串口与控制链路两段，右下角不再出现任何 TF 字样。
// 坐标树信息仍在右面板「机器人坐标」里，本栏不再参与。

} // namespace

// 会自己打省略号的值标签：窗口拉窄时文字截断，而不是把整窗往宽里顶。
// 不用 Q_OBJECT（没有信号槽），所以也不需要 moc，可以只活在这个 .cpp 里。
class ElideLabel : public QLabel
{
public:
    explicit ElideLabel(QWidget *parent = nullptr)
        : QLabel(parent)
    {
        setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
        // Ignored：允许布局把宽度压到 sizeHint 以下。默认策略下最小宽度就是整句话，
        // 窗口一窄就变成「状态栏要求更宽」——那正是要避免的横向溢出。
        setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
    }

    // 全文。显示时按当前宽度取省略版本；tooltip 始终给全文（拉窄了也查得到端口号）。
    void setFullText(const QString &text)
    {
        if (full_ == text)
            return;
        full_ = text;
        setToolTip(text);
        updateElided_();
    }

    QString fullText() const { return full_; }

protected:
    void resizeEvent(QResizeEvent *e) override
    {
        QLabel::resizeEvent(e);
        updateElided_();
    }

private:
    void updateElided_()
    {
        // 从中间省略：这段文字两头都有信息（端口号在前、状态在后），一刀切右边
        // 会把「已连接 / 已关闭」切掉，只剩个看不懂的端口名。
        // 宽度还没被布局算出来之前（第一次 polish）按当前值算一次，随后 resizeEvent 会补上。
        QLabel::setText(fontMetrics().elidedText(full_, Qt::ElideMiddle, qMax(0, width())));
    }

    QString full_;
};

namespace {

struct Segment {
    QWidget *host = nullptr;
    QLabel *dot = nullptr;
    ElideLabel *value = nullptr;
};

// 一段 = [圆点] + 名字 + 值。圆点三处状态色由调用方按 state 着色（见 app.qss.in），
// 两个圆点共用一个 objectName —— 同一件事只有一条样式，不做两份。
Segment makeSegment(QWidget *parent, const QString &caption, bool withDot)
{
    Segment s;
    s.host = new QWidget(parent);

    auto *row = new QHBoxLayout(s.host);
    row->setContentsMargins(0, 0, 0, 0);
    row->setSpacing(kSegGap);

    if (withDot) {
        s.dot = new QLabel(s.host);
        s.dot->setObjectName(QStringLiteral("statusBarDot"));
        s.dot->setFixedSize(kDotPx, kDotPx);
        s.dot->setAttribute(Qt::WA_TransparentForMouseEvents);
        row->addWidget(s.dot, 0, Qt::AlignVCenter);
    }

    auto *captionLabel = new QLabel(caption, s.host);
    captionLabel->setObjectName(QStringLiteral("statusBarCaption"));
    row->addWidget(captionLabel, 0, Qt::AlignVCenter);

    s.value = new ElideLabel(s.host);
    s.value->setObjectName(QStringLiteral("statusBarValue"));
    row->addWidget(s.value, 1);

    return s;
}

QFrame *makeSeparator(QWidget *parent)
{
    auto *sep = new QFrame(parent);
    sep->setObjectName(QStringLiteral("statusBarSep"));
    sep->setFixedSize(kSepW, kSepH);
    sep->setAttribute(Qt::WA_TransparentForMouseEvents);
    return sep;
}

// 一段落地的三件事：文字、圆点、值色。
// ok 才是绿/红；**没有文字**（还没连过）时圆点回灰、值显示占位符 ——
// 「没数据」和「断了」是两回事，不该都画成红的。
void applyState(QLabel *dot, ElideLabel *value, const QString &text, bool ok)
{
    const bool hasData = !text.trimmed().isEmpty();

    value->setFullText(hasData ? text : QStringLiteral("—"));

    if (!dot)
        return;

    Theme::setState(dot, hasData ? (ok ? QStringLiteral("ok") : QStringLiteral("err"))
                                 : QString());
    Theme::setState(value, (hasData && !ok) ? QStringLiteral("err") : QString());
}

} // namespace

StatusBarEx::StatusBarEx(QWidget *parent)
    : QStatusBar(parent)
{
    setObjectName(QStringLiteral("statusBar"));
    setFixedHeight(kBarHeight);
    // 右下角的尺寸把手在本窗里没用（无边框窗口的缩放走 FramelessHelper 的边带），
    // 留着只会白占一条右边。
    setSizeGripEnabled(false);

    // QStatusBar 自带一个给 showMessage() 用的消息标签；它平时不占位（只有
    // showMessage 期间才是可见的），而本项目从不调 showMessage，所以不必特意处理它。
    // 两段靠下面这次 addWidget(host, 1) 拿到整条宽度。

    auto *host = new QWidget(this);
    auto *row = new QHBoxLayout(host);
    row->setContentsMargins(Theme::Metric::Gap, 0, Theme::Metric::Gap, 0);
    row->setSpacing(Theme::Metric::Gap);

    const Segment port = makeSegment(host, tr("串口"), true);
    const Segment link = makeSegment(host, tr("控制链路"), true);
    portDot_ = port.dot;
    portValue_ = port.value;
    linkDot_ = link.dot;
    linkValue_ = link.value;

    // 两段 + 一条竖线，均分整条宽度（原来的第三段 TF 帧名已按要求删除）。
    row->addWidget(port.host, 1);
    row->addWidget(makeSeparator(host), 0, Qt::AlignVCenter);
    row->addWidget(link.host, 1);

    // 交给 QStatusBar 自己的布局（消息标签已藏），stretch 1 让它占满整条。
    addWidget(host, 1);

    // 初值：还没连过任何东西。三处（本栏 / 串口页标签 / 标题栏胶囊）说的都是这一句，
    // 而真正的状态很快就由 SerialPort 的信号接管。
    setPortState(tr("未连接"), false);
    setControlLinkState(tr("未连接"), false);
}

void StatusBarEx::setPortState(const QString &text, bool ok)
{
    applyState(portDot_, portValue_, text, ok);
}

void StatusBarEx::setControlLinkState(const QString &text, bool ok)
{
    applyState(linkDot_, linkValue_, text, ok);
}
