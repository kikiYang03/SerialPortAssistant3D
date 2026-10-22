#include "TitleBar.h"

#include "FramelessHelper.h"
#include "IconTint.h"
#include "Theme.h"

#include <QAbstractButton>
#include <QCoreApplication>
#include <QEvent>
#include <QFrame>
#include <QHBoxLayout>
#include <QIcon>
#include <QLabel>
#include <QLinearGradient>
#include <QMenu>
#include <QMouseEvent>
#include <QPainter>
#include <QPixmap>

namespace {

// 尺寸全部取自 docs/prototype-v2.html 的 .titlebar / .cap / .pill，
// 一律在代码里定 —— QSS 的 min-width/min-height 会落到 setMinimumWidth()，
// 在 setFixedSize() 之后再写会把钳制冲掉（04 踩过：三键被压成 14px）。
constexpr int kBarHeight   = 38;
constexpr int kCaptionW    = 46;
// 原型里的三键图标是 10px，但那是浏览器按次像素抗锯齿渲染的。SVG 的 viewBox 是 24、
// stroke 2 —— 画到 10px 时线宽只有 0.83px，Qt 这边会被抗锯齿摊成一层浅灰。
// 取 12px 正好等于 1 个物理像素的线宽，观感和原型一致。
constexpr int kIconPx      = 12;
constexpr int kLogoPx      = 16;
constexpr int kPillHeight  = 21;
constexpr int kDotPx       = 6;

// 版本号**可配置**：唯一来源是 config.ini 的 `[App] version`，由 main.cpp 在读配置时
// 写进 QCoreApplication::applicationVersion()，标题栏这里只取那一份。
// kFallbackVersion 只在「跑的是个裸 exe、旁边没有 config.ini」时兜底，
// 免得标题栏上出现一片空白。
const char *const kFallbackVersion = "v2.8";

QString currentAppVersion()
{
    const QString v = QCoreApplication::applicationVersion().trimmed();
    return v.isEmpty() ? QString::fromLatin1(kFallbackVersion) : v;
}

} // namespace

// 标题栏上的一个键：整块 46×38 可点、自绘（不写死任何色值，全部经 Theme / IconTint 取）。
// 之所以不用 QPushButton + QSS：悬停要同时换「底色」和「图标色」，而且关闭键悬停是
// 红底反白 —— 用一个 paintEvent 表达比「QSS 换底色 + enterEvent 换图标」两条路更直。
//
// 不用 Q_OBJECT：没有新增信号槽，QAbstractButton 的 clicked() 照常可连。
class CaptionButton : public QAbstractButton
{
public:
    // iconName 为空 = 画三个点（溢出入口，图标集里没有 more-horizontal）。
    // danger = 关闭键：悬停红底、图标反白。
    CaptionButton(const QString &iconName, bool danger, QWidget *parent)
        : QAbstractButton(parent)
        , iconName_(iconName)
        , danger_(danger)
    {
        setFixedSize(kCaptionW, kBarHeight);
        setFocusPolicy(Qt::NoFocus);
        setCursor(Qt::ArrowCursor);
        // 命中测试：按钮本来就按 HTCLIENT 处理，这里再显式打一次，免得将来
        // FramelessHelper 的「按钮兜底」被改写时这个键悄悄变成拖动区。
        setProperty(FramelessHelper::kHitProperty,
                    QString::fromLatin1(FramelessHelper::kHitClient));
    }

protected:
    void paintEvent(QPaintEvent *) override
    {
        // QAbstractButton 的 enter/leave 自己会 update()，所以这里读 underMouse() 就够了。
        const bool hot = underMouse() || isDown();
        QPainter p(this);

        QColor bg;
        if (danger_ && hot)
            bg = isDown() ? Theme::color(QStringLiteral("err")).darker(115)
                          : Theme::color(QStringLiteral("err"));
        else if (isDown())
            bg = Theme::color(QStringLiteral("pressed"));
        else if (hot)
            bg = Theme::color(QStringLiteral("hover"));

        if (bg.isValid())
            p.fillRect(rect(), bg);

        const QColor ink = (danger_ && hot)
                               ? Theme::color(QStringLiteral("on-accent"))
                               : IconTint::stateColor(hot ? IconTint::Hover : IconTint::Normal);

        if (iconName_.isEmpty()) {
            p.setPen(Qt::NoPen);
            p.setBrush(ink);
            const int cy = height() / 2;
            for (int i = -1; i <= 1; ++i)
                p.drawEllipse(QPoint(width() / 2 + i * 5, cy), 1, 1);
            return;
        }

        const QPixmap pm = IconTint::pixmap(iconName_, ink, kIconPx, devicePixelRatioF());
        const QRect box(QPoint((width() - kIconPx) / 2, (height() - kIconPx) / 2),
                        QSize(kIconPx, kIconPx));
        p.drawPixmap(box, pm);   // 给了目标矩形，高 DPI 下由 Qt 按 dpr 缩放
    }

private:
    QString iconName_;
    bool danger_ = false;
};

namespace {

// 程序图标：优先用窗口自己的（mainwindow.ui 里指向 :/images/logo256.ico）。
// 取不到就退回一块 accent → cyan 的渐变方块 —— 标题栏左上角空着比这更难看。
QPixmap logoPixmap(const QIcon &icon, qreal dpr)
{
    if (!icon.isNull()) {
        // Qt5 的 QIcon::pixmap 没有 dpr 重载：按物理像素要图，再自己标 dpr，
        // 这样高 DPI 下不是拿一张 16px 的图让系统去拉伸。
        const int px = qRound(kLogoPx * dpr);
        QPixmap raw = icon.pixmap(QSize(px, px));
        if (!raw.isNull()) {
            raw.setDevicePixelRatio(dpr);
            return raw;
        }
    }

    QPixmap pm(QSize(qRound(kLogoPx * dpr), qRound(kLogoPx * dpr)));
    pm.setDevicePixelRatio(dpr);
    pm.fill(Qt::transparent);

    QPainter p(&pm);
    p.setRenderHint(QPainter::Antialiasing);
    QLinearGradient g(0, 0, kLogoPx, kLogoPx);
    g.setColorAt(0.0, Theme::color(QStringLiteral("accent")));
    g.setColorAt(1.0, Theme::color(QStringLiteral("cyan")));
    p.setPen(Qt::NoPen);
    p.setBrush(g);
    p.drawRoundedRect(QRectF(0, 0, kLogoPx, kLogoPx), 4, 4);
    return pm;
}

} // namespace

TitleBar::TitleBar(QWidget *window)
    : QWidget(window)
    , window_(window)
{
    Q_ASSERT(window_);
    setObjectName(QStringLiteral("titleBar"));
    setFixedHeight(kBarHeight);
    // 底色由 QSS 给（#titleBar）。QWidget 子类要自己开这一位，否则样式表的
    // background 不会画 —— 只剩下面 paintEvent 里那两条 1px 线。
    setAttribute(Qt::WA_StyledBackground, true);

    auto *row = new QHBoxLayout(this);
    row->setContentsMargins(Theme::Metric::Gap, 0, 0, 0);   // 左 12px，右 0：三键贴边
    row->setSpacing(0);

    logo_ = new QLabel(this);
    logo_->setObjectName(QStringLiteral("titleLogo"));
    logo_->setFixedSize(kLogoPx, kLogoPx);
    logo_->setAttribute(Qt::WA_TransparentForMouseEvents);
    updateLogo();
    row->addWidget(logo_);

    brand_ = new QLabel(window_->windowTitle(), this);
    brand_->setObjectName(QStringLiteral("titleBrand"));
    row->addSpacing(9);            // 原型里 .title 的 margin-left
    row->addWidget(brand_);
    connect(window_, &QWidget::windowTitleChanged, brand_, &QLabel::setText);
    connect(window_, &QWidget::windowIconChanged, this, &TitleBar::updateLogo);

    version_ = new QLabel(currentAppVersion(), this);
    version_->setObjectName(QStringLiteral("titleVersion"));
    row->addSpacing(8);
    row->addWidget(version_);

    auto *sep = new QFrame(this);
    sep->setObjectName(QStringLiteral("titleSep"));
    sep->setFixedSize(1, 14);
    sep->setAttribute(Qt::WA_TransparentForMouseEvents);
    row->addSpacing(12);
    row->addWidget(sep);
    row->addSpacing(12);

    // 连接状态胶囊：描边、圆点、文字三处各自按 state 着色（见 app.qss.in），
    // 它们共用同一个状态名，都由 setPortState() 经 Theme::setState 写入。
    pill_ = new QFrame(this);
    pill_->setObjectName(QStringLiteral("portPill"));
    pill_->setFixedHeight(kPillHeight);
    pill_->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    // 胶囊必须 HTCLIENT：否则点在上面会被当成标题栏空白，变成拖窗口。
    pill_->setProperty(FramelessHelper::kHitProperty,
                       QString::fromLatin1(FramelessHelper::kHitClient));

    auto *pillRow = new QHBoxLayout(pill_);
    pillRow->setContentsMargins(9, 0, 9, 0);
    pillRow->setSpacing(5);

    pillDot_ = new QLabel(pill_);
    pillDot_->setObjectName(QStringLiteral("portDot"));
    pillDot_->setFixedSize(kDotPx, kDotPx);
    pillDot_->setAttribute(Qt::WA_TransparentForMouseEvents);

    pillText_ = new QLabel(pill_);
    pillText_->setObjectName(QStringLiteral("portPillText"));
    pillText_->setAttribute(Qt::WA_TransparentForMouseEvents);

    pillRow->addWidget(pillDot_);
    pillRow->addWidget(pillText_);
    row->addWidget(pill_);
    row->addStretch(1);

    // 溢出入口（06 用）：先空着 → 不可见也不占位。
    overflow_ = new QMenu(this);
    overflow_->installEventFilter(this);

    overflowBtn_ = new CaptionButton(QString(), false, this);
    overflowBtn_->setObjectName(QStringLiteral("capMore"));
    connect(overflowBtn_, &QAbstractButton::clicked, this, [this]() {
        if (!overflow_ || overflow_->isEmpty())
            return;
        const QSize hint = overflow_->sizeHint();
        const QPoint at = overflowBtn_->mapToGlobal(
            QPoint(overflowBtn_->width() - hint.width(), overflowBtn_->height() + 2));
        overflow_->popup(at);
    });
    row->addWidget(overflowBtn_);

    auto *minBtn = new CaptionButton(QStringLiteral("minus"), false, this);
    minBtn->setObjectName(QStringLiteral("capMin"));
    minBtn->setToolTip(tr("最小化"));

    auto *maxBtn = new CaptionButton(QStringLiteral("square"), false, this);
    maxBtn->setObjectName(QStringLiteral("capMax"));
    maxBtn->setToolTip(tr("最大化 / 还原"));
    // HTMAXBUTTON 让 Win11 悬停时弹 Snap 布局；Win10 忽略该值。命中到它时由
    // FramelessHelper 自己切最大化（04），下面的 clicked 只是非 Windows 平台的落点。
    maxBtn->setProperty(FramelessHelper::kHitProperty,
                        QString::fromLatin1(FramelessHelper::kHitMaxButton));

    auto *closeBtn = new CaptionButton(QStringLiteral("x"), true, this);
    closeBtn->setObjectName(QStringLiteral("capClose"));
    closeBtn->setToolTip(tr("关闭"));

    row->addWidget(minBtn);
    row->addWidget(maxBtn);
    row->addWidget(closeBtn);

    connect(minBtn, &QAbstractButton::clicked, window_, &QWidget::showMinimized);
    connect(maxBtn, &QAbstractButton::clicked, this, &TitleBar::toggleMaximize);
    connect(closeBtn, &QAbstractButton::clicked, window_, &QWidget::close);

    // 初值：启动到第一次连上串口之间，胶囊不该是空的。
    setPortState(QString(), false);
    syncOverflowEntry();
}

QMenu *TitleBar::overflowMenu() const
{
    return overflow_;
}

void TitleBar::setPortState(const QString &detail, bool connected)
{
    const QString state = connected ? QStringLiteral("ok") : QStringLiteral("err");

    // 三处状态名一致、各有一条简单选择器：胶囊描边 / 圆点底色 / 文字色。
    // setState 是幂等的（03），每秒刷新的路径上重复设置不会抖。
    Theme::setState(pill_, state);
    Theme::setState(pillDot_, state);
    Theme::setState(pillText_, state);

    pillText_->setText(connected ? tr("已连接") : tr("未连接"));
    pill_->setToolTip(detail);
}

void TitleBar::setBrand(const QString &name, const QString &version)
{
    brand_->setText(name);
    version_->setText(version.isEmpty() ? currentAppVersion() : version);
}

void TitleBar::paintEvent(QPaintEvent *)
{
    QPainter p(this);
    p.setPen(Theme::color(QStringLiteral("line")));

    // 顶部 1px 描边只在 Win10 上画：那边窗口是直角，没有它就像缺了道边；
    // Win11 由 DWM 给 8px 圆角，多画一条会在圆角里露出个直角。
    if (!FramelessHelper::usesDwmRounding(window_))
        p.drawLine(0, 0, width() - 1, 0);

    // 标题栏底边：与下面的命令栏分开。
    p.drawLine(0, height() - 1, width() - 1, height() - 1);
}

void TitleBar::mouseDoubleClickEvent(QMouseEvent *e)
{
    // 兜底（04 的票面留的那句）：HTCAPTION 上的双击由系统处理，Qt 根本收不到事件；
    // 能走到这里说明命中的是标题栏里返回 HTCLIENT 的子控件（品牌名、胶囊内的空白）
    // 或者是非 Windows 平台 —— 两种情况下都该切换最大化。
    if (e->button() == Qt::LeftButton) {
        toggleMaximize();
        e->accept();
        return;
    }
    QWidget::mouseDoubleClickEvent(e);
}

bool TitleBar::eventFilter(QObject *watched, QEvent *e)
{
    // 菜单里增删动作时自己决定入口是否显示，调用方（06）不用通知。
    if (watched == overflow_
        && (e->type() == QEvent::ActionAdded || e->type() == QEvent::ActionRemoved)) {
        syncOverflowEntry();
    }
    return QWidget::eventFilter(watched, e);
}

void TitleBar::toggleMaximize()
{
    if (window_->isMaximized())
        window_->showNormal();
    else
        window_->showMaximized();
}

void TitleBar::syncOverflowEntry()
{
    if (!overflow_ || !overflowBtn_)
        return;
    const bool has = !overflow_->isEmpty();
    overflowBtn_->setVisible(has);
    overflowBtn_->setToolTip(has ? tr("更多") : QString());
}

bool TitleBar::event(QEvent *e)
{
    // 窗口被拖到另一块缩放不同的屏上（或系统缩放改了）时，dpr 会变。
    // 程序图标是构造期按当时的 dpr 烘成 QPixmap 的，不重出就会一直糊着。
    // ScreenChangeInternal 是 Qt5 里唯一可靠的「换了屏」通知（Qt 6.6 才有 DPRChange）。
    if (e->type() == QEvent::ScreenChangeInternal)
        refreshDprArtifacts();
    return QWidget::event(e);
}

void TitleBar::refreshDprArtifacts()
{
    updateLogo();
    update();   // 三键图标每帧现取，但要让它们立刻重画一遍
}

void TitleBar::updateLogo()
{
    logo_->setPixmap(logoPixmap(window_->windowIcon(), devicePixelRatioF()));
}
