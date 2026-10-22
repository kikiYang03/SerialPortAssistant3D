#include "FloatingBar.h"

#include "IconTint.h"
#include "Theme.h"

#include <QAbstractButton>
#include <QColor>
#include <QEasingCurve>
#include <QEvent>
#include <QHBoxLayout>
#include <QMouseEvent>
#include <QPainter>
#include <QPixmap>
#include <QVariant>
#include <QVariantAnimation>

namespace {

// 尺寸全部取自 docs/prototype-v2.html 的 .fbar / .fbtn（浏览器里的 30px 按钮、16px 图标）。
// 一律在代码里定：QSS 的 min-width/min-height 会落到 setMinimumWidth()，
// 在 setFixedSize() 之后再写会把钳制冲掉（04 踩过一次，标题栏三键被压成 14px）。
constexpr int kBtnPx      = 30;
constexpr int kBtnRadius  = 7;
constexpr int kIconPx     = 16;
constexpr int kBarRadius  = 10;   // < 高度 38 的一半，圆角才画得出来
constexpr int kPad        = 4;    // .fbar padding: 4px
constexpr int kGap        = 2;    // .fbar gap: 2px
constexpr int kDividerW   = 7;    // 1px 竖线 + 两侧各 3px 留白（.fdiv margin: 0 3px）
constexpr int kDividerH   = 18;

// ---------------------------------------------------------------- 浮层的配色
//
// 浮层压在**深色画布**（Theme::palette().canvas）上，不是压在浅色页面上，所以配色
// 不能直接借用页面的那套（@bg / @line 放上去就成了一块白斑）。全部从画布色与
// on-accent（白）派生，不写字面量：
//   * 底色 = 画布提亮一档 + 92% 不透明 —— 比画布亮一点能看出「这里有一层东西」，
//     但不至于抢走点云的注意力；
//   * 交互层（悬停底 / 分隔线 / 描边）才是**半透明白**：三层都靠它，鼠标划过时
//     像玻璃上抹了一下，跟原型的观感一致。

// 半透明白。alpha 是唯一的参数，色值来自 Theme。
QColor whiteAlpha(qreal a)
{
    QColor c = Theme::palette().onAccent;
    c.setAlphaF(a);
    return c;
}

// 浮层与胶囊共用的「深色玻璃」底色（不透明的那一份）。
// 半透明的版本由 panelColor() 在它基础上加 alpha 得到。
QColor panelBase()
{
    QColor c = Theme::palette().canvas;
    return c.lighter(150);   // 画布 #0f1620 → 约 #16202e，够淡，不抢点云
}

QColor panelColor()
{
    QColor c = panelBase();
    c.setAlphaF(0.92);       // 留一点透明：底下的点云透得过一点点，看得出是浮层
    return c;
}

// 把两个颜色按比例混成实色（a = top 的占比）。用于「想淡一点但不能带 alpha」的地方。
QColor blend(const QColor &top, const QColor &base, qreal a)
{
    const qreal keep = 1.0 - a;
    return QColor(qRound(top.red()   * a + base.red()   * keep),
                  qRound(top.green() * a + base.green() * keep),
                  qRound(top.blue()  * a + base.blue()  * keep));
}

// 图标色必须在**不透明**的前提下表达「淡一点」：
// Qt5 的 SVG 渲染器只认 #rrggbb（它走的是 SVG Tiny 语法），#aarrggbb 会被当成
// 非法颜色整条丢掉 —— 结果是图标干脆不画（本票第一次跑起来就是这样，六个键全空）。
// 所以这里按「白压在浮层底色上」的观感把 alpha 混成实色。
QColor inkOnPanel(qreal a)
{
    return blend(Theme::palette().onAccent, panelBase(), a);
}

// 带 alpha 的同色（底色用：它要压在半透明浮层上，必须真的半透明）
QColor withAlpha(const QColor &c, qreal a)
{
    QColor out = c;
    out.setAlphaF(a);
    return out;
}

// 两个颜色线性插值，alpha 一起算。只用在「悬停 → 按下」这一小段：
// 两端都是半透明底，直接插值不会出现 CSS 里 transparent 那种偏灰的问题。
QColor mix(const QColor &from, const QColor &to, qreal t)
{
    const qreal k = qBound(0.0, t, 1.0);
    const qreal keep = 1.0 - k;
    return QColor(qRound(from.red()   * keep + to.red()   * k),
                  qRound(from.green() * keep + to.green() * k),
                  qRound(from.blue()  * keep + to.blue()  * k),
                  qRound(from.alpha() * keep + to.alpha() * k));
}

// ================================================================ 三档外观
//
// 原型 .fbtn 的三档（transition:background .16s,color .16s）：
//   静止 底色透明 / 图标偏灰（不是纯白：纯白压在深底上比文字还亮，会抢戏）
//   悬停 白色 10% 底 / 图标接近纯白
//   按下 accent 32% 底 / 图标纯白
// 危险键（清理画布，不可撤销）的悬停/按下改用 err 色相。
//
// 过渡**只在这三档之间插值**，不给每一帧算一个随手的新颜色：图标走 IconTint 的缓存，
// 颜色每帧都变的话缓存会每帧长一条（60fps 的鼠标划过就是几十条），所以图标改成
// 「两张固定图交叉淡入」（见 BarButton::paintEvent），缓存键的数量与动画无关。
constexpr qreal kStateIdle  = 0.0;
constexpr qreal kStateHover = 1.0;
constexpr qreal kStateDown  = 2.0;

QColor backHover(bool danger)
{
    return danger ? withAlpha(Theme::color(QStringLiteral("err")), 0.28) : whiteAlpha(0.10);
}

QColor backDown(bool danger)
{
    return withAlpha(Theme::color(danger ? QStringLiteral("err") : QStringLiteral("accent")), 0.32);
}

// 底色：静止是「无」，所以前半段用 alpha 淡入，后半段才在两种实色之间插值。
QColor backAt(qreal t, bool danger)
{
    if (t <= kStateHover)
        return withAlpha(backHover(danger), backHover(danger).alphaF() * qBound(0.0, t, 1.0));
    return mix(backHover(danger), backDown(danger), t - kStateHover);
}

QColor inkHover(bool danger)
{
    if (danger) {
        // 危险键悬停偏红：与普通键的「白亮一下」区分开（清理画布不可撤销）
        const QColor red = Theme::color(QStringLiteral("err")).lighter(165);
        return blend(red, panelBase(), 0.92);
    }
    return inkOnPanel(0.95);
}

QColor inkDown()
{
    return inkOnPanel(1.0);
}

/* ---------------------------------------------------------------- 按钮 */

// 一颗 30×30 的图标键。自绘：底色与图标色一起从状态里换，QSS 表达不了
// 「按下时图标也变亮」这件事（QIcon 只有 Normal/Disabled/Active 三档）。
//
// **不用 Q_OBJECT**：没有新增信号槽，QAbstractButton 的 clicked() 够用
// （与标题栏的 CaptionButton 同一个写法）。所以动画用 QVariantAnimation
// （它自己就是 QObject，回调里写成员 + update()），不引 QPropertyAnimation。
//
// 过渡的范围只有「外观」：动画里只做 update()，不动 geometry、不动事件掩码、
// 不装事件过滤 —— 悬浮条压在 GL 视口上，一旦这里影响了命中测试，
// 表现就是「画布突然拖不动」，那是本票最不能出的问题。
class BarButton : public QAbstractButton
{
public:
    BarButton(const QString &iconName, bool danger, QWidget *parent)
        : QAbstractButton(parent)
        , iconName_(iconName)
        , danger_(danger)
    {
        // objectName 只是给测试数按钮用的（控制台测试按名字找控件，不测私有成员）。
        setObjectName(QStringLiteral("barButton"));
        setFixedSize(kBtnPx, kBtnPx);
        // 悬浮条不吃 Tab 焦点：视口里没有可编辑的东西，焦点滚到图标键上只会更难看清
        setFocusPolicy(Qt::NoFocus);
        setCursor(Qt::PointingHandCursor);

        anim_ = new QVariantAnimation(this);
        anim_->setEasingCurve(QEasingCurve::OutCubic);
        connect(anim_, &QVariantAnimation::valueChanged, this, [this](const QVariant &v) {
            state_ = v.toReal();
            update();
        });
    }

protected:
    void paintEvent(QPaintEvent *) override
    {
        QPainter p(this);
        p.setRenderHint(QPainter::Antialiasing, true);

        const QColor bg = backAt(state_, danger_);
        if (bg.alpha() > 0) {
            p.setPen(Qt::NoPen);
            p.setBrush(bg);
            p.drawRoundedRect(QRectF(rect()), kBtnRadius, kBtnRadius);
        }

        // 图标：两张固定颜色的图交叉淡入，而不是每帧生成一个插值色去渲染 ——
        // 后者的缓存键每帧都不一样，划一次鼠标就能攒出几十张 16px 的图。
        const QRect box(QPoint((width() - kIconPx) / 2, (height() - kIconPx) / 2),
                        QSize(kIconPx, kIconPx));
        if (state_ <= kStateHover) {
            drawIcon(p, box, inkOnPanel(0.62), 1.0 - state_);
            drawIcon(p, box, inkHover(danger_), state_);
        } else {
            drawIcon(p, box, inkHover(danger_), kStateDown - state_);
            drawIcon(p, box, inkDown(), state_ - kStateHover);
        }
    }

    // QAbstractButton 只保证为「按下/释放」重绘，悬停的三档外观要自己驱动动画。
    void enterEvent(QEvent *e) override
    {
        QAbstractButton::enterEvent(e);
        animateTo(isDown() ? kStateDown : kStateHover);
    }

    void leaveEvent(QEvent *e) override
    {
        QAbstractButton::leaveEvent(e);
        animateTo(isDown() ? kStateDown : kStateIdle);
    }

    void mousePressEvent(QMouseEvent *e) override
    {
        QAbstractButton::mousePressEvent(e);
        animateTo(isDown() ? kStateDown : kStateHover);
    }

    void mouseReleaseEvent(QMouseEvent *e) override
    {
        QAbstractButton::mouseReleaseEvent(e);
        // 松手时鼠标可能已经不在按钮上了（按下后拖出去再松），那就退回静止
        animateTo(underMouse() ? kStateHover : kStateIdle);
    }

private:
    void drawIcon(QPainter &p, const QRect &box, const QColor &ink, qreal opacity)
    {
        if (opacity <= 0.0)
            return;
        const QPixmap pm = IconTint::pixmap(iconName_, ink, kIconPx, devicePixelRatioF());
        // 给目标矩形而不是裸坐标：高 DPI 下由 Qt 按 dpr 缩放，图标不会糊
        p.setOpacity(qBound(0.0, opacity, 1.0));
        p.drawPixmap(box, pm);
        p.setOpacity(1.0);
    }

    // 前往某一档。时长按「跨了几档」算：静止↔悬停与悬停↔按下各自都是 160ms，
    // 一次跨两档（按下后直接移出）夹到把手那一档的 200ms，不会拖成 320ms 的黏滞。
    void animateTo(qreal target)
    {
        if (!anim_ || qFuzzyCompare(state_ + 1.0, target + 1.0))
            return;

        anim_->stop();
        const int span = qRound(Theme::Motion::HoverMs * qAbs(target - state_));
        anim_->setDuration(qBound(Theme::Motion::HoverMs, span, Theme::Motion::ToggleMs));
        anim_->setStartValue(state_);
        anim_->setEndValue(target);
        anim_->start();
    }

    QString iconName_;
    bool danger_ = false;
    qreal state_ = kStateIdle;   // 0 = 静止，1 = 悬停，2 = 按下（中间值是过渡帧）
    QVariantAnimation *anim_ = nullptr;
};

/* ---------------------------------------------------------------- 分隔线 */

// 分组之间的 1px 竖线。**纯装饰**：显式声明鼠标穿透，压在它上面的那一下
// 直接落回 GL 视口（spec 实现决策 26）。
class BarDivider : public QWidget
{
public:
    explicit BarDivider(QWidget *parent)
        : QWidget(parent)
    {
        setObjectName(QStringLiteral("barDivider"));
        setFixedSize(kDividerW, kDividerH);
        setAttribute(Qt::WA_TransparentForMouseEvents, true);
    }

protected:
    void paintEvent(QPaintEvent *) override
    {
        QPainter p(this);
        p.fillRect(QRect((width() - 1) / 2, 0, 1, height()), whiteAlpha(0.10));
    }
};

} // namespace

FloatingBar::FloatingBar(QWidget *parent)
    : QWidget(parent)
{
    setObjectName(QStringLiteral("floatingBar"));
    // 固定尺寸：这两个策略 + 构造末尾的 setFixedSize 一起保证「按内容收窄」。
    setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    setFocusPolicy(Qt::NoFocus);
    // 条子上不放光标：鼠标在视口里该是抓手，划过按钮时变手型由按钮自己设
    setCursor(Qt::ArrowCursor);

    auto *lay = new QHBoxLayout(this);
    lay->setContentsMargins(kPad, kPad, kPad, kPad);
    lay->setSpacing(kGap);

    // 顺序与原型一致：复位 | 左右 | 上下 | 清理
    addAction(ResetView, QStringLiteral("rotate-ccw"), tr("复位视角"));
    addDivider();
    addAction(YawLeft,  QStringLiteral("chevron-left"),  tr("左转 %1°").arg(kStepDegrees));
    addAction(YawRight, QStringLiteral("chevron-right"), tr("右转 %1°").arg(kStepDegrees));
    addDivider();
    addAction(PitchUp,   QStringLiteral("chevron-up"),   tr("上仰 %1°").arg(kStepDegrees));
    addAction(PitchDown, QStringLiteral("chevron-down"), tr("下俯 %1°").arg(kStepDegrees));
    addDivider();
    addAction(ClearCanvas, QStringLiteral("eraser"), tr("清理画布"), /*danger=*/true);

    applyContentSize();
}

void FloatingBar::addAction(Action action, const QString &iconName, const QString &tip, bool danger)
{
    auto *btn = new BarButton(iconName, danger, this);
    // 提示用 Qt 自带的 tooltip（全局 QSS 里已有 QToolTip 规则，观感与应用其它按钮一致）。
    // 没有自绘那个「按在键上方」的深色气泡：气泡要画到条子矩形之外，就得把条子做高、
    // 或者另开一个顶层窗口 —— 前者会让「按内容固定尺寸」这条算出一块空白区，
    // 后者等于给浮层再加一个窗口，都是这笔改动最该避免的东西。
    btn->setToolTip(tip);
    btn->setAccessibleName(tip);

    connect(btn, &QAbstractButton::clicked, this, [this, action]() {
        emit actionTriggered(action);
    });

    static_cast<QHBoxLayout *>(layout())->addWidget(btn);
}

void FloatingBar::addDivider()
{
    static_cast<QHBoxLayout *>(layout())->addWidget(new BarDivider(this));
}

void FloatingBar::applyContentSize()
{
    // 布局自己算出来要多大就是多大 —— 这是「按内容固定尺寸」的唯一来源。
    contentSize_ = layout()->sizeHint();
    setFixedSize(contentSize_);
}

QRect FloatingBar::placeRect(const QSize &viewport, const QSize &bar)
{
    // 整数除法（向下取整）：同一组尺寸永远同一个结果，奇数差时稳定偏 0.5px 一侧。
    const int x = qMax(0, (viewport.width() - bar.width()) / 2);
    // 视口比条子还矮时贴顶（y 不为负），否则底部留 kBottomMargin
    const int y = qMax(0, viewport.height() - bar.height() - kBottomMargin);
    return QRect(QPoint(x, y), bar);
}

QSize FloatingBar::sizeHint() const
{
    return contentSize_;
}

QSize FloatingBar::minimumSizeHint() const
{
    // 与 sizeHint 一致：条子没有「更小也还能用」的形态，压小只会把图标裁掉
    return contentSize_;
}

void FloatingBar::paintEvent(QPaintEvent *)
{
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing, true);

    // 半像素内缩：1px 描边落在像素中心上，四边才是实的（与 IconButton 同一个做法）
    const QRectF box = QRectF(rect()).adjusted(0.5, 0.5, -0.5, -0.5);
    p.setPen(QPen(whiteAlpha(0.10), 1));
    p.setBrush(panelColor());
    p.drawRoundedRect(box, kBarRadius, kBarRadius);
}
