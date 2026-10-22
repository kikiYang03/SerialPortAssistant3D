#include "PanelRail.h"

#include "IconTint.h"
#include "Theme.h"

#include <QAbstractButton>
#include <QEasingCurve>
#include <QEvent>
#include <QFontMetrics>
#include <QMoveEvent>
#include <QMouseEvent>
#include <QPainter>
#include <QPainterPath>
#include <QResizeEvent>
#include <QShowEvent>
#include <QStackedLayout>
#include <QVariant>
#include <QVariantAnimation>
#include <QtGlobal>

/* ================================================================ 宽度策略（纯函数） */

namespace PanelWidth {

int panelWidth(const Memory &m)
{
    if (m.collapsed)
        return kRail;
    // 记忆为空（还没拖过）就用默认宽度 —— 「首次返回默认宽度」只有这一条来路。
    return m.expanded > 0 ? m.expanded : kDefaultExpanded;
}

Memory remember(const Memory &m, int width)
{
    // 收起态的 26px 不是用户的意愿，写进记忆里等于「下次展开只剩窄条」。
    if (m.collapsed || width <= 0)
        return m;

    Memory r = m;
    r.expanded = width;
    return r;
}

Memory toggle(const Memory &m, int currentWidth)
{
    Memory r = m;
    if (!m.collapsed) {
        // 收起的这一刻：把**此刻面板的真实宽度**记下来。
        // 窄条那个宽度不算数 —— 那不是用户拖出来的值，记下来只会让「下次展开」变成展开成一条缝。
        if (currentWidth > kRail)
            r.expanded = currentWidth;
        r.collapsed = true;
    } else {
        // 展开：宽度一个字都不动，于是回到收起前的那个值。
        r.collapsed = false;
    }
    return r;
}

bool canPin(int containerWidth, int panel)
{
    // 钉完之后画布还得剩 kMinViewport：容器太窄时硬钉一个 296，
    // 表现就是「窗口一小，画布直接被挤没」。
    return containerWidth - panel - kHandle >= kMinViewport;
}

QList<int> splitSizes(int containerWidth, int panel, int handle)
{
    const int avail = qMax(0, containerWidth - qMax(0, handle));
    const int p = qBound(0, panel, avail);   // 面板比容器还宽时，视口拿 0 而不是负数
    return QList<int>() << (avail - p) << p;
}

} // namespace PanelWidth

namespace PanelRailGeometry {

QRect toggleRect(const QRect &panelRect)
{
    // 左缘外侧：右缘与面板齐平（原型 left:-13px / width:13px）
    const int x = panelRect.left() - kToggleW;

    int y = panelRect.top() + (panelRect.height() - kToggleH) / 2;
    // 容器比按钮还矮时贴顶：负坐标的按钮会被裁掉一半，那是「点不到」的另一种写法
    if (y < panelRect.top())
        y = panelRect.top();

    return QRect(x, y, kToggleW, kToggleH);
}

} // namespace PanelRailGeometry

/* ================================================================ 窄条 */

namespace {

// 顶部留白 / 圆点与文字之间 / 字与字之间（原型 .prail padding:12px 0; letter-spacing:3px）
constexpr int kRailPadTop = Theme::Metric::Gap;   // 12
constexpr int kDotSize    = 4;
constexpr int kDotGap     = Theme::Metric::Gap;   // 12
constexpr int kCharGap    = 3;

} // namespace

PanelRail::PanelRail(QWidget *parent)
    : QWidget(parent)
{
    setCursor(Qt::PointingHandCursor);   // 整条都能点（点回来比找那枚把手更直觉）
    setToolTip(tr("展开属性面板"));
}

QSize PanelRail::sizeHint() const
{
    // 宽就是窄条宽度；高按「点 + 间距 + 四个字竖排」的真实高度算，
    // 布局里它会被拉到面板高度，这里只是给个不生气的初值。
    QFont f = font();
    f.setPixelSize(Theme::Metric::FontPxHint);
    const QFontMetrics fm(f);
    const int textH = fm.height() * 4 + kCharGap * 3;
    return QSize(PanelWidth::kRail, kRailPadTop + kDotSize + kDotGap + textH + kRailPadTop);
}

QSize PanelRail::minimumSizeHint() const
{
    return QSize(PanelWidth::kRail, 0);
}

void PanelRail::paintEvent(QPaintEvent *)
{
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing, true);
    p.setRenderHint(QPainter::TextAntialiasing, true);

    // 底色与面板一致：收起来之后这条**还是面板**，只是窄了
    p.fillRect(rect(), Theme::color(hover_ ? QStringLiteral("hover") : QStringLiteral("panel")));

    // 顶部小圆点（原型 .prail .pdot）：整条窄条上唯一的实心点，用来锚住视线
    p.setPen(Qt::NoPen);
    p.setBrush(Theme::color(QStringLiteral("line")));
    p.drawEllipse(QPoint(width() / 2, kRailPadTop + kDotSize / 2), kDotSize / 2, kDotSize / 2);

    // 竖排「属性面板」：一个一个字往下摆（不是整段旋转）。
    // 汉字竖排本来就是「字正着、往下走」，旋转 90° 反而要歪头看，
    // 也就省掉了 QTransform 与「按 DPI 算旋转」的麻烦。
    QFont f = font();
    f.setPixelSize(Theme::Metric::FontPxHint);   // 11px，与「操作说明」同一档
    p.setFont(f);
    const QFontMetrics fm(f);

    const QString text = tr("属性面板");
    const int step = fm.height() + kCharGap;
    int y = kRailPadTop + kDotSize + kDotGap;

    p.setPen(Theme::color(hover_ ? QStringLiteral("text-2") : QStringLiteral("text-3")));
    for (int i = 0; i < text.size(); ++i) {
        p.drawText(QRect(0, y, width(), fm.height()), Qt::AlignHCenter | Qt::AlignVCenter,
                   text.at(i));
        y += step;
    }
}

void PanelRail::mousePressEvent(QMouseEvent *e)
{
    if (e->button() == Qt::LeftButton) {
        emit expandRequested();
        e->accept();
        return;
    }
    QWidget::mousePressEvent(e);
}

// 普通 QWidget 不会为悬停重绘，这里自己记一位（与 CollapsibleSection 同一套路）
void PanelRail::enterEvent(QEvent *e)
{
    hover_ = true;
    update();
    QWidget::enterEvent(e);
}

void PanelRail::leaveEvent(QEvent *e)
{
    hover_ = false;
    update();
    QWidget::leaveEvent(e);
}

/* ================================================================ 收起/展开共用的把手 */

namespace {

// 原型 `.collapsebtn`：13×46、左缘圆角 5px、右缘无边框、白底 + 1px 描边。
//
// 自己画（QAbstractButton 子类，与标题栏三键同一套路），因为它要的是一种很窄的形态：
// 白色小把手贴在面板左缘，悬停时底色抬一档、箭头转成强调色 —— 这些用 QSS 表达要靠
// `#objectName` 再拼一张表，反而比二十行 paintEvent 更难读。
//
// 两态之间**位置、尺寸、外框都不变，只有箭头掉头**（spec 实现决策 17）：所以这里
// 只画 chevron-left，收起时把它绕中心转 180° —— 正好就是 chevron-right
// （两张 SVG 同形，与 CollapsibleSection 里 -90° 的那段是同一个道理）。
// 时长取 `Theme::Motion::ToggleMs`（原型 .collapsebtn svg 的 transform 是 .2s）。
//
// 动画只有外观：角度与底色在绘制时插值，几何与命中测试一动不动。
class ToggleHandle : public QAbstractButton
{
public:
    explicit ToggleHandle(QWidget *parent = nullptr)
        : QAbstractButton(parent)
    {
        setFocusPolicy(Qt::NoFocus);          // 键盘焦点留给面板里的输入控件
        setCursor(Qt::PointingHandCursor);
        setFixedSize(PanelRailGeometry::kToggleW, PanelRailGeometry::kToggleH);

        flip_ = new QVariantAnimation(this);
        flip_->setEasingCurve(QEasingCurve::OutCubic);
        connect(flip_, &QVariantAnimation::valueChanged, this, [this](const QVariant &v) {
            flipAngle_ = v.toReal();
            update();
        });

        hoverAnim_ = new QVariantAnimation(this);
        hoverAnim_->setEasingCurve(QEasingCurve::OutCubic);
        hoverAnim_->setDuration(Theme::Motion::HoverMs);
        connect(hoverAnim_, &QVariantAnimation::valueChanged, this, [this](const QVariant &v) {
            hoverT_ = v.toReal();
            update();
        });
    }

    // 箭头方向 = 「点下去会往哪走」：展开着朝左（收起来），收起后朝右（展开回来）。
    void setCollapsed(bool collapsed)
    {
        if (collapsed_ == collapsed)
            return;
        collapsed_ = collapsed;

        const qreal target = collapsed_ ? 180.0 : 0.0;
        // 不可见时（构造期、面板还没上屏）直接到位，别为一个看不见的控件跑定时器
        if (!flip_ || !isVisible()) {
            if (flip_)
                flip_->stop();
            flipAngle_ = target;
            update();
            return;
        }
        flip_->stop();
        flip_->setDuration(Theme::Motion::ToggleMs);
        flip_->setStartValue(flipAngle_);
        flip_->setEndValue(target);
        flip_->start();
    }

    QSize sizeHint() const override
    {
        return QSize(PanelRailGeometry::kToggleW, PanelRailGeometry::kToggleH);
    }

protected:
    void paintEvent(QPaintEvent *) override
    {
        QPainter p(this);
        p.setRenderHint(QPainter::Antialiasing, true);

        // 半径 5 不超过短边（13）的一半，圆角不会被 Qt 整个退回直角
        constexpr qreal r = 5.0;
        const QRectF box(0.5, 0.5, width() - 1.0, height() - 1.0);

        // 形状：左缘圆角、右缘直角（右缘与面板齐平，不需要描边）
        QPainterPath shape;
        shape.moveTo(box.right(), box.top());
        shape.lineTo(box.left() + r, box.top());
        shape.arcTo(QRectF(box.left(), box.top(), 2 * r, 2 * r), 90, 90);
        shape.lineTo(box.left(), box.bottom() - r);
        shape.arcTo(QRectF(box.left(), box.bottom() - 2 * r, 2 * r, 2 * r), 180, 90);
        shape.lineTo(box.right(), box.bottom());
        shape.closeSubpath();

        // 底色：从面板色过渡到「悬停」或「按下」（原型 transition:background .16s）
        const QColor target = isDown()    ? Theme::color(QStringLiteral("pressed"))
                              : hoverT_ > 0.0 ? Theme::color(QStringLiteral("hover"))
                                              : Theme::color(QStringLiteral("panel"));
        p.fillPath(shape, mixColor(Theme::color(QStringLiteral("panel")), target, hoverT_));

        // 描边只走左 / 上 / 下三条：右缘那条画出来就是面板里的一条竖线（原型 border-right:0）
        QPainterPath edge;
        edge.moveTo(box.right(), box.top());
        edge.lineTo(box.left() + r, box.top());
        edge.arcTo(QRectF(box.left(), box.top(), 2 * r, 2 * r), 90, 90);
        edge.lineTo(box.left(), box.bottom() - r);
        edge.arcTo(QRectF(box.left(), box.bottom() - 2 * r, 2 * r, 2 * r), 180, 90);
        edge.lineTo(box.right(), box.bottom());
        p.strokePath(edge, QPen(Theme::color(QStringLiteral("line")), 1));

        // 图标 10px：原型 .collapsebtn svg 就是 10×10。**这里不能套用标题栏的 12px 结论** ——
        // 按钮宽 13、内宽 11，12px 的图标连一个像素的余量都没有。
        constexpr int px = 10;
        const int left = (width() - px) / 2;
        const int top = (height() - px) / 2;

        // 角度 0 / 180 都是 90 的整数倍：落点仍是整数像素，静止时一个像素都不重采样。
        p.save();
        p.translate(left + px / 2.0, top + px / 2.0);
        if (!qFuzzyIsNull(flipAngle_) && !qFuzzyCompare(flipAngle_, 180.0))
            p.setRenderHint(QPainter::SmoothPixmapTransform, true);
        p.rotate(flipAngle_);

        // 图标色：text-3 → accent（原型 transition:color .16s）。
        // 两张固定颜色的图交叉淡入，而不是每帧生成插值色去渲染 —— 后者的缓存键每帧都新。
        drawIcon(p, Theme::color(QStringLiteral("text-3")), 1.0 - hoverT_);
        drawIcon(p, Theme::color(QStringLiteral("accent")), hoverT_);
        p.restore();
    }

    void enterEvent(QEvent *e) override
    {
        animateHover(1.0);
        QAbstractButton::enterEvent(e);
    }

    void leaveEvent(QEvent *e) override
    {
        animateHover(isDown() ? 1.0 : 0.0);
        QAbstractButton::leaveEvent(e);
    }

    void mousePressEvent(QMouseEvent *e) override
    {
        QAbstractButton::mousePressEvent(e);
        animateHover(1.0);
    }

    void mouseReleaseEvent(QMouseEvent *e) override
    {
        QAbstractButton::mouseReleaseEvent(e);
        animateHover(underMouse() ? 1.0 : 0.0);
    }

private:
    static QColor mixColor(const QColor &from, const QColor &to, qreal t)
    {
        const qreal k = qBound(0.0, t, 1.0);
        const qreal keep = 1.0 - k;
        return QColor(qRound(from.red()   * keep + to.red()   * k),
                      qRound(from.green() * keep + to.green() * k),
                      qRound(from.blue()  * keep + to.blue()  * k));
    }

    void drawIcon(QPainter &p, const QColor &ink, qreal opacity)
    {
        if (opacity <= 0.0)
            return;
        constexpr int px = 10;
        const QPixmap pm = IconTint::pixmap(QStringLiteral("chevron-left"), ink, px,
                                           devicePixelRatioF());
        p.setOpacity(qBound(0.0, opacity, 1.0));
        // 坐标是相对刚才 translate 过去的图标中心，所以这里从负半格画起
        p.drawPixmap(QPointF(-px / 2.0, -px / 2.0), pm);
        p.setOpacity(1.0);
    }

    // QAbstractButton 不给 enter/leave 重绘，这里自己驱动（与 BarButton 同一套路）
    void animateHover(qreal target)
    {
        if (!hoverAnim_ || qFuzzyCompare(hoverT_ + 1.0, target + 1.0))
            return;
        hoverAnim_->stop();
        hoverAnim_->setStartValue(hoverT_);
        hoverAnim_->setEndValue(target);
        hoverAnim_->start();
    }

    bool collapsed_ = false;
    qreal flipAngle_ = 0.0;   // 0 = 展开（朝左），180 = 收起（朝右）
    qreal hoverT_ = 0.0;      // 0 = 静止，1 = 悬停/按下
    QVariantAnimation *flip_ = nullptr;
    QVariantAnimation *hoverAnim_ = nullptr;
};

} // namespace

/* ================================================================ 容器 */

PanelHost::PanelHost(QWidget *parent)
    : QWidget(parent)
{
    rail_ = new PanelRail(this);
    connect(rail_, &PanelRail::expandRequested, this, &PanelHost::toggleCollapsed);

    // 两态同一个位置叠着放：切态只换页，不动几何 —— 面板的左缘因此不会跳一下。
    stack_ = new QStackedLayout(this);
    stack_->setContentsMargins(0, 0, 0, 0);
    stack_->addWidget(rail_);
    stack_->setCurrentWidget(rail_);

    toggle_ = new ToggleHandle(this);
    connect(toggle_, &QAbstractButton::clicked, this, &PanelHost::toggleCollapsed);

    // 展开态默认的最小宽度：没拖过宽度之前，面板就是这么宽
    setMinimumWidth(PanelWidth::kMinExpanded);
}

void PanelHost::setExpandedContent(QWidget *content)
{
    if (!content || content == expanded_)
        return;

    expanded_ = content;
    stack_->addWidget(content);
    applyState();
}

void PanelHost::setToggleLayer(QWidget *layer)
{
    // 覆盖层必须是本控件的**祖先**：定位用的是 mapTo(layer, ...)，Qt 那条路只认祖先。
    if (!layer || !layer->isAncestorOf(this))
        return;

    toggleLayer_ = layer;

    // 父对象换成覆盖层：Qt 会把子控件裁到父控件矩形内，而按钮有 13px 压在视口上，
    // 留在 PanelHost 里那 13px 就一个像素都画不出来（实测）。
    if (toggle_->parentWidget() != layer)
        toggle_->setParent(layer);

    placeToggleButton();
    toggle_->show();
}

void PanelHost::rememberExpandedWidth(int width)
{
    memory_ = PanelWidth::remember(memory_, width);
}

void PanelHost::setCollapsed(bool collapsed)
{
    if (memory_.collapsed == collapsed)
        return;

    // 收起这一刻把当前真实宽度记下来；展开时它原样留着（= 收起前的宽度）
    memory_ = PanelWidth::toggle(memory_, width());

    applyState();
    emit collapsedChanged(memory_.collapsed);
}

void PanelHost::toggleCollapsed()
{
    setCollapsed(!memory_.collapsed);
}

void PanelHost::applyState()
{
    const bool rail = memory_.collapsed;

    if (expanded_)
        stack_->setCurrentWidget(rail ? static_cast<QWidget *>(rail_) : expanded_);
    else
        stack_->setCurrentWidget(rail_);

    // 收起态的下限就是窄条本身：分裂器（childrenCollapsible=false）因此不会拦住 26px。
    // 展开态的下限是面板下限，防止拖到把内容挤成一条。
    setMinimumWidth(rail ? PanelWidth::kRail : PanelWidth::kMinExpanded);

    // 箭头方向与提示：收起态朝右（点一下展开），展开态朝左。
    // raise() 是硬要求：收起态面板只剩 26px，把手压在视口之上才看得见。
    ToggleHandle *handle = static_cast<ToggleHandle *>(toggle_);
    handle->setCollapsed(rail);
    handle->setToolTip(rail ? tr("展开面板") : tr("收起面板"));
    handle->raise();

    placeToggleButton();
    updateGeometry();
}

void PanelHost::placeToggleButton()
{
    if (!toggle_)
        return;

    // 坐标系：本类的几何是相对父控件（分裂器）的，覆盖层未必是同一个父控件，
    // 所以按「本类原点在覆盖层里的位置」整体换算，换算本身是纯函数。
    QWidget *layer = toggle_->parentWidget();
    if (!layer)
        return;

    const QRect panelRect(mapTo(layer, QPoint(0, 0)), size());
    toggle_->setGeometry(PanelRailGeometry::toggleRect(panelRect));
    toggle_->raise();
}

void PanelHost::resizeEvent(QResizeEvent *e)
{
    QWidget::resizeEvent(e);
    placeToggleButton();
}

// 拖分隔条 / 换态时面板会整体平移，按钮跟着走 —— 这一条少了，把手会停在旧位置
void PanelHost::moveEvent(QMoveEvent *e)
{
    QWidget::moveEvent(e);
    placeToggleButton();
}

void PanelHost::showEvent(QShowEvent *e)
{
    QWidget::showEvent(e);
    placeToggleButton();
}

// 被塞进分裂器时会 reparent（addWidget）：按钮这时才拿得到正确的祖先坐标系。
// 注意这里**不能**退回「父对象 = 分裂器」—— QSplitter 会把任何子控件当成第三个面板
// （实测 count 2→3）。覆盖层还没指定时就先留在本类里，反正它此刻也确实没地方摆。
bool PanelHost::event(QEvent *e)
{
    if (e->type() == QEvent::ParentChange && toggle_ && toggleLayer_) {
        if (toggle_->parentWidget() != toggleLayer_)
            toggle_->setParent(toggleLayer_);
        placeToggleButton();
        toggle_->show();
    }
    return QWidget::event(e);
}
