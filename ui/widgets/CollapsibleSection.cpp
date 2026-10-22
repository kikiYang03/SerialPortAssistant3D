#include "CollapsibleSection.h"

#include "IconTint.h"
#include "Theme.h"

#include <QFontMetrics>
#include <QMouseEvent>
#include <QPainter>
#include <QPropertyAnimation>
#include <QVBoxLayout>

namespace {

// 标题行左右内边距（原型 .sech padding: 0 12px）
constexpr int kPadX = Theme::Metric::Gap;

// chevron 的逻辑像素：原型 .sech .chev 12×12。
// 10px 时 SVG 的线宽（24 viewBox / stroke 2）会落到 0.83 物理像素，
// 被抗锯齿摊成一片浅灰；12px 刚好一个物理像素（标题栏的图标同一个结论，见工单 05）。
constexpr int kChevronPx = 12;

// 标题与 chevron 之间
constexpr int kChevronGap = Theme::Metric::GapSm;

// 内容区四周：左右与标题对齐，底部留一格，免得贴住下一条分隔线
constexpr int kBodyPadBottom = Theme::Metric::GapSm;

// 收起态的角度。原型 `.sec.collapsed .chev{transform:rotate(-90deg)}`。
//
// 为什么可以只拿 chevron-down 转出来：两张图标是同一段路径的两个方向 ——
// chevron-down 是 `m6 9 6 6 6-6`（x 6..18 / y 9..15），绕中心转 -90° 之后横竖范围互换，
// 正好落在 x 9..15 / y 6..18，与 chevron-right 的路径完全重合。所以不必在运行时切两张图，
// 也就不必为「转动中间态」再开一套缓存键。
constexpr qreal kCollapsedAngle = -90.0;

} // namespace

CollapsibleSection::CollapsibleSection(const QString &title, QWidget *parent)
    : QWidget(parent), title_(title)
{
    // 悬停要换整行的底色与 chevron 的颜色，不按键也得收到 mouseMove
    setMouseTracking(true);
    setFocusPolicy(Qt::NoFocus);   // 键盘焦点留给页面里的输入控件

    body_ = new QWidget(this);
    bodyLayout_ = new QVBoxLayout(body_);
    bodyLayout_->setContentsMargins(kPadX, 0, kPadX, kBodyPadBottom);
    bodyLayout_->setSpacing(Theme::Metric::GapSm);

    // 标题行不是控件，是这块 widget 的顶部一条：布局给它让出高度，绘制在 paintEvent 里。
    auto *root = new QVBoxLayout(this);
    root->setContentsMargins(0, Theme::Metric::SectionHeaderH, 0, 0);
    root->setSpacing(0);
    root->addWidget(body_);

    chevronAnim_ = new QPropertyAnimation(this, "chevronAngle", this);
    chevronAnim_->setDuration(Theme::Motion::SlideMs);
    chevronAnim_->setEasingCurve(QEasingCurve::OutCubic);
}

QRect CollapsibleSection::headerRect() const
{
    return QRect(0, 0, width(), Theme::Metric::SectionHeaderH);
}

void CollapsibleSection::setTitle(const QString &title)
{
    if (title_ == title)
        return;
    title_ = title;
    update();
}

void CollapsibleSection::setCollapsed(bool collapsed)
{
    if (collapsed_ == collapsed)
        return;

    collapsed_ = collapsed;
    // 只收内容区：标题行留着，否则收起来之后就没有能点回来的地方了。
    body_->setVisible(!collapsed_);
    // 隐藏的控件会被布局当作空项，尺寸自然会缩到「标题行」那一条，不用自己算高度。
    // （内容区是瞬间收掉的：让高度也动起来会把下面 6 个分组一起带着跳，反而更晃。）
    updateGeometry();
    animateChevronTo(collapsed_ ? kCollapsedAngle : 0.0);
    update();
    emit collapsedChanged(collapsed_);
}

void CollapsibleSection::animateChevronTo(qreal target)
{
    if (!chevronAnim_)
        return;

    // 没显示出来就没有「过程」可看：直接到位，省掉一次看不见的定时器序列。
    if (!isVisible()) {
        chevronAnim_->stop();
        setChevronAngle(target);
        return;
    }

    // 从**当前真实角度**接着转：连点两下不会先跳回另一端再出发。
    chevronAnim_->stop();
    if (qFuzzyCompare(chevronAngle_ + 1.0, target + 1.0)) {
        setChevronAngle(target);
        return;
    }
    chevronAnim_->setStartValue(chevronAngle_);
    chevronAnim_->setEndValue(target);
    chevronAnim_->start();
}

void CollapsibleSection::setChevronAngle(qreal angle)
{
    if (qFuzzyCompare(chevronAngle_ + 1.0, angle + 1.0))
        return;
    chevronAngle_ = angle;
    update();   // 只重画标题行那条，动画的每一帧也只花这么点
}

void CollapsibleSection::toggle()
{
    setCollapsed(!collapsed_);
}

void CollapsibleSection::paintEvent(QPaintEvent *)
{
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing, true);
    p.setRenderHint(QPainter::TextAntialiasing, true);

    const QRect hdr = headerRect();

    // 悬停底：整行一条，与原型 .sech:hover 一个量级（借 hover 色，不另立一档）
    if (hoverHeader_ && isEnabled())
        p.fillRect(hdr, Theme::color(QStringLiteral("hover")));

    // 底部分隔线：每组一条。最后一组（操作说明）下面也留一条 —— 它正好收回
    // 面板内容的末尾，比「最后一条没有线」更稳定：增删分组时不用回来改谁带线。
    const QColor line = Theme::color(QStringLiteral("line"));
    p.setPen(QPen(line, 1));
    p.drawLine(0, height() - 1, width(), height() - 1);

    // 标题：字号与正文同级（原型里标题与内容同为 12px），靠字重与位置分层，
    // 不靠放大字号 —— 侧边栏里字号一多，读数就不成组了。
    QFont titleFont = font();
    titleFont.setWeight(QFont::DemiBold);
    p.setFont(titleFont);

    const QFontMetrics fm(titleFont);
    const int textLeft = kPadX;
    const int textRight = width() - kPadX - kChevronPx - kChevronGap;
    const QRect textRect(textLeft, hdr.top(), qMax(0, textRight - textLeft), hdr.height());

    p.setPen(Theme::color(QStringLiteral("text-1")));
    p.drawText(textRect, Qt::AlignLeft | Qt::AlignVCenter,
               fm.elidedText(title_, Qt::ElideRight, textRect.width()));

    // chevron：展开朝下、收起朝右（同一张 chevron-down 转 -90°，理由见文件头的
    // kCollapsedAngle）。悬停时抬一档颜色 —— 颜色是静态切换，只有角度是过渡的：
    // 原型对 .chev 也只写了 transform 的 transition。
    const QColor iconColor = Theme::color(hoverHeader_ ? QStringLiteral("text-2")
                                                       : QStringLiteral("text-3"));
    const QPixmap chev = IconTint::pixmap(QStringLiteral("chevron-down"), iconColor, kChevronPx,
                                        devicePixelRatioF());
    const int chevLeft = width() - kPadX - kChevronPx;
    const int chevTop = hdr.top() + (hdr.height() - kChevronPx) / 2;

    if (qFuzzyIsNull(chevronAngle_)) {
        // 展开态（也是动画落点）：走整数坐标直贴，12px 图标一个像素都不被重采样
        p.drawPixmap(chevLeft, chevTop, chev);
    } else {
        // 绕图标框中心转。0 与 -90 都是 90 的整数倍，落点仍是整数像素，
        // 所以静止时不会发虚；只有中间的几十毫秒是重采样的。
        p.save();
        p.setRenderHint(QPainter::SmoothPixmapTransform, true);
        p.translate(chevLeft + kChevronPx / 2.0, chevTop + kChevronPx / 2.0);
        p.rotate(chevronAngle_);
        p.drawPixmap(QPointF(-kChevronPx / 2.0, -kChevronPx / 2.0), chev);
        p.restore();
    }
}

void CollapsibleSection::mousePressEvent(QMouseEvent *e)
{
    if (e->button() == Qt::LeftButton && headerRect().contains(e->pos())) {
        toggle();
        e->accept();
        return;
    }
    QWidget::mousePressEvent(e);
}

void CollapsibleSection::mouseMoveEvent(QMouseEvent *e)
{
    setHeaderHover(headerRect().contains(e->pos()));
    QWidget::mouseMoveEvent(e);
}

void CollapsibleSection::leaveEvent(QEvent *e)
{
    setHeaderHover(false);
    QWidget::leaveEvent(e);
}

// 普通 QWidget 不会为悬停重绘，光标也要跟着「只有标题行能点」来变：
// 光标挂在整块 widget 上的话，内容区空白处也会变成手型，那是在骗人。
void CollapsibleSection::setHeaderHover(bool hover)
{
    if (hoverHeader_ == hover)
        return;
    hoverHeader_ = hover;
    setCursor(hover ? Qt::PointingHandCursor : Qt::ArrowCursor);
    update();
}
