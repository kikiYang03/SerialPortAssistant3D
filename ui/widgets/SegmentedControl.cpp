#include "SegmentedControl.h"

#include "Theme.h"

#include <QMouseEvent>
#include <QPainter>
#include <QPropertyAnimation>

namespace {

constexpr int kTrackPadX  = 2;    // 托盘内边距（原型 .seg padding: 2px）
constexpr int kSegmentPadX = 14;  // 每段文字左右的内边距（原型 padding: 5px 14px）
constexpr int kMinSegmentW = 76;  // 段宽下限：三字标题也不至于挤成一条

} // namespace

SegmentedControl::SegmentedControl(QWidget *parent)
    : QWidget(parent)
{
    setObjectName(QStringLiteral("segmentedControl"));
    setFixedHeight(Theme::Metric::ControlH);
    setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    setMouseTracking(true);                      // 悬停要换文字色，不按键也得收到 mouseMove
    setCursor(Qt::PointingHandCursor);

    m_anim = new QPropertyAnimation(this, "thumbPos", this);
    // 时长与折叠箭头共用一个常量（Theme::Motion::SlideMs，原型 .seg 是 .16s）：
    // 同一个界面里两处滑动各用各的数字，迟早会滑出不齐的节奏。
    m_anim->setDuration(Theme::Motion::SlideMs);
    m_anim->setEasingCurve(QEasingCurve::OutCubic);
}

void SegmentedControl::addSegment(const QString &text)
{
    m_texts.append(text);
    updateMetrics();

    if (m_current < 0) {
        // 第一段直接到位，不滑（初值没有「上一段」可滑）
        m_current = 0;
        m_thumbPos = 0.0;
        emit currentChanged(m_current);
    }
}

void SegmentedControl::updateMetrics()
{
    const QFontMetrics fm = fontMetrics();
    int w = kMinSegmentW;
    for (const QString &t : m_texts)
        w = qMax(w, fm.horizontalAdvance(t) + 2 * kSegmentPadX);
    m_segmentWidth = w;

    setFixedWidth(2 * kTrackPadX + m_segmentWidth * qMax(1, m_texts.size()));
    update();
}

QSize SegmentedControl::sizeHint() const
{
    return QSize(2 * kTrackPadX + m_segmentWidth * qMax(1, m_texts.size()),
                 Theme::Metric::ControlH);
}

QSize SegmentedControl::minimumSizeHint() const
{
    return sizeHint();
}

void SegmentedControl::setCurrentIndex(int index)
{
    if (index < 0 || index >= m_texts.size() || index == m_current)
        return;

    const int previous = m_current;
    m_current = index;
    emit currentChanged(m_current);

    // 从「当前实际停留的位置」滑，而不是从上一个目标滑：连点两下时不会回跳
    if (previous < 0 || !isVisible()) {
        m_anim->stop();
        setThumbPos(index);
    } else {
        m_anim->stop();
        m_anim->setStartValue(m_thumbPos);
        m_anim->setEndValue(qreal(index));
        m_anim->start();
    }
}

void SegmentedControl::setThumbPos(qreal pos)
{
    if (qFuzzyCompare(m_thumbPos + 1.0, pos + 1.0))
        return;
    m_thumbPos = pos;
    update();
}

QRectF SegmentedControl::trackRect() const
{
    return QRectF(rect()).adjusted(kTrackPadX, kTrackPadX, -kTrackPadX, -kTrackPadX);
}

QRectF SegmentedControl::segmentRect(qreal pos) const
{
    const QRectF track = trackRect();
    return QRectF(track.left() + pos * m_segmentWidth, track.top(),
                  m_segmentWidth, track.height());
}

int SegmentedControl::indexAt(const QPoint &pos) const
{
    if (m_segmentWidth <= 0)
        return -1;
    const QRectF track = trackRect();
    if (!track.contains(pos))
        return -1;
    const int i = int((pos.x() - track.left()) / m_segmentWidth);
    return (i >= 0 && i < m_texts.size()) ? i : -1;
}

void SegmentedControl::paintEvent(QPaintEvent *)
{
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing, true);
    p.setRenderHint(QPainter::TextAntialiasing, true);

    const QColor line = Theme::color(QStringLiteral("line"));

    // 托盘：底色退一格（hover 就是原型里那块 #eef2f7 的量级），与页面白底分开
    const QRectF track = trackRect().adjusted(0.5, 0.5, -0.5, -0.5);
    p.setPen(QPen(line, 1));
    p.setBrush(Theme::color(QStringLiteral("hover")));
    p.drawRoundedRect(track, Theme::Metric::Radius, Theme::Metric::Radius);

    // 指示条：白块 + 1px 描边（原型用投影，这里用描边替代，不引 QGraphicsDropShadowEffect——
    // 那个会强制整块走离屏渲染，代价远大于一条线）
    if (m_current >= 0 && m_segmentWidth > 0) {
        const QRectF thumb = segmentRect(m_thumbPos);
        p.setPen(QPen(line, 1));
        p.setBrush(Theme::color(QStringLiteral("bg")));
        p.drawRoundedRect(thumb, Theme::Metric::RadiusSm, Theme::Metric::RadiusSm);
    }

    // 文字：选中 accent，悬停 text1，其余 text2（与全局「重要性阶梯」一致）
    for (int i = 0; i < m_texts.size(); ++i) {
        const QRectF cell = segmentRect(i);
        QColor color = Theme::color(QStringLiteral("text-2"));
        if (i == m_current)
            color = Theme::color(QStringLiteral("accent"));
        else if (i == m_hover)
            color = Theme::color(QStringLiteral("text-1"));

        p.setPen(color);
        p.drawText(QRectF(cell.left(), cell.top(), cell.width(), cell.height()),
                   Qt::AlignCenter, m_texts.at(i));
    }
}

void SegmentedControl::mousePressEvent(QMouseEvent *e)
{
    if (e->button() != Qt::LeftButton) {
        QWidget::mousePressEvent(e);
        return;
    }
    const int index = indexAt(e->pos());
    if (index < 0)
        return;
    // 只有用户点击才发 activated：它才是「换页」的触发源
    setCurrentIndex(index);
    emit segmentActivated(index);
}

void SegmentedControl::mouseMoveEvent(QMouseEvent *e)
{
    const int hover = indexAt(e->pos());
    if (hover != m_hover) {
        m_hover = hover;
        update();
    }
    QWidget::mouseMoveEvent(e);
}

void SegmentedControl::leaveEvent(QEvent *e)
{
    if (m_hover != -1) {
        m_hover = -1;
        update();
    }
    QWidget::leaveEvent(e);
}
