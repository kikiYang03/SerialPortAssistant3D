#include "IconButton.h"

#include "IconTint.h"
#include "Theme.h"

#include <QEvent>
#include <QFontMetrics>
#include <QPainter>

namespace {

constexpr int kIconPx = 14;   // 原型 .btn svg{width:14px;height:14px}
constexpr int kPadX   = 12;   // 原型 .btn padding: 0 12px
constexpr int kGap    = 6;    // 图标与文字之间

} // namespace

IconButton::IconButton(QWidget *parent)
    : QAbstractButton(parent)
{
    setObjectName(QStringLiteral("iconButton"));
    setCursor(Qt::PointingHandCursor);
    setFocusPolicy(Qt::NoFocus);                       // 命令栏不吃 Tab 焦点，焦点留给页面里的输入控件
    setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    setFixedHeight(Theme::Metric::ControlH);
    updateWidth();
}

void IconButton::setIconName(const QString &name)
{
    if (iconName_ == name)
        return;
    iconName_ = name;
    updateWidth();
    update();
}

void IconButton::setLabel(const QString &text)
{
    if (label_ == text)
        return;
    label_ = text;
    QAbstractButton::setText(text);   // 无障碍 / 工具提示读这里；绘制用 label_
    updateWidth();
    update();
}

void IconButton::setAppearance(Appearance appearance)
{
    if (appearance_ == appearance)
        return;
    appearance_ = appearance;
    update();
}

void IconButton::updateWidth()
{
    int w = 2 * kPadX;
    if (!iconName_.isEmpty()) {
        w += kIconPx;
        if (!label_.isEmpty())
            w += kGap;
    }
    if (!label_.isEmpty())
        w += fontMetrics().horizontalAdvance(label_);
    setFixedWidth(w);
}

QSize IconButton::sizeHint() const
{
    return QSize(width(), Theme::Metric::ControlH);
}

QSize IconButton::minimumSizeHint() const
{
    return sizeHint();
}

void IconButton::paintEvent(QPaintEvent *)
{
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing, true);

    const bool enabled = isEnabled();
    const bool hover = enabled && underMouse();
    const bool down = enabled && isDown();

    QColor bg;
    QColor border;
    QColor fg;         // 文字色
    QColor iconColor;  // 图标色

    if (!enabled) {
        bg = Theme::color(QStringLiteral("disabled-bg"));
        border = Theme::color(QStringLiteral("line"));
        fg = Theme::color(QStringLiteral("text-3"));
        iconColor = fg;
    } else if (appearance_ == Primary) {
        // 实底：悬停 / 按下都从同一个 accent 派生，不另立两个色值
        const QColor accent = Theme::color(QStringLiteral("accent"));
        bg = down ? accent.darker(115) : (hover ? accent.darker(108) : accent);
        border = bg;
        fg = Theme::color(QStringLiteral("on-accent"));
        iconColor = fg;
    } else {
        bg = down ? Theme::color(QStringLiteral("pressed"))
                  : (hover ? Theme::color(QStringLiteral("hover"))
                           : Theme::color(QStringLiteral("bg")));
        border = Theme::color(QStringLiteral("line-strong"));
        fg = Theme::color(QStringLiteral("text-1"));
        iconColor = hover ? Theme::color(QStringLiteral("text-1"))
                          : Theme::color(QStringLiteral("text-2"));
    }

    // 半像素内缩：1px 描边落在像素中心上，四边才是实的，不会一边虚一边实
    const QRectF box = QRectF(rect()).adjusted(0.5, 0.5, -0.5, -0.5);
    p.setPen(QPen(border, 1));
    p.setBrush(bg);
    p.drawRoundedRect(box, Theme::Metric::Radius, Theme::Metric::Radius);

    const QFontMetrics fm = fontMetrics();
    const int iconW = iconName_.isEmpty() ? 0 : kIconPx;
    const int textW = label_.isEmpty() ? 0 : fm.horizontalAdvance(label_);
    const int total = iconW + (iconW > 0 && textW > 0 ? kGap : 0) + textW;

    int x = (width() - total) / 2;
    if (iconW > 0) {
        // dpr 显式传本控件的：不传的话 IconTint 会退回主屏 dpr，窗口挪到另一块缩放不同的
        // 屏上就成了「按错的 dpr 出图再缩放」，图标发虚（工单 12 复核的那条）。
        const QPixmap pm = IconTint::pixmap(iconName_, iconColor, kIconPx, devicePixelRatioF());
        p.drawPixmap(x, (height() - kIconPx) / 2, pm);
        x += iconW + (textW > 0 ? kGap : 0);
    }
    if (textW > 0) {
        p.setPen(fg);
        p.drawText(QRect(x, 0, textW, height()), Qt::AlignLeft | Qt::AlignVCenter, label_);
    }
}

// QAbstractButton 不会为悬停重绘：图标与底色都要从三态里换一个，所以自己叫一次 update()
void IconButton::enterEvent(QEvent *e)
{
    QAbstractButton::enterEvent(e);
    update();
}

void IconButton::leaveEvent(QEvent *e)
{
    QAbstractButton::leaveEvent(e);
    update();
}
