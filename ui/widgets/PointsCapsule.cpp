#include "PointsCapsule.h"

#include "Theme.h"

#include <QColor>
#include <QFontMetrics>
#include <QPainter>
#include <QPen>

namespace {

// 与 docs/prototype-v2.html 的 .chip 一致：padding 4px 9px、圆角 5、字段名与点数间隔 6。
constexpr int kPadX        = 9;
constexpr int kPadY        = 4;
constexpr int kChipRadius  = 5;
constexpr int kChipGap     = 6;   // .hud.tr gap: 6px

// 浮层压在深色画布上，配色从画布色与 on-accent（白）派生，不写字面量
// （与 FloatingBar.cpp 里那三个函数同一个口径，两边各留一份是因为它们都很短、
//   且不希望为此多一个公共头 —— 变色值时要一起改的就是这两处）。
QColor whiteAlpha(qreal a)
{
    QColor c = Theme::palette().onAccent;
    c.setAlphaF(a);
    return c;
}

QColor panelColor()
{
    QColor c = Theme::palette().canvas;
    c = c.lighter(150);
    c.setAlphaF(0.92);
    return c;
}

// 数值列宽度的量化步长：位数不变时宽度不变，跨位时一次跳 4px 的整数倍，
// 而不是每次重绘都挪一两个像素（那才叫「抖」）。
constexpr int kValueStep = 4;

} // namespace

PointsCapsule::PointsCapsule(QWidget *parent)
    : QWidget(parent)
{
    setObjectName(QStringLiteral("pointsCapsule"));
    // 纯读数，鼠标穿透：压在它上面按下拖动应当照常转视角（spec 实现决策 26）
    setAttribute(Qt::WA_TransparentForMouseEvents, true);
    setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    setFocusPolicy(Qt::NoFocus);

    keyRealtime_ = tr("实时点云");
    keyMap_      = tr("地图点云");

    // 字号取 Theme 的小号：这一块是辅助读数，不该和正文抢。
    // 数值加粗：点数是这一块唯一要读的东西。
    keyFont_ = font();
    keyFont_.setPointSize(Theme::Metric::FontPtSmall);

    valueFont_ = keyFont_;
    valueFont_.setBold(true);

    valueRealtime_ = formatCount(realtime_);
    valueMap_      = formatCount(map_);

    buildGeometry();
}

void PointsCapsule::buildGeometry()
{
    const QFontMetrics fmKey(keyFont_);
    const QFontMetrics fmValue(valueFont_);

    // 两枚 chip 的字段名用同一个宽度（取长的那个），左右两块的数字才对得齐
    keyWidth_ = qMax(fmKey.horizontalAdvance(keyRealtime_), fmKey.horizontalAdvance(keyMap_));

    // 数值列：两个数里较宽的那个，向上取整到 kValueStep 的倍数。
    // 下限给一个数字的宽度，免得刚开张时（"0"）挤成一条。
    const int need = qMax(fmValue.horizontalAdvance(valueRealtime_),
                          fmValue.horizontalAdvance(valueMap_));
    valueWidth_ = qMax(fmValue.horizontalAdvance(QStringLiteral("0")),
                       ((need + kValueStep - 1) / kValueStep) * kValueStep);

    const int chipW = kPadX * 2 + keyWidth_ + kChipGap + valueWidth_;
    const int chipH = qMax(fmKey.height(), fmValue.height()) + kPadY * 2;

    chipRealtime_ = QRect(0, 0, chipW, chipH);
    chipMap_      = QRect(chipW + kChipGap, 0, chipW, chipH);

    // 按内容固定尺寸：整块就是两枚 chip 并排，不多一个像素
    contentSize_ = QSize(chipMap_.right() + 1, chipH);
    setFixedSize(contentSize_);
}

void PointsCapsule::setCounts(int realtimePoints, int mapPoints)
{
    if (realtimePoints == realtime_ && mapPoints == map_)
        return;   // 500ms 推一次，值没变就别重绘

    realtime_ = realtimePoints;
    map_      = mapPoints;

    const QString rt  = formatCount(realtime_);
    const QString map = formatCount(map_);
    const bool textChanged = (rt != valueRealtime_) || (map != valueMap_);
    if (!textChanged) {
        update();   // 同一个数字，只是重绘也没必要再排一次几何
        return;
    }
    valueRealtime_ = rt;
    valueMap_      = map;

    // 位数变了才可能改尺寸；位置是按尺寸算出来的，所以宽高真变了要请外面重摆。
    // 宽度量化到 4px 之后，这一步只在跨位数时发生（"0" → "24.5 k"），不是每帧。
    const QSize before = contentSize_;
    buildGeometry();
    if (contentSize_ != before)
        emit contentSizeChanged();

    update();
}


QRect PointsCapsule::placeRect(const QSize &viewport, const QSize &capsule)
{
    // 极端尺寸下不要负坐标：窄视口里贴左边、矮视口里贴顶（被裁总比跑到视口外好）
    const int x = qMax(0, viewport.width() - capsule.width() - kMargin);
    const int y = qMax(0, qMin(kMargin, viewport.height() - capsule.height()));
    return QRect(QPoint(x, y), capsule);
}

QString PointsCapsule::formatCount(int points)
{
    if (points < 0)
        return QStringLiteral("-");   // 还没数据：与竞态无关，只是「没到过」

    if (points < 1000)
        return QString::number(points);

    if (points < 1000000)
        return QString::number(points / 1000.0, 'f', 1) + QStringLiteral(" k");

    return QString::number(points / 1000000.0, 'f', 2) + QStringLiteral(" M");
}

QSize PointsCapsule::sizeHint() const
{
    return contentSize_;
}

QSize PointsCapsule::minimumSizeHint() const
{
    // 读数块没有「更小也还能用」的形态：压小就是把某一枚 chip 裁掉
    return contentSize_;
}

void PointsCapsule::paintEvent(QPaintEvent *)
{
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing, true);

    paintChip(p, chipRealtime_, keyRealtime_, valueRealtime_);
    paintChip(p, chipMap_,      keyMap_,      valueMap_);
}

void PointsCapsule::paintChip(QPainter &p, const QRect &chip, const QString &key,
                              const QString &value) const
{
    // 半像素内缩：1px 描边落在像素中心上，四边才是实的
    const QRectF box = QRectF(chip).adjusted(0.5, 0.5, -0.5, -0.5);
    p.setPen(QPen(whiteAlpha(0.10), 1));
    p.setBrush(panelColor());
    p.drawRoundedRect(box, kChipRadius, kChipRadius);

    const QRect textBox(chip.left() + kPadX, chip.top() + kPadY,
                        chip.width() - kPadX * 2, chip.height() - kPadY * 2);

    // 字段名：比数值弱一档（白 45%），压在深底上约等于原型里的 #6b7c93
    p.setFont(keyFont_);
    p.setPen(whiteAlpha(0.45));
    p.drawText(QRect(textBox.left(), textBox.top(), keyWidth_, textBox.height()),
               Qt::AlignLeft | Qt::AlignVCenter, key);

    // 点数：右对齐在固定宽列里 —— 位数变化只改内容宽度，整块不动
    p.setFont(valueFont_);
    p.setPen(whiteAlpha(0.88));
    p.drawText(QRect(textBox.left() + keyWidth_ + kChipGap, textBox.top(), valueWidth_,
                     textBox.height()),
               Qt::AlignRight | Qt::AlignVCenter, value);
}
