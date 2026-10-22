#include "dualrangeslider.h"
#include "RangeLimits.h"
#include <QPainter>
#include <QFontMetrics>
#include <QMouseEvent>
#include <QDebug>
#include <cmath>

DualRangeSlider::DualRangeSlider(QWidget* parent)
    : QWidget(parent)
{
    setMinimumSize(200, 40);
    m_sliderWidth = 200;
    updateHandleRects();
}

void DualRangeSlider::setRange(double min, double max)
{
    m_min = min;
    m_max = max;

    // 限制当前值在范围内
    m_lowerValue = std::max(m_min, std::min(m_lowerValue, m_max));
    m_upperValue = std::max(m_min, std::min(m_upperValue, m_max));

    updateHandleRects();
    update();
}

void DualRangeSlider::setValues(double lower, double upper)
{
    // 夹范围 + 反了就调头：这一段是纯逻辑，搬到 RangeLimits 里单独测（工单 08）
    const RangeLimits::Interval v = RangeLimits::normalize(lower, upper, m_min, m_max);
    m_lowerValue = v.lower;
    m_upperValue = v.upper;

    updateHandleRects();
    update();
}

void DualRangeSlider::setValueSuffix(const QString& suffix)
{
    if (m_suffix == suffix)
        return;
    m_suffix = suffix;
    update();   // 只是文字变了，手柄位置不用重算
}

int DualRangeSlider::valueToPos(double value) const
{
    double normalized = (value - m_min) / (m_max - m_min);
    return m_handleRadius + static_cast<int>(normalized * (m_sliderWidth - 2 * m_handleRadius));
}

double DualRangeSlider::posToValue(int pos) const
{
    int effectivePos = std::max(m_handleRadius, std::min(pos, m_sliderWidth - m_handleRadius));
    double normalized = (effectivePos - m_handleRadius) / static_cast<double>(m_sliderWidth - 2 * m_handleRadius);
    return m_min + normalized * (m_max - m_min);
}

void DualRangeSlider::updateHandleRects()
{
    int lowerPos = valueToPos(m_lowerValue);
    int upperPos = valueToPos(m_upperValue);

    m_lowerHandle = QRect(lowerPos - m_handleRadius, 10,
                          2 * m_handleRadius, 2 * m_handleRadius);
    m_upperHandle = QRect(upperPos - m_handleRadius, 10,
                          2 * m_handleRadius, 2 * m_handleRadius);
}

void DualRangeSlider::paintEvent(QPaintEvent* event)
{
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);

    // 绘制背景轨道
    painter.setPen(Qt::NoPen);
    painter.setBrush(QColor(200, 200, 200));
    painter.drawRoundedRect(m_handleRadius, 15,
                            m_sliderWidth - 2 * m_handleRadius, 6, 3, 3);

    // 绘制选中区域
    int lowerPos = valueToPos(m_lowerValue);
    int upperPos = valueToPos(m_upperValue);

    painter.setBrush(QColor(100, 150, 255));
    painter.drawRoundedRect(lowerPos, 15,
                            upperPos - lowerPos, 6, 3, 3);

    // 绘制滑块手柄
    painter.setBrush(QColor(70, 130, 220));
    painter.drawEllipse(m_lowerHandle);
    painter.drawEllipse(m_upperHandle);

    // 绘制手柄上的小点
    painter.setBrush(Qt::white);
    painter.drawEllipse(m_lowerHandle.center(), 3, 3);
    painter.drawEllipse(m_upperHandle.center(), 3, 3);

    // 绘制数值标签
    painter.setPen(Qt::black);
    QFont font = painter.font();
    font.setPointSize(8);
    painter.setFont(font);

    // 以手柄为中心、贴到控件两端时向内收：原先固定写 center.x() - 10，
    // 左边那个手柄停在最左时，「-1.0」的负号会被裁到控件外面，读数看起来成了「1.0」
    // —— 一个显示**相反数**的读数比不显示更糟（工单 08 顺手修）。
    const QFontMetrics fm(font);
    const int textY = height() - 5;
    const auto drawValue = [&](int handleCenterX, double value) {
        // 带上单位（" m"）：文本框里只有「-1.0」「2.0」时，读不出这个数是米、是度还是格。
        const QString text = QString::number(value, 'f', 1) + m_suffix;
        const int textW = fm.horizontalAdvance(text);
        int x = handleCenterX - textW / 2;
        x = std::max(0, std::min(x, width() - textW));
        painter.drawText(x, textY, text);
    };
    drawValue(m_lowerHandle.center().x(), m_lowerValue);
    drawValue(m_upperHandle.center().x(), m_upperValue);
}

void DualRangeSlider::mousePressEvent(QMouseEvent* event)
{
    QPoint pos = event->pos();

    if (m_lowerHandle.contains(pos)) {
        m_draggingLower = true;
        m_draggingUpper = false;
    } else if (m_upperHandle.contains(pos)) {
        m_draggingUpper = true;
        m_draggingLower = false;
    } else {
        // 点击轨道中间，移动最接近的手柄
        int clickPos = pos.x();
        int lowerDist = std::abs(clickPos - m_lowerHandle.center().x());
        int upperDist = std::abs(clickPos - m_upperHandle.center().x());

        if (lowerDist < upperDist) {
            m_draggingLower = true;
        } else {
            m_draggingUpper = true;
        }
    }
}

void DualRangeSlider::mouseMoveEvent(QMouseEvent* event)
{
    if (!m_draggingLower && !m_draggingUpper) {
        return;
    }

    // 值到哪儿为止由 RangeLimits 决定（下限不得越过上限、越界被夹住、留 0.1 的间距），
    // 这里只负责把鼠标 x 换成一个数值、把结果重画出来 —— 工单 08 把这段约束搬出去，
    // 是为了它能进控制台测试；手感与原来逐字一致（原来就是三行 min/max）。
    const double requested = posToValue(event->pos().x());
    const RangeLimits::Interval current{m_lowerValue, m_upperValue};
    const RangeLimits::Interval next =
        m_draggingLower ? RangeLimits::dragLower(requested, current, m_min, m_max)
                        : RangeLimits::dragUpper(requested, current, m_min, m_max);

    m_lowerValue = next.lower;
    m_upperValue = next.upper;

    updateHandleRects();
    update();

    emit rangeChanged(m_lowerValue, m_upperValue);
}

void DualRangeSlider::mouseReleaseEvent(QMouseEvent* event)
{
    m_draggingLower = false;
    m_draggingUpper = false;
}

void DualRangeSlider::resizeEvent(QResizeEvent* event)
{
    m_sliderWidth = width() - 20;
    updateHandleRects();
}
