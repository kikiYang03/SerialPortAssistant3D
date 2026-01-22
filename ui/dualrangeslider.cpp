#include "dualrangeslider.h"
#include <QPainter>
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
    m_lowerValue = std::max(m_min, std::min(lower, m_max));
    m_upperValue = std::max(m_min, std::min(upper, m_max));

    // 确保 lower <= upper
    if (m_lowerValue > m_upperValue) {
        std::swap(m_lowerValue, m_upperValue);
    }

    updateHandleRects();
    update();
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

    QString lowerText = QString::number(m_lowerValue, 'f', 1);
    QString upperText = QString::number(m_upperValue, 'f', 1);

    painter.drawText(m_lowerHandle.center().x() - 10, 35, lowerText);
    painter.drawText(m_upperHandle.center().x() - 10, 35, upperText);
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

    double newValue = posToValue(event->pos().x());

    if (m_draggingLower) {
        // 限制不能超过upper
        m_lowerValue = std::min(newValue, m_upperValue - 0.1);
    } else if (m_draggingUpper) {
        // 限制不能低于lower
        m_upperValue = std::max(newValue, m_lowerValue + 0.1);
    }

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
