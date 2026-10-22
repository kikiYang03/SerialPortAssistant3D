#ifndef DUALRANGESLIDER_H
#define DUALRANGESLIDER_H
#include <QWidget>
#include <QCheckBox>
#include <QSlider>
#include <QDoubleSpinBox>
#include <QLabel>
#include <QGroupBox>

class DualRangeSlider : public QWidget
{
    Q_OBJECT
public:
    explicit DualRangeSlider(QWidget* parent = nullptr);

    void setRange(double min, double max);
    void setValues(double lower, double upper);

    // 画在两端手柄下面的**单位后缀**（例如 " m"）。零长度 = 只画数字（旧观感）。
    // 数字本身只保留一位小数，光看「-1.0」「2.0」读不出量纲 —— 由调用方决定挂什么单位，
    // 本控件不假设它是米。
    void setValueSuffix(const QString& suffix);
    double lowerValue() const { return m_lowerValue; }
    double upperValue() const { return m_upperValue; }

signals:
    void rangeChanged(double lower, double upper);

protected:
    void paintEvent(QPaintEvent* event) override;
    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;
    void resizeEvent(QResizeEvent* event) override;

private:
    double m_min = -5.0;
    double m_max = 20.0;
    double m_lowerValue = -5.0;
    double m_upperValue = 5.0;
    QString m_suffix;   // 读数后缀（单位），见 setValueSuffix()
    int m_sliderWidth = 200;
    int m_sliderHeight = 30;
    bool m_draggingLower = false;
    bool m_draggingUpper = false;
    int m_handleRadius = 8;

    int valueToPos(double value) const;
    double posToValue(int pos) const;
    void updateHandleRects();
    QRect m_lowerHandle;
    QRect m_upperHandle;
};

#endif // DUALRANGESLIDER_H
