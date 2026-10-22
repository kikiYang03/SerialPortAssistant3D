#ifndef SEGMENTEDCONTROL_H
#define SEGMENTEDCONTROL_H

#include <QStringList>
#include <QWidget>

class QPropertyAnimation;

// 分段控件（工单 06）：命令栏左段的三页切换，自绘 + 指示条滑动。
//
// 为什么不用 QTabBar：它自带一整套基线、边框与「标签页」语义，改到像原型反而要盖掉它
// 画的每一样东西；而且它的选中态是瞬间跳过去的。原型要的是「一个圆角托盘里一块会
// 滑过去的白块」，这只有自己画才做得到。
//
// 三层，从下往上：托盘（浅底 + 1px 描边）→ 白块（指示条）→ 文字（选中 accent / 其余 text2）。
// 指示条的位置是一个 qreal 属性（thumbPos = 0 表示第一段、1.5 表示第二段与第三段之间），
// 切换时用 QPropertyAnimation 滑过去；页面本身是瞬间换的 —— 动画只装饰指示条，
// 不拖住中央堆栈切换。
//
// 两个信号的分工是硬要求：
//   * segmentActivated(index) —— 只有**用户点击**才发，命令栏用它驱动页切换；
//   * currentChanged(index)    —— 任何变化都发（含外部 setCurrentIndex），
//     用来告诉外界「现在停在哪」，**不要**拿它去驱动页切换，否则「堆栈改命令栏、
//     命令栏又改堆栈」会绕成环。
class SegmentedControl : public QWidget
{
    Q_OBJECT
    // 指示条的浮点位置。给动画用，业务代码不要碰。
    Q_PROPERTY(qreal thumbPos READ thumbPos WRITE setThumbPos)

public:
    explicit SegmentedControl(QWidget *parent = nullptr);

    void addSegment(const QString &text);
    int count() const { return m_texts.size(); }
    int currentIndex() const { return m_current; }

    QSize sizeHint() const override;
    QSize minimumSizeHint() const override;

public slots:
    // 外部同步入口（中央堆栈改页时用）：不发 segmentActivated，因此不会反向驱动堆栈。
    void setCurrentIndex(int index);

signals:
    void segmentActivated(int index);
    void currentChanged(int index);

protected:
    void paintEvent(QPaintEvent *e) override;
    void mousePressEvent(QMouseEvent *e) override;
    void mouseMoveEvent(QMouseEvent *e) override;
    void leaveEvent(QEvent *e) override;

private:
    qreal thumbPos() const { return m_thumbPos; }
    void setThumbPos(qreal pos);

    QRectF trackRect() const;
    QRectF segmentRect(qreal pos) const;   // pos 可以是小数（滑动中间态）
    int indexAt(const QPoint &pos) const;
    void updateMetrics();

    QStringList m_texts;
    int m_current = -1;
    int m_hover = -1;
    qreal m_thumbPos = 0.0;

    int m_segmentWidth = 0;                // 每段等宽，由最长的那个标题撑出来
    QPropertyAnimation *m_anim = nullptr;
};

#endif // SEGMENTEDCONTROL_H
