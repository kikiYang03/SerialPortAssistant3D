#ifndef COLLAPSIBLESECTION_H
#define COLLAPSIBLESECTION_H

#include <QString>
#include <QWidget>

class QPropertyAnimation;
class QVBoxLayout;

// 右面板的折叠分组（工单 08）：一行标题 + 一块内容区，折叠时**只隐藏内容区**，
// 标题行留着 —— 「收起」不是「消失」，标题就是收回来那个入口。
//
// 为什么自绘而不用 QGroupBox：QGroupBox 的标题是画在边框上的图例，既不能点、
// 也不能整行悬停；7 个摞在一起时只有「框线 + 图例」这一种层次。
// 这里把标题做成 32px 的整行，配一枚 chevron（朝下 = 展开、朝右 = 收起），
// 点行内任意位置就能开合，与原型 `.sech` 一致。
//
// 用法：
//     auto *sec = new CollapsibleSection(tr("点云显示"), side);
//     sec->bodyLayout()->addWidget(ckRealtime);   // 内容往 bodyLayout() 里塞
//
// 内容的父对象**是** section 内部那块 content()，不是 section 本身 —— 调用方
// 负责 new 出来的控件交给布局之后就不用管归属了。
//
// 折叠状态只活在这个对象里（不落盘）。本页在 QStackedWidget 里建一次就一直活着，
// 所以「切页面来回不重置」是结构上成立的，不需要另存一份状态。
//
// 颜色、字号、尺寸全部来自 Theme：标题底色 hover/none、chevron 的 text-3/text-2、
// 底部 1px 分隔线的 line。这里不写任何色值字面量。
//
// 箭头只在**开合时**转（工单 12）：chevron-down 原地绕中心转到 -90° 就是 chevron-right
// （两张 SVG 的路径本来就同形，见 .cpp 里的说明），时长取 `Theme::Motion::SlideMs`。
// 过渡只改绘制用的角度，不碰几何、不加事件过滤 —— 标题行的命中区域在动画期间不变。
class CollapsibleSection : public QWidget
{
    Q_OBJECT
    // 折叠箭头的角度（0 = 展开朝下，-90 = 收起朝右）。给动画用，业务代码不要碰。
    Q_PROPERTY(qreal chevronAngle READ chevronAngle WRITE setChevronAngle)

public:
    explicit CollapsibleSection(const QString &title, QWidget *parent = nullptr);

    // 内容区的容器（控件的父对象用它）；布局由 bodyLayout() 直接拿。
    QWidget *content() const { return body_; }
    QVBoxLayout *bodyLayout() const { return bodyLayout_; }

    QString title() const { return title_; }
    void setTitle(const QString &title);

    bool isCollapsed() const { return collapsed_; }

    // 标题行在控件内的矩形。命中测试与绘制共用同一份换算 —— 两处各算一次，
    // 迟早会出现「点得到的地方和看起来不一样」。
    QRect headerRect() const;

public slots:
    // 折叠/展开。重复设置同值不做事，也不发信号。
    void setCollapsed(bool collapsed);
    void toggle();

signals:
    // 只有用户或调用方真的改变了状态才发（重复设置同值不发）。
    void collapsedChanged(bool collapsed);

protected:
    void paintEvent(QPaintEvent *e) override;
    void mousePressEvent(QMouseEvent *e) override;
    void mouseMoveEvent(QMouseEvent *e) override;
    void leaveEvent(QEvent *e) override;

private:
    void setHeaderHover(bool hover);

    // 箭头转到目标角度。不可见时直接跳到位（没必要为一个看不见的控件跑动画）。
    void animateChevronTo(qreal target);
    qreal chevronAngle() const { return chevronAngle_; }
    void setChevronAngle(qreal angle);

    QString title_;
    QWidget *body_ = nullptr;
    QVBoxLayout *bodyLayout_ = nullptr;
    bool collapsed_ = false;
    bool hoverHeader_ = false;

    qreal chevronAngle_ = 0.0;                 // 0 = 展开朝下，-90 = 收起朝右
    QPropertyAnimation *chevronAnim_ = nullptr;
};

#endif // COLLAPSIBLESECTION_H
