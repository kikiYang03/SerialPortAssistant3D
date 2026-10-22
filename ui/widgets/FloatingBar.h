#ifndef FLOATINGBAR_H
#define FLOATINGBAR_H

#include <QRect>
#include <QSize>
#include <QString>
#include <QWidget>

// 视口底部居中的胶囊操作条（工单 10）。
//
// 存在理由：转个视角不该跑到面板最下面找按钮。它取代了原来右侧面板里那组
// （早已被注释掉的）「视角旋转」竖条 —— 视口里不再有第二排工具栏。
//
// 三条硬约束（这一处是本次风险最高的地方，违反了的症状是「画布突然拖不动」）：
//   1. **按内容固定尺寸**：构造时就把尺寸钉成布局的 sizeHint，绝不铺满视口。
//      铺满 + 内部布局的全屏容器等于把整个视口接管了。
//   2. **不拦鼠标**：条子自己的留白（4px 内边距、分隔线）不消费鼠标事件 ——
//      基类 QWidget::mousePressEvent 默认把事件 ignore 掉，于是它冒泡到父对象
//      GLWidget，画布照常旋转。按钮本身只响应单击（QAbstractButton 的默认语义：
//      按下后拖出去就不算点击），所以「在按钮上按下再拖」不会被当成旋转。
//   3. **不碰渲染**：六个按钮只发信号，由 Ros3DPage 接到 GLWidget 既有的公开槽上，
//      本类连 glwidget.h 都不 include（顺带让它能在控制台测试里单独链接）。
//
// 外观与尺寸对齐 docs/prototype-v2.html 的 .fbar / .fbtn：30×30 按钮、16px 图标、
// 圆角 10、按钮间距 2、分组之间一条 1px 竖线。
class FloatingBar : public QWidget
{
    Q_OBJECT

public:
    // 六个按钮的语义。用枚举而不是六个独立信号：调用方一个 switch 就能把
    // 「按钮 → GLWidget 的哪个公开槽」写在一处，读起来就是那张映射表。
    enum Action {
        ResetView,    // 复位视角   → GLWidget::setCamera()
        YawLeft,      // 左转 15°   → GLWidget::addYaw(-15)
        YawRight,     // 右转 15°   → GLWidget::addYaw(+15)
        PitchUp,      // 上仰 15°   → GLWidget::addPitch(-15)
        PitchDown,    // 下俯 15°   → GLWidget::addPitch(+15)
        ClearCanvas   // 清理画布   → GLWidget::clearMap()
    };
    Q_ENUM(Action)

    // 一次转动的角度。15° = 「转得动又不会一下飞掉」，可以连点几下微调
    // （spec 用户故事 29）。步进只有这一处定义，提示文案里的「15°」也由它拼。
    static constexpr int kStepDegrees = 15;

    // 与视口下沿的距离（spec 实现决策 18）
    static constexpr int kBottomMargin = 24;

    explicit FloatingBar(QWidget *parent = nullptr);

    // 纯函数：给定视口尺寸与本控件尺寸，返回本控件在视口坐标系里的矩形。
    //
    //   * 水平居中、下边距 kBottomMargin；
    //   * 视口比控件还窄时 x 夹到 0（不能给负坐标，那会被 move() 当成相对位移）；
    //   * 居中用**整数除法**：同一组尺寸永远得到同一个结果。差值为奇数时稳定地偏
    //     0.5px 一侧，不会因为四舍五入在两个值之间来回抖（"抖动"就是这么产生的）。
    //
    // 单纯为了能在控制台测试里跑，所以是静态函数、不读任何成员。
    static QRect placeRect(const QSize &viewport, const QSize &bar);

    QSize sizeHint() const override;
    QSize minimumSizeHint() const override;

signals:
    void actionTriggered(FloatingBar::Action action);

protected:
    void paintEvent(QPaintEvent *e) override;

private:
    void addAction(Action action, const QString &iconName, const QString &tip, bool danger = false);
    void addDivider();
    // 把尺寸钉成「布局要多少就是多少」。构造末尾调一次，之后不再变。
    void applyContentSize();

    QSize contentSize_;
};

#endif // FLOATINGBAR_H
