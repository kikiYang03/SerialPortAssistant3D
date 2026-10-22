#ifndef ICONBUTTON_H
#define ICONBUTTON_H

#include <QAbstractButton>
#include <QString>

// 图标 + 文字的按钮（工单 06）：三态自绘，图标与文字永远同色。
//
// 为什么不用 QPushButton + setIcon：QIcon 只有 Normal/Disabled/Active 三档，且没法让
// 「实底主按钮按下时图标跟着反白」—— 图标与文字一旦不同色，白底上就会浮出一枚灰度图标。
// 这里三种状态的颜色全部从 Theme 取，图标由 IconTint 现渲染。
//
// 外观只有两种（对应原型里的 .btn 与 .btn.primary）：
//   Outline —— 白底 + 描边，「读取参数」，以及**已连接**时的主按钮；
//   Primary —— 强调色实底，**未连接**时的「连接设备」。
//
// 文字不用 QAbstractButton::setText()：那个接口不通知尺寸变化，宽度就永远停在初值上。
// 用 setLabel()，内部顺带把文字写进 text()（无障碍/工具提示仍能读到）。
class IconButton : public QAbstractButton
{
    Q_OBJECT

public:
    enum Appearance {
        Outline,   // 白底描边
        Primary    // 实底主按钮
    };

    explicit IconButton(QWidget *parent = nullptr);

    // 图标名 = resource/icons/<name>.svg 的文件名（不含扩展名）。空名表示不画图标。
    void setIconName(const QString &name);
    QString iconName() const { return iconName_; }

    // 显示文字。宽度随之重算（图标 + 间距 + 文字 + 两侧内边距）。
    void setLabel(const QString &text);
    QString label() const { return label_; }

    void setAppearance(Appearance appearance);
    Appearance appearance() const { return appearance_; }

    QSize sizeHint() const override;
    QSize minimumSizeHint() const override;

protected:
    void paintEvent(QPaintEvent *e) override;
    void enterEvent(QEvent *e) override;
    void leaveEvent(QEvent *e) override;

private:
    void updateWidth();

    QString iconName_;
    QString label_;
    Appearance appearance_ = Outline;
};

#endif // ICONBUTTON_H
