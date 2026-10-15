#ifndef WAITDIALOG_H
#define WAITDIALOG_H

#include <QDialog>

class QLabel;
class QProgressBar;
class QPushButton;
class QCloseEvent;
class QKeyEvent;

// 模态等待框——长耗时操作「等待—确认/报错」闭环里的那个「等待」。
//
// 默认形态（工单 04 保存地图、工单 06 保存参数）：不确定进度指示、没有取消按钮、
// Esc 与标题栏关闭按钮都挡住。之所以不给「取消」：模块侧已经在落盘了，取消没有真实语义。
// 工单 10（下载地图）用 setProgress() + setCancelEnabled(true) 在它基础上扩出
// 带进度、可取消的变体——同一个组件，不写第二份。
//
// 组件只管「等」，不管「多久算超时」：超时是各条链自己的事（存图按模块类型取
// 3D 30 秒 / 2D 10 秒，参数保存 10 秒，地图下传分块间与整包两级）。
// 结果也不由组件判断：调用方拿到回包或超时后就 closeWait() 关掉它，再自己弹结果提示。
class WaitDialog : public QDialog
{
    Q_OBJECT
public:
    WaitDialog(const QString &title, const QString &message, QWidget *parent = nullptr);

    void setMessage(const QString &message);

    // 进度：默认不确定（忙碌）指示。total <= 0 或 done < 0 时回到不确定形态。
    void setProgress(int done, int total);

    // 默认 false。true 时取消按钮才出现，Esc / 关闭按钮才有「取消」的含义。
    void setCancelEnabled(bool enabled);

signals:
    void cancelled();

protected:
    void reject() override;                        // Esc
    void keyPressEvent(QKeyEvent *event) override; // Esc（显式挡住：验收项盯的就是这条）
    void closeEvent(QCloseEvent *event) override;  // 标题栏的关闭按钮

private:
    QLabel *labMessage_ = nullptr;
    QProgressBar *barProgress_ = nullptr;
    QPushButton *btnCancel_ = nullptr;
    bool cancelEnabled_ = false;
};

#endif // WAITDIALOG_H
