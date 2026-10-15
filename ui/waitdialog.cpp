#include "waitdialog.h"

#include <QCloseEvent>
#include <QHBoxLayout>
#include <QKeyEvent>
#include <QLabel>
#include <QProgressBar>
#include <QPushButton>
#include <QVBoxLayout>

WaitDialog::WaitDialog(const QString &title, const QString &message, QWidget *parent)
    : QDialog(parent)
{
    setWindowTitle(title);
    setMinimumWidth(340);   // 文案要换行，别让框缩成一条

    // 等待期间用户碰不了别的地方 —— 但日志窗口仍要在背后滚动可见，
    // 「失败即沉默」的三条链全靠日志给精确结论（这是应用级模态，不是全屏遮挡）。
    setModal(true);

    labMessage_ = new QLabel(message, this);
    labMessage_->setWordWrap(true);

    barProgress_ = new QProgressBar(this);
    barProgress_->setRange(0, 0);   // 0/0 = 不确定进度（忙碌）指示
    barProgress_->setTextVisible(false);

    // 默认没有取消按钮（工单 04 / 06 都没有取消语义），工单 10 才 setCancelEnabled(true)
    btnCancel_ = new QPushButton(tr("取消"), this);
    btnCancel_->hide();
    connect(btnCancel_, &QPushButton::clicked, this, &WaitDialog::reject);

    auto *btnLay = new QHBoxLayout;
    btnLay->addStretch(1);
    btnLay->addWidget(btnCancel_);

    auto *lay = new QVBoxLayout(this);
    lay->addWidget(labMessage_);
    lay->addWidget(barProgress_);
    lay->addLayout(btnLay);
}

void WaitDialog::setMessage(const QString &message)
{
    labMessage_->setText(message);
}

void WaitDialog::setProgress(int done, int total)
{
    if (total <= 0 || done < 0) {
        barProgress_->setRange(0, 0);   // 回到不确定形态
        return;
    }
    barProgress_->setRange(0, total);
    barProgress_->setValue(done);
}

void WaitDialog::setCancelEnabled(bool enabled)
{
    cancelEnabled_ = enabled;
    btnCancel_->setVisible(enabled);
}

void WaitDialog::reject()
{
    // 等待中的框没有「取消」语义：Esc 与走 reject 的关闭路径一并吃掉
    if (!cancelEnabled_) return;

    emit cancelled();
    QDialog::reject();
}

void WaitDialog::keyPressEvent(QKeyEvent *event)
{
    // Esc 不走基类那条自动 reject 的路：整条路径自己拿着，
    // 「Esc 被挡住」是验收项，不能靠基类行为兜着。
    if (event->key() == Qt::Key_Escape) {
        event->accept();
        if (cancelEnabled_) reject();   // reject() 里再判一次，语义只在一处写
        return;
    }
    QDialog::keyPressEvent(event);
}

void WaitDialog::closeEvent(QCloseEvent *event)
{
    // 标题栏的关闭按钮：同样是「取消」的意思，没开取消就当没点
    if (!cancelEnabled_) {
        event->ignore();
        return;
    }

    // 不走 QDialog::closeEvent()：它内部还会再调一次 reject()，cancelled 会发两遍
    emit cancelled();
    event->accept();
}
