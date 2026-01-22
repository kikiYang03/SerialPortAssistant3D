#include "adminmode.h"
#include "utils/config/config.h"
#include <QInputDialog>
#include <QMessageBox>
#include <QEvent>
#include <QDateTime>

static const QString ADMIN_PWD = Config::instance().value("Admin/password").toString();

AdminMode& AdminMode::instance()
{
    static AdminMode ins;
    return ins;
}
AdminMode::AdminMode(QObject *parent): QObject(parent) {}

void AdminMode::install(QWidget *w, bool edit, bool visible)
{
    if (!w) return;
    w->installEventFilter(this);
    if (edit)      m_editControls << w;
    if (visible)   m_visibleControls << w;
    // 初始化一次状态
    w->setEnabled(!edit || m_admin);
    w->setVisible(!visible || m_admin);
}

bool AdminMode::eventFilter(QObject *obj, QEvent *ev)
{
    // 禁止非管理员时期获得焦点
    if (!m_admin && m_editControls.contains(static_cast<QWidget*>(obj)))
        if (ev->type() == QEvent::FocusIn || ev->type() == QEvent::MouseButtonPress)
            return true; // 吃掉事件
    return QObject::eventFilter(obj, ev);
}

void AdminMode::login(const QString &pwd)
{
    if (pwd == ADMIN_PWD) {
        m_admin = true;
        for (auto w : m_editControls)   w->setEnabled(true);
        for (auto w : m_visibleControls) w->setVisible(true);
        emit adminStateChanged(true);
        // 登录成功提示
        QMessageBox::information(nullptr, QStringLiteral("管理员模式"),
                                 QStringLiteral("已登录管理员模式"));
    } else {
        QMessageBox::warning(nullptr, "错误", "密码错误");
    }
}

void AdminMode::logout()
{
    m_admin = false;
    for (auto w : m_editControls)   w->setEnabled(false);
    for (auto w : m_visibleControls) w->setVisible(false);
    emit adminStateChanged(false);
    // 退出成功提示
    QMessageBox::information(nullptr, QStringLiteral("管理员模式"),
                             QStringLiteral("已退出管理员模式"));
}

void AdminMode::appendMessage(const QString &msg)
{
    if (!instance().isAdmin()) return;
    emit instance().logAdminMessage(
        QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss >> [管理员] ") + msg);
}
