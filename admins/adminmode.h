#ifndef ADMINMODE_H
#define ADMINMODE_H

#include <QObject>
#include <QSet>
#include <QWidget>

class AdminMode : public QObject
{
    Q_OBJECT
public:
    static AdminMode& instance();
    bool isAdmin() const { return m_admin; }

    // 登录/退出
    void login(const QString& pwd);
    void logout();

    // 由构造函数调用，把需要受控的控件注册进来
    void install(QWidget *w, bool edit = false, bool visible = false);

    // 统一打印管理员日志
    static void appendMessage(const QString &msg);

signals:
    void adminStateChanged(bool admin);
    void logAdminMessage(const QString &msg);   // 内部信号

private:
    explicit AdminMode(QObject *parent = nullptr);
    bool eventFilter(QObject *obj, QEvent *ev) override;

    bool m_admin = false;
    QSet<QWidget*> m_editControls;   // 仅管理员可编辑
    QSet<QWidget*> m_visibleControls;// 仅管理员可见
};

#endif // ADMINMODE_H
