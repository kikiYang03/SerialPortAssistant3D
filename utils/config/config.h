#ifndef CONFIG_H
#define CONFIG_H

#include <QObject>
#include <QSettings>
#include <QString>
#include <QDebug>

class Config : public QObject
{
    Q_OBJECT
public:
    static Config &instance();          // 全局访问点

    /* 通用读取：支持分组，如 "Network/Port" */
    QVariant value(const QString &key,
                   const QVariant &defaultValue = {}) const;

private:
    explicit Config(QObject *parent = nullptr);
    Q_DISABLE_COPY(Config)

    QSettings m_settings;
};

#endif // CONFIG_H
