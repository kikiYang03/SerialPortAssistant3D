#include "config.h"
#include <QCoreApplication>
#include <QDir>
#include <QFile>

Config &Config::instance()
{
    static Config inst;
    return inst;
}

Config::Config(QObject *parent)
    : QObject(parent)
    , m_settings(QDir::current().filePath("config.ini"), QSettings::IniFormat)
{
    if (!QFile::exists(m_settings.fileName()))
        qWarning() << "config.ini not found in" << m_settings.fileName();
}

QVariant Config::value(const QString &key, const QVariant &defaultValue) const
{
    return m_settings.value(key, defaultValue);
}
