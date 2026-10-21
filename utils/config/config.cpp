#include "config.h"
#include <QCoreApplication>
#include <QDir>
#include <QFile>

namespace {

// config.ini 的候选位置，按优先级排列。
// 之所以不能只用固定一种：影子构建时 exe 在 debug/ 子目录、ini 在构建根目录；
// 发布时两者通常并排放在同一个文件夹；而在 Qt Creator 里工作时「当前工作目录」
// 未必等于其中任何一个，所以老的 QDir::current() 方案并不可靠。
QStringList candidatePaths()
{
    // 静态初始化阶段（QApplication 还没构造）applicationDirPath() 返回空串，
    // 这时退回当前工作目录，避免拼出 "/config.ini" 这种绝对路径。
    const QString exeDir = QCoreApplication::instance()
                               ? QCoreApplication::applicationDirPath()
                               : QDir::currentPath();

    QStringList list;
    list << exeDir + QStringLiteral("/config.ini");              // 发布: exe 旁边
    list << exeDir + QStringLiteral("/../config.ini");           // 构建: exe 在 debug/，ini 在上一级
    list << exeDir + QStringLiteral("/../config/config.ini");    // 直接跑源码树里的 exe
    list << QDir::current().filePath(QStringLiteral("config.ini")); // 兜底: 当前工作目录
    return list;
}

QString resolveConfigPath()
{
    const QStringList candidates = candidatePaths();
    for (const QString &raw : candidates) {
        const QString path = QDir::cleanPath(raw);
        if (QFile::exists(path))
            return path;
    }
    return QDir::cleanPath(candidates.first());
}

} // namespace

Config &Config::instance()
{
    static Config inst;
    return inst;
}

Config::Config(QObject *parent)
    : QObject(parent)
    , m_settings(resolveConfigPath(), QSettings::IniFormat)
{
    if (!QFile::exists(m_settings.fileName())) {
        qWarning() << "config.ini not found, using built-in defaults. tried:";
        const QStringList candidates = candidatePaths();
        for (const QString &raw : candidates)
            qWarning() << "   " << QDir::cleanPath(raw);
    } else {
        qInfo() << "config.ini loaded from" << m_settings.fileName();
    }
}

QVariant Config::value(const QString &key, const QVariant &defaultValue) const
{
    return m_settings.value(key, defaultValue);
}
