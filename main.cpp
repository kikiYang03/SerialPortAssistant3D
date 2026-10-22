#include "mainwindow.h"

#include <QApplication>
#include <QTextCodec>

#include "Theme.h"
#include "utils/config/config.h"

int main(int argc, char *argv[])
{
    // 高 DPI 的两条，必须赶在 QApplication 构造之前设，构造之后再设无效。
    //
    // ① 缩放交给 Qt：125% / 150% 下按逻辑像素排版、按 dpr 出图。不开这一位的话，
    //    Windows 会把 96dpi 的一整块位图拉伸后再贴上来 —— 那种糊是「全窗口都糊」，
    //    靠图标按 dpr 出图是救不回来的（图标再清也会被一起拉伸）。
    //    100% 屏幕上这一位是空操作，所以本机观感不变。
    //    Qt6 默认就开着，那边再设会拿到警告，所以只在 Qt5 里写。
    // ② 位图按设备像素出图（QPixmap::setDevicePixelRatio 要它才生效）。
#if QT_VERSION < QT_VERSION_CHECK(6, 0, 0)
    QCoreApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
#endif
    QCoreApplication::setAttribute(Qt::AA_UseHighDpiPixmaps);

    QApplication a(argc, argv);

    // 全局外观只在这里落地一次：字体 + QPalette + QSS 模板替换。
    // 全应用的色值、字号、圆角、间距都从 Theme 来，别处不要再写死。
    Theme::init(&a);

    // 版本号可配置：config.ini 的 `[App] version`。只在这里读一次并写进 QApplication，
    // 标题栏（以及将来任何要显示版本的界面）统一取 applicationVersion()，不各自去翻 ini。
    // 读不到（没放 config.ini / 该项留空）就保持 Qt 的默认空串，由 TitleBar 用内置兜底。
    const QString appVersion =
        Config::instance().value(QStringLiteral("App/version")).toString().trimmed();
    if (!appVersion.isEmpty())
        QCoreApplication::setApplicationVersion(appVersion);

    qRegisterMetaType<TFMsg>("TFMsg");
    qRegisterMetaType<CloudMsg>("CloudMsg");
    qRegisterMetaType<MapCloudMsg>("MapCloudMsg");



//设置中文编码
#if (QT_VERSION <= QT_VERSION_CHECK(5,0,0))
#if _MSC_VER
    QTextCodec *codec = QTextCodec::codecForName("gbk");
#else
    QTextCodec *codec = QTextCodec::codecForName("utf-8");
#endif
    QTextCodec::setCodecForLocale(codec);
    QTextCodec::setCodecForCStrings(codec);
    QTextCodec::setCodecForTr(codec);
#else
    QTextCodec *codec = QTextCodec::codecForName("utf-8");
    QTextCodec::setCodecForLocale(codec);
#endif


    MainWindow w;
    w.show();
    return a.exec();
}
