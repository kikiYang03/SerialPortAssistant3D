#include "mainwindow.h"

#include <QApplication>
#include <QTextCodec>
#include <QFont>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    // 设置全局样式表来改变字体大小
    // 设置全局字体
    QFont globalFont;
    globalFont.setFamily("Microsoft YaHei"); // 微软雅黑
    // globalFont.setFamily("SimSun"); // 微软雅黑
    globalFont.setPointSize(12);              // 字体大小
    globalFont.setStyleHint(QFont::SansSerif); // 字体样式提示

    a.setFont(globalFont);

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
