#ifndef ICONTINT_H
#define ICONTINT_H

#include <QColor>
#include <QPixmap>
#include <QString>
#include <QStringList>

#include "Theme.h"

// 图标源与运行时着色（工单 02）。
//
// QIcon 直接吃 SVG 有个硬伤：没法按 hover / disabled 换色，而这三态在界面里是常态。
// 所以自己渲染：读 SVG 文本 → 把 currentColor 换成目标色 → QSvgRenderer 渲染成 QPixmap
// → 按 devicePixelRatio 出图并 setDevicePixelRatio。
//
// 对外只暴露「名称 + 颜色（或三态）+ 逻辑像素 + dpr → QPixmap」。
// 三态色值由 Theme 给出（text2 / accent / text3），调用方不写色值；需要例外颜色
// （比如悬浮条压在深色画布上、关闭键悬停变红）时用显式颜色重载，颜色同样从 Theme 取。
//
// 用法：
//   label->setPixmap(IconTint::pixmap("rotate-ccw", IconTint::Hover, 16));
//   btn->setIcon(QIcon(IconTint::pixmap("plug", IconTint::Normal, 14, dpr)));
//   // 例外色：Theme::palette().err
//   close->setPixmap(IconTint::pixmap("x", Theme::palette().err, 10, dpr));
//
// 注意：pixmap() 会建 QPixmap，必须在 QGuiApplication 起来之后调用。
class IconTint
{
public:
    enum State {
        Normal,     // 常规
        Hover,      // 悬停 / 选中
        Disabled    // 禁用
    };

    // 一个上下文的三态色值。来源固定是 Theme 的调色板，不在这里写字面量。
    struct Palette
    {
        QColor normal;
        QColor hover;
        QColor disabled;
    };

    // normal = text2（次要文字色），hover = accent，disabled = text3（禁用文字色）。
    static Palette palette(Theme::Variant v = Theme::Variant::Light);
    static QColor stateColor(State s, Theme::Variant v = Theme::Variant::Light);

    // 名称 + 显式颜色。logicalPx 是逻辑像素（DPR 之前），dpr <= 0 时取主屏 dpr。
    static QPixmap pixmap(const QString &name, const QColor &color, int logicalPx,
                          qreal dpr = 0.0);
    // 名称 + 三态，色值从 palette() 取。
    static QPixmap pixmap(const QString &name, State state, int logicalPx, qreal dpr = 0.0,
                          Theme::Variant v = Theme::Variant::Light);

    // 纯文本替换：currentColor → 目标色。不含 currentColor 的文本原样返回，不碰结构。
    static QString tintedSvg(const QString &svg, const QColor &color);

    // 缓存键：名称 | 颜色 | 逻辑像素 | dpr。同名不同色、同名不同 dpr 必须是不同的键。
    static QString cacheKey(const QString &name, const QColor &color, int logicalPx, qreal dpr);
    static QString cacheKey(const QString &name, State state, int logicalPx, qreal dpr,
                            Theme::Variant v = Theme::Variant::Light);

    static void clearCache();
    static int cacheCount();    // 已缓存的图数量（测试缝用来确认复用没有新增渲染）

    static QString svgPath(const QString &name);    // ":/icons/<name>.svg"
    static QString svgSource(const QString &name);  // 读一次就记住，含空串（缺失）
    static bool exists(const QString &name);
    static QStringList availableIcons();            // 已进 qrc 的图标名
};

#endif // ICONTINT_H
