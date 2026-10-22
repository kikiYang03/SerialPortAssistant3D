#include "IconTint.h"

#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QGuiApplication>
#include <QHash>
#include <QPainter>
#include <QScreen>
#include <QSvgRenderer>

namespace {

// 图标源与渲染结果都放在静态容器里：进程内唯一一份，跨控件复用。
struct Store
{
    QHash<QString, QString> sources;   // 名称 → SVG 文本（缺失记空串）
    QHash<QString, QPixmap> pixmaps;   // cacheKey → 已渲染的图
};

Store &store()
{
    static Store s;
    return s;
}

// SVG 属性的颜色写法：不透明用 #rrggbb，半透明补上 alpha（Qt 的 SVG 支持 #aarrggbb）。
QString svgColor(const QColor &c)
{
    if (!c.isValid())
        return QString();
    return c.alpha() == 255 ? c.name(QColor::HexRgb) : c.name(QColor::HexArgb);
}

// dpr <= 0 时取主屏；没有 GUI（纯控制台场景）时退化成 1.0，绝不因为没屏幕而崩。
qreal effectiveDpr(qreal dpr)
{
    if (dpr > 0.0)
        return dpr;
    if (qobject_cast<QGuiApplication *>(QCoreApplication::instance())) {
        if (const QScreen *screen = QGuiApplication::primaryScreen()) {
            const qreal s = screen->devicePixelRatio();
            if (s > 0.0)
                return s;
        }
    }
    return 1.0;
}

// dpr 归一化到千分位。
//
// 必要性：dpr 是从「屏幕的物理 DPI / 96」算出来的浮点数，同一块屏在不同调用点可能
// 得到 1.2500000000000002 这种尾巴。缓存键里差这一位就是**两张不同的图**：一张按 1.25
// 出，另一张按 1.2500000000000002 出，而后者被 setDevicePixelRatio 标成 1.25 来画 ——
// 逻辑尺寸差之毫厘，缩放时就是一层若有若无的虚边（也就是「切换缩放之后图标串了」）。
// 千分位远细于任何真实缩放档（100/125/150/175/200%），归一化不会合并两个真实档位。
qreal normalizedDpr(qreal dpr)
{
    const qreal eff = effectiveDpr(dpr);
    return qRound(eff * 1000.0) / 1000.0;
}

} // namespace

/* ---------- 三态色：只从 Theme 取 ---------- */

IconTint::Palette IconTint::palette(Theme::Variant v)
{
    const Theme::Palette &p = Theme::palette(v);
    Palette out;
    out.normal = p.text2;    // #6b7688：图标常态就是次要文字的重量
    out.hover = p.accent;    // #2563eb：悬停变蓝，与原型一致
    out.disabled = p.text3;  // #9aa4b4：禁用与禁用文字同色
    return out;
}

QColor IconTint::stateColor(State s, Theme::Variant v)
{
    const Palette p = palette(v);
    switch (s) {
    case Hover:
        return p.hover;
    case Disabled:
        return p.disabled;
    case Normal:
    default:
        return p.normal;
    }
}

/* ---------- 着色 ---------- */

QString IconTint::tintedSvg(const QString &svg, const QColor &color)
{
    const QString c = svgColor(color);
    if (svg.isEmpty() || c.isEmpty())
        return svg;

    // 纯文本替换：只动 currentColor 这个字面量，不解析、不重排 XML。
    // 没有 currentColor 的图标里两次 replace 都是空操作，文本原样返回。
    QString out = svg;
    out.replace(QLatin1String("currentColor"), c);
    out.replace(QLatin1String("currentcolor"), c);  // 防御手写源里的小写写法
    return out;
}

/* ---------- 缓存键 ---------- */

QString IconTint::cacheKey(const QString &name, const QColor &color, int logicalPx, qreal dpr)
{
    // 颜色、逻辑像素、dpr 缺一不可：16@2x 与 32@1x 的物理尺寸一样但 dpr 不同，
    // 出图必须分开（否则 setDevicePixelRatio 会把其中一方画成一半大）。
    // dpr 走 normalizedDpr：同一档缩放不能因为浮点尾巴被当成两档（见上面的说明）。
    return QStringLiteral("%1|%2|%3|%4")
        .arg(name, svgColor(color))
        .arg(logicalPx)
        .arg(QString::number(normalizedDpr(dpr), 'g', 6));
}

QString IconTint::cacheKey(const QString &name, State state, int logicalPx, qreal dpr,
                           Theme::Variant v)
{
    return cacheKey(name, stateColor(state, v), logicalPx, dpr);
}

/* ---------- 源 ---------- */

QString IconTint::svgPath(const QString &name)
{
    if (name.isEmpty())
        return QString();
    return QStringLiteral(":/icons/%1.svg").arg(name);
}

QString IconTint::svgSource(const QString &name)
{
    if (name.isEmpty())
        return QString();

    Store &s = store();
    const auto it = s.sources.constFind(name);
    if (it != s.sources.constEnd())
        return it.value();

    QString text;
    QFile f(svgPath(name));
    if (f.open(QIODevice::ReadOnly))
        text = QString::fromUtf8(f.readAll());

    s.sources.insert(name, text);    // 缺失也记下来，避免反复开不存在的文件
    return text;
}

bool IconTint::exists(const QString &name)
{
    return !svgSource(name).isEmpty();
}

QStringList IconTint::availableIcons()
{
    QStringList out;
    const QStringList files =
        QDir(QStringLiteral(":/icons")).entryList(QStringList() << QStringLiteral("*.svg"),
                                                 QDir::Files, QDir::Name);
    out.reserve(files.size());
    for (const QString &f : files)
        out.append(QFileInfo(f).completeBaseName());
    return out;
}

/* ---------- 渲染 ---------- */

QPixmap IconTint::pixmap(const QString &name, const QColor &color, int logicalPx, qreal dpr)
{
    if (name.isEmpty() || logicalPx <= 0 || !color.isValid())
        return QPixmap();

    const qreal eff = normalizedDpr(dpr);
    const QString key = cacheKey(name, color, logicalPx, eff);

    Store &s = store();
    const auto hit = s.pixmaps.constFind(key);
    if (hit != s.pixmaps.constEnd() && !hit.value().isNull())
        return hit.value();    // 同名同色同尺寸同 dpr：直接复用，不重渲染

    QPixmap out;
    const QString svg = svgSource(name);
    if (!svg.isEmpty()) {
        QSvgRenderer renderer(tintedSvg(svg, color).toUtf8());
        if (renderer.isValid()) {
            // 物理像素按 dpr 放大，再告诉 Qt 这张图的 dpr，逻辑尺寸才等于 logicalPx。
            const int side = qMax(1, qRound(logicalPx * eff));
            out = QPixmap(side, side);
            out.fill(Qt::transparent);
            QPainter p(&out);
            p.setRenderHint(QPainter::Antialiasing, true);
            p.setRenderHint(QPainter::SmoothPixmapTransform, true);
            renderer.render(&p, QRectF(0, 0, side, side));
            p.end();
            out.setDevicePixelRatio(eff);
        }
    }

    if (!out.isNull())
        s.pixmaps.insert(key, out);    // 失败（缺图/坏 SVG）不缓存，下次还能重试
    return out;
}

QPixmap IconTint::pixmap(const QString &name, State state, int logicalPx, qreal dpr,
                         Theme::Variant v)
{
    return pixmap(name, stateColor(state, v), logicalPx, dpr);
}

/* ---------- 缓存 ---------- */

void IconTint::clearCache()
{
    store().pixmaps.clear();
}

int IconTint::cacheCount()
{
    return store().pixmaps.size();
}
