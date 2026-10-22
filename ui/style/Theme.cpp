#include "Theme.h"

#include <QApplication>
#include <QDebug>
#include <QFile>
#include <QStyle>
#include <QVariant>
#include <QWidget>
#include <QtGlobal>

// 样式地基（工单 01）。这里只有数据与两个纯函数，不碰任何窗口，
// 所以 tests/test_theme.cpp 能在没有 QApplication 的进程里把它们跑一遍。
namespace Theme {

namespace {

QString hex(const QColor &c)
{
    return c.name(QColor::HexRgb); // 统一成 #rrggbb 小写，QSS 认这个形式
}

QString px(int v)
{
    return QString::number(v) + QStringLiteral("px");
}

QString pt(int v)
{
    return QString::number(v) + QStringLiteral("pt");
}

// 占位符名字里允许的字符：小写字母、数字、连字符。
// 只认这一小撮字符，替换时才不会把 "a@b.com" 之类的东西卷进来。
bool isTokenChar(QChar c)
{
    const ushort u = c.unicode();
    return (u >= 'a' && u <= 'z') || (u >= '0' && u <= '9') || u == '-';
}

/* ---------------------------------------------------------------- 状态色 */

// 动态属性名。QSS 侧按这个名字选（`QLabel[state="err"]`）。
// 改名等于界面全变回正文色，所以钉成常量、两边只此一处。
const char kStateProperty[] = "state";

// 测试缝（见 Theme.h）：默认空指针，生产路径上只是一次判空。
PolishObserver g_polishObserver = nullptr;

// 状态名归一化：认识的四个原样留着，其余（含空串）一律当成「没有状态」。
QString normalizeState(const QString &state)
{
    const QString s = state.trimmed().toLower();
    if (s == QLatin1String("ok") || s == QLatin1String("warn") ||
        s == QLatin1String("err") || s == QLatin1String("info"))
        return s;
    return QString();
}

} // namespace

const Palette &palette(Variant v)
{
    // 浅色：取自 docs/prototype-v2.html 的 :root，是本轮的视觉基准。
    static const Palette light = {
        QColor(QStringLiteral("#eff2f6")), // page
        QColor(QStringLiteral("#ffffff")), // bg
        QColor(QStringLiteral("#fbfcfd")), // title
        QColor(QStringLiteral("#f7f9fc")), // cmd
        QColor(QStringLiteral("#ffffff")), // panel
        QColor(QStringLiteral("#e4e9f0")), // line
        QColor(QStringLiteral("#d3dae4")), // lineStrong
        QColor(QStringLiteral("#1c2532")), // text1
        QColor(QStringLiteral("#6b7688")), // text2
        QColor(QStringLiteral("#9aa4b4")), // text3
        QColor(QStringLiteral("#ffffff")), // onAccent
        QColor(QStringLiteral("#2563eb")), // accent
        QColor(QStringLiteral("#eaf1fe")), // accentSoft
        QColor(QStringLiteral("#06a6c4")), // cyan
        QColor(QStringLiteral("#12b76a")), // ok
        QColor(QStringLiteral("#b26a00")), // warn
        QColor(QStringLiteral("#c62828")), // err
        QColor(QStringLiteral("#f1f5fb")), // hover
        QColor(QStringLiteral("#e8eef8")), // pressed
        QColor(QStringLiteral("#f4f6f9")), // disabledBg
        QColor(QStringLiteral("#0f1620")), // canvas
        QColor(QStringLiteral("#60a5fa")), // frame（浅蓝：贴在浅灰页面底上够清楚，又不抢标题栏的活）
    };

    // 深色只为「将来换肤只需换一张表」而存在，本期没有任何调用方。
    // 取值按浅色的层次关系反推，不追求一次到位——真做深色时还要单独校一遍对比度。
    static const Palette dark = {
        QColor(QStringLiteral("#0b1018")), // page
        QColor(QStringLiteral("#141b26")), // bg
        QColor(QStringLiteral("#111823")), // title
        QColor(QStringLiteral("#101722")), // cmd
        QColor(QStringLiteral("#141b26")), // panel
        QColor(QStringLiteral("#24303f")), // line
        QColor(QStringLiteral("#2f3d4e")), // lineStrong
        QColor(QStringLiteral("#e6ecf5")), // text1
        QColor(QStringLiteral("#a4b0c0")), // text2
        QColor(QStringLiteral("#6f7c8e")), // text3
        QColor(QStringLiteral("#ffffff")), // onAccent
        QColor(QStringLiteral("#3b82f6")), // accent
        QColor(QStringLiteral("#16233a")), // accentSoft
        QColor(QStringLiteral("#22d3ee")), // cyan
        QColor(QStringLiteral("#22c55e")), // ok
        QColor(QStringLiteral("#f59e0b")), // warn
        QColor(QStringLiteral("#ef4444")), // err
        QColor(QStringLiteral("#1c2734")), // hover
        QColor(QStringLiteral("#1a2431")), // pressed
        QColor(QStringLiteral("#121a24")), // disabledBg
        QColor(QStringLiteral("#0f1620")), // canvas
        QColor(QStringLiteral("#3b82f6")), // frame（深色底上浅蓝要提亮一档才看得见）
    };

    return v == Variant::Dark ? dark : light;
}

QFont font(Variant v)
{
    QFont f;
    // 与原型一致：优先 Segoe UI（拉丁字母清爽），中文落到微软雅黑。
    // 家族列表要在 qApp 建立之后才解析得准，所以这里是「声明」不是「探测」。
    f.setFamilies({QStringLiteral("Segoe UI"), QStringLiteral("Microsoft YaHei")});
    f.setFamily(QStringLiteral("Microsoft YaHei"));
    f.setPointSize(Metric::FontPtBase);
    f.setStyleHint(QFont::SansSerif);
    Q_UNUSED(v);
    return f;
}

QPalette qpalette(Variant v)
{
    const Palette &p = palette(v);

    QPalette qp;
    qp.setColor(QPalette::Window, p.bg);
    qp.setColor(QPalette::WindowText, p.text1);
    qp.setColor(QPalette::Base, p.bg);
    qp.setColor(QPalette::AlternateBase, p.cmd);
    qp.setColor(QPalette::Text, p.text1);
    qp.setColor(QPalette::Button, p.bg);
    qp.setColor(QPalette::ButtonText, p.text1);
    qp.setColor(QPalette::BrightText, p.onAccent);
    qp.setColor(QPalette::Highlight, p.accent);
    qp.setColor(QPalette::HighlightedText, p.onAccent);
    qp.setColor(QPalette::Mid, p.line);
    qp.setColor(QPalette::ToolTipBase, p.bg);
    qp.setColor(QPalette::ToolTipText, p.text1);

    // 禁用态：文字一律退到 text3。少了这一组，灰掉的按钮仍会是「能点的样子」。
    qp.setColor(QPalette::Disabled, QPalette::WindowText, p.text3);
    qp.setColor(QPalette::Disabled, QPalette::Text, p.text3);
    qp.setColor(QPalette::Disabled, QPalette::ButtonText, p.text3);
    qp.setColor(QPalette::Disabled, QPalette::Highlight, p.disabledBg);
    qp.setColor(QPalette::Disabled, QPalette::HighlightedText, p.text3);
    return qp;
}

QHash<QString, QString> tokens(Variant v)
{
    const Palette &p = palette(v);

    QHash<QString, QString> t;
    t.insert(QStringLiteral("page"), hex(p.page));
    t.insert(QStringLiteral("bg"), hex(p.bg));
    t.insert(QStringLiteral("title"), hex(p.title));
    t.insert(QStringLiteral("cmd"), hex(p.cmd));
    t.insert(QStringLiteral("panel"), hex(p.panel));
    t.insert(QStringLiteral("line"), hex(p.line));
    t.insert(QStringLiteral("line-strong"), hex(p.lineStrong));
    t.insert(QStringLiteral("text-1"), hex(p.text1));
    t.insert(QStringLiteral("text-2"), hex(p.text2));
    t.insert(QStringLiteral("text-3"), hex(p.text3));
    t.insert(QStringLiteral("on-accent"), hex(p.onAccent));
    t.insert(QStringLiteral("accent"), hex(p.accent));
    t.insert(QStringLiteral("accent-soft"), hex(p.accentSoft));
    t.insert(QStringLiteral("cyan"), hex(p.cyan));
    t.insert(QStringLiteral("ok"), hex(p.ok));
    t.insert(QStringLiteral("warn"), hex(p.warn));
    t.insert(QStringLiteral("err"), hex(p.err));
    t.insert(QStringLiteral("hover"), hex(p.hover));
    t.insert(QStringLiteral("pressed"), hex(p.pressed));
    t.insert(QStringLiteral("disabled-bg"), hex(p.disabledBg));
    t.insert(QStringLiteral("frame"), hex(p.frame));

    // 尺度：QSS 里的裸数字全部从这里来，改一处全应用跟着动。
    t.insert(QStringLiteral("radius-sm"), px(Metric::RadiusSm));
    t.insert(QStringLiteral("radius"), px(Metric::Radius));
    t.insert(QStringLiteral("radius-lg"), px(Metric::RadiusLg));
    t.insert(QStringLiteral("control-h"), px(Metric::ControlH));
    t.insert(QStringLiteral("control-h-sm"), px(Metric::ControlHSmall));
    t.insert(QStringLiteral("gap-xs"), px(Metric::GapXs));
    t.insert(QStringLiteral("gap-sm"), px(Metric::GapSm));
    t.insert(QStringLiteral("gap"), px(Metric::Gap));
    t.insert(QStringLiteral("gap-lg"), px(Metric::GapLg));
    t.insert(QStringLiteral("gap-xl"), px(Metric::GapXl));
    t.insert(QStringLiteral("font-size"), pt(Metric::FontPtBase));
    t.insert(QStringLiteral("font-size-sm"), pt(Metric::FontPtSmall));
    t.insert(QStringLiteral("font-size-xs"), pt(Metric::FontPtTiny));
    t.insert(QStringLiteral("font-size-hint"), px(Metric::FontPxHint));

    return t;
}

QString substitute(const QString &tmpl, const QHash<QString, QString> &table)
{
    // 一次扫描，而不是对每个键做一遍 replace()：键之间是前缀关系
    // （`@line` / `@line-strong`、`@accent` / `@accent-soft`），
    // 逐个 replace 会让 `@line-strong` 先被吃成 `#e4e9f0-strong`。
    QString out;
    out.reserve(tmpl.size());

    const int n = tmpl.size();
    int i = 0;
    while (i < n) {
        const QChar c = tmpl.at(i);
        if (c != QLatin1Char('@')) {
            out.append(c);
            ++i;
            continue;
        }

        int j = i + 1;
        while (j < n && isTokenChar(tmpl.at(j)))
            ++j;

        // 孤零零一个 @（或后面跟了不认识的字符）：原样吐出去，不动它。
        if (j == i + 1) {
            out.append(c);
            ++i;
            continue;
        }

        const QString name = tmpl.mid(i + 1, j - i - 1);
        const auto it = table.constFind(name);
        if (it == table.constEnd()) {
            // 未知占位符整段保留：留个显眼的 @xxx 比留一片空白好查。
            out.append(tmpl.mid(i, j - i));
        } else {
            out.append(it.value());
        }
        i = j;
    }

    return out;
}

QString templateText()
{
    QFile f(QStringLiteral(":/style/app.qss.in"));
    if (!f.open(QIODevice::ReadOnly | QIODevice::Text))
        return QString();
    return QString::fromUtf8(f.readAll());
}

QString styleSheet(Variant v)
{
    const QString tmpl = templateText();
    if (tmpl.isEmpty())
        return QString();
    return substitute(tmpl, tokens(v));
}

void init(QApplication *app, Variant v)
{
    if (!app)
        return;

    // 「只调一次」是硬要求：QSS 每次 setStyleSheet 都会让所有控件重走 polish，
    // 反复设置等于全窗重排。这里挡一道，调用方写错也不会变成性能坑。
    static bool applied = false;
    if (applied)
        return;
    applied = true;

    app->setFont(font(v));
    app->setPalette(qpalette(v));

    const QString qss = styleSheet(v);
    if (qss.isEmpty()) {
        // 资源没进 qrc 时不能静默：静默的结果是整个界面退回原生外观，
        // 看起来像「样式没生效」而不是「资源丢了」。
        qWarning("Theme: 样式模板 :/style/app.qss.in 读不到，界面将退回 Qt 原生外观");
        return;
    }
    app->setStyleSheet(qss);
}

QColor color(const QString &token, Variant v)
{
    const QHash<QString, QString> t = tokens(v); // 复制一份：别让迭代器指到临时对象上
    const auto it = t.constFind(token);
    if (it == t.constEnd())
        return QColor();
    return QColor(it.value());
}

void setState(QWidget *widget, const QString &state)
{
    if (!widget)
        return;

    const QString next = normalizeState(state);
    // 拼错了要看得见：静默忽略的结果是「颜色停在上一状态」，比日志里多一行难查得多。
    if (next.isEmpty() && !state.trimmed().isEmpty())
        qWarning("Theme::setState: 不认识的状态 \"%s\"，按清空处理", qPrintable(state));

    const QVariant current = widget->property(kStateProperty);
    const QString currentState = current.isValid() ? current.toString() : QString();

    // 幂等：同值直接返回。这一条不是优化，是正确性——状态标签挂在每秒刷新的路径上，
    // 每帧 unpolish/polish 一次会让整个控件重排，看起来就是「界面在抖」。
    if (currentState == next)
        return;

    // 空状态写回**无效** QVariant：属性选择器 [state="err"] 于是不再命中，控件回到正文色。
    // （写空串同样不命中，但无效值更干净：property() 一看就知道这个控件没有状态。）
    widget->setProperty(kStateProperty, next.isEmpty() ? QVariant() : QVariant(next));

    // 动态属性不会自己触发重新解析样式，必须走这一对；少了 update() 则要等下一次重绘。
    if (QStyle *s = widget->style()) {
        s->unpolish(widget);
        s->polish(widget);
    }
    widget->update();

    if (g_polishObserver)
        g_polishObserver(widget, next);
}

QString stateOf(const QWidget *widget)
{
    if (!widget)
        return QString();
    const QVariant v = widget->property(kStateProperty);
    return v.isValid() ? v.toString() : QString();
}

void setPolishObserver(PolishObserver observer)
{
    g_polishObserver = observer;
}

} // namespace Theme
