#ifndef THEME_H
#define THEME_H

#include <QColor>
#include <QFont>
#include <QHash>
#include <QPalette>
#include <QString>

class QApplication;
class QWidget;

// 全应用样式的唯一入口：调色板 + 尺度阶梯 + QSS 模板替换 + QPalette 同步。
//
// 三件事只在这里做一次（`Theme::init()`），别处不要再 `setStyleSheet` 全量样式、
// 也不要再写色值字面量：
//   1. 字号 / 间距 / 圆角收敛成常量（`Metric`），代码里不再出现裸字号；
//   2. 颜色只有一份来源（`palette()`），QSS 里写 `@accent` 这类占位符，启动时替换；
//   3. QPalette 与 QSS 同步设置——只设 QSS 不设调色板，弹窗、`QComboBox` 弹层、
//      滚动条角落会露白，出现「部分亮部分暗」。
//
// 占位符是本项目自己的语法（Qt5 的 QSS 没有变量，也没有 var()）：
// 模板里出现 `@accent`，`substitute()` 把它换成 `#2563eb`。未知占位符**原样保留**，
// 不会变成空串——静默丢样式比留个显眼的 @xxx 难查得多。
namespace Theme {

// 换肤只换这张表：本期只启用 Light，Dark 的取值一并备好但没有任何调用方。
enum class Variant {
    Light,
    Dark,
};

// 颜色只有这份定义。QSS 侧的占位符名见 `tokens()`，两边一一对应。
struct Palette {
    QColor page;       // 窗口外/页底色（比白更暗一格，用来垫起面板）
    QColor bg;         // 控件与页面底色
    QColor title;      // 标题栏底
    QColor cmd;        // 命令栏/表头/状态栏底
    QColor panel;      // 右面板底
    QColor line;       // 常规描边
    QColor lineStrong; // 需要更明显一点的描边（输入框、按钮边）
    QColor text1;      // 主文字
    QColor text2;      // 次要文字
    QColor text3;      // 更弱的辅助文字，也是禁用态文字
    QColor onAccent;   // 压在强调色上的文字
    QColor accent;     // 强调色（选中、焦点、主按钮）
    QColor accentSoft; // 强调色的浅底色（悬停、选中行）
    QColor cyan;       // 渐变线的第二段，与 accent 成对出现
    QColor ok;         // 状态色：正常
    QColor warn;       // 状态色：注意
    QColor err;        // 状态色：错误
    QColor hover;      // 按钮悬停底
    QColor pressed;    // 按钮按下底
    QColor disabledBg; // 禁用态底
    QColor canvas;     // 视口画布（固定深色，**不进 QSS**，只在代码里取）
    QColor frame;      // 窗口外框（整个程序四周那一圈浅蓝）
};

// 尺度阶梯（逻辑像素）。QSS 通过 `@radius` / `@control-h` 这类占位符引用同一份取值，
// 免得模板里散着一堆裸数字。
namespace Metric {
constexpr int RadiusSm      = 4;
constexpr int Radius        = 6;
constexpr int RadiusLg      = 8;
constexpr int ControlH      = 28; // 按钮/输入框标准高度
constexpr int ControlHSmall = 24; // 紧凑行（表头、悬浮条按钮）
// 参数表第一列里的折叠按钮（−/+ 方块）的**内容盒**尺寸。
// 两个坑都写在这儿，改之前先读完：
//  1) QSS 的 min-width/min-height 会由 QStyleSheetStyle 落到 setMinimumWidth/Height 上，
//     而且**要再加一圈描边**（这里 1px 描边 → 26px 写出去、量到的是 28px）。所以代码里
//     不要再用 setFixedSize 跟它抢：两套取值一打架，min 比 max 大，按钮就成了被裁一半的方块。
//  2) 表格 cell widget 的可用区不是列宽/行高，而是「减掉 QSS 里 QTableWidget::item 的
//     padding（各 @gap-sm / @gap-xs）再加 1px 网格线」。列宽/行高要按 28 反推（见 params.cpp）。
constexpr int FoldBtnPx     = 26;
constexpr int SectionHeaderH = 32; // 折叠分组标题行（原型 .sech，整行可点）
// 窗口最外圈那条描边的宽度。真正的绘制在 FramelessWindow 里的 WindowFrame 覆盖条上
// （QSS 给 QMainWindow 写 border 会被标题栏 / 中央区 / 状态栏盖掉，见那边的注释）。
constexpr int FramePx        = 2;
constexpr int GapXs         = 4;
constexpr int GapSm         = 8;
constexpr int Gap           = 12;
constexpr int GapLg         = 16;
constexpr int GapXl         = 24;
constexpr int FontPtBase    = 12; // 正文（沿用改造前的取值，观感调整留给收尾票）
constexpr int FontPtSmall   = 10; // 次要文本
constexpr int FontPtTiny    = 9;  // 注释、计数
// 参数设置页专用的一档：这张表是「逐行对号 + 念说明」的阅读场景，
// 屏幕前隔着一段距离看，比正文再大一档才不费眼。
// 只作用于 `Params` 那一页（见 app.qss.in「参数设置页字号」那段），
// 想整体调大 / 调小就改这一个常数——列宽与最小行高都按它换算（params.cpp 的 scaleForParamFont）。
constexpr int FontPtParam   = 14;
// 「操作说明」这类备注：**用 px 不用 pt**（spec 定的就是 11px）。
// pt 在 125%/150% 缩放下会被系统放大到读起来像正文，而这一档要的就是「比正文小一截且稳定」。
constexpr int FontPxHint    = 11;
} // namespace Metric

// ---------------------------------------------------------------- 动效时长
//
// 全应用的过渡时长只有这三档，取值对齐 docs/prototype-v2.html 的 transition
// （.seg .16s / .sech .18s / .collapsebtn .2s / .fbtn .16s），都落在 spec 要求的
// 150~250ms 里。
//
// **过渡只给四类控件**：命令栏分段指示条、折叠分组箭头、悬浮条按钮、面板收起把手。
// 其余按钮一律静态切色 —— 全应用都加动画会显得黏（spec 用户故事 52 的原话）。
//
// 另一条硬约束：过渡**只作用于外观**（绘制时插值的角度 / 颜色 / 不透明度），
// 不动几何、不改事件掩码、不装事件过滤。所以动画期间控件的命中区域始终就是它自己的
// 矩形，不存在「动画进行中漏掉鼠标」的窗口期（悬浮条尤其不能犯这个错）。
namespace Motion {
constexpr int HoverMs  = 160; // 悬停 / 按下的底色与图标渐变
constexpr int SlideMs  = 180; // 命令栏分段指示条滑动、折叠分组箭头翻转
constexpr int ToggleMs = 200; // 面板收起把手的箭头掉头
} // namespace Motion

// 调色板。默认浅色，也是本期唯一启用的那套。
const Palette &palette(Variant v = Variant::Light);

// 全局字体（家族 + 基准字号 + style hint）。
QFont font(Variant v = Variant::Light);

// 与 QSS 手工配对的 QPalette：QSS 管不到的地方（弹窗、弹层、滚动条角落）走这里。
QPalette qpalette(Variant v = Variant::Light);

// 占位符替换表：键是不带 `@` 的名字（"accent"），值是替换后的文本（"#2563eb"、"6px"）。
QHash<QString, QString> tokens(Variant v = Variant::Light);

// 纯函数：把模板里的占位符按表替换掉。不认识的名字原样保留。
// 单测（tests/test_theme.cpp）只盯这一个函数。
QString substitute(const QString &tmpl, const QHash<QString, QString> &table);

// 读 qrc 里的模板原文（`:/style/app.qss.in`）。资源缺失时返回空串。
QString templateText();

// 替换完成的样式表 = substitute(templateText(), tokens(v))。
QString styleSheet(Variant v = Variant::Light);

// 程序启动时**只调一次**：字体 + QPalette + QSS。重复调用直接返回。
void init(QApplication *app, Variant v = Variant::Light);

// 按占位符名取色（"accent" → QColor("#2563eb")）。代码里要色值时走这里，
// 不要在别处写 "#2563eb"。名字不认识返回无效颜色。
QColor color(const QString &token, Variant v = Variant::Light);

// ---------------------------------------------------------------- 状态色
//
// 「这段文字现在是什么状态」表达成一个**动态属性**，颜色只在 QSS 里定
// （`QLabel[state="err"]` 这类属性选择器）。这样同一处语义永远只有一个色值，
// 业务代码里不再出现任何色值字面量。
//
//     Theme::setState(ui->lblPortState, "ok");    // 正常（绿）
//     Theme::setState(ui->lblWifiState, "warn");  // 注意（橙）
//     Theme::setState(ui->lblPortState, "err");   // 故障（红）
//     Theme::setState(ui->optLabel,     "info");  // 中性提示（强调色）
//     Theme::setState(ui->optLabel,     "");      // 清掉，回到正文色
//
// 内部只做三件事：写属性 → `unpolish`/`polish` → `update()`。
//
// **幂等是硬要求**：值没变就直接返回，不重复 polish。这些标签挂在每秒刷新的路径上
// （串口状态、TCP 重连、收发计数），每帧重排一遍界面会抖。
//
// 不认识的状态名按「清掉」处理并 `qWarning` —— 拼错了要看得见，不能静默留着上一次的颜色。
void setState(QWidget *widget, const QString &state);

// 控件当前的状态值（"ok"/"warn"/"err"/"info"）。没设过或已清掉返回空串。
QString stateOf(const QWidget *widget);

// 诊断缝：`setState` 每**真正**触发一次 polish 就回调一次。
// 生产代码不装（默认空指针，代价只是每调用一次判空）；
// tests/test_state.cpp 装一个计数器，用来断言「重复设置同值不重复 polish」。
using PolishObserver = void (*)(QWidget *widget, const QString &state);
void setPolishObserver(PolishObserver observer);

} // namespace Theme

#endif // THEME_H
