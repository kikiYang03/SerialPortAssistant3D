#ifndef RANGELIMITS_H
#define RANGELIMITS_H

#include <algorithm>

// Z 轴过滤区间的约束（工单 08 的测试缝）。
//
// 这是从 DualRangeSlider 的鼠标事件里剥出来的**纯逻辑**：没有 QWidget、没有像素换算，
// 只有「这一次想拖到哪 → 实际允许停在哪」。剥出来是为了能进控制台测试
// （tests/test_panel.cpp）——控件那一层只负责把鼠标位置换成数值、再把结果画出来。
//
// 三条规矩（原来就写在控件里，这里只是搬到可以单独验证的地方，行为一字未改）：
//
//   1) 下限不得越过上限：两者之间至少留 Step；
//   2) 越界被夹住：两端都不许跑出 [min, max]；
//   3) Step = 0.1 是**最小间距**，不是栅格量化 —— 手柄停在鼠标指向的连续位置上，
//      只在要贴上对面手柄时才退让 0.1。量化会把拖动变成「一格一格跳」，
//      而本票要求「手感不变」，所以不量化。
namespace RangeLimits {

constexpr double Step = 0.1;

struct Interval
{
    double lower;
    double upper;
};

inline double clamp(double v, double lo, double hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

// 拖下限：先夹进 [min, max]，再给上限让出 Step；让不出来（上限已经贴着 min）时退回 min，
// 不许为了「留出 Step」而越过 min。
inline Interval dragLower(double requested, const Interval &current, double min, double max)
{
    double lower = clamp(requested, min, max);
    lower = std::min(lower, current.upper - Step);
    lower = std::max(lower, min);
    return Interval{lower, current.upper};
}

// 拖上限：与 dragLower 对称（不得低于下限 + Step，不得越过 max）。
inline Interval dragUpper(double requested, const Interval &current, double min, double max)
{
    double upper = clamp(requested, min, max);
    upper = std::max(upper, current.lower + Step);
    upper = std::min(upper, max);
    return Interval{current.lower, upper};
}

// 一次性给两个值（setValues 用）：各自夹进 [min, max]，反了就换过来。
inline Interval normalize(double lower, double upper, double min, double max)
{
    Interval r{clamp(lower, min, max), clamp(upper, min, max)};
    if (r.lower > r.upper)
        std::swap(r.lower, r.upper);
    return r;
}

} // namespace RangeLimits

#endif // RANGELIMITS_H
