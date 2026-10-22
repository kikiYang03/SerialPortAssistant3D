#ifndef WINDOWGEOMETRY_H
#define WINDOWGEOMETRY_H

#include <QList>
#include <QRect>
#include <QSize>

// 窗口几何的恢复策略（工单 11）。
//
// 刻意做成**不碰窗口、不碰 QScreen 的纯函数**：输入是「上次记下的矩形」与
// 「当前各显示器的可用区」，输出是「这次该摆在哪儿」。多显示器拔插、分辨率变小、
// 记录整个落在屏外这些分支，都能在控制台里跑一遍，不需要真的插两块屏。
//
// 只算矩形，不负责读写 QSettings、不负责最大化态 —— 那些是 FramelessWindow 的事。
namespace WindowGeometry {

// saved     上次记下的窗口框（逻辑像素、全局坐标）。宽或高 ≤ 0 视为没有记录。
// available 当前各显示器的可用区（工作区，不含任务栏）。顺序无关。
// minimum   窗口的最小尺寸（布局给的硬下限）。
//
// 返回可直接 setGeometry 的矩形；saved 无有效记录或 available 里没有一块能用的屏时
// 返回空矩形 —— 调用方据此**跳过**恢复，把首次摆放交回系统默认（绝不自己编一个位置）。
QRect restoreRect(const QRect &saved, const QList<QRect> &available, const QSize &minimum);

} // namespace WindowGeometry

#endif // WINDOWGEOMETRY_H
