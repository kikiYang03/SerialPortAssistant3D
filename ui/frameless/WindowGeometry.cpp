#include "WindowGeometry.h"

#include <QPoint>
#include <QtGlobal>

namespace {

// 两块矩形的相交面积（不相交为 0）。
qint64 intersectionArea(const QRect &a, const QRect &b)
{
    const QRect r = a.intersected(b);
    if (r.isEmpty())
        return 0;
    return qint64(r.width()) * qint64(r.height());
}

// 两点距离的平方：只用来比大小，不必开方。
qint64 squaredDistance(const QPoint &a, const QPoint &b)
{
    const qint64 dx = qint64(a.x()) - b.x();
    const qint64 dy = qint64(a.y()) - b.y();
    return dx * dx + dy * dy;
}

} // namespace

QRect WindowGeometry::restoreRect(const QRect &saved, const QList<QRect> &available,
                                  const QSize &minimum)
{
    // 没有记录就什么都别确定：调用方会跳过恢复，窗口按系统默认摆放
    // （也就是主屏上、尺寸来自 .ui），不会摆到屏外也不会缩成 0。
    if (saved.width() <= 0 || saved.height() <= 0)
        return QRect();

    QList<QRect> areas;
    for (const QRect &area : available) {
        if (area.width() > 0 && area.height() > 0)
            areas.append(area);
    }
    if (areas.isEmpty())
        return QRect();

    // 1) 找回上次待的那块屏：先是「交集最大」，都交不上再看「中心最近」。
    //    后者兜住「显示器拔了 / 分辨率变窄、记录整个落在屏外」——这时不能随手取
    //    列表里第一块，否则窗口会跑到一台没人预期的屏上。
    QRect target = areas.first();
    qint64 best = 0;
    for (const QRect &area : areas) {
        const qint64 cur = intersectionArea(saved, area);
        if (cur > best) {
            best = cur;
            target = area;
        }
    }
    if (best == 0) {
        target = areas.first();
        qint64 nearest = squaredDistance(saved.center(), target.center());
        for (int i = 1; i < areas.size(); ++i) {
            const qint64 cur = squaredDistance(saved.center(), areas.at(i).center());
            if (cur < nearest) {
                nearest = cur;
                target = areas.at(i);
            }
        }
    }

    // 2) 尺寸：不小于最小尺寸、不大于这块屏的可用区。
    //    屏比最小尺寸还窄时以最小尺寸为准——窗口比屏宽，总好过压成一条缝。
    const int w = qBound(minimum.width(), saved.width(), qMax(target.width(), minimum.width()));
    const int h = qBound(minimum.height(), saved.height(), qMax(target.height(), minimum.height()));

    // 3) 位置：记录与这块屏完全不相交时居中摆（贴着角会让人以为窗口「卡住了」），
    //    否则沿用原位置；最后一律夹进可用区，保证标题栏与右下角都摸得到。
    int x = saved.x();
    int y = saved.y();
    if (best == 0) {
        x = target.center().x() - w / 2;
        y = target.center().y() - h / 2;
    }
    x = (w >= target.width()) ? target.left() : qBound(target.left(), x, target.right() - w + 1);
    y = (h >= target.height()) ? target.top() : qBound(target.top(), y, target.bottom() - h + 1);

    return QRect(x, y, w, h);
}
