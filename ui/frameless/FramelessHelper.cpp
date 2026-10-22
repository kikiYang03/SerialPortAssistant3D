#include "FramelessHelper.h"

#include <QAbstractButton>
#include <QHash>
#include <QPoint>
#include <QPointer>
#include <QRect>
#include <QSize>
#include <QString>
#include <QVariant>
#include <QWidget>
#include <QWindow>

#ifdef Q_OS_WIN

#ifndef WIN32_LEAN_AND_MEAN
#  define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#  define NOMINMAX
#endif
#include <windows.h>
#include <windowsx.h>   // GET_X_LPARAM
#include <dwmapi.h>     // DwmSetWindowAttribute

// Win11 才有这个属性号（SDK 10.0.19041 的头文件里还没有），自己补上。
#ifndef DWMWA_WINDOW_CORNER_PREFERENCE
#  define DWMWA_WINDOW_CORNER_PREFERENCE 33
#endif
// DWMWCP_ROUND：8px 圆角。Win10 上这个调用直接返回 E_INVALIDARG，
// 我们正是拿返回值判断「系统会不会替我们画圆角」，不做版本嗅探。
static const int kDwmCornerRound = 2;

namespace {

const int kDefaultResizeBorder = 8;

// 每个窗口一份。窗口销毁时一并删掉，避免多实例（含测试）时指针串味。
struct State
{
    QPointer<QWidget> titleBar;
    int resizeBorder = kDefaultResizeBorder;
    bool dwmChecked = false;
    bool dwmRounded = false;
    bool syncingFrame = false;   // 正在补 FRAMECHANGED，别自己咬自己
};

QHash<const QWidget *, State *> &registry()
{
    static QHash<const QWidget *, State *> r;
    return r;
}

State *stateOf(const QWidget *window)
{
    return registry().value(window, nullptr);
}

// 命中测试的输入是物理像素，界面里的一切都按逻辑像素算，进来先除 DPR。
qreal dprOf(const QWidget *window)
{
    if (QWindow *h = window->windowHandle())
        return h->devicePixelRatio();
    return window->devicePixelRatioF();
}

QPoint logicalPos(QWidget *window, LPARAM lParam)
{
    const QPoint physical(GET_X_LPARAM(lParam), GET_Y_LPARAM(lParam));
    const qreal dpr = dprOf(window);
    const QPoint global(qRound(physical.x() / dpr), qRound(physical.y() / dpr));
    return window->mapFromGlobal(global);
}

// 8px 边带 → 缩放命中码。四角优先，否则取单边。
LRESULT pickResizeHandle(const QPoint &p, const QSize &size, int border)
{
    const bool left   = p.x() >= 0 && p.x() < border;
    const bool right  = p.x() < size.width() && p.x() >= size.width() - border;
    const bool top    = p.y() >= 0 && p.y() < border;
    const bool bottom = p.y() < size.height() && p.y() >= size.height() - border;

    if (top && left)     return HTTOPLEFT;
    if (top && right)    return HTTOPRIGHT;
    if (bottom && left)  return HTBOTTOMLEFT;
    if (bottom && right) return HTBOTTOMRIGHT;
    if (left)            return HTLEFT;
    if (right)           return HTRIGHT;
    if (top)             return HTTOP;
    if (bottom)          return HTBOTTOM;
    return HTNOWHERE;
}

// 标题栏内的命中：从命中的子控件往上走到标题栏为止，看沿途谁「要点击」。
// 返回 HTNOWHERE 表示这一块是空白，可以当标题栏拖。
LRESULT titleBarHit(QWidget *titleBar, const QPoint &posInBar)
{
    QWidget *hit = titleBar->childAt(posInBar);
    while (hit && hit != titleBar) {
        const QString role = hit->property(FramelessHelper::kHitProperty).toString();
        if (role == QLatin1String(FramelessHelper::kHitMaxButton))
            return HTMAXBUTTON;
        if (role == QLatin1String(FramelessHelper::kHitClient))
            return HTCLIENT;
        // 没打属性的按钮也要能点：否则「其余三键与控件必须返回 HTCLIENT」这条就落空了。
        if (qobject_cast<QAbstractButton *>(hit))
            return HTCLIENT;
        hit = hit->parentWidget();
    }
    return HTNOWHERE;
}

MONITORINFO monitorInfoOf(HWND hwnd)
{
    MONITORINFO mi;
    mi.cbSize = sizeof(MONITORINFO);
    if (!GetMonitorInfoW(MonitorFromWindow(hwnd, MONITOR_DEFAULTTONEAREST), &mi)) {
        mi.rcMonitor = RECT{0, 0, 0, 0};
        mi.rcWork    = RECT{0, 0, 0, 0};
    }
    return mi;
}

// 窗口矩形是否已经正好贴在这块屏的 work area 上。
// 用来判断「最大化尺寸是我们自己钉的（WM_GETMINMAXINFO），还是系统默认外扩了一圈」——
// 前者不能再按边框内缩，否则内容会小一圈。
bool windowRectIsWorkArea(HWND hwnd)
{
    RECT wr;
    if (!GetWindowRect(hwnd, &wr))
        return false;
    const MONITORINFO mi = monitorInfoOf(hwnd);
    const auto near2 = [](int a, int b) { return qAbs(a - b) <= 2; };
    return near2(wr.left, mi.rcWork.left) && near2(wr.top, mi.rcWork.top)
        && near2(wr.right, mi.rcWork.right) && near2(wr.bottom, mi.rcWork.bottom);
}

void applyDwmRounding(HWND hwnd, State &st)
{
    st.dwmChecked = true;
    const int pref = kDwmCornerRound;
    const HRESULT hr = ::DwmSetWindowAttribute(hwnd, DWMWA_WINDOW_CORNER_PREFERENCE,
                                              &pref, sizeof(pref));
    st.dwmRounded = SUCCEEDED(hr);
}

// 「启动时就该是最大化」这条路径（工单 11 恢复最大化态）与运行期按最大化键不同：
// 首次 ShowWindow(SW_SHOWMAXIMIZED) 时，系统在 WM_NCCALCSIZE 之后又把窗口矩形往外
// 扩了一圈边框（实测工作区 1920×1040 → 窗口矩形 -8,-8,1936×1056），而客户区是在
// NCCALCSIZE 里定的，于是客户区也变成 1936×1056 —— 内容被屏幕边裁掉 8px。
// 只在「客户区比工作区还大」时补一次 SWP_FRAMECHANGED，让 NCCALCSIZE 拿新的
// 窗口矩形重算；运行期最大化的那条路客户区本来就正好等于工作区，不会被碰。
void syncMaximizedClientArea(HWND hwnd, State &st)
{
    if (st.syncingFrame)
        return;

    RECT cr;
    if (!GetClientRect(hwnd, &cr))
        return;
    const MONITORINFO mi = monitorInfoOf(hwnd);
    const int cw = cr.right - cr.left;
    const int ch = cr.bottom - cr.top;
    const int ww = mi.rcWork.right - mi.rcWork.left;
    const int wh = mi.rcWork.bottom - mi.rcWork.top;
    if (ww <= 0 || wh <= 0 || (cw <= ww && ch <= wh))
        return;

    st.syncingFrame = true;
    SetWindowPos(hwnd, nullptr, 0, 0, 0, 0,
                 SWP_NOMOVE | SWP_NOSIZE | SWP_NOZORDER | SWP_NOACTIVATE | SWP_FRAMECHANGED);
    st.syncingFrame = false;
}

} // namespace

// ---------------------------------------------------------------- 公开面

void FramelessHelper::install(QWidget *window)
{
    if (!window || stateOf(window))
        return;

    auto *st = new State;
    registry().insert(window, st);

    // 刻意什么都不改：不设 Qt::FramelessWindowHint、不碰 GWL_STYLE。
    // 窗口必须继续带着 WS_CAPTION / WS_THICKFRAME —— DWM 投影、贴边吸附、
    // Win+方向键、任务栏点击最小化全部挂在它们身上，去掉就全废了。
    // 我们要的只是「客户区 = 整窗」，那件事在 WM_NCCALCSIZE 里做。

    QObject::connect(window, &QObject::destroyed, [window]() {
        delete registry().take(window);
    });
}

void FramelessHelper::setTitleBar(QWidget *window, QWidget *titleBar)
{
    if (State *st = stateOf(window))
        st->titleBar = titleBar;
}

void FramelessHelper::setResizeBorder(QWidget *window, int px)
{
    if (State *st = stateOf(window))
        st->resizeBorder = qBound(0, px, 32);
}

bool FramelessHelper::usesDwmRounding(const QWidget *window)
{
    State *st = stateOf(window);
    if (!st)
        return false;
    if (!st->dwmChecked) {
        // 还没收到过任何原生消息（窗口还没建出来）：先当作没有系统圆角。
        QWindow *h = const_cast<QWidget *>(window)->windowHandle();
        if (!h)
            return false;
        applyDwmRounding(reinterpret_cast<HWND>(h->winId()), *st);
    }
    return st->dwmRounded;
}

bool FramelessHelper::handleNativeEvent(QWidget *window, void *message, long *result)
{
    State *st = stateOf(window);
    if (!st || !message || !result)
        return false;

    MSG *msg = static_cast<MSG *>(message);
    HWND hwnd = msg->hwnd;

    // 原生窗口一出现就把圆角偏好递上去（只需要一次）。
    if (!st->dwmChecked)
        applyDwmRounding(hwnd, *st);

    switch (msg->message) {

    case WM_NCCALCSIZE: {
        if (msg->wParam != TRUE)
            return false;   // 极少见路径（只问客户区大小），交回默认实现

        auto *params = reinterpret_cast<NCCALCSIZE_PARAMS *>(msg->lParam);

        // 默认最大化会把窗口向外扩一圈边框，好让内容正好落在 work area 上。
        // 我们的客户区 = 整窗，这圈必须补回去，否则内容被屏幕边 / 任务栏裁掉。
        // 但如果 WM_GETMINMAXINFO 已经把最大化尺寸钉成了 work area，就不能再缩。
        if (IsZoomed(hwnd) && !windowRectIsWorkArea(hwnd)) {
            const int bw = GetSystemMetrics(SM_CXSIZEFRAME)
                         + GetSystemMetrics(SM_CXPADDEDBORDER);
            params->rgrc[0].left   += bw;
            params->rgrc[0].right  -= bw;
            params->rgrc[0].top    += bw;
            params->rgrc[0].bottom -= bw;
        }

        *result = 0;    // 客户区 = 整窗，这就是「没有系统边框」的全部实现
        return true;
    }

    case WM_NCHITTEST: {
        // 先让系统算一遍：窗口外沿的缩放边带由它给出，直接采用。
        // 但 HTCAPTION 不能直接采用 —— 窗口仍带 WS_CAPTION，系统可能把顶部
        // 整条都算成标题栏，那样三键就收不到点击了。那块地方由下面自己分配。
        const LRESULT sys = DefWindowProcW(hwnd, msg->message, msg->wParam, msg->lParam);
        if (sys != HTCLIENT && sys != HTCAPTION) {
            *result = sys;
            return true;
        }

        const QPoint pos = logicalPos(window, msg->lParam);
        const QSize size = window->size();
        // 最大化态没有可拖的边（系统对普通窗口也是这个行为）。
        const auto edgeAt = [&]() -> LRESULT {
            return IsZoomed(hwnd) ? HTNOWHERE
                                  : pickResizeHandle(pos, size, st->resizeBorder);
        };

        // 1) 标题栏：最大化键先认（Snap 弹层要用 HTMAXBUTTON），
        //    其余按钮/控件给 HTCLIENT，空白处才轮到缩放边带与拖动。
        if (QWidget *bar = st->titleBar.data()) {
            const QRect barRect(bar->mapTo(window, QPoint(0, 0)), bar->size());
            if (barRect.contains(pos)) {
                const LRESULT barHit = titleBarHit(bar, bar->mapFrom(window, pos));
                if (barHit != HTNOWHERE) {
                    *result = barHit;
                    return true;
                }
                const LRESULT edge = edgeAt();
                *result = (edge != HTNOWHERE) ? edge : HTCAPTION;
                return true;
            }
        }

        // 2) 其余区域：8px 边带缩放，别处都是客户区。
        const LRESULT edge = edgeAt();
        *result = (edge != HTNOWHERE) ? edge : HTCLIENT;
        return true;
    }

    case WM_NCLBUTTONDOWN:
        if (msg->wParam == HTMAXBUTTON) {
            // 命中值是我们自己给的，动作也自己落地。走 Qt 的接口而不是 ShowWindow，
            // Qt 内部的窗口状态才会跟着更新（否则 isMaximized() 会撒谎）。
            if (window->isMaximized())
                window->showNormal();
            else
                window->showMaximized();
            *result = 0;
            return true;
        }
        break;

    case WM_SIZE:
        // 「首次显示即最大化」那条路要补一次客户区（见 syncMaximizedClientArea）。
        if (msg->wParam == SIZE_MAXIMIZED)
            syncMaximizedClientArea(hwnd, *st);
        break;

    case WM_GETMINMAXINFO: {
        // 先让系统把 ptMaxTrackSize / ptMinTrackSize 这些填成默认值：
        // 一旦自己接管这条消息，没填的字段会被当成 0，窗口就再也拉不大了。
        DefWindowProcW(hwnd, msg->message, msg->wParam, msg->lParam);

        auto *mmi = reinterpret_cast<MINMAXINFO *>(msg->lParam);
        const MONITORINFO mi = monitorInfoOf(hwnd);
        if (mi.rcWork.right > mi.rcWork.left && mi.rcWork.bottom > mi.rcWork.top) {
            // 最大化尺寸按「当前这块屏」的 work area 修正（不是主屏、
            // 也不含任务栏），跨屏最大化不会跑到另一块屏上去。
            mmi->ptMaxPosition.x = mi.rcWork.left - mi.rcMonitor.left;
            mmi->ptMaxPosition.y = mi.rcWork.top - mi.rcMonitor.top;
            mmi->ptMaxSize.x     = mi.rcWork.right - mi.rcWork.left;
            mmi->ptMaxSize.y     = mi.rcWork.bottom - mi.rcWork.top;
        }
        *result = 0;
        return true;
    }

    default:
        break;
    }
    return false;
}

#else // !Q_OS_WIN

// 非 Windows 平台：这套无边框方案只有 Win32 一份，其它平台保持原生边框。
// 保留同样的签名，Linux 构建（见 linux编译指令.md）不会因为缺符号而失败。
namespace {
QHash<const QWidget *, int> g_resizeBorder;
}

void FramelessHelper::install(QWidget *) {}
void FramelessHelper::setTitleBar(QWidget *, QWidget *) {}
void FramelessHelper::setResizeBorder(QWidget *window, int px) { g_resizeBorder.insert(window, px); }
bool FramelessHelper::handleNativeEvent(QWidget *, void *, long *) { return false; }
bool FramelessHelper::usesDwmRounding(const QWidget *) { return false; }

#endif // Q_OS_WIN
