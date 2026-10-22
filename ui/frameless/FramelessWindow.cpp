#include "FramelessWindow.h"

#include "FramelessHelper.h"
#include "Theme.h"
#include "WindowGeometry.h"

#include <QCloseEvent>
#include <QCoreApplication>
#include <QDir>
#include <QGuiApplication>
#include <QMoveEvent>
#include <QPainter>
#include <QRegion>
#include <QResizeEvent>
#include <QScreen>
#include <QSettings>
#include <QShowEvent>
#include <QStandardPaths>
#include <QTimer>

/* 整个程序四周那圈浅蓝描边（Theme::Metric::FramePx）。

   为什么不是 QSS 的 `QMainWindow { border: ... }`：标题栏、中央区、状态栏三块把客户区
   铺满了，画在它们下面的边框看不见；靠 `setContentsMargins` 让内容内缩也不行 ——
   QMainWindowLayout 不认这个边距（实测边缘像素一动不动）。

   所以用一个盖在最上层、只留四条边带的小控件：
   - `setMask()` 把中间整块裁掉，剩下的 2px 环不透明地刷成 @frame；
   - `WA_TransparentForMouseEvents` 让点击/拖拽照旧穿过去（边缘缩放本来走原生命中测试）。 */
class WindowFrame : public QWidget
{
public:
    explicit WindowFrame(QWidget *parent)
        : QWidget(parent)
    {
        setAttribute(Qt::WA_TransparentForMouseEvents);
        setAttribute(Qt::WA_NoSystemBackground);
        setFocusPolicy(Qt::NoFocus);
    }

    // 边宽只在 Theme::Metric 里定，这里不存第二份。
    int band() const { return Theme::Metric::FramePx; }

    void fitTo(const QRect &area)
    {
        setGeometry(area);
        QRegion ring(area);
        ring -= QRegion(area.adjusted(band(), band(), -band(), -band()));
        setMask(ring);
        raise();   // 后加的标题栏 / 状态栏会压到上层，每次重排都顶回来
    }

protected:
    void paintEvent(QPaintEvent *) override
    {
        QPainter p(this);
        p.fillRect(rect(), Theme::color(QStringLiteral("frame")));
    }
};

namespace {

// 窗口框是「这台机器上这个用户」的临时偏好，刻意**不写进随包分发的 config.ini**：
// 那份由运维按机器配、可能落在只读目录里，而且它是「输入」不是「运行时输出」。
// 用户配置目录第一次写入时自己建出来。
QString settingsFile()
{
    const QString dir = QStandardPaths::writableLocation(QStandardPaths::AppConfigLocation);
    if (dir.isEmpty())
        return QCoreApplication::applicationDirPath() + QStringLiteral("/window.ini");
    QDir().mkpath(dir);
    return dir + QStringLiteral("/window.ini");
}

const char *const kGeometryKey  = "Window/geometry";
const char *const kMaximizedKey = "Window/maximized";

} // namespace

FramelessWindow::FramelessWindow(QWidget *parent)
    : QMainWindow(parent)
{
    // 只接管消息，窗口样式一位都不改（不设 FramelessWindowHint）。
    FramelessHelper::install(this);

    // 窗口最外圈那条浅蓝边。构造期只是先把覆盖条建出来，几何与 z-order 在
    // resize / show 里补 —— 那时 MainWindow 已经把自己的标题栏和状态栏挂上去了，
    // 会把这层压到下面。
    frame_ = new WindowFrame(this);
    frame_->fitTo(rect());
}

FramelessWindow::~FramelessWindow() = default;

QWidget *FramelessWindow::titleBar() const
{
    return titleBar_;
}

void FramelessWindow::setTitleBar(QWidget *bar)
{
    if (bar == titleBar_)
        return;

    if (bar) {
        bar->setParent(this);
        setMenuWidget(bar);   // QMainWindow 接管所有权，占的是菜单栏那一行
        bar->show();
    } else {
        setMenuWidget(nullptr);
    }

    titleBar_ = bar;
    FramelessHelper::setTitleBar(this, bar);
}

void FramelessWindow::setResizeBorder(int px)
{
    FramelessHelper::setResizeBorder(this, px);
}

bool FramelessWindow::nativeEvent(const QByteArray &eventType, void *message, long *result)
{
#ifdef Q_OS_WIN
    if (FramelessHelper::handleNativeEvent(this, message, result))
        return true;
#endif
    return QMainWindow::nativeEvent(eventType, message, result);
}

// ---------------------------------------------------------------- 窗口几何记忆（工单 11）

void FramelessWindow::restoreWindowState()
{
    QSettings settings(settingsFile(), QSettings::IniFormat);
    const QRect saved = settings.value(QString::fromLatin1(kGeometryKey)).toRect();
    const bool wasMaximized = settings.value(QString::fromLatin1(kMaximizedKey), false).toBool();

    // 没有记录（首次运行）：一个字节都不动。位置交给系统默认，尺寸沿用 .ui，
    // 绝不会摆到屏外，也不会被我们写成 0×0。
    if (!saved.isValid())
        return;

    QList<QRect> areas;
    for (const QScreen *screen : QGuiApplication::screens())
        areas.append(screen->availableGeometry());

    // 校验在 WindowGeometry 里：选屏、夹尺寸、把框收回可用区。
    // 上次那台显示器要是拔了，这里会把它挪到最近的屏中央，而不是留在虚无里。
    const QRect target = WindowGeometry::restoreRect(saved, areas, minimumSize());
    if (!target.isEmpty()) {
        setGeometry(target);
        normalFrame_ = target;   // 这就是「还原之后该回到的框」，先记下来
    }

    // 最大化态要跟着回来。走 Qt 的 setWindowState 而不是 ShowWindow：
    // 此刻窗口还没显示（QShowEvent 早于原生 show），状态会在 show 时一次落地，
    // 不会先闪一下正常态再弹成最大化；isMaximized() 之后也不会撒谎。
    if (wasMaximized) {
        setWindowState(windowState() | Qt::WindowMaximized);

        // 兜底：个别时候（Qt 在 show 之后又按 widget 自己的 geometry 摆了一次）
        // 这个状态会被吃掉，窗口以正常尺寸起来。下一轮事件循环确认一次就好，
        // 已经是最大化时什么也不做。
        QTimer::singleShot(0, this, [this]() {
            if (!isMaximized())
                showMaximized();
        });
    }
}

void FramelessWindow::rememberNormalFrame()
{
    // 最大化 / 最小化 / 全屏态下的 geometry() 已经不是用户心里的那个框了，
    // 这些事件（拖拽最大化、Aero Snap 都会走到）不能覆盖记录。
    if (isMaximized() || isMinimized() || isFullScreen())
        return;
    if (windowState() & (Qt::WindowMaximized | Qt::WindowMinimized | Qt::WindowFullScreen))
        return;
    if (!isVisible())
        return;   // 还没显示：布局没跑完，此刻的尺寸不代表最终结果

    const QRect frame = geometry();
    if (!frame.isValid())
        return;

    // 还有一个坑：点最大化时 Qt 会先送来若干个 resize/move 事件，那一刻 isMaximized()
    // 可能还没跟上（实测于是把 0,0 1920×1040 当成「正常框」记了下来）。
    // 判据用几何本身：铺满甚至超出工作区的框不可能是用户摆出来的窗口。
    const QScreen *scr = screen();
    if (scr) {
        const QRect area = scr->availableGeometry();
        if (area.isValid() && frame.width() >= area.width() && frame.height() >= area.height())
            return;
    }

    normalFrame_ = frame;
}

void FramelessWindow::saveWindowState()
{
    QRect frame = normalFrame_;
    if (!frame.isValid()) {
        // 从头到尾没记到过正常态（几乎不会发生）：退回 Qt 的说法，
        // 至少别把最大化尺寸当成「下次还原的尺寸」写进去。
        frame = (isMaximized() || isMinimized() || isFullScreen()) ? normalGeometry()
                                                                   : geometry();
    }

    // 拿不到可信的框就别写，宁可留着上一次的记录。
    if (!frame.isValid() || frame.width() <= 0 || frame.height() <= 0)
        return;

    QSettings settings(settingsFile(), QSettings::IniFormat);
    settings.setValue(QString::fromLatin1(kGeometryKey), frame);
    settings.setValue(QString::fromLatin1(kMaximizedKey), isMaximized());
    settings.sync();
}

void FramelessWindow::closeEvent(QCloseEvent *e)
{
    saveWindowState();
    QMainWindow::closeEvent(e);
}

void FramelessWindow::moveEvent(QMoveEvent *e)
{
    QMainWindow::moveEvent(e);
    rememberNormalFrame();
}

void FramelessWindow::resizeEvent(QResizeEvent *e)
{
    QMainWindow::resizeEvent(e);
    rememberNormalFrame();

    // 外框跟着窗口一起变，并重新顶到最上层（最大化 / 还原都会走到这里）。
    if (frame_)
        frame_->fitTo(rect());
}

// ---------------------------------------------------------------- 显示

void FramelessWindow::showEvent(QShowEvent *e)
{
    // 只恢复一次：之后 show/hide 往返（最小化到任务栏再点开）由系统与用户说了算，
    // 不能每次显示都把窗口拽回记录里的位置。
    // 放在这里而不是构造函数：.ui 的 resize() 在子类构造函数里跑，早于这里，
    // 也就不会把我们恢复出来的框覆盖掉。
    if (!geometryRestored_) {
        geometryRestored_ = true;
        restoreWindowState();
    }

    QMainWindow::showEvent(e);

    // 首次显示时子类才把标题栏 / 状态栏挂完，这里把外框顶回最上层并贴合当前尺寸。
    if (frame_)
        frame_->fitTo(rect());

    // 圆角偏好要等原生窗口存在才递得上去，首次绘制可能抢在它前面。
    // 这里补一次重绘，让标题栏按真实结果决定要不要画顶部那条描边。
    if (titleBar_)
        titleBar_->update();
}
