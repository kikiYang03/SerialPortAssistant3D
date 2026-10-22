#ifndef PANELRAIL_H
#define PANELRAIL_H

#include <QList>
#include <QRect>
#include <QSize>
#include <QWidget>

class QAbstractButton;
class QStackedLayout;

// 右面板的两态容器（工单 09）：
//
//   PanelHost ── 展开态 = 右面板滚动区（默认 296px）
//              └ 收起态 = PanelRail（26px 窄条，竖排「属性面板」）
//
// 一个控件承载两态，是因为「收起」不能只是把面板藏起来：宽度要还给视口、
// 还要留一条能点回来的窄条，这两件事都得在同一个容器上做，分裂器才只需面对一个子控件。
//
// 三件事写在这个文件里：
//   * PanelWidth       —— 宽度策略的**纯函数**（不碰窗口，控制台测试直接跑）；
//   * PanelRail        —— 收起后的 26px 窄条（自绘）；
//   * PanelHost        —— 容器本身 + 那个**收起/展开共用的**小按钮。
//
// 颜色、字号、尺寸一律来自 Theme 与下面的常量，这里不写任何色值字面量。

namespace PanelWidth {

// 展开态的默认宽度（原型 .panel width:296px）。只是**首次**的取值，
// 之后跟着用户拖出来的宽度走。
constexpr int kDefaultExpanded = 296;

// 收起态的窄条宽度：本票的硬指标（用户故事 47 / 原型 .panel.collapsed）。
constexpr int kRail = 26;

// 展开态的分隔条宽度（与全局 QSS 的 `QSplitter::handle` 那一节同一个值）。
constexpr int kHandle = 6;

// 展开态面板的下限：再窄，「目标点指引」那几个数值框就会挤成一列。
// 与 `QScrollArea::setMinimumWidth(240)` 是同一个数，放这里是为了让纯函数能判「容器够不够宽」。
constexpr int kMinExpanded = 240;

// 视口的下限（与 `gl_->setMinimumWidth(240)` 同一个数）：钉宽之后画布还得剩这么多。
constexpr int kMinViewport = 240;

// 面板宽度的记忆。`expanded` = 用户上次拖出来的宽度，**0 表示还没拖过**（用默认值）。
//
// 收起态**不动** `expanded` —— 「展开 → 收起 → 展开回到收起前的值」这条就是它的直接结果，
// 不需要另存一份「收起前的宽度」。
struct Memory {
    bool collapsed = false;
    int  expanded  = 0;
};

// 当前态该给面板多宽：收起态恒为窄条宽度，展开态给记住的值（没记过就是默认值）。
int panelWidth(const Memory &m);

// 记下用户拖出来的宽度。收起态、非法值（<=0）一律忽略 —— 收起时的 26px 不是用户的意愿。
Memory remember(const Memory &m, int width);

// 开合一次。`currentWidth` 是**此刻面板的真实宽度**：收起来的那一刻要把它记下来，
// 展开时则原样保留（这样回来还是收起前的宽度）。
Memory toggle(const Memory &m, int currentWidth);

// 容器够不够宽到「钉」一个确定的面板宽度：钉完之后画布仍要剩下 kMinViewport。
// 不够就不钉，交给分裂器按比例分（启动瞬间宽度还没排出来的那种情况也走这条）。
bool canPin(int containerWidth, int panel);

// 分裂器两段宽度（**已扣掉把手**：QSplitter::setSizes 收的是不含把手的宽度）。
// 面板宽度先夹进可用宽度里，所以「容器比面板还窄」时视口拿 0 而不是负数。
QList<int> splitSizes(int containerWidth, int panel, int handle);

} // namespace PanelWidth

// 收起按钮的几何换算（纯函数）：按钮恒为 13×46，贴在面板**左缘外侧**、垂直居中。
//
// 为什么在外侧：原型 `.collapsebtn{left:-13px}` 是一枚从面板伸进画布的把手 ——
// 左缘看不见描边、右缘与面板齐平，收起后同一枚把手还在原地，只有箭头翻向。
// 参数是面板矩形（任意坐标系），返回按钮在同一坐标系里的矩形。
namespace PanelRailGeometry {

constexpr int kToggleW = 13;   // 原型 .collapsebtn width:13px
constexpr int kToggleH = 46;   // 原型 .collapsebtn height:46px

QRect toggleRect(const QRect &panelRect);

} // namespace PanelRailGeometry

// 收起后的窄条：竖排「属性面板」+ 一个小圆点。
//
// 它不是装饰：面板收起来之后，这条是「面板去哪了」的说明，点它就能展开回来
// （共用的那枚按钮在左缘，也不冲突 —— 两个入口都指向同一个动作）。
class PanelRail : public QWidget
{
    Q_OBJECT

public:
    explicit PanelRail(QWidget *parent = nullptr);

    QSize sizeHint() const override;
    QSize minimumSizeHint() const override;

signals:
    // 点窄条 = 展开。真正的状态切换在 PanelHost 里，这里只发「我想展开」。
    void expandRequested();

protected:
    void paintEvent(QPaintEvent *e) override;
    void mousePressEvent(QMouseEvent *e) override;
    void enterEvent(QEvent *e) override;
    void leaveEvent(QEvent *e) override;

private:
    bool hover_ = false;
};

// 两态容器。对外只有「收起/展开/现在多宽」三件事，分裂器的重排由调用方（Ros3DPage）
// 在 collapsedChanged 里做 —— 容器不该反过来指挥分裂器。
class PanelHost : public QWidget
{
    Q_OBJECT

public:
    explicit PanelHost(QWidget *parent = nullptr);

    // 展开态的内容（右面板那个滚动区）。加进来之后归本类管，调用方不用再管归属。
    void setExpandedContent(QWidget *content);

    // 收起按钮的**覆盖层**：按钮要压 13px 到视口上，而 Qt 会把子控件裁到父控件矩形内
    // （实测：完全落在父控件外的子控件一个像素都不画），所以它的父对象只能是同时盖住
    // 视口与面板的那一层 —— Ros3DPage。定位仍由本类按自己的 geometry() 算，
    // 不会与分裂器脱节（这是「按钮挂在分裂器外面的浮层上」真正要防的那件事）。
    void setToggleLayer(QWidget *layer);

    bool isCollapsed() const { return memory_.collapsed; }

    // 记住的展开宽度；0 = 还没拖过（此刻用默认宽度）。
    int expandedWidth() const { return memory_.expanded; }

    // 当前态该多宽（纯函数转一手，见 PanelWidth::panelWidth）。
    int preferredWidth() const { return PanelWidth::panelWidth(memory_); }

    // 用户拖完分隔条后把宽度记下来（收起态忽略）。
    void rememberExpandedWidth(int width);

public slots:
    void setCollapsed(bool collapsed);
    void toggleCollapsed();

signals:
    void collapsedChanged(bool collapsed);

protected:
    void resizeEvent(QResizeEvent *e) override;
    void moveEvent(QMoveEvent *e) override;
    void showEvent(QShowEvent *e) override;
    bool event(QEvent *e) override;

private:
    // 两态共用的收尾：栈页、最小宽度、按钮箭头与提示。重复调用无副作用。
    void applyState();
    // 按自己的 geometry() 摆那枚按钮（垂直居中、左缘压 13px 到视口）。
    void placeToggleButton();

    QStackedLayout *stack_ = nullptr;
    PanelRail *rail_ = nullptr;
    QWidget *expanded_ = nullptr;
    QWidget *toggleLayer_ = nullptr;

    // 类型在 .cpp 里（自绘 QAbstractButton），头文件不关心它长什么样。
    QAbstractButton *toggle_ = nullptr;

    PanelWidth::Memory memory_;
};

#endif // PANELRAIL_H
