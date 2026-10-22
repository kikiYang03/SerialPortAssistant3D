#ifndef FRAMELESSWINDOW_H
#define FRAMELESSWINDOW_H

#include <QMainWindow>
#include <QRect>

class QCloseEvent;
class QMoveEvent;
class QResizeEvent;
class QShowEvent;
class WindowFrame;

// 无边框窗口（工单 04）。
//
// 关键分歧点：**不设 Qt::FramelessWindowHint**。窗口保留 WS_CAPTION，只在
// WM_NCCALCSIZE 里把客户区撑满整个窗口矩形 —— 于是 DWM 投影、贴边吸附、
// Win+方向键、任务栏点击最小化全部照旧工作。Win32 细节见 FramelessHelper.cpp。
//
// 标题栏挂在 QMainWindow::setMenuWidget() 上（Qt 给自绘标题栏预留的槽位），
// 中央区布局与状态栏都不动。标题栏内容本身是 ui/shell/TitleBar（05），
// 本类只负责「挂上去 + 告诉命中测试哪一块是标题栏」。
//
// 窗口几何记忆（工单 11）也落在这一层：位置 / 大小 / 最大化态在**第一次显示前**
// 恢复、在**关闭时**保存。放这里而不是 MainWindow，是因为「窗口框」属于外壳，
// 子类只管往里塞内容，不用记得调 save/restore。
class FramelessWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit FramelessWindow(QWidget *parent = nullptr);
    ~FramelessWindow() override;

    QWidget *titleBar() const;

    // 指定标题栏。内部交给 setMenuWidget()，**所有权归窗口**；传 nullptr 表示去掉。
    // 注意：setMenuWidget() 会连旧 QMenuBar 里的 QAction 一起删掉，动作要挂在窗口上。
    void setTitleBar(QWidget *bar);

    // 边缘缩放的命中带宽度（逻辑像素），默认 8。
    void setResizeBorder(int px);

protected:
    bool nativeEvent(const QByteArray &eventType, void *message, long *result) override;
    void showEvent(QShowEvent *e) override;
    void moveEvent(QMoveEvent *e) override;
    void resizeEvent(QResizeEvent *e) override;
    void closeEvent(QCloseEvent *e) override;

private:
    void restoreWindowState();
    void saveWindowState();
    void rememberNormalFrame();

    QWidget *titleBar_ = nullptr;
    // 窗口最外圈那条浅蓝描边的载体（见 FramelessWindow.cpp 里的 WindowFrame）。
    // 它必须常驻最上层：标题栏 / 状态栏都是后加的子控件，会把它压下去。
    WindowFrame *frame_ = nullptr;
    bool geometryRestored_ = false;
    // 上一次「正常态」的框。**刻意自己记**，不用 QWidget::normalGeometry()：
    // 那是 Windows 的还原矩形，客户区 = 整窗时它会随着每次最大化漂移
    // （实测 800×600 最大化一次后变成 784×584，再还原再最大化又变 768×579）。
    QRect normalFrame_;
};

#endif // FRAMELESSWINDOW_H
