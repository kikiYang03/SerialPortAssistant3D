#ifndef FRAMELESSHELPER_H
#define FRAMELESSHELPER_H

class QWidget;

// 无边框窗口的 Win32 细节全部在这个实现文件里：只有 FramelessHelper.cpp
// 会 include <windows.h>，其余任何地方都不该跟着被卷进去。
//
// ---- 公开面（调用方只该用这三个） ------------------------------------------
//   install()          挂上：接管 WM_NCCALCSIZE / WM_NCHITTEST /
//                      WM_NCLBUTTONDOWN / WM_GETMINMAXINFO
//   setTitleBar()      指定标题栏控件，命中测试要知道「哪一块能拖、哪个是最大化键」
//   setResizeBorder()  边缘缩放命中带宽度（逻辑像素），默认 8
//
// ---- 窗口与实现之间的管道（只有 FramelessWindow 用） ----------------------
//   handleNativeEvent()  FramelessWindow::nativeEvent 转发进来
//   usesDwmRounding()    Win11 系统圆角是否真的生效（没生效就说明是 Win10，
//                        窗口要自己画那条 1px 描边）
//
// 换实现（例如将来引入 QWindowKit）只替换 FramelessHelper.cpp，上面的签名保持不变，
// 调用方零改动。
namespace FramelessHelper {

// 标题栏上的控件用这个动态属性告诉命中测试「我是什么」：
//   Qt 属性名 framelessHit，取值：
//     "maxbutton" → 返回 HTMAXBUTTON（Win11 悬停最大化键会弹 Snap 布局；Win10 忽略该值）
//     "client"    → 返回 HTCLIENT（点击照常落到 Qt 控件上）
//   没打属性的 QAbstractButton 按 "client" 处理；其余控件一律当成可拖动区（HTCAPTION）。
const char *const kHitProperty  = "framelessHit";
const char *const kHitMaxButton = "maxbutton";
const char *const kHitClient    = "client";

void install(QWidget *window);
void setTitleBar(QWidget *window, QWidget *titleBar);
void setResizeBorder(QWidget *window, int px);

bool handleNativeEvent(QWidget *window, void *message, long *result);
bool usesDwmRounding(const QWidget *window);

} // namespace FramelessHelper

#endif // FRAMELESSHELPER_H
