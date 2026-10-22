#include "ros3dpage.h"
#include "tcpclient.h"
#include "protocolros3d.h"
#include "CollapsibleSection.h"   // 右面板折叠分组（工单 08）
#include "Theme.h"     // 状态色：setState 只写属性，颜色在全局 QSS 里定（工单 03）
#include "FloatingBar.h"     // 视口底部悬浮条（工单 10）
#include "PointsCapsule.h"   // 视口右上角点数胶囊（工单 10）
#include "PanelRail.h"       // 右面板两态容器 + 26px 窄条 + 共用把手（工单 09）

#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QGridLayout>
#include <QFrame>
#include <QPushButton>
#include <QLabel>
#include <QFileDialog>
#include <QtMath>
#include <QTimer>
#include <QCheckBox>
#include <QMessageBox>
#include <QScrollArea>
#include <QSplitter>
#include <QEvent>
#include <QResizeEvent>
#include <QSignalBlocker>

namespace {

// 键值对里的「名字」（x / y / yaw …）。字号与颜色在全局 QSS 的
// `QLabel[role="kvKey"]` 里（工单 08），这里只说「我是哪一类文字」——
// 与 #statusTitle / #note 是同一个套路，不在代码里写样式。
QLabel* makeKvKey(const QString& text, QWidget* parent)
{
    auto* lab = new QLabel(text, parent);
    lab->setProperty("role", QStringLiteral("kvKey"));
    return lab;
}

// 数值：等宽字体（QSS 里的 `QLabel[role="kvValue"]`）。10 Hz 刷新时小数位不左右跳，
// 六个读数在一列里对齐，扫一眼就能比较大小。
void markKvValue(QLabel* lab)
{
    lab->setProperty("role", QStringLiteral("kvValue"));
}

} // namespace

Ros3DPage::Ros3DPage(QWidget* parent)
    : QWidget(parent)
{
    gl_ = new GLWidget(this);

    // ---------- 视口浮层（工单 10）----------
    // 两个都是 **gl_ 的子控件**：坐标即视口坐标，换算式只面对「视口尺寸 + 控件尺寸」。
    //
    // 位置由 relayoutOverlays() 在两种时机给：本页 resize（视口跟着变）与 gl_ 自己的
    // resize（拖分隔条时**只改视口、不改本页大小**，本页的 resizeEvent 不会来）。
    // 这就是为什么除了 resizeEvent 还要盯一下 gl_ 的 resize —— 不改 glwidget.cpp
    // 的替代做法就是一个事件过滤器，它只旁听，不参与任何绘制。
    bar_     = new FloatingBar(gl_);
    capsule_ = new PointsCapsule(gl_);

    // 六个按钮走的全是 GLWidget **既有**的公开槽，一行动画/相机代码都不新增
    // （spec 实现决策 19）。表就这么一张，改按钮语义只改这里。
    connect(bar_, &FloatingBar::actionTriggered, this, [this](FloatingBar::Action action) {
        switch (action) {
        case FloatingBar::ResetView:   gl_->setCamera();                       break;
        case FloatingBar::YawLeft:     gl_->addYaw(-FloatingBar::kStepDegrees);   break;
        case FloatingBar::YawRight:    gl_->addYaw(+FloatingBar::kStepDegrees);   break;
        case FloatingBar::PitchUp:     gl_->addPitch(-FloatingBar::kStepDegrees); break;
        case FloatingBar::PitchDown:   gl_->addPitch(+FloatingBar::kStepDegrees); break;
        case FloatingBar::ClearCanvas: gl_->clearMap();                        break;
        }
    });

    gl_->installEventFilter(this);   // 只为了盯 QEvent::Resize，见 eventFilter()

    // 胶囊的宽度跟着点数走（"0" → "24.5 k"），而位置是按尺寸算的：尺寸一变就得重摆，
    // 否则它会长出右边距（甚至压到面板上）。宽度的量化在 PointsCapsule 里，
    // 这里只负责「跟着挪一下」。
    connect(capsule_, &PointsCapsule::contentSizeChanged, this, &Ros3DPage::relayoutOverlays);

    // 点数读数：每 500ms 取一次两个 getter 的快照。**没有计时埋点** ——
    // 这一块只回答「地图是不是还在长」，不显示帧率、不显示渲染耗时。
    // setCounts() 对「值没变」直接返回，所以这个定时器平时不产生任何重绘。
    pointsTimer_ = new QTimer(this);
    pointsTimer_->setInterval(500);
    connect(pointsTimer_, &QTimer::timeout, this, [this]() {
        capsule_->setCounts(gl_->realtimePointCount(), gl_->mapPointCount());
    });

    // ---------- 侧边栏 ----------
    auto* side = new QFrame(this);
    side->setFrameShape(QFrame::StyledPanel);
    // 宽度不再写死：侧边栏放在滚动区域里，宽度由外层分隔条决定（见下方主布局），
    // 这里只留一个下限，防止拖到把内容挤成一条
    side->setMinimumWidth(200);

    // ---------- 七个分组（工单 08）----------
    // 原来的 7 个 QGroupBox 换成自绘折叠分组：标题行 + 内容区，折叠时只收内容区。
    // **顺序与原功能完全一致**：机器人坐标 → 开机点 → 目标点指引 → 点云显示
    // → Z 轴范围控制 → 操作按钮 → 操作说明。一项不多、一项不少。
    // 里面的控件与信号连接一行没改，只是父对象从 QGroupBox 换成 section 的内容区；
    // 分组之间也不再另加 10px 间距 —— 分隔线已经是分组的边界，再叠一层会显得太碎。

    // ---------- 1 机器人坐标 ----------
    auto* secCoords = new CollapsibleSection(tr("机器人坐标"), side);
    labX_   = new QLabel("-", secCoords->content());
    labY_   = new QLabel("-", secCoords->content());
    labZ_   = new QLabel("-", secCoords->content());
    labYaw_ = new QLabel("-", secCoords->content());
    labPitch_ = new QLabel("-", secCoords->content());
    labRoll_ = new QLabel("-", secCoords->content());
    for (QLabel* lab : {labX_, labY_, labZ_, labYaw_, labPitch_, labRoll_})
        markKvValue(lab);

    // 三行两列：左边 xyz、右边 yaw/pitch/roll（六个数值在一屏内看得完）。
    // 键名去掉冒号、键值各占一格：原先是「x: 」这种带标点的长键名，白占宽度。
    auto* tfGrid = new QGridLayout;
    tfGrid->setHorizontalSpacing(Theme::Metric::GapSm);
    tfGrid->setVerticalSpacing(Theme::Metric::GapXs);

    tfGrid->addWidget(makeKvKey(QStringLiteral("x"), secCoords->content()),     0, 0);
    tfGrid->addWidget(labX_,   0, 1);
    tfGrid->addWidget(makeKvKey(QStringLiteral("yaw"), secCoords->content()),   0, 2);
    tfGrid->addWidget(labYaw_, 0, 3);

    tfGrid->addWidget(makeKvKey(QStringLiteral("y"), secCoords->content()),     1, 0);
    tfGrid->addWidget(labY_,   1, 1);
    tfGrid->addWidget(makeKvKey(QStringLiteral("pitch"), secCoords->content()), 1, 2);
    tfGrid->addWidget(labPitch_, 1, 3);

    tfGrid->addWidget(makeKvKey(QStringLiteral("z"), secCoords->content()),     2, 0);
    tfGrid->addWidget(labZ_,   2, 1);
    tfGrid->addWidget(makeKvKey(QStringLiteral("roll"), secCoords->content()),  2, 2);
    tfGrid->addWidget(labRoll_, 2, 3);

    // 让数值列可拉伸，保持对齐
    tfGrid->setColumnStretch(1, 1);
    tfGrid->setColumnStretch(3, 1);
    secCoords->bodyLayout()->addLayout(tfGrid);

    // ---------- 2 开机点（工单 09）----------
    // 「这一趟开机那一刻，机器人中心落在图里哪儿」——与上面「机器人坐标」是**同一类读数**，
    // 所以整块紧挨着它：操作员一眼能同时读到「机器人现在在哪」和「这次从哪开机的」。
    // （状态那一片（`secTarget`）放的是**状态**：重定位 / 规划。这三行是**数值**，
    //   两片各归各的地方，不往一起塞。）
    //
    // 平时整块 hide()：只有定位模式且 `0x02` 到过才有得显示。不画时连数值一起收起来，
    // 侧边栏里不留一块全是「-」的格子——那正好会被当成卡顿。**为什么不显示**那句话
    // 在下面「点云显示」那一块里说。
    //
    // 显形时是**展开**的：这一块平时整块收着，一旦出来就是「有数可看了」；
    // 若还停在收起态，就只剩一个标题，和「拿到后显示 x/y/yaw」自相矛盾。
    originSection_ = new CollapsibleSection(tr("开机点"), side);
    labOriginX_   = new QLabel("-", originSection_->content());
    labOriginY_   = new QLabel("-", originSection_->content());
    labOriginYaw_ = new QLabel("-", originSection_->content());
    for (QLabel* lab : {labOriginX_, labOriginY_, labOriginYaw_})
        markKvValue(lab);

    auto* originGrid = new QGridLayout;
    originGrid->setHorizontalSpacing(Theme::Metric::GapSm);
    originGrid->setVerticalSpacing(Theme::Metric::GapXs);
    originGrid->addWidget(makeKvKey(QStringLiteral("x"), originSection_->content()),   0, 0);
    originGrid->addWidget(labOriginX_,   0, 1);
    originGrid->addWidget(makeKvKey(QStringLiteral("y"), originSection_->content()),   1, 0);
    originGrid->addWidget(labOriginY_,   1, 1);
    originGrid->addWidget(makeKvKey(QStringLiteral("yaw"), originSection_->content()), 2, 0);
    originGrid->addWidget(labOriginYaw_, 2, 1);
    originGrid->setColumnStretch(1, 1);   // 数值列可拉伸，与上面那块同一个对齐习惯
    originSection_->bodyLayout()->addLayout(originGrid);
    originSection_->hide();

    // ---------- 3 目标点指引 ----------
    auto* secTarget = new CollapsibleSection(tr("目标点指引"), side);
    auto* targetBody = secTarget->content();
    auto* btnNavGoal = new QPushButton(tr("设置目标点"), targetBody);
    btnNavGoal->setCheckable(true);

    // 发送目标点按钮
    btnSendNavGoal_ = new QPushButton(tr("发送目标点"), targetBody);
    btnSendNavGoal_->setEnabled(false); // 初始状态为不可用，直到目标点确认

    spinNavX_ = new QDoubleSpinBox(targetBody);
    spinNavY_ = new QDoubleSpinBox(targetBody);
    spinNavZ_ = new QDoubleSpinBox(targetBody);
    spinNavYaw_ = new QDoubleSpinBox(targetBody);

    // 配置数值框范围和精度
    auto setupNavSpinBox = [](QDoubleSpinBox* sp, const QString& suffix, double min, double max) {
        sp->setRange(min, max);
        sp->setDecimals(2);
        sp->setSingleStep(0.1);
        sp->setSuffix(suffix);
    };
    setupNavSpinBox(spinNavX_, " m", -1000.0, 1000.0);
    setupNavSpinBox(spinNavY_, " m", -1000.0, 1000.0);
    setupNavSpinBox(spinNavZ_, " m", -100.0, 100.0);
    spinNavZ_->setValue(1.0);  // Z轴初始化为1.00m
    setupNavSpinBox(spinNavYaw_, " °", -180.0, 180.0);
    spinNavYaw_->setDecimals(0);       // 小数位数设为 0
    spinNavYaw_->setSingleStep(1.0);   // 每次点击上下箭头增减 1 度

    // 「Z = 当前机器人高度」：勾上之后 Z 由位姿帧驱动，微调框置灰 —— 两个入口同时能改
    // 同一个数，最终谁赢取决于调用顺序，那种事在界面上没法解释。所以只留一个主人。
    ckNavZFollow_ = new QCheckBox(tr("Z = 当前机器人高度"), targetBody);
    ckNavZFollow_->setToolTip(tr("勾选后目标点高度跟随机器人当前高度，Z 微调框不可手动修改"));

    // 状态帧 (0x0B)：初值必须是「未收到…」——一包都没收到既不代表「进行中」，
    // 也不代表模块明说「未响应」（那是 3；模块听到过上游自述之前一包都不发）
    labNavStatus_   = new QLabel(tr("未收到规划状态"), targetBody);
    labRelocStatus_ = new QLabel(tr("未收到重定位状态"), targetBody);

    // 帧名校验 (工单 05) 的红状态位。**不留初值文案**：它是异常留痕，
    // 正常时不占地方（下面整块 hide()），只有真出现帧名不符才显形。
    labFrameNameTitle_  = new QLabel(tr("帧名校验:"), targetBody);
    labFrameNameStatus_ = new QLabel(targetBody);

    auto* targetLay = new QGridLayout;
    targetLay->setHorizontalSpacing(Theme::Metric::GapSm);
    targetLay->setVerticalSpacing(Theme::Metric::GapSm);
    targetLay->addWidget(btnNavGoal, 0, 0, 1, 2);
    targetLay->addWidget(btnSendNavGoal_, 0, 2, 1, 2);
    targetLay->addWidget(new QLabel("X:"), 1, 0); targetLay->addWidget(spinNavX_, 1, 1);
    targetLay->addWidget(new QLabel("Y:"), 1, 2); targetLay->addWidget(spinNavY_, 1, 3);
    targetLay->addWidget(new QLabel("Z:"), 2, 0); targetLay->addWidget(spinNavZ_, 2, 1);
    targetLay->addWidget(new QLabel("Yaw:"), 2, 2); targetLay->addWidget(spinNavYaw_, 2, 3);
    // 单独占一行、跨满四列：它管的是整个 Z（微调框 + 画布取点平面），不是旁边那一格的附属品
    targetLay->addWidget(ckNavZFollow_, 3, 0, 1, 4);
    secTarget->bodyLayout()->addLayout(targetLay);

    // 勾选状态只做两件事：把 Z 框置灰、把判据交给 gl_。真正的「取哪个高度」在
    // GLWidget::setNavZFollowRobot 里（进模式 / 取点 / 回填三处都要用同一条判据，
    // 分两处各算一次迟早会不一致）。
    connect(ckNavZFollow_, &QCheckBox::toggled, this, [this](bool on) {
        spinNavZ_->setEnabled(!on);
        gl_->setNavZFollowRobot(on);
        // 勾上那一刻先把框里的数同步过去（没收到过位姿就保持原值，等位姿来了再跟）
        if (on && hasRobotZ_) {
            const QSignalBlocker blocker(spinNavZ_);   // 只改显示，不再回头喂 gl_
            spinNavZ_->setValue(robotZ_);
        }
    });

    // 状态这两行改成「字段名一行、值下一行」：一行里塞「字段名 + 值」在窄侧边栏里
    // 会被挤断或撑宽，分开后值独占一行，缩到最窄也读得全。
    auto* labNavTitle   = new QLabel(tr("规划状态:"), targetBody);
    auto* labRelocTitle = new QLabel(tr("重定位状态:"), targetBody);

    // 状态区（4 行，外加异常时才显形的帧名校验）**不进上面那个 grid**：
    // 这些标签是 wordWrap 的，QGridLayout 里一有 heightForWidth 的项，Qt5 的
    // grid 尺寸计算就会把整块的建议高度算成 0 —— 实测表现是这一组被压成一条缝、
    // 文字叠在一起。它们本来就各占一整行，用竖直布局既对，也躲开了那个坑。
    auto* statusLay = new QVBoxLayout;
    statusLay->setSpacing(Theme::Metric::GapXs);
    statusLay->addWidget(labNavTitle);
    statusLay->addWidget(labNavStatus_);
    statusLay->addWidget(labRelocTitle);
    statusLay->addWidget(labRelocStatus_);
    // 帧名校验挤在紧下面：它与上面几块状态是同一片显示区（协议文档 4.7 说的
    // 「帧名不符的红状态位可以复用同一个显示区」），只在异常时出现。
    statusLay->addWidget(labFrameNameTitle_);
    statusLay->addWidget(labFrameNameStatus_);
    secTarget->bodyLayout()->addLayout(statusLay);

    // 字段名与值分两级：只挂 objectName，样子在全局 QSS 里（QLabel#statusTitle / #statusValue）。
    // 原来这两人各带一段行内 setStyleSheet，色值散在代码里（工单 03 收敛）。
    // 值比字段名低一级：缩进 + 加粗；**颜色不在这里定**——状态色只走 Theme::setState。
    labNavTitle->setObjectName(QStringLiteral("statusTitle"));
    labRelocTitle->setObjectName(QStringLiteral("statusTitle"));
    labFrameNameTitle_->setObjectName(QStringLiteral("statusTitle"));

    labNavStatus_->setObjectName(QStringLiteral("statusValue"));
    labRelocStatus_->setObjectName(QStringLiteral("statusValue"));

    // 红字 + 加粗：状态位变红就是这条改动的「留痕」。加粗来自 #statusValue，
    // 红来自 setState("err")——与串口断线用的是同一个色值，不再各写一份。
    labFrameNameStatus_->setObjectName(QStringLiteral("statusValue"));
    Theme::setState(labFrameNameStatus_, QStringLiteral("err"));
    // 一上来就藏起来：没有帧名不符时它连同标题一行都不该出现，
    // 否则侧边栏里挂着一个空标签，会被当成「有个状态一直没数据」。
    labFrameNameTitle_->hide();
    labFrameNameStatus_->hide();

    // 侧边栏拖到最窄时值也别把整块撑宽，让它自己折行
    labNavStatus_->setWordWrap(true);
    labRelocStatus_->setWordWrap(true);
    // 帧名的值里带着两个名字，最窄时一定会换行
    labFrameNameStatus_->setWordWrap(true);

    // ---------- 4 点云显示 ----------
    auto* secCloud = new CollapsibleSection(tr("点云显示"), side);
    auto* ckRealtime = new QCheckBox(tr("实时点云"), secCloud->content());
    auto* ckMap       = new QCheckBox(tr("地图点云"), secCloud->content());
    ckRealtime->setChecked(true);
    ckMap->setChecked(true);

    secCloud->bodyLayout()->addWidget(ckRealtime);
    secCloud->bodyLayout()->addWidget(ckMap);

    // 这里**只有两个开关**：实时点云、地图点云，各自直连既有的两个显示开关。
    // 2D / 3D 是数据自己判定的（扫到 2D 就按 2D 画），不提供手动开关 ——
    // 给一个「选了也不生效」的开关，比不给更容易误导（工单 08 / spec D6）。
    //
    // 原来这里还挂着一条「2D 扫描暂不显示：位姿数据已陈旧」的说明标签（工单 07 加的），
    // 本票把它**连控件带 connect 一起删了**：它说的是内部自动判定的结果，
    // 摆到界面上只会让人以为坏了。GLWidget 侧的判断与通知保持不变，
    // 只是没有界面接收者（`scan2DNoteChanged` 现在没人接）。

    // 开机点「为什么不显示」（工单 09）：这一块管的就是「图上画什么」，
    // 所以「建图模式下不画开机点」这句说明留在这里。初值不写死文案：
    // GLWidget 推第一次就会填上（见构造函数末尾那次 publishOriginMarkerState()）——
    // **也可能一直没文案**：「还没读到过模式参数（0x13）」就不配一句话，这里保持空着、收着。
    labOriginNote_ = new QLabel(secCloud->content());
    labOriginNote_->setWordWrap(true);
    labOriginNote_->setObjectName(QStringLiteral("note"));   // 状态说明那一类文字（橘色）
    labOriginNote_->hide();
    secCloud->bodyLayout()->addWidget(labOriginNote_);

    // ---------- 5 Z 轴范围控制 ----------
    // 控件与行为一字不动（下限不越过上限、步长 0.1、手感不变），只换容器。
    // 区间约束的纯逻辑在 ui/widgets/RangeLimits.h，由 tests/test_panel.cpp 覆盖。
    auto* secZ = new CollapsibleSection(tr("Z 轴范围控制"), side);

    ckZFilter_ = new QCheckBox(tr("启用 Z 轴过滤"), secZ->content());
    ckZFilter_->setChecked(false);

    // 双滑块
    dualSlider_ = new DualRangeSlider(secZ->content());
    dualSlider_->setRange(-1.0, 5.0);  // 设置范围-1~5m
    dualSlider_->setValues(-1.0, 2.0);  // 初始值
    dualSlider_->setMinimumHeight(60);
    // 读数带上单位：两个手柄下面原先只有「-1.0」「2.0」，读不出量纲。
    // 单位属于**调用方的语义**，所以由这里给，控件本身不假设是米。
    dualSlider_->setValueSuffix(QStringLiteral(" m"));

    // 单位说明单独一行：滑块上的后缀是贴在数字后面的，字很小、挤在刻度边上，
    // 侧边栏再窄一点还会被夹到控件边界内。这一行把「米」和「这段区间管什么」说完整。
    auto* labZUnit = new QLabel(tr("单位：米（m），只显示高度在区间内的点云"), secZ->content());
    labZUnit->setWordWrap(true);
    labZUnit->setProperty("role", QStringLiteral("note"));   // 弱文字：说明而已，不是读数

    // 布局 - 只保留启用按钮和双滑块
    secZ->bodyLayout()->addWidget(ckZFilter_);
    secZ->bodyLayout()->addWidget(dualSlider_);
    secZ->bodyLayout()->addWidget(labZUnit);

    // ---------- 6 操作按钮 ----------
    auto* secActions = new CollapsibleSection(tr("操作按钮"), side);
    auto* actionsBody = secActions->content();
    auto* btnClear = new QPushButton(tr("清理地图"), actionsBody);
    auto* btnReset = new QPushButton(tr("初始化相机"), actionsBody);
    btnSave_ = new QPushButton(tr("保存地图"), actionsBody);
    btnSave_->setEnabled(false);   // 初始不可点，连上链路后由 setLinkConnected() 打开

    // 紧挨着「保存地图」：两者都是模块侧地图文件的操作，但**门禁不再共用**——
    // 存图在串口上也能做（子命令 02），下传地图只有 TCP / UDP（串口没有子命令 05，见 .h）
    btnDownload_ = new QPushButton(tr("下载地图"), actionsBody);
    btnDownload_->setEnabled(false);

    // 两列两行（原型同一个排法）：四个按钮原先是竖着一列，占了四行的高度，
    // 而它们彼此之间没有先后顺序，两列正好把这一块压掉一半。
    auto* actionsLay = new QGridLayout;
    actionsLay->setHorizontalSpacing(Theme::Metric::GapSm);
    actionsLay->setVerticalSpacing(Theme::Metric::GapSm);
    actionsLay->addWidget(btnClear,       0, 0);
    actionsLay->addWidget(btnReset,       0, 1);
    actionsLay->addWidget(btnSave_,       1, 0);
    actionsLay->addWidget(btnDownload_,   1, 1);   // 紧邻「保存地图」
    actionsLay->setColumnStretch(0, 1);
    actionsLay->setColumnStretch(1, 1);
    secActions->bodyLayout()->addLayout(actionsLay);

    // ---------- 7 操作说明 ----------
    // 字号在全局 QSS 的 QLabel#instructions 里（@font-size-hint = 11px）：
    // 原先这行写死 18px，是整块面板里最刺眼的粗陋点 —— 它是最不需要抢眼的文字。
    auto* secInst = new CollapsibleSection(tr("操作说明"), side);
    auto* labInstructions = new QLabel(
        tr("1. 左键旋转视角<br>2. 右键移动视角<br>3. 滚轮缩放"),
        secInst->content());
    labInstructions->setWordWrap(true);
    labInstructions->setObjectName(QStringLiteral("instructions"));
    secInst->bodyLayout()->addWidget(labInstructions);
    // 默认收起：它是「看过一次就不用再看」的备注，不该在默认视图里占掉三行。
    secInst->setCollapsed(true);
    // ---------- 侧边栏布局：七组，顺序即面板上的顺序 ----------
    // 分组之间不再另加 spacing：每组自带 1px 分隔线 + 底部留白，再叠 10px 会显得太碎。
    auto* sideLay = new QVBoxLayout(side);
    sideLay->setContentsMargins(0, 0, 0, 0);
    sideLay->setSpacing(0);
    sideLay->addWidget(secCoords);
    sideLay->addWidget(originSection_);   // 开机点：与「机器人坐标」同类读数，紧挨着它（工单 09）
    sideLay->addWidget(secTarget);
    sideLay->addWidget(secCloud);
    sideLay->addWidget(secZ);
    sideLay->addWidget(secActions);
    sideLay->addWidget(secInst);
    sideLay->addStretch(1);

    // ---------- 展开态：把侧边栏放入滚动区域 ----------
    // 父对象不在这里给：它马上要交给 PanelHost 的栈（两态共用同一个位置，切态只换页）。
    auto* scrollArea = new QScrollArea;
    scrollArea->setWidget(side);
    scrollArea->setWidgetResizable(true);  // 允许内容自适应宽度
    scrollArea->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);  // 禁用横向滚动
    scrollArea->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);     // 需要时显示纵向滚动
    // 展开态的下限：容得下滚动条本身。收起态与它无关 —— 那时候整页换成窄条，
    // 面板的最小宽度由 PanelHost 自己降回 26px（这就是「收起态不再被拦住」的来路）。
    scrollArea->setMinimumWidth(PanelWidth::kMinExpanded);

    // ---------- 面板两态容器（工单 09）----------
    // 展开 = 上面那个滚动区；收起 = 26px 窄条（竖排「属性面板」）。
    // 收起/展开用**同一个**把手，它挂在「同时盖住视口与面板」的这一层（本页）上 ——
    // 见 PanelHost::setToggleLayer 里的说明（Qt 会把子控件裁到父控件矩形内）。
    panelHost_ = new PanelHost(this);
    panelHost_->setExpandedContent(scrollArea);

    // ---------- 主布局 ----------
    // 左右之间用分隔条：拖它就能把「机器人坐标 / 目标点指引」这块调宽或调窄。
    // 窗口整体缩放时多出来的空间全给画布（stretch 0 = 1 / 1 = 0），侧边栏保持用户拖出来的宽度。
    splitter_ = new QSplitter(Qt::Horizontal, this);
    // 不允许拖到宽度 0：收起态给的是 26px 窄条，比 0 大，所以这条不会跟收起打架。
    splitter_->setChildrenCollapsible(false);
    // 分隔条外观在全局 QSS 的 QSplitter::handle 里（平时透明、悬停淡淡一层强调色）。
    splitter_->setHandleWidth(PanelWidth::kHandle);
    gl_->setMinimumWidth(PanelWidth::kMinViewport);   // 画布也留个下限，别让人把它拖成一条缝
    splitter_->addWidget(gl_);
    splitter_->addWidget(panelHost_);
    panelHost_->setToggleLayer(this);   // 把手挂到本页上（唯一能容下「压在视口上那半」的一层）
    splitter_->setStretchFactor(0, 1);
    splitter_->setStretchFactor(1, 0);
    // 初始：侧边栏 296（原型值），与改动前的 300 是同一个观感
    splitter_->setSizes(QList<int>()
                        << 700 << PanelWidth::kDefaultExpanded);

    // 收起/展开之后重排分裂器（把手宽度、两段宽度都在 applyPanelLayout 里定）
    connect(panelHost_, &PanelHost::collapsedChanged, this, [this](bool) { applyPanelLayout(); });

    // 用户拖过分隔条就记下宽度：下次展开回到这里，而不是回到出厂默认值。
    // 只在展开态记 —— 收起态的 26px 不是用户的意愿（PanelWidth::remember 里也挡了一道）。
    connect(splitter_, &QSplitter::splitterMoved, this, [this](int, int) {
        if (panelHost_ && !panelHost_->isCollapsed())
            panelHost_->rememberExpandedWidth(splitter_->sizes().value(1));
    });

    // setSizes 给的绝对值会被 QSplitter 按「当时还没量出来的宽度」等比缩放（1000 -> 实际宽度），
    // 所以等第一次布局跑完再钉一次 296，启动时的观感才和改动前一样；之后拖过就归用户了。
    //
    // 与收起的冲突（工单 09）在这一行解决：**只做一次**，且只在展开态做 ——
    // 收起态直接跳过，否则窄条会在启动几十毫秒后被顶回 296。
    QTimer::singleShot(0, this, [this]() {
        if (panelPinned_)
            return;
        panelPinned_ = true;

        if (!panelHost_ || !splitter_ || panelHost_->isCollapsed())
            return;

        const int panel = panelHost_->preferredWidth();
        // 还没排出来（宽度为 0）或容器太窄：维持等比那版，不硬钉
        if (!PanelWidth::canPin(splitter_->width(), panel))
            return;

        splitter_->setSizes(PanelWidth::splitSizes(splitter_->width(), panel, PanelWidth::kHandle));
    });

    // 左侧留一条页面底色（@bg）的边距：画布不再顶到窗口左缘。
    // 原先这里是 0 —— 深色视口与窗口左边框重合，那一条看起来就成了「画布自己」。
    // 取 Gap（12）与命令栏 / 状态栏自身的内边距同值，画布左缘正好压在两条栏
    // 内容的同一条竖线上；右边与上下三边不动（仍是满幅，右边由右面板接着）。
    auto* mainLay = new QHBoxLayout(this);
    mainLay->setContentsMargins(Theme::Metric::Gap, 0, 0, 0);
    mainLay->addWidget(splitter_);

    // ---------- 浮层收尾（工单 10）----------
    // 摆一次位置（此刻视口可能还没量出来，摆出来的只是初值），然后把两者抬到 GL 之上。
    // **raise() 只在这里调一次**：之后的位置变化只需 move()，不用每帧去动 z 序
    // （spec 实现决策 18）。点数定时器也在这里起步 —— 界面已经接好了。
    relayoutOverlays();
    bar_->raise();
    capsule_->raise();
    pointsTimer_->start();

    // ---------- 连接原有按钮 ----------
    connect(btnClear, &QPushButton::clicked, gl_, &GLWidget::clearMap);
    connect(btnReset, &QPushButton::clicked, gl_, &GLWidget::setCamera);
    connect(btnSave_, &QPushButton::clicked, gl_, &GLWidget::saveMapToFile);
    // 真正落盘在模块侧（AA 00 02 0A），这里只把请求交给 SerialPort，由它发帧、等回包、超时判失败
    connect(btnSave_, &QPushButton::clicked, this, [this](){ emit saveMapRequested(); });

    // 下载地图：请求同样交给 SerialPort（发帧、收分块、校验、选路径、落盘都在它那里，
    // 发送方与等待方同处一地），这里只发一个信号
    connect(btnDownload_, &QPushButton::clicked, this, [this](){ emit downloadMapRequested(); });

    // ---------- 长按连发定时器：已删除（工单 10）----------
    // 原来这里是给面板里那组「视角旋转」按钮做的连发逻辑。那组按钮已经被视口底部的
    // 悬浮条取代，而悬浮条按 spec 的定案**只做「点一下 15°」、不做连发也不做拖拽**
    // —— 连发会和「在按钮上按下再拖」抢手势，把本该交给画布的拖动吃掉。
    // 要连点几下微调，鼠标连点几下就够了。

    // ---------- TF 显示更新 ----------
    connect(gl_, &GLWidget::tfInfoChanged, this,
            [this](double x, double y, double z,
                   int yaw, int pitch, int roll){
                labX_->setText(QString::number(x, 'f', 2) + " m");
                labY_->setText(QString::number(y, 'f', 2) + " m");
                labZ_->setText(QString::number(z, 'f', 2) + " m");

                labYaw_->setText(QString::number(yaw)   + "°");
                labPitch_->setText(QString::number(pitch) + "°");
                labRoll_->setText(QString::number(roll)  + "°");

                // 「Z = 当前机器人高度」：高度就从这一路位姿帧来（与上面那个 z 是同一个数，
                // 不另开数据源）。勾着的时候把框里的显示也顶过去 —— 框已置灰，
                // 但读数必须跟着走，否则操作员看到的 Z 和真正会发出去的 Z 是两个数。
                robotZ_    = z;
                hasRobotZ_ = true;
                if (ckNavZFollow_->isChecked()) {
                    const QSignalBlocker blocker(spinNavZ_);
                    spinNavZ_->setValue(z);
                }
            });

    // 开机点（工单 09）：同上，文案与数值都由 gl_ 算好，这里只摆出来。
    // （2D 扫描「为什么不画」那条说明在工单 08 里删了：GLWidget 仍会发 scan2DNoteChanged，
    //   但界面不再有接收者 —— 那是内部自动判定的结果，摆出来只会像「界面坏了」。）
    connect(gl_, &GLWidget::originMarkerChanged,
            this, &Ros3DPage::onOriginMarkerChanged);

    // 接好之后**主动推第一次**。原因是状态可能在本页构造之前就已定型：用户先点过「读取参数」、
    // `0x02` 也到过，那次变化发生在信号接通之前，而它**不会再变**，信号又只在变化时发 ——
    // 不推这一次，界面上那三个数一整场都出不来。
    // （「没读到过模式参数」不画、也没有说明的那一种不在此列：界面初值就是收着的样子，
    //   推与不推长得一样。）
    gl_->publishOriginMarkerState();

    // 信号连接
    connect(ckRealtime, &QCheckBox::toggled, gl_, &GLWidget::setShowRealtimeCloud);
    connect(ckMap,      &QCheckBox::toggled, gl_, &GLWidget::setShowMapCloud);

    // ---------- Z轴范围控制信号连接 ----------
    // 双滑块值改变
    connect(dualSlider_, &DualRangeSlider::rangeChanged,
            this, &Ros3DPage::onDualRangeChanged);

    // 启用/禁用过滤
    connect(ckZFilter_, &QCheckBox::toggled, gl_, &GLWidget::setZFilterEnabled);

    connect(gl_, &GLWidget::navGoalSet, this, [this](double x, double y, double z, double yaw) {
        double yaw_deg = yaw * 180.0 / M_PI;
        QString msg = QString("生成目标点 -> X: %1, Y: %2, Z: %3, Yaw: %4°")
                          .arg(x, 0, 'f', 2)
                          .arg(y, 0, 'f', 2)
                          .arg(z, 0, 'f', 2)
                          .arg(yaw_deg, 0, 'f', 1);
        qDebug() << msg;

        // 目标点已设置，启用发送按钮
        btnSendNavGoal_->setEnabled(true);
    });

    // 监听 Toggle 状态改变
    connect(btnNavGoal, &QPushButton::toggled, this, [=](bool checked) {
        if (checked) {
            btnNavGoal->setText(tr("退出指点模式"));
            gl_->setNavMode(true);
            // 正在指点时，禁用发送按钮，防止误触
            btnSendNavGoal_->setEnabled(false);
        } else {
            btnNavGoal->setText(tr("设置目标点"));
            gl_->setNavMode(false);
            // 这里不需要写 setEnabled(true)，因为 gl_->setNavMode(false) 会触发 gl_ 发射 navGoalSet 信号，
            // 从而在上面的 connect 里启用发送按钮。
        }
    });

    // 【新增】连接发送按钮的点击槽函数
    connect(btnSendNavGoal_, &QPushButton::clicked, this, &Ros3DPage::onSendNavGoalClicked);


    // 接收GLWidget鼠标拖动时的坐标更新信号
    connect(gl_, &GLWidget::navTargetUpdated, this, &Ros3DPage::onNavTargetUpdated);

    // 监听UI上微调框的修改信号
    void (QDoubleSpinBox::*valueChangedSignal)(double) = &QDoubleSpinBox::valueChanged;
    connect(spinNavX_, valueChangedSignal, this, &Ros3DPage::onNavSpinBoxChanged);
    connect(spinNavY_, valueChangedSignal, this, &Ros3DPage::onNavSpinBoxChanged);
    connect(spinNavZ_, valueChangedSignal, this, &Ros3DPage::onNavSpinBoxChanged);
    connect(spinNavYaw_, valueChangedSignal, this, &Ros3DPage::onNavSpinBoxChanged);
}

// ---------------------------------------------------------------- 视口浮层（工单 10）

// 两个浮层的位置都在这里算：**不在 GLWidget 的 resizeEvent 里做**（那是渲染路径，
// 本次改造一行不动），而是在外面按视口几何换算，换算式是各自类里的纯函数。
//
// 为什么要两个入口：
//   * 本页 resize —— 窗口缩放、页切换，视口跟着变。这条也是 spec 指定的落点。
//   * gl_ 的 resize —— **拖分隔条**时视口变宽变窄，但本页尺寸不变，本页的 resizeEvent
//     根本不会来。少了这一条，拖动分隔条时悬浮条会一直停在旧的中心位置上。
void Ros3DPage::relayoutOverlays()
{
    if (!bar_ || !capsule_)
        return;

    // 父对象是 gl_，所以视口坐标系就是父控件坐标系，不需要 mapTo / geometry 偏移
    const QSize viewport = gl_->size();
    bar_->move(FloatingBar::placeRect(viewport, bar_->size()).topLeft());
    capsule_->move(PointsCapsule::placeRect(viewport, capsule_->size()).topLeft());
}

void Ros3DPage::resizeEvent(QResizeEvent *e)
{
    QWidget::resizeEvent(e);
    relayoutOverlays();
}

// ---------------------------------------------------------------- 面板两态（工单 09）

// 收起/展开之后重排分裂器。就两件事：
//
//   1) **把手宽度**：收起态必须真的收到 0，否则窄条左边会多出一条 6px 的缝（硬约束）。
//      这件事要**两处一起做**，少一处都不行（两处都实测过）：
//        a. `setHandleWidth(0)` —— QSplitter 内部按它给把手留位置；
//        b. 对把手本体 `setFixedWidth(0)` —— 只做 a 的话，全局 QSS 里
//           `QSplitter::handle:horizontal { width: 6px }` 会把把手撑回来（实测还剩 4px）。
//      只做 b 的话，QSplitter 仍按 handleWidth()=6 算可用宽度，setSizes 会被等比缩掉
//      那 6px —— 表现为窄条左边一条深色缝。展开时两处都要解回 6。
//
//   2) **两段宽度**：收起态给窄条（26），展开态给记住的那个宽度
//      （纯换算在 PanelWidth::splitSizes，控制台测试直接跑那一个）。
//
// 换算式不在这里现算：这里只负责「问状态、下命令」，所以两态之间不会出现
// 两套互相打架的宽度口径。
void Ros3DPage::applyPanelLayout()
{
    if (!splitter_ || !panelHost_)
        return;

    const bool rail = panelHost_->isCollapsed();

    if (QWidget* handle = splitter_->handle(1)) {
        if (rail) {
            handle->setFixedWidth(0);
        } else {
            handle->setMinimumWidth(0);              // 解开收起时那个 fixed
            handle->setMaximumWidth(QWIDGETSIZE_MAX);
        }
    }
    // 把手本体收成 0 还不够：QSplitter 内部仍按 handleWidth() 给把手留位置
    // （两处不对齐的表现是 setSizes 被等比缩掉那 6px，窄条左边多出一条深色缝）。
    splitter_->setHandleWidth(rail ? 0 : PanelWidth::kHandle);

    const int avail = splitter_->width();
    if (avail <= 0)
        return;   // 还没排出来：首次显示那条 singleShot 会兜住

    const int handle = rail ? 0 : PanelWidth::kHandle;
    splitter_->setSizes(PanelWidth::splitSizes(avail, panelHost_->preferredWidth(), handle));
}

// 只旁听 gl_ 的尺寸变化，其余一律原样放行 —— 这不是「改 GLWidget 的 resizeEvent」，
// GLWidget 的实现文件一个字节都不动，事件照旧先到它自己手里。
bool Ros3DPage::eventFilter(QObject *watched, QEvent *e)
{
    if (watched == gl_ && e->type() == QEvent::Resize)
        relayoutOverlays();
    return QWidget::eventFilter(watched, e);
}

// 新增槽函数
void Ros3DPage::onDualRangeChanged(double lower, double upper)
{
    // 更新GLWidget
    gl_->setZFilterRange(lower, upper);
}


void Ros3DPage::onNavTargetUpdated(double x, double y, double z, double yaw_deg)
{
    // 收到画布上的点位置，更新UI，但屏蔽信号避免循环触发
    spinNavX_->blockSignals(true);
    spinNavY_->blockSignals(true);
    spinNavZ_->blockSignals(true);
    spinNavYaw_->blockSignals(true);

    spinNavX_->setValue(x);
    spinNavY_->setValue(y);
    spinNavZ_->setValue(z);
    spinNavYaw_->setValue(yaw_deg);

    spinNavX_->blockSignals(false);
    spinNavY_->blockSignals(false);
    spinNavZ_->blockSignals(false);
    spinNavYaw_->blockSignals(false);
}

void Ros3DPage::onNavSpinBoxChanged()
{
    // UI框数值发生变动时，把新值喂给 GL 画布
    gl_->updateNavTargetFromUI(
        spinNavX_->value(),
        spinNavY_->value(),
        spinNavZ_->value(),
        spinNavYaw_->value()
        );
}


// 指令链路状态（TCP / UDP / 串口）：未连接时「保存地图」不可点。
// 串口自协议 v2.9 起也能存图（命令帧 AA 00 02 …，子命令号与 TCP 相同），
// 所以这里不再要求「必须是 TCP / UDP」，只要求「有一条能发指令的链路」。
void Ros3DPage::setLinkConnected(bool connected)
{
    if (btnSave_) btnSave_->setEnabled(connected);
}

// 「下载地图」是唯一仍然只认 TCP / UDP 的那个：串口命令面是五条子命令，**没有 05**。
// 两条门禁分开的代价是多一个信号，但不分的话，串口模式下这颗键会亮着 ——
// 点下去只能白等一个超时（模块对 05 静默丢弃，一帧都不回）。
void Ros3DPage::setDownloadAvailable(bool available)
{
    if (btnDownload_) btnDownload_->setEnabled(available);
}


// 模块重启判定成功后清掉本页的「模块运行态镜像」（工单 02）。这些标签与锁存描述的都是
// 重启前那个模块，模块回来之前它们一律不该继续显示旧值 —— 那会让人以为数据还是新鲜的。
// 入口只有一个：SerialPort::restartSucceeded。超时失败路径不会走到这里（模块很可能
// 根本没重启，界面上的数据依然有效）。
void Ros3DPage::clearModuleState()
{
    // 1. 3D 视图那一片：复用「清理地图」按钮的同一个入口，一次调用搞定 —— 点云、体素地图、
    //    2D 地图、2D 扫描（激光）、最优轨迹、机器人位姿、重定位结果、运动拖尾，以及指点模式的
    //    目标点几何（目标点本身、朝向、hasNavTarget_ / isNavGoalSet_ 两个标志）都在里头。
    //    「2D 扫描为什么不画」那句话（工单 07）也随它一起走：gl_ 复位状态时会发一个空文案，
    //    上面那个连接会把标签收起来，这里不必再碰一次。
    //    「开机点」那一块（工单 09）同理：`0x02` 那条边被抹掉之后 gl_ 会立刻重推状态，
    //    数值与说明一起跟着走，这里也不必再碰一次。
    gl_->clearMap();

    // 2. 上面那个入口碰不到的：TF 数值标签回到构造时给的占位「-」。
    //    不回位就会永远显示重启前最后一帧位姿 —— 那个值此刻已经无处可查，也就无从证伪。
    QLabel* tfLabels[] = {labX_, labY_, labZ_, labYaw_, labPitch_, labRoll_};
    for (QLabel* lab : tfLabels) {
        lab->setText(QStringLiteral("-"));
    }

    // 「Z = 当前机器人高度」那份高度缓存同理作废：gl_ 那边 tf_.clear() 之后已经取不到高度了，
    // 这里若还留着上一趟的数，勾选项看着像有来源，其实是个无处可查的旧值。
    hasRobotZ_ = false;
    robotZ_    = 0.0;

    // 3. 规划 / 重定位状态标签回到初始文案：它们显示的是「一包都没收到」，
    //    正是重启后的真实状态（模块要等听到上游自述才发状态帧）。
    //    「开机点」那几个数不用在这里碰（工单 09）：上面 gl_->clearMap() 抹掉 `0x02`
    //    那条边之后 gl_ 会立刻重推状态，整块就收起来了，不会留上一趟的数。
    labNavStatus_->setText(tr("未收到规划状态"));
    labRelocStatus_->setText(tr("未收到重定位状态"));

    // 4. 跳变锁存同时复位：-1 = 「这一路还没收到过任何一包」。三件（飞控 / 重定位 / 规划）
    //    都在内 —— 飞控不占格子，但它也有锁存，漏掉它那一件重启后就再也写不出日志。
    //    锁存不清的话，模块重启后重新上报的第一帧若与重启前的旧值相等，会被当成「没变化」
    //    静默掉，日志里永远看不到「首次收到」，现场也就没法判断模块到底回来没有。
    lastNavState_   = -1;
    lastRelocState_ = -1;
    lastFcState_    = -1;

    // 5. 帧名不符的红状态位同理：它说的是**重启前那个模块**的帧名对不上，
    //    新模块回来之前继续挂着，就等于拿旧结论给新模块定罪。锁存一起清掉，
    //    这样新模块若仍不符，会重新亮红并再写一行日志（而不是被当成「没变化」静默掉）。
    labFrameNameTitle_->hide();
    labFrameNameStatus_->clear();
    labFrameNameStatus_->hide();
    lastFrameNameMismatch_.clear();
}


// 规划四档 → 中文。`3` 是模块的一条明确说法「我没听到过规划器的自述」，
// 不是「未知」也不是「规划中」：模块听到过任一上游（含飞控）就会开始发这一帧，
// 所以 3 很常见，不能让它落进「未知状态」。
QString Ros3DPage::navStatusText(int state)
{
    switch (state) {
    case 0:  return tr("规划中");
    case 1:  return tr("规划成功");
    case 2:  return tr("规划出错");
    case 3:  return tr("未响应");
    default: return tr("未知状态(%1)").arg(state);
    }
}

// 重定位四档 → 中文，与规划那四档同义（0 进行中 / 1 成功 / 2 失败 / 3 未响应）。
// 「未响应」是模块说的话（它没听到过重定位上游）；「一包都没收到」由 lastRelocState_ < 0
// 表示，标签停在「未收到重定位状态」——链路问题与模块问题必须显示得不一样。
QString Ros3DPage::relocStatusText(int state)
{
    switch (state) {
    case 0:  return tr("进行中");
    case 1:  return tr("成功");
    case 2:  return tr("失败");
    case 3:  return tr("未响应");
    default: return tr("未知状态(%1)").arg(state);
    }
}

// 飞控五档 → 中文（与串口 `0x05` 的 Byte 2 同一套取值，见协议文档那张枚举表）。
// 只进日志、不占格子：串口链已有飞控状态显示，两条链如何合并显示是独立话题。
QString Ros3DPage::fcStatusText(int state)
{
    switch (state) {
    case 0:  return tr("未连接");
    case 1:  return tr("未解锁");
    case 2:  return tr("已解锁地面");
    case 3:  return tr("空中悬停");
    case 4:  return tr("执行航点");
    default: return tr("未知状态(%1)").arg(state);
    }
}

// 状态帧 (0x0B)：飞控 + 重定位 + 路径规划
// 三件都是「当前值」不是「事件」：模块一旦听到过任一上游自述就 10 Hz 重发同一组值，
// 所以每一件都只在相邻两包发生跳变时才动标签，日志窗口写不下这种 10 Hz 的流水。
void Ros3DPage::onNavStatusUpdated(const NavStatusMsg& msg)
{
    // 一件状态的跳变处理：没跳变就什么都不做；跳变了刷新对应的格子（label 为空的没有格子）。
    const auto applyJump = [](int& last, int value, QLabel* label, QString (*textOf)(int)) {
        if (value == last) return;

        last = value;
        if (label) label->setText(textOf(value));
    };

    applyJump(lastRelocState_, msg.relocalization, labRelocStatus_, &relocStatusText);
    applyJump(lastNavState_, msg.planning, labNavStatus_, &navStatusText);
    // 飞控不占格子：只在锁存里记着（串口链已有飞控状态显示，合并显示是独立话题）
    applyJump(lastFcState_, msg.fcState, nullptr, &fcStatusText);
}

// 帧名不符 (工单 05)：解析层已经把该帧丢了，这里只负责在侧边栏留痕。
//
// **不弹窗、不阻塞**——这是刻意的：两端不同版时操作员该继续能操作，看见红字就知道
// 对不上，而不是被一个对话框拦在路上。也**不把期望的帧名写出来**：期望值只有一份
// （tftree.cpp 那张对照表），界面上写「应该是 base_init_map」就等于把它抄成了第二份，
// 将来改协议又会多出一处能跟它不一致的地方。这里只显示**载荷里读到的**名字。
void Ros3DPage::onFrameNameRejected(quint8 frameNo, const QString& frameId, const QString& childFrameId)
{
    // 两位十六进制，与协议文档里 `0x01` / `0x0B` 的写法一致
    QString text = QStringLiteral("0x%1 帧名不符：%2")
                       .arg(frameNo, 2, 16, QLatin1Char('0'))
                       .arg(frameId);
    // 只有父系那一侧的帧（0x03 那种载荷里只有一个帧名字段）不显示那个空箭头
    if (!childFrameId.isEmpty())
        text += QStringLiteral(" → %1").arg(childFrameId);

    // 不符的帧按 10 Hz 以上重发，同一个不符每帧都刷一遍都是白做：
    // 只在文案**变化时**动标签（与上面状态帧「只在跳变时动」同一条规矩）。
    if (text == lastFrameNameMismatch_)
        return;
    lastFrameNameMismatch_ = text;

    labFrameNameStatus_->setText(text);
    labFrameNameTitle_->show();
    labFrameNameStatus_->show();
}

// 开机点（工单 09）。文案与数值都由 GLWidget::publishOriginMarkerState() 算好
// （判据**唯一**：`0x13` 说没说定位 + `0x02` 那条边到没到），这里只摆出来。
//
// `shown` = 画着：把三个数摆上、那句说明收掉。
// 不画：**整块数值收起来**（`hide()`），不留一行占位 —— 「建图模式下不绘制该标记、
// 也没有残留数值」这条因此是结构上成立的：数值跟着整块一起消失，不靠谁记得去清。
// 为什么不显示那句话摆到「点云显示」那一块去；
// 而「本会话还没读到过模式参数（0x13）」时那句话也是空的，两处都不显 —— 一趟还没开张，
// 没有变故要说，界面上保持构造时的样子就是对的。
//
// **不写日志**：模式是用户点「读取参数」时才变的低频事，但
// 「还没收到过 0x02」在收到第一帧前一直成立，逐次记日志没有意义；它描述的是**当前状态**。
void Ros3DPage::onOriginMarkerChanged(bool shown, const QString &note, double x, double y, double yawDeg)
{
    if (shown) {
        // 数值口径与「机器人坐标」那一组一致（两位小数 + 米 / 整数度）：
        // 两块读数就在上下相邻的位置，写法不统一会显得是两个来源的数。
        labOriginX_->setText(QString::number(x, 'f', 2) + " m");
        labOriginY_->setText(QString::number(y, 'f', 2) + " m");
        labOriginYaw_->setText(QString::number(yawDeg, 'f', 0) + "°");

        labOriginNote_->clear();
        labOriginNote_->hide();
        // 显形时**顺带展开**：用户的折叠选择在本次运行内算数，但「整块本来收着、
        // 刚冒出来」那一刻只剩标题的话，等于让人再点一次才看得到数（工单 08）。
        if (originSection_->isCollapsed())
            originSection_->setCollapsed(false);
        originSection_->show();
        return;
    }

    originSection_->hide();

    // 空 `note` = 「没话说」（还没读到过模式参数）：只把整块收起来，不留一句话。
    // 与上面「画着」那一路一样要走 clear()：'0x13' 从读出过到被忘掉（模块重启 / 重新连接）
    // 中间可能隔着「建图」「还没收到 0x02」两句话，不清就会留着上一句。
    if (note.isEmpty()) {
        labOriginNote_->clear();
        labOriginNote_->hide();
        return;
    }

    labOriginNote_->setText(note);
    labOriginNote_->show();
}

void Ros3DPage::onSendNavGoalClicked()
{
    // TcpClient* tcpClient = TcpClient::getInstance();

    // if (!tcpClient->isConnected()) {
    //     QMessageBox::warning(this, "错误", "请先建立TCP连接");
    //     return;
    // }

    // 从界面微调框获取坐标和角度
    double x = spinNavX_->value();
    double y = spinNavY_->value();
    double z = spinNavZ_->value();
    double yaw_deg = spinNavYaw_->value();

    // 触发信号，将数据交给专门处理通信的模块
    emit sendNavGoalRequested(x, y, z, yaw_deg);

    // (可选) 打印日志
    qDebug() << "界面触发发送导航目标点请求...";

    // 将角度转换为 ego-planner 需要的弧度制
    // double yaw_rad = yaw_deg * M_PI / 180.0;

    // // 构建下发给 ego-planner 的目标点帧: AA 04 [JSON] 0A
    // QByteArray frame = ProtocolRos3D::buildNavGoalFrame(x, y, z, yaw_rad);

    // 发送数据
    // tcpClient->sendData(frame);

    // qDebug() << "发送导航目标点请求，数据:" << frame.toHex(' ');

    // 如果你在界面底部有日志栏，可以打印相关信息
    // QString timestamp = QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss >> 用户操作: ");
    // emit appendMessage(timestamp + QString("发送目标点 (X:%1, Y:%2, Z:%3, Yaw:%4°)").arg(x).arg(y).arg(z).arg(yaw_deg));
}
