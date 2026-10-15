#include "ros3dpage.h"
#include "tcpclient.h"
#include "protocolros3d.h"

#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QFormLayout>
#include <QFrame>
#include <QGroupBox>
#include <QPushButton>
#include <QLabel>
#include <QFileDialog>
#include <QtMath>
#include <QTimer>
#include <QCheckBox>
#include <QMessageBox>
#include <QScrollArea>
#include <QSplitter>

Ros3DPage::Ros3DPage(QWidget* parent)
    : QWidget(parent)
{
    gl_ = new GLWidget(this);

    // ---------- 侧边栏 ----------
    auto* side = new QFrame(this);
    side->setFrameShape(QFrame::StyledPanel);
    // 宽度不再写死：侧边栏放在滚动区域里，宽度由外层分隔条决定（见下方主布局），
    // 这里只留一个下限，防止拖到把内容挤成一条
    side->setMinimumWidth(200);

    // TF group
    auto* tfBox = new QGroupBox(tr("机器人坐标"), side);
    labX_   = new QLabel("-", tfBox);
    labY_   = new QLabel("-", tfBox);
    labZ_   = new QLabel("-", tfBox);
    labYaw_ = new QLabel("-", tfBox);
    labPitch_ = new QLabel("-", tfBox);
    labRoll_ = new QLabel("-", tfBox);

    // 三行两列：左边 xyz，右边 yaw/pitch/roll
    auto* tfGrid = new QGridLayout(tfBox);

    // 第 0 行
    tfGrid->addWidget(new QLabel("x: "),     0, 0);
    tfGrid->addWidget(labX_,              0, 1);
    tfGrid->addWidget(new QLabel("yaw: "),  0, 2);
    tfGrid->addWidget(labYaw_,            0, 3);

    // 第 1 行
    tfGrid->addWidget(new QLabel("y: "),     1, 0);
    tfGrid->addWidget(labY_,              1, 1);
    tfGrid->addWidget(new QLabel("pitch: "), 1, 2);
    tfGrid->addWidget(labPitch_,          1, 3);

    // 第 2 行
    tfGrid->addWidget(new QLabel("z: "),     2, 0);
    tfGrid->addWidget(labZ_,              2, 1);
    tfGrid->addWidget(new QLabel("roll: "),  2, 2);
    tfGrid->addWidget(labRoll_,           2, 3);

    // 让数值列可拉伸，保持对齐
    tfGrid->setColumnStretch(1, 1);
    tfGrid->setColumnStretch(3, 1);

    // 操作按钮栏
    auto* controlBox = new QGroupBox(tr("操作按钮"), side);
    auto* btnClear = new QPushButton(tr("清理地图"), controlBox);
    auto* btnReset = new QPushButton(tr("初始化相机"), controlBox);
    btnSave_ = new QPushButton(tr("保存地图"), controlBox);
    btnSave_->setEnabled(false);   // 初始不可点，连上链路后由 setLinkConnected() 打开

    // 紧挨着「保存地图」：两者都是模块侧地图文件的操作，门禁也共用同一条
    btnDownload_ = new QPushButton(tr("下载地图"), controlBox);
    btnDownload_->setEnabled(false);

    // ---------- 目标点设置 (新增面板) ----------
    auto* targetBox = new QGroupBox(tr("目标点指引"), side);
    auto* btnNavGoal = new QPushButton(tr("设置目标点"), targetBox);
    btnNavGoal->setCheckable(true);

    // 发送目标点按钮
    btnSendNavGoal_ = new QPushButton(tr("发送目标点"), targetBox);
    btnSendNavGoal_->setEnabled(false); // 初始状态为不可用，直到目标点确认

    spinNavX_ = new QDoubleSpinBox(targetBox);
    spinNavY_ = new QDoubleSpinBox(targetBox);
    spinNavZ_ = new QDoubleSpinBox(targetBox);
    spinNavYaw_ = new QDoubleSpinBox(targetBox);

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

    // 状态帧 (0x0B)：初值必须是「未收到…」——一包都没收到既不代表「进行中」，
    // 也不代表模块明说「未响应」（那是 3；模块听到过上游自述之前一包都不发）
    labNavStatus_   = new QLabel(tr("未收到规划状态"), targetBox);
    labRelocStatus_ = new QLabel(tr("未收到重定位状态"), targetBox);

    auto* targetLay = new QGridLayout(targetBox);
    targetLay->addWidget(btnNavGoal, 0, 0, 1, 2);
    targetLay->addWidget(btnSendNavGoal_, 0, 2, 1, 2);
    targetLay->addWidget(new QLabel("X:"), 1, 0); targetLay->addWidget(spinNavX_, 1, 1);
    targetLay->addWidget(new QLabel("Y:"), 1, 2); targetLay->addWidget(spinNavY_, 1, 3);
    targetLay->addWidget(new QLabel("Z:"), 2, 0); targetLay->addWidget(spinNavZ_, 2, 1);
    targetLay->addWidget(new QLabel("Yaw:"), 2, 2); targetLay->addWidget(spinNavYaw_, 2, 3);
    // 状态这两行改成「字段名一行、值下一行」：一行里塞「字段名 + 值」在窄侧边栏里
    // 会被挤断或撑宽，分开后值独占一行，缩到最窄也读得全。名字跨满四列，值缩进对齐。
    auto* labNavTitle   = new QLabel(tr("规划状态:"), targetBox);
    auto* labRelocTitle = new QLabel(tr("重定位状态:"), targetBox);

    targetLay->addWidget(labNavTitle, 3, 0, 1, 4);
    targetLay->addWidget(labNavStatus_, 4, 0, 1, 4);
    targetLay->addWidget(labRelocTitle, 5, 0, 1, 4);
    targetLay->addWidget(labRelocStatus_, 6, 0, 1, 4);

    // 值比字段名低一级：缩进一点，和上面的名字错开
    const QString statusTitleStyle = "QLabel { color: #666666; }";
    const QString statusValueStyle = "QLabel { padding-left: 12px; font-weight: bold; }";
    labNavTitle->setStyleSheet(statusTitleStyle);
    labRelocTitle->setStyleSheet(statusTitleStyle);
    labNavStatus_->setStyleSheet(statusValueStyle);
    labRelocStatus_->setStyleSheet(statusValueStyle);

    // 侧边栏拖到最窄时值也别把整块撑宽，让它自己折行
    labNavStatus_->setWordWrap(true);
    labRelocStatus_->setWordWrap(true);

    auto* controlLay = new QVBoxLayout(controlBox);
    controlLay->addWidget(btnClear);
    controlLay->addWidget(btnReset);
    controlLay->addWidget(btnSave_);
    controlLay->addWidget(btnDownload_);   // 紧邻「保存地图」
    controlLay->addStretch();   // 把按钮顶到上面

    // 新增：视角旋转按钮组
    // auto* rotBox = new QGroupBox(tr("视角旋转"), side);
    // auto* yawLeft   = new QPushButton(tr("左转 ◀"), rotBox);
    // auto* yawRight  = new QPushButton(tr("右转 ▶"), rotBox);
    // auto* pitchUp   = new QPushButton(tr("上仰 ▲"), rotBox);
    // auto* pitchDown = new QPushButton(tr("下俯 ▼"), rotBox);

    // auto* rotLay = new QGridLayout(rotBox);
    // rotLay->addWidget(pitchUp,   0, 0, 1, 2);
    // rotLay->addWidget(yawLeft,   1, 0);
    // rotLay->addWidget(yawRight,  1, 1);
    // rotLay->addWidget(pitchDown, 2, 0, 1, 2);

    // ---------- 新增：点云显隐开关 ----------
    auto* cloudBox = new QGroupBox(tr("点云显示"), side);
    auto* ckRealtime = new QCheckBox(tr("实时点云"), cloudBox);
    auto* ckMap       = new QCheckBox(tr("地图点云"), cloudBox);
    ckRealtime->setChecked(true);
    ckMap->setChecked(true);

    auto* cloudLay = new QVBoxLayout(cloudBox);
    cloudLay->addWidget(ckRealtime);
    cloudLay->addWidget(ckMap);

    // ---------- 修改：Z轴范围控制 ----------
    auto* zFilterBox = new QGroupBox(tr("Z轴范围控制"), side);

    ckZFilter_ = new QCheckBox(tr("启用Z轴过滤"), zFilterBox);
    ckZFilter_->setChecked(false);

    // 双滑块
    dualSlider_ = new DualRangeSlider(zFilterBox);
    dualSlider_->setRange(-1.0, 5.0);  // 设置范围-1~5m
    dualSlider_->setValues(-1.0, 2.0);  // 初始值
    dualSlider_->setMinimumHeight(60);

    // 布局 - 只保留启用按钮和双滑块
    auto* zFilterLay = new QVBoxLayout(zFilterBox);
    zFilterLay->addWidget(ckZFilter_);
    zFilterLay->addWidget(dualSlider_);
    zFilterLay->addStretch();
    // ---------- 操作说明 ----------
    auto* instBox = new QGroupBox(tr("操作说明"), side);
    auto* labInstructions = new QLabel(
        tr("1. 左键旋转视角<br>2. 右键移动视角<br>3. 滚轮缩放"),
        instBox);
    labInstructions->setWordWrap(true);
    labInstructions->setStyleSheet("QLabel { font-size: 18px; }");

    auto* instLay = new QVBoxLayout(instBox);
    instLay->addWidget(labInstructions);
    instLay->addStretch();


    // 更新侧边栏布局，将zFilterBox添加到cloudBox之后：
    auto* sideLay = new QVBoxLayout(side);
    sideLay->addWidget(tfBox);
    sideLay->addSpacing(10);
    sideLay->addWidget(targetBox);   // <--- 加入目标面板
    sideLay->addSpacing(10);
    // sideLay->addWidget(rotBox);
    // sideLay->addSpacing(10);
    sideLay->addWidget(cloudBox);
    sideLay->addSpacing(10);
    sideLay->addWidget(zFilterBox);  // 新增的Z轴范围控制
    sideLay->addSpacing(10);
    sideLay->addWidget(controlBox);
    sideLay->addSpacing(10);
    sideLay->addWidget(instBox);
    sideLay->addSpacing(10);
    sideLay->addStretch(1);

    // ---------- 将侧边栏放入滚动区域 ----------
    auto* scrollArea = new QScrollArea(this);
    scrollArea->setWidget(side);
    scrollArea->setWidgetResizable(true);  // 允许内容自适应宽度
    scrollArea->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);  // 禁用横向滚动
    scrollArea->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);     // 需要时显示纵向滚动
    // 不再写死宽度：初始宽度由分隔条的 setSizes 给，之后由用户拖；下限要容下滚动条本身
    scrollArea->setMinimumWidth(240);

    // ---------- 主布局 ----------
    // 左右之间用分隔条：拖它就能把「机器人坐标 / 目标点指引」这块调宽或调窄。
    // 窗口整体缩放时多出来的空间全给画布（stretch 0 = 1 / 1 = 0），侧边栏保持用户拖出来的宽度。
    auto* splitter = new QSplitter(Qt::Horizontal, this);
    splitter->setChildrenCollapsible(false);   // 不允许把任一侧拖到完全消失
    splitter->setHandleWidth(6);
    splitter->setStyleSheet(
        "QSplitter::handle:horizontal { background-color: #c8c8c8; }"
        "QSplitter::handle:horizontal:hover { background-color: #8ab4f8; }");
    gl_->setMinimumWidth(240);   // 画布也留个下限，别让人把它拖成一条缝
    splitter->addWidget(gl_);
    splitter->addWidget(scrollArea);
    splitter->setStretchFactor(0, 1);
    splitter->setStretchFactor(1, 0);
    splitter->setSizes(QList<int>() << 700 << 300);   // 初始：侧边栏约 300，与改动前一致

    // setSizes 给的绝对值会被 QSplitter 按「当时还没量出来的宽度」等比缩放（1000 -> 实际宽度），
    // 所以等第一次布局跑完再钉一次 300，启动时的观感才和改动前一样；之后拖过就归用户了。
    QTimer::singleShot(0, splitter, [splitter]() {
        if (splitter->width() <= 340) return;   // 还没排出来（宽度为 0 或太窄），就维持等比那版
        splitter->setSizes(QList<int>() << splitter->width() - 300 << 300);
    });

    auto* mainLay = new QHBoxLayout(this);
    mainLay->setContentsMargins(0,0,0,0);
    mainLay->addWidget(splitter);

    // ---------- 连接原有按钮 ----------
    connect(btnClear, &QPushButton::clicked, gl_, &GLWidget::clearMap);
    connect(btnReset, &QPushButton::clicked, gl_, &GLWidget::setCamera);
    connect(btnSave_, &QPushButton::clicked, gl_, &GLWidget::saveMapToFile);
    // 真正落盘在模块侧（AA 00 02 0A），这里只把请求交给 SerialPort，由它发帧、等回包、超时判失败
    connect(btnSave_, &QPushButton::clicked, this, [this](){ emit saveMapRequested(); });

    // 下载地图：请求同样交给 SerialPort（发帧、收分块、校验、选路径、落盘都在它那里，
    // 发送方与等待方同处一地），这里只发一个信号
    connect(btnDownload_, &QPushButton::clicked, this, [this](){ emit downloadMapRequested(); });

    // ---------- 长按连发定时器 ----------
    // auto* repeatTimer = new QTimer(this);
    // repeatTimer->setInterval(200);          // 200 ms 连发间隔
    // repeatTimer->setSingleShot(false);

    // auto connectRepeat = [&](QPushButton* btn, int yawDelta, int pitchDelta){
    //     connect(btn, &QPushButton::pressed, this, [=](){
    //         gl_->addYaw(yawDelta);
    //         gl_->addPitch(pitchDelta);
    //         repeatTimer->disconnect();
    //         connect(repeatTimer, &QTimer::timeout, this, [=](){
    //             gl_->addYaw(yawDelta);
    //             gl_->addPitch(pitchDelta);
    //         });
    //         repeatTimer->start();
    //     });
    //     connect(btn, &QPushButton::released, this, [repeatTimer](){
    //         repeatTimer->stop();
    //     });
    // };

    // connectRepeat(yawLeft,   -5, 0);
    // connectRepeat(yawRight,  +5, 0);
    // connectRepeat(pitchUp,    0, -5);
    // connectRepeat(pitchDown,  0, +5);

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
            });

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


// 控制指令链路状态（TCP / UDP）：未连接时「保存地图」与「下载地图」都不可点
// （两个按钮共用同一条门禁：控制指令只走 TCP / UDP，串口链不承载这一类消息）
void Ros3DPage::setLinkConnected(bool connected)
{
    if (btnSave_) btnSave_->setEnabled(connected);
    if (btnDownload_) btnDownload_->setEnabled(connected);
}


// 模块重启判定成功后清掉本页的「模块运行态镜像」（工单 02）。这些标签与锁存描述的都是
// 重启前那个模块，模块回来之前它们一律不该继续显示旧值 —— 那会让人以为数据还是新鲜的。
// 入口只有一个：SerialPort::restartSucceeded。超时失败路径不会走到这里（模块很可能
// 根本没重启，界面上的数据依然有效）。
void Ros3DPage::clearModuleState()
{
    // 1. 3D 视图那一片：复用「清理地图」按钮的同一个入口，一次调用搞定 —— 点云、体素地图、
    //    2D 地图、2D 扫描（激光）、最优轨迹、机器人位姿、雷达位姿、运动拖尾，以及指点模式的
    //    目标点几何（目标点本身、朝向、hasNavTarget_ / isNavGoalSet_ 两个标志）都在里头。
    gl_->clearMap();

    // 2. 上面那个入口碰不到的：TF 数值标签回到构造时给的占位「-」。
    //    不回位就会永远显示重启前最后一帧位姿 —— 那个值此刻已经无处可查，也就无从证伪。
    QLabel* tfLabels[] = {labX_, labY_, labZ_, labYaw_, labPitch_, labRoll_};
    for (QLabel* lab : tfLabels) {
        lab->setText(QStringLiteral("-"));
    }

    // 3. 规划 / 重定位状态标签回到初始文案：它们显示的是「一包都没收到」，
    //    正是重启后的真实状态（模块要等听到上游自述才发状态帧）。
    labNavStatus_->setText(tr("未收到规划状态"));
    labRelocStatus_->setText(tr("未收到重定位状态"));

    // 4. 跳变锁存同时复位：-1 = 「这一路还没收到过任何一包」。三件（飞控 / 重定位 / 规划）
    //    都在内 —— 飞控不占格子，但它也有锁存，漏掉它那一件重启后就再也写不出日志。
    //    锁存不清的话，模块重启后重新上报的第一帧若与重启前的旧值相等，会被当成「没变化」
    //    静默掉，日志里永远看不到「首次收到」，现场也就没法判断模块到底回来没有。
    lastNavState_   = -1;
    lastRelocState_ = -1;
    lastFcState_    = -1;

    // 5. 位置控制指令 (0x08) 的「首次收到」锁存同理：模块回来后那一路要能重新写一行。
    posCmdSeen_ = false;
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
// 所以每一件都只在相邻两包发生跳变时才动标签 / 写日志，日志不会按 10 Hz 刷屏。
void Ros3DPage::onNavStatusUpdated(const NavStatusMsg& msg)
{
    // 一件状态的跳变处理：没跳变就什么都不做；跳变了写一行日志，
    // 有格子的（label 非空）顺便刷标签。last < 0 表示此前一包都没收到过。
    const auto applyJump = [this](const QString& name, int& last, int value,
                                  QLabel* label, QString (*textOf)(int)) {
        if (value == last) return;

        const QString text = textOf(value);
        emit appendMessage(last < 0
                               ? QStringLiteral("%1: 首次收到 %2").arg(name, text)
                               : QStringLiteral("%1: %2 -> %3")
                                     .arg(name, textOf(last), text));

        last = value;
        if (label) label->setText(text);
    };

    applyJump(QStringLiteral("重定位状态"), lastRelocState_, msg.relocalization,
              labRelocStatus_, &relocStatusText);
    applyJump(QStringLiteral("规划状态"), lastNavState_, msg.planning,
              labNavStatus_, &navStatusText);
    // 飞控不占格子：只解析、只写日志（串口链已有飞控状态显示，合并显示是独立话题）
    applyJump(QStringLiteral("飞控状态"), lastFcState_, msg.fcState,
              nullptr, &fcStatusText);
}

// 位置控制指令 (0x08)
// 模块 100 Hz 逐帧下发期望位置，值每帧都在动，逐帧写日志会把接收窗口淹掉；
// 数值日志由解析端按秒节流，这里只在首次收到时写一行，确认这一路接上了。
void Ros3DPage::onPosCmdUpdated(const PosCmdMsg& msg)
{
    if (posCmdSeen_) return;

    posCmdSeen_ = true;
    emit appendMessage(QStringLiteral("位置控制指令: 首次收到 (topic=%1)").arg(msg.topic));
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
