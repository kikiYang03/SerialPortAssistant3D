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

Ros3DPage::Ros3DPage(QWidget* parent)
    : QWidget(parent)
{
    gl_ = new GLWidget(this);

    // ---------- 侧边栏 ----------
    auto* side = new QFrame(this);
    side->setFrameShape(QFrame::StyledPanel);
    side->setFixedWidth(280);

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

    // ---------- 目标点设置 (新增面板) ----------
    auto* targetBox = new QGroupBox(tr("目标点指引"), side);
    auto* btnNavGoal = new QPushButton(tr("指点模式(设目标)"), targetBox);
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
    setupNavSpinBox(spinNavYaw_, " °", -180.0, 180.0);
    spinNavYaw_->setDecimals(0);       // 小数位数设为 0
    spinNavYaw_->setSingleStep(1.0);   // 每次点击上下箭头增减 1 度

    auto* targetLay = new QGridLayout(targetBox);
    targetLay->addWidget(btnNavGoal, 0, 0, 1, 2);
    targetLay->addWidget(btnSendNavGoal_, 0, 2, 1, 2);
    targetLay->addWidget(new QLabel("X:"), 1, 0); targetLay->addWidget(spinNavX_, 1, 1);
    targetLay->addWidget(new QLabel("Y:"), 1, 2); targetLay->addWidget(spinNavY_, 1, 3);
    targetLay->addWidget(new QLabel("Z:"), 2, 0); targetLay->addWidget(spinNavZ_, 2, 1);
    targetLay->addWidget(new QLabel("Yaw:"), 2, 2); targetLay->addWidget(spinNavYaw_, 2, 3);

    // auto* btnSave  = new QPushButton(tr("保存地图"), controlBox);


    auto* controlLay = new QVBoxLayout(controlBox);
    controlLay->addWidget(btnClear);
    controlLay->addWidget(btnReset);
    // controlLay->addWidget(btnNavGoal);
    // controlLay->addWidget(btnSave);
    controlLay->addStretch();   // 把按钮顶到上面

    // btnSave->setEnabled(false);

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
    dualSlider_->setRange(-5.0, 20.0);  // 设置范围-5~20m
    dualSlider_->setValues(-5.0, 5.0);  // 初始值
    dualSlider_->setMinimumHeight(60);   // 新增这一行

    // 数值显示和微调框
    labZMin_ = new QLabel("-5.0 m", zFilterBox);
    labZMax_ = new QLabel("5.0 m", zFilterBox);

    spinZMin_ = new QDoubleSpinBox(zFilterBox);
    spinZMin_->setRange(-5.0, 20.0);
    spinZMin_->setValue(-5.0);
    spinZMin_->setSingleStep(0.5);
    spinZMin_->setSuffix(" m");

    spinZMax_ = new QDoubleSpinBox(zFilterBox);
    spinZMax_->setRange(-5.0, 20.0);
    spinZMax_->setValue(5.0);
    spinZMax_->setSingleStep(0.5);
    spinZMax_->setSuffix(" m");

    // 布局
    auto* zFilterLay = new QGridLayout(zFilterBox);
    zFilterLay->addWidget(ckZFilter_, 0, 0, 1, 3);
    zFilterLay->addWidget(dualSlider_, 1, 0, 1, 3);

    zFilterLay->addWidget(new QLabel(tr("最小值:")), 2, 0);
    zFilterLay->addWidget(spinZMin_, 2, 1);
    zFilterLay->addWidget(labZMin_, 2, 2);

    zFilterLay->addWidget(new QLabel(tr("最大值:")), 3, 0);
    zFilterLay->addWidget(spinZMax_, 3, 1);
    zFilterLay->addWidget(labZMax_, 3, 2);
    // 新增：把滑块区域撑开
    zFilterLay->setRowStretch(4, 1);   // 第4行（空行）占全部剩余空间
    zFilterLay->addWidget(new QWidget(zFilterBox), 4, 0); // 占位widget
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

    // ---------- 主布局 ----------
    auto* mainLay = new QHBoxLayout(this);
    mainLay->setContentsMargins(0,0,0,0);
    mainLay->addWidget(gl_, 1);
    mainLay->addWidget(side, 0);

    // ---------- 连接原有按钮 ----------
    connect(btnClear, &QPushButton::clicked, gl_, &GLWidget::clearMap);
    connect(btnReset, &QPushButton::clicked, gl_, &GLWidget::setCamera);
    // connect(btnSave,  &QPushButton::clicked, this, [this](){ gl_->saveMapToFile(); });

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
    // ---------- 修改：Z轴范围控制信号连接 ----------
    // 双滑块值改变
    connect(dualSlider_, &DualRangeSlider::rangeChanged,
            this, &Ros3DPage::onDualRangeChanged);

    // 微调框值改变同步到双滑块
    connect(spinZMin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, [this](double value) {
                double upper = spinZMax_->value();
                if (value > upper) {
                    value = upper;
                    spinZMin_->setValue(value);
                }
                dualSlider_->setValues(value, upper);
                gl_->setZFilterRange(value, upper);
                labZMin_->setText(QString::number(value, 'f', 1) + " m");
            });

    connect(spinZMax_, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, [this](double value) {
                double lower = spinZMin_->value();
                if (value < lower) {
                    value = lower;
                    spinZMax_->setValue(value);
                }
                dualSlider_->setValues(lower, value);
                gl_->setZFilterRange(lower, value);
                labZMax_->setText(QString::number(value, 'f', 1) + " m");
            });

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
            btnNavGoal->setText(tr("指点模式(设目标)"));
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
    // 更新微调框（阻止信号循环）
    spinZMin_->blockSignals(true);
    spinZMin_->setValue(lower);
    spinZMin_->blockSignals(false);

    spinZMax_->blockSignals(true);
    spinZMax_->setValue(upper);
    spinZMax_->blockSignals(false);

    // 更新标签
    labZMin_->setText(QString::number(lower, 'f', 1) + " m");
    labZMax_->setText(QString::number(upper, 'f', 1) + " m");

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


void Ros3DPage::onSendNavGoalClicked()
{
    TcpClient* tcpClient = TcpClient::getInstance();

    if (!tcpClient->isConnected()) {
        QMessageBox::warning(this, "错误", "请先建立TCP连接");
        return;
    }

    // 从界面微调框获取坐标和角度
    double x = spinNavX_->value();
    double y = spinNavY_->value();
    double z = spinNavZ_->value();
    double yaw_deg = spinNavYaw_->value();

    // 将角度转换为 ego-planner 需要的弧度制
    double yaw_rad = yaw_deg * M_PI / 180.0;

    // 构建下发给 ego-planner 的目标点帧: AA 04 [JSON] 0A
    QByteArray frame = ProtocolRos3D::buildNavGoalFrame(x, y, z, yaw_rad);

    // 发送数据
    tcpClient->sendData(frame);

    qDebug() << "发送导航目标点请求，数据:" << frame.toHex(' ');

    // 如果你在界面底部有日志栏，可以打印相关信息
    // QString timestamp = QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss >> 用户操作: ");
    // emit appendMessage(timestamp + QString("发送目标点 (X:%1, Y:%2, Z:%3, Yaw:%4°)").arg(x).arg(y).arg(z).arg(yaw_deg));
}
