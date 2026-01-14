#include "ros3dpage.h"
#include "glwidget.h"

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

Ros3DPage::Ros3DPage(QWidget* parent)
    : QWidget(parent)
{
    gl_ = new GLWidget(this);

    // ---------- 侧边栏 ----------
    auto* side = new QFrame(this);
    side->setFrameShape(QFrame::StyledPanel);
    side->setFixedWidth(280);

    auto* btnClear = new QPushButton(tr("清理地图"), side);
    auto* btnReset = new QPushButton(tr("初始化相机"), side);
    auto* btnSave  = new QPushButton(tr("保存地图"), side);
    btnSave->setEnabled(false);

    // TF group
    auto* tfBox = new QGroupBox(tr("机器人坐标"), side);
    labX_   = new QLabel("-", tfBox);
    labY_   = new QLabel("-", tfBox);
    labZ_   = new QLabel("-", tfBox);
    labYaw_ = new QLabel("-", tfBox);
    labPitch_ = new QLabel("-", tfBox);
    labRoll_ = new QLabel("-", tfBox);

    auto* form = new QFormLayout(tfBox);
    form->addRow("x",   labX_);
    form->addRow("y",   labY_);
    form->addRow("z",   labZ_);
    form->addRow("yaw", labYaw_);
    form->addRow("pitch", labPitch_);
    form->addRow("roll",  labRoll_);

    // 新增：视角旋转按钮组
    auto* rotBox = new QGroupBox(tr("视角旋转"), side);
    auto* yawLeft   = new QPushButton(tr("左转 ◀"), rotBox);
    auto* yawRight  = new QPushButton(tr("右转 ▶"), rotBox);
    auto* pitchUp   = new QPushButton(tr("上仰 ▲"), rotBox);
    auto* pitchDown = new QPushButton(tr("下俯 ▼"), rotBox);

    auto* rotLay = new QGridLayout(rotBox);
    rotLay->addWidget(pitchUp,   0, 0, 1, 2);
    rotLay->addWidget(yawLeft,   1, 0);
    rotLay->addWidget(yawRight,  1, 1);
    rotLay->addWidget(pitchDown, 2, 0, 1, 2);

    auto* sideLay = new QVBoxLayout(side);
    sideLay->addWidget(tfBox);
    sideLay->addSpacing(10);
    sideLay->addWidget(rotBox);
    sideLay->addSpacing(10);
    sideLay->addWidget(btnClear);
    sideLay->addWidget(btnReset);
    sideLay->addWidget(btnSave);
    sideLay->addStretch(1);

    // ---------- 主布局 ----------
    auto* mainLay = new QHBoxLayout(this);
    mainLay->setContentsMargins(0,0,0,0);
    mainLay->addWidget(gl_, 1);
    mainLay->addWidget(side, 0);

    // ---------- 连接原有按钮 ----------
    connect(btnClear, &QPushButton::clicked, gl_, &GLWidget::clearMap);
    connect(btnReset, &QPushButton::clicked, gl_, &GLWidget::resetCamera);
    connect(btnSave,  &QPushButton::clicked, this, [this](){ gl_->saveMapToFile(); });

    // ---------- 长按连发定时器 ----------
    auto* repeatTimer = new QTimer(this);
    repeatTimer->setInterval(200);          // 200 ms 连发间隔
    repeatTimer->setSingleShot(false);

    auto connectRepeat = [&](QPushButton* btn, int yawDelta, int pitchDelta){
        connect(btn, &QPushButton::pressed, this, [=](){
            gl_->addYaw(yawDelta);
            gl_->addPitch(pitchDelta);
            repeatTimer->disconnect();
            connect(repeatTimer, &QTimer::timeout, this, [=](){
                gl_->addYaw(yawDelta);
                gl_->addPitch(pitchDelta);
            });
            repeatTimer->start();
        });
        connect(btn, &QPushButton::released, this, [repeatTimer](){
            repeatTimer->stop();
        });
    };

    connectRepeat(yawLeft,   -5, 0);
    connectRepeat(yawRight,  +5, 0);
    connectRepeat(pitchUp,    0, -5);
    connectRepeat(pitchDown,  0, +5);

    // ---------- TF 显示更新 ----------
    connect(gl_, &GLWidget::tfInfoChanged, this,
            [this](double x,double y,double z,
                   double yaw,double pitch,double roll){
                labX_->setText(QString::number(x, 'f', 3));
                labY_->setText(QString::number(y, 'f', 3));
                labZ_->setText(QString::number(z, 'f', 3));
                labYaw_->setText(QString::number(qRadiansToDegrees(yaw),   'f', 1));
                labPitch_->setText(QString::number(qRadiansToDegrees(pitch),'f', 1));
                labRoll_->setText(QString::number(qRadiansToDegrees(roll), 'f', 1));
            });
}
