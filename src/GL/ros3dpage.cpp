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


    auto* sideLay = new QVBoxLayout(side);
    sideLay->addWidget(tfBox);
    sideLay->addSpacing(10);
    sideLay->addWidget(btnClear);
    sideLay->addWidget(btnReset);
    sideLay->addWidget(btnSave);
    sideLay->addStretch(1);

    // ---------- 主布局（左 GL，右侧边栏） ----------
    auto* mainLay = new QHBoxLayout(this);
    mainLay->setContentsMargins(0,0,0,0);
    mainLay->addWidget(gl_, 1);
    mainLay->addWidget(side, 0);

    // ---------- 连接按钮到 GLWidget 的功能槽 ----------
    connect(btnClear, &QPushButton::clicked, gl_, &GLWidget::clearMap);
    connect(btnReset, &QPushButton::clicked, gl_, &GLWidget::resetCamera);

    connect(btnSave, &QPushButton::clicked, this, [this](){
        gl_->saveMapToFile();
    });

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
