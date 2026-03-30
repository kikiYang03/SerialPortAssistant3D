#ifndef ROS3DPAGE_H
#define ROS3DPAGE_H
#pragma once
#include <QWidget>
#include <QCheckBox>
#include <QSlider>
#include <QDoubleSpinBox>
#include "dualrangeslider.h"
#include "glwidget.h"

class GLWidget;
class QLabel;
class QPushButton;

class Ros3DPage : public QWidget
{
    Q_OBJECT
public:
    explicit Ros3DPage(QWidget* parent = nullptr);

    GLWidget* glWidget() const { return gl_; }

signals:
    void sendNavGoalRequested(double x, double y, double z, double yaw_deg);

private:
    GLWidget* gl_ = nullptr;

    // TF 显示
    QLabel* labX_   = nullptr;
    QLabel* labY_   = nullptr;
    QLabel* labZ_   = nullptr;
    QLabel* labYaw_ = nullptr;
    QLabel* labPitch_ = nullptr;
    QLabel* labRoll_  = nullptr;

    // Z轴范围控制
    QCheckBox* ckZFilter_ = nullptr;
    DualRangeSlider* dualSlider_ = nullptr;  // 双滑块

    // 目标点控制组件
    QPushButton* btnNavGoal_ = nullptr;
    QDoubleSpinBox* spinNavX_ = nullptr;
    QDoubleSpinBox* spinNavY_ = nullptr;
    QDoubleSpinBox* spinNavZ_ = nullptr;
    QDoubleSpinBox* spinNavYaw_ = nullptr;

    QPushButton* btnSendNavGoal_ = nullptr;

private slots:
    void onDualRangeChanged(double lower, double upper);  // 双滑块的取值

    void onNavTargetUpdated(double x, double y, double z, double yaw_deg);
    void onNavSpinBoxChanged();

    void onSendNavGoalClicked();
};

#endif // ROS3DPAGE_H
