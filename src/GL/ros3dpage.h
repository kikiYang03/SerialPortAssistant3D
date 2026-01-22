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
    QLabel* labZMin_ = nullptr;
    QLabel* labZMax_ = nullptr;
    QSlider* sliderZMin_ = nullptr;
    QSlider* sliderZMax_ = nullptr;
    QDoubleSpinBox* spinZMin_ = nullptr;
    QDoubleSpinBox* spinZMax_ = nullptr;
    DualRangeSlider* dualSlider_ = nullptr;  // 新增：双滑块

private slots:
    void onDualRangeChanged(double lower, double upper);  // 新增
};

#endif // ROS3DPAGE_H
