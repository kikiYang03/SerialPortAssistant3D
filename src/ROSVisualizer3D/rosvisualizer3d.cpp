#include "rosvisualizer3d.h"
#include "ui_rosvisualizer3d.h"

ROSVisualizer3D::ROSVisualizer3D(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::ROSVisualizer3D)
{
    ui->setupUi(this);
}

ROSVisualizer3D::~ROSVisualizer3D()
{
    delete ui;
}
