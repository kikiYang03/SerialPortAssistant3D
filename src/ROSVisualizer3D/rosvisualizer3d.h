#ifndef ROSVISUALIZER3D_H
#define ROSVISUALIZER3D_H

#include <QWidget>

namespace Ui {
class ROSVisualizer3D;
}

class ROSVisualizer3D : public QWidget
{
    Q_OBJECT

public:
    explicit ROSVisualizer3D(QWidget *parent = nullptr);
    ~ROSVisualizer3D();

private:
    Ui::ROSVisualizer3D *ui;
};

#endif // ROSVISUALIZER3D_H
