#ifndef COORDINATE_H
#define COORDINATE_H

#include <QWidget>
#include <QPixmap>
#include <QDebug>
#include <QFrame>  // 新增

namespace Ui {
class Coordinate;
}

// 新增：自定义画布类
class CanvasWidget : public QFrame
{
    Q_OBJECT

public:
    explicit CanvasWidget(QWidget *parent = nullptr);

    void updateCoordinates(qint16 x, qint16 y, qint16 z, qint16 yaw);
    void resetView();
    void centerOnArrow();

protected:
    void paintEvent(QPaintEvent *event) override;
    void wheelEvent(QWheelEvent *event) override;
    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;

private:
    void drawArrow(QPainter &painter, double x, double y, double angle);
    bool loadArrowImage(const QString &filePath);

    qint16 m_x;
    qint16 m_y;
    qint16 m_z;
    qint16 m_yaw;
    double m_scale;

    bool m_dragging;
    QPoint m_lastMousePos;
    QPoint m_offset;

    QPixmap m_arrowPixmap;
    bool m_arrowLoaded;
};

class Coordinate : public QWidget
{
    Q_OBJECT

public:
    explicit Coordinate(QWidget *parent = nullptr);
    ~Coordinate();

    void updateCoordinates(qint16 x, qint16 y, qint16 z, qint16 yaw);

private slots:
    void on_centralBtn_clicked();

    void on_centerArrowBtn_clicked();

private:
    Ui::Coordinate *ui;
    CanvasWidget *m_canvas;  // 修改为自定义画布类
};

#endif // COORDINATE_H
