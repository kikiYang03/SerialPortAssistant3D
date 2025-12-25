#ifndef ROSVISUALIZER_H
#define ROSVISUALIZER_H

#include <QWidget>
#include <QGraphicsView>
#include <QGraphicsScene>
#include <QComboBox>
#include <QVBoxLayout>
#include <QTimer>
#include <QWheelEvent>
#include <QImage>
#include <QPixmap>
#include <QGraphicsPixmapItem>
#include "serialport.h"

// 前向声明
namespace Ui {
class ROSVisualizer;
}

// ====================== 可缩放视图类 ======================
class ZoomableView : public QGraphicsView {
    Q_OBJECT
public:
    explicit ZoomableView(QWidget *parent = nullptr)
        : QGraphicsView(parent), m_dragging(false)
    {
        setRenderHint(QPainter::Antialiasing);
        setRenderHint(QPainter::SmoothPixmapTransform);
        setBackgroundBrush(Qt::lightGray);
        setDragMode(QGraphicsView::ScrollHandDrag);
        setTransformationAnchor(QGraphicsView::AnchorUnderMouse);
    }

protected:
    void wheelEvent(QWheelEvent *event) override {
        const double scaleFactor = 1.15;
        if (event->angleDelta().y() > 0)
            scale(scaleFactor, scaleFactor);
        else
            scale(1.0 / scaleFactor, 1.0 / scaleFactor);
    }

private:
    bool m_dragging;
};

// ====================== 主类 ======================
class ROSVisualizer : public QWidget {
    Q_OBJECT
public:
    explicit ROSVisualizer(QWidget *parent = nullptr);
    ~ROSVisualizer();

public slots:
    void updateMap(const OccupancyGrid& map);
    void updateScan(const LaserScan& scan);
    void updateTf(const TFMessage& tf);
    void clearVisualization();  // 添加公共清空方法

private slots:
    void on_clearBtn_clicked();

    void on_locateBtn_clicked();

    void on_saveMapBtn_clicked();

signals:
    // 添加消息显示信号
    void appendMessage(const QString &message);

private:
    void setupUI();
    void drawGrid();
    void drawAxes();
    void refreshMapPixmap();
    void refreshScanOverlay();
    void refreshTfOverlay();
    void updateTrajectory(const QPointF& currentPos);
    void updateRobotPose(const QPointF& position, double yaw_deg);
    void initRobotPose();

private:
    Ui::ROSVisualizer *ui;  // UI 对象指针

    ProtocolHandler *m_protocolHandler;  // 协议处理器
    QGraphicsScene *m_scene;

    // 地图、激光、TF 数据
    OccupancyGrid m_currentMap;
    LaserScan m_currentScan;
    TFMessage m_currentTf;

    // 地图与覆盖层
    QGraphicsPixmapItem *m_mapItem = nullptr;
    QGraphicsItemGroup *m_scanGroup = nullptr;
    QGraphicsItemGroup *m_tfGroup = nullptr;

    const float GRID_STEP = 1.0f; // 每格1m

    // 添加轨迹历史存储
    QVector<QPointF> m_trajectoryPoints;
    QGraphicsItemGroup* m_trajectoryGroup = nullptr;
};

#endif
