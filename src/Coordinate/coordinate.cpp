#include "coordinate.h"
#include "ui_coordinate.h"
#include <QPainter>
#include <QWheelEvent>
#include <QtMath>
#include <QMouseEvent>
#include <QFileInfo>
#include <QDebug>

// ---------------- CanvasWidget 实现 ----------------
CanvasWidget::CanvasWidget(QWidget *parent)
    : QFrame(parent)
    , m_x(0)
    , m_y(0)
    , m_z(0)
    , m_yaw(0)
    , m_scale(40.0) // 默认 1m = 40像素
    , m_dragging(false)
    , m_arrowLoaded(false)
{
    setFrameStyle(QFrame::Box);
    setLineWidth(1);
    setStyleSheet("background-color: white;");
    setMinimumSize(400, 400);
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

    if (!loadArrowImage(":/images/arrow.png")) {
        qDebug() << "无法加载箭头图片，将使用默认绘制方式";
    }
}

bool CanvasWidget::loadArrowImage(const QString &filePath)
{
    QFileInfo fileInfo(filePath);
    if (fileInfo.exists()) {
        m_arrowPixmap.load(filePath);
        if (!m_arrowPixmap.isNull()) {
            m_arrowLoaded = true;
            return true;
        }
    }
    return false;
}

void CanvasWidget::updateCoordinates(qint16 x, qint16 y, qint16 z, qint16 yaw)
{
    m_x = x;
    m_y = y;
    m_z = z;
    m_yaw = yaw;
    update();
}

void CanvasWidget::resetView()
{
    m_offset = QPoint(0, 0);
    m_scale = 40.0;
    update();
}

void CanvasWidget::paintEvent(QPaintEvent *event)
{
    Q_UNUSED(event);
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, true);

    int width = this->width();
    int height = this->height();

    // 背景
    painter.fillRect(0, 0, width, height, Qt::white);

    // 视图中心点（画布原点）
    int centerX = width / 2 + m_offset.x();
    int centerY = height / 2 + m_offset.y();

    // ---------------- 绘制网格 ----------------
    painter.setPen(QPen(Qt::lightGray, 1, Qt::DotLine));

    double gridSize = 1.0; // 每格 1m
    int gridCount = 10;    // 共10格（±5m）
    int halfGrid = gridCount / 2;
    double gridPixel = m_scale * gridSize;

    for (int i = -halfGrid; i <= halfGrid; ++i) {
        // 垂直线（L方向）
        painter.drawLine(centerX + i * gridPixel, centerY - halfGrid * gridPixel,
                         centerX + i * gridPixel, centerY + halfGrid * gridPixel);
        // 水平线（F方向）
        painter.drawLine(centerX - halfGrid * gridPixel, centerY + i * gridPixel,
                         centerX + halfGrid * gridPixel, centerY + i * gridPixel);
    }

    // ---------------- 绘制坐标系原点（FLU） ----------------
    // FLU: F（前）- 屏幕上方, L（左）- 屏幕左方, U（上）暂不显示
    double axisLength = 1.0 * m_scale; // 1m 长度
    QPen axisPen(Qt::black, 1); // 与网格线等粗
    painter.setPen(axisPen);

    // F轴（前 - 上方向，y减小）
    painter.drawLine(QPointF(centerX, centerY),
                     QPointF(centerX, centerY - axisLength));

    // L轴（左 - x减小）
    painter.drawLine(QPointF(centerX, centerY),
                     QPointF(centerX - axisLength, centerY));

    // 绘制箭头
    painter.setPen(QPen(Qt::black, 1.5));
    double arrowSize = 6;

    // F箭头
    painter.drawLine(QPointF(centerX, centerY - axisLength),
                     QPointF(centerX - arrowSize, centerY - axisLength + arrowSize));
    painter.drawLine(QPointF(centerX, centerY - axisLength),
                     QPointF(centerX + arrowSize, centerY - axisLength + arrowSize));

    // L箭头
    painter.drawLine(QPointF(centerX - axisLength, centerY),
                     QPointF(centerX - axisLength + arrowSize, centerY - arrowSize));
    painter.drawLine(QPointF(centerX - axisLength, centerY),
                     QPointF(centerX - axisLength + arrowSize, centerY + arrowSize));

    // 标签
    QFont font = painter.font();
    font.setPointSize(10);
    painter.setFont(font);
    painter.drawText(centerX - 15, centerY - axisLength - 5, "F");
    painter.drawText(centerX - axisLength - 10, centerY + 15, "L");
}



void CanvasWidget::drawArrow(QPainter &painter, double x, double y, double angle)
{
    double drawAngle = -angle; // FLU中顺时针为负，Qt中逆时针为正

    if (m_arrowLoaded) {
        int arrowSize = 30;
        painter.save();
        painter.translate(x, y);
        painter.rotate(drawAngle);
        QRect arrowRect(-arrowSize / 2, -arrowSize / 2, arrowSize, arrowSize);
        painter.drawPixmap(arrowRect, m_arrowPixmap);
        painter.restore();
    } else {
        double arrowLength = 30;
        double radian = qDegreesToRadians(drawAngle);
        double endX = x + arrowLength * qCos(radian);
        double endY = y + arrowLength * qSin(radian);

        painter.setPen(QPen(Qt::red, 2));
        painter.drawLine(static_cast<int>(x), static_cast<int>(y),
                         static_cast<int>(endX), static_cast<int>(endY));

        double arrowAngle = qDegreesToRadians(30.0);
        double leftX = endX - 10 * qCos(radian - arrowAngle);
        double leftY = endY - 10 * qSin(radian - arrowAngle);
        double rightX = endX - 10 * qCos(radian + arrowAngle);
        double rightY = endY - 10 * qSin(radian + arrowAngle);

        painter.drawLine(static_cast<int>(endX), static_cast<int>(endY),
                         static_cast<int>(leftX), static_cast<int>(leftY));
        painter.drawLine(static_cast<int>(endX), static_cast<int>(endY),
                         static_cast<int>(rightX), static_cast<int>(rightY));
    }
}

void CanvasWidget::wheelEvent(QWheelEvent *event)
{
    if (event->angleDelta().y() > 0)
        m_scale *= 1.1;
    else
        m_scale /= 1.1;

    if (m_scale < 10.0) m_scale = 10.0;
    if (m_scale > 200.0) m_scale = 200.0;
    update();
    event->accept();
}

void CanvasWidget::mousePressEvent(QMouseEvent *event)
{
    if (event->button() == Qt::LeftButton) {
        m_dragging = true;
        m_lastMousePos = event->pos();
        setCursor(Qt::ClosedHandCursor);
    }
}

void CanvasWidget::mouseMoveEvent(QMouseEvent *event)
{
    if (m_dragging) {
        QPoint delta = event->pos() - m_lastMousePos;
        m_offset += delta;
        m_lastMousePos = event->pos();
        update();
    }
}

void CanvasWidget::mouseReleaseEvent(QMouseEvent *event)
{
    if (event->button() == Qt::LeftButton) {
        m_dragging = false;
        setCursor(Qt::ArrowCursor);
    }
}

void CanvasWidget::centerOnArrow()
{
    int width = this->width();
    int height = this->height();
    int centerX = width / 2;
    int centerY = height / 2;

    double scaledX = -m_y / 100.0 * m_scale;
    double scaledY = -m_x / 100.0 * m_scale;

    int arrowX = centerX + scaledX;
    int arrowY = centerY + scaledY;

    int targetOffsetX = centerX - arrowX;
    int targetOffsetY = centerY - arrowY;

    m_offset = QPoint(targetOffsetX, targetOffsetY);
    update();
}

// ---------------- Coordinate 界面类实现 ----------------
Coordinate::Coordinate(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::Coordinate)
{
    ui->setupUi(this);

    m_canvas = new CanvasWidget(this);
    m_canvas->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

    QLayout *layout = ui->canvasWidget->parentWidget()->layout();
    if (!layout) {
        layout = new QVBoxLayout(ui->canvasWidget->parentWidget());
        layout->setContentsMargins(0, 0, 0, 0);
    }
    layout->replaceWidget(ui->canvasWidget, m_canvas);
    delete ui->canvasWidget;
    ui->canvasWidget = nullptr;

    connect(ui->centralBtn, &QPushButton::clicked, this, &Coordinate::on_centralBtn_clicked);
    connect(ui->centerArrowBtn, &QPushButton::clicked, this, &Coordinate::on_centerArrowBtn_clicked);
}

Coordinate::~Coordinate()
{
    delete ui;
}

void Coordinate::updateCoordinates(qint16 x, qint16 y, qint16 z, qint16 yaw)
{
    m_canvas->updateCoordinates(x, y, z, yaw);
}

void Coordinate::on_centralBtn_clicked()
{
    m_canvas->resetView();
}

void Coordinate::on_centerArrowBtn_clicked()
{
    m_canvas->centerOnArrow();
}
