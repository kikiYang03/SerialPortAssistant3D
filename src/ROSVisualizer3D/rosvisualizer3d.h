#include <QWidget>
#include <QJsonObject>
#include <QVector>
#include <utils/glscene/glscene.h>
#include <utils/protocol/protocoldispatcher.h>


namespace Ui {
class ROSVisualizer3D;
}

class ROSVisualizer3D : public QWidget
{
    Q_OBJECT
public:
    explicit ROSVisualizer3D(QWidget *parent = nullptr);
    ~ROSVisualizer3D();

public slots:
    void onProtocolJson(int cmd, const QJsonObject& obj);

private:
    void handleTF(const QJsonObject& obj);
    void handleScan(const QJsonObject& obj);
    void handleMap(const QJsonObject& obj);

private:
    Ui::ROSVisualizer3D *ui;
    GLScene* glScene_;

    QMap<QString, Transform> tfMap_;
    QVector<Point3D> scanPoints_;
    QVector<Transform> tfList_;
};
