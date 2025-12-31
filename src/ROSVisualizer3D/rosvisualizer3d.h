#include <QWidget>
#include <QJsonObject>
#include <QVector>
#include <utils/glscene/glscene.h>
#include <utils/protocol/protocol_msg.h>

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
    void onTFUpdated(const TFMsg& msg);
    void onMapUpdated(const MapMsg& msg);
    void onCloudUpdated(const CloudMsg& msg);

private:
    Ui::ROSVisualizer3D *ui;
    GLScene* glScene_;

    QMap<QString, Transform> tfMap_;
    QVector<Point3D> scanPoints_;
    QVector<Transform> tfList_;
};
