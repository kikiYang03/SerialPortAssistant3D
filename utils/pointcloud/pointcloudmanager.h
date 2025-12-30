#ifndef POINTCLOUDMANAGER_H
#define POINTCLOUDMANAGER_H

#include <QObject>
#include <QVector>
#include <QMatrix4x4>

struct PointXYZI {
    float x, y, z;
    float intensity;
};

class PointCloudManager : public QObject {
    Q_OBJECT
public:
    explicit PointCloudManager(QObject* parent = nullptr);

    // cloud_registered（10Hz）
    void addRegisteredCloud(const QVector<PointXYZI>& cloud,
                            const QMatrix4x4& T_map_camera);

    // Laser_map（1Hz）
    void setLaserMap(const QVector<PointXYZI>& cloud,
                     const QMatrix4x4& T_map_camera);

    const QVector<PointXYZI>& accumulatedCloud() const;
    const QVector<PointXYZI>& laserMapCloud() const;

    void clear();

private:
    QVector<PointXYZI> accumulated_;
    QVector<PointXYZI> laserMap_;
};

#endif // POINTCLOUDMANAGER_H
