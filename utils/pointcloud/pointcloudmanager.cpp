#include "pointcloudmanager.h"

static inline PointXYZI transformPoint(const PointXYZI& p,
                                       const QMatrix4x4& T)
{
    QVector4D v(p.x, p.y, p.z, 1.0f);
    v = T * v;
    return { v.x(), v.y(), v.z(), p.intensity };
}

PointCloudManager::PointCloudManager(QObject* parent)
    : QObject(parent)
{
}

void PointCloudManager::addRegisteredCloud(
    const QVector<PointXYZI>& cloud,
    const QMatrix4x4& T_map_camera)
{
    accumulated_.reserve(accumulated_.size() + cloud.size());
    for (const auto& p : cloud)
        accumulated_.push_back(transformPoint(p, T_map_camera));
}

void PointCloudManager::setLaserMap(
    const QVector<PointXYZI>& cloud,
    const QMatrix4x4& T_map_camera)
{
    laserMap_.clear();
    laserMap_.reserve(cloud.size());
    for (const auto& p : cloud)
        laserMap_.push_back(transformPoint(p, T_map_camera));
}

const QVector<PointXYZI>& PointCloudManager::accumulatedCloud() const
{
    return accumulated_;
}

const QVector<PointXYZI>& PointCloudManager::laserMapCloud() const
{
    return laserMap_;
}

void PointCloudManager::clear()
{
    accumulated_.clear();
    laserMap_.clear();
}
