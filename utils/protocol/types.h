#ifndef TYPES_H
#define TYPES_H

#include <QVector3D>
#include <QQuaternion>
#include <QString>

struct Point3D {
    float x, y, z;

    // 转换到QVector3D
    inline QVector3D toVector3D() const {
        return QVector3D(x, y, z);
    }

    // 从QVector3D转换
    inline static Point3D fromVector3D(const QVector3D& vec) {
        return {static_cast<float>(vec.x()),
                static_cast<float>(vec.y()),
                static_cast<float>(vec.z())};
    }
};

struct Transform {
    QString frame;     // parent frame
    QString child;     // child frame
    QVector3D t;       // translation
    QQuaternion q;     // rotation
};
#endif // TYPES_H
