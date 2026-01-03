#ifndef PROTOCOL_MSG_H
#define PROTOCOL_MSG_H
#include<QString>
#include<QVector3D>
#include<QQuaternion>
#include <QDebug>

struct TFMsg {
    QString frame_id;
    QString child_frame_id;
    QVector3D t;
    QQuaternion q;
};

struct CloudMsg {
    QString frame_id;
    QVector<QVector3D> points;   // 或先只存统计量
};

struct MapCloudMsg {
    QString frame_id;
    QVector<QVector3D> points;
};

struct Transform {
    QString frame;
    QString child;
    QVector3D t;
    QQuaternion q;
};


inline QDebug operator<<(QDebug dbg, const TFMsg &m)
{
    QDebugStateSaver saver(dbg);
    dbg.nospace() << "TFMsg{frame=" << m.frame_id
                  << ", child=" << m.child_frame_id
                  << ", t=" << m.t
                  << ", q=" << m.q << '}';
    return dbg;
}

inline QDebug operator<<(QDebug dbg, const CloudMsg &m)
{
    QDebugStateSaver saver(dbg);
    dbg.nospace() << "CloudMsg{frame=" << m.frame_id
                  << ", points=" << m.points.size() << '}';
    return dbg;
}

inline QDebug operator<<(QDebug dbg, const MapCloudMsg &m)
{
    QDebugStateSaver saver(dbg);
    dbg.nospace() << "MapCloudMsg{frame=" << m.frame_id
                  << ", points=" << m.points.size() << '}';
    return dbg;
}

inline QDebug operator<<(QDebug dbg, const Transform &t)
{
    QDebugStateSaver saver(dbg);
    dbg.nospace() << "Transform{frame=" << t.frame
                  << ", child=" << t.child
                  << ", t=" << t.t
                  << ", q=" << t.q << '}';
    return dbg;
}
#endif // PROTOCOL_MSG_H
