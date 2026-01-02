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

struct MapMsg {
    int width;
    int height;
    float resolution;
    float origin_x;
    float origin_y;
    QVector<int8_t> cells; // 解 RLE 后 or 保留 RLE
};

struct MapCloudMsg {
    QString frame_id;
    QVector<QVector3D> points;
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

inline QDebug operator<<(QDebug dbg, const MapMsg &m)
{
    QDebugStateSaver saver(dbg);
    dbg.nospace() << "MapMsg{w=" << m.width
                  << ", h=" << m.height
                  << ", res=" << m.resolution
                  << ", cells=" << m.cells.size() << '}';
    return dbg;
}

#endif // PROTOCOL_MSG_H
