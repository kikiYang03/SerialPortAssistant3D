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

struct PathMsg {
    QVector<QVector3D> points;
};

// 新协议消息结构体
struct RobotPoseMsg {
    QString frame_id;        // "map"
    QString child_frame_id;  // "base_link"
    double x, y, z;
    double qx, qy, qz, qw;
};

struct LidarPoseMsg {
    QString frame_id;        // "map"
    QString child_frame_id;  // "laser_link"
    double x, y, z;
    double qx, qy, qz, qw;
};

// 2D激光雷达扫描数据 (0x05)
struct Scan2DMsg {
    QString frame_id;
    double angle_min;
    double angle_max;
    double angle_increment;
    int range_count;
    QVector<float> ranges;
};

// 2D占用栅格地图 (0x06)
struct Map2DMsg {
    QString frame_id;
    int width;
    int height;
    double resolution;
    double origin_x;
    double origin_y;
    QVector<int8_t> data;  // 解压后的地图数据
};

struct Transform {
    QString frame;
    QString child;
    QVector3D t;
    QQuaternion q;
};

Q_DECLARE_METATYPE(TFMsg)
Q_DECLARE_METATYPE(CloudMsg)
Q_DECLARE_METATYPE(MapCloudMsg)
Q_DECLARE_METATYPE(PathMsg)
Q_DECLARE_METATYPE(RobotPoseMsg)
Q_DECLARE_METATYPE(LidarPoseMsg)
Q_DECLARE_METATYPE(Scan2DMsg)
Q_DECLARE_METATYPE(Map2DMsg)


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

inline QDebug operator<<(QDebug dbg, const PathMsg &m)
{
    QDebugStateSaver saver(dbg);
    dbg.nospace() << "PathMsg {points=" << m.points.size() << '}';
    return dbg;
}

inline QDebug operator<<(QDebug dbg, const RobotPoseMsg &m)
{
    QDebugStateSaver saver(dbg);
    dbg.nospace() << "RobotPoseMsg{frame=" << m.frame_id
                  << ", child=" << m.child_frame_id
                  << ", pos=(" << m.x << "," << m.y << "," << m.z << ")}";
    return dbg;
}

inline QDebug operator<<(QDebug dbg, const LidarPoseMsg &m)
{
    QDebugStateSaver saver(dbg);
    dbg.nospace() << "LidarPoseMsg{frame=" << m.frame_id
                  << ", child=" << m.child_frame_id
                  << ", pos=(" << m.x << "," << m.y << "," << m.z << ")}";
    return dbg;
}

inline QDebug operator<<(QDebug dbg, const Scan2DMsg &m)
{
    QDebugStateSaver saver(dbg);
    dbg.nospace() << "Scan2DMsg{frame=" << m.frame_id
                  << ", ranges=" << m.range_count << " points}";
    return dbg;
}

inline QDebug operator<<(QDebug dbg, const Map2DMsg &m)
{
    QDebugStateSaver saver(dbg);
    dbg.nospace() << "Map2DMsg{size=" << m.width << "x" << m.height
                  << ", resolution=" << m.resolution << "}";
    return dbg;
}
#endif // PROTOCOL_MSG_H
