#ifndef PROTOCOL_MSG_H
#define PROTOCOL_MSG_H
#include<QString>
#include<QByteArray>
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

// 位置控制指令 (0x08)
struct PosCmdMsg {
    QString topic;   // 恒为 "pos_cmd"
    double x, y, z, yaw;
};

// 状态帧 (0x0B)：飞控 + 重定位 + 路径规划三件共用一帧
// 三件都是「当前值」不是「事件」：模块听到过任一上游自述后就按 10 Hz 一直重发。
struct NavStatusMsg {
    QString topic;          // 恒为 "status"
    int fcState = 0;        // 飞控，0~4 五档；0 在这里同时是初值（飞控链路没起）
    int relocalization = 3; // 重定位，0 进行中 / 1 成功 / 2 失败 / 3 未响应（初值）
    int planning = 3;       // 路径规划，取值同重定位

    // 线上还有第五个键 `state`，恒等于 `planning`——那是给只认单值键的旧上位机留的兼容别名。
    // 这里不给它单独立一个成员：同值同义要的是「两者不一致」根本不可表示，
    // 而不是靠「解析后记得同步一下」去维持两份值。
    int state() const { return planning; }
};

// 新协议消息结构体
// 父系名一律是**图的原点系** `base_init_map`（机器人建图那趟开机那一刻的中心所在的位置）；
// v2.5/03~v2.7/10 的中间名 `world` 已废。
struct RobotPoseMsg {
    QString frame_id;        // "base_init_map"
    QString child_frame_id;  // "base_link"
    double x, y, z;
    double qx, qy, qz, qw;
};

// 重定位结果 (0x02)：**本次开机**的原点在图里的位姿——不是雷达位姿。
//
// v2.6 时这一帧是「雷达位姿 `base_init -> laser_link`」，载荷字段**一个都没变**，
// 所以沿用旧名字（LidarPoseMsg）会是一条静默通道：名字还在旧义、行为已换新义。
// 这条边整条是静态的，模块**定位成功前一条都不发**，收端按超时判「还没定位」。
struct RelocResultMsg {
    QString frame_id;        // "base_init_map"
    QString child_frame_id;  // "base_init"
    double x, y, z;
    double qx, qy, qz, qw;
};

// 2D激光雷达扫描数据 (0x05)
//
// ⚠️ 载荷里**没有帧名字段**：帧是硬约定 `base_link`（模块侧 /scan 已按安装位姿折进机器人中心），
// 上位机读不到、**不可校验**。所以这里刻意不留一个永远是空串的 frame_id 成员
// ——留了就会有人当它可读。
struct Scan2DMsg {
    double angle_min;
    double angle_max;
    double angle_increment;
    int range_count;
    QVector<float> ranges;
};

// 2D占用栅格地图 (0x06)
//
// ⚠️ 同样**没有帧名字段**：帧硬约定 `base_init_map`（栅格像素天生长在图的原点系上），
// 载荷只有 origin_x / origin_y 两个**平移**数、没有姿态字段。
struct Map2DMsg {
    int width;
    int height;
    double resolution;
    double origin_x;
    double origin_y;
    QVector<int8_t> data;  // 解压后的地图数据
};

// 地图下传分块 (0x0C)：一包一块，块是「整包」按固定字节数切开的一段。
// topic / name / bytes / crc32 四个字段在**每一块**上都重复一遍，且描述的都是**整包**——
// 它们是模块随每块附上的自述，不是本块的属性。拼装与校验见 utils/protocol/maparchive.h。
struct MapChunkMsg {
    QString topic;    // 恒为 "map_archive"
    QString name;     // 包名：3D 是 map_3d.zip，2D 是 map_2d.zip
    int seq = 0;      // 本块序号，从 1 起（0 与「没发过」分不开）
    int total = 0;    // 总共几块；收满 total 块才算收全
    int bytes = 0;    // 整包的字节数（不是本块的）
    QString crc32;    // 整包的 CRC-32，线上形如 "0x1234abcd"（不是本块的）
    QByteArray data;  // 本块的字节——线上的 base64 在解析层已经解开
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
Q_DECLARE_METATYPE(PosCmdMsg)
Q_DECLARE_METATYPE(NavStatusMsg)
Q_DECLARE_METATYPE(RobotPoseMsg)
Q_DECLARE_METATYPE(RelocResultMsg)
Q_DECLARE_METATYPE(Scan2DMsg)
Q_DECLARE_METATYPE(Map2DMsg)
Q_DECLARE_METATYPE(MapChunkMsg)


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

inline QDebug operator<<(QDebug dbg, const PosCmdMsg &m)
{
    QDebugStateSaver saver(dbg);
    dbg.nospace() << "PosCmdMsg{topic=" << m.topic
                  << ", pos=(" << m.x << "," << m.y << "," << m.z
                  << "), yaw=" << m.yaw << '}';
    return dbg;
}

inline QDebug operator<<(QDebug dbg, const NavStatusMsg &m)
{
    QDebugStateSaver saver(dbg);
    dbg.nospace() << "NavStatusMsg{topic=" << m.topic
                  << ", fc_state=" << m.fcState
                  << ", relocalization=" << m.relocalization
                  << ", planning=" << m.planning << '}';
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

inline QDebug operator<<(QDebug dbg, const RelocResultMsg &m)
{
    QDebugStateSaver saver(dbg);
    dbg.nospace() << "RelocResultMsg{frame=" << m.frame_id
                  << ", child=" << m.child_frame_id
                  << ", pos=(" << m.x << "," << m.y << "," << m.z << ")}";
    return dbg;
}

inline QDebug operator<<(QDebug dbg, const Scan2DMsg &m)
{
    QDebugStateSaver saver(dbg);
    // 不打帧名：载荷里没有这个字段，帧是硬约定的 base_link
    dbg.nospace() << "Scan2DMsg{ranges=" << m.range_count << " points}";
    return dbg;
}

inline QDebug operator<<(QDebug dbg, const Map2DMsg &m)
{
    QDebugStateSaver saver(dbg);
    dbg.nospace() << "Map2DMsg{size=" << m.width << "x" << m.height
                  << ", resolution=" << m.resolution << "}";
    return dbg;
}

// 只打元信息，不打 payload：一块几十 KB，逐块打出来会淹掉接收窗口。
inline QDebug operator<<(QDebug dbg, const MapChunkMsg &m)
{
    QDebugStateSaver saver(dbg);
    dbg.nospace() << "MapChunkMsg{name=" << m.name
                  << ", seq=" << m.seq << '/' << m.total
                  << ", bytes=" << m.bytes
                  << ", data=" << m.data.size() << "B}";
    return dbg;
}
#endif // PROTOCOL_MSG_H
