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

// 3D 点云 (0x03)。
//
// **载荷坐标已经在图的原点系（`base_init_map`）里**（v2.8 口径）：`points` 里的数就是图上
// 的坐标，谁都不许再对它做变换——图形层既不做嫁接、也不折安装位姿 T（工单 06）。
// v2.6 那套是「点在 laser_link 系、靠位姿嫁接进图」，已废；两版的差别只在 `frame_id`
// 的值上，所以那条分辨新旧的责任落在帧名校验（工单 05）——这里的 `frame_id` 是**可校验**的。
struct CloudMsg {
    QString frame_id;            // "base_init_map"
    QVector<QVector3D> points;   // 图系坐标，直接用
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
// 父系名 v2.7/14 起回归 base_init（机器人建图那趟的起点）；v2.5/03~v2.7/10 的中间名 world 已废。
struct RobotPoseMsg {
    QString frame_id;        // "base_init"
    QString child_frame_id;  // "base_link"
    double x, y, z;
    double qx, qy, qz, qw;
};

// 重定位结果 (0x02)：这一趟的**开机点**在图里哪儿。
//
// v2.8 起它的语义是 `base_init_map → base_init`（静态）；v2.6 那条 `base_init → laser_link`
// 的「雷达位姿」已废——字段形状与 v2.6 一模一样（x,y,z + 四元数 + 两个帧名），所以照旧解析
// 不会报错，只能靠帧名分辨。
//
// **不参与任何坐标变换**：它不再是谁的摆放依据。留着只做一件事——在界面上显示开机点的位置
// （工单 09，落在那条静态边上）。
struct RelocalizationPoseMsg {
    QString frame_id;        // "base_init_map"
    QString child_frame_id;  // "base_init"
    double x, y, z;
    double qx, qy, qz, qw;
};

// 2D激光雷达扫描数据 (0x05)
//
// **没有 frame_id 成员**：0x05 的载荷里根本没有帧名字段，模块也不发这个帧名。
// 它的系（base_link）是**硬约定**——唯一落点在 utils/protocol/tftree.cpp 的
// kFrameNameExpectations 那张表里（那一行记着 `fromPayload = false`，不参与比对）。
// 以前这里有个 `frame_id` 成员，它永远是空的（`obj["frame_id"]` 取不到），
// 留着只会让人以为这帧名读了、也校验了。删掉它，这个误会才不可能再发生（工单 05）。
struct Scan2DMsg {
    double angle_min;
    double angle_max;
    double angle_increment;
    int range_count;
    QVector<float> ranges;
};

// 2D占用栅格地图 (0x06)
//
// 与 Scan2DMsg 同理**没有 frame_id 成员**：0x06 的载荷里没有帧名字段，
// 它的系（base_init_map）是硬约定，唯一落点在 tftree.cpp 的对照表里（工单 05）。
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
Q_DECLARE_METATYPE(RelocalizationPoseMsg)
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

inline QDebug operator<<(QDebug dbg, const RelocalizationPoseMsg &m)
{
    QDebugStateSaver saver(dbg);
    dbg.nospace() << "RelocalizationPoseMsg{frame=" << m.frame_id
                  << ", child=" << m.child_frame_id
                  << ", pos=(" << m.x << "," << m.y << "," << m.z << ")}";
    return dbg;
}

// 不打帧名：0x05 的载荷里就没有这个字段（见 Scan2DMsg 的说明）。
inline QDebug operator<<(QDebug dbg, const Scan2DMsg &m)
{
    QDebugStateSaver saver(dbg);
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
