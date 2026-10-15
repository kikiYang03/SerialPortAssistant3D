#ifndef TFTREE_H
#define TFTREE_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string_view>

using Vec3 = Eigen::Vector3d;
using Quat = Eigen::Quaterniond;
using Mat4 = Eigen::Matrix4d;

// 位姿摆放纯模块（v2.8 口径）——上位机**唯一**的坐标摆放出口。
//
// 只承载两条边，两条都从图的原点系 base_init_map 出发：
//
//   Edge::BaseLink   base_init_map → base_link   由 0x01 喂，随帧更新（动态边）
//   Edge::BaseInit   base_init_map → base_init   由 0x02 喂，静态、**仅供显示**
//
// 边按**落点**（子帧）命名：v2.6 那条 `base_init → laser_link`（雷达位姿）是本次要消灭的
// 旧模型，本模块里**不存在**任何以 laser_link 为落点的边，也不存在第二个父系。
//
// 对外三件事：喂一帧位姿（setPose）、把一个点摆进 base_init_map（placeInMap）、
// 判这条边还新不新鲜（isFresh）。摆放动作只此一处，**调用点不得自建矩阵**。
//
// **不依赖 Qt**（只用 Eigen 与标准库）：脱离 Qt 才编得动 tests/test_tftree.pro 那份用例，
// 而本次改动的对错在实机上看不出来（安装位姿 T 全 0 ⇒ base_init_map ≡ base_init），
// 验收只能落在那份用例上。
class TfTree
{
public:
    using Clock = std::chrono::steady_clock;
    using Stamp = Clock::time_point;

    // 边 = 「父系恒为 base_init_map，落点是哪个子帧」。
    enum class Edge {
        BaseLink = 0,   // 0x01：base_init_map → base_link
        BaseInit = 1,   // 0x02：base_init_map → base_init（静态、仅供显示）
    };

    // 边只有两条。加第三条（比如把 laser_link 那条旧边复活）先得过这两道关：
    // 下面这条断言，以及用例里盯住它的那条 CHECK。
    static constexpr int kEdgeCount = 2;
    static_assert(static_cast<int>(Edge::BaseLink) == 0 &&
                      static_cast<int>(Edge::BaseInit) == 1,
                  "边一共两条，枚举值就是内部数组的下标，不许插队");

    // 新鲜度窗口：最近一帧还在窗口内 ⇒ 该边可用。500 ms = 10 Hz 的 5 个周期。
    // 两条边共用同一个判据（工单 07 的「0x01 陈旧即不摆放」与工单 08 的
    // 「0x02 超时即未定位」取的都必须是这一个值，不许各自另定超时）。
    static constexpr std::chrono::milliseconds kFreshWindow{500};

    // 喂一帧。位姿按**线上形状**给：平移 t + 四元数 q——注意 Eigen 的构造是 **w 在前**
    // （`Quat(qw, qx, qy, qz)`），而线上是 `qx,qy,qz,qw`，别顺手把线上顺序喂进来。
    // `at` 是这一帧的**到达时刻**：时间由调用方给、模块自己不读钟，
    // 「恰好 500 ms 判新鲜」这条边界只能在用例里用定死的时间点钉住。
    // 位姿→矩阵这一折就落在这里；四元数按原样用，不替调用方归一化（线上发的就该是单位四元数）。
    void setPose(Edge e, const Vec3 &t, const Quat &q, Stamp at);

    // 该边最近一帧是否还在 kFreshWindow 内：**恰好 500 ms 判新鲜，刚过即不新鲜**。
    // 从没喂过的边一律不新鲜。
    bool isFresh(Edge e, Stamp now) const;

    // 摆放：把**属于该边子帧**的一点摆进 base_init_map（父 ← 子：p_父 = R·p_子 + t）。
    // 边没喂过、或已超过 kFreshWindow ⇒ 返回空——超时**不摆放**：
    // 既不拿上一帧的旧位姿凑，也不用恒等变换兜底（那会伪造一个「机器人就在图原点」的假象）。
    //
    // ⚠️ 这个入口**只服务「载荷在子帧下」的点**——今天只有 `0x05` 扫描（base_link 系）。
    // `0x03` 点云与 `0x06` 栅格地图的载荷**本来就在 base_init_map 系里**，上位机对它们一行
    // 变换都不做（工单 06）：把它们送进这里不仅白做一次恒等乘法，还会被上面「超时不摆放」
    // 那条判据连累——未定位时整条点云都画不出来（静默错误）。
    std::optional<Vec3> placeInMap(Edge e, const Vec3 &p, Stamp now) const;

    // 该边当前的矩阵（父 ← 子），**只给渲染用**：画机器人箭头、画轴。
    // 它**不判新鲜度**——界面上「最后一帧停在哪儿」照旧要画得出来（是否该画由调用方按 isFresh 定）。
    // 摆点一律走 placeInMap()，**别拿这个自己乘**。
    std::optional<Mat4> edgeTransform(Edge e) const;

    // 两条边一起清掉（位姿与到达时刻都清）。
    void clear();

private:
    struct EdgeState {
        std::optional<Mat4> T;  // 父 ← 子
        Stamp at{};             // 最近一帧的到达时刻
    };

    mutable std::mutex m_;
    EdgeState edges_[kEdgeCount];
};

// —— 帧名期望表与校验入口（工单 02）——

// 帧名从「读了不用」变成「读了就校验」。这是本次唯一一处**主动拒绝**：0x01 的父系从
// `world` 改成了 `base_init_map`，而载荷字段一个都没变——不读帧名，这种改名就是一条静默通道。
//
// 期望值只在 **tftree.cpp 里的 kFrameNameExpectations 那一张表**上出现一份：协议再改帧名
// 只改那一处，**别把帧名字符串抄到解析层或界面层**。那张表也是 0x05 / 0x06 两个「硬约定」
// 帧名唯一的落点（这两帧的帧名模块不发，表里记着只为口径查得到）。
//
// 帧号 = 协议里 `AA NN [json] 0A` 的那个 NN（**不是**串口那套 0x01/0x04/0x05 的编号）。
// 哪一帧带帧名字段由协议定，不由此处的实现决定：
//
//   0x01 机器人位姿   载荷里 frame_id / child_frame_id 都在  → 两侧都校
//   0x02 重定位结果   同上                                  → 两侧都校
//   0x03 3D 点云      载荷里只有 frame_id                   → 只校父系
//   0x05 2D 扫描      载荷里**没有**帧名字段                → 不可校，一律接受
//   0x06 2D 地图      同上                                  → 不可校，一律接受
//
// 校验只此一个入口，落在解析层（工单 05）：拿到 JSON、知道帧号的那一层调它，对不上就丢帧。
// **不要在那里另写一套期望值**——那正是这张表要收掉的东西。
//
// ⚠️ 「接受」不等于「校验通过」：`0x05` / `0x06` **恒返回 true**，因为这两帧的帧名
// （硬约定 `base_link` / `base_init_map`）模块根本不发、上位机读不到，**不可校验**。
// 表外的帧号（0x07 / 0x08 / 0x09 / 0x0B / 0x0C 这些本来就不带帧名字段的，或干脆是协议外的号）
// 同理判接受——本模块只对上面这五帧有帧名口径，别的帧没有期望，也就无从校验。
// 谁要把这些当成「已校验」，先得让模块把帧名发出来。
bool frameNamesAccepted(uint8_t frameNo,
                        std::string_view frameId,
                        std::string_view childFrameId);

#endif // TFTREE_H
