#include "tftree.h"

#include <cstddef>
#include <cstdint>
#include <string_view>

namespace {

// 边的枚举值就是 edges_ 的下标（头文件里有断言盯着）。
inline std::size_t slot(TfTree::Edge e) { return static_cast<std::size_t>(e); }

} // namespace

void TfTree::setPose(Edge e, const Vec3 &t, const Quat &q, Stamp at)
{
    Mat4 T = Mat4::Identity();
    T.block<3, 3>(0, 0) = q.toRotationMatrix();   // 先转
    T.block<3, 1>(0, 3) = t;                      // 后移（t 不进旋转）

    std::lock_guard<std::mutex> lk(m_);
    EdgeState &s = edges_[slot(e)];
    s.T  = T;
    s.at = at;
}

bool TfTree::isFresh(Edge e, Stamp now) const
{
    std::lock_guard<std::mutex> lk(m_);
    const EdgeState &s = edges_[slot(e)];
    // 恰好 500 ms 判新鲜：窗口取闭区间。
    return s.T.has_value() && (now - s.at) <= kFreshWindow;
}

std::optional<Vec3> TfTree::placeInMap(Edge e, const Vec3 &p, Stamp now) const
{
    std::lock_guard<std::mutex> lk(m_);
    const EdgeState &s = edges_[slot(e)];

    // 没喂过、或这一帧已经超出 500 ms ⇒ 不摆放（返回空，由调用方决定不画）。
    if (!s.T.has_value() || (now - s.at) > kFreshWindow)
        return std::nullopt;

    const Mat4 &T = *s.T;
    return T.block<3, 3>(0, 0) * p + T.block<3, 1>(0, 3);
}

std::optional<Mat4> TfTree::edgeTransform(Edge e) const
{
    std::lock_guard<std::mutex> lk(m_);
    return edges_[slot(e)].T;
}

void TfTree::clear()
{
    std::lock_guard<std::mutex> lk(m_);
    for (EdgeState &s : edges_) {
        s.T.reset();
        s.at = Stamp{};
    }
}

/* ---------- 帧名期望表与校验入口（工单 02） ---------- */

namespace {

// 对照表的一行 = 一帧。**期望的帧名只在这张表里出现一份**：协议再改帧名只改这里，
// 解析层与界面层一律调下面那个入口，不许自己抄一份期望值。
struct FrameNameExpectation {
    uint8_t     frameNo;       // 协议里的帧号 `AA NN …`
    const char *frameId;       // 期望的 frame_id（父系）；nullptr = 本帧没有这一侧
    const char *childFrameId;  // 期望的 child_frame_id（子帧）；nullptr = 本帧没有这一侧
    bool        fromPayload;   // 载荷里带不带帧名字段；false ⇒ 上面两个是**硬约定**，不比对
};

constexpr FrameNameExpectation kFrameNameExpectations[] = {
    // 帧号   期望 frame_id（父系）  期望 child_frame_id（子帧）  载荷里带帧名吗   是哪一帧
    {0x01, "base_init_map", "base_link",  true },  // 机器人位姿：图的原点系 → 机器人中心（父系曾是 world）
    {0x02, "base_init_map", "base_init",  true },  // 重定位结果：图的原点系 → 本次开机那一刻的原点
    {0x03, "base_init_map", nullptr,      true },  // 3D 点云：载荷里只有父系这一个字段
    {0x05, "base_link",     nullptr,      false},  // 2D 扫描：**硬约定**，载荷里没有帧名字段 ⇒ 不可校
    {0x06, "base_init_map", nullptr,      false},  // 2D 地图：**硬约定**，载荷里没有帧名字段 ⇒ 不可校
};

} // namespace

// 见 tftree.h 里那段说明。0x05 / 0x06（以及表外帧号）恒接受，那是「**不可校验**」而非
// 「校验通过」——它们返回 true 不是因为比中了，而是因为**根本没有可比的东西**。
bool frameNamesAccepted(uint8_t frameNo, std::string_view frameId, std::string_view childFrameId)
{
    for (const FrameNameExpectation &e : kFrameNameExpectations) {
        if (e.frameNo != frameNo)
            continue;

        // 载荷里没有帧名字段 ⇒ **不可校验，一律接受**——而且连传进来的那两个名字都不读：
        // 读了也只能是空的或别人乱塞的，拿它判事就成了伪造一个「校验过了」的假象。
        if (!e.fromPayload)
            return true;

        // 逐侧比，期望为 nullptr 的那一侧（本帧没有这个字段）不参与。
        // 用 string_view 比 = 逐字节相等，大小写不折叠、不做任何归一化：帧名是协议里的字面值。
        if (e.frameId && frameId != std::string_view(e.frameId))
            return false;
        if (e.childFrameId && childFrameId != std::string_view(e.childFrameId))
            return false;
        return true;
    }

    // 表外的帧号：本模块对它没有期望（协议里带帧名字段的只有上面那几帧），判接受。
    // 与 0x05 / 0x06 是同一条道理：**不可校验就是不可校验**，不往身上记一笔「已校验」。
    return true;
}
