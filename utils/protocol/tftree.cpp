#include "tftree.h"

/* ---------------- 两条边的存放与摆放 ---------------- */

void TfTree::setPose(Edge e, const Vec3 &t, const Quat &q, Stamp at)
{
    // 位姿 → 矩阵这一折只在这里做；调用点不得自建矩阵（自建就会出现「箭头一个矩阵、
    // 扫描另一个矩阵」那种两边各画各的静默错）。
    Mat4 T = Mat4::Identity();
    T.block<3, 3>(0, 0) = q.toRotationMatrix();
    T.block<3, 1>(0, 3) = t;

    std::lock_guard<std::mutex> lk(m_);
    EdgeState &s = edges_[static_cast<int>(e)];
    s.T  = T;
    s.at = at;
}

bool TfTree::isFresh(Edge e, Stamp now) const
{
    std::lock_guard<std::mutex> lk(m_);
    const EdgeState &s = edges_[static_cast<int>(e)];
    if (!s.T) return false;                       // 从没喂过：一律不新鲜
    return (now - s.at) <= kFreshWindow;          // 恰好 500 ms 仍算新鲜
}

std::optional<Vec3> TfTree::placeInMap(Edge e, const Vec3 &p, Stamp now) const
{
    std::lock_guard<std::mutex> lk(m_);
    const EdgeState &s = edges_[static_cast<int>(e)];

    // 边没喂过、或已过窗：返回空。**不兜底**——恒等兜底会把点摆到图原点，
    // 界面上看起来就是「机器人在图的原点」，一个不会报错的假象。
    if (!s.T) return std::nullopt;
    if ((now - s.at) > kFreshWindow) return std::nullopt;

    return Vec3(s.T->block<3, 3>(0, 0) * p + s.T->block<3, 1>(0, 3));   // R·p + t
}

std::optional<Mat4> TfTree::edgeTransform(Edge e) const
{
    // 刻意**不判新鲜度**：渲染要的是「最后一帧停在哪儿」，陈旧与否由调用方按 isFresh 定。
    std::lock_guard<std::mutex> lk(m_);
    return edges_[static_cast<int>(e)].T;
}

void TfTree::clear()
{
    std::lock_guard<std::mutex> lk(m_);
    for (int i = 0; i < kEdgeCount; ++i) {
        edges_[i].T.reset();
        edges_[i].at = Stamp{};
    }
}

/* ---------------- 帧名期望表 ---------------- */

// 本表是帧名口径的**唯一落点**：协议再改帧名只改这里。
// `nullptr` = 本帧的载荷里没有这一侧的字段，**没有可比的东西**（不是「校验通过」）。
namespace {

struct FrameNameExpectation {
    uint8_t frameNo;
    const char *parent;   // 期望的父系；nullptr = 本帧不比父系
    const char *child;    // 期望的子系；nullptr = 本帧不比子系
};

constexpr FrameNameExpectation kFrameNameExpectations[] = {
    // 0x01 机器人位姿：父系 v2.7/14 起回归 base_init_map —— 载荷字段没变，
    // 所以「模块还是发 world」时**不读帧名就发现不了**，这行是本次唯一能挡住它的地方。
    {0x01, "base_init_map", "base_link"},
    // 0x02 重定位结果：父系 = 图的原点系，子帧 = **本次开机**的原点。
    // 子帧这一侧是真门：写成 base_link 就是把重定位结果当成机器人位姿来摆。
    {0x02, "base_init_map", "base_init"},
    // 0x03 3D 点云：载荷里只有 frame_id 这一个帧名字段，子系那一侧**不许收紧**——
    // 收紧成也要比子系，每帧点云都会被丢掉，界面上是点云整条不画（静默错误）。
    {0x03, "base_init_map", nullptr},
};

}  // namespace

bool frameNamesAccepted(uint8_t frameNo,
                        std::string_view frameId,
                        std::string_view childFrameId)
{
    for (const FrameNameExpectation &e : kFrameNameExpectations) {
        if (e.frameNo != frameNo) continue;
        // 逐字节比：不改大小写、不去空白、不做任何归一化（线上发的是什么就比什么）。
        if (e.parent && frameId != e.parent) return false;
        if (e.child && childFrameId != e.child) return false;
        return true;
    }
    // 表外的帧号（含 0x05 / 0x06 这两个载荷里根本没有帧名字段的）：本模块对它没有期望，
    // 也就无从校验 ⇒ 判接受。**这不是「校验通过」**，是「没有可比的东西」。
    return true;
}
