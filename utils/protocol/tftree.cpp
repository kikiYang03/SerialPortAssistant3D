#include "tftree.h"


std::optional<Mat4> TfTree::dfs(const QString &target,
                                const QString &source) const
{
    if (target == source) return Mat4::Identity();
    for (const auto &[edge, T] : data_) {
        const QString &par = edge.first;
        const QString &chi = edge.second;
        if (chi == target) {
            auto up = dfs(par, source);
            if (!up) continue;
            return (*up) * T;
        }
    }
    return std::nullopt;
}

void TfTree::setTransform(const QString &parent,
                          const QString &child,
                          const Vec3 &t,
                          const Quat &q)
{
    Mat4 T = Mat4::Identity();
    T.block<3,3>(0,0) = q.matrix();
    T.block<3,1>(0,3) = t;

    std::lock_guard<std::mutex> lk(m_);
    /* 正向边 */
    data_[{parent, child}] = T;
    /* 反向边 */
    data_[{child, parent}] = T.inverse();

    /* 继续保留你的快速路径缓存 */
    if (parent == "map" && child == "camera_init") {
        T_map_camInit_ = T;
    } else if (parent == "camera_init" && child == "body") {
        T_camInit_body_ = T;
    } else if (parent == "body" && child == "base_link") {
        T_body_baselink = T;
    }
}

std::optional<Mat4> TfTree::lookupMapCameraInit() const
{
    std::lock_guard<std::mutex> lk(m_);
    return T_map_camInit_;
}

std::optional<Mat4> TfTree::lookupCameraInitBody() const
{
    std::lock_guard<std::mutex> lk(m_);
    return T_camInit_body_;
}
