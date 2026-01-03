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
