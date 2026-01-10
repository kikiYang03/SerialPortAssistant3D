#ifndef TFTREE_H
#define TFTREE_H

#include <QString>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <optional>
#include <mutex>
#include <map>

using Vec3 = Eigen::Vector3d;
using Quat = Eigen::Quaterniond;
using Mat4 = Eigen::Matrix4d;

class TfTree
{
public:
    void setTransform(const QString &parent,
                      const QString &child,
                      const Vec3 &t,
                      const Quat &q);

    // 固定路径 lookup，O(1)
    std::optional<Mat4> lookupMapCameraInit() const;
    std::optional<Mat4> lookupCameraInitBody() const;


    std::optional<Mat4> lookup(const QString &target,
                               const QString &source) const
    {
        if(target == source) return Mat4::Identity();
        std::lock_guard<std::mutex> lk(m_);
        return dfs(target, source);
    }

    // 给 paint 时遍历用
    using Edge = std::pair<QString,QString>;
    std::map<Edge,Mat4> allEdges() const
    { std::lock_guard<std::mutex> lk(m_); return data_; }

private:
    mutable std::mutex m_;
    std::map<Edge,Mat4> data_;

    std::optional<Mat4> dfs(const QString &cur,
                            const QString &target) const;

    // 新增：只存最新
    std::optional<Mat4> T_map_camInit_;      // map -> camera_init
    std::optional<Mat4> T_camInit_body_;     // camera_init -> body
    std::optional<Mat4> T_body_baselink;     // body -> base_link
};
#endif // TFTREE_H
