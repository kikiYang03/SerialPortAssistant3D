#include "tftree.h"

void TfTree::setMapToBaseLink(const Mat4& T)
{
    std::lock_guard<std::mutex> lk(m_);
    T_map_base_link_ = T;
}

void TfTree::setMapToLaser(const Mat4& T)
{
    std::lock_guard<std::mutex> lk(m_);
    T_map_laser_ = T;
}

std::optional<Mat4> TfTree::getMapToBaseLink() const
{
    std::lock_guard<std::mutex> lk(m_);
    return T_map_base_link_;
}

std::optional<Mat4> TfTree::getMapToLaser() const
{
    std::lock_guard<std::mutex> lk(m_);
    return T_map_laser_;
}

bool TfTree::hasMapToBaseLink() const
{
    std::lock_guard<std::mutex> lk(m_);
    return T_map_base_link_.has_value();
}

bool TfTree::hasMapToLaser() const
{
    std::lock_guard<std::mutex> lk(m_);
    return T_map_laser_.has_value();
}

void TfTree::clear()
{
    std::lock_guard<std::mutex> lk(m_);
    T_map_base_link_.reset();
    T_map_laser_.reset();
}