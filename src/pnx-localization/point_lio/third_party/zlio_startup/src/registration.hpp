/**
 * @file registration.hpp
 * @brief 声明扫描粗配准及其局部质心地图。
 */
#pragma once

#include <unordered_map>

#include "zlio/config.hpp"
#include "zlio/voxel.hpp"
#include "zlio/window.hpp"

namespace zlio {
class Registration {
public:
    /// 建立拥有粗地图质心及体素索引的配准器。
    explicit Registration(RegistrationConfig config) : config_(config) {}
    /// 对去畸变点云降采样、配准并更新粗地图。
    void update(WindowState& state, std::vector<Vec3> points, const Pose& extrinsic);
    /// 同时清理粗地图、索引及边界回退状态。
    void clear() {
        points_.clear();
        centroid_indices_.clear();
        index_valid_ = true;
    }

private:
    RegistrationConfig config_;
    std::vector<Vec3> points_;
    std::unordered_map<VoxelKey, std::size_t, VoxelHash> centroid_indices_;
    bool index_valid_ = true;
    /// 按体素聚合已有质心和新扫描点。
    void updateCentroids(const WindowState& state, const std::vector<Vec3>& points);
};

}  // namespace zlio
