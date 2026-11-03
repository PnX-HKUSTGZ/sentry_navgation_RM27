/**
 * @file voxel.hpp
 * @brief 提供整型体素坐标、哈希及浮点空间到体素的统一映射。
 */
#pragma once

#include <cmath>

#include "zlio/types.hpp"

namespace zlio {

/// 三维整型体素坐标，不包含空间分辨率或父子层级。
struct VoxelKey {
    std::int64_t x, y, z;
    /// 判断两个键是否表示相同的三维体素。
    bool operator==(const VoxelKey& other) const {
        return x == other.x && y == other.y && z == other.z;
    }
};

/// 将三轴整数坐标组合为体素哈希值。
struct VoxelHash {
    /// 以无符号运算组合坐标，使负坐标的哈希行为保持明确。
    std::size_t operator()(const VoxelKey& key) const {
        const auto x = static_cast<std::uint64_t>(key.x);
        const auto y = static_cast<std::uint64_t>(key.y);
        const auto z = static_cast<std::uint64_t>(key.z);
        return static_cast<std::size_t>((x * 73856093ULL) ^ (y * 19349663ULL) ^ (z * 83492791ULL));
    }
};

/// 按各轴向下取整计算体素键；size 须为正，坐标商须可由 int64 表示。
inline VoxelKey voxelKey(const Vec3& point, double size) {
    return {static_cast<std::int64_t>(std::floor(point.x() / size)),
            static_cast<std::int64_t>(std::floor(point.y() / size)),
            static_cast<std::int64_t>(std::floor(point.z() / size))};
}

/// 返回指定体素的几何中心，单位与体素尺寸一致。
inline Vec3 voxelCenter(const VoxelKey& key, double size) {
    return size * Vec3(static_cast<double>(key.x) + .5, static_cast<double>(key.y) + .5,
                       static_cast<double>(key.z) + .5);
}

}  // namespace zlio
