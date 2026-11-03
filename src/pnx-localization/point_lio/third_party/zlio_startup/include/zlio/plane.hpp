/**
 * @file plane.hpp
 * @brief 定义支持点、平面观测及其不确定性表示。
 */
#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <utility>
#include <vector>

#include "zlio/types.hpp"

namespace zlio {

/// 平面第二散布特征值的数值下限，单位为平方米，用于确认二维切向支撑。
inline constexpr double kMinPlaneTangentialVariance = 1e-12;

/// 管理同批支持点共享的不可变导航位姿协方差。
class MapPoseCovariance {
    struct Block {
        const Mat6 covariance;
        std::atomic<std::size_t> references{1};
        /// 保存共享位姿协方差并建立首个引用。
        explicit Block(const Mat6& value) : covariance(value) {}
    };
    Block* block_ = nullptr;

public:
    /// 创建空的位姿协方差引用。
    MapPoseCovariance() noexcept = default;
    /// 创建由当前引用持有的位姿协方差。
    explicit MapPoseCovariance(const Mat6& value) : block_(new Block(value)) {}
    /// 取得同一位姿协方差的共享所有权。
    MapPoseCovariance(const MapPoseCovariance& other) noexcept : block_(other.block_) {
        if (block_)
            block_->references.fetch_add(1, std::memory_order_relaxed);
    }
    /// 转移位姿协方差的共享引用。
    MapPoseCovariance(MapPoseCovariance&& other) noexcept
        : block_(std::exchange(other.block_, nullptr)) {}
    /// 释放引用，并在最后一个引用销毁时回收协方差。
    ~MapPoseCovariance() {
        if (block_ && block_->references.fetch_sub(1, std::memory_order_acq_rel) == 1)
            delete block_;
    }
    /// 以传入引用替换当前位姿协方差。
    MapPoseCovariance& operator=(MapPoseCovariance other) noexcept {
        std::swap(block_, other.block_);
        return *this;
    }
    /// 返回位姿协方差地址，空引用返回空指针。
    const Mat6* get() const noexcept { return block_ ? &block_->covariance : nullptr; }
    /// 访问已持有的位姿协方差。
    const Mat6& operator*() const noexcept { return block_->covariance; }
    /// 判断引用是否持有位姿协方差。
    explicit operator bool() const noexcept { return block_ != nullptr; }
    /// 释放当前引用并恢复为空。
    void reset() noexcept {
        MapPoseCovariance empty;
        std::swap(block_, empty.block_);
    }
};

/// 保存世界系支持点、独立测量协方差及可选的共同位姿误差。
struct MapPoint {
    Vec3 point;
    Mat3 independent_covariance;
    /// 同批支持点的导航位姿协方差。
    MapPoseCovariance shared_pose_covariance{};
    /// 合成独立测量项与位姿传播项的点边缘协方差。
    Mat3 marginalCovariance() const;
};

/// 按升序散布特征值检查绝对厚度和相对平面形状。
bool planarSpectrum(const Vec3& eigenvalues, double threshold, double eigenvalue_ratio,
                    double middle_ratio);

/// 保存点面匹配的几何量及观测方差分量。
struct PlaneMatch {
    Vec3 normal;
    double residual = 0, plane_variance = 0;
    /// 未解释的表面散布，与已估计平面参数的不确定度分别保留。
    double surface_variance = 0;
    /// 面参数按两个法向切向坐标及法向中心偏移排列。
    Vec3 parameter_jacobian = Vec3::Zero();
    const std::array<double, 6>* parameter_covariance = nullptr;
};

/// 将观测变换到世界系，分开保存独立测量噪声与同批共同位姿误差。
std::vector<MapPoint> transformForMap(const std::vector<Observation>& points,
                                      const Estimate& estimate);

}  // namespace zlio
