/**
 * @file registration.cpp
 * @brief 以近邻平面约束计算启动扫描位姿并更新质心地图。
 */
#include "registration.hpp"

#include <Eigen/QR>
#include <limits>
#include <nanoflann.hpp>
#include <stdexcept>

#include "zlio/state.hpp"

namespace zlio {
namespace {
/// 按输入顺序对各体素中的点求均值。
std::vector<Vec3> centroids(const std::vector<Vec3>& points, double size) {
    struct Cell {
        Vec3 sum = Vec3::Zero();
        std::size_t count = 0;
    };
    std::unordered_map<VoxelKey, Cell, VoxelHash> cells;
    for (const auto& point : points) {
        auto& cell = cells[voxelKey(point, size)];
        cell.sum += point;
        ++cell.count;
    }
    std::vector<Vec3> result;
    result.reserve(cells.size());
    for (const auto& entry : cells)
        result.push_back(entry.second.sum / entry.second.count);
    return result;
}

struct CloudView {
    const std::vector<Vec3>& points;
    /// 向近邻索引提供当前粗地图点数。
    std::size_t kdtree_get_point_count() const { return points.size(); }
    /// 读取指定粗地图点的一个坐标分量。
    double kdtree_get_pt(std::size_t i, std::size_t axis) const { return points[i][axis]; }
    /// 通知近邻索引自行计算粗地图包围盒。
    template <class Box>
    bool kdtree_get_bbox(Box&) const {
        return false;
    }
};
using Tree = nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<double, CloudView>,
                                                 CloudView, 3, std::size_t>;
struct PoseSystem {
    Mat6 information = Mat6::Zero();
    Vec6 gradient = Vec6::Zero();
};

/// 校验粗状态的名义量和协方差，阻止无效结果进入粗地图。
void validateState(const WindowState& state) {
    if (!state.rotation.allFinite() || !state.position.allFinite() || !state.velocity.allFinite() ||
        !state.gyro_bias.allFinite() || !state.accel_bias.allFinite() ||
        !state.gravity.allFinite() || !numericallyPositiveCovariance(state.covariance))
        throw NumericalError("Invalid coarse-registration state or covariance");
}

class RegistrationProblem {
public:
    /// 借用本帧粗地图和点云，建立近邻索引及复用 QR 工作区。
    RegistrationProblem(const std::vector<Vec3>& map, const std::vector<Vec3>& points,
                        const RegistrationConfig& config)
        : config_(config),
          map_{map},
          tree_(3, map_, {10}),
          points_(points),
          normals_(points.size()),
          offsets_(points.size(), -1),
          indices_(config.neighbors),
          distances_(config.neighbors),
          support_(config.neighbors, 3),
          plane_solver_(config.neighbors, 3),
          right_hand_side_(Eigen::VectorXd::Constant(config.neighbors, -1)) {}

    /// 按需重建点面关联并累加当前位姿的几何系统。
    PoseSystem linearize(const WindowState& state, bool rematch) {
        PoseSystem system;
        for (std::size_t i = 0; i < points_.size(); ++i) {
            const Vec3 world = state.rotation * points_[i] + state.position;
            if (rematch && !matchPlane(world, normals_[i], offsets_[i]))
                continue;
            if (offsets_[i] < 0)
                continue;
            Vec6 jacobian;
            jacobian.head<3>() = skew(points_[i]) * state.rotation.transpose() * normals_[i];
            jacobian.tail<3>() = normals_[i];
            system.information += jacobian * jacobian.transpose();
            system.gradient -= jacobian * (normals_[i].dot(world) + offsets_[i]);
        }
        return system;
    }

private:
    /// 从指定近邻数拟合平面并检查支持点的方程残差。
    bool matchPlane(const Vec3& world, Vec3& normal, double& offset) {
        offset = -1;
        if (tree_.knnSearch(world.data(), indices_.size(), indices_.data(), distances_.data()) !=
            indices_.size())
            return false;
        for (int k = 0; k < config_.neighbors; ++k)
            support_.row(k) = map_.points[indices_[k]];
        plane_solver_.compute(support_);
        const Vec3 plane = plane_solver_.solve(right_hand_side_);
        if (!plane.allFinite() || plane.norm() < 1e-12 ||
            (support_ * plane + Eigen::VectorXd::Ones(config_.neighbors)).cwiseAbs().maxCoeff() >
                config_.plane_residual_tolerance)
            return false;
        offset = 1 / plane.norm();
        normal = plane * offset;
        return true;
    }
    RegistrationConfig config_;
    CloudView map_;
    Tree tree_;
    const std::vector<Vec3>& points_;
    std::vector<Vec3> normals_;
    std::vector<double> offsets_;
    std::vector<std::size_t> indices_;
    std::vector<double> distances_;
    Eigen::Matrix<double, Eigen::Dynamic, 3> support_;
    Eigen::ColPivHouseholderQR<Eigen::Matrix<double, Eigen::Dynamic, 3>> plane_solver_;
    Eigen::VectorXd right_hand_side_;
};

/// 在传播先验上迭代粗点面更新并更新粗协方差。
void solveRegistration(WindowState& state, RegistrationProblem& problem,
                       const RegistrationConfig& config) {
    const WindowState prior = state;
    const Mat15 inverse = inversePositiveDefinite(state.covariance);
    int rematches = 0;
    bool rematch = true, converged = false;
    for (int iteration = 0; iteration < config.maximum_iterations; ++iteration) {
        const auto system = problem.linearize(state, rematch);
        Mat15 information = inverse / config.prior_covariance_scale;
        information.topLeftCorner<6, 6>() += system.information;
        const Mat15 gain = inversePositiveDefinite(information);
        const Eigen::Matrix<double, 15, 6> correction = gain.leftCols<6>() * system.information;
        const Vec15 difference = prior - state;
        const Vec15 delta =
            gain.leftCols<6>() * system.gradient + difference - correction * difference.head<6>();
        if (!delta.allFinite())
            throw NumericalError("Coarse-registration increment is non-finite");
        state += delta;
        rematch = false;
        // 旋转弧度与位置米数只控制数值迭代停止。
        if (delta.head<3>().norm() < config.rotation_tolerance &&
            delta.segment<3>(3).norm() < config.position_tolerance) {
            rematch = true;
            converged = true;
            ++rematches;
        }
        if (iteration == config.maximum_iterations - 2 && !converged)
            rematch = true;
        if (rematches >= 2 || iteration == config.maximum_iterations - 1) {
            // 由信息系统直接恢复等价后验，避免单位阵与近单位增益相减。
            state.covariance = gain / config.prior_covariance_scale;
            validateState(state);
            break;
        }
    }
}

}  // namespace

/// 更新新扫描触及的质心格，并维护体素索引。
void Registration::updateCentroids(const WindowState& state, const std::vector<Vec3>& points) {
    if (!index_valid_) {
        for (const Vec3& point : points)
            points_.push_back(state.rotation * point + state.position);
        points_ = centroids(points_, config_.voxel_size);
        centroid_indices_.clear();
        index_valid_ = true;
        for (std::size_t i = 0; i < points_.size(); ++i)
            index_valid_ &=
                centroid_indices_.emplace(voxelKey(points_[i], config_.voxel_size), i).second;
        return;
    }
    struct Cell {
        Vec3 sum = Vec3::Zero();
        std::size_t count = 0, slot = std::numeric_limits<std::size_t>::max();
    };
    std::unordered_map<VoxelKey, Cell, VoxelHash> updates;
    for (const Vec3& point : points) {
        const Vec3 world = state.rotation * point + state.position;
        const auto key = voxelKey(world, config_.voxel_size);
        auto [entry, added] = updates.try_emplace(key);
        auto& cell = entry->second;
        if (added) {
            const auto previous = centroid_indices_.find(key);
            if (previous != centroid_indices_.end()) {
                cell.slot = previous->second;
                // 已有质心以单位权重参与本批聚合。
                cell.sum = points_[cell.slot];
                cell.count = 1;
            }
        }
        cell.sum += world;
        ++cell.count;
    }
    for (const auto& entry : updates) {
        const auto& cell = entry.second;
        const Vec3 center = cell.sum / cell.count;
        if (cell.slot == std::numeric_limits<std::size_t>::max()) {
            centroid_indices_.emplace(entry.first, points_.size());
            points_.push_back(center);
        } else {
            points_[cell.slot] = center;
        }
        // 质心跨越体素边界时标记索引待重建。
        if (!(voxelKey(center, config_.voxel_size) == entry.first))
            index_valid_ = false;
    }
}

/// 在雷达系降采样后完成配准，并按接受位姿更新粗地图。
void Registration::update(WindowState& state, std::vector<Vec3> points, const Pose& extrinsic) {
    validateState(state);
    for (auto& point : points)
        point = extrinsic.inverseApply(point);
    points = centroids(points, config_.voxel_size);
    for (auto& point : points)
        point = extrinsic.apply(point);
    WindowState candidate = state;
    if (points_.size() >= config_.minimum_map_points) {
        RegistrationProblem problem(points_, points, config_);
        solveRegistration(candidate, problem, config_);
    }
    // 数值验收完成后才更新地图及调用方状态。
    validateState(candidate);
    updateCentroids(candidate, points);
    state = candidate;
}
}  // namespace zlio
