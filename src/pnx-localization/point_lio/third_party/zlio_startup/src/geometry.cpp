/**
 * @file geometry.cpp
 * @brief 构造启动窗口的跨帧平面约束并计算几何导数、代价及方向支撑。
 */
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <numeric>
#include <optional>
#include <stdexcept>
#include <unordered_map>

#include "bootstrap.hpp"
#include "zlio/config.hpp"
#include "zlio/plane.hpp"
#include "zlio/state.hpp"
#include "zlio/voxel.hpp"

namespace zlio {
namespace {
inline const Mat3 kIdentity3 = Mat3::Identity();
inline constexpr double kPlaneWeight = 1.;
template <class T>
using AlignedVector = std::vector<T, Eigen::aligned_allocator<T>>;
/// 保存点集的公开充分统计量，并提供对应的值运算。
struct PointMoments {
    Mat3 second_moment;
    Vec3 point_sum;
    int count;
    /// 初始化空点集的充分统计量。
    PointMoments();
    /// 累加一个点的零阶、一阶和二阶矩。
    void push(const Vec3& point);
    /// 由充分统计量形成中心化散布矩阵。
    Mat3 covariance() const;
    /// 合并另一组点的充分统计量。
    PointMoments& operator+=(const PointMoments& other);
    /// 将局部点矩按帧位姿转换到世界坐标。
    void transform(const PointMoments& local, const WindowState& state);
};
class PlaneConstraints {
private:
    struct Workspace;
    struct Plane {
        struct Support {
            int frame;
            PointMoments moments;
        };
        std::vector<Support> frames;
        Vec3 eigenvalues;
        Mat3 eigenvectors;
        PointMoments aggregate;
    };
    std::vector<Plane> planes_;
    double fixed_cost_ = 0;  // 单帧平面的常数代价。
    int frame_count_;

public:
    /// 返回具有跨帧信息的有效平面数量。
    std::size_t size() const { return planes_.size(); }
    /// 返回当前几何窗口的帧数。
    int frameCount() const { return frame_count_; }
    /// 汇总当前缓存法向的三维方向支撑。
    Mat3 normalSupport() const;
    /// 检查有效平面的帧间共享图是否连通。
    bool connected() const;
    /// 建立给定窗口帧数的空几何约束集合。
    explicit PlaneConstraints(int frame_count);

    /// 保存有效平面，单帧自由平面仅计入常数代价。
    void add(const std::vector<PointMoments>& frame_moments, const Vec3& eigenvalues,
             const Mat3& eigenvectors, const PointMoments& aggregate);

    /// 组装指定半开平面区间的位姿 Hessian、梯度和代价。
    void linearize(const std::vector<WindowState>& states, int begin, int end,
                   Eigen::MatrixXd& hessian, Eigen::VectorXd& gradient, double& residual);

    /// 重算指定半开区间的几何代价与特征缓存。
    void evaluateCost(const std::vector<WindowState>& states, int begin, int end, double& residual);
};
/// 将点数及一、二阶矩初始化为零。
PointMoments::PointMoments() {
    second_moment.setZero();
    point_sum.setZero();
    count = 0;
}
/// 将一个点累加到充分统计量。
void PointMoments::push(const Vec3& point) {
    ++count;
    second_moment += point * point.transpose();
    point_sum += point;
}
/// 从点矩计算关于点集均值的散布矩阵。
Mat3 PointMoments::covariance() const {
    const Vec3 center = point_sum / count;
    return second_moment / count - center * center.transpose();
}
/// 合并另一组点的充分统计量。
PointMoments& PointMoments::operator+=(const PointMoments& other) {
    second_moment += other.second_moment;
    point_sum += other.point_sum;
    count += other.count;
    return *this;
}
/// 在不展开原始点的情况下变换点矩。
void PointMoments::transform(const PointMoments& local, const WindowState& state) {
    count = local.count;
    point_sum = state.rotation * local.point_sum + count * state.position;
    const Mat3 mixed_moment = state.rotation * local.point_sum * state.position.transpose();
    second_moment = state.rotation * local.second_moment * state.rotation.transpose() +
                    mixed_moment + mixed_moment.transpose() +
                    count * state.position * state.position.transpose();
}
/// 保存几何约束对应的窗口帧数。
PlaneConstraints::PlaneConstraints(int frame_count) : frame_count_(frame_count) {
}

/// 区分跨帧几何约束与单帧不变常数并保存平面。
void PlaneConstraints::add(const std::vector<PointMoments>& frame_moments, const Vec3& eigenvalues,
                           const Mat3& eigenvectors, const PointMoments& aggregate) {
    const auto participating =
        std::count_if(frame_moments.begin(), frame_moments.end(),
                      [](const PointMoments& frame) { return frame.count > 0; });
    // 单帧自由平面的特征值不随刚体位姿改变，不能提供帧间约束。
    if (participating == 0)
        return;
    if (participating == 1) {
        fixed_cost_ += kPlaneWeight * eigenvalues[0];
        return;
    }
    Plane plane;
    plane.eigenvalues = eigenvalues;
    plane.eigenvectors = eigenvectors;
    plane.aggregate = aggregate;
    plane.frames.reserve(participating);
    for (int i = 0; i < frame_count_; ++i)
        if (frame_moments[i].count)
            plane.frames.push_back({i, frame_moments[i]});
    planes_.push_back(std::move(plane));
}

/// 按共享平面合并帧集合并检查整窗连接性。
bool PlaneConstraints::connected() const {
    if (frame_count_ == 0)
        return false;
    std::vector<int> component(frame_count_);
    std::iota(component.begin(), component.end(), 0);
    const auto root = [&](int frame) {
        while (component[frame] != frame)
            frame = component[frame];
        return frame;
    };
    for (const auto& plane : planes_) {
        int anchor = -1;
        for (const auto& support : plane.frames) {
            const int frame = support.frame;
            if (anchor < 0)
                anchor = frame;
            else
                component[root(frame)] = root(anchor);
        }
    }
    for (int frame = 1; frame < frame_count_; ++frame)
        if (root(frame) != root(0))
            return false;
    return true;
}

struct PlaneConstraints::Workspace {
    /// 为单个分区预留各帧几何导数工作区。
    explicit Workspace(int count)
        : rotation_derivatives(count),
          rotation_normal_products(count),
          scatter_normal_jacobians(count) {}
    /// 由平面谱建立法向、中心及法向变化的曲率项。
    void prepare(const Plane& plane) {
        total_points = plane.aggregate.count;
        centroid = plane.aggregate.point_sum / total_points;
        normal = plane.eigenvectors.col(0);
        normal_outer_product = normal * normal.transpose();
        normal_curvature.setZero();
        for (int i = 1; i < 3; ++i) {
            const Vec3 direction = plane.eigenvectors.col(i);
            normal_curvature += 2.0 / (plane.eigenvalues[0] - plane.eigenvalues[i]) * direction *
                                direction.transpose();
        }
    }
    /// 组装一个平面内单帧的梯度及对角位姿块。
    void addFrame(const PointMoments& moments, const WindowState& state, int i,
                  Eigen::MatrixXd& hessian, Eigen::VectorXd& gradient) {
        // 计算当前帧点矩对平面散布的导数。
        const double weight = kPlaneWeight;
        Eigen::Matrix3d second_moment = moments.second_moment;
        Eigen::Vector3d point_sum = moments.point_sum;
        Eigen::Matrix3d rotation = state.rotation;
        double frame_points = moments.count;

        Eigen::Matrix3d point_sum_cross;
        point_sum_cross = skew(point_sum);
        Eigen::Vector3d local_normal = rotation.transpose() * normal;
        Eigen::Matrix3d local_normal_cross;
        local_normal_cross = skew(local_normal);

        Eigen::Vector3d projected_second_moment = second_moment * local_normal;
        rotation_derivatives[i] = point_sum_cross * local_normal;
        rotation_normal_products[i] = rotation_derivatives[i] * normal.transpose();

        Eigen::Vector3d relative_position = state.position - centroid;
        double normal_offset = normal.dot(relative_position);

        Eigen::Matrix3d rotation_moment_term =
            skew(projected_second_moment) + point_sum_cross * normal_offset;
        Eigen::Vector3d centered_point_sum =
            rotation * point_sum + frame_points * relative_position;
        scatter_normal_jacobians[i].block<3, 3>(0, 0) =
            (rotation * second_moment + relative_position * point_sum.transpose()) *
                local_normal_cross -
            rotation * rotation_moment_term;
        scatter_normal_jacobians[i].block<3, 3>(0, 3) =
            centered_point_sum * normal.transpose() + centered_point_sum.dot(normal) * kIdentity3;
        scatter_normal_jacobians[i] /= total_points;

        const Eigen::Matrix<double, 6, 1> pose_gradient =
            scatter_normal_jacobians[i].transpose() * normal;
        gradient.block<6, 1>(6 * i, 0) += weight * pose_gradient;

        // 累加法向曲率及旋转、平移的二阶项。
        const Eigen::Matrix3d rotation_translation_block =
            2.0 / total_points * (1.0 - frame_points / total_points) * rotation_normal_products[i];
        Eigen::Matrix<double, 6, 6> pose_block = scatter_normal_jacobians[i].transpose() *
                                                 normal_curvature * scatter_normal_jacobians[i];
        pose_block.block<3, 3>(0, 0) +=
            2.0 / total_points * (rotation_moment_term - local_normal_cross * second_moment) *
                local_normal_cross -
            2.0 / total_points / total_points * rotation_derivatives[i] *
                rotation_derivatives[i].transpose() -
            0.5 * skew(pose_gradient.block<3, 1>(0, 0));
        pose_block.block<3, 3>(0, 3) += rotation_translation_block;
        pose_block.block<3, 3>(3, 0) += rotation_translation_block.transpose();
        pose_block.block<3, 3>(3, 3) +=
            2.0 / total_points * (frame_points - frame_points * frame_points / total_points) *
            normal_outer_product;

        hessian.block<6, 6>(6 * i, 6 * i) += weight * pose_block;
    }
    /// 组装同一平面所连接帧之间的上三角位姿块。
    void addCrossTerms(const Plane& plane, Eigen::MatrixXd& hessian) const {
        const double weight = kPlaneWeight;
        for (std::size_t a = 0; a < plane.frames.size(); ++a) {
            const int i = plane.frames[a].frame;
            const double frame_points = plane.frames[a].moments.count;
            for (std::size_t b = a + 1; b < plane.frames.size(); ++b) {
                const int j = plane.frames[b].frame;
                const double other_points = plane.frames[b].moments.count;
                Eigen::Matrix<double, 6, 6> pose_block = scatter_normal_jacobians[i].transpose() *
                                                         normal_curvature *
                                                         scatter_normal_jacobians[j];
                pose_block.block<3, 3>(0, 0) += -2.0 / total_points / total_points *
                                                rotation_derivatives[i] *
                                                rotation_derivatives[j].transpose();
                pose_block.block<3, 3>(0, 3) +=
                    -2.0 * other_points / total_points / total_points * rotation_normal_products[i];
                pose_block.block<3, 3>(3, 0) += -2.0 * frame_points / total_points / total_points *
                                                rotation_normal_products[j].transpose();
                pose_block.block<3, 3>(3, 3) += -2.0 * frame_points * other_points / total_points /
                                                total_points * normal_outer_product;

                hessian.block<6, 6>(6 * i, 6 * j) += weight * pose_block;
            }
        }
    }
    int total_points = 0;
    Vec3 centroid, normal;
    Mat3 normal_outer_product, normal_curvature;
    AlignedVector<Vec3> rotation_derivatives;
    AlignedVector<Mat3> rotation_normal_products;
    AlignedVector<Eigen::Matrix<double, 3, 6>> scatter_normal_jacobians;
};

/// 按固定顺序累加分区导数并补齐对称跨帧块。
void PlaneConstraints::linearize(const std::vector<WindowState>& states, int begin, int end,
                                 Eigen::MatrixXd& hessian, Eigen::VectorXd& gradient,
                                 double& cost) {
    hessian.setZero();
    gradient.setZero();
    cost = begin == 0 ? fixed_cost_ : 0;
    Workspace workspace(frame_count_);
    for (int index = begin; index < end; ++index) {
        const auto& plane = planes_[index];
        workspace.prepare(plane);
        for (const auto& support : plane.frames)
            workspace.addFrame(support.moments, states[support.frame], support.frame, hessian,
                               gradient);
        workspace.addCrossTerms(plane, hessian);
        cost += kPlaneWeight * plane.eigenvalues[0];
    }
    for (int i = 1; i < frame_count_; ++i)
        for (int j = 0; j < i; ++j)
            hessian.block<6, 6>(6 * i, 6 * j) = hessian.block<6, 6>(6 * j, 6 * i).transpose();
}

/// 按试探位姿变换点矩并更新分区的谱和代价缓存。
void PlaneConstraints::evaluateCost(const std::vector<WindowState>& states, int begin, int end,
                                    double& residual) {
    residual = begin == 0 ? fixed_cost_ : 0;

    PointMoments transformed_points;

    for (int a = begin; a < end; a++) {
        const auto& frame_moments = planes_[a].frames;
        PointMoments aggregate;

        for (const auto& support : frame_moments) {
            transformed_points.transform(support.moments, states[support.frame]);
            aggregate += transformed_points;
        }

        Eigen::Vector3d centroid = aggregate.point_sum / aggregate.count;
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> decomposition(
            aggregate.second_moment / aggregate.count - centroid * centroid.transpose());
        Eigen::Vector3d eigenvalues = decomposition.eigenvalues();

        planes_[a].eigenvalues = decomposition.eigenvalues();
        planes_[a].eigenvectors = decomposition.eigenvectors();
        planes_[a].aggregate = aggregate;

        residual += kPlaneWeight * eigenvalues[0];
    }
}
/// 累加有效平面的法向外积作为方向支撑。
Mat3 PlaneConstraints::normalSupport() const {
    Mat3 support = Mat3::Zero();
    for (const auto& plane : planes_)
        support += plane.eigenvectors.col(0) * plane.eigenvectors.col(0).transpose();
    return support;
}

struct IndexedPoint {
    Vec3 local, world;
    int frame;
};
/// 递归细分体素并保留满足谱判据的平面统计量。
void planeFactors(const std::vector<IndexedPoint>& points, int depth, const Vec3& center,
                  double size, double threshold, const StartupOptimizationConfig& config,
                  PlaneConstraints& factors) {
    if (points.size() <= config.minimum_points)
        return;
    PointMoments aggregate;
    for (const auto& p : points)
        aggregate.push(p.world);
    Eigen::SelfAdjointEigenSolver<Mat3> eigen(aggregate.covariance());
    Vec3 values = eigen.eigenvalues();
    const Mat3& vectors = eigen.eigenvectors();
    if (!values.allFinite())
        return;
    if (values[0] < threshold && values[0] < config.plane_eigenvalue_ratio * values[2]) {
        if (!planarSpectrum(values, threshold, config.plane_eigenvalue_ratio,
                            config.plane_middle_ratio))
            return;
        std::vector<PointMoments> clusters(factors.frameCount());
        for (const auto& point : points)
            clusters[point.frame].push(point.local);
        factors.add(clusters, values, vectors, aggregate);
    } else if (depth < config.maximum_depth) {
        std::array<std::vector<IndexedPoint>, 8> children;
        for (const auto& p : points) {
            const int index = 4 * (p.world.x() > center.x()) + 2 * (p.world.y() > center.y()) +
                              (p.world.z() > center.z());
            children[index].push_back(p);
        }
        for (int i = 0; i < 8; ++i) {
            const Vec3 offset((i & 4) ? 1 : -1, (i & 2) ? 1 : -1, (i & 1) ? 1 : -1);
            planeFactors(children[i], depth + 1, center + size * .25 * offset, size * .5, threshold,
                         config, factors);
        }
    }
}
/// 按当前世界点分组构建一轮窗口平面约束。
PlaneConstraints buildPlaneConstraints(const std::vector<std::vector<Vec3>>& clouds,
                                       const std::vector<WindowState>& states, bool tight,
                                       const StartupOptimizationConfig& config) {
    std::unordered_map<VoxelKey, std::vector<IndexedPoint>, VoxelHash> roots;
    for (int i = 0; i < static_cast<int>(states.size()); ++i)
        for (const Vec3& local : clouds[i]) {
            const Vec3 world = states[i].rotation * local + states[i].position;
            roots[voxelKey(world, config.voxel_size)].push_back({local, world, i});
        }
    PlaneConstraints result(static_cast<int>(states.size()));
    for (const auto& root : roots)
        planeFactors(root.second, 0, voxelCenter(root.first, config.voxel_size), config.voxel_size,
                     config.plane_variance[tight ? 1 : 0], config, result);
    return result;
}
}  // namespace

struct StartupPlaneModel::Impl {
    StartupOptimizationConfig config;
    std::optional<PlaneConstraints> planes;
    /// 固定平面模型的构建配置。
    explicit Impl(const StartupOptimizationConfig& value) : config(value) {}
};

/// 建立尚未构造约束的窗口几何实现。
StartupPlaneModel::StartupPlaneModel(const StartupOptimizationConfig& config)
    : impl_(std::make_unique<Impl>(config)) {
}
/// 释放全部平面统计量及当前几何缓存。
StartupPlaneModel::~StartupPlaneModel() = default;

/// 按当前点云和粗细阶段完整重建几何关联。
void StartupPlaneModel::rebuild(const std::vector<std::vector<Vec3>>& clouds,
                                const std::vector<WindowState>& states, bool refined) {
    impl_->planes.emplace(buildPlaneConstraints(clouds, states, refined, impl_->config));
}

/// 返回当前有效跨帧平面数量。
std::size_t StartupPlaneModel::size() const {
    return impl_->planes ? impl_->planes->size() : 0;
}

/// 返回已构建几何窗口的帧数。
int StartupPlaneModel::frameCount() const {
    return impl_->planes ? impl_->planes->frameCount() : 0;
}

/// 检查当前有效平面是否连接整个窗口。
bool StartupPlaneModel::connected() const {
    return impl_->planes && impl_->planes->connected();
}

/// 读取已提交几何缓存的方向支撑，未构建时报告错误。
Mat3 StartupPlaneModel::normalSupport() const {
    if (!impl_->planes)
        throw std::logic_error("window constraints have not been constructed");
    return impl_->planes->normalSupport();
}

/// 将独立分区的导数组装交给私有几何约束。
void StartupPlaneModel::linearize(const std::vector<WindowState>& states, int begin, int end,
                                  Eigen::MatrixXd& hessian, Eigen::VectorXd& gradient,
                                  double& cost) {
    impl_->planes->linearize(states, begin, end, hessian, gradient, cost);
}

/// 更新独立分区的试探几何缓存与代价。
void StartupPlaneModel::evaluateCost(const std::vector<WindowState>& states, int begin, int end,
                                     double& cost) {
    impl_->planes->evaluateCost(states, begin, end, cost);
}

}  // namespace zlio
