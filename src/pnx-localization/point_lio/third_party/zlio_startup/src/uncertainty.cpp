/**
 * @file uncertainty.cpp
 * @brief 在固定规范下传播启动加权估计器的不确定度，并转换各帧导航误差边缘。
 */
#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <limits>
#include <unordered_map>

#include "bootstrap.hpp"
#include "plane_factor.hpp"
#include "solver.hpp"
#include "zlio/config.hpp"
#include "zlio/plane.hpp"
#include "zlio/state.hpp"
#include "zlio/voxel.hpp"

namespace zlio {
namespace {
struct Support {
    const Observation* observation;
    Vec3 world;
    int frame;
};
struct Recovery {
    const std::vector<WindowState>& states;
    const StartupOptimizationConfig& config;
    Eigen::MatrixXd information, score_covariance;
    std::size_t planes = 0;

    /// 消去局部平面参数并累加各帧的信息与梯度噪声。
    void accumulate(const std::vector<Support>& points, const Vec3& center,
                    const Mat3& eigenvectors) {
        std::vector<int> frames;
        for (const auto& point : points)
            if (std::find(frames.begin(), frames.end(), point.frame) == frames.end())
                frames.push_back(point.frame);
        if (frames.size() < 2)
            return;
        std::vector<Vec3> origins;
        std::vector<plane::MomentStatistics> moments(frames.size());
        origins.reserve(frames.size());
        for (const int frame : frames)
            origins.push_back(states[frame].rotation.transpose() *
                              (center - states[frame].position));
        for (const auto& point : points) {
            const std::size_t frame =
                std::find(frames.begin(), frames.end(), point.frame) - frames.begin();
            const auto& observation = *point.observation;
            moments[frame].add(MapPoint{observation.point, observation.covariance}, origins[frame]);
        }
        FreePlaneFactor factor(frames.size());
        for (std::size_t f = 0; f < frames.size(); ++f) {
            const auto& state = states[frames[f]];
            factor.addFrame(f, moments[f], origins[f], state.rotation, state.position, center,
                            eigenvectors);
        }
        auto plane = factor.eliminate();
        if (!plane.valid)
            return;
        // 启动几何代价为最小特征值，故半平方形式的权重为 2/N。
        const double weight = 2. / points.size();
        for (std::size_t i = 0; i < frames.size(); ++i)
            for (std::size_t j = 0; j < frames.size(); ++j) {
                information.block<6, 6>(15 * frames[i], 15 * frames[j]) +=
                    weight * plane.information.block<6, 6>(6 * i, 6 * j);
                score_covariance.block<6, 6>(15 * frames[i], 15 * frames[j]) +=
                    weight * weight * plane.score_covariance.block<6, 6>(6 * i, 6 * j);
            }
        ++planes;
    }
    /// 按初始化细阶段的体素分组与几何验收规则构造约束。
    void partition(const std::vector<Support>& points, const Vec3& center, double size, int depth) {
        if (points.size() <= config.minimum_points)
            return;
        Vec3 mean = Vec3::Zero();
        for (const auto& point : points)
            mean += point.world;
        mean /= points.size();
        Mat3 scatter = Mat3::Zero();
        for (const auto& point : points) {
            const Vec3 d = point.world - mean;
            scatter.noalias() += d * d.transpose();
        }
        scatter /= points.size();
        Eigen::SelfAdjointEigenSolver<Mat3> eigen(scatter);
        if (eigen.info() != Eigen::Success)
            return;
        const auto& values = eigen.eigenvalues();
        if (values[0] < config.plane_variance[1] &&
            values[0] < config.plane_eigenvalue_ratio * values[2]) {
            if (planarSpectrum(values, config.plane_variance[1], config.plane_eigenvalue_ratio,
                               config.plane_middle_ratio))
                accumulate(points, mean, eigen.eigenvectors());
        } else if (depth < config.maximum_depth) {
            std::array<std::vector<Support>, 8> children;
            for (const auto& p : points) {
                const int i = 4 * (p.world.x() > center.x()) + 2 * (p.world.y() > center.y()) +
                              (p.world.z() > center.z());
                children[i].push_back(p);
            }
            for (int i = 0; i < 8; ++i) {
                const Vec3 offset((i & 4) ? 1 : -1, (i & 2) ? 1 : -1, (i & 1) ? 1 : -1);
                partition(children[i], center + .25 * size * offset, .5 * size, depth + 1);
            }
        }
    }
};

/// 计算将世界重力旋转到负 z 轴的旋转矩阵。
Mat3 alignment(const Vec3& gravity) {
    return Eigen::Quaterniond::FromTwoVectors(gravity, -Vec3::UnitZ()).toRotationMatrix();
}
/// 检查规范化联合信息并恢复切空间中的估计协方差。
bool solveCovariance(const Recovery& recovery, const Eigen::Matrix<double, 3, 2>& tangent,
                     StartupCovariance& result, Eigen::MatrixXd& covariance) {
    Eigen::MatrixXd H = projectStartupMatrix(recovery.information, tangent);
    Eigen::MatrixXd B = projectStartupMatrix(recovery.score_covariance, tangent);
    H = .5 * (H + H.transpose()).eval();
    B = .5 * (B + B.transpose()).eval();
    if (!H.allFinite() || !B.allFinite()) {
        result.failure = StartupCovarianceFailure::NumericalFailure;
        result.reason = "nonfinite reduced system";
        return false;
    }
    result.failure = StartupCovarianceFailure::WeakInformation;
    if ((H.diagonal().array() <= 0).any()) {
        result.reason = "nonpositive information diagonal";
        return false;
    }
    // 在单位对角尺度下检查联合信息的数值秩。
    const Eigen::VectorXd scale = H.diagonal().cwiseSqrt().cwiseInverse();
    const Eigen::MatrixXd normalized = scale.asDiagonal() * H * scale.asDiagonal();
    const Eigen::LDLT<Eigen::MatrixXd> factor(normalized);
    if (factor.info() != Eigen::Success) {
        result.failure = StartupCovarianceFailure::NumericalFailure;
        result.reason = "information factorization failed";
        return false;
    }
    if (!factor.isPositive()) {
        result.reason = "nonpositive information";
        return false;
    }
    if (factor.vectorD().minCoeff() < 1e-10) {
        result.reason = "weak reduced information";
        return false;
    }
    result.information_supported = true;
    result.failure = StartupCovarianceFailure::NumericalFailure;
    // 传播加权梯度噪声得到联合估计协方差。
    const Eigen::MatrixXd inverse = factor.solve(Eigen::MatrixXd::Identity(H.rows(), H.cols()));
    covariance = scale.asDiagonal() * inverse * scale.asDiagonal() * B * scale.asDiagonal() *
                 inverse * scale.asDiagonal();
    return true;
}

/// 将联合协方差转换为各帧重力对齐后的导航边缘。
bool recoverMarginals(const std::vector<WindowState>& states,
                      const Eigen::Matrix<double, 3, 2>& tangent,
                      const Eigen::MatrixXd& reduced_covariance, StartupCovariance& result) {
    const int dimension = 15 * states.size() + 3;
    const Mat3 align = alignment(states[0].gravity);
    // 计算重力对齐旋转随重力方向变化的导数。
    Mat3 gravity_jacobian;
    constexpr double kEpsilon = 1e-5;
    for (int j = 0; j < 3; ++j) {
        Vec3 perturb = Vec3::Zero();
        perturb[j] = kEpsilon;
        const auto plus =
            Eigen::Quaterniond(alignment(states[0].gravity + perturb) * align.transpose());
        const auto minus =
            Eigen::Quaterniond(alignment(states[0].gravity - perturb) * align.transpose());
        gravity_jacobian.col(j) = (logRotation(plus) - logRotation(minus)) / (2 * kEpsilon);
    }
    // 逐帧转换导航误差坐标并检查协方差半正定性。
    result.frames.reserve(states.size());
    for (std::size_t f = 0; f < states.size(); ++f) {
        const Mat3 R = align * states[f].rotation;
        const Vec3 p = align * (states[f].position - states[0].position),
                   v = align * states[f].velocity;
        Eigen::MatrixXd J = Eigen::MatrixXd::Zero(15, dimension);
        J.block<3, 3>(0, 15 * f) = skew(p) * R;
        J.block<3, 3>(0, 15 * f + 3) = align;
        J.block<3, 3>(3, 15 * f) = R;
        J.block<3, 3>(3, dimension - 3) = gravity_jacobian;
        J.block<3, 3>(6, 15 * f) = skew(v) * R;
        J.block<3, 3>(6, 15 * f + 6) = align;
        J.block<3, 3>(9, 15 * f + 9) = R;
        J.block<3, 3>(12, 15 * f + 9) = skew(v) * R;
        J.block<3, 3>(12, 15 * f + 12) = R;
        const Eigen::MatrixXd projected = projectStartupJacobian(J, tangent);
        Mat15 P = projected * reduced_covariance * projected.transpose();
        P = .5 * (P + P.transpose()).eval();
        if (!P.allFinite()) {
            result.reason = "nonfinite marginal";
            return false;
        }
        const Eigen::SelfAdjointEigenSolver<Mat15> eig(P, Eigen::EigenvaluesOnly);
        const double tolerance =
            128 * std::numeric_limits<double>::epsilon() * 15 * std::max(P.norm(), 1e-30);
        if (eig.info() != Eigen::Success || eig.eigenvalues()[0] < -tolerance) {
            result.reason = "indefinite frame covariance";
            return false;
        }
        result.frames.push_back(P);
    }
    if (!numericallyPositiveCovariance(result.frames.back())) {
        result.reason = "invalid last-state covariance";
        return false;
    }
    return true;
}

}  // namespace

/// 组装线性化目标及梯度噪声，形成各帧在重力对齐后导航图表中的协方差。
StartupCovariance recoverStartupCovariance(const std::vector<WindowState>& states,
                                           const std::vector<WindowFrame>& frames,
                                           const std::vector<std::vector<Observation>>& clouds,
                                           const StartupOptimizationConfig& config,
                                           const Vec3& bias, const Vec3& bias_std) {
    StartupCovariance result;
    if (states.size() < 2 || states.size() != frames.size() || states.size() != clouds.size()) {
        result.reason = "invalid window size";
        return result;
    }
    const int dimension = 15 * states.size() + 3;
    Recovery recovery{states, config, Eigen::MatrixXd(dimension, dimension),
                      Eigen::MatrixXd(dimension, dimension)};
    StartupInertialModel inertial(config, bias, bias_std);
    inertial.rebuild(states, frames);
    Eigen::VectorXd gradient(dimension);
    inertial.linearize(states, recovery.information, gradient);
    // 惯性与偏置先验梯度乘 w，其噪声协方差须乘 w²。
    recovery.score_covariance = config.imu_weight * recovery.information;
    std::unordered_map<VoxelKey, std::vector<Support>, VoxelHash> roots;
    for (std::size_t f = 0; f < states.size(); ++f)
        for (const auto& point : clouds[f]) {
            const Vec3 world = states[f].rotation * point.point + states[f].position;
            roots[voxelKey(world, config.voxel_size)].push_back(
                {&point, world, static_cast<int>(f)});
        }
    for (const auto& root : roots)
        recovery.partition(root.second, voxelCenter(root.first, config.voxel_size),
                           config.voxel_size, 0);
    if (recovery.planes < config.minimum_planes) {
        result.failure = StartupCovarianceFailure::WeakInformation;
        result.reason = "insufficient supported planes";
        return result;
    }

    const auto tangent = gravityTangent(states[0].gravity);
    Eigen::MatrixXd covariance;
    if (!solveCovariance(recovery, tangent, result, covariance) ||
        !recoverMarginals(states, tangent, covariance, result))
        return result;
    result.valid = true;
    result.failure = StartupCovarianceFailure::None;
    return result;
}
}  // namespace zlio
