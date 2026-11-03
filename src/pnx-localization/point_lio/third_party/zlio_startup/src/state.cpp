/**
 * @file state.cpp
 * @brief 实现导航误差坐标、旋转 Jacobian 及完整协方差坐标转换。
 */
#include "zlio/state.hpp"

#include <Eigen/Cholesky>
#include <Eigen/LU>
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace zlio {

/// 构造向量对应的叉乘反对称矩阵。
Mat3 skew(const Vec3& vector) {
    Mat3 result;
    result << 0, -vector.z(), vector.y(), vector.z(), 0, -vector.x(), -vector.y(), vector.x(), 0;
    return result;
}

/// 以弧度旋转向量构造单位四元数，小角度使用归一化一阶展开。
Eigen::Quaterniond expRotation(const Vec3& tangent) {
    const double angle = tangent.norm();
    if (angle < 1e-10) {
        return Eigen::Quaterniond(1, tangent.x() / 2, tangent.y() / 2, tangent.z() / 2)
            .normalized();
    }
    return Eigen::Quaterniond(Eigen::AngleAxisd(angle, tangent / angle));
}

/// 选择四元数正半球并计算主支旋转向量。
Vec3 logRotation(Eigen::Quaterniond rotation) {
    rotation.normalize();
    if (rotation.w() < 0)
        rotation.coeffs() *= -1;
    const double sine = rotation.vec().norm();
    if (sine < 1e-10)
        return 2 * rotation.vec();
    return (2 * std::atan2(sine, rotation.w()) / sine) * rotation.vec();
}

namespace {
/// 计算 SO(3) 左 Jacobian，小角度展开避免相近数相减。
Mat3 leftRotationJacobian(const Vec3& angle) {
    const double squared = angle.squaredNorm();
    const Mat3 cross = skew(angle);
    const double first = squared < 1e-8 ? .5 - squared / 24 + squared * squared / 720
                                        : (1 - std::cos(std::sqrt(squared))) / squared;
    const double second = squared < 1e-8
                              ? 1. / 6 - squared / 120 + squared * squared / 5040
                              : (1 - std::sin(std::sqrt(squared)) / std::sqrt(squared)) / squared;
    return Mat3::Identity() + first * cross + second * cross * cross;
}

/// 计算旋转主支上的左 Jacobian 逆，小角度使用同阶级数展开。
Mat3 inverseLeftRotationJacobian(const Vec3& angle) {
    const double squared = angle.squaredNorm();
    const Mat3 cross = skew(angle);
    const double coefficient =
        squared < 1e-8
            ? 1. / 12 + squared / 720 + squared * squared / 30240
            : (1 - .5 * std::sqrt(squared) / std::tan(.5 * std::sqrt(squared))) / squared;
    return Mat3::Identity() - .5 * cross + coefficient * cross * cross;
}

/// phi1(A)=∫₀¹Exp(sA)ds；包含半直积的全部非对角耦合。
template <int Size>
Eigen::Matrix<double, Size, Size> exponentialJacobian(
    const Eigen::Matrix<double, Size, Size>& algebra) {
    using Matrix = Eigen::Matrix<double, Size, Size>;
    Matrix sum = Matrix::Identity(), term = sum;
    for (int order = 1; order <= 60; ++order) {
        term = (term * algebra / static_cast<double>(order + 1)).eval();
        sum += term;
        if (term.cwiseAbs().maxCoeff() <= 2e-16 * std::max(1., sum.cwiseAbs().maxCoeff()))
            return sum;
    }
    throw NumericalError("SD group Jacobian series did not converge");
}

/// 齐次伽利略子群 (R,v) 的李代数伴随，顺序为旋转、速度。
Mat6 velocityAlgebra(const Vec3& angle, const Vec3& velocity) {
    Mat6 result = Mat6::Zero();
    result.topLeftCorner<3, 3>() = skew(angle);
    result.bottomRightCorner<3, 3>() = skew(angle);
    result.bottomLeftCorner<3, 3>() = skew(velocity);
    return result;
}

/// SD 群伴随微分，坐标顺序为 [ρ_p,θ,ρ_v,β_g,β_a]。
Mat15 stateAlgebra(const Vec15& error) {
    Mat15 result = Mat15::Zero();
    const Mat3 angle = skew(error.segment<3>(3));
    for (int block : {0, 3, 6, 9, 12})
        result.block<3, 3>(block, block) = angle;
    result.block<3, 3>(0, 3) = skew(error.head<3>());
    result.block<3, 3>(6, 3) = skew(error.segment<3>(6));
    result.block<3, 3>(9, 3) = skew(error.segment<3>(9));
    result.block<3, 3>(12, 3) = skew(error.segment<3>(12));
    result.block<3, 3>(12, 6) = skew(error.segment<3>(9));
    result.block<3, 3>(12, 9) = skew(error.segment<3>(6));
    return result;
}
}  // namespace

/// 使用 SE₂(3)⋉se(3) 的完整群指数左乘，将局部导航误差注入状态。
State retract(const State& state, const Vec15& error) {
    State result = state;
    const auto correction = expRotation(error.segment<3>(3));
    const Mat3 jacobian = leftRotationJacobian(error.segment<3>(3));
    result.pose.rotation = (correction * state.pose.rotation).normalized();
    result.pose.position = correction * state.pose.position + jacobian * error.head<3>();
    result.velocity = correction * state.velocity + jacobian * error.segment<3>(6);
    const Vec6 bias =
        exponentialJacobian<6>(-velocityAlgebra(error.segment<3>(3), error.segment<3>(6))) *
        error.tail<6>();
    result.gyro_bias += state.pose.rotation.conjugate() * bias.head<3>();
    result.accel_bias +=
        state.pose.rotation.conjugate() * (bias.tail<3>() - skew(state.velocity) * bias.head<3>());
    return result;
}

/// 计算 SD 群误差的主支对数，与完整群指数注入互逆。
Vec15 difference(const State& state, const State& reference) {
    const auto rotation = state.pose.rotation * reference.pose.rotation.conjugate();
    Vec15 error;
    error.segment<3>(3) = logRotation(rotation);
    const Mat3 inverse_jacobian = inverseLeftRotationJacobian(error.segment<3>(3));
    error.head<3>() = inverse_jacobian * (state.pose.position - rotation * reference.pose.position);
    error.segment<3>(6) = inverse_jacobian * (state.velocity - rotation * reference.velocity);
    Vec6 bias;
    bias.head<3>() = reference.pose.rotation * (state.gyro_bias - reference.gyro_bias);
    bias.tail<3>() = reference.pose.rotation * (state.accel_bias - reference.accel_bias) +
                     skew(reference.velocity) * bias.head<3>();
    error.tail<6>() =
        exponentialJacobian<6>(-velocityAlgebra(error.segment<3>(3), error.segment<3>(6)))
            .partialPivLu()
            .solve(bias);
    return error;
}

/// 将世界点的六维位姿 Jacobian 扩展为十五维导航误差 Jacobian。
PointJacobian pointJacobian(const Vec3& world_point) {
    PointJacobian jacobian = PointJacobian::Zero();
    jacobian.leftCols<6>() = pointPoseJacobian(world_point);
    return jacobian;
}

/// 构造世界点对左侧导航位姿误差的 Jacobian [I,−[q]×]。
Eigen::Matrix<double, 3, 6> pointPoseJacobian(const Vec3& world_point) {
    Eigen::Matrix<double, 3, 6> jacobian;
    jacobian.leftCols<3>().setIdentity();
    jacobian.rightCols<3>() = -skew(world_point);
    return jacobian;
}

namespace {
/// 归一化矩阵量纲并验证对称性、正定性和相对数值秩。
bool factorPositiveDefinite(const Mat15& matrix, Vec15& scaling,
                            Eigen::LDLT<Mat15>& decomposition) {
    if (!matrix.allFinite() || (matrix.diagonal().array() <= 0).any())
        return false;
    scaling = matrix.diagonal().cwiseSqrt().cwiseInverse();
    const Mat15 normalized = scaling.asDiagonal() * matrix * scaling.asDiagonal();
    if (!normalized.allFinite() || !normalized.isApprox(normalized.transpose(), 1e-10))
        return false;
    decomposition.compute(normalized);
    if (decomposition.info() != Eigen::Success)
        return false;
    const auto pivots = decomposition.vectorD();
    const double tolerance =
        std::numeric_limits<double>::epsilon() * matrix.rows() * pivots.cwiseAbs().maxCoeff();
    return pivots.allFinite() && (pivots.array() > tolerance).all();
}
}  // namespace

/// 消除对角量纲后检查协方差对称性、正定性和相对数值秩。
bool numericallyPositiveCovariance(const Mat15& covariance) {
    Vec15 scaling;
    Eigen::LDLT<Mat15> decomposition;
    return factorPositiveDefinite(covariance, scaling, decomposition);
}

/// 用归一化正定分解求解逆矩阵，并验证结果的数值有效性。
Mat15 inversePositiveDefinite(const Mat15& matrix) {
    Vec15 scaling;
    Eigen::LDLT<Mat15> decomposition;
    if (!factorPositiveDefinite(matrix, scaling, decomposition))
        throw NumericalError("Matrix is not numerically symmetric positive definite");
    const Mat15 inverse =
        scaling.asDiagonal() * decomposition.solve(scaling.asDiagonal().toDenseMatrix());
    const Mat15 symmetric = .5 * inverse + .5 * inverse.transpose();
    if (!numericallyPositiveCovariance(symmetric))
        throw NumericalError("Positive-definite matrix solve produced an invalid inverse");
    return symmetric;
}

/// 将导航误差协方差转换为世界系位置与左旋转的位姿边缘协方差。
Mat6 poseCovariance(const State& state, const Mat15& covariance) {
    Eigen::Matrix<double, 6, 15> jacobian = Eigen::Matrix<double, 6, 15>::Zero();
    jacobian.topRows<3>() = pointJacobian(state.pose.position);
    jacobian.block<3, 3>(3, 3).setIdentity();
    return jacobian * covariance * jacobian.transpose();
}

/// 计算恒角速度对应的矩阵旋转增量。
Mat3 rotationIncrement(const Vec3& angular_velocity, double duration) {
    const Mat3 identity = Mat3::Identity();
    const double angular_speed = angular_velocity.norm();
    if (angular_speed > 1e-7) {
        const Vec3 axis = angular_velocity / angular_speed;
        const Mat3 cross = skew(axis);
        const double angle = angular_speed * duration;
        return identity + std::sin(angle) * cross + (1.0 - std::cos(angle)) * cross * cross;
    }

    return identity;
}

/// Log(Exp(δ)Exp(e)) 对 δ 的微分，即完整 SD 群左 Jacobian 的逆。
Mat15 priorErrorJacobian(const State& state, const State& reference) {
    const Vec15 error = difference(state, reference);
    return errorResetJacobian(error).partialPivLu().solve(Mat15::Identity());
}

/// Log(Exp(d+ε)Exp(−d)) 对 ε 的微分，即完整 SD 群左 Jacobian。
Mat15 errorResetJacobian(const Vec15& increment) {
    return exponentialJacobian<15>(stateAlgebra(increment));
}

}  // namespace zlio
