/**
 * @file solver.hpp
 * @brief 提供联合窗口的规范投影与正阻尼步长计算。
 */
#pragma once
#include <Eigen/Cholesky>
#include <algorithm>
#include <cmath>

#include "zlio/types.hpp"

namespace zlio {
/// 构造与重力方向正交的二维切空间基。
inline Eigen::Matrix<double, 3, 2> gravityTangent(const Vec3& gravity) {
    const Vec3 direction = gravity.normalized();
    Eigen::Matrix<double, 3, 2> tangent;
    tangent.col(0) = direction.unitOrthogonal();
    tangent.col(1) = direction.cross(tangent.col(0));
    return tangent;
}

/// 固定首帧位姿并把公共重力投影到二维切空间。
inline Eigen::MatrixXd projectStartupMatrix(const Eigen::MatrixXd& matrix,
                                            const Eigen::Matrix<double, 3, 2>& tangent) {
    const Eigen::Index free = matrix.rows() - 9, gravity = matrix.rows() - 3;
    Eigen::MatrixXd result(free + 2, free + 2);
    result.topLeftCorner(free, free) = matrix.block(6, 6, free, free);
    result.topRightCorner(free, 2).noalias() = matrix.block(6, gravity, free, 3) * tangent;
    result.bottomLeftCorner(2, free).noalias() =
        tangent.transpose() * matrix.block(gravity, 6, 3, free);
    result.bottomRightCorner<2, 2>().noalias() =
        tangent.transpose() * matrix.bottomRightCorner<3, 3>() * tangent;
    return result;
}

/// 将状态 Jacobian 转换到固定规范的窗口坐标。
inline Eigen::MatrixXd projectStartupJacobian(const Eigen::MatrixXd& jacobian,
                                              const Eigen::Matrix<double, 3, 2>& tangent) {
    const Eigen::Index free = jacobian.cols() - 9;
    Eigen::MatrixXd result(jacobian.rows(), free + 2);
    result.leftCols(free) = jacobian.middleCols(6, free);
    result.rightCols<2>().noalias() = jacobian.rightCols<3>() * tangent;
    return result;
}
struct DampedStartupStep {
    Eigen::VectorXd increment;
    double predicted_reduction = 0.;
    bool valid = false;
};

/// 用正对角阻尼求解局部二次模型并计算预测下降量。
inline DampedStartupStep dampedStartupStep(const Eigen::MatrixXd& hessian,
                                           const Eigen::VectorXd& gradient, double damping) {
    DampedStartupStep result;
    const double floor = std::max(1e-12, 1e-12 * hessian.diagonal().cwiseAbs().maxCoeff());
    const Eigen::VectorXd scale = hessian.diagonal().cwiseAbs().cwiseMax(floor);
    Eigen::MatrixXd damped = hessian;
    damped.diagonal() += damping * scale;
    if (!damped.allFinite())
        return result;
    const Eigen::LDLT<Eigen::MatrixXd> factor(damped);
    if (factor.info() != Eigen::Success || !factor.isPositive() ||
        (factor.vectorD().array() <= 0).any())
        return result;
    result.increment = factor.solve(-gradient);
    if (!result.increment.allFinite() || gradient.dot(result.increment) >= 0.)
        return result;
    // 根据阻尼法方程计算原二次目标的下降量。
    result.predicted_reduction =
        .5 * result.increment.dot(damping * scale.cwiseProduct(result.increment) - gradient);
    result.valid = std::isfinite(result.predicted_reduction) && result.predicted_reduction > 0.;
    return result;
}
}  // namespace zlio
