/**
 * @file state.hpp
 * @brief 声明十五维导航误差的注入、差分、Jacobian 与协方差坐标转换。
 */
#pragma once

#include <stdexcept>

#include "zlio/types.hpp"

namespace zlio {

/// 表示估计计算未通过有限性、数值秩或正定性检查。
class NumericalError : public std::runtime_error {
public:
    /// 保存数值失败的诊断说明。
    using std::runtime_error::runtime_error;
};

/// 返回满足 skew(a)b=a×b 的反对称矩阵。
Mat3 skew(const Vec3& vector);
/// 计算有限时间旋转增量；角速度为 rad/s，时长为秒，保留小角速度截断。
Mat3 rotationIncrement(const Vec3& angular_velocity, double duration);
/// 将弧度旋转向量映射为单位四元数。
Eigen::Quaterniond expRotation(const Vec3& tangent);
/// 返回单位旋转的主支旋转向量，角度不超过 π。
Vec3 logRotation(Eigen::Quaterniond rotation);
/**
 * @brief 按 [ρ_p,θ,ρ_v,β_g,β_a] 顺序通过 SD 群指数注入十五维导航误差。
 * @details 令 Q=Exp(θ)、J=J_l(θ)，则 R'=QR、p'=Qp+Jρ_p、v'=Qv+Jρ_v。
 * 令 c=phi1(−ad_(θ,ρ_v))[β_g,β_a]，则 bg'=bg+Rᵀc_g、
 * ba'=ba+Rᵀ(c_a−[v]×c_g)。局部零点的一阶微分与导航误差坐标约定一致。
 */
State retract(const State& state, const Vec15& error);
/// 计算相对 reference 的完整 SD 群对数，在旋转主支内与 retract 互逆。
Vec15 difference(const State& state, const State& reference);
/// 返回世界点对十五维导航误差的 Jacobian，仅位姿六列非零。
PointJacobian pointJacobian(const Vec3& world_point);
/// 返回世界点对 [e_p,e_θ] 的 Jacobian [I,−[q]×]。
Eigen::Matrix<double, 3, 6> pointPoseJacobian(const Vec3& world_point);
/// 归一化量纲后检查对称正定与数值秩，不裁剪特征值或注入抖动。
bool numericallyPositiveCovariance(const Mat15& covariance);
/// 按相同正定规则分解十五维矩阵并求其对称逆，数值失败时抛出异常。
Mat15 inversePositiveDefinite(const Mat15& matrix);
/**
 * @brief 返回当前局部增量到 reference 先验误差坐标的 Jacobian。
 * @details 对 difference(retract(state,δ),reference) 在 δ=0 处求导。
 */
Mat15 priorErrorJacobian(const State& state, const State& reference);
/**
 * @brief 将注入增量附近的协方差搬到新状态零误差坐标，保留完整耦合块。
 * @details 对 difference(retract(X,increment+ε),retract(X,increment)) 在 ε=0 处求导。
 */
Mat15 errorResetJacobian(const Vec15& increment);
/// 返回世界系位置及左旋转增量的位姿协方差，顺序为位置、旋转。
Mat6 poseCovariance(const State& state, const Mat15& covariance);

}  // namespace zlio
