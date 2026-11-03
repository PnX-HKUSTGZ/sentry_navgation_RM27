/**
 * @file inertial.hpp
 * @brief 定义惯性区间及导航状态传播。
 */
#pragma once
#include "zlio/trajectory.hpp"

namespace zlio {
/// 返回测量端点所属的原始区间长度。
Time originalImuInterval(const Imu& left, const Imu& right);
/// 插值原始角速度和比力测量。
Imu interpolateImu(const Imu& left, const Imu& right, Time time);
/// 计算导航误差的连续动力学矩阵。
Mat15 dynamicsJacobian(const State& state, const Imu& sample, const Vec3& gravity);
/// 计算惯性噪声到导航误差的映射矩阵。
Eigen::Matrix<double, 15, 12> noiseJacobian(const State& state);
/// 推进完整导航估计并返回对应的名义轨迹段。
ImuSegment propagateInertial(Estimate& estimate, const Imu& left, const Imu& right,
                             const Vec3& gravity, const ImuNoise& noise);
}  // namespace zlio
