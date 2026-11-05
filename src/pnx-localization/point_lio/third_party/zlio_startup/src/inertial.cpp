/**
 * @file inertial.cpp
 * @brief 计算惯性区间测量、导航传播与过程噪声。
 */
#include "zlio/inertial.hpp"

#include <stdexcept>

#include "zlio/state.hpp"

namespace zlio {
/// 组装与导航误差注入一致的连续动力学矩阵。
Mat15 dynamicsJacobian(const State& state, const Imu& sample, const Vec3& gravity) {
    Mat15 result = Mat15::Zero();
    const Vec3 world_angular_velocity =
        state.pose.rotation * (sample.angular_velocity - state.gyro_bias);
    const Mat3 rotation_rate = skew(world_angular_velocity);
    result.block<3, 3>(0, 6).setIdentity();
    result.block<3, 3>(0, 9) = -skew(state.pose.position);
    result.block<3, 3>(3, 9) = -Mat3::Identity();
    result.block<3, 3>(6, 3) = skew(gravity);
    result.block<3, 3>(6, 12) = -Mat3::Identity();
    result.block<3, 3>(9, 9) = rotation_rate;
    result.block<3, 3>(12, 9) =
        skew(state.pose.rotation * (sample.acceleration - state.accel_bias) +
             skew(state.velocity) * world_angular_velocity + gravity);
    result.block<3, 3>(12, 12) = rotation_rate;
    return result;
}

/// 组装 IMU 系四组独立噪声到导航误差的映射矩阵。
Eigen::Matrix<double, 15, 12> noiseJacobian(const State& state) {
    Eigen::Matrix<double, 15, 12> result = Eigen::Matrix<double, 15, 12>::Zero();
    const Mat3 rotation = state.pose.rotation.toRotationMatrix();
    result.block<3, 3>(0, 0) = skew(state.pose.position) * rotation;
    result.block<3, 3>(3, 0) = rotation;
    result.block<3, 3>(6, 0) = skew(state.velocity) * rotation;
    result.block<3, 3>(6, 3) = rotation;
    result.block<3, 3>(9, 6) = -rotation;
    result.block<3, 3>(12, 6) = -skew(state.velocity) * rotation;
    result.block<3, 3>(12, 9) = -rotation;
    return result;
}

/// 由原始采样区间的有效测量方差与偏置 PSD 组成十二维方差率。
Eigen::Matrix<double, 12, 1> ImuNoise::varianceRates(double original_interval) const {
    if (!(original_interval > 0) || !std::isfinite(original_interval))
        throw std::invalid_argument("raw IMU interval duration must be finite and positive");
    Eigen::Matrix<double, 12, 1> result;
    result << Vec3::Constant(gyro_sample_variance * original_interval),
        Vec3::Constant(accel_sample_variance * original_interval), Vec3::Constant(gyro_bias_psd),
        Vec3::Constant(accel_bias_psd);
    return result;
}

/// 校验传播区间身份并返回其原始时长，嵌套插值不会增加独立噪声预算。
Time originalImuInterval(const Imu& left, const Imu& right) {
    if (right.time <= left.time)
        throw std::invalid_argument("IMU interval endpoints must be strictly increasing");
    Time begin = left.time, end = right.time;
    bool tagged = false;
    for (const Imu* sample : {&left, &right}) {
        if (sample->source_begin == sample->source_end)
            continue;
        if (sample->source_end <= sample->source_begin ||
            (tagged && (begin != sample->source_begin || end != sample->source_end)))
            throw std::invalid_argument(
                "invalid or inconsistent raw interval for IMU interpolation");
        begin = sample->source_begin;
        end = sample->source_end;
        tagged = true;
    }
    if (left.time < begin || right.time > end)
        throw std::invalid_argument(
            "IMU propagation interval exceeds its raw measurement interval");
    return end - begin;
}

/// 插值有序端点的角速度与比力，并继承原始采样区间边界。
Imu interpolateImu(const Imu& left, const Imu& right, Time time) {
    if (right.time <= left.time || time < left.time || time > right.time)
        throw std::invalid_argument(
            "invalid IMU interpolation interval or query time out of bounds");
    if (time == left.time)
        return left;
    if (time == right.time)
        return right;
    const Time interval = originalImuInterval(left, right);
    const Time begin = left.source_end > left.source_begin     ? left.source_begin
                       : right.source_end > right.source_begin ? right.source_begin
                                                               : left.time;
    const double ratio = seconds(time - left.time) / seconds(right.time - left.time);
    return {time, left.angular_velocity + ratio * (right.angular_velocity - left.angular_velocity),
            left.acceleration + ratio * (right.acceleration - left.acceleration), begin,
            begin + interval};
}

/// 积分名义轨迹并按原始 IMU 区间预算传播一阶协方差。
ImuSegment propagateInertial(Estimate& estimate, const Imu& left, const Imu& right,
                             const Vec3& gravity, const ImuNoise& noise) {
    ImuSegment segment(estimate.state, left, right, gravity);
    const double duration = seconds(right.time - left.time);
    const Imu midpoint = interpolateImu(left, right, left.time + (right.time - left.time) / 2);
    const Mat15 transition =
        Mat15::Identity() + duration * dynamicsJacobian(estimate.state, midpoint, gravity);
    const auto gain = noiseJacobian(estimate.state);

    // 一阶白增量近似：虚拟切段共享同一个原始测量区间的预算。
    const auto variances =
        (duration * noise.varianceRates(seconds(originalImuInterval(left, right)))).eval();
    Mat15 covariance = transition * estimate.covariance * transition.transpose() +
                       gain * variances.asDiagonal() * gain.transpose();
    estimate.state = segment.end();
    estimate.covariance = .5 * (covariance + covariance.transpose()).eval();
    return segment;
}

}  // namespace zlio
