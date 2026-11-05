/**
 * @file trajectory.cpp
 * @brief 积分名义运动、查询轨迹并计算运动曲率。
 */
#include "zlio/trajectory.hpp"

#include <algorithm>
#include <stdexcept>
#include <utility>

#include "zlio/inertial.hpp"
#include "zlio/input.hpp"
namespace zlio {
namespace {
/// 以区间平均角速度执行归一化四元数步进，时长单位为秒。
Eigen::Quaterniond integrateRotation(const Eigen::Quaterniond& rotation,
                                     const Vec3& angular_velocity, double duration) {
    Eigen::Quaterniond increment(1, .5 * duration * angular_velocity.x(),
                                 .5 * duration * angular_velocity.y(),
                                 .5 * duration * angular_velocity.z());
    return (rotation * increment).normalized();
}
}  // namespace

/// 积分平均角速度与线性世界加速度，建立拥有独立参数的轨迹段。
ImuSegment::ImuSegment(const State& start, const Imu& left, const Imu& right, const Vec3& gravity)
    : start_(start), end_(start) {
    if (left.time != start.time || right.time <= left.time)
        throw std::invalid_argument(
            "IMU trajectory start must match the state time and precede the end time");
    const double duration = seconds(right.time - left.time);
    angular_velocity_ = left.angular_velocity - start.gyro_bias;
    angular_acceleration_ = (right.angular_velocity - left.angular_velocity) / duration;
    end_.time = right.time;
    end_.pose.rotation = integrateRotation(
        start.pose.rotation, angular_velocity_ + .5 * angular_acceleration_ * duration, duration);
    world_acceleration_ = start.pose.rotation * (left.acceleration - start.accel_bias) + gravity;
    const Vec3 end_acceleration =
        end_.pose.rotation * (right.acceleration - start.accel_bias) + gravity;
    world_acceleration_rate_ = (end_acceleration - world_acceleration_) / duration;
    end_.pose.position += start.velocity * duration +
                          .5 * world_acceleration_ * duration * duration +
                          world_acceleration_rate_ * duration * duration * duration / 6;
    end_.velocity +=
        world_acceleration_ * duration + .5 * world_acceleration_rate_ * duration * duration;
}

/// 从当前 IMU 区间起点插值指定采样时刻的位姿。
Pose ImuSegment::at(Time time) const {
    if (time < start_.time || time > end_.time)
        throw std::out_of_range("query time is outside the IMU trajectory segment");
    const double duration = seconds(time - start_.time);
    Pose result;
    const auto& rotation = start_.pose.rotation;
    result.rotation = integrateRotation(
        rotation, angular_velocity_ + .5 * angular_acceleration_ * duration, duration);
    result.position = start_.pose.position + start_.velocity * duration +
                      .5 * world_acceleration_ * duration * duration +
                      world_acceleration_rate_ * duration * duration * duration / 6;
    return result;
}

/// 计算轨迹子区间内的速度增量。
Vec3 ImuSegment::velocityChange(Time begin, Time end) const {
    if (begin < start_.time || end > end_.time || begin > end)
        throw std::out_of_range(
            "velocity increment query is outside the inertial trajectory segment");
    const double left = seconds(begin - start_.time), right = seconds(end - start_.time);
    return world_acceleration_ * (right - left) +
           .5 * world_acceleration_rate_ * (right * right - left * left);
}

/// 将轨迹两端及世界运动参数共同变换，保持整段刚性校正一致。
void ImuSegment::transformWorld(const Pose& correction) {
    for (State* state : {&start_, &end_}) {
        state->pose.position = correction.apply(state->pose.position);
        state->pose.rotation = (correction.rotation * state->pose.rotation).normalized();
        state->velocity = correction.rotation * state->velocity;
    }
    world_acceleration_ = correction.rotation * world_acceleration_;
    world_acceleration_rate_ = correction.rotation * world_acceleration_rate_;
}

namespace {
/// 对分段线性角速度求连续最小二乘斜率。
Vec3 angularAcceleration(const ImuHistory& history, Time begin, Time end) {
    const double span = seconds(end - begin);
    if (!(span > 0.))
        return Vec3::Zero();
    const auto& samples = history.samples();
    Imu left = history.at(begin);
    auto next = std::upper_bound(samples.begin(), samples.end(), begin,
                                 [](Time stamp, const Imu& sample) { return stamp < sample.time; });
    Vec3 first_moment = Vec3::Zero();
    while (left.time < end) {
        const Imu right = next != samples.end() && next->time <= end ? *next++ : history.at(end);
        const double dt = seconds(right.time - left.time);
        const double centered = seconds(left.time - begin) - .5 * span;
        first_moment += .5 * centered * dt * (left.angular_velocity + right.angular_velocity) +
                        dt * dt * (left.angular_velocity / 6. + right.angular_velocity / 3.);
        left = right;
    }
    return 12. / (span * span * span) * first_moment;
}

}  // namespace
/// 追加已计算的惯性轨迹段。
void Trajectory::append(ImuSegment segment) {
    segments_.push_back(std::move(segment));
}
/// 对本次观测的轨迹段施加世界系校正。
void Trajectory::correct(Time begin, const Pose& correction) {
    for (auto& segment : segments_)
        if (segment.start().time >= begin)
            segment.transformWorld(correction);
}
/// 回收指定时刻之前的轨迹段。
void Trajectory::discardBefore(Time time) {
    while (!segments_.empty() && segments_.front().end().time < time)
        segments_.pop_front();
}
/// 汇总固定时间范围中的连续运动变化。
Vec6 Trajectory::curvature(const ImuHistory& history, const State& reference, Time begin) const {
    Vec6 result = Vec6::Zero();
    const double span = seconds(reference.time - begin);
    if (!(span > 0.))
        return result;
    Vec3 velocity_change = Vec3::Zero();
    for (const auto& segment : segments_) {
        const Time left = std::max(begin, segment.start().time);
        const Time right = std::min(reference.time, segment.end().time);
        if (right > left)
            velocity_change += segment.velocityChange(left, right);
    }
    result.head<3>() = reference.pose.rotation.conjugate() * (velocity_change / span);
    result.tail<3>() = angularAcceleration(history, begin, reference.time);
    return result;
}
}  // namespace zlio
