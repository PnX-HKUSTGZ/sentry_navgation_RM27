/**
 * @file types.hpp
 * @brief 定义世界系导航状态、传感器采样时间及点观测的基础数据类型。
 */
#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace zlio {

using Time = std::int64_t;  // 真实采样时刻，整数纳秒。
using Vec3 = Eigen::Vector3d;
using Mat3 = Eigen::Matrix3d;
using Mat6 = Eigen::Matrix<double, 6, 6>;
using Vec6 = Eigen::Matrix<double, 6, 1>;
using Vec15 = Eigen::Matrix<double, 15, 1>;
using Mat15 = Eigen::Matrix<double, 15, 15>;
using PointJacobian = Eigen::Matrix<double, 3, 15>;

/// 将整数纳秒时长转换为秒。
inline double seconds(Time duration) {
    return static_cast<double>(duration) * 1e-9;
}
/// 将可表示的秒时长四舍五入为整数纳秒。
inline Time nanoseconds(double duration) {
    return static_cast<Time>(std::llround(duration * 1e9));
}

/// 刚性位姿；旋转须为单位四元数，作用方向由持有该位姿的接口约定。
struct Pose {
    Eigen::Quaterniond rotation = Eigen::Quaterniond::Identity();
    Vec3 position = Vec3::Zero();
    /// 将源坐标系点旋转并平移到目标坐标系。
    Vec3 apply(const Vec3& point) const { return rotation * point + position; }
    /// 将目标坐标系点按同一刚性位姿逆变换回源坐标系。
    Vec3 inverseApply(const Vec3& point) const { return rotation.conjugate() * (point - position); }
};

/**
 * @brief 位姿将 IMU 系映射到世界系；位置、速度在世界系，两种偏置在 IMU 系。
 * @details 位置为 m，速度为 m/s，陀螺偏置为 rad/s，加速度偏置为 m/s²。
 */
struct State {
    Time time = 0;
    Pose pose;
    Vec3 velocity = Vec3::Zero();
    Vec3 gyro_bias = Vec3::Zero();
    Vec3 accel_bias = Vec3::Zero();
};

/// IMU 系测量；acceleration 为未去重力的比力（m/s²），角速度为 rad/s。
struct Imu {
    Time time = 0;
    Vec3 angular_velocity = Vec3::Zero();
    Vec3 acceleration = Vec3::Zero();

    Time source_begin = 0, source_end = 0;  // 插值所属原始区间；真实样本两者均为零。
};

/// 原始雷达系回波，保留真实获取时间及窗口内唯一 ID。
struct Point {
    Vec3 position = Vec3::Zero();
    Time time = 0;
    std::uint64_t id = 0;
};
using Scan = std::vector<Point>;

/// 采样时位于参考雷达系、滤波时位于参考 IMU 系，点协方差以 m² 随坐标同步转换。
struct Observation {
    Vec3 point = Vec3::Zero();
    Mat3 covariance = Mat3::Zero();
    Time time = 0;
    std::uint64_t id = 0;
};

/// 当前误差图表下的观测正规方程，仅包含本次观测信息而不重复加入先验。
struct Linearization {
    Mat15 information = Mat15::Zero();
    Vec15 gradient = Vec15::Zero();
    std::size_t count = 0;
};

/// 状态及其局部导航误差协方差，顺序为位置、姿态、速度、两种偏置误差。
struct Estimate {
    State state;
    Mat15 covariance = Mat15::Identity();
};

}  // namespace zlio
