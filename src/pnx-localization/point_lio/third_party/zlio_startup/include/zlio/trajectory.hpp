/**
 * @file trajectory.hpp
 * @brief 保存连续名义轨迹并查询逐点运动。
 */
#pragma once
#include <deque>

#include "zlio/config.hpp"
namespace zlio {
class ImuHistory;
/// 采用恒角加速度和世界加速度变化率的名义轨迹，点旋转从该区间起点积分。
class ImuSegment {
public:
    /// 从状态同采样时刻的左端点积分到右端点，重力为世界系 m/s²。
    ImuSegment(const State& start, const Imu& left, const Imu& right, const Vec3& gravity);
    /// 返回轨迹段起点状态的只读引用。
    const State& start() const { return start_; }
    /// 返回轨迹段末端状态的只读引用。
    const State& end() const { return end_; }
    /// 在闭区间内插值位姿，区间外查询抛出异常。
    Pose at(Time time) const;
    /// 积分闭区间内的名义速度变化，不包含相邻段之间的后验校正跳变。
    Vec3 velocityChange(Time begin, Time end) const;
    /// 对整段轨迹施加世界系刚性校正，不改变 IMU 系偏置。
    void transformWorld(const Pose& correction);

private:
    State start_, end_;
    Vec3 angular_velocity_, angular_acceleration_, world_acceleration_, world_acceleration_rate_;
};

/// 管理观测区间内的名义轨迹及运动曲率。
class Trajectory {
public:
    using Iterator = std::deque<ImuSegment>::const_iterator;
    /// 追加一个已传播的惯性轨迹段。
    void append(ImuSegment segment);
    /// 返回轨迹首段的只读迭代器。
    Iterator begin() const { return segments_.begin(); }
    /// 返回轨迹末段之后的只读迭代器。
    Iterator end() const { return segments_.end(); }
    /// 校正指定时间之后的完整轨迹段。
    void correct(Time begin, const Pose& correction);
    /// 移除指定时刻之前不再需要的轨迹段。
    void discardBefore(Time time);
    /// 计算参考机体系中的线加速度和角加速度。
    Vec6 curvature(const ImuHistory& history, const State& reference, Time begin) const;

private:
    std::deque<ImuSegment> segments_;
};
}  // namespace zlio
