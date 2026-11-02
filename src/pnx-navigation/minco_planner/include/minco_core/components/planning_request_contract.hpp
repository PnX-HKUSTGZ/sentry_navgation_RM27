#ifndef MINCO_PLANNER__PLANNING_REQUEST_CONTRACT_HPP_
#define MINCO_PLANNER__PLANNING_REQUEST_CONTRACT_HPP_

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>

namespace minco_planner::planning_contract {

inline bool finiteQuaternion(const geometry_msgs::msg::Quaternion & q)
{
  return std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z) && std::isfinite(q.w) &&
         q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w > 1.0e-12;
}

inline double quaternionYaw(const geometry_msgs::msg::Quaternion & q)
{
  const double inverse_norm = 1.0 / std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  const double x = q.x * inverse_norm;
  const double y = q.y * inverse_norm;
  const double z = q.z * inverse_norm;
  const double w = q.w * inverse_norm;
  return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

inline bool samePlanningGoal(const geometry_msgs::msg::PoseStamped & lhs,
  const geometry_msgs::msg::PoseStamped & rhs,
  double position_tolerance = 1.0e-3,
  double yaw_tolerance = 1.0e-3)
{
  if (lhs.header.frame_id.empty() || lhs.header.frame_id != rhs.header.frame_id ||
      !std::isfinite(position_tolerance) || position_tolerance < 0.0 || !std::isfinite(yaw_tolerance) ||
      yaw_tolerance < 0.0 || !std::isfinite(lhs.pose.position.x) || !std::isfinite(lhs.pose.position.y) ||
      !std::isfinite(lhs.pose.position.z) || !std::isfinite(rhs.pose.position.x) ||
      !std::isfinite(rhs.pose.position.y) || !std::isfinite(rhs.pose.position.z) ||
      !finiteQuaternion(lhs.pose.orientation) || !finiteQuaternion(rhs.pose.orientation)) {
    return false;
  }

  const double dx = lhs.pose.position.x - rhs.pose.position.x;
  const double dy = lhs.pose.position.y - rhs.pose.position.y;
  const double dz = lhs.pose.position.z - rhs.pose.position.z;
  if (std::hypot(std::hypot(dx, dy), dz) > position_tolerance) {
    return false;
  }

  constexpr double kTwoPi = 6.28318530717958647692;
  const double yaw_delta =
    std::remainder(quaternionYaw(lhs.pose.orientation) - quaternionYaw(rhs.pose.orientation), kTwoPi);
  return std::isfinite(yaw_delta) && std::abs(yaw_delta) <= yaw_tolerance;
}

inline int64_t planningStampNanoseconds(const builtin_interfaces::msg::Time & stamp)
{
  constexpr int64_t kNanosecondsPerSecond = 1000000000LL;
  if (stamp.sec < 0 || stamp.nanosec >= static_cast<uint32_t>(kNanosecondsPerSecond)) {
    return 0;
  }
  return static_cast<int64_t>(stamp.sec) * kNanosecondsPerSecond + static_cast<int64_t>(stamp.nanosec);
}

inline builtin_interfaces::msg::Time monotonicPlanningStamp(
  const builtin_interfaces::msg::Time & candidate, const builtin_interfaces::msg::Time & previous)
{
  constexpr int64_t kNanosecondsPerSecond = 1000000000LL;
  constexpr int64_t kMaxStampNanoseconds =
    static_cast<int64_t>(std::numeric_limits<int32_t>::max()) * kNanosecondsPerSecond +
    (kNanosecondsPerSecond - 1);

  const int64_t previous_ns = planningStampNanoseconds(previous);
  if (previous_ns >= kMaxStampNanoseconds) {
    throw std::overflow_error("planning session stamp exhausted builtin_interfaces/Time range");
  }
  const int64_t candidate_ns = planningStampNanoseconds(candidate);
  const int64_t next_ns = std::max<int64_t>({1LL, candidate_ns, previous_ns + 1LL});

  builtin_interfaces::msg::Time result;
  result.sec = static_cast<int32_t>(next_ns / kNanosecondsPerSecond);
  result.nanosec = static_cast<uint32_t>(next_ns % kNanosecondsPerSecond);
  return result;
}

}  // namespace minco_planner::planning_contract

#endif  // MINCO_PLANNER__PLANNING_REQUEST_CONTRACT_HPP_
