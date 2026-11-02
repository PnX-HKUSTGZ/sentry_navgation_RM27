// Copyright 2025 Lihan Chen
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef FAKE_VEL_TRANSFORM__ODOM_VALIDATION_HPP_
#define FAKE_VEL_TRANSFORM__ODOM_VALIDATION_HPP_

#include <cmath>
#include <cstdint>

#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace fake_vel_transform
{
namespace odom_validation
{

inline bool finiteTwist(const geometry_msgs::msg::Twist & twist)
{
  return std::isfinite(twist.linear.x) && std::isfinite(twist.linear.y) &&
         std::isfinite(twist.linear.z) && std::isfinite(twist.angular.x) &&
         std::isfinite(twist.angular.y) && std::isfinite(twist.angular.z);
}

inline bool finitePose(const geometry_msgs::msg::Pose & pose)
{
  const auto & orientation = pose.orientation;
  const double quaternion_norm_squared =
    orientation.x * orientation.x + orientation.y * orientation.y + orientation.z * orientation.z +
    orientation.w * orientation.w;
  return std::isfinite(pose.position.x) && std::isfinite(pose.position.y) &&
         std::isfinite(pose.position.z) && std::isfinite(orientation.x) &&
         std::isfinite(orientation.y) && std::isfinite(orientation.z) &&
         std::isfinite(orientation.w) && std::isfinite(quaternion_norm_squared) &&
         quaternion_norm_squared > 1e-12;
}

inline bool finiteOdometry(const nav_msgs::msg::Odometry & odometry)
{
  return finitePose(odometry.pose.pose) && finiteTwist(odometry.twist.twist);
}

inline bool validStampFields(const builtin_interfaces::msg::Time & stamp)
{
  return stamp.sec >= 0 && stamp.nanosec < 1000000000U && (stamp.sec != 0 || stamp.nanosec != 0U);
}

inline bool freshStamp(
  const builtin_interfaces::msg::Time & stamp, int64_t now_nanoseconds, double timeout,
  double future_tolerance)
{
  if (
    !validStampFields(stamp) || now_nanoseconds <= 0 || !std::isfinite(timeout) || timeout <= 0.0 ||
    !std::isfinite(future_tolerance) || future_tolerance < 0.0) {
    return false;
  }

  constexpr int64_t nanoseconds_per_second = 1000000000LL;
  const int64_t stamp_nanoseconds =
    static_cast<int64_t>(stamp.sec) * nanoseconds_per_second + stamp.nanosec;
  const int64_t age_nanoseconds = now_nanoseconds - stamp_nanoseconds;
  const long double timeout_nanoseconds =
    static_cast<long double>(timeout) * nanoseconds_per_second;
  const long double future_tolerance_nanoseconds =
    static_cast<long double>(future_tolerance) * nanoseconds_per_second;
  return static_cast<long double>(age_nanoseconds) <= timeout_nanoseconds &&
         static_cast<long double>(age_nanoseconds) >= -future_tolerance_nanoseconds;
}

}  // namespace odom_validation
}  // namespace fake_vel_transform

#endif  // FAKE_VEL_TRANSFORM__ODOM_VALIDATION_HPP_
