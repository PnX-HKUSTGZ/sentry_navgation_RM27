#pragma once

#include <cmath>
#include <limits>
#include <string>

#include <Eigen/Geometry>
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/buffer.h"

namespace minco_planner::planning_odometry
{

inline bool normalizedQuaternion(
  const geometry_msgs::msg::Quaternion & message, Eigen::Quaterniond & quaternion)
{
  quaternion = Eigen::Quaterniond(message.w, message.x, message.y, message.z);
  const double norm_squared = quaternion.squaredNorm();
  if (!quaternion.coeffs().allFinite() || !std::isfinite(norm_squared) || norm_squared < 1e-12) {
    return false;
  }
  quaternion.normalize();
  return true;
}

// The mounting offset retains the existing rigid, yaw-aligned base convention.
// Articulated sensor mounts need joint feedback before this offset can describe
// chassis velocity; a world-frame rotation alone does not supply that feedback.
inline bool planarVelocity(
  const nav_msgs::msg::Odometry & odometry, const std::string & planning_frame,
  tf2_ros::Buffer * transforms, double offset_x, double offset_y,
  Eigen::Vector3d & velocity, std::string & reason)
{
  velocity.setConstant(std::numeric_limits<double>::quiet_NaN());
  if (odometry.header.frame_id.empty() || planning_frame.empty()) {
    reason = "EMPTY_ODOMETRY_FRAME";
    return false;
  }
  if (odometry.header.stamp.sec < 0 || odometry.header.stamp.nanosec >= 1000000000U ||
    (odometry.header.stamp.sec == 0 && odometry.header.stamp.nanosec == 0U))
  {
    reason = "INVALID_ODOMETRY_STAMP";
    return false;
  }

  Eigen::Quaterniond parent_from_child;
  const auto & twist = odometry.twist.twist;
  const Eigen::Vector3d linear(twist.linear.x, twist.linear.y, twist.linear.z);
  const Eigen::Vector3d angular(twist.angular.x, twist.angular.y, twist.angular.z);
  if (!normalizedQuaternion(odometry.pose.pose.orientation, parent_from_child) ||
    !linear.allFinite() || !angular.allFinite() ||
    !std::isfinite(offset_x) || !std::isfinite(offset_y))
  {
    reason = "INVALID_ODOMETRY_VELOCITY";
    return false;
  }

  Eigen::Quaterniond planning_from_parent = Eigen::Quaterniond::Identity();
  if (odometry.header.frame_id != planning_frame) {
    if (!transforms) {
      reason = "ODOMETRY_TRANSFORM_UNAVAILABLE";
      return false;
    }
    try {
      const auto transform = transforms->lookupTransform(
        planning_frame, odometry.header.frame_id, rclcpp::Time(odometry.header.stamp));
      if (!normalizedQuaternion(transform.transform.rotation, planning_from_parent)) {
        reason = "INVALID_ODOMETRY_TRANSFORM";
        return false;
      }
    } catch (const tf2::TransformException & exception) {
      reason = "ODOMETRY_TRANSFORM_UNAVAILABLE: " + std::string(exception.what());
      return false;
    }
  }

  const Eigen::Matrix3d rotation = parent_from_child.toRotationMatrix();
  const double yaw = std::atan2(rotation(1, 0), rotation(0, 0));
  const Eigen::Vector3d lever(
    std::cos(yaw) * offset_x - std::sin(yaw) * offset_y,
    std::sin(yaw) * offset_x + std::cos(yaw) * offset_y, 0.0);
  const Eigen::Vector3d linear_parent = parent_from_child * linear;
  const Eigen::Vector3d angular_parent = parent_from_child * angular;
  const Eigen::Vector3d corrected =
    planning_from_parent * (linear_parent - angular_parent.cross(lever));
  if (!corrected.allFinite()) {
    reason = "INVALID_TRANSFORMED_VELOCITY";
    return false;
  }
  // Drop height only after both 3D rotations, not in the tilted IMU frame.
  velocity = Eigen::Vector3d(corrected.x(), corrected.y(), 0.0);
  reason = "NONE";
  return true;
}

}  // namespace minco_planner::planning_odometry
