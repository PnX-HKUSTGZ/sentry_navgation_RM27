#ifndef RM_27_STIMULATION__TWIST_TRANSFORM_HPP_
#define RM_27_STIMULATION__TWIST_TRANSFORM_HPP_

#include <array>
#include <cstddef>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_with_covariance.hpp>
#include <tf2/LinearMath/Transform.h>

namespace rm_27_stimulation
{

// base_from_child is the usual TF transform: the child origin and axes expressed in base.
// The input twist is measured at the base origin and expressed in base. The result is measured
// at the child origin and expressed in child, as required by nav_msgs/Odometry::child_frame_id.
inline geometry_msgs::msg::Twist transformTwistFromBaseToChild(
  const geometry_msgs::msg::Twist & base_twist, const tf2::Transform & base_from_child)
{
  const tf2::Vector3 linear_base(
    base_twist.linear.x, base_twist.linear.y, base_twist.linear.z);
  const tf2::Vector3 angular_base(
    base_twist.angular.x, base_twist.angular.y, base_twist.angular.z);

  const tf2::Matrix3x3 child_from_base = base_from_child.getBasis().transpose();
  const tf2::Vector3 linear_child = child_from_base *
    (linear_base + angular_base.cross(base_from_child.getOrigin()));
  const tf2::Vector3 angular_child = child_from_base * angular_base;

  geometry_msgs::msg::Twist child_twist;
  child_twist.linear.x = linear_child.x();
  child_twist.linear.y = linear_child.y();
  child_twist.linear.z = linear_child.z();
  child_twist.angular.x = angular_child.x();
  child_twist.angular.y = angular_child.y();
  child_twist.angular.z = angular_child.z();
  return child_twist;
}

inline geometry_msgs::msg::TwistWithCovariance transformTwistFromBaseToChild(
  const geometry_msgs::msg::TwistWithCovariance & base_twist,
  const tf2::Transform & base_from_child)
{
  geometry_msgs::msg::TwistWithCovariance child_twist;
  child_twist.twist = transformTwistFromBaseToChild(base_twist.twist, base_from_child);

  // Build the Jacobian for [linear, angular]. For an angular basis vector e,
  // the child-origin linear velocity contribution is R_child_base * (e x p_base_child).
  std::array<double, 36> jacobian{};
  const tf2::Matrix3x3 child_from_base = base_from_child.getBasis().transpose();
  for (std::size_t column = 0; column < 3; ++column) {
    tf2::Vector3 basis(0.0, 0.0, 0.0);
    basis[static_cast<int>(column)] = 1.0;
    const tf2::Vector3 rotated_basis = child_from_base * basis;
    const tf2::Vector3 lever_arm_velocity =
      child_from_base * basis.cross(base_from_child.getOrigin());
    for (std::size_t row = 0; row < 3; ++row) {
      jacobian[row * 6 + column] = rotated_basis[static_cast<int>(row)];
      jacobian[row * 6 + column + 3] = lever_arm_velocity[static_cast<int>(row)];
      jacobian[(row + 3) * 6 + column + 3] = rotated_basis[static_cast<int>(row)];
    }
  }

  std::array<double, 36> intermediate{};
  for (std::size_t row = 0; row < 6; ++row) {
    for (std::size_t column = 0; column < 6; ++column) {
      for (std::size_t inner = 0; inner < 6; ++inner) {
        intermediate[row * 6 + column] +=
          jacobian[row * 6 + inner] * base_twist.covariance[inner * 6 + column];
      }
    }
  }
  for (std::size_t row = 0; row < 6; ++row) {
    for (std::size_t column = 0; column < 6; ++column) {
      for (std::size_t inner = 0; inner < 6; ++inner) {
        child_twist.covariance[row * 6 + column] +=
          intermediate[row * 6 + inner] * jacobian[column * 6 + inner];
      }
    }
  }
  return child_twist;
}

}  // namespace rm_27_stimulation

#endif  // RM_27_STIMULATION__TWIST_TRANSFORM_HPP_
