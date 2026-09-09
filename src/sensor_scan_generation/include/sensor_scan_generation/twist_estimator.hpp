// Copyright 2026
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

#ifndef SENSOR_SCAN_GENERATION__TWIST_ESTIMATOR_HPP_
#define SENSOR_SCAN_GENERATION__TWIST_ESTIMATOR_HPP_

#include <cmath>

#include "tf2/LinearMath/Transform.hpp"

namespace sensor_scan_generation
{

struct ChildFrameTwist
{
  tf2::Vector3 linear{0.0, 0.0, 0.0};
  tf2::Vector3 angular{0.0, 0.0, 0.0};
};

inline bool isFinite(const tf2::Vector3 & vector)
{
  return std::isfinite(vector.x()) && std::isfinite(vector.y()) && std::isfinite(vector.z());
}

inline bool isFinite(const tf2::Quaternion & quaternion)
{
  return std::isfinite(quaternion.x()) && std::isfinite(quaternion.y()) &&
         std::isfinite(quaternion.z()) && std::isfinite(quaternion.w());
}

// Both components of nav_msgs/Odometry.twist are expressed in child_frame_id.
inline bool estimateChildFrameTwist(
  const tf2::Transform & previous_parent_from_child,
  const tf2::Transform & current_parent_from_child, double dt, ChildFrameTwist & twist)
{
  twist = ChildFrameTwist{};
  if (!std::isfinite(dt) || dt <= 1.0e-6) {
    return false;
  }

  tf2::Quaternion previous_rotation = previous_parent_from_child.getRotation();
  tf2::Quaternion current_rotation = current_parent_from_child.getRotation();
  if (
    !isFinite(previous_parent_from_child.getOrigin()) ||
    !isFinite(current_parent_from_child.getOrigin()) || !isFinite(previous_rotation) ||
    !isFinite(current_rotation) || previous_rotation.length2() <= 1.0e-12 ||
    current_rotation.length2() <= 1.0e-12) {
    return false;
  }
  previous_rotation.normalize();
  current_rotation.normalize();

  const tf2::Vector3 linear_parent =
    (current_parent_from_child.getOrigin() - previous_parent_from_child.getOrigin()) / dt;
  twist.linear = tf2::quatRotate(current_rotation.inverse(), linear_parent);

  // Right multiplication gives the relative rotation in child coordinates. Canonicalizing the
  // quaternion sign selects the shortest rotation and avoids spikes when q and -q alternate.
  tf2::Quaternion child_rotation_delta = previous_rotation.inverse() * current_rotation;
  child_rotation_delta.normalize();
  if (child_rotation_delta.w() < 0.0) {
    child_rotation_delta.setValue(
      -child_rotation_delta.x(), -child_rotation_delta.y(), -child_rotation_delta.z(),
      -child_rotation_delta.w());
  }
  const double angle = child_rotation_delta.getAngle();
  if (angle > 1.0e-12) {
    twist.angular = child_rotation_delta.getAxis() * (angle / dt);
  }

  if (!isFinite(twist.linear) || !isFinite(twist.angular)) {
    twist = ChildFrameTwist{};
    return false;
  }
  return true;
}

}  // namespace sensor_scan_generation

#endif  // SENSOR_SCAN_GENERATION__TWIST_ESTIMATOR_HPP_
