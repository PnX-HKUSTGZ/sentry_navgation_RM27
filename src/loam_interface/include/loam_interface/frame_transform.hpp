#ifndef LOAM_INTERFACE__FRAME_TRANSFORM_HPP_
#define LOAM_INTERFACE__FRAME_TRANSFORM_HPP_

#include "tf2/LinearMath/Transform.hpp"

namespace loam_interface
{
// A gravity-aligned input defines the local odom axes and origin directly.
// The mounting transform relates the moving sensor to the robot, not two
// gravity-aligned world frames. The legacy mode retains its former convention.
inline tf2::Transform odomFromLidarOdom(
  bool lidar_odom_is_gravity_aligned, const tf2::Transform & base_from_lidar)
{
  return lidar_odom_is_gravity_aligned ? tf2::Transform::getIdentity() : base_from_lidar;
}
}  // namespace loam_interface

#endif  // LOAM_INTERFACE__FRAME_TRANSFORM_HPP_
