#ifndef LOAM_INTERFACE__WORLD_ALIGNMENT_HPP_
#define LOAM_INTERFACE__WORLD_ALIGNMENT_HPP_

#include "tf2/LinearMath/Transform.hpp"

namespace loam_interface
{

// The initial base pose defines odom. LIO already includes the sensor's initial
// attitude (including gravity alignment), so remove that pose before applying
// the mounting transform. The mounting rotation is not a world-frame rotation.
inline tf2::Transform initialWorldAlignment(
  const tf2::Transform & base_from_sensor, const tf2::Transform & lio_from_sensor)
{
  return base_from_sensor * lio_from_sensor.inverse();
}

}  // namespace loam_interface

#endif  // LOAM_INTERFACE__WORLD_ALIGNMENT_HPP_
