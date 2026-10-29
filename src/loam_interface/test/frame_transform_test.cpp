#include <gtest/gtest.h>

#include "loam_interface/frame_transform.hpp"

TEST(FrameTransform, GravityAlignedInputPreservesPoseAndCloudCoordinates)
{
  tf2::Quaternion mount_rotation;
  mount_rotation.setRPY(-0.523598775598, 0.0, 0.0);
  const tf2::Transform mount(mount_rotation, tf2::Vector3(0.0, 0.126, 0.130));
  const auto odom_from_input = loam_interface::odomFromLidarOdom(true, mount);
  tf2::Quaternion sensor_rotation;
  sensor_rotation.setRPY(-0.4, 0.2, 0.3);
  const tf2::Transform input_from_sensor(sensor_rotation, tf2::Vector3(1, 2, 3));
  const auto output_pose = odom_from_input * input_from_sensor;
  EXPECT_NEAR((output_pose.getOrigin() - input_from_sensor.getOrigin()).length(), 0.0, 1e-12);
  EXPECT_NEAR(output_pose.getRotation().angleShortestPath(sensor_rotation), 0.0, 1e-12);
  const tf2::Vector3 point(4, -2, 1);
  EXPECT_NEAR((odom_from_input * point - point).length(), 0.0, 1e-12);
}

TEST(FrameTransform, LegacyModeRetainsMountTransform)
{
  tf2::Quaternion rotation;
  rotation.setRPY(-0.523598775598, 0.0, 0.0);
  const tf2::Transform mount(rotation, tf2::Vector3(0.0, 0.126, 0.130));
  const auto result = loam_interface::odomFromLidarOdom(false, mount);
  EXPECT_NEAR((result.getOrigin() - mount.getOrigin()).length(), 0.0, 1e-12);
  EXPECT_NEAR(result.getRotation().angleShortestPath(rotation), 0.0, 1e-12);
}
