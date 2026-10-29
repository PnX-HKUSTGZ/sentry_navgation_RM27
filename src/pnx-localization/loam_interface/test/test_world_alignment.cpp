#include <gtest/gtest.h>

#include "loam_interface/world_alignment.hpp"

namespace
{
tf2::Transform pose(double x, double y, double z, double roll, double yaw = 0.0)
{
  tf2::Quaternion q;
  q.setRPY(roll, 0.0, yaw);
  return tf2::Transform(q, tf2::Vector3(x, y, z));
}

TEST(WorldAlignment, TiltedSensorDoesNotTiltTheNavigationWorld)
{
  // simulation_waking_robot.xacro: both sensor frames share this mounting pose.
  const double roll = -0.523598775598;
  const auto mount = pose(0.0, 0.126, 0.130, roll);
  const auto initial_lio = pose(0.0, 0.0, 0.0, roll);
  const auto alignment = loam_interface::initialWorldAlignment(mount, initial_lio);
  const auto base = alignment * initial_lio * mount.inverse();
  EXPECT_NEAR(alignment.getRotation().angleShortestPath(tf2::Quaternion::getIdentity()), 0.0, 1e-9);
  EXPECT_NEAR(base.getOrigin().length(), 0.0, 1e-9);
  EXPECT_NEAR(base.getRotation().angleShortestPath(tf2::Quaternion::getIdentity()), 0.0, 1e-9);

  const auto moved_lio = pose(1.0, 0.0, 0.0, roll);
  const auto moved_base = alignment * moved_lio * mount.inverse();
  EXPECT_NEAR(moved_base.getOrigin().x(), 1.0, 1e-9);
  EXPECT_NEAR(moved_base.getOrigin().y(), 0.0, 1e-9);
  EXPECT_NEAR(moved_base.getOrigin().z(), 0.0, 1e-9);
}

TEST(WorldAlignment, InitialLioPositionAndYawAreRemoved)
{
  const auto mount = pose(0.0, 0.126, 0.130, -0.523598775598);
  const auto initial_lio = pose(1.2, -0.5, 0.3, -0.523598775598, 0.7);
  const auto alignment = loam_interface::initialWorldAlignment(mount, initial_lio);
  const auto base = alignment * initial_lio * mount.inverse();
  EXPECT_NEAR(base.getOrigin().length(), 0.0, 1e-9);
  EXPECT_NEAR(base.getRotation().angleShortestPath(tf2::Quaternion::getIdentity()), 0.0, 1e-9);
}
}  // namespace
