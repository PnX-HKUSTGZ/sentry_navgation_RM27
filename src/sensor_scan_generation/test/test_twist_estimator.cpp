#include <gtest/gtest.h>

#include <cmath>

#include "sensor_scan_generation/twist_estimator.hpp"

namespace sensor_scan_generation
{
namespace
{

tf2::Quaternion quaternionFromRpy(double roll, double pitch, double yaw)
{
  tf2::Quaternion quaternion;
  quaternion.setRPY(roll, pitch, yaw);
  quaternion.normalize();
  return quaternion;
}

TEST(TwistEstimator, ExpressesParentTranslationInCurrentChildFrame)
{
  const tf2::Quaternion parent_from_child = quaternionFromRpy(0.0, 0.0, M_PI_2);
  const tf2::Transform previous(parent_from_child, tf2::Vector3(0.0, 0.0, 0.0));
  const tf2::Transform current(parent_from_child, tf2::Vector3(0.0, 1.0, 0.0));

  ChildFrameTwist twist;
  ASSERT_TRUE(estimateChildFrameTwist(previous, current, 0.5, twist));
  EXPECT_NEAR(twist.linear.x(), 2.0, 1.0e-12);
  EXPECT_NEAR(twist.linear.y(), 0.0, 1.0e-12);
  EXPECT_NEAR(twist.linear.z(), 0.0, 1.0e-12);
  EXPECT_NEAR(twist.angular.length(), 0.0, 1.0e-12);
}

TEST(TwistEstimator, ExpressesAngularVelocityInChildFrame)
{
  const tf2::Quaternion previous_rotation = quaternionFromRpy(0.3, -0.4, 0.7);
  tf2::Quaternion child_delta(tf2::Vector3(1.0, 0.0, 0.0), 0.2);
  tf2::Quaternion current_rotation = previous_rotation * child_delta;
  current_rotation.normalize();
  const tf2::Transform previous(previous_rotation, tf2::Vector3(1.0, 2.0, 3.0));
  const tf2::Transform current(current_rotation, tf2::Vector3(1.0, 2.0, 3.0));

  ChildFrameTwist twist;
  ASSERT_TRUE(estimateChildFrameTwist(previous, current, 0.2, twist));
  EXPECT_NEAR(twist.angular.x(), 1.0, 1.0e-12);
  EXPECT_NEAR(twist.angular.y(), 0.0, 1.0e-12);
  EXPECT_NEAR(twist.angular.z(), 0.0, 1.0e-12);
}

TEST(TwistEstimator, UsesShortestRotationAcrossQuaternionSignFlip)
{
  const tf2::Quaternion previous_rotation = quaternionFromRpy(-0.2, 0.1, 0.4);
  tf2::Quaternion child_delta(tf2::Vector3(0.0, 0.0, 1.0), 0.04);
  tf2::Quaternion equivalent_negative_current = previous_rotation * child_delta;
  equivalent_negative_current.setValue(
    -equivalent_negative_current.x(), -equivalent_negative_current.y(),
    -equivalent_negative_current.z(), -equivalent_negative_current.w());
  const tf2::Transform previous(previous_rotation, tf2::Vector3(0.0, 0.0, 0.0));
  const tf2::Transform current(equivalent_negative_current, tf2::Vector3(0.0, 0.0, 0.0));

  ChildFrameTwist twist;
  ASSERT_TRUE(estimateChildFrameTwist(previous, current, 0.02, twist));
  EXPECT_NEAR(twist.angular.x(), 0.0, 1.0e-12);
  EXPECT_NEAR(twist.angular.y(), 0.0, 1.0e-12);
  EXPECT_NEAR(twist.angular.z(), 2.0, 1.0e-12);
}

TEST(TwistEstimator, RejectsNonPositiveTimeDelta)
{
  const tf2::Transform pose = tf2::Transform::getIdentity();
  ChildFrameTwist twist;
  EXPECT_FALSE(estimateChildFrameTwist(pose, pose, 0.0, twist));
  EXPECT_DOUBLE_EQ(twist.linear.length2(), 0.0);
  EXPECT_DOUBLE_EQ(twist.angular.length2(), 0.0);
}

}  // namespace
}  // namespace sensor_scan_generation
