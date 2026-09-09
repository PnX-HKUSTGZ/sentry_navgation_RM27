#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_with_covariance.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>

#include "rm_27_stimulation/ground_truth_state_buffer.hpp"
#include "rm_27_stimulation/twist_transform.hpp"

namespace rm_27_stimulation
{
namespace
{

constexpr double kTolerance = 1.0e-12;

geometry_msgs::msg::Twist makeTwist(
  double vx, double vy, double vz, double wx, double wy, double wz)
{
  geometry_msgs::msg::Twist twist;
  twist.linear.x = vx;
  twist.linear.y = vy;
  twist.linear.z = vz;
  twist.angular.x = wx;
  twist.angular.y = wy;
  twist.angular.z = wz;
  return twist;
}

GroundTruthState makeState(
  int64_t stamp_nanoseconds, double x, double yaw, double value)
{
  tf2::Quaternion rotation;
  rotation.setRPY(0.0, 0.0, yaw);

  GroundTruthState state;
  state.stamp_nanoseconds = stamp_nanoseconds;
  state.odom_to_base = tf2::Transform(rotation, tf2::Vector3(x, 2.0 * x, -x));
  state.base_twist.twist = makeTwist(value, 2.0 * value, 3.0 * value, 4.0 * value,
      5.0 * value, 6.0 * value);
  state.pose_covariance.fill(value);
  state.base_twist.covariance.fill(10.0 * value);
  return state;
}

TEST(GroundTruthTwistTransform, AppliesSideMountedLidarLeverArm)
{
  const tf2::Transform base_from_lidar(
    tf2::Quaternion::getIdentity(), tf2::Vector3(0.0, 0.18, 0.14));
  const auto lidar_twist = transformTwistFromBaseToChild(
    makeTwist(1.0, -0.4, 0.2, 0.0, 0.0, 2.0), base_from_lidar);

  EXPECT_NEAR(lidar_twist.linear.x, 0.64, kTolerance);
  EXPECT_NEAR(lidar_twist.linear.y, -0.4, kTolerance);
  EXPECT_NEAR(lidar_twist.linear.z, 0.2, kTolerance);
  EXPECT_NEAR(lidar_twist.angular.x, 0.0, kTolerance);
  EXPECT_NEAR(lidar_twist.angular.y, 0.0, kTolerance);
  EXPECT_NEAR(lidar_twist.angular.z, 2.0, kTolerance);
}

TEST(GroundTruthTwistTransform, RotatesLinearAndAngularVelocityIntoChildFrame)
{
  tf2::Quaternion base_from_lidar_rotation;
  base_from_lidar_rotation.setRPY(0.0, 0.0, M_PI_2);
  const tf2::Transform base_from_lidar(
    base_from_lidar_rotation, tf2::Vector3(0.0, 0.0, 0.0));
  const auto lidar_twist = transformTwistFromBaseToChild(
    makeTwist(1.0, 2.0, 3.0, 4.0, 5.0, 6.0), base_from_lidar);

  EXPECT_NEAR(lidar_twist.linear.x, 2.0, kTolerance);
  EXPECT_NEAR(lidar_twist.linear.y, -1.0, kTolerance);
  EXPECT_NEAR(lidar_twist.linear.z, 3.0, kTolerance);
  EXPECT_NEAR(lidar_twist.angular.x, 5.0, kTolerance);
  EXPECT_NEAR(lidar_twist.angular.y, -4.0, kTolerance);
  EXPECT_NEAR(lidar_twist.angular.z, 6.0, kTolerance);
}

TEST(GroundTruthTwistTransform, AppliesLeverArmBeforeRotatingIntoChildFrame)
{
  tf2::Quaternion base_from_lidar_rotation;
  base_from_lidar_rotation.setRPY(0.0, 0.0, M_PI_2);
  const tf2::Transform base_from_lidar(
    base_from_lidar_rotation, tf2::Vector3(1.0, 0.0, 0.0));
  const auto lidar_twist = transformTwistFromBaseToChild(
    makeTwist(1.0, 0.0, 0.0, 0.0, 0.0, 2.0), base_from_lidar);

  EXPECT_NEAR(lidar_twist.linear.x, 2.0, kTolerance);
  EXPECT_NEAR(lidar_twist.linear.y, -1.0, kTolerance);
  EXPECT_NEAR(lidar_twist.linear.z, 0.0, kTolerance);
  EXPECT_NEAR(lidar_twist.angular.z, 2.0, kTolerance);
}

TEST(GroundTruthTwistTransform, TransformsTwistCovarianceWithTheSameAdjoint)
{
  geometry_msgs::msg::TwistWithCovariance base_twist;
  base_twist.covariance[5 * 6 + 5] = 4.0;
  const tf2::Transform base_from_lidar(
    tf2::Quaternion::getIdentity(), tf2::Vector3(0.0, 0.18, 0.0));

  const auto lidar_twist = transformTwistFromBaseToChild(base_twist, base_from_lidar);

  EXPECT_NEAR(lidar_twist.covariance[0 * 6 + 0], 0.1296, kTolerance);
  EXPECT_NEAR(lidar_twist.covariance[0 * 6 + 5], -0.72, kTolerance);
  EXPECT_NEAR(lidar_twist.covariance[5 * 6 + 0], -0.72, kTolerance);
  EXPECT_NEAR(lidar_twist.covariance[5 * 6 + 5], 4.0, kTolerance);
  for (std::size_t row = 0; row < 6; ++row) {
    for (std::size_t column = 0; column < 6; ++column) {
      if ((row == 0 && column == 0) || (row == 0 && column == 5) ||
        (row == 5 && column == 0) || (row == 5 && column == 5))
      {
        continue;
      }
      EXPECT_NEAR(lidar_twist.covariance[row * 6 + column], 0.0, kTolerance);
    }
  }
}

TEST(GroundTruthStateBuffer, DelayedUpperSampleCompletesBracket)
{
  GroundTruthStateBuffer buffer(2.0);
  ASSERT_TRUE(buffer.insert(makeState(1000000000LL, 0.0, 0.0, 0.0)));

  GroundTruthState output;
  EXPECT_EQ(
    buffer.interpolate(1050000000LL, 0.10, output),
    GroundTruthLookupStatus::NEED_BRACKET);

  ASSERT_TRUE(buffer.insert(makeState(1100000000LL, 10.0, M_PI_2, 10.0)));
  ASSERT_EQ(
    buffer.interpolate(1050000000LL, 0.10, output),
    GroundTruthLookupStatus::READY);
  EXPECT_EQ(output.stamp_nanoseconds, 1050000000LL);
  EXPECT_NEAR(output.odom_to_base.getOrigin().x(), 5.0, kTolerance);
  EXPECT_NEAR(output.odom_to_base.getOrigin().y(), 10.0, kTolerance);
  EXPECT_NEAR(output.odom_to_base.getOrigin().z(), -5.0, kTolerance);
}

TEST(GroundTruthStateBuffer, AcceptsOutOfOrderSamplesAndInterpolatesData)
{
  GroundTruthStateBuffer buffer(2.0);
  ASSERT_TRUE(buffer.insert(makeState(1200000000LL, 20.0, M_PI_2, 20.0)));
  ASSERT_TRUE(buffer.insert(makeState(1000000000LL, 0.0, 0.0, 0.0)));

  GroundTruthState output;
  ASSERT_EQ(
    buffer.interpolate(1100000000LL, 0.20, output),
    GroundTruthLookupStatus::READY);
  EXPECT_NEAR(output.odom_to_base.getOrigin().x(), 10.0, kTolerance);
  EXPECT_NEAR(output.base_twist.twist.linear.x, 10.0, kTolerance);
  EXPECT_NEAR(output.base_twist.twist.angular.z, 60.0, kTolerance);
  EXPECT_NEAR(output.pose_covariance[17], 10.0, kTolerance);
  EXPECT_NEAR(output.base_twist.covariance[31], 100.0, kTolerance);
}

TEST(GroundTruthStateBuffer, UsesShortestQuaternionSlerpPath)
{
  GroundTruthStateBuffer buffer(2.0);
  ASSERT_TRUE(buffer.insert(makeState(1000000000LL, 0.0, 170.0 * M_PI / 180.0, 0.0)));
  ASSERT_TRUE(buffer.insert(makeState(1100000000LL, 0.0, -170.0 * M_PI / 180.0, 0.0)));

  GroundTruthState output;
  ASSERT_EQ(
    buffer.interpolate(1050000000LL, 0.10, output),
    GroundTruthLookupStatus::READY);
  const tf2::Vector3 rotated_x = output.odom_to_base.getBasis() * tf2::Vector3(1.0, 0.0, 0.0);
  EXPECT_NEAR(rotated_x.x(), -1.0, 1.0e-9);
  EXPECT_NEAR(rotated_x.y(), 0.0, 1.0e-9);
}

TEST(GroundTruthStateBuffer, EnforcesInterpolationGapAndExactSampleBoundaries)
{
  GroundTruthStateBuffer buffer(2.0);
  ASSERT_TRUE(buffer.insert(makeState(1000000000LL, 0.0, 0.0, 0.0)));
  ASSERT_TRUE(buffer.insert(makeState(1100000000LL, 10.0, 0.0, 10.0)));

  GroundTruthState output;
  EXPECT_EQ(
    buffer.interpolate(1050000000LL, 0.099999999, output),
    GroundTruthLookupStatus::GAP_TOO_LARGE);
  EXPECT_EQ(
    buffer.interpolate(1050000000LL, 0.10, output),
    GroundTruthLookupStatus::READY);
  EXPECT_EQ(
    buffer.interpolate(1000000000LL, 0.001, output),
    GroundTruthLookupStatus::READY);
  EXPECT_EQ(
    buffer.interpolate(999999999LL, 0.10, output),
    GroundTruthLookupStatus::NEED_BRACKET);
  EXPECT_EQ(
    buffer.interpolate(1100000001LL, 0.10, output),
    GroundTruthLookupStatus::NEED_BRACKET);

  GroundTruthStateBuffer startup_buffer(2.0);
  ASSERT_TRUE(startup_buffer.insert(makeState(0, 0.0, 0.0, 0.0)));
  EXPECT_EQ(
    startup_buffer.interpolate(0, 0.10, output),
    GroundTruthLookupStatus::READY);
}

TEST(GroundTruthStateBuffer, RejectsNonFiniteStateWithoutPoisoningBuffer)
{
  GroundTruthStateBuffer buffer(2.0);
  GroundTruthState invalid = makeState(1000000000LL, 0.0, 0.0, 0.0);
  invalid.base_twist.twist.linear.x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(buffer.insert(invalid));
  EXPECT_EQ(buffer.size(), 0U);
}

TEST(GroundTruthStateBuffer, SteadyTimeoutIsFailClosedAtBoundary)
{
  using namespace std::chrono_literals;
  const std::chrono::steady_clock::time_point received{};

  EXPECT_FALSE(scanMatchExpired(received, received + 199ms, 0.20));
  EXPECT_TRUE(scanMatchExpired(received, received + 200ms, 0.20));
  EXPECT_TRUE(scanMatchExpired(received, received - 1ms, 0.20));
  EXPECT_TRUE(scanMatchExpired(received, received, 0.0));
}

}  // namespace
}  // namespace rm_27_stimulation
