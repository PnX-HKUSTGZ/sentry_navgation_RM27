#include <chrono>
#include <limits>

#include <gtest/gtest.h>

#include "rm_27_stimulation/ground_truth_odometry.hpp"

namespace rm_27_stimulation {

TEST(GroundTruthOdometry, RotatesPhysicsVelocitiesIntoTiltedBodyFrame) {
  const gz::math::Pose3d pose(-4.7, -3.4, 0.22, -0.25, 0.12, 0.8);
  const gz::math::Vector3d linear(0.6, -0.8, 0.15);
  const gz::math::Vector3d angular(0.02, 0.03, -0.1);
  const auto message = MakeGroundTruthOdometry(
      pose, pose.Rot().RotateVector(linear), pose.Rot().RotateVector(angular),
      std::chrono::nanoseconds(1234567890), "world", "base_footprint");
  ASSERT_TRUE(message);
  EXPECT_NEAR(message->twist().linear().x(), linear.X(), 1e-12);
  EXPECT_NEAR(message->twist().linear().y(), linear.Y(), 1e-12);
  EXPECT_NEAR(message->twist().linear().z(), linear.Z(), 1e-12);
  EXPECT_NEAR(message->twist().angular().x(), angular.X(), 1e-12);
  EXPECT_NEAR(message->twist().angular().y(), angular.Y(), 1e-12);
  EXPECT_NEAR(message->twist().angular().z(), angular.Z(), 1e-12);
  EXPECT_EQ(message->header().stamp().sec(), 1);
  EXPECT_EQ(message->header().stamp().nsec(), 234567890);
  EXPECT_EQ(message->header().data(0).value(0), "world");
  EXPECT_EQ(message->header().data(1).value(0), "base_footprint");
}

TEST(GroundTruthOdometry, TiltedRestRemainsFiniteWithoutPoseDifferencing) {
  gz::math::Pose3d pose(0, 0, 0.2, -0.25, 0.001, 0.002);
  pose.Rot().W() *= 1.0000000001;
  const auto message = MakeGroundTruthOdometry(
      pose, gz::math::Vector3d::Zero, gz::math::Vector3d::Zero,
      std::chrono::seconds(10), "world", "base_footprint");
  ASSERT_TRUE(message);
  EXPECT_EQ(message->twist().angular().x(), 0.0);
  EXPECT_EQ(message->twist().angular().y(), 0.0);
  EXPECT_EQ(message->twist().angular().z(), 0.0);
  EXPECT_NEAR(message->pose().orientation().w() * message->pose().orientation().w() +
                  message->pose().orientation().x() * message->pose().orientation().x() +
                  message->pose().orientation().y() * message->pose().orientation().y() +
                  message->pose().orientation().z() * message->pose().orientation().z(),
              1.0, 1e-12);
}

TEST(GroundTruthOdometry, RejectsInvalidPhysicsInsteadOfInventingZeroVelocity) {
  const auto nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(MakeGroundTruthOdometry(
      gz::math::Pose3d::Zero, {nan, 0, 0}, gz::math::Vector3d::Zero,
      std::chrono::seconds(1), "world", "base_footprint"));
  EXPECT_FALSE(MakeGroundTruthOdometry(
      gz::math::Pose3d::Zero, gz::math::Vector3d::Zero, {0, 0, nan},
      std::chrono::seconds(1), "world", "base_footprint"));
  EXPECT_FALSE(MakeGroundTruthOdometry(
      gz::math::Pose3d(nan, 0, 0, 0, 0, 0), gz::math::Vector3d::Zero,
      gz::math::Vector3d::Zero, std::chrono::seconds(1), "world", "base_footprint"));
}

} // namespace rm_27_stimulation
