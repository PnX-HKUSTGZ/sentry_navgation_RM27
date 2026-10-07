#include <gtest/gtest.h>

#include <limits>

#include "minco_controller/reference_progress.hpp"

TEST(ReferenceProgress, NewTrajectoryStartsAtSpatialPickup) {
  double spatial = -1.0;
  double index = -1.0;
  ASSERT_TRUE(minco_controller::computeReferenceIndex(
      2.5, false, 90.0, 90.0, 10.0, 0.05, 100.0, 0.25, spatial, index));
  EXPECT_DOUBLE_EQ(spatial, 2.5);
  EXPECT_DOUBLE_EQ(index, 2.5);
}

TEST(ReferenceProgress, SameTrajectoryAdvancesAndNeverMovesBackward) {
  double spatial = -1.0;
  double index = -1.0;
  ASSERT_TRUE(minco_controller::computeReferenceIndex(
      0.0, true, 0.0, 0.0, 0.05, 0.05, 100.0, 0.25, spatial, index));
  EXPECT_NEAR(index, 1.0, 1e-12);

  ASSERT_TRUE(minco_controller::computeReferenceIndex(
      3.0, true, 3.0, 8.0, 0.05, 0.05, 100.0, 0.25, spatial, index));
  EXPECT_NEAR(index, 8.0, 1e-12);

  ASSERT_TRUE(minco_controller::computeReferenceIndex(
      2.8, true, spatial, index, 0.05, 0.05, 100.0, 0.25, spatial, index));
  EXPECT_NEAR(spatial, 3.0, 1e-12);
  EXPECT_NEAR(index, 8.0, 1e-12);
}

TEST(ReferenceProgress, TimeAdvanceIsBoundedBySpatialPickup) {
  double spatial = -1.0;
  double index = -1.0;
  ASSERT_TRUE(minco_controller::computeReferenceIndex(
      4.0, true, 4.0, 4.0, 1.0, 0.05, 100.0, 0.25, spatial, index));
  EXPECT_NEAR(index, 9.0, 1e-12);

  ASSERT_TRUE(minco_controller::computeReferenceIndex(
      8.0, true, spatial, index, 0.05, 0.05, 100.0, 0.25, spatial, index));
  EXPECT_NEAR(index, 10.0, 1e-12);
}

TEST(ReferenceProgress, ClampsAtTrajectoryEndAndRejectsInvalidTime) {
  double spatial = -1.0;
  double index = -1.0;
  ASSERT_TRUE(minco_controller::computeReferenceIndex(
      8.0, true, 8.0, 9.0, 1.0, 0.05, 10.0, 0.25, spatial, index));
  EXPECT_DOUBLE_EQ(index, 10.0);

  EXPECT_FALSE(minco_controller::computeReferenceIndex(
      0.0, true, 0.0, 0.0, -0.01, 0.05, 10.0, 0.25, spatial, index));
  EXPECT_FALSE(minco_controller::computeReferenceIndex(
      0.0, true, 0.0, 0.0, std::numeric_limits<double>::quiet_NaN(), 0.05, 10.0,
      0.25, spatial, index));
  EXPECT_FALSE(minco_controller::computeReferenceIndex(
      0.0, true, 0.0, 0.0, 0.05, 0.05, 10.0, -0.01, spatial, index));
}

TEST(ReferenceProgress, StationaryStartupFindsFirstEffectiveSpeed) {
  const std::vector<double> speeds{0.0,  0.01, 0.03, 0.06,
                                   0.09, 0.11, 0.14, 0.10};
  double index = 2.0;
  ASSERT_TRUE(minco_controller::applyStationaryStartupReferenceFloor(
      speeds, 0.0, 0.03, 0.10, 0.05, 0.75, 0.0, 7.0, index));
  EXPECT_DOUBLE_EQ(index, 5.0);
}

TEST(ReferenceProgress, MovingRobotKeepsNormalReference) {
  const std::vector<double> speeds{0.0, 0.04, 0.08, 0.12, 0.16};
  double index = 1.5;
  ASSERT_TRUE(minco_controller::applyStationaryStartupReferenceFloor(
      speeds, 0.04, 0.03, 0.10, 0.05, 0.75, 1.0, 4.0, index));
  EXPECT_DOUBLE_EQ(index, 1.5);
}

TEST(ReferenceProgress, StationaryStartupRespectsLeadAndTrajectoryEnd) {
  const std::vector<double> speeds{0.0, 0.02, 0.04, 0.12, 0.15};
  double index = 0.0;
  ASSERT_TRUE(minco_controller::applyStationaryStartupReferenceFloor(
      speeds, 0.0, 0.03, 0.10, 0.05, 0.10, 0.0, 4.0, index));
  EXPECT_DOUBLE_EQ(index, 0.0);

  index = 3.5;
  ASSERT_TRUE(minco_controller::applyStationaryStartupReferenceFloor(
      speeds, 0.0, 0.03, 0.10, 0.05, 0.75, 3.5, 4.0, index));
  EXPECT_DOUBLE_EQ(index, 4.0);
}

TEST(ReferenceProgress, StationaryStartupRejectsInvalidConfiguration) {
  const std::vector<double> speeds{0.0, 0.1};
  double index = 0.0;
  EXPECT_FALSE(minco_controller::applyStationaryStartupReferenceFloor(
      speeds, 0.0, 0.03, 0.10, 0.0, 0.75, 0.0, 1.0, index));
  EXPECT_FALSE(minco_controller::applyStationaryStartupReferenceFloor(
      speeds, 0.0, 0.03, -0.10, 0.05, 0.75, 0.0, 1.0, index));
}

TEST(ReferenceProgress, EffectiveStartupReferenceBypassesOnlySoftwareDeadzone) {
  EXPECT_TRUE(minco_controller::shouldSuppressPlanarCommand(0.04, 0.05, false));
  EXPECT_FALSE(minco_controller::shouldSuppressPlanarCommand(0.04, 0.05, true));
  EXPECT_FALSE(
      minco_controller::shouldSuppressPlanarCommand(0.05, 0.05, false));
}

TEST(ReferenceProgress, StationaryStartupRaisesSmallCommandAlongMpcDirection) {
  double vx = 0.03;
  double vy = 0.04;
  ASSERT_TRUE(minco_controller::applyStationaryStartupCommandFloor(
      true, 0.12, 1.0, 0.0, vx, vy));
  EXPECT_NEAR(std::hypot(vx, vy), 0.12, 1.0e-12);
  EXPECT_GT(vx, 0.0);
  EXPECT_GT(vy, 0.0);
}

TEST(ReferenceProgress, StationaryStartupUsesReferenceForZeroCommand) {
  double vx = 0.0;
  double vy = 0.0;
  ASSERT_TRUE(minco_controller::applyStationaryStartupCommandFloor(
      true, 0.12, -3.0, 4.0, vx, vy));
  EXPECT_NEAR(vx, -0.072, 1.0e-12);
  EXPECT_NEAR(vy, 0.096, 1.0e-12);
}

TEST(ReferenceProgress, StartupCommandFloorDoesNotAffectNormalTracking) {
  double vx = 0.02;
  double vy = 0.0;
  ASSERT_TRUE(minco_controller::applyStationaryStartupCommandFloor(
      false, 0.12, 1.0, 0.0, vx, vy));
  EXPECT_DOUBLE_EQ(vx, 0.02);
  EXPECT_DOUBLE_EQ(vy, 0.0);
}
