#include <gtest/gtest.h>

#include <limits>

#include "minco_controller/reference_progress.hpp"

TEST(ReferenceProgress, NewTrajectoryStartsAtSpatialPickup) {
  double spatial = -1.0;
  double index = -1.0;
  ASSERT_TRUE(minco_controller::computeReferenceIndex(
      2.5, false, 90.0, 90.0, 10.0, 0.05, 100.0, 0.25,
      spatial, index));
  EXPECT_DOUBLE_EQ(spatial, 2.5);
  EXPECT_DOUBLE_EQ(index, 2.5);
}

TEST(ReferenceProgress, SameTrajectoryAdvancesAndNeverMovesBackward) {
  double spatial = -1.0;
  double index = -1.0;
  ASSERT_TRUE(minco_controller::computeReferenceIndex(
      0.0, true, 0.0, 0.0, 0.05, 0.05, 100.0, 0.25,
      spatial, index));
  EXPECT_NEAR(index, 1.0, 1e-12);

  ASSERT_TRUE(minco_controller::computeReferenceIndex(
      3.0, true, 3.0, 8.0, 0.05, 0.05, 100.0, 0.25,
      spatial, index));
  EXPECT_NEAR(index, 8.0, 1e-12);

  ASSERT_TRUE(minco_controller::computeReferenceIndex(
      2.8, true, spatial, index, 0.05, 0.05, 100.0, 0.25,
      spatial, index));
  EXPECT_NEAR(spatial, 3.0, 1e-12);
  EXPECT_NEAR(index, 8.0, 1e-12);
}

TEST(ReferenceProgress, TimeAdvanceIsBoundedBySpatialPickup) {
  double spatial = -1.0;
  double index = -1.0;
  ASSERT_TRUE(minco_controller::computeReferenceIndex(
      4.0, true, 4.0, 4.0, 1.0, 0.05, 100.0, 0.25,
      spatial, index));
  EXPECT_NEAR(index, 9.0, 1e-12);

  ASSERT_TRUE(minco_controller::computeReferenceIndex(
      8.0, true, spatial, index, 0.05, 0.05, 100.0, 0.25,
      spatial, index));
  EXPECT_NEAR(index, 10.0, 1e-12);
}

TEST(ReferenceProgress, ClampsAtTrajectoryEndAndRejectsInvalidTime) {
  double spatial = -1.0;
  double index = -1.0;
  ASSERT_TRUE(minco_controller::computeReferenceIndex(
      8.0, true, 8.0, 9.0, 1.0, 0.05, 10.0, 0.25,
      spatial, index));
  EXPECT_DOUBLE_EQ(index, 10.0);

  EXPECT_FALSE(minco_controller::computeReferenceIndex(
      0.0, true, 0.0, 0.0, -0.01, 0.05, 10.0, 0.25,
      spatial, index));
  EXPECT_FALSE(minco_controller::computeReferenceIndex(
      0.0, true, 0.0, 0.0,
      std::numeric_limits<double>::quiet_NaN(), 0.05, 10.0, 0.25,
      spatial, index));
  EXPECT_FALSE(minco_controller::computeReferenceIndex(
      0.0, true, 0.0, 0.0, 0.05, 0.05, 10.0, -0.01,
      spatial, index));
}
