#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "minco_controller/tilt_speed_limit.hpp"

namespace minco_controller::tilt_speed_limit {
namespace {

TEST(TiltSpeedLimit, LeavesFlatGroundAtFullSpeed) {
  EXPECT_DOUBLE_EQ(speedLimit(0.01, 0.02, 0.08, 0.18, 1.0, 0.6), 1.0);
}

TEST(TiltSpeedLimit, InterpolatesAndSaturatesOnSlope) {
  EXPECT_NEAR(speedLimit(0.0, 0.13, 0.08, 0.18, 1.0, 0.6), 0.8, 1.0e-12);
  EXPECT_DOUBLE_EQ(speedLimit(0.0, 0.30, 0.08, 0.18, 1.0, 0.6), 0.6);
}

TEST(TiltSpeedLimit, AppliesRadialLimitWithoutChangingDirection) {
  double vx = 0.6;
  double vy = 0.8;
  ASSERT_TRUE(apply(0.5, vx, vy));
  EXPECT_NEAR(vx, 0.3, 1.0e-12);
  EXPECT_NEAR(vy, 0.4, 1.0e-12);
}

TEST(TiltSpeedLimit, InvalidInputFailsClosed) {
  double vx = 1.0;
  double vy = 0.0;
  EXPECT_FALSE(apply(std::numeric_limits<double>::quiet_NaN(), vx, vy));
  EXPECT_DOUBLE_EQ(vx, 0.0);
  EXPECT_DOUBLE_EQ(vy, 0.0);
  EXPECT_DOUBLE_EQ(speedLimit(0.0, 0.0, 0.2, 0.1, 1.0, 0.6), 0.0);
}

TEST(TiltSpeedLimit, UphillFloorActivationPreservesFrontierSpeedLimit) {
  EXPECT_TRUE(shouldApplyUphillCommandFloor(true, 0.20, 0.30));
  EXPECT_FALSE(shouldApplyUphillCommandFloor(false, 0.20, 0.30));
  EXPECT_TRUE(shouldApplyUphillCommandFloor(false, 0.45, 0.30));
  EXPECT_FALSE(shouldApplyUphillCommandFloor(
      false, std::numeric_limits<double>::quiet_NaN(), 0.30));
}

TEST(TiltSpeedLimit, SmoothlyBlendsCommandAcrossUphillGradeRange) {
  double vx = 0.20;
  double vy = 0.0;
  double grade = 0.0;
  double floor = 0.0;
  const double midpoint_grade = 0.105;
  ASSERT_TRUE(applyUphillCommandFloor(true, 0.03, 0.18, 0.90, 0.0,
                                      -std::asin(midpoint_grade), 0.0, vx, vy,
                                      grade, floor));
  EXPECT_NEAR(grade, midpoint_grade, 1.0e-12);
  EXPECT_NEAR(floor, 0.55, 1.0e-12);
  EXPECT_NEAR(vx, 0.55, 1.0e-12);
  EXPECT_NEAR(vy, 0.0, 1.0e-12);
}

TEST(TiltSpeedLimit, HasNoStepAtEntryAndReachesFullAssistOnSteepGrade) {
  double vx = 0.20;
  double vy = 0.0;
  double grade = 0.0;
  double floor = 0.0;
  ASSERT_TRUE(applyUphillCommandFloor(true, 0.03, 0.18, 0.90, 0.0,
                                      -std::asin(0.03), 0.0, vx, vy, grade,
                                      floor));
  EXPECT_NEAR(vx, 0.20, 1.0e-12);
  EXPECT_NEAR(floor, 0.20, 1.0e-12);

  ASSERT_TRUE(applyUphillCommandFloor(true, 0.03, 0.18, 0.90, 0.0,
                                      -std::asin(0.28), 0.0, vx, vy, grade,
                                      floor));
  EXPECT_NEAR(vx, 0.90, 1.0e-12);
  EXPECT_NEAR(floor, 0.90, 1.0e-12);
}

TEST(TiltSpeedLimit, DoesNotRaiseCommandDirectedDownhill) {
  double vx = -0.12;
  double vy = 0.0;
  double grade = 0.0;
  double floor = 0.0;

  ASSERT_TRUE(applyUphillCommandFloor(true, 0.03, 0.18, 0.90, 0.0, -0.17, 0.0,
                                      vx, vy, grade, floor));
  EXPECT_LT(grade, -0.16);
  EXPECT_NEAR(vx, -0.12, 1.0e-12);
}

TEST(TiltSpeedLimit, HandlesLateralUphillDirectionAndYaw) {
  // With yaw=90 degrees, global +x is body -y. Negative roll makes body -y
  // point uphill.
  double vx = 0.10;
  double vy = 0.0;
  double grade = 0.0;
  double floor = 0.0;
  ASSERT_TRUE(applyUphillCommandFloor(true, 0.03, 0.12, 0.80, -0.20, 0.0,
                                      1.5707963267948966, vx, vy, grade,
                                      floor));
  EXPECT_GT(grade, 0.14);
  EXPECT_NEAR(std::hypot(vx, vy), 0.80, 1.0e-12);
}

TEST(TiltSpeedLimit, DoesNotRaiseInactiveOrFlatCommands) {
  double vx = 0.10;
  double vy = 0.0;
  double grade = 0.0;
  double floor = 0.0;
  ASSERT_TRUE(applyUphillCommandFloor(false, 0.03, 0.18, 0.90, 0.0, -0.17, 0.0,
                                      vx, vy, grade, floor));
  EXPECT_NEAR(vx, 0.10, 1.0e-12);

  ASSERT_TRUE(applyUphillCommandFloor(true, 0.03, 0.18, 0.90, 0.0, 0.0, 0.0, vx,
                                      vy, grade, floor));
  EXPECT_NEAR(vx, 0.10, 1.0e-12);
  EXPECT_NEAR(grade, 0.0, 1.0e-12);
}

TEST(TiltSpeedLimit, RejectsInvalidAssistGradeRange) {
  double vx = 0.10;
  double vy = 0.0;
  double grade = 0.0;
  double floor = 0.0;
  EXPECT_FALSE(applyUphillCommandFloor(true, 0.18, 0.18, 0.90, 0.0, -0.20, 0.0,
                                       vx, vy, grade, floor));
}

} // namespace
} // namespace minco_controller::tilt_speed_limit
