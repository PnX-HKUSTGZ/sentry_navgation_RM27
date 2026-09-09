#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "minco_controller/tilt_speed_limit.hpp"

namespace minco_controller::tilt_speed_limit
{
namespace
{

TEST(TiltSpeedLimit, LeavesFlatGroundAtFullSpeed)
{
  EXPECT_DOUBLE_EQ(speedLimit(0.01, 0.02, 0.08, 0.18, 1.0, 0.6), 1.0);
}

TEST(TiltSpeedLimit, InterpolatesAndSaturatesOnSlope)
{
  EXPECT_NEAR(speedLimit(0.0, 0.13, 0.08, 0.18, 1.0, 0.6), 0.8, 1.0e-12);
  EXPECT_DOUBLE_EQ(speedLimit(0.0, 0.30, 0.08, 0.18, 1.0, 0.6), 0.6);
}

TEST(TiltSpeedLimit, AppliesRadialLimitWithoutChangingDirection)
{
  double vx = 0.6;
  double vy = 0.8;
  ASSERT_TRUE(apply(0.5, vx, vy));
  EXPECT_NEAR(vx, 0.3, 1.0e-12);
  EXPECT_NEAR(vy, 0.4, 1.0e-12);
}

TEST(TiltSpeedLimit, InvalidInputFailsClosed)
{
  double vx = 1.0;
  double vy = 0.0;
  EXPECT_FALSE(apply(std::numeric_limits<double>::quiet_NaN(), vx, vy));
  EXPECT_DOUBLE_EQ(vx, 0.0);
  EXPECT_DOUBLE_EQ(vy, 0.0);
  EXPECT_DOUBLE_EQ(speedLimit(0.0, 0.0, 0.2, 0.1, 1.0, 0.6), 0.0);
}

}  // namespace
}  // namespace minco_controller::tilt_speed_limit
