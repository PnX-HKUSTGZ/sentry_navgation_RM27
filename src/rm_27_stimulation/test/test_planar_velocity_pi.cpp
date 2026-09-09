#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "rm_27_stimulation/planar_velocity_pi.hpp"

namespace rm_27_stimulation
{
namespace
{

TEST(PlanarVelocityPi, ZeroIntegralGainPreservesProportionalServo)
{
  PlanarVelocityPi controller;
  const auto force = controller.Update(0.5, -0.25, 0.01, 80.0, 0.0, 0.0, 120.0);

  EXPECT_DOUBLE_EQ(force.x, 40.0);
  EXPECT_DOUBLE_EQ(force.y, -20.0);
  EXPECT_DOUBLE_EQ(controller.IntegralForce().x, 0.0);
  EXPECT_DOUBLE_EQ(controller.IntegralForce().y, 0.0);
}

TEST(PlanarVelocityPi, IntegralForceAccumulatesAndHasVectorLimit)
{
  PlanarVelocityPi controller;
  for (int i = 0; i < 100; ++i)
  {
    controller.Update(0.3, 0.4, 0.1, 0.0, 10.0, 5.0, 100.0);
  }

  const auto integral = controller.IntegralForce();
  EXPECT_NEAR(std::hypot(integral.x, integral.y), 5.0, 1e-12);
  EXPECT_NEAR(integral.x, 3.0, 1e-12);
  EXPECT_NEAR(integral.y, 4.0, 1e-12);
}

TEST(PlanarVelocityPi, TotalForceSaturationStopsWindupButAllowsUnwinding)
{
  PlanarVelocityPi controller;

  for (int i = 0; i < 20; ++i)
  {
    const auto force = controller.Update(10.0, 0.0, 0.1, 20.0, 10.0, 50.0, 100.0);
    EXPECT_NEAR(force.x, 100.0, 1e-12);
  }
  EXPECT_DOUBLE_EQ(controller.IntegralForce().x, 0.0);

  controller.Update(-1.0, 0.0, 0.1, 20.0, 10.0, 50.0, 100.0);
  EXPECT_NEAR(controller.IntegralForce().x, -1.0, 1e-12);
}

TEST(PlanarVelocityPi, ResetAndInvalidInputDiscardStoredIntegral)
{
  PlanarVelocityPi controller;
  controller.Update(1.0, 0.0, 0.1, 0.0, 10.0, 10.0, 100.0);
  ASSERT_GT(controller.IntegralForce().x, 0.0);

  controller.Reset();
  EXPECT_DOUBLE_EQ(controller.IntegralForce().x, 0.0);

  controller.Update(1.0, 0.0, 0.1, 0.0, 10.0, 10.0, 100.0);
  const auto force = controller.Update(
      std::numeric_limits<double>::quiet_NaN(), 0.0, 0.1,
      80.0, 10.0, 10.0, 100.0);
  EXPECT_DOUBLE_EQ(force.x, 0.0);
  EXPECT_DOUBLE_EQ(force.y, 0.0);
  EXPECT_DOUBLE_EQ(controller.IntegralForce().x, 0.0);
}

}  // namespace
}  // namespace rm_27_stimulation
