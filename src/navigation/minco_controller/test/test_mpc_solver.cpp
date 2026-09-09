#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

#include "minco_controller/mpc_solver.hpp"

namespace {

minco_controller::MPCConfig testConfig() {
  minco_controller::MPCConfig config;
  config.dt = 0.1;
  config.horizon = 3;
  config.vx_min = -1.0;
  config.vx_max = 1.0;
  config.vy_min = -1.0;
  config.vy_max = 1.0;
  config.omega_min = -1.0;
  config.omega_max = 1.0;
  return config;
}

std::vector<minco_controller::ReferencePoint> testReference() {
  std::vector<minco_controller::ReferencePoint> reference(3);
  for (size_t i = 0; i < reference.size(); ++i) {
    reference[i].pos = Eigen::Vector2d(1.0 + 0.1 * static_cast<double>(i), 0.0);
    reference[i].vel = Eigen::Vector2d(0.5, 0.0);
  }
  return reference;
}

} // namespace

TEST(MpcSolver, ValidProblemProducesFiniteBoundedControl) {
  minco_controller::MpcSolver solver(testConfig());
  minco_controller::State state;
  minco_controller::Control control;
  std::vector<minco_controller::State> prediction;

  ASSERT_TRUE(solver.solve(state, testReference(), control, &prediction));
  EXPECT_TRUE(std::isfinite(control.vx));
  EXPECT_TRUE(std::isfinite(control.vy));
  EXPECT_TRUE(std::isfinite(control.omega));
  EXPECT_GE(control.vx, -1.0);
  EXPECT_LE(control.vx, 1.0);
  EXPECT_LE(std::hypot(control.vx, control.vy), 1.0 + 1.0e-9);
  EXPECT_EQ(prediction.size(), 4U);
}

TEST(MpcSolver, RadialSpeedLimitClosesIndependentAxisConstraintCorners) {
  auto config = testConfig();
  config.max_planar_speed = 0.4;
  minco_controller::MpcSolver solver(config);
  minco_controller::State state;
  minco_controller::Control control;
  auto reference = testReference();
  for (auto &point : reference) {
    point.pos = Eigen::Vector2d(10.0, 10.0);
    point.vel = Eigen::Vector2d(1.0, 1.0);
  }

  ASSERT_TRUE(solver.solve(state, reference, control));
  EXPECT_LE(std::hypot(control.vx, control.vy), 0.4 + 1.0e-9);
  EXPECT_LE(solver.getLastControl().head<2>().norm(), 0.4 + 1.0e-9);
}

TEST(MpcSolver, ResetClearsPreviousControl) {
  minco_controller::MpcSolver solver(testConfig());
  minco_controller::State state;
  minco_controller::Control control;
  ASSERT_TRUE(solver.solve(state, testReference(), control));

  solver.reset();
  EXPECT_TRUE(solver.getLastControl().isZero(0.0));
}

TEST(MpcSolver, NonfiniteReferenceFailsClosedAndClearsWarmState) {
  minco_controller::MpcSolver solver(testConfig());
  minco_controller::State state;
  minco_controller::Control control;
  ASSERT_TRUE(solver.solve(state, testReference(), control));

  auto invalid_reference = testReference();
  invalid_reference[1].yaw = std::numeric_limits<double>::quiet_NaN();
  control.vx = 99.0;
  control.vy = 99.0;
  control.omega = 99.0;
  EXPECT_FALSE(solver.solve(state, invalid_reference, control));
  EXPECT_DOUBLE_EQ(control.vx, 0.0);
  EXPECT_DOUBLE_EQ(control.vy, 0.0);
  EXPECT_DOUBLE_EQ(control.omega, 0.0);
  EXPECT_TRUE(solver.getLastControl().isZero(0.0));
}

TEST(MpcSolver, InvalidBoundsFailBeforeQpAndClearOutput) {
  auto config = testConfig();
  config.vx_min = 1.0;
  config.vx_max = -1.0;
  minco_controller::MpcSolver solver(config);
  minco_controller::State state;
  minco_controller::Control control;
  control.vx = 99.0;

  EXPECT_FALSE(solver.solve(state, testReference(), control));
  EXPECT_DOUBLE_EQ(control.vx, 0.0);
  EXPECT_TRUE(solver.getLastControl().isZero(0.0));
}

TEST(MpcSolver, InvalidCostWeightsFailClosed) {
  auto config = testConfig();
  config.q_cross = -1.0;
  minco_controller::MpcSolver solver(config);
  minco_controller::State state;
  minco_controller::Control control;
  control.vx = 99.0;

  EXPECT_FALSE(solver.solve(state, testReference(), control));
  EXPECT_DOUBLE_EQ(control.vx, 0.0);
  EXPECT_TRUE(solver.getLastControl().isZero(0.0));
}

TEST(MpcSolver, InvalidRadialSpeedLimitFailsClosed) {
  auto config = testConfig();
  config.max_planar_speed = 0.0;
  minco_controller::MpcSolver solver(config);
  minco_controller::State state;
  minco_controller::Control control;

  EXPECT_FALSE(solver.solve(state, testReference(), control));
  EXPECT_DOUBLE_EQ(control.vx, 0.0);
  EXPECT_TRUE(solver.getLastControl().isZero(0.0));
}
