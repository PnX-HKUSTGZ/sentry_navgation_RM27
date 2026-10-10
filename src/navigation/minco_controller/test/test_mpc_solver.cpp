#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>
#include <Eigen/Geometry>

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

TEST(MpcSolver, CrossTrackCorrectionFollowsHolonomicMotionAndRotatesWithPath) {
  auto config = testConfig();
  config.q_along = 3.0;
  config.q_cross = 60.0;
  config.R = Eigen::Vector3d(1.5, 1.5, 1.0);
  config.use_acc_constraints = false;
  for (const bool stopped_reference : {false, true}) {
    Eigen::Vector2d baseline;
    for (const double angle : {0.0, 0.785398163397, 1.570796326795, -0.7}) {
      const Eigen::Rotation2Dd rotation(angle);
      minco_controller::State state;
      const Eigen::Vector2d position = rotation * Eigen::Vector2d(0.0, 0.04);
      state.x = position.x();
      state.y = position.y();
      std::vector<minco_controller::ReferencePoint> reference(3);
      for (size_t i = 0; i < reference.size(); ++i) {
        reference[i].pos = rotation * Eigen::Vector2d(0.05 * (i + 1), 0.0);
        reference[i].vel = rotation * Eigen::Vector2d(stopped_reference ? 0.0 : 0.5, 0.0);
        reference[i].yaw = 0.0;  // The chassis does not turn with its path.
      }
      minco_controller::MpcSolver solver(config);
      minco_controller::Control control;
      ASSERT_TRUE(solver.solve(state, reference, control));
      const Eigen::Vector2d command(control.vx, control.vy);
      if (angle == 0.0) {
        baseline = command;
        EXPECT_LT(baseline.y(), -0.05);
      } else {
        EXPECT_TRUE(command.isApprox(rotation * baseline, 1.0e-5)) << "angle=" << angle;
      }
    }
  }
}

TEST(MpcSolver, StationaryFrontierBootstrapReachesCommandDeadzone) {
  auto config = testConfig();
  config.dt = 0.05;
  config.horizon = 10;
  config.Q = Eigen::Vector3d(3.0, 3.0, 2.0);
  config.q_along = 3.0;
  config.q_cross = 12.0;
  config.R = Eigen::Vector3d(1.5, 1.5, 1.0);
  config.use_acc_constraints = true;
  config.ax_min = -1.0;
  config.ax_max = 1.0;
  config.ay_min = -1.0;
  config.ay_max = 1.0;

  constexpr double kLength = 0.739;
  constexpr double kDuration = 4.0;
  constexpr double kBootstrapTime = 0.65;
  std::vector<minco_controller::ReferencePoint> reference(
      static_cast<size_t>(config.horizon));
  for (int i = 0; i < config.horizon; ++i) {
    const double t = kBootstrapTime + static_cast<double>(i) * config.dt;
    const double ratio = std::clamp(t / kDuration, 0.0, 1.0);
    const double ratio2 = ratio * ratio;
    const double ratio3 = ratio2 * ratio;
    const double ratio4 = ratio3 * ratio;
    const double ratio5 = ratio4 * ratio;
    reference[static_cast<size_t>(i)].pos.x() =
        kLength * (10.0 * ratio3 - 15.0 * ratio4 + 6.0 * ratio5);
    reference[static_cast<size_t>(i)].vel.x() =
        (kLength / kDuration) * (30.0 * ratio2 - 60.0 * ratio3 + 30.0 * ratio4);
  }

  minco_controller::MpcSolver solver(config);
  minco_controller::State state;
  minco_controller::Control control;
  ASSERT_TRUE(solver.solve(state, reference, control));
  EXPECT_GE(control.vx, 0.05 - 1.0e-8);
}

TEST(MpcSolver, HardwareFrontierBootstrapProducesAccelerationLimitedKick) {
  auto config = testConfig();
  config.dt = 0.05;
  config.horizon = 10;
  config.Q = Eigen::Vector3d(3.0, 3.0, 2.0);
  config.q_along = 3.0;
  config.q_cross = 12.0;
  config.R = Eigen::Vector3d(1.5, 1.5, 1.0);
  config.use_acc_constraints = true;
  config.ax_min = -0.8;
  config.ax_max = 0.8;
  config.ay_min = -0.8;
  config.ay_max = 0.8;

  constexpr double kLength = 0.739;
  constexpr double kDuration = 7.0;
  constexpr double kBootstrapTime = 1.40;
  std::vector<minco_controller::ReferencePoint> reference(
      static_cast<size_t>(config.horizon));
  for (int i = 0; i < config.horizon; ++i) {
    const double t = kBootstrapTime + static_cast<double>(i) * config.dt;
    const double ratio = std::clamp(t / kDuration, 0.0, 1.0);
    const double ratio2 = ratio * ratio;
    const double ratio3 = ratio2 * ratio;
    const double ratio4 = ratio3 * ratio;
    const double ratio5 = ratio4 * ratio;
    reference[static_cast<size_t>(i)].pos.x() =
        kLength * (10.0 * ratio3 - 15.0 * ratio4 + 6.0 * ratio5);
    reference[static_cast<size_t>(i)].vel.x() =
        (kLength / kDuration) * (30.0 * ratio2 - 60.0 * ratio3 + 30.0 * ratio4);
  }

  minco_controller::MpcSolver solver(config);
  minco_controller::State state;
  minco_controller::Control control;
  ASSERT_TRUE(solver.solve(state, reference, control));
  EXPECT_GT(control.vx, 0.0);
  EXPECT_LE(control.vx, config.ax_max * config.dt + 1.0e-8);
  EXPECT_LT(control.vx, 0.05);
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
