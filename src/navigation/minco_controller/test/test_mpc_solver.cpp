#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>
#include <utility>
#include <Eigen/Geometry>

#include "minco_controller/mpc_solver.hpp"
#include "minco_controller/odometry_prediction.hpp"

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

TEST(MpcSolver, DeltaPenaltyDampsNoisyLateralCorrectionsWithoutSteadySpeedLoss) {
  auto config = testConfig();
  config.dt = 0.05;
  config.horizon = 20;
  config.q_cross = 120.0;
  config.R = {1.5, 1.5, 1.0};
  config.use_acc_constraints = true;
  struct Metrics { double variation{0.0}; double max_error{0.0}; double speed{0.0}; };
  const auto track = [&](double weight) {
    auto cfg = config;
    cfg.control_delta_weight = {weight, weight, 0.5};
    minco_controller::MpcSolver solver(cfg);
    minco_controller::State truth;
    truth.y = 0.04;
    minco_controller::Control previous;
    Metrics result;
    for (int step = 0; step < 240; ++step) {
      auto measured = truth;
      // Alternating centimetre-scale localization error at a 20 Hz loop.
      measured.y += (step % 2 ? -0.01 : 0.01);
      std::vector<minco_controller::ReferencePoint> reference(cfg.horizon);
      for (int k = 0; k < cfg.horizon; ++k) {
        reference[k].pos = {truth.x + 0.5 * (k + 1) * cfg.dt, 0.0};
        reference[k].vel = {0.5, 0.0};
      }
      minco_controller::Control command;
      EXPECT_TRUE(solver.solve(measured, reference, command));
      EXPECT_LE(std::abs(command.vy - previous.vy), cfg.ay_max * cfg.dt + 1e-6);
      // First-order chassis response, matching the simulation's ~0.13 s lag.
      truth.vx += (command.vx - truth.vx) * (1.0 - std::exp(-cfg.dt / 0.13));
      truth.vy += (command.vy - truth.vy) * (1.0 - std::exp(-cfg.dt / 0.13));
      truth.x += truth.vx * cfg.dt;
      truth.y += truth.vy * cfg.dt;
      if (step >= 80) {
        result.variation += std::pow(command.vy - previous.vy, 2);
        result.max_error = std::max(result.max_error, std::abs(truth.y));
        result.speed += truth.vx / 160.0;
      }
      previous = command;
    }
    return result;
  };
  const auto baseline = track(0.0);
  RecordProperty("baseline_lateral_delta_energy", baseline.variation);
  for (const double weight : {3.0, 6.0}) {
    SCOPED_TRACE(weight);
    const auto damped = track(weight);
    EXPECT_LT(damped.variation, 0.5 * baseline.variation);
    EXPECT_LT(damped.max_error, 0.005);
    EXPECT_NEAR(damped.speed, 0.5, 0.01);
    RecordProperty(weight == 3.0 ? "damped3_lateral_delta_energy" :
        "damped6_lateral_delta_energy", damped.variation);
  }
}

TEST(MpcSolver, DeltaPenaltyResetUsesMeasuredVelocityAndInvalidWeightsFailClosed) {
  auto config = testConfig();
  config.control_delta_weight = {3.0, 3.0, 0.5};
  minco_controller::MpcSolver solver(config);
  minco_controller::State state;
  state.vx = 0.5;
  auto reference = testReference();
  for (size_t i = 0; i < reference.size(); ++i) {
    reference[i].pos = {0.5 * (i + 1) * config.dt, 0.0};
  }
  minco_controller::Control command;
  ASSERT_TRUE(solver.solve(state, reference, command));
  EXPECT_NEAR(command.vx, 0.5, 1e-6);
  ASSERT_TRUE(solver.solve(state, testReference(), command));
  solver.reset();
  ASSERT_TRUE(solver.solve(state, reference, command));
  EXPECT_NEAR(command.vx, 0.5, 1e-6);
  for (double invalid : {-1.0, std::numeric_limits<double>::quiet_NaN(),
                          std::numeric_limits<double>::infinity()}) {
    config.control_delta_weight.y() = invalid;
    solver.setConfig(config);
    EXPECT_FALSE(solver.solve(state, reference, command));
    EXPECT_DOUBLE_EQ(command.vx, 0.0);
    EXPECT_TRUE(solver.getLastControl().isZero());
  }
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

TEST(OdometryPrediction, UsesWorldVelocityAndMeasuredAgeWithoutChangingVelocity) {
  minco_controller::State state{1., 2., 3.13, -.4, .6, .5};
  ASSERT_TRUE(minco_controller::predictOdometryToNow(state, 10., 10.15, .25, .05));
  EXPECT_NEAR(state.x, .94, 1.e-12); EXPECT_NEAR(state.y, 2.09, 1.e-12);
  EXPECT_NEAR(state.yaw, std::atan2(std::sin(3.205), std::cos(3.205)), 1.e-12);
  EXPECT_DOUBLE_EQ(state.vx, -.4); EXPECT_DOUBLE_EQ(state.vy, .6);
  auto future = state;
  EXPECT_TRUE(minco_controller::predictOdometryToNow(future, 10.02, 10., .25, .05));
  EXPECT_DOUBLE_EQ(future.x, state.x); EXPECT_DOUBLE_EQ(future.y, state.y);
  for (const double stamp : {9.7, 10.06, std::numeric_limits<double>::quiet_NaN()}) {
    auto invalid = state;
    EXPECT_FALSE(minco_controller::predictOdometryToNow(invalid, stamp, 10., .25, .05));
    EXPECT_DOUBLE_EQ(invalid.x, state.x);
  }
}

TEST(MpcSolver, TimestampPredictionReducesCrossTrackOvershoot) {
  // 10 Hz odometry, 100 ms delivery delay, 20 Hz MPC and a 130 ms force
  // actuator. Start 8 cm beside a straight path. No truth enters the controller;
  // the simulated plant state is used only to produce delayed observations.
  auto simulate = [](bool compensate) {
    auto config = testConfig(); config.dt = .05; config.horizon = 20;
    config.q_cross = 60.; config.R = Eigen::Vector3d(1.5,1.5,1.);
    config.control_delta_weight = Eigen::Vector3d(6.,6.,.5);
    config.use_acc_constraints = true;
    config.ax_min = config.ay_min = -1.5; config.ax_max = config.ay_max = 1.5;
    minco_controller::MpcSolver solver(config);
    minco_controller::State actual; actual.y = .08; actual.vx = .6;
    std::vector<minco_controller::State> history;
    double total = 0., min_y = .08;
    for (int k = 0; k < 200; ++k) {
      history.push_back(actual);
      const int sample = std::max(0, (k-2)/2*2);
      auto state = history[sample];
      if (compensate) {
        EXPECT_TRUE(minco_controller::predictOdometryToNow(
            state, 1.+sample*.05, 1.+k*.05, .25, .05));
      }
      state.x += state.vx*.12; state.y += state.vy*.12;
      std::vector<minco_controller::ReferencePoint> ref(20);
      for (int i=0; i<20; ++i) {
        ref[i].pos = Eigen::Vector2d(state.x+.6*(i+1)*.05, 0.);
        ref[i].vel = Eigen::Vector2d(.6,0.);
      }
      minco_controller::Control u;
      EXPECT_TRUE(solver.solve(state, ref, u));
      const double decay = std::exp(-.05/.13);
      actual.vx = decay*actual.vx+(1.-decay)*u.vx;
      actual.vy = decay*actual.vy+(1.-decay)*u.vy;
      actual.x += actual.vx*.05; actual.y += actual.vy*.05;
      total += std::abs(actual.vy)*.05; min_y = std::min(min_y, actual.y);
    }
    return std::make_pair(total, std::max(0., -min_y));
  };
  const auto delayed = simulate(false), corrected = simulate(true);
  // Measure the complete transient, including its initial overshoot.
  EXPECT_GT(delayed.second, .008);
  EXPECT_LT(corrected.second, .001);
  EXPECT_LT(corrected.first, delayed.first*.9);
}
