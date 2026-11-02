// Copyright 2026 sentry-navigation-RM27 contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#include "traj_opt/minco_optimizer.hpp"
#include "minco_core/components/start_guide_pruning.hpp"
#include "minco_core/components/segment_speed_limit.hpp"
#include "minco_core/minco_utils.hpp"

namespace minco_planner
{
namespace
{

TEST(SegmentSpeedLimit, AnObstacleDoesNotSlowTheWholeHorizon)
{
  const auto distance = [](const Eigen::Vector3d &p) {
    return std::hypot(p.x() - 1.5, p.y() - 0.30);
  };
  for (int i = 0; i < 3; ++i) {
    const double limit = segmentObstacleSpeedLimit(
      Eigen::Vector3d(i, 0, 0), Eigen::Vector3d(i + 1, 0, 0),
      1.8, 0.45, 1.35, distance);
    EXPECT_DOUBLE_EQ(limit, i == 1 ? 1.35 : 1.8);
  }
}

TEST(SegmentSpeedLimit, ChecksInteriorAndPropagatesNecessaryBraking)
{
  const double cap = segmentObstacleSpeedLimit(
    Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(1, 0, 0), 1.8, 0.20, 1.0,
    [](const Eigen::Vector3d &p) {return std::abs(p.x() - 0.5);});
  ASSERT_DOUBLE_EQ(cap, 1.0);
  std::vector<double> limits{1.8, cap, 1.8};
  utils::VelPropogation({0.10, 0.10, 1.0}, 2.0, limits);
  EXPECT_LT(limits.front(), 1.8);
  EXPECT_LE(limits.front() * limits.front(), 1.0 + 2.0 * 2.0 * 0.10 + 1.0e-9);
  EXPECT_DOUBLE_EQ(limits[1], 1.0);
}

TEST(SegmentSpeedLimit, UnknownDistanceDoesNotReplaceCollisionAuthorization)
{
  EXPECT_DOUBLE_EQ(segmentObstacleSpeedLimit(
    Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitX(), 1.8, 0.45, 1.35,
    [](const Eigen::Vector3d &) {return std::numeric_limits<double>::quiet_NaN();}), 1.8);
}

TEST(SegmentSpeedLimit, HolonomicTurnTuningKeepsGentleCornersMoving)
{
  const double gentle = utils::calCurvatureDecay(0.7853981634, 1.8, 0.20, 1.57, 0.65, 1.0);
  const double sharp = utils::calCurvatureDecay(1.5707963268, 1.8, 0.20, 1.57, 0.65, 1.0);
  EXPECT_GT(gentle, 1.4);
  EXPECT_LT(sharp, gentle);
  EXPECT_LT(sharp, 1.2);
  EXPECT_DOUBLE_EQ(utils::calCurvatureDecay(0.10, 1.8, 0.20, 1.57, 0.65, 1.0), 1.8);
}

MincoOptimizer::Config makeConfig()
{
  MincoOptimizer::Config config;
  config.max_vel = 0.7;
  config.max_acc = 1.0;
  config.rho = 100.0;
  config.smooth_eps = 0.01;
  config.integral_res = 16;
  config.opt_accuracy = 0.001;
  config.time_allocation_iters = 15;
  config.print_optimizer_log = false;
  config.magnitudeBounds.resize(3);
  config.magnitudeBounds << 0.3, config.max_vel, config.max_acc;
  config.penaltyWeights.resize(5);
  config.penaltyWeights << 0.0, 1500.0, 1500.0, 100.0, 50.0;
  return config;
}

void expectFeasible(const Eigen::Vector3d & start_velocity)
{
  auto config = makeConfig();
  MincoOptimizer optimizer(config);

  const std::vector<Eigen::Vector3d> waypoints{
    Eigen::Vector3d(0.0, 0.0, 0.0),
    Eigen::Vector3d(0.5, 0.1, 0.0),
    Eigen::Vector3d(1.0, 0.0, 0.0)};
  Eigen::Matrix3d start_state = Eigen::Matrix3d::Zero();
  start_state.col(0) = waypoints.front();
  start_state.col(1) = start_velocity;
  Eigen::Matrix3d end_state = Eigen::Matrix3d::Zero();
  end_state.col(0) = waypoints.back();

  vec_Vec3f initial_points{waypoints[1]};
  VecDf initial_times(2);
  initial_times << 0.3, 0.3;
  optimizer.setInitPsAndTs(initial_points, initial_times);

  VecDf local_velocity_limits(2);
  local_velocity_limits.setConstant(config.max_vel);
  geometry_utils::Trajectory trajectory;
  const double result = optimizer.optimize(
    waypoints, start_state, end_state, local_velocity_limits, trajectory,
    config.max_vel);

  ASSERT_TRUE(std::isfinite(result));
  ASSERT_FALSE(trajectory.empty());
  EXPECT_DOUBLE_EQ(result, optimizer.lastObjectiveTotal());
  EXPECT_LE(trajectory.getMaxVelRate(), config.max_vel * 1.001);
  EXPECT_LE(trajectory.getMaxAccRate(), config.max_acc * 1.001);
  EXPECT_TRUE(trajectory.getVel(0.0).isApprox(start_velocity, 1.0e-6));
  EXPECT_TRUE(trajectory.getAcc(0.0).isApprox(Eigen::Vector3d::Zero(), 1.0e-6));
  const double duration = trajectory.getTotalDuration();
  EXPECT_TRUE(trajectory.getPos(duration).isApprox(waypoints.back(), 1.0e-6));
  EXPECT_NEAR(trajectory.getVel(duration).norm(), 0.0, 1.0e-6);
  EXPECT_NEAR(trajectory.getAcc(duration).norm(), 0.0, 1.0e-6);
  EXPECT_GT(optimizer.lastTimeAllocationIterations(), 0);
  EXPECT_TRUE(std::isfinite(optimizer.lastPeakVelocity()));
  EXPECT_TRUE(std::isfinite(optimizer.lastPeakAcceleration()));
}

TEST(MincoOptimizerDynamicsTest, RetimesTrajectoryWithStationaryBoundary)
{
  expectFeasible(Eigen::Vector3d::Zero());
}

TEST(MincoOptimizerDynamicsTest, OpenGridPickupDoesNotForceMovingChassisToReverse)
{
  auto config = makeConfig();
  config.max_vel = 1.2;
  config.magnitudeBounds << 0.33, config.max_vel, config.max_acc;
  config.guide_lateral_weight = 1000000.0;
  config.guide_lateral_tolerance = 0.003;
  config.opt_accuracy = 1.0e-5;
  std::vector<Eigen::Vector3d> path{{0, 0, 0}, {-0.01, 0.05, 0},
    {0.25, 0.05, 0}, {0.5, 0.05, 0}, {0.75, 0.05, 0}, {1.0, 0.05, 0}, {1.5, 0.05, 0}};
  Eigen::Matrix3d start = Eigen::Matrix3d::Zero(), end = Eigen::Matrix3d::Zero();
  start.col(1) = Eigen::Vector3d(1.0, 0.0, 0.0);
  end.col(0) = path.back();
  end.col(1) = Eigen::Vector3d(1.0, 0.0, 0.0);
  pruneOpenStartGuide(path, start.col(0), start.col(1), 0.025,
    [](const Eigen::Vector3d &) {return true;});
  EXPECT_GT(path[1].x(), 0.20);
  MincoOptimizer optimizer(config);
  geometry_utils::Trajectory trajectory;
  const VecDf limits = VecDf::Constant(path.size() - 1U, config.max_vel);
  ASSERT_TRUE(std::isfinite(optimizer.optimize(path, start, end, limits, trajectory, config.max_vel)));
  EXPECT_LT(trajectory.getTotalDuration(), 4.0);
  EXPECT_LE(trajectory.getMaxVelRate(), config.max_vel * 1.02);
  EXPECT_LE(trajectory.getMaxAccRate(), config.max_acc * 1.02);
  EXPECT_TRUE(trajectory.getVel(0.0).isApprox(start.col(1), 1.0e-6));
}

TEST(MincoOptimizerDynamicsTest, PickupPruningPreservesCornerWhenReplacementChordIsBlocked)
{
  std::vector<Eigen::Vector3d> path{{0, 0, 0}, {0, 0.1, 0}, {0.3, 0.1, 0}, {1, 0.1, 0}};
  const auto original = path;
  // Endpoints are clear, but a wall crosses the middle of the shortcut.
  pruneOpenStartGuide(path, path.front(), {1, 0, 0}, 0.025,
    [](const Eigen::Vector3d & point) {return point.x() < 0.10 || point.x() > 0.20;});
  ASSERT_EQ(path.size(), original.size());
  EXPECT_TRUE(path[1].isApprox(original[1]));
}

TEST(MincoOptimizerDynamicsTest, RetimesTrajectoryAndPreservesNonzeroStartVelocity)
{
  const Eigen::Vector3d start_velocity(0.35, 0.0, 0.0);
  expectFeasible(start_velocity);
}

TEST(MincoOptimizerDynamicsTest, RejectsNonpositiveTimeAllocationIterations)
{
  auto config = makeConfig();
  config.time_allocation_iters = 0;
  EXPECT_THROW(MincoOptimizer optimizer(config), std::invalid_argument);

  config.time_allocation_iters = -1;
  EXPECT_THROW(MincoOptimizer optimizer(config), std::invalid_argument);
}

TEST(MincoOptimizerDynamicsTest, RejectsInvalidTerminalVelocityRatio)
{
  auto config = makeConfig();
  config.terminal_velocity_ratio = 0.0;
  EXPECT_THROW(MincoOptimizer optimizer(config), std::invalid_argument);

  config.terminal_velocity_ratio = 1.01;
  EXPECT_THROW(MincoOptimizer optimizer(config), std::invalid_argument);

  config.terminal_velocity_ratio = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(MincoOptimizer optimizer(config), std::invalid_argument);
}

TEST(MincoOptimizerDynamicsTest, RejectsTrajectoryThatRetimesIntoLongCrawl)
{
  auto config = makeConfig();
  config.max_trajectory_duration = 0.1;
  MincoOptimizer optimizer(config);
  const std::vector<Eigen::Vector3d> waypoints{
    Eigen::Vector3d::Zero(), Eigen::Vector3d(1.0, 0.0, 0.0)};
  Eigen::Matrix3d start_state = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d end_state = Eigen::Matrix3d::Zero();
  end_state.col(0) = waypoints.back();
  VecDf local_velocity_limits(1);
  local_velocity_limits << config.max_vel;
  geometry_utils::Trajectory trajectory;

  const double result = optimizer.optimize(
    waypoints, start_state, end_state, local_velocity_limits, trajectory,
    config.max_vel);

  EXPECT_FALSE(std::isfinite(result));
  EXPECT_TRUE(trajectory.empty());
  EXPECT_GT(optimizer.lastTotalDuration(), config.max_trajectory_duration);
  EXPECT_EQ(optimizer.lastFailureReason(),
            MincoOptimizer::FailureReason::DURATION_LIMIT);
}

TEST(MincoOptimizerDynamicsTest, EnforcesPerTrajectoryVelocityLimit)
{
  auto config = makeConfig();
  MincoOptimizer optimizer(config);
  const std::vector<Eigen::Vector3d> waypoints{
    Eigen::Vector3d::Zero(), Eigen::Vector3d(0.4, 0.0, 0.0)};
  Eigen::Matrix3d start_state = Eigen::Matrix3d::Zero();
  Eigen::Matrix3d end_state = Eigen::Matrix3d::Zero();
  end_state.col(0) = waypoints.back();
  VecDf initial_times(1);
  initial_times << 0.5;
  optimizer.setInitPsAndTs({}, initial_times);
  constexpr double kBootstrapVelocity = 0.25;
  VecDf local_velocity_limits(1);
  local_velocity_limits << kBootstrapVelocity;
  geometry_utils::Trajectory trajectory;

  const double result = optimizer.optimize(
    waypoints, start_state, end_state, local_velocity_limits, trajectory,
    kBootstrapVelocity);

  ASSERT_TRUE(std::isfinite(result));
  ASSERT_FALSE(trajectory.empty());
  EXPECT_LE(trajectory.getMaxVelRate(), kBootstrapVelocity * 1.001);
  EXPECT_LE(trajectory.getMaxAccRate(), config.max_acc * 1.001);
}

TEST(MincoOptimizerDynamicsTest, FailsClosedWhenBoundaryStateExceedsLimit)
{
  auto config = makeConfig();
  MincoOptimizer optimizer(config);
  const std::vector<Eigen::Vector3d> waypoints{
    Eigen::Vector3d::Zero(), Eigen::Vector3d(1.0, 0.0, 0.0)};
  Eigen::Matrix3d start_state = Eigen::Matrix3d::Zero();
  start_state.col(1) = Eigen::Vector3d(2.0 * config.max_vel, 0.0, 0.0);
  Eigen::Matrix3d end_state = Eigen::Matrix3d::Zero();
  end_state.col(0) = waypoints.back();
  VecDf local_velocity_limits(1);
  local_velocity_limits << config.max_vel;
  geometry_utils::Trajectory trajectory;

  const double result = optimizer.optimize(
    waypoints, start_state, end_state, local_velocity_limits, trajectory,
    config.max_vel);

  EXPECT_FALSE(std::isfinite(result));
  EXPECT_TRUE(trajectory.empty());
  EXPECT_EQ(optimizer.lastFailureReason(),
            MincoOptimizer::FailureReason::DYNAMIC_FEASIBILITY);
}

TEST(MincoOptimizerDynamicsTest, RollingEndpointDoesNotRetimeIntoCrawlOrRequireStop)
{
  auto config = makeConfig();
  config.max_vel = 1.5;
  config.max_acc = 1.0;
  config.terminal_velocity_ratio = 0.85;
  config.magnitudeBounds << 0.3, config.max_vel, config.max_acc;
  MincoOptimizer optimizer(config);
  const std::vector<Eigen::Vector3d> waypoints{
    Eigen::Vector3d(0.0, 0.0, 0.0),
    Eigen::Vector3d(0.37, 0.0, 0.0),
    Eigen::Vector3d(0.74, 0.0, 0.0),
    Eigen::Vector3d(1.11, 0.0, 0.0),
    Eigen::Vector3d(1.48, 0.0, 0.0)};
  Eigen::Matrix3d start_state = Eigen::Matrix3d::Zero();
  start_state.col(0) = waypoints.front();
  Eigen::Matrix3d end_state = Eigen::Matrix3d::Zero();
  end_state.col(0) = waypoints.back();
  end_state.col(1) = Eigen::Vector3d(
    config.max_vel * config.terminal_velocity_ratio, 0.0, 0.0);
  VecDf local_velocity_limits(waypoints.size() - 1U);
  local_velocity_limits.setConstant(config.max_vel);
  geometry_utils::Trajectory trajectory;

  const double rolling_result = optimizer.optimize(
    waypoints, start_state, end_state, local_velocity_limits, trajectory,
    config.max_vel);
  ASSERT_TRUE(std::isfinite(rolling_result)) << "duration=" << optimizer.lastTotalDuration();
  ASSERT_FALSE(trajectory.empty());
  EXPECT_LT(trajectory.getTotalDuration(), 4.0);
  EXPECT_TRUE(trajectory.getVel(0.0).isApprox(start_state.col(1), 1.0e-6));
  EXPECT_TRUE(trajectory.getVel(trajectory.getTotalDuration()).isApprox(end_state.col(1), 1.0e-6));
  EXPECT_LE(trajectory.getMaxVelRate(), config.max_vel * 1.02);
  EXPECT_LE(trajectory.getMaxAccRate(), config.max_acc * 1.02);

  end_state.col(1).setZero();
  const double stopped_result = optimizer.optimize(
    waypoints, start_state, end_state, local_velocity_limits, trajectory,
    config.max_vel);
  EXPECT_TRUE(std::isfinite(stopped_result));
  EXPECT_FALSE(trajectory.empty());
  EXPECT_LE(trajectory.getMaxVelRate(), config.max_vel * 1.001);
}

TEST(MincoOptimizerDynamicsTest, GuidePenaltyControlsBowingBetweenKnots)
{
  auto config = makeConfig();
  config.max_vel = 1.5;
  config.max_acc = 1.8;
  config.magnitudeBounds << 0.31, config.max_vel, config.max_acc;
  config.penaltyWeights(3) = 800.0;
  config.guide_lateral_weight = 1000000.0;
  config.guide_lateral_tolerance = 0.003;
  MincoOptimizer optimizer(config);
  const std::vector<Eigen::Vector3d> waypoints{
    {0.0, -0.10, 0.0}, {0.35, 0.0, 0.0}, {0.70, 0.0, 0.0}, {1.10, 0.0, 0.0}};
  Eigen::Matrix3d start = Eigen::Matrix3d::Zero(), end = Eigen::Matrix3d::Zero();
  start.col(0) = waypoints.front();
  start.col(1) = Eigen::Vector3d(0.8, 0.0, 0.0);
  end.col(0) = waypoints.back();
  end.col(1) = Eigen::Vector3d(1.0, 0.0, 0.0);
  VecDf limits = VecDf::Constant(3, config.max_vel);
  geometry_utils::Trajectory trajectory;
  ASSERT_TRUE(std::isfinite(optimizer.optimize(
    waypoints, start, end, limits, trajectory, config.max_vel)));
  ASSERT_EQ(trajectory.getPieceNum(), 3);
  double lateral_peak = 0.0;
  for (int i = 1; i < trajectory.getPieceNum(); ++i) {
    for (int j = 0; j <= 100; ++j) {
      const auto point = trajectory[i].getPos(trajectory[i].getDuration() * j / 100.0);
      lateral_peak = std::max(lateral_peak, std::abs(point.y()));
    }
  }
  EXPECT_LT(lateral_peak, 0.025);
  EXPECT_LT(trajectory.getTotalDuration(), 4.0);
  EXPECT_LE(trajectory.getMaxVelRate(), config.max_vel * 1.02);
  EXPECT_LE(trajectory.getMaxAccRate(), config.max_acc * 1.02);
  EXPECT_TRUE(trajectory.getVel(0.0).isApprox(start.col(1), 1.0e-6));
  EXPECT_TRUE(trajectory.getVel(trajectory.getTotalDuration()).isApprox(end.col(1), 1.0e-6));
}

TEST(MincoOptimizerDynamicsTest, DenseRollingGuideKeepsShortDurationWithBoundaryAcceleration)
{
  for (const Eigen::Vector2d & boundary : std::vector<Eigen::Vector2d>{
      {0.9, -1.7}, {0.9, 0.0}, {0.9, 1.7}, {1.0, 0.0}}) {
    const double acceleration = boundary.y();
    SCOPED_TRACE(acceleration);
    SCOPED_TRACE(boundary.x());
    auto config = makeConfig();
    config.max_vel = 1.5;
    config.max_acc = 1.8;
    config.magnitudeBounds << 0.31, config.max_vel, config.max_acc;
    config.penaltyWeights(3) = 800.0;
    config.guide_lateral_weight = 1000000.0;
    config.guide_lateral_tolerance = 0.003;
    config.opt_accuracy = 0.00001;
    MincoOptimizer optimizer(config);
    std::vector<Eigen::Vector3d> waypoints;
    for (int i = 0; i <= 10; ++i) {
      waypoints.emplace_back(0.15 * i, 0.0, 0.0);
    }
    Eigen::Matrix3d start = Eigen::Matrix3d::Zero(), end = Eigen::Matrix3d::Zero();
    start.col(1) = Eigen::Vector3d(boundary.x(), 0.0, 0.0);
    start.col(2) = Eigen::Vector3d(acceleration, 0.0, 0.0);
    end.col(0) = waypoints.back();
    end.col(1) = Eigen::Vector3d(0.85, 0.0, 0.0);
    VecDf limits = VecDf::Constant(10, 1.0);
    geometry_utils::Trajectory trajectory;
    ASSERT_TRUE(std::isfinite(optimizer.optimize(waypoints, start, end, limits, trajectory, 1.0)))
      << optimizer.lastTotalDuration();
    EXPECT_LT(trajectory.getTotalDuration(), 4.0);
    EXPECT_LE(trajectory.getMaxVelRate(), 1.02);
    EXPECT_LE(trajectory.getMaxAccRate(), config.max_acc * 1.02);
    EXPECT_TRUE(trajectory.getAcc(0.0).isApprox(start.col(2), 1.0e-6));
    EXPECT_TRUE(trajectory.getVel(trajectory.getTotalDuration()).isApprox(end.col(1), 1.0e-6));
  }
}

}  // namespace
}  // namespace minco_planner
