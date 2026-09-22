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

namespace minco_planner
{
namespace
{

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
}

}  // namespace
}  // namespace minco_planner
