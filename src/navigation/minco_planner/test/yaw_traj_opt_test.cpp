#include <gtest/gtest.h>

#include "traj_opt/yaw_traj_opt.h"

namespace traj_opt
{
namespace
{

geometry_utils::Trajectory makeVerticalPositionTrajectory(double duration)
{
  Eigen::MatrixXd coefficients(3, 6);
  coefficients.setZero();
  coefficients(1, 4) = 1.0;
  geometry_utils::Trajectory trajectory;
  trajectory.emplace_back(duration, coefficients);
  return trajectory;
}

TEST(YawTrajOptTest, ExplicitOmniGoalIsNotReplacedByPathTangent)
{
  constexpr double kInitialYaw = 0.305;
  constexpr double kGoalYaw = 0.0;
  const auto position = makeVerticalPositionTrajectory(1.0);
  Eigen::Vector4d initial_state = Eigen::Vector4d::Zero();
  Eigen::Vector4d goal_state = Eigen::Vector4d::Zero();
  initial_state(0) = kInitialYaw;
  goal_state(0) = kGoalYaw;

  YawTrajOpt optimizer(1.2);
  geometry_utils::Trajectory yaw;
  ASSERT_TRUE(optimizer.optimize(
      initial_state, goal_state, position, yaw, 5, false, false));
  ASSERT_FALSE(yaw.empty());

  EXPECT_NEAR(yaw.getPos(0.0).x(), kInitialYaw, 1.0e-9);
  EXPECT_NEAR(yaw.getPos(yaw.getTotalDuration()).x(), kGoalYaw, 1.0e-9);
  EXPECT_LE(yaw.getMaxVelRate(), 1.2 + 2.0);
}

}  // namespace
}  // namespace traj_opt
