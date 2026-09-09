#include <gtest/gtest.h>

#include <limits>
#include <string>

#include "minco_controller/input_validation.hpp"

namespace {

ros_interfaces::msg::MpcPositionCommand validCommand()
{
  ros_interfaces::msg::MpcPositionCommand command;
  command.command_flag = ros_interfaces::msg::MpcPositionCommand::NORMAL_COMMAND;
  command.mpc_horizon = 2U;
  command.cmds.resize(2);
  for (auto & point : command.cmds) {
    point.trajectory_id = 7U;
    point.position.x = 1.0;
    point.velocity.x = 0.2;
    point.yaw = 0.1;
  }
  return command;
}

}  // namespace

TEST(InputValidation, AcceptsCompleteFiniteNormalCommand)
{
  std::string reason;
  EXPECT_TRUE(minco_controller::input_validation::validNormalCommand(validCommand(), &reason));
  EXPECT_EQ(reason, "NONE");
}

TEST(InputValidation, RejectsBlockUnknownEmptyAndHorizonMismatch)
{
  std::string reason;
  auto command = validCommand();
  command.command_flag = ros_interfaces::msg::MpcPositionCommand::BLOCK_COMMAND;
  EXPECT_FALSE(minco_controller::input_validation::validNormalCommand(command, &reason));

  command.command_flag = 42U;
  EXPECT_FALSE(minco_controller::input_validation::validNormalCommand(command, &reason));

  command = validCommand();
  command.cmds.clear();
  command.mpc_horizon = 0U;
  EXPECT_FALSE(minco_controller::input_validation::validNormalCommand(command, &reason));

  command = validCommand();
  command.mpc_horizon = 1U;
  EXPECT_FALSE(minco_controller::input_validation::validNormalCommand(command, &reason));
  EXPECT_EQ(reason, "HORIZON_MISMATCH");
}

TEST(InputValidation, RejectsNonfiniteAndMixedTrajectoryIds)
{
  std::string reason;
  auto command = validCommand();
  command.cmds.back().jerk.y = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(minco_controller::input_validation::validNormalCommand(command, &reason));
  EXPECT_EQ(reason, "NONFINITE_TRAJECTORY");

  command = validCommand();
  command.cmds.back().trajectory_id = 8U;
  EXPECT_FALSE(minco_controller::input_validation::validNormalCommand(command, &reason));
  EXPECT_EQ(reason, "TRAJECTORY_ID_MISMATCH");
}

TEST(InputValidation, PoseAndOdometryRequireFiniteNonzeroQuaternion)
{
  geometry_msgs::msg::Pose pose;
  pose.orientation.w = 0.0;
  EXPECT_FALSE(minco_controller::input_validation::finitePose(pose));

  pose.orientation.w = 1.0;
  EXPECT_TRUE(minco_controller::input_validation::finitePose(pose));
  pose.position.x = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(minco_controller::input_validation::finitePose(pose));

  nav_msgs::msg::Odometry odom;
  odom.pose.pose.orientation.w = 1.0;
  EXPECT_TRUE(minco_controller::input_validation::finiteOdometry(odom));
  odom.twist.twist.angular.z = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(minco_controller::input_validation::finiteOdometry(odom));
}

TEST(InputValidation, StampFreshnessIsFailClosedAtInvalidStaleAndFutureValues)
{
  using minco_controller::input_validation::freshStamp;
  EXPECT_TRUE(freshStamp(9.8, 10.0, 0.25, 0.05));
  EXPECT_TRUE(freshStamp(10.05, 10.0, 0.25, 0.05));
  EXPECT_FALSE(freshStamp(9.7, 10.0, 0.25, 0.05));
  EXPECT_FALSE(freshStamp(10.051, 10.0, 0.25, 0.05));
  EXPECT_FALSE(freshStamp(0.0, 10.0, 0.25, 0.05));
  EXPECT_FALSE(freshStamp(std::numeric_limits<double>::quiet_NaN(), 10.0, 0.25, 0.05));
}
