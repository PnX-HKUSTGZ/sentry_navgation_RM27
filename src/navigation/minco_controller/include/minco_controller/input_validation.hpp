#pragma once

#include <string>

#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ros_interfaces/msg/mpc_position_command.hpp"

namespace minco_controller::input_validation {

bool finitePose(const geometry_msgs::msg::Pose & pose);

bool finiteOdometry(const nav_msgs::msg::Odometry & odom);

bool validNormalCommand(
  const ros_interfaces::msg::MpcPositionCommand & command, std::string * reason = nullptr);

bool freshStamp(
  double stamp_seconds, double now_seconds, double timeout_seconds, double future_tolerance_seconds);

double stampSeconds(const builtin_interfaces::msg::Time & stamp);

}  // namespace minco_controller::input_validation
