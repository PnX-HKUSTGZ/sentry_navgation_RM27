#include "minco_controller/input_validation.hpp"

#include <cmath>
#include <cstdint>

namespace minco_controller::input_validation {
namespace {

bool finiteVector3(const geometry_msgs::msg::Vector3 & value)
{
  return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

bool finitePoint(const geometry_msgs::msg::Point & value)
{
  return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

void setReason(std::string * reason, const char * value)
{
  if (reason) {
    *reason = value;
  }
}

}  // namespace

bool finitePose(const geometry_msgs::msg::Pose & pose)
{
  if (!finitePoint(pose.position) || !std::isfinite(pose.orientation.x) ||
      !std::isfinite(pose.orientation.y) || !std::isfinite(pose.orientation.z) ||
      !std::isfinite(pose.orientation.w)) {
    return false;
  }

  const double norm_squared =
    pose.orientation.x * pose.orientation.x + pose.orientation.y * pose.orientation.y +
    pose.orientation.z * pose.orientation.z + pose.orientation.w * pose.orientation.w;
  return std::isfinite(norm_squared) && norm_squared > 1.0e-12;
}

bool finiteOdometry(const nav_msgs::msg::Odometry & odom)
{
  return finitePose(odom.pose.pose) && finiteVector3(odom.twist.twist.linear) &&
         finiteVector3(odom.twist.twist.angular);
}

bool validNormalCommand(const ros_interfaces::msg::MpcPositionCommand & command, std::string * reason)
{
  if (command.command_flag != ros_interfaces::msg::MpcPositionCommand::NORMAL_COMMAND &&
      command.command_flag != ros_interfaces::msg::MpcPositionCommand::BRAKING_COMMAND &&
      command.command_flag != ros_interfaces::msg::MpcPositionCommand::RECOVERY_COMMAND) {
    setReason(reason, "NOT_NORMAL");
    return false;
  }
  if (command.cmds.empty()) {
    setReason(reason, "EMPTY_TRAJECTORY");
    return false;
  }
  if (static_cast<size_t>(command.mpc_horizon) != command.cmds.size()) {
    setReason(reason, "HORIZON_MISMATCH");
    return false;
  }

  const uint32_t trajectory_id = command.cmds.front().trajectory_id;
  for (const auto & point : command.cmds) {
    if (point.trajectory_id != trajectory_id) {
      setReason(reason, "TRAJECTORY_ID_MISMATCH");
      return false;
    }
    if (!finitePoint(point.position) || !finiteVector3(point.velocity) ||
        !finiteVector3(point.acceleration) || !finiteVector3(point.jerk) ||
        !finiteVector3(point.angular_velocity) || !std::isfinite(point.yaw) ||
        !std::isfinite(point.yaw_dot) || !std::isfinite(point.vel_norm) || !std::isfinite(point.acc_norm)) {
      setReason(reason, "NONFINITE_TRAJECTORY");
      return false;
    }
    for (const double gain : point.kx) {
      if (!std::isfinite(gain)) {
        setReason(reason, "NONFINITE_TRAJECTORY");
        return false;
      }
    }
    for (const double gain : point.kv) {
      if (!std::isfinite(gain)) {
        setReason(reason, "NONFINITE_TRAJECTORY");
        return false;
      }
    }
  }

  setReason(reason, "NONE");
  return true;
}

bool validRecoveryCommand(const ros_interfaces::msg::MpcPositionCommand & command,
  double max_speed, double max_distance, double max_duration, std::string * reason)
{
  if (command.command_flag != ros_interfaces::msg::MpcPositionCommand::RECOVERY_COMMAND ||
      !validNormalCommand(command, reason) || command.cmds.size() < 2U ||
      !std::isfinite(max_speed) || max_speed <= 0.0 ||
      !std::isfinite(max_distance) || max_distance <= 0.0 ||
      !std::isfinite(max_duration) || max_duration <= 0.0) {
    setReason(reason, "INVALID_RECOVERY");
    return false;
  }
  const auto & first = command.cmds.front();
  const double speed = std::hypot(first.velocity.x, first.velocity.y);
  if (speed <= 1.0e-6 || speed > max_speed + 1.0e-6) {
    setReason(reason, "RECOVERY_SPEED");
    return false;
  }
  const double dx = first.velocity.x / speed, dy = first.velocity.y / speed;
  double previous_progress = 0.0;
  for (const auto & point : command.cmds) {
    const double x = point.position.x - first.position.x;
    const double y = point.position.y - first.position.y;
    const double progress = x * dx + y * dy;
    if (progress < previous_progress - 1.0e-6 ||
        progress > max_distance + 1.0e-6 || progress / speed > max_duration + 1.0e-6 ||
        std::abs(x * dy - y * dx) > 1.0e-6 ||
        std::hypot(point.velocity.x - first.velocity.x, point.velocity.y - first.velocity.y) > 1.0e-6 ||
        std::abs(point.velocity.z) > 1.0e-6 ||
        std::abs(point.position.z - first.position.z) > 1.0e-6 ||
        std::abs(std::remainder(point.yaw - first.yaw, 2.0 * std::acos(-1.0))) > 1.0e-6 ||
        std::abs(point.yaw_dot) > 1.0e-6 ||
        std::hypot(point.acceleration.x, point.acceleration.y) > 1.0e-6) {
      setReason(reason, "RECOVERY_NOT_BOUNDED_STRAIGHT");
      return false;
    }
    previous_progress = progress;
  }
  if (previous_progress <= 1.0e-6) {
    setReason(reason, "RECOVERY_ZERO_LENGTH");
    return false;
  }
  setReason(reason, "NONE");
  return true;
}

double stampSeconds(const builtin_interfaces::msg::Time & stamp)
{
  if (stamp.sec < 0 || stamp.nanosec >= 1000000000U) {
    return -1.0;
  }
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1.0e-9;
}

bool freshStamp(
  double stamp_seconds, double now_seconds, double timeout_seconds, double future_tolerance_seconds)
{
  if (!std::isfinite(stamp_seconds) || !std::isfinite(now_seconds) || !std::isfinite(timeout_seconds) ||
      timeout_seconds <= 0.0 || !std::isfinite(future_tolerance_seconds) ||
      future_tolerance_seconds < 0.0 || stamp_seconds <= 0.0) {
    return false;
  }

  const double age = now_seconds - stamp_seconds;
  constexpr double kTimeEpsilon = 1.0e-9;
  return std::isfinite(age) && age >= -future_tolerance_seconds - kTimeEpsilon &&
         age <= timeout_seconds + kTimeEpsilon;
}

}  // namespace minco_controller::input_validation
