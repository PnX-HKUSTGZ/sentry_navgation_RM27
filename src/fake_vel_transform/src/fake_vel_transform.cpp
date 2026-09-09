// Copyright 2025 Lihan Chen
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

#include "fake_vel_transform/fake_vel_transform.hpp"

#include <cmath>
#include <stdexcept>

#include "fake_vel_transform/odom_validation.hpp"
#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace fake_vel_transform
{

constexpr double EPSILON = 1e-5;
constexpr double CONTROLLER_TIMEOUT = 0.5;

FakeVelTransform::FakeVelTransform(const rclcpp::NodeOptions & options)
: Node("fake_vel_transform", options)
{
  RCLCPP_INFO(get_logger(), "Start FakeVelTransform!");

  this->declare_parameter<std::string>("robot_base_frame", "gimbal_link");
  this->declare_parameter<std::string>("fake_robot_base_frame", "gimbal_link_fake");
  this->declare_parameter<std::string>("odom_topic", "odom");
  this->declare_parameter<std::string>("local_plan_topic", "local_plan");
  this->declare_parameter<std::string>("cmd_spin_topic", "cmd_spin");
  this->declare_parameter<std::string>("input_cmd_vel_topic", "");
  this->declare_parameter<std::string>("output_cmd_vel_topic", "");
  this->declare_parameter<std::string>("terrain_map_topic", "terrain_map");
  this->declare_parameter<bool>("enable_cmd_spin", true);
  this->declare_parameter<bool>("terrain_guard_enabled", false);
  this->declare_parameter<bool>("use_latest_odom_for_cmd", false);
  this->declare_parameter<double>("odom_timeout", 0.2);
  this->declare_parameter<double>("odom_future_tolerance", 0.05);
  this->declare_parameter<double>("terrain_timeout", 0.5);
  this->declare_parameter<float>("init_spin_speed", 0.0);

  this->get_parameter("robot_base_frame", robot_base_frame_);
  this->get_parameter("fake_robot_base_frame", fake_robot_base_frame_);
  this->get_parameter("odom_topic", odom_topic_);
  this->get_parameter("local_plan_topic", local_plan_topic_);
  this->get_parameter("cmd_spin_topic", cmd_spin_topic_);
  this->get_parameter("input_cmd_vel_topic", input_cmd_vel_topic_);
  this->get_parameter("output_cmd_vel_topic", output_cmd_vel_topic_);
  this->get_parameter("terrain_map_topic", terrain_map_topic_);
  this->get_parameter("enable_cmd_spin", enable_cmd_spin_);
  this->get_parameter("terrain_guard_enabled", terrain_guard_enabled_);
  this->get_parameter("use_latest_odom_for_cmd", use_latest_odom_for_cmd_);
  this->get_parameter("odom_timeout", odom_timeout_);
  this->get_parameter("odom_future_tolerance", odom_future_tolerance_);
  this->get_parameter("terrain_timeout", terrain_timeout_);
  this->get_parameter("init_spin_speed", spin_speed_);

  if (!std::isfinite(odom_timeout_) || odom_timeout_ <= 0.0) {
    throw std::invalid_argument("fake_vel_transform.odom_timeout must be positive");
  }
  if (!std::isfinite(odom_future_tolerance_) || odom_future_tolerance_ < 0.0) {
    throw std::invalid_argument(
      "fake_vel_transform.odom_future_tolerance must be finite and nonnegative");
  }
  if (terrain_guard_enabled_ && (!std::isfinite(terrain_timeout_) || terrain_timeout_ <= 0.0)) {
    throw std::invalid_argument(
      "fake_vel_transform.terrain_timeout must be positive when the terrain guard is enabled");
  }
  if (!enable_cmd_spin_) {
    spin_speed_ = 0.0F;
  } else if (!std::isfinite(spin_speed_)) {
    throw std::invalid_argument("fake_vel_transform.init_spin_speed must be finite");
  }

  current_robot_base_angle_ = 0.0;
  last_controller_activate_time_ = this->now();

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  cmd_vel_chassis_pub_ =
    this->create_publisher<geometry_msgs::msg::Twist>(output_cmd_vel_topic_, 1);

  if (enable_cmd_spin_) {
    cmd_spin_sub_ = this->create_subscription<example_interfaces::msg::Float32>(
      cmd_spin_topic_, 1,
      std::bind(&FakeVelTransform::cmdSpinCallback, this, std::placeholders::_1));
  }
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    input_cmd_vel_topic_, 10,
    std::bind(&FakeVelTransform::cmdVelCallback, this, std::placeholders::_1));
  if (terrain_guard_enabled_) {
    terrain_map_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      terrain_map_topic_, rclcpp::SensorDataQoS().keep_last(1),
      std::bind(&FakeVelTransform::terrainMapCallback, this, std::placeholders::_1));
  }

  odom_sub_filter_.subscribe(this, odom_topic_);
  local_plan_sub_filter_.subscribe(this, local_plan_topic_);
  odom_sub_filter_.registerCallback(
    std::bind(&FakeVelTransform::odometryCallback, this, std::placeholders::_1));
  local_plan_sub_filter_.registerCallback(
    std::bind(&FakeVelTransform::localPlanCallback, this, std::placeholders::_1));

  // In Navigation2 Humble release, the velocity is published by the controller without timestamped.
  // We consider the velocity is published at the same time as local_plan.
  // Therefore, we use ApproximateTime policy to synchronize `cmd_vel` and `odometry`.
  sync_ = std::make_unique<message_filters::Synchronizer<SyncPolicy>>(
    SyncPolicy(100), odom_sub_filter_, local_plan_sub_filter_);
  sync_->registerCallback(
    std::bind(&FakeVelTransform::syncCallback, this, std::placeholders::_1, std::placeholders::_2));

  // 50Hz Timer to send transform from `robot_base_frame` to `fake_robot_base_frame`
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(20), std::bind(&FakeVelTransform::publishTransform, this));
}

void FakeVelTransform::cmdSpinCallback(const example_interfaces::msg::Float32::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
  if (!std::isfinite(msg->data)) {
    spin_speed_ = 0.0F;
    RCLCPP_WARN(get_logger(), "Rejecting non-finite spin command");
    return;
  }
  spin_speed_ = msg->data;
}

void FakeVelTransform::odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr & msg)
{
  const auto now = this->now();
  double yaw = 0.0;
  const char * reason = "UNKNOWN";
  if (!validOdometry(*msg, now, yaw, reason)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Rejecting odometry: %s", reason);
    return;
  }

  std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
  latest_odom_stamp_ = rclcpp::Time(msg->header.stamp, get_clock()->get_clock_type());
  has_latest_odom_stamp_ = true;

  // NOTE: Haven't synced with local_plan
  if (
    use_latest_odom_for_cmd_ ||
    (now - last_controller_activate_time_).seconds() > CONTROLLER_TIMEOUT) {
    current_robot_base_angle_ = yaw;
  }
}

void FakeVelTransform::terrainMapCallback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & /*msg*/)
{
  std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
  latest_terrain_receive_time_ = this->now();
  has_latest_terrain_ = true;
}

void FakeVelTransform::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
  const auto now = this->now();

  if (!odom_validation::finiteTwist(*msg)) {
    publishStop();
    motion_command_active_ = false;
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Rejecting non-finite velocity command");
    return;
  }

  if (use_latest_odom_for_cmd_) {
    if (!odometryFresh(now)) {
      publishStop();
      motion_command_active_ = false;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Rejecting velocity command because no valid odometry newer than %.3f s is available",
        odom_timeout_);
      return;
    }

    publishGuardedVelocity(transformVelocity(msg, current_robot_base_angle_), now);
    return;
  }

  const bool is_zero_vel = std::abs(msg->linear.x) < EPSILON && std::abs(msg->linear.y) < EPSILON &&
                           std::abs(msg->angular.z) < EPSILON;
  if (is_zero_vel || (now - last_controller_activate_time_).seconds() > CONTROLLER_TIMEOUT) {
    // If received velocity cannot be synchronized, publish it directly
    auto aft_tf_vel = transformVelocity(msg, current_robot_base_angle_);
    publishGuardedVelocity(aft_tf_vel, now);
  } else {
    latest_cmd_vel_ = msg;
  }
}

void FakeVelTransform::localPlanCallback(const nav_msgs::msg::Path::ConstSharedPtr & /*msg*/)
{
  std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
  // Consider nav2_controller_server is activated when receiving local_plan
  last_controller_activate_time_ = this->now();
}

void FakeVelTransform::syncCallback(
  const nav_msgs::msg::Odometry::ConstSharedPtr & odom_msg,
  const nav_msgs::msg::Path::ConstSharedPtr & /*local_plan_msg*/)
{
  const auto now = this->now();
  double yaw = 0.0;
  const char * reason = "UNKNOWN";
  if (!validOdometry(*odom_msg, now, yaw, reason)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000, "Rejecting synchronized odometry: %s", reason);
    return;
  }

  std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
  latest_odom_stamp_ = rclcpp::Time(odom_msg->header.stamp, get_clock()->get_clock_type());
  has_latest_odom_stamp_ = true;
  current_robot_base_angle_ = yaw;

  if (use_latest_odom_for_cmd_) {
    return;
  }

  geometry_msgs::msg::Twist::SharedPtr current_cmd_vel;
  {
    if (!latest_cmd_vel_) {
      return;
    }
    current_cmd_vel = latest_cmd_vel_;
  }

  geometry_msgs::msg::Twist aft_tf_vel =
    transformVelocity(current_cmd_vel, current_robot_base_angle_);

  publishGuardedVelocity(aft_tf_vel, now);
}

void FakeVelTransform::publishTransform()
{
  std::lock_guard<std::mutex> lock(cmd_vel_mutex_);

  const auto now = this->now();
  if (use_latest_odom_for_cmd_ && !odometryFresh(now)) {
    if (motion_command_active_) {
      publishStop();
      motion_command_active_ = false;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Stopping because no valid odometry newer than %.3f s is available", odom_timeout_);
    }
    return;
  }

  if (terrain_guard_enabled_ && !terrainMapFresh(now)) {
    if (motion_command_active_) {
      publishStop();
      motion_command_active_ = false;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Stopping because no terrain map newer than %.3f s is available", terrain_timeout_);
    }
    return;
  }

  if (!has_latest_odom_stamp_) {
    return;
  }

  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = latest_odom_stamp_;
  t.header.frame_id = robot_base_frame_;
  t.child_frame_id = fake_robot_base_frame_;
  tf2::Quaternion q;
  q.setRPY(0, 0, -current_robot_base_angle_);
  t.transform.rotation = tf2::toMsg(q);
  tf_broadcaster_->sendTransform(t);
}

bool FakeVelTransform::validOdometry(
  const nav_msgs::msg::Odometry & odometry, const rclcpp::Time & now, double & yaw,
  const char *& reason) const
{
  if (!odom_validation::finiteOdometry(odometry)) {
    reason = "pose, orientation, or twist is invalid/non-finite";
    return false;
  }
  if (!odom_validation::freshStamp(
        odometry.header.stamp, now.nanoseconds(), odom_timeout_, odom_future_tolerance_)) {
    reason = "header stamp is zero, invalid, stale, or too far in the future";
    return false;
  }

  const auto & orientation = odometry.pose.pose.orientation;
  tf2::Quaternion quaternion(orientation.x, orientation.y, orientation.z, orientation.w);
  quaternion.normalize();
  yaw = tf2::getYaw(quaternion);
  if (!std::isfinite(yaw)) {
    reason = "orientation does not produce a finite yaw";
    return false;
  }

  reason = "NONE";
  return true;
}

bool FakeVelTransform::odometryFresh(const rclcpp::Time & now) const
{
  if (!has_latest_odom_stamp_) {
    return false;
  }

  builtin_interfaces::msg::Time stamp = latest_odom_stamp_;
  return odom_validation::freshStamp(
    stamp, now.nanoseconds(), odom_timeout_, odom_future_tolerance_);
}

bool FakeVelTransform::terrainMapFresh(const rclcpp::Time & now) const
{
  if (!terrain_guard_enabled_) {
    return true;
  }
  if (!has_latest_terrain_) {
    return false;
  }
  const double age = (now - latest_terrain_receive_time_).seconds();
  return std::isfinite(age) && age >= 0.0 && age <= terrain_timeout_;
}

bool FakeVelTransform::publishGuardedVelocity(
  const geometry_msgs::msg::Twist & command, const rclcpp::Time & now)
{
  const bool is_motion =
    std::abs(command.linear.x) >= EPSILON || std::abs(command.linear.y) >= EPSILON ||
    std::abs(command.linear.z) >= EPSILON || std::abs(command.angular.x) >= EPSILON ||
    std::abs(command.angular.y) >= EPSILON || std::abs(command.angular.z) >= EPSILON;

  if (is_motion && !terrainMapFresh(now)) {
    publishStop();
    motion_command_active_ = false;
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Rejecting velocity command because no terrain map newer than %.3f s is available",
      terrain_timeout_);
    return false;
  }

  cmd_vel_chassis_pub_->publish(command);
  motion_command_active_ = is_motion;
  return true;
}

void FakeVelTransform::publishStop() { cmd_vel_chassis_pub_->publish(geometry_msgs::msg::Twist{}); }

geometry_msgs::msg::Twist FakeVelTransform::transformVelocity(
  const geometry_msgs::msg::Twist::SharedPtr & twist, double yaw_diff)
{
  geometry_msgs::msg::Twist aft_tf_vel;
  aft_tf_vel.angular.z = twist->angular.z + spin_speed_;
  aft_tf_vel.linear.x = twist->linear.x * cos(yaw_diff) + twist->linear.y * sin(yaw_diff);
  aft_tf_vel.linear.y = -twist->linear.x * sin(yaw_diff) + twist->linear.y * cos(yaw_diff);
  return aft_tf_vel;
}

}  // namespace fake_vel_transform

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(fake_vel_transform::FakeVelTransform)
