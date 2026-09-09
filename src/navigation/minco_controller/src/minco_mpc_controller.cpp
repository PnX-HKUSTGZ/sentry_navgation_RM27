#include "minco_controller/minco_mpc_controller.hpp"
#include "color_text.hpp"
#include "log.hpp"
#include "minco_controller/input_validation.hpp"
#include "minco_controller/reference_progress.hpp"
#include "minco_controller/tilt_speed_limit.hpp"
#include <iostream>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>

#include <Eigen/Geometry>

#ifdef MINCO_DEBUG
#include <chrono>
#include <iostream>
#endif

#include "nav2_util/node_utils.hpp"

#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "pluginlib/class_list_macros.hpp"

namespace custom_log {
void log_info_line(std::string_view text) { std::cout << text; }
} // namespace custom_log

namespace minco_controller {

namespace {

uint64_t sessionToken(const builtin_interfaces::msg::Time &stamp) {
  constexpr uint32_t kNanosecondsPerSecond = 1000000000U;
  if (stamp.sec < 0 || stamp.nanosec >= kNanosecondsPerSecond) {
    return 0U;
  }
  return static_cast<uint64_t>(stamp.sec) * kNanosecondsPerSecond +
         static_cast<uint64_t>(stamp.nanosec);
}

} // namespace

MincoMpcController::~MincoMpcController() { mpc_perf_monitor_.close(); }

double MincoMpcController::normalizeYaw(double yaw) {
  return std::atan2(std::sin(yaw), std::cos(yaw));
}

void MincoMpcController::resetSolverState() {
  std::lock_guard<std::mutex> lock(solver_mtx_);
  if (solver_) {
    solver_->reset();
  }
  prev_u_global_.setZero();
  integral_err_.setZero();
}

geometry_msgs::msg::TwistStamped
MincoMpcController::failClosedCommand(const rclcpp::Time &stamp,
                                      const std::string &reason) {
  geometry_msgs::msg::TwistStamped command;
  command.header.stamp = stamp;
  command.header.frame_id = global_frame_;

  {
    std::lock_guard<std::mutex> lock(data_mtx_);
    has_tracked_ref_ = false;
  }
  resetSolverState();

  if (cmd_vel_mpc_pub_) {
    geometry_msgs::msg::Twist zero;
    cmd_vel_mpc_pub_->publish(zero);
  }

  if (auto node = node_.lock()) {
    RCLCPP_WARN_THROTTLE(logger_, *node->get_clock(), 1000,
                         "Minco MPC fail-closed: %s", reason.c_str());
  }
  return command;
}

bool MincoMpcController::snapshotFreshInputs(
    const rclcpp::Time &now,
    const std::chrono::steady_clock::time_point &steady_now,
    nav_msgs::msg::Odometry::SharedPtr &odom, uint64_t &trajectory_generation,
    std::string &reason) const {
  std::lock_guard<std::mutex> lock(data_mtx_);
  if (!trajectory_session_gate_.active()) {
    reason = "INACTIVE";
    return false;
  }
  if (blocked_) {
    reason = "BLOCKED";
    return false;
  }
  if (!latest_opt_path_ || !has_opt_path_rx_time_) {
    reason = "NO_TRAJECTORY";
    return false;
  }
  if (!latest_odom_ || !has_odom_rx_time_) {
    reason = "NO_ODOMETRY";
    return false;
  }

  const double trajectory_rx_age =
      std::chrono::duration<double>(steady_now - last_opt_path_rx_time_)
          .count();
  const double odom_rx_age =
      std::chrono::duration<double>(steady_now - last_odom_rx_time_).count();
  if (!std::isfinite(trajectory_rx_age) || trajectory_rx_age < 0.0 ||
      trajectory_rx_age > trajectory_timeout_) {
    reason = "TRAJECTORY_RX_STALE";
    return false;
  }
  if (!std::isfinite(odom_rx_age) || odom_rx_age < 0.0 ||
      odom_rx_age > odom_timeout_) {
    reason = "ODOMETRY_RX_STALE";
    return false;
  }

  std::string validation_reason;
  if (!input_validation::validNormalCommand(*latest_opt_path_,
                                            &validation_reason)) {
    reason = "INVALID_TRAJECTORY_" + validation_reason;
    return false;
  }
  if (!input_validation::finiteOdometry(*latest_odom_)) {
    reason = "NONFINITE_ODOMETRY";
    return false;
  }

  const double now_seconds = now.seconds();
  if (!input_validation::freshStamp(
          input_validation::stampSeconds(latest_opt_path_->header.stamp),
          now_seconds, trajectory_timeout_, future_stamp_tolerance_)) {
    reason = "TRAJECTORY_STAMP_STALE";
    return false;
  }
  if (!input_validation::freshStamp(
          input_validation::stampSeconds(latest_odom_->header.stamp),
          now_seconds, odom_timeout_, future_stamp_tolerance_)) {
    reason = "ODOMETRY_STAMP_STALE";
    return false;
  }

  odom = latest_odom_;
  trajectory_generation = trajectory_generation_;
  reason = "NONE";
  return true;
}

void MincoMpcController::configureMpcPerfLogging(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr &node) {
  const std::string default_mpc_csv_path = "/tmp/mpc_perf_detailed.csv";

  nav2_util::declare_parameter_if_not_declared(
      node, name_ + ".performance.enable", rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(
      node, name_ + ".performance.print_enable", rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(
      node, name_ + ".performance.detailed_csv_enable",
      rclcpp::ParameterValue(false));
  nav2_util::declare_parameter_if_not_declared(
      node, name_ + ".performance.odom_sub_debug_enable",
      rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(
      node, name_ + ".performance.print_period_sec",
      rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(
      node, name_ + ".performance.csv_flush_every_n",
      rclcpp::ParameterValue(30));
  nav2_util::declare_parameter_if_not_declared(
      node, name_ + ".performance.mpc_csv_path",
      rclcpp::ParameterValue(default_mpc_csv_path));
  nav2_util::declare_parameter_if_not_declared(
      node, name_ + ".performance.run_id", rclcpp::ParameterValue(""));
  nav2_util::declare_parameter_if_not_declared(
      node, name_ + ".performance.scenario", rclcpp::ParameterValue(""));
  nav2_util::declare_parameter_if_not_declared(
      node, name_ + ".performance.variant", rclcpp::ParameterValue(""));

  bool performance_enable = true;
  bool print_enable = true;
  bool detailed_csv_enable = false;
  bool odom_sub_debug_enable = true;
  double print_period_sec = 1.0;
  int csv_flush_every_n = 30;
  std::string mpc_csv_path = default_mpc_csv_path;
  std::string run_id;
  std::string scenario;
  std::string variant;
  node->get_parameter(name_ + ".performance.enable", performance_enable);
  node->get_parameter(name_ + ".performance.print_enable", print_enable);
  node->get_parameter(name_ + ".performance.detailed_csv_enable",
                      detailed_csv_enable);
  node->get_parameter(name_ + ".performance.odom_sub_debug_enable",
                      odom_sub_debug_enable);
  node->get_parameter(name_ + ".performance.print_period_sec",
                      print_period_sec);
  node->get_parameter(name_ + ".performance.csv_flush_every_n",
                      csv_flush_every_n);
  node->get_parameter(name_ + ".performance.mpc_csv_path", mpc_csv_path);
  node->get_parameter(name_ + ".performance.run_id", run_id);
  node->get_parameter(name_ + ".performance.scenario", scenario);
  node->get_parameter(name_ + ".performance.variant", variant);

  MpcPerformanceConfig perf_cfg;
  perf_cfg.enable = performance_enable;
  perf_cfg.print_enable = print_enable;
  perf_cfg.detailed_csv_enable = detailed_csv_enable;
  perf_cfg.odom_sub_debug_enable = odom_sub_debug_enable;
  perf_cfg.detailed_csv_path = mpc_csv_path;
  perf_cfg.run_id = run_id;
  perf_cfg.scenario = scenario;
  perf_cfg.variant = variant;
  perf_cfg.print_period_sec = print_period_sec;
  perf_cfg.csv_flush_every_n = csv_flush_every_n;

  mpc_perf_monitor_.configure(perf_cfg, logger_);
}

void MincoMpcController::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) {
  node_ = parent;
  name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;

  auto node = parent.lock();
  if (!node) {
    throw std::invalid_argument(
        "MincoMpcController requires a live lifecycle node");
  }
  logger_ = node->get_logger();

  global_frame_ = costmap_ros_->getGlobalFrameID();
  base_frame_ = costmap_ros_->getBaseFrameID();
  configureMpcPerfLogging(node);

  // 声明/读取参数
  nav2_util::declare_parameter_if_not_declared(node, name + ".dt",
                                               rclcpp::ParameterValue(0.05));
  nav2_util::declare_parameter_if_not_declared(node, name + ".lookahead_time",
                                               rclcpp::ParameterValue(0.5));

  nav2_util::declare_parameter_if_not_declared(
      node, name + ".Q",
      rclcpp::ParameterValue(std::vector<double>{5.0, 5.0, 2.0}));
  nav2_util::declare_parameter_if_not_declared(node, name + ".q_along",
                                               rclcpp::ParameterValue(3.0));
  nav2_util::declare_parameter_if_not_declared(node, name + ".q_cross",
                                               rclcpp::ParameterValue(12.0));
  nav2_util::declare_parameter_if_not_declared(
      node, name + ".R",
      rclcpp::ParameterValue(std::vector<double>{1.0, 1.0, 0.5}));

  nav2_util::declare_parameter_if_not_declared(node, name + ".vx_min",
                                               rclcpp::ParameterValue(-1.0));
  nav2_util::declare_parameter_if_not_declared(node, name + ".vx_max",
                                               rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(node, name + ".vy_min",
                                               rclcpp::ParameterValue(-1.0));
  nav2_util::declare_parameter_if_not_declared(node, name + ".vy_max",
                                               rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(
      node, name + ".max_planar_speed", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(node, name + ".omega_min",
                                               rclcpp::ParameterValue(-2.0));
  nav2_util::declare_parameter_if_not_declared(node, name + ".omega_max",
                                               rclcpp::ParameterValue(2.0));
  nav2_util::declare_parameter_if_not_declared(node, name + ".fixed_wz",
                                               rclcpp::ParameterValue(0.0));
  nav2_util::declare_parameter_if_not_declared(node, name + ".odom_timeout",
                                               rclcpp::ParameterValue(0.25));
  nav2_util::declare_parameter_if_not_declared(
      node, name + ".trajectory_timeout", rclcpp::ParameterValue(1.5));
  nav2_util::declare_parameter_if_not_declared(
      node, name + ".future_stamp_tolerance", rclcpp::ParameterValue(0.05));
  nav2_util::declare_parameter_if_not_declared(
      node, name + ".deadzone_speed_threshold", rclcpp::ParameterValue(0.02));
  nav2_util::declare_parameter_if_not_declared(
      node, name + ".control_delay_compensation", rclcpp::ParameterValue(0.25));
  nav2_util::declare_parameter_if_not_declared(
      node, name + ".reference_progress_max_lead_time",
      rclcpp::ParameterValue(0.25));
  nav2_util::declare_parameter_if_not_declared(
      node, name + ".slope_slowdown_start_angle",
      rclcpp::ParameterValue(0.08));
  nav2_util::declare_parameter_if_not_declared(
      node, name + ".slope_full_slowdown_angle",
      rclcpp::ParameterValue(0.18));
  nav2_util::declare_parameter_if_not_declared(
      node, name + ".slope_speed_limit", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(
      node, name + ".use_small_gyro_mode", rclcpp::ParameterValue(true));

  nav2_util::declare_parameter_if_not_declared(
      node, name + ".use_acc_constraints", rclcpp::ParameterValue(false));
  nav2_util::declare_parameter_if_not_declared(node, name + ".ax_min",
                                               rclcpp::ParameterValue(-2.0));
  nav2_util::declare_parameter_if_not_declared(node, name + ".ax_max",
                                               rclcpp::ParameterValue(2.0));
  nav2_util::declare_parameter_if_not_declared(node, name + ".ay_min",
                                               rclcpp::ParameterValue(-2.0));
  nav2_util::declare_parameter_if_not_declared(node, name + ".ay_max",
                                               rclcpp::ParameterValue(2.0));
  nav2_util::declare_parameter_if_not_declared(node, name + ".alpha_min",
                                               rclcpp::ParameterValue(-4.0));
  nav2_util::declare_parameter_if_not_declared(node, name + ".alpha_max",
                                               rclcpp::ParameterValue(4.0));

  nav2_util::declare_parameter_if_not_declared(
      node, name + ".odom_frame", rclcpp::ParameterValue("camera_init"));
  nav2_util::declare_parameter_if_not_declared(node, name + ".map_frame",
                                               rclcpp::ParameterValue("map"));
  nav2_util::declare_parameter_if_not_declared(node, name + ".lidar_offset_x",
                                               rclcpp::ParameterValue(0.0));
  nav2_util::declare_parameter_if_not_declared(node, name + ".lidar_offset_y",
                                               rclcpp::ParameterValue(0.0));
  nav2_util::declare_parameter_if_not_declared(
      node, name + ".lidar_roll_offset", rclcpp::ParameterValue(0.0));
  nav2_util::declare_parameter_if_not_declared(
      node, name + ".opt_path_topic", rclcpp::ParameterValue(opt_path_topic_));
  nav2_util::declare_parameter_if_not_declared(
      node, name + ".odom_topic", rclcpp::ParameterValue(odom_topic_));
  nav2_util::declare_parameter_if_not_declared(
      node, name + ".cmd_vel_mpc_topic",
      rclcpp::ParameterValue(cmd_vel_mpc_topic_));

  double dt = 0.05;
  double lookahead_time = 0.5;
  node->get_parameter(name + ".dt", dt);
  node->get_parameter(name + ".lookahead_time", lookahead_time);
  if (!std::isfinite(dt) || dt <= 0.0 || !std::isfinite(lookahead_time) ||
      lookahead_time <= 0.0) {
    throw std::invalid_argument(
        "MincoMpcController dt and lookahead_time must be finite and positive");
  }

  std::vector<double> Qv;
  std::vector<double> Rv;
  node->get_parameter(name + ".Q", Qv);
  node->get_parameter(name + ".R", Rv);

  mpc_config_.dt = dt;
  mpc_config_.lookahead_time = lookahead_time;
  mpc_config_.horizon = static_cast<int>(std::ceil(lookahead_time / dt));
  if (Qv.size() == 3) {
    mpc_config_.Q = Eigen::Vector3d(Qv[0], Qv[1], Qv[2]);
  }
  node->get_parameter(name + ".q_along", mpc_config_.q_along);
  node->get_parameter(name + ".q_cross", mpc_config_.q_cross);
  if (Rv.size() == 3) {
    mpc_config_.R = Eigen::Vector3d(Rv[0], Rv[1], Rv[2]);
  }

  if (Qv.size() != 3 || Rv.size() != 3 || !mpc_config_.Q.allFinite() ||
      !mpc_config_.R.allFinite() || (mpc_config_.Q.array() < 0.0).any() ||
      (mpc_config_.R.array() <= 0.0).any() ||
      !std::isfinite(mpc_config_.q_along) ||
      !std::isfinite(mpc_config_.q_cross) || mpc_config_.q_along < 0.0 ||
      mpc_config_.q_cross < 0.0) {
    throw std::invalid_argument(
        "MincoMpcController requires Q/q_along/q_cross >= 0 and R > 0");
  }

  node->get_parameter(name + ".vx_min", mpc_config_.vx_min);
  node->get_parameter(name + ".vx_max", mpc_config_.vx_max);
  node->get_parameter(name + ".vy_min", mpc_config_.vy_min);
  node->get_parameter(name + ".vy_max", mpc_config_.vy_max);
  node->get_parameter(name + ".max_planar_speed",
                      mpc_config_.max_planar_speed);
  node->get_parameter(name + ".omega_min", mpc_config_.omega_min);
  node->get_parameter(name + ".omega_max", mpc_config_.omega_max);
  node->get_parameter(name + ".fixed_wz", fixed_wz_);
  node->get_parameter(name + ".odom_timeout", odom_timeout_);
  node->get_parameter(name + ".trajectory_timeout", trajectory_timeout_);
  node->get_parameter(name + ".future_stamp_tolerance",
                      future_stamp_tolerance_);
  node->get_parameter(name + ".deadzone_speed_threshold",
                      deadzone_speed_threshold_);
  node->get_parameter(name + ".control_delay_compensation",
                      control_delay_compensation_);
  node->get_parameter(name + ".reference_progress_max_lead_time",
                      reference_progress_max_lead_time_);
  node->get_parameter(name + ".slope_slowdown_start_angle",
                      slope_slowdown_start_angle_);
  node->get_parameter(name + ".slope_full_slowdown_angle",
                      slope_full_slowdown_angle_);
  node->get_parameter(name + ".slope_speed_limit", slope_speed_limit_);
  node->get_parameter(name + ".use_small_gyro_mode", use_small_gyro_mode_);

  node->get_parameter(name + ".use_acc_constraints",
                      mpc_config_.use_acc_constraints);
  node->get_parameter(name + ".ax_min", mpc_config_.ax_min);
  node->get_parameter(name + ".ax_max", mpc_config_.ax_max);
  node->get_parameter(name + ".ay_min", mpc_config_.ay_min);
  node->get_parameter(name + ".ay_max", mpc_config_.ay_max);
  node->get_parameter(name + ".alpha_min", mpc_config_.alpha_min);
  node->get_parameter(name + ".alpha_max", mpc_config_.alpha_max);

  node->get_parameter(name + ".odom_frame", odom_frame_);
  node->get_parameter(name + ".map_frame", map_frame_);
  node->get_parameter(name + ".lidar_offset_x", lidar_offset_x_);
  node->get_parameter(name + ".lidar_offset_y", lidar_offset_y_);
  node->get_parameter(name + ".lidar_roll_offset", lidar_roll_offset_);
  node->get_parameter(name + ".opt_path_topic", opt_path_topic_);
  node->get_parameter(name + ".odom_topic", odom_topic_);
  node->get_parameter(name + ".cmd_vel_mpc_topic", cmd_vel_mpc_topic_);

  if (!std::isfinite(odom_timeout_) || odom_timeout_ <= 0.0 ||
      !std::isfinite(trajectory_timeout_) || trajectory_timeout_ <= 0.0 ||
      !std::isfinite(future_stamp_tolerance_) ||
      future_stamp_tolerance_ < 0.0 ||
      !std::isfinite(reference_progress_max_lead_time_) ||
      reference_progress_max_lead_time_ < 0.0 ||
      !std::isfinite(slope_slowdown_start_angle_) ||
      slope_slowdown_start_angle_ < 0.0 ||
      !std::isfinite(slope_full_slowdown_angle_) ||
      slope_full_slowdown_angle_ <= slope_slowdown_start_angle_ ||
      !std::isfinite(slope_speed_limit_) || slope_speed_limit_ <= 0.0 ||
      !std::isfinite(mpc_config_.max_planar_speed) ||
      mpc_config_.max_planar_speed <= 0.0) {
    throw std::invalid_argument("MincoMpcController timeouts must be finite "
                                "(timeouts > 0, future tolerance/reference "
                                "lead/slope start >= 0), slope full angle must "
                                "exceed its start angle, and speed limits must "
                                "be finite and positive");
  }
  slope_speed_limit_ =
      std::min(slope_speed_limit_, mpc_config_.max_planar_speed);

  solver_ = std::make_unique<MpcSolver>(mpc_config_);

  // 订阅优化轨迹
  opt_path_sub_ =
      node->create_subscription<ros_interfaces::msg::MpcPositionCommand>(
          opt_path_topic_, rclcpp::SystemDefaultsQoS(),
          std::bind(&MincoMpcController::onOptPath, this,
                    std::placeholders::_1));

  // 订阅里程计：用于延迟补偿和mpc输入状态
  auto odom_qos =
      rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
  odom_sub_ = node->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, odom_qos,
      std::bind(&MincoMpcController::onOdom, this, std::placeholders::_1));

  mpc_predict_path_pub_ =
      node->create_publisher<nav_msgs::msg::Path>("mpc_predict_path", 1);
  mpc_real_path_pub_ =
      node->create_publisher<nav_msgs::msg::Path>("mpc_real_path", 1);
  cmd_vel_mpc_pub_ =
      node->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_mpc_topic_, 1);
  real_path_history_.clear();
  last_real_path_pub_time_ = node->now();
  {
    std::lock_guard<std::mutex> lock(data_mtx_);
    latest_opt_path_.reset();
    pending_opt_path_.reset();
    latest_odom_.reset();
    has_opt_path_rx_time_ = false;
    has_pending_opt_path_rx_time_ = false;
    has_odom_rx_time_ = false;
    has_tracked_ref_ = false;
    blocked_ = true;
    trajectory_session_gate_.configure();
    ++trajectory_generation_;
  }

  RCLCPP_INFO(logger_,
              "%s: MincoMpcController configured (dt=%.3f, "
              "lookahead_time=%.3f, deadzone=%.3f, delay_comp=%.3f, "
              "reference_max_lead=%.3f, slope_slowdown=[%.3f,%.3f]rad, "
              "slope_speed_limit=%.3f, "
              "small_gyro=%s, fixed_wz=%.3f, odom_timeout=%.3f, "
              "trajectory_timeout=%.3f, future_tolerance=%.3f, "
              "q_along=%.3f, q_cross=%.3f, lidar_offset_x=%.3f, "
              "lidar_offset_y=%.3f, lidar_roll_offset=%.3f, "
              "opt_path_topic=%s, odom_topic=%s, cmd_vel_mpc_topic=%s)",
              name_.c_str(), dt, lookahead_time, deadzone_speed_threshold_,
              control_delay_compensation_, reference_progress_max_lead_time_,
              slope_slowdown_start_angle_, slope_full_slowdown_angle_,
              slope_speed_limit_,
              use_small_gyro_mode_ ? "true" : "false", fixed_wz_, odom_timeout_,
              trajectory_timeout_, future_stamp_tolerance_, mpc_config_.q_along,
              mpc_config_.q_cross, lidar_offset_x_, lidar_offset_y_,
              lidar_roll_offset_, opt_path_topic_.c_str(), odom_topic_.c_str(),
              cmd_vel_mpc_topic_.c_str());
}

void MincoMpcController::extractGlobalVelocityAndYaw(
    const nav_msgs::msg::Odometry::SharedPtr &odom, double &vx_global,
    double &vy_global, double &omega_global, double &yaw_global) const {
  // 1. 提取 3D 体轴速度 (IMU 物理中心的局部速度)
  Eigen::Vector3d v_body_imu(odom->twist.twist.linear.x,
                             odom->twist.twist.linear.y,
                             odom->twist.twist.linear.z);

  // 2. 提取全量 3D 姿态四元数
  Eigen::Quaterniond q(
      odom->pose.pose.orientation.w, odom->pose.pose.orientation.x,
      odom->pose.pose.orientation.y, odom->pose.pose.orientation.z);

  // 3. 3D 旋转：将倾斜机体系下的速度无损映射回与地面平行的全局系
  Eigen::Vector3d v_imu_global = q * v_body_imu;

  // Odometry twist is expressed in child_frame_id. Rotate angular velocity
  // as well so omega is the yaw rate around the odometry frame's Z axis.
  const Eigen::Vector3d omega_body(odom->twist.twist.angular.x,
                                   odom->twist.twist.angular.y,
                                   odom->twist.twist.angular.z);
  omega_global = (q * omega_body).z();
  yaw_global = tf2::getYaw(odom->pose.pose.orientation);

  // 5. 计算全局杆臂补偿 (将底盘水平安装偏置旋转到全局系)
  double offset_global_x = std::cos(yaw_global) * lidar_offset_x_ -
                           std::sin(yaw_global) * lidar_offset_y_;
  double offset_global_y = std::sin(yaw_global) * lidar_offset_x_ +
                           std::cos(yaw_global) * lidar_offset_y_;
  // 6. 计算底盘旋转中心的全局速度
  vx_global = v_imu_global.x() + omega_global * offset_global_y;
  vy_global = v_imu_global.y() - omega_global * offset_global_x;
}

void MincoMpcController::cleanup() {
  mpc_perf_monitor_.close();
  {
    std::scoped_lock lock(data_mtx_, plan_mtx_);
    trajectory_session_gate_.deactivate();
    blocked_ = true;
    ++trajectory_generation_;
    latest_opt_path_.reset();
    pending_opt_path_.reset();
    latest_odom_.reset();
    has_opt_path_rx_time_ = false;
    has_pending_opt_path_rx_time_ = false;
    has_odom_rx_time_ = false;
    has_tracked_ref_ = false;
    global_plan_ = nav_msgs::msg::Path{};
  }
  if (auto node = node_.lock()) {
    (void)failClosedCommand(node->now(), "CLEANUP");
  } else {
    resetSolverState();
  }

  opt_path_sub_.reset();
  odom_sub_.reset();
  mpc_predict_path_pub_.reset();
  mpc_real_path_pub_.reset();
  cmd_vel_mpc_pub_.reset();
  {
    std::lock_guard<std::mutex> lock(solver_mtx_);
    solver_.reset();
  }
}

void MincoMpcController::activate() {
  auto node = node_.lock();
  const rclcpp::Time now = node ? node->now() : rclcpp::Time(0, 0, RCL_ROS_TIME);
  {
    std::scoped_lock lock(data_mtx_, plan_mtx_);
    global_plan_ = nav_msgs::msg::Path{};
    trajectory_session_gate_.activate();
    blocked_ = true;
    ++trajectory_generation_;
    latest_opt_path_.reset();
    pending_opt_path_.reset();
    has_opt_path_rx_time_ = false;
    has_pending_opt_path_rx_time_ = false;
    has_tracked_ref_ = false;
  }
  (void)failClosedCommand(now, "ACTIVATED_WAITING_FOR_PLAN");
}

void MincoMpcController::deactivate() {
  {
    std::scoped_lock lock(data_mtx_, plan_mtx_);
    trajectory_session_gate_.deactivate();
    blocked_ = true;
    ++trajectory_generation_;
    latest_opt_path_.reset();
    pending_opt_path_.reset();
    has_opt_path_rx_time_ = false;
    has_pending_opt_path_rx_time_ = false;
    has_tracked_ref_ = false;
    global_plan_ = nav_msgs::msg::Path{};
  }
  if (auto node = node_.lock()) {
    (void)failClosedCommand(node->now(), "DEACTIVATED");
  } else {
    resetSolverState();
  }
}

void MincoMpcController::setPlan(const nav_msgs::msg::Path &path) {
  auto node = node_.lock();
  const rclcpp::Time now = node ? node->now() : rclcpp::Time(0, 0, RCL_ROS_TIME);
  const uint64_t plan_token = sessionToken(path.header.stamp);
  const auto steady_now = std::chrono::steady_clock::now();
  bool promoted_pending = false;
  bool refreshed = false;
  bool rejected = false;
  uint64_t expected_token = 0U;
  {
    std::scoped_lock lock(data_mtx_, plan_mtx_);
    const auto update =
        trajectory_session_gate_.setPlan(plan_token, !path.poses.empty());
    if (update == TrajectorySessionGate::PlanUpdate::REJECTED) {
      rejected = true;
    } else {
      global_plan_ = path;
    }

    if (update == TrajectorySessionGate::PlanUpdate::REFRESHED) {
      refreshed = true;
    } else if (update == TrajectorySessionGate::PlanUpdate::AUTHORIZED) {
      blocked_ = true;
      ++trajectory_generation_;
      latest_opt_path_.reset();
      has_opt_path_rx_time_ = false;
      has_tracked_ref_ = false;

      if (pending_opt_path_ && has_pending_opt_path_rx_time_ && node &&
          trajectory_session_gate_.classifyNormal(sessionToken(
              pending_opt_path_->planning_stamp)) ==
              TrajectorySessionGate::NormalDisposition::ACCEPT) {
        const double receive_age = std::chrono::duration<double>(
            steady_now - pending_opt_path_rx_time_).count();
        const bool receive_fresh = std::isfinite(receive_age) &&
                                   receive_age >= 0.0 &&
                                   receive_age <= trajectory_timeout_;
        const bool stamp_fresh = input_validation::freshStamp(
            input_validation::stampSeconds(pending_opt_path_->header.stamp),
            now.seconds(), trajectory_timeout_, future_stamp_tolerance_);
        if (receive_fresh && stamp_fresh) {
          latest_opt_path_ = pending_opt_path_;
          last_opt_path_rx_time_ = pending_opt_path_rx_time_;
          has_opt_path_rx_time_ = true;
          blocked_ = false;
          ++trajectory_generation_;
          promoted_pending = true;
        }
      }
      pending_opt_path_.reset();
      has_pending_opt_path_rx_time_ = false;
    } else if (update == TrajectorySessionGate::PlanUpdate::CLEARED) {
      blocked_ = true;
      ++trajectory_generation_;
      latest_opt_path_.reset();
      pending_opt_path_.reset();
      has_opt_path_rx_time_ = false;
      has_pending_opt_path_rx_time_ = false;
      has_tracked_ref_ = false;
    }
    expected_token = trajectory_session_gate_.expectedToken();
  }

  if (rejected) {
    if (node) {
      RCLCPP_WARN_THROTTLE(
          logger_, *node->get_clock(), 1000,
          "Minco MPC ignored stale/invalid Nav2 plan token=%llu expected=%llu",
          static_cast<unsigned long long>(plan_token),
          static_cast<unsigned long long>(expected_token));
    }
    return;
  }
  if (refreshed || promoted_pending) {
    return;
  }
  (void)failClosedCommand(
      now, path.poses.empty() ? "EMPTY_PLAN" : "NEW_PLAN_WAITING_FOR_TRAJECTORY");
}

void MincoMpcController::setSpeedLimit(const double &speed_limit,
                                       const bool &percentage) {
  speed_limit_ = speed_limit;
  speed_limit_percentage_ = percentage;
}

void MincoMpcController::onOptPath(
    const ros_interfaces::msg::MpcPositionCommand::SharedPtr msg) {
  auto node = node_.lock();
  const uint64_t planning_token = msg ? sessionToken(msg->planning_stamp) : 0U;

  if (msg && msg->command_flag ==
                 ros_interfaces::msg::MpcPositionCommand::BLOCK_COMMAND) {
    bool block_accepted = false;
    {
      std::lock_guard<std::mutex> lock(data_mtx_);
      block_accepted = trajectory_session_gate_.announceBlock(planning_token);
      if (block_accepted || planning_token == 0U) {
        blocked_ = true;
        ++trajectory_generation_;
        latest_opt_path_.reset();
        pending_opt_path_.reset();
        has_opt_path_rx_time_ = false;
        has_pending_opt_path_rx_time_ = false;
        has_tracked_ref_ = false;
      }
    }
    if (block_accepted || planning_token == 0U) {
      const rclcpp::Time stamp =
          node ? node->now() : rclcpp::Time(0, 0, RCL_ROS_TIME);
      (void)failClosedCommand(
          stamp, planning_token == 0U ? "BLOCK_WITH_INVALID_SESSION"
                                     : "BLOCK_COMMAND");
    }
    return;
  }

  std::string validation_reason;
  bool valid_normal = false;
  if (!msg) {
    validation_reason = "NULL_TRAJECTORY";
  } else if (msg->command_flag !=
             ros_interfaces::msg::MpcPositionCommand::NORMAL_COMMAND) {
    validation_reason =
        "UNKNOWN_COMMAND_FLAG_" + std::to_string(msg->command_flag);
  } else if (!input_validation::validNormalCommand(*msg, &validation_reason)) {
    validation_reason = "INVALID_NORMAL_" + validation_reason;
  } else if (!node || !input_validation::freshStamp(
                          input_validation::stampSeconds(msg->header.stamp),
                          node->now().seconds(), trajectory_timeout_,
                          future_stamp_tolerance_)) {
    validation_reason = "INVALID_NORMAL_STAMP";
  } else if (planning_token == 0U) {
    validation_reason = "INVALID_PLANNING_SESSION";
  } else {
    valid_normal = true;
  }

  const auto receive_time = std::chrono::steady_clock::now();
  bool accepted = false;
  bool waiting_for_plan = false;
  bool ignored_stale_session = false;
  uint64_t expected_token = 0U;
  {
    std::lock_guard<std::mutex> lock(data_mtx_);
    if (valid_normal) {
      const auto disposition =
          trajectory_session_gate_.classifyNormal(planning_token);
      if (disposition ==
          TrajectorySessionGate::NormalDisposition::WAIT_FOR_PLAN) {
        pending_opt_path_ = msg;
        pending_opt_path_rx_time_ = receive_time;
        has_pending_opt_path_rx_time_ = true;
        blocked_ = true;
        waiting_for_plan = true;
        validation_reason = "NORMAL_WAITING_FOR_MATCHING_PLAN";
        valid_normal = false;
      } else if (disposition ==
                 TrajectorySessionGate::NormalDisposition::REJECT) {
        ignored_stale_session = true;
        validation_reason = "NORMAL_FOR_STALE_SESSION";
        valid_normal = false;
      }
    }

    if (valid_normal) {
      const uint32_t new_traj_id = msg->cmds.front().trajectory_id;
      const uint32_t old_traj_id =
          (latest_opt_path_ && !latest_opt_path_->cmds.empty())
              ? latest_opt_path_->cmds.front().trajectory_id
              : 0U;

      if (new_traj_id != old_traj_id) {
        has_tracked_ref_ = false;
      }

      latest_opt_path_ = msg;
      last_opt_path_rx_time_ = receive_time;
      has_opt_path_rx_time_ = true;
      pending_opt_path_.reset();
      has_pending_opt_path_rx_time_ = false;
      blocked_ = false;
      ++trajectory_generation_;
      accepted = true;
    } else if (!waiting_for_plan && !ignored_stale_session) {
      blocked_ = true;
      ++trajectory_generation_;
      latest_opt_path_.reset();
      pending_opt_path_.reset();
      has_opt_path_rx_time_ = false;
      has_pending_opt_path_rx_time_ = false;
      has_tracked_ref_ = false;
    }
    expected_token = trajectory_session_gate_.expectedToken();
  }

  if (ignored_stale_session) {
    if (node) {
      RCLCPP_WARN_THROTTLE(
          logger_, *node->get_clock(), 1000,
          "Minco MPC ignored trajectory for stale session token=%llu expected=%llu",
          static_cast<unsigned long long>(planning_token),
          static_cast<unsigned long long>(expected_token));
    }
    return;
  }
  if (!accepted) {
    const rclcpp::Time stamp =
        node ? node->now() : rclcpp::Time(0, 0, RCL_ROS_TIME);
    (void)failClosedCommand(stamp, validation_reason);
  }
}

void MincoMpcController::onOdom(const nav_msgs::msg::Odometry::SharedPtr msg) {
  const auto receive_time = std::chrono::steady_clock::now();
  auto node = node_.lock();
  if (node && msg) {
    mpc_perf_monitor_.recordOdomCallback(node->now(), msg->header.stamp);
  }

  std::lock_guard<std::mutex> lk(data_mtx_);
  latest_odom_ = msg;
  last_odom_rx_time_ = receive_time;
  has_odom_rx_time_ = static_cast<bool>(msg);
}

bool MincoMpcController::transformPathToOdom(
    const ros_interfaces::msg::MpcPositionCommand::SharedPtr &opt,
    std::vector<ros_interfaces::msg::PositionCommand> &out_cmds) const {
  std::string source_frame = opt->header.frame_id;
  if (source_frame.empty()) {
    source_frame = map_frame_;
    auto node_ptr = node_.lock();
    if (node_ptr) {
      RCLCPP_WARN_THROTTLE(
          logger_, *node_ptr->get_clock(), 10000,
          "Received opt_path with empty frame_id, defaulting to '%s'",
          source_frame.c_str());
    }
  }

  // 从costmap获取全局坐标系，优先使用global_frame_，如果未设置则退回到odom_frame_
  std::string target_frame = global_frame_;
  if (target_frame.empty()) {
    target_frame = odom_frame_;
  }

  if (source_frame == target_frame) {
    out_cmds = opt->cmds;
    return true;
  }

  // 如果坐标系不同，尝试进行转换
  try {
    geometry_msgs::msg::TransformStamped transform =
        tf_->lookupTransform(target_frame, source_frame, tf2::TimePointZero);

    out_cmds = opt->cmds;
    for (auto &cmd : out_cmds) {
      // Transform position
      geometry_msgs::msg::PointStamped p_in, p_out;
      p_in.header.frame_id = source_frame;
      p_in.point = cmd.position;
      tf2::doTransform(p_in, p_out, transform);
      cmd.position = p_out.point;

      // Transform velocity (vector)
      geometry_msgs::msg::Vector3Stamped v_in, v_out;
      v_in.header.frame_id = source_frame;
      v_in.vector = cmd.velocity;
      tf2::doTransform(v_in, v_out, transform);
      cmd.velocity = v_out.vector;

      // Transform acceleration (vector)
      geometry_msgs::msg::Vector3Stamped a_in, a_out;
      a_in.header.frame_id = source_frame;
      a_in.vector = cmd.acceleration;
      tf2::doTransform(a_in, a_out, transform);
      cmd.acceleration = a_out.vector;

      // Transform jerk (vector)
      geometry_msgs::msg::Vector3Stamped j_in, j_out;
      j_in.header.frame_id = source_frame;
      j_in.vector = cmd.jerk;
      tf2::doTransform(j_in, j_out, transform);
      cmd.jerk = j_out.vector;

      // Transform yaw
      double yaw_diff = tf2::getYaw(transform.transform.rotation);
      cmd.yaw = normalizeYaw(cmd.yaw + yaw_diff);
    }

  } catch (tf2::TransformException &ex) {
    return false;
  }
  return true;
}

bool MincoMpcController::buildReferenceFromOptPath(
    const State &curr, std::vector<ReferencePoint> &out_ref) const {
  auto node = node_.lock();
  if (!node) {
    return false;
  }
  const rclcpp::Time now = node->now();

  // 获取最新的优化轨迹和里程计数据，以及当前跟踪状态（索引、时间戳、轨迹ID）
  ros_interfaces::msg::MpcPositionCommand::SharedPtr opt;
  double tracked_ref_idx = 0.0;
  double tracked_spatial_idx = 0.0;
  rclcpp::Time tracked_ref_time;
  uint32_t tracked_opt_traj_id = 0u;
  bool has_tracked_ref = false;
  {
    std::lock_guard<std::mutex> lk(data_mtx_);
    opt = latest_opt_path_;
    tracked_ref_idx = tracked_ref_idx_;
    tracked_spatial_idx = tracked_spatial_idx_;
    tracked_ref_time = tracked_ref_time_;
    tracked_opt_traj_id = tracked_opt_traj_id_;
    has_tracked_ref = has_tracked_ref_;
  }

  if (!opt || opt->cmds.empty()) {
    return false;
  }

  // 坐标系转换处理
  std::vector<ros_interfaces::msg::PositionCommand> cmds;
  if (!transformPathToOdom(opt, cmds)) {
    return false;
  }

  const size_t n_cmds = cmds.size();
  const double planner_dt = 1.0 / mpc_config_.planner_freq;
  if (planner_dt <= 1.0e-6) {
    return false;
  }

  // 最近点搜索（基于最小欧式距离），并计算更精确的最近点投影索引
  size_t best_idx = 0;
  double best_d2 = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < n_cmds; ++i) {
    const double dx = cmds[i].position.x - curr.x;
    const double dy = cmds[i].position.y - curr.y;
    const double d2 = dx * dx + dy * dy;
    if (d2 < best_d2) {
      best_d2 = d2;
      best_idx = i;
    }
  }

  double nearest_idx_float = static_cast<double>(best_idx);

  if (best_idx < n_cmds - 1) {
    const auto &p_curr = cmds[best_idx];
    const auto &p_next = cmds[best_idx + 1];

    Eigen::Vector2d a_vec(p_next.position.x - p_curr.position.x,
                          p_next.position.y - p_curr.position.y);
    Eigen::Vector2d b_vec(curr.x - p_curr.position.x,
                          curr.y - p_curr.position.y);

    double len_sq = a_vec.squaredNorm();
    if (len_sq > 1e-6) {
      double projection = a_vec.dot(b_vec) / len_sq;
      if (projection > -0.5 && projection < 1.0) {
        nearest_idx_float += projection;
      }
    }
  }

  nearest_idx_float = std::max(0.0, nearest_idx_float);

  const uint32_t current_traj_id =
      (!opt->cmds.empty()) ? opt->cmds.front().trajectory_id : 0u;

  // A new trajectory starts from its spatial pickup. While tracking that same
  // trajectory, time progression escapes a zero-velocity t=0 sample, but is
  // capped relative to the current spatial pickup so a slow vehicle cannot be
  // left behind by a short stopping trajectory.
  const bool same_opt_traj =
      has_tracked_ref && tracked_opt_traj_id == current_traj_id;
  const double elapsed_since_update =
      same_opt_traj ? (now - tracked_ref_time).seconds() : 0.0;
  double current_idx_float = 0.0;
  double current_spatial_idx = 0.0;
  if (!computeReferenceIndex(
          nearest_idx_float, same_opt_traj, tracked_spatial_idx,
          tracked_ref_idx,
          elapsed_since_update, planner_dt,
          static_cast<double>(n_cmds - 1),
          reference_progress_max_lead_time_, current_spatial_idx,
          current_idx_float)) {
    return false;
  }

  {
    std::lock_guard<std::mutex> lk(data_mtx_);
    tracked_spatial_idx_ = current_spatial_idx;
    tracked_ref_idx_ = current_idx_float;
    tracked_ref_time_ = now;
    tracked_opt_traj_id_ = current_traj_id;
    has_tracked_ref_ = true;
  }

  double current_traj_time = current_idx_float * planner_dt;

  // 让 MPC 始终去追踪未来一段时间的参考点，用于控制延迟补偿
  double control_delay = control_delay_compensation_;

  current_traj_time += control_delay;

  const int N = mpc_config_.horizon;
  const double mpc_dt = mpc_config_.dt;
  out_ref.clear();
  out_ref.reserve(static_cast<size_t>(N));

  for (int k = 0; k < N; ++k) {
    double target_time = current_traj_time + k * mpc_dt;
    double target_idx_float = target_time / planner_dt;

    if (target_idx_float < 0.0)
      target_idx_float = 0.0;
    if (target_idx_float > static_cast<double>(n_cmds - 1)) {
      target_idx_float = static_cast<double>(n_cmds - 1);
    }

    size_t target_idx = static_cast<size_t>(std::floor(target_idx_float));
    size_t next_idx = target_idx + 1;
    double alpha = target_idx_float - static_cast<double>(target_idx);

    ReferencePoint rp;
    if (next_idx < n_cmds) {
      // 二次前馈插值（泰勒展开），使用 P/V/A/J
      // 进行插值，补偿控制延迟带来的误差。
      const double dt = std::max(0.0, std::min(planner_dt, alpha * planner_dt));
      const double dt2 = dt * dt;
      const double dt3 = dt2 * dt;

      const Eigen::Vector2d p_i(cmds[target_idx].position.x,
                                cmds[target_idx].position.y);
      const Eigen::Vector2d v_i(cmds[target_idx].velocity.x,
                                cmds[target_idx].velocity.y);
      const Eigen::Vector2d a_i(cmds[target_idx].acceleration.x,
                                cmds[target_idx].acceleration.y);
      const Eigen::Vector2d j_i(cmds[target_idx].jerk.x,
                                cmds[target_idx].jerk.y);

      rp.pos = p_i + v_i * dt + 0.5 * a_i * dt2 + (1.0 / 6.0) * j_i * dt3;
      rp.vel = v_i + a_i * dt + 0.5 * j_i * dt2;

      // 线性角度插值
      rp.yaw = interpolateYaw(cmds[target_idx].yaw, cmds[next_idx].yaw, alpha);
      rp.yaw_rate =
          interpolate(cmds[target_idx].yaw_dot, cmds[next_idx].yaw_dot, alpha);
    } else {
      const auto &p_end = cmds.back();
      rp.pos = Eigen::Vector2d(p_end.position.x, p_end.position.y);
      rp.vel = Eigen::Vector2d(0.0, 0.0);
      rp.yaw = p_end.yaw;
      rp.yaw_rate = 0.0;
    }
    out_ref.push_back(rp);
  }

  // 对参考序列的 yaw 进行连续性处理
  if (!out_ref.empty()) {
    double diff = out_ref[0].yaw - curr.yaw;
    // 将 diff 限制在 [-PI, PI]
    diff = std::atan2(std::sin(diff), std::cos(diff));
    out_ref[0].yaw = curr.yaw + diff;

    for (size_t i = 1; i < out_ref.size(); ++i) {
      double d_yaw = out_ref[i].yaw - out_ref[i - 1].yaw;
      d_yaw = std::atan2(std::sin(d_yaw), std::cos(d_yaw));
      out_ref[i].yaw = out_ref[i - 1].yaw + d_yaw;
    }
  }
  return true;
}

void MincoMpcController::applyGravityCompensation(
    const nav_msgs::msg::Odometry::SharedPtr &odom, double &vx, double &vy) {
  if (!odom) {
    return;
  }

  tf2::Quaternion q;
  tf2::fromMsg(odom->pose.pose.orientation, q);

  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  // std::cout << "Original roll: " << roll << ", pitch: " << pitch << ", yaw: "
  // << yaw << std::endl;
  const double true_roll = roll - lidar_roll_offset_;
  constexpr double angle_threshold = 0.05;
  constexpr double k_gravity_x = 1.0;
  constexpr double k_gravity_y = 20.0;

  double body_comp_x = 0.0;
  double body_comp_y = 0.0;
  if (std::abs(pitch) > angle_threshold) {
    body_comp_x = k_gravity_x * std::sin(-pitch);
  }
  if (std::abs(true_roll) > angle_threshold) {
    body_comp_y = k_gravity_y * std::sin(std::abs(true_roll));
  }

  if (body_comp_x == 0.0 && body_comp_y == 0.0) {
    return;
  }

  const double global_comp_x =
      body_comp_x * std::cos(yaw) - body_comp_y * std::sin(yaw);
  const double global_comp_y =
      body_comp_x * std::sin(yaw) + body_comp_y * std::cos(yaw);

  vx += global_comp_x;
  vy += global_comp_y;

  vx = std::clamp(vx, mpc_config_.vx_min, mpc_config_.vx_max);
  vy = std::clamp(vy, mpc_config_.vy_min, mpc_config_.vy_max);
}

geometry_msgs::msg::TwistStamped MincoMpcController::computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped &pose,
    const geometry_msgs::msg::Twist &velocity,
    nav2_core::GoalChecker *goal_checker) {
  const bool record_perf = mpc_perf_monitor_.detailedCsvEnabled();
  const auto cycle_start = record_perf
                               ? std::chrono::steady_clock::now()
                               : std::chrono::steady_clock::time_point{};
  auto node = node_.lock();
  if (!node) {
    return failClosedCommand(rclcpp::Time(0, 0, RCL_ROS_TIME),
                             "NODE_UNAVAILABLE");
  }

  const rclcpp::Time now = node->now();
  std::optional<MpcPerfSample> perf;
  if (record_perf) {
    perf.emplace();
    perf->stamp_ros = now.seconds();
    perf->stamp_steady_ns = MpcPerformanceMonitor::steadyNowNs();
  }
  auto finish = [&](geometry_msgs::msg::TwistStamped out_cmd, bool success,
                    const std::string &) {
    if (perf) {
      perf->success = success;
      perf->cycle_time_ms = std::chrono::duration<double, std::milli>(
                                std::chrono::steady_clock::now() - cycle_start)
                                .count();
      if (last_mpc_perf_stamp_ns_ > 0 &&
          perf->stamp_steady_ns > last_mpc_perf_stamp_ns_) {
        const double dt_sec = static_cast<double>(perf->stamp_steady_ns -
                                                  last_mpc_perf_stamp_ns_) *
                              1.0e-9;
        if (dt_sec > 1.0e-9) {
          perf->controller_hz = 1.0 / dt_sec;
        }
      }
      last_mpc_perf_stamp_ns_ = perf->stamp_steady_ns;
      perf->cmd_vx = out_cmd.twist.linear.x;
      perf->cmd_vy = out_cmd.twist.linear.y;
      perf->cmd_wz = out_cmd.twist.angular.z;
      mpc_perf_monitor_.recordMpcSample(*perf);
    }
    return out_cmd;
  };
  const auto stop = [&](const std::string &reason, bool success = false) {
    return finish(failClosedCommand(node->now(), reason), success, reason);
  };

  nav_msgs::msg::Odometry::SharedPtr latest_odom;
  uint64_t trajectory_generation = 0;
  std::string input_reason;
  if (!input_validation::finitePose(pose.pose)) {
    return stop("NONFINITE_POSE");
  }
  if (!snapshotFreshInputs(now, std::chrono::steady_clock::now(), latest_odom,
                           trajectory_generation, input_reason)) {
    return stop(input_reason);
  }

  // The Nav2 pose may use the yaw-cancelled gimbal_yaw_fake frame. Keep the
  // complete MPC state in the odometry/global frame instead of mixing that
  // synthetic yaw with odometry-frame trajectory references.
  if (latest_odom->header.frame_id.empty() ||
      latest_odom->header.frame_id != global_frame_) {
    return stop("ODOMETRY_FRAME_MISMATCH");
  }

  // Odometry.pose is parent<-child; Odometry.twist is expressed in
  // child_frame_id.
  State curr;
  curr.x = latest_odom->pose.pose.position.x;
  curr.y = latest_odom->pose.pose.position.y;
  extractGlobalVelocityAndYaw(latest_odom, curr.vx, curr.vy, curr.omega,
                              curr.yaw);
  curr.yaw = normalizeYaw(curr.yaw);

  const double noise_threshold = 0.03;
  if (std::abs(curr.vx) < noise_threshold) {
    curr.vx = 0.0;
  }
  if (std::abs(curr.vy) < noise_threshold) {
    curr.vy = 0.0;
  }
  if (std::abs(curr.omega) < noise_threshold) {
    curr.omega = 0.0;
  }
  if (!std::isfinite(curr.x) || !std::isfinite(curr.y) ||
      !std::isfinite(curr.yaw) || !std::isfinite(curr.vx) ||
      !std::isfinite(curr.vy) || !std::isfinite(curr.omega)) {
    return stop("NONFINITE_STATE");
  }
  const double measured_planar_speed = std::hypot(curr.vx, curr.vy);
  if (measured_planar_speed > mpc_config_.max_planar_speed + 0.05) {
    RCLCPP_WARN_THROTTLE(
        logger_, *node->get_clock(), 1000,
        "Minco MPC measured planar speed %.3f m/s exceeds the %.3f m/s "
        "planner/controller contract; output remains radially limited.",
        measured_planar_speed, mpc_config_.max_planar_speed);
  }

  geometry_msgs::msg::TwistStamped cmd;
  cmd.header.stamp = now;
  cmd.header.frame_id = global_frame_;

  // Sync Nav2 success semantics using poses expressed in the same frame. The
  // path received by setPlan() is normally in map, while curr is in odom.
  if (goal_checker != nullptr) {
    geometry_msgs::msg::PoseStamped goal_pose;
    bool has_goal_pose = false;
    {
      std::lock_guard<std::mutex> lk(plan_mtx_);
      if (!global_plan_.poses.empty()) {
        goal_pose = global_plan_.poses.back();
        has_goal_pose = true;
      }
    }

    if (has_goal_pose) {
      try {
        if (goal_pose.header.frame_id.empty()) {
          goal_pose.header.frame_id = map_frame_;
        }
        if (goal_pose.header.frame_id != global_frame_) {
          goal_pose = tf_->transform(goal_pose, global_frame_);
        }
      } catch (const tf2::TransformException &) {
        has_goal_pose = false;
      }
    }

    if (has_goal_pose &&
        goal_checker->isGoalReached(latest_odom->pose.pose, goal_pose.pose,
                                    velocity)) {
      {
        std::lock_guard<std::mutex> lock(data_mtx_);
        blocked_ = true;
        ++trajectory_generation_;
        latest_opt_path_.reset();
        pending_opt_path_.reset();
        has_opt_path_rx_time_ = false;
        has_pending_opt_path_rx_time_ = false;
        has_tracked_ref_ = false;
      }
      return stop("GOAL_REACHED", true);
    }
  }

  if (control_delay_compensation_ > 1e-3) {
    curr.x += curr.vx * control_delay_compensation_;
    curr.y += curr.vy * control_delay_compensation_;
    curr.yaw += curr.omega * control_delay_compensation_;
    curr.yaw = normalizeYaw(curr.yaw);
  }

  // 2) 构造参考序列：优先 /opt_path
  std::vector<ReferencePoint> ref;
  bool ok_ref = buildReferenceFromOptPath(curr, ref);

  if (!ok_ref) {
    return stop("NO_REFERENCE");
  }
  if (perf && !ref.empty()) {
    perf->ref_vx = ref.front().vel.x();
    perf->ref_vy = ref.front().vel.y();
    perf->ref_wz = ref.front().yaw_rate;
  }

  // 3) 调用 MPC 求解（输出 global_frame_ 系速度，此处为 camera_init）
  Control u_global;
  std::vector<State> pred_states;
  const auto solve_start = record_perf
                               ? std::chrono::steady_clock::now()
                               : std::chrono::steady_clock::time_point{};
#ifdef MINCO_DEBUG
  auto t_start = std::chrono::high_resolution_clock::now();
#endif
  bool success = false;
  {
    std::lock_guard<std::mutex> lock(solver_mtx_);
    if (solver_) {
      success = solver_->solve(curr, ref, u_global, &pred_states);
    }
  }
  if (perf) {
    perf->solve_time_ms = std::chrono::duration<double, std::milli>(
                              std::chrono::steady_clock::now() - solve_start)
                              .count();
  }

  publishVisualization(pred_states, curr);

#ifdef MINCO_DEBUG
  auto t_end = std::chrono::high_resolution_clock::now();
  if (success && !ref.empty()) {
    double dt_ms =
        std::chrono::duration<double, std::milli>(t_end - t_start).count();

    double plan_vx = u_global.vx;
    double plan_vy = u_global.vy;
    double ref_vx = ref[0].vel.x();
    double ref_vy = ref[0].vel.y();
    double v_err_x = plan_vx - ref_vx;
    double v_err_y = plan_vy - ref_vy;

    double curr_x = curr.x;
    double curr_y = curr.y;
    double ref_x = ref[0].pos.x();
    double ref_y = ref[0].pos.y();
    double p_err_x = curr_x - ref_x;
    double p_err_y = curr_y - ref_y;

    custom_log::log_block(std::string("\033[34m[MincoMpc] "), NV(plan_vx),
                          NV(plan_vy), NV(ref_vx), NV(ref_vy), NV(v_err_x),
                          NV(v_err_y), NV(curr_x), NV(curr_y), NV(ref_x),
                          NV(ref_y), NV(p_err_x), NV(p_err_y), NV(dt_ms));
  }
#endif

  if (!success) {
    std::cout << color_text::RED << "[MincoMpc] Solver Failed!"
              << color_text::RESET << std::endl;
    // Debug info for failure
    if (!ref.empty()) {
      std::cout << "[MincoMpc] Ref Size: " << ref.size() << std::endl;
      std::cout << "[MincoMpc] Curr: " << curr.x << ", " << curr.y << ", "
                << curr.yaw << std::endl;
      std::cout << "[MincoMpc] Ref[0]: " << ref[0].pos.x() << ", "
                << ref[0].pos.y() << ", " << ref[0].yaw << std::endl;
    }

    return stop("QP_FAILED");
  }

  // 5) 直接下发全局坐标系速度
  double vx_mpc = u_global.vx;
  double vy_mpc = u_global.vy;
  double wz = fixed_wz_;

  // 小陀螺模式：当启用时，始终输出固定的旋转速度，而不使用 MPC 输出的角速度。
  if (!use_small_gyro_mode_) {
    wz = std::min(mpc_config_.omega_max,
                  std::max(mpc_config_.omega_min, u_global.omega));
  }

  double output_delay = 0.025;
  double phase_delay = curr.omega * output_delay;
  double cos_phase = std::cos(phase_delay);
  double sin_phase = std::sin(phase_delay);
  double vx = cos_phase * vx_mpc + sin_phase * vy_mpc;
  double vy = -sin_phase * vx_mpc + cos_phase * vy_mpc;
  // 5) 处理 Nav2 setSpeedLimit
  if (speed_limit_ > 1e-6) {
    const double v_norm = std::hypot(vx, vy);
    if (v_norm > 1e-6) {
      double scale = 1.0;
      if (speed_limit_percentage_) {
        scale = std::clamp(speed_limit_ / 100.0, 0.0, 1.0);
      } else {
        scale = std::min(1.0, speed_limit_ / v_norm);
      }
      vx *= scale;
      vy *= scale;
    }
  }

  tf2::Quaternion attitude;
  tf2::fromMsg(latest_odom->pose.pose.orientation, attitude);
  double roll = 0.0;
  double pitch = 0.0;
  double attitude_yaw = 0.0;
  tf2::Matrix3x3(attitude).getRPY(roll, pitch, attitude_yaw);
  roll -= lidar_roll_offset_;
  const double tilt_limit = tilt_speed_limit::speedLimit(
      roll, pitch, slope_slowdown_start_angle_, slope_full_slowdown_angle_,
      mpc_config_.max_planar_speed, slope_speed_limit_);
  const double speed_before_tilt_limit = std::hypot(vx, vy);
  if (!tilt_speed_limit::apply(tilt_limit, vx, vy)) {
    return stop("INVALID_TILT_SPEED_LIMIT");
  }
  if (speed_before_tilt_limit > tilt_limit + 1.0e-3) {
    RCLCPP_INFO_THROTTLE(
        logger_, *node->get_clock(), 2000,
        "Minco MPC slope limit active: roll=%.3f pitch=%.3f speed %.3f -> %.3f m/s",
        roll, pitch, speed_before_tilt_limit, tilt_limit);
  }

  // 6) 死区截断
  const double deadzone = deadzone_speed_threshold_;
  if (std::hypot(vx, vy) < deadzone) {
    vx = 0.0;
    vy = 0.0;
  }
  if (!std::isfinite(vx) || !std::isfinite(vy) || !std::isfinite(wz)) {
    return stop("NONFINITE_COMMAND");
  }
  nav_msgs::msg::Odometry::SharedPtr current_odom;
  uint64_t current_generation = 0;
  if (!snapshotFreshInputs(node->now(), std::chrono::steady_clock::now(),
                           current_odom, current_generation, input_reason)) {
    return stop(input_reason);
  }
  bool generation_current = false;
  {
    std::lock_guard<std::mutex> lock(data_mtx_);
    generation_current = !blocked_ &&
                         current_generation == trajectory_generation &&
                         trajectory_generation_ == trajectory_generation;
    if (generation_current && cmd_vel_mpc_pub_) {
      geometry_msgs::msg::Twist raw_cmd;
      raw_cmd.linear.x = vx;
      raw_cmd.linear.y = vy;
      raw_cmd.angular.z = wz;
      cmd_vel_mpc_pub_->publish(raw_cmd);
    }
  }
  if (!generation_current) {
    return stop("TRAJECTORY_CHANGED_DURING_SOLVE");
  }
  // applyGravityCompensation(latest_odom, vx, vy);
  cmd.twist.linear.x = vx;
  cmd.twist.linear.y = vy;
  cmd.twist.angular.z = wz;
  return finish(cmd, true, "NONE");
}

void MincoMpcController::publishVisualization(
    const std::vector<State> &pred_path, const State &curr_state) {
  auto node = node_.lock();
  if (!node)
    return;

  rclcpp::Time now = node->now();

  // 1. 发布预测路径
  if (mpc_predict_path_pub_ &&
      mpc_predict_path_pub_->get_subscription_count() > 0 &&
      !pred_path.empty()) {
    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = now;
    path_msg.header.frame_id = global_frame_;

    for (const auto &s : pred_path) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = path_msg.header;
      ps.pose.position.x = s.x;
      ps.pose.position.y = s.y;
      ps.pose.position.z = 0.0;

      tf2::Quaternion q;
      q.setRPY(0, 0, s.yaw);
      ps.pose.orientation = tf2::toMsg(q);
      path_msg.poses.push_back(ps);
    }
    mpc_predict_path_pub_->publish(path_msg);
  }

  // 2. 发布实际路径
  geometry_msgs::msg::PoseStamped ps;
  ps.header.stamp = now;
  ps.header.frame_id = global_frame_;
  ps.pose.position.x = curr_state.x;
  ps.pose.position.y = curr_state.y;
  ps.pose.position.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(0, 0, curr_state.yaw);
  ps.pose.orientation = tf2::toMsg(q);

  if (real_path_history_.empty()) {
    real_path_history_.push_back(ps);
  } else {
    const auto &last = real_path_history_.back();
    double dist = std::hypot(last.pose.position.x - ps.pose.position.x,
                             last.pose.position.y - ps.pose.position.y);
    // 简单的距离过滤，避免原地不动时数据堆积
    if (dist > 0.02) {
      real_path_history_.push_back(ps);
    }
  }

  // 保持历史长度
  if (real_path_history_.size() > 5000) {
    size_t remove_count = real_path_history_.size() - 5000;
    real_path_history_.erase(real_path_history_.begin(),
                             real_path_history_.begin() + remove_count);
  }

  if (mpc_real_path_pub_ && mpc_real_path_pub_->get_subscription_count() > 0) {
    // 降频发布：1.0 Hz
    if ((now - last_real_path_pub_time_).seconds() > 1.0) {
      nav_msgs::msg::Path path_msg;
      path_msg.header.stamp = now;
      path_msg.header.frame_id = global_frame_;
      // 如果历史太长，可以只发布最近的一部分，或者对历史进行下采样（本例直接发布全部，但频率低）
      path_msg.poses = real_path_history_;
      mpc_real_path_pub_->publish(path_msg);
      last_real_path_pub_time_ = now;
    }
  }
}

} // namespace minco_controller

PLUGINLIB_EXPORT_CLASS(minco_controller::MincoMpcController,
                       nav2_core::Controller)
