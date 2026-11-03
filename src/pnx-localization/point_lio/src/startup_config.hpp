#pragma once
#include "joint_startup.hpp"
#include "parameters.h"

inline point_lio::JointStartupConfig readJointStartupConfig(rclcpp::Node & node)
{
  point_lio::JointStartupConfig config;
  const double magnitude = Eigen::Vector3d(gravity[0], gravity[1], gravity[2]).norm();
  config.solver.gravity = magnitude;
  config.solver.body_from_lidar.rotation = Eigen::Quaterniond(Lidar_R_wrt_IMU);
  config.solver.body_from_lidar.position = Lidar_T_wrt_IMU;
  config.solver.noise.gyro_sample_variance = imu_meas_omg_cov;
  config.solver.noise.accel_sample_variance = imu_meas_acc_cov;
  config.acceleration_scale = magnitude / acc_norm;
  config.max_imu_gap = max_imu_time_gap;
  config.max_scan_duration = max_scan_duration;
  config.acceleration_limit = satu_acc;
  config.angular_velocity_limit = satu_gyro;
  config.check_saturation = check_satu;
  config.min_scan_points = min_scan_points;
  auto & c = config.solver.initialization;
  const auto count = [&node](const std::string & name, int value, int minimum, int maximum) {
    const int result = node.declare_parameter<int>("initialization." + name, value);
    if (result < minimum || result > maximum) {
      throw std::invalid_argument("invalid initialization." + name);
    }
    return result;
  };
  c.window_frames = count("window_frames", 20, 3, 60);
  c.seed_imu_samples = count("seed_imu_samples", 31, 2, 1000);
  c.maximum_refinements = count("maximum_refinements", 15, 4, 30);
  c.optimization.thread_count = count("num_threads", 2, 1, 8);
  c.optimization.minimum_planes = count("minimum_planes", 10, 3, 10000);
  config.max_scan_points = count("max_scan_points", 30000, min_scan_points, 200000);
  c.sample_size = node.declare_parameter<double>("initialization.sample_size", 0.1);
  c.minimum_directional_support = node.declare_parameter<double>(
    "initialization.minimum_directional_support", 15.0);
  const auto bias = node.declare_parameter<std::vector<double>>(
    "initialization.accel_bias", std::vector<double>{0.0, 0.0, 0.0});
  const auto bias_std = node.declare_parameter<std::vector<double>>(
    "initialization.accel_bias_std", std::vector<double>{0.3, 0.3, 0.3});
  if (bias.size() != 3 || bias_std.size() != 3) {
    throw std::invalid_argument("initialization accel bias/prior must have three components");
  }
  config.solver.initial_accel_bias = Eigen::Vector3d(bias.data());
  config.solver.initial_accel_bias_std = Eigen::Vector3d(bias_std.data());
  return config;
}
