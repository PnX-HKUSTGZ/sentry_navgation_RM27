// Copyright 2026 RM27 contributors. SPDX-License-Identifier: Apache-2.0
#pragma once
#include "global_relocalization/core.hpp"
#include <memory>
#include <vector>

namespace global_relocalization {
struct RawImuSample {
  double stamp{0.0};
  Eigen::Vector3d angular_velocity{Eigen::Vector3d::Zero()};
  Eigen::Vector3d linear_acceleration{Eigen::Vector3d::Zero()};
};
struct RawFrontendParams {
  bool range_filter{false}, deterministic{false}, imu_rotation_prior{false}, numerical_recovery{false};
  // Optional command-model prior for a base commanded to spin in place.
  // It is soft, leaves all LiDAR observability/motion gates intact, and stays
  // disabled for general moving-base mapping frontends.
  bool stationary_translation_prior{false};
  double stationary_translation_sigma_floor{0.005};
  size_t submap_scans{1};
  double min_range{0.5}, max_range{30.0};
  double gyro_noise_density{0.005}, rotation_sigma_floor{0.005};
  std::string diagnostic_directory;
  bool record_iterations{false};
  double stationary_duration{0.6};
  size_t stationary_min_samples{40};
  double stationary_max_gyro{0.08}, stationary_gyro_std{0.015};
  double gravity_magnitude{9.81}, gravity_tolerance{0.8}, stationary_accel_std{0.20};
  double max_imu_gap{0.035}, max_scan_gap{0.35}, imu_history_sec{10.0};
  double leaf_size{0.12}, max_correspondence{0.5};
  int max_iterations{15}, retry_iterations{15};
  double max_fitness{0.12}, min_inlier_ratio{0.65};
  size_t min_points{80};
  double max_rotation_prior_error{0.08}, max_base_translation_step{0.12};
  double max_base_linear_speed{0.35};
  double min_geometry_eigen_ratio{0.001}, min_information_eigen_ratio{1e-6};
  double diagnostic_budget_sec{0.100};
};
struct RawIteration {
  int attempt{0}, iteration{0}, inner{0};
  double objective{0}, candidate_objective{0}, lambda{0}, rotation_increment{0}, translation_increment{0};
  bool accepted{false};
  Eigen::Matrix<double,6,1> increment{Eigen::Matrix<double,6,1>::Zero()};
};
struct RawRegistrationAttempt {
  int attempt{0},iterations{0};
  bool converged{false},transform_finite{false};
  double fitness{1e9},inlier_ratio{0},information_eigen_ratio{0},rotation_prior_error{0},translation_step{0},linear_speed{0};
  std::vector<std::string> rejected_gates;
};
inline bool rawRegistrationRetryEligible(const RawRegistrationAttempt & attempt,const RawFrontendParams & p) {
  return !attempt.converged&&attempt.transform_finite&&std::isfinite(attempt.fitness)&&
    std::isfinite(attempt.inlier_ratio)&&std::isfinite(attempt.information_eigen_ratio)&&
    attempt.fitness<=p.max_fitness&&attempt.inlier_ratio>=p.min_inlier_ratio&&
    attempt.information_eigen_ratio>=p.min_information_eigen_ratio;
}
struct RawFrontendResult {
  bool accepted{false}, gravity_initialized{false}, imu_covered{false};
  bool converged{false}, over_budget{false};
  bool transform_finite{false};
  int iterations{0},first_attempt_iterations{0},retry_count{0};
  std::vector<std::string> rejected_gates;
  std::vector<RawRegistrationAttempt> registration_attempts;
  std::string reason, failure_class{"hard_fault"};
  bool recovery_pending{false};
  size_t recovery_attempts{0}, recovered_scan_count{0};
  size_t input_points{0}, finite_points{0}, range_points{0}, submap_size{0};
  std::vector<RawIteration> iteration_trace;
  // Same sensor-frame finite/range filtering feeds local registration, query and confirmation.
  Cloud filtered_sensor_cloud;
  // Reference is the initial sensor origin, gravity aligned, with unknown global yaw.
  Eigen::Isometry3d reference_from_sensor{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d reference_from_base{Eigen::Isometry3d::Identity()};
  Eigen::Vector3d gyro_bias{Eigen::Vector3d::Zero()};  // In IMU coordinates.
  double cumulative_yaw{0.0}, angular_speed{0.0}, linear_speed{0.0};
  double fitness{1e9}, inlier_ratio{0.0}, translation_step{0.0};
  double rotation_prior_error{0.0}, geometry_eigen_ratio{0.0};
  double information_eigen_ratio{0.0}, processing_sec{0.0};
  double rotation_prior_sigma{0.0},rotation_prior_weight{0.0};
  double translation_prior_sigma{0.0},translation_prior_weight{0.0};
  size_t inliers{0}, points{0};
};
// ROS-free. addImu may run concurrently with the single processScan worker;
// it takes only a short buffer lock, never the registration lock. reset must run
// after that worker has stopped. Extrinsics are fixed during one action/trial.
class StationaryRawFrontend {
public:
  StationaryRawFrontend(const RawFrontendParams & params,
    const Eigen::Isometry3d & sensor_from_imu,
    const Eigen::Isometry3d & base_from_sensor);
  ~StationaryRawFrontend();
  StationaryRawFrontend(const StationaryRawFrontend &) = delete;
  StationaryRawFrontend & operator=(const StationaryRawFrontend &) = delete;
  void reset();
  // Node calls immediately after publishing zero velocity. Older queued scans cannot recover.
  void authorizeRecoveryAfterStop(double stamp);
  bool addImu(const RawImuSample & sample, std::string * reason = nullptr);
  RawFrontendResult processScan(const Cloud & sensor_cloud, double stamp,
    bool instantaneous = true);
private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace global_relocalization
