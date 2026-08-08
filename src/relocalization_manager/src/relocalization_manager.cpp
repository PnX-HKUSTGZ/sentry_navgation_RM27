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

#include "relocalization_manager/relocalization_manager.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <limits>
#include <rclcpp/logging.hpp>
#include <sstream>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "pcl/common/common.h"
#include "pcl/common/transforms.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/kdtree/kdtree_flann.h"
#include "pcl_conversions/pcl_conversions.h"
#include "small_gicp/pcl/pcl_registration.hpp"
#include "small_gicp/util/downsampling_omp.hpp"
#include "tf2_eigen/tf2_eigen.hpp"

namespace relocalization_manager
{
namespace
{
//把配置里的prior_pcd_transform转成Eigen变换，再作用到整张先验PCD地图上
Eigen::Affine3d poseVectorToAffine(const std::vector<double> & pose)
{
  Eigen::Affine3d transform = Eigen::Affine3d::Identity();
  if (pose.size() < 6) {
    return transform;
  }

  transform.translation() << pose[0], pose[1], pose[2];
  const Eigen::Quaterniond rotation = Eigen::AngleAxisd(pose[5], Eigen::Vector3d::UnitZ()) *
                                      Eigen::AngleAxisd(pose[4], Eigen::Vector3d::UnitY()) *
                                      Eigen::AngleAxisd(pose[3], Eigen::Vector3d::UnitX());
  transform.linear() = rotation.toRotationMatrix();
  return transform;
}

//判断一个 pose vector 是否近似为单位变换。
bool isIdentityPoseVector(const std::vector<double> & pose)
{
  if (pose.size() < 6) {
    return true;
  }

  return std::all_of(
    pose.begin(), pose.begin() + 6, [](double value) { return std::abs(value) < 1e-9; });
}

//打印一份点云的点数和 x/y/z 空间范围，主要用于调试先验 PCD 地图是否加载正确，以及经prior_pcd_transform变换后是否符合预期
void logCloudBounds(
  const rclcpp::Logger & logger, const std::string & label,
  const pcl::PointCloud<pcl::PointXYZ> & cloud)
{
  if (cloud.empty()) {
    RCLCPP_INFO(logger, "%s cloud is empty", label.c_str());
    return;
  }

  pcl::PointXYZ min_pt;
  pcl::PointXYZ max_pt;
  pcl::getMinMax3D(cloud, min_pt, max_pt);
  RCLCPP_INFO(
    logger, "%s cloud: points=%zu bbox x=[%.3f, %.3f] y=[%.3f, %.3f] z=[%.3f, %.3f]", label.c_str(),
    cloud.size(), min_pt.x, max_pt.x, min_pt.y, max_pt.y, min_pt.z, max_pt.z);
}

//把配置里的prior_pcd_transform转成Eigen::Isometry3d
Eigen::Isometry3d poseVectorToIsometry(const std::vector<double> & pose)
{
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  const Eigen::Affine3d affine = poseVectorToAffine(pose);
  transform.translation() = affine.translation();
  transform.linear() = affine.linear();
  return transform;
}

double normalizeAngle(double angle)
{
  constexpr double pi = 3.14159265358979323846;
  while (angle > pi) {
    angle -= 2.0 * pi;
  }
  while (angle < -pi) {
    angle += 2.0 * pi;
  }
  return angle;
}

double yawFromIsometry(const Eigen::Isometry3d & transform)
{
  const Eigen::Matrix3d rotation = transform.rotation();
  return std::atan2(rotation(1, 0), rotation(0, 0));
}

std::chrono::nanoseconds secondsToChrono(double seconds)
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(seconds));
}

std::string replacePathExtension(const std::string & path, const std::string & extension)
{
  if (path.empty()) {
    return "";
  }

  const std::size_t last_separator = path.find_last_of('/');
  const std::size_t last_dot = path.find_last_of('.');
  if (
    last_dot == std::string::npos ||
    (last_separator != std::string::npos && last_dot < last_separator)) {
    return path + extension;
  }
  return path.substr(0, last_dot) + extension;
}

std::string scanContextDatabasePathFromPriorPcd(const std::string & prior_pcd_file)
{
  std::string database_path = replacePathExtension(prior_pcd_file, ".scdb");
  const std::string pcd_directory_token = "/pcd/";
  const std::size_t pcd_directory_position = database_path.find(pcd_directory_token);
  if (pcd_directory_position != std::string::npos) {
    database_path.replace(pcd_directory_position, pcd_directory_token.size(), "/scan_context/");
  }
  return database_path;
}

std::string lowerCopy(std::string value)
{
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char character) {
    return static_cast<char>(std::tolower(character));
  });
  return value;
}

std::string normalizedFrameId(std::string frame_id)
{
  while (!frame_id.empty() && frame_id.front() == '/') {
    frame_id.erase(frame_id.begin());
  }
  return frame_id;
}

bool isScanContextBuildMode(const scan_context::ScanContextParams & params)
{
  return params.mode == "build";
}

bool isScanContextPriorBuildMode(const scan_context::ScanContextParams & params)
{
  return params.mode == "build" && params.build_source == "prior_pcd";
}

bool isScanContextLiveBuildMode(const scan_context::ScanContextParams & params)
{
  return params.mode == "build" && params.build_source == "live";
}

bool isScanContextQueryMode(const scan_context::ScanContextParams & params)
{
  return params.mode == "query";
}

double initPoseZ(const std::vector<double> & init_pose)
{
  return init_pose.size() >= 3 ? init_pose[2] : 0.0;
}

geometry_msgs::msg::Pose poseFromIsometry(const Eigen::Isometry3d & transform)
{
  geometry_msgs::msg::Pose pose;
  const Eigen::Vector3d translation = transform.translation();
  const Eigen::Quaterniond rotation(transform.rotation());
  pose.position.x = translation.x();
  pose.position.y = translation.y();
  pose.position.z = translation.z();
  pose.orientation.x = rotation.x();
  pose.orientation.y = rotation.y();
  pose.orientation.z = rotation.z();
  pose.orientation.w = rotation.w();
  return pose;
}

}  // namespace

SmallGicpVerifier::SmallGicpVerifier(
  const rclcpp::Logger & logger, const SmallGicpParams & params,
  const std::vector<double> & prior_pcd_transform)
: logger_(logger), params_(params), prior_pcd_transform_(prior_pcd_transform)
{
  global_map_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  prior_filter_map_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  prior_kdtree_ = std::make_shared<pcl::KdTreeFLANN<pcl::PointXYZ>>();
  register_ = std::make_shared<
    small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP>>();
}

bool SmallGicpVerifier::loadGlobalMap(const std::string & file_name)
{
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_name, *global_map_) == -1) {
    RCLCPP_ERROR(logger_, "Couldn't read PCD file: %s", file_name.c_str());
    return false;
  }

  RCLCPP_INFO(logger_, "Loaded global map with %zu points", global_map_->points.size());
  logCloudBounds(logger_, "Prior PCD raw", *global_map_);

  if (!isIdentityPoseVector(prior_pcd_transform_)) {
    const Eigen::Affine3d prior_pcd_transform = poseVectorToAffine(prior_pcd_transform_);
    transformGlobalMap(prior_pcd_transform, "Prior PCD transformed");
    RCLCPP_INFO_STREAM(
      logger_, "Applied prior_pcd_transform xyz/rpy = ["
                 << prior_pcd_transform_[0] << ", " << prior_pcd_transform_[1] << ", "
                 << prior_pcd_transform_[2] << ", " << prior_pcd_transform_[3] << ", "
                 << prior_pcd_transform_[4] << ", " << prior_pcd_transform_[5] << "]");
  } else {
    prepareTarget();
  }
  return true;
}

bool SmallGicpVerifier::setGlobalMap(const pcl::PointCloud<pcl::PointXYZ> & global_map)
{
  if (global_map.empty()) {
    RCLCPP_ERROR(logger_, "Cannot configure Small GICP with an empty global map.");
    return false;
  }

  *global_map_ = global_map;
  prepareTarget();
  return target_ && !target_->empty() && target_tree_;
}

void SmallGicpVerifier::transformGlobalMap(
  const Eigen::Affine3d & transform, const std::string & label)
{
  pcl::transformPointCloud(*global_map_, *global_map_, transform);
  logCloudBounds(logger_, label, *global_map_);
  prepareTarget();
}

const pcl::PointCloud<pcl::PointXYZ> & SmallGicpVerifier::globalMap() const { return *global_map_; }

void SmallGicpVerifier::updatePriorBounds()
{
  has_prior_bounds_ = false;
  if (!global_map_ || global_map_->empty()) {
    return;
  }

  pcl::getMinMax3D(*global_map_, prior_min_pt_, prior_max_pt_);
  has_prior_bounds_ = true;
}

void SmallGicpVerifier::filterSourceByPriorMap(
  const pcl::PointCloud<pcl::PointXYZ> & source, const Eigen::Isometry3d & map_to_odom_guess,
  pcl::PointCloud<pcl::PointXYZ> & filtered_source) const
{
  filtered_source.clear();
  filtered_source.header = source.header;

  if (!params_.filter_source_by_prior_map) {
    filtered_source = source;
    return;
  }

  if (!has_prior_bounds_) {
    filtered_source = source;
    return;
  }

  if (!map_to_odom_guess.matrix().allFinite()) {
    filtered_source = source;
    return;
  }

  const bool filter_by_prior_neighbor = prior_kdtree_ &&
                                        std::isfinite(params_.prior_neighbor_max_distance) &&
                                        params_.prior_neighbor_max_distance > 0.0;
  const double prior_bounds_margin =
    filter_by_prior_neighbor ? params_.prior_neighbor_max_distance : 0.0;
  const double max_neighbor_distance_sq =
    params_.prior_neighbor_max_distance * params_.prior_neighbor_max_distance;
  std::vector<int> nearest_indices(1);
  std::vector<float> nearest_distances_sq(1);

  filtered_source.reserve(source.size());
  for (const auto & point : source.points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
      continue;
    }

    const Eigen::Vector3d point_in_map =
      map_to_odom_guess * Eigen::Vector3d(point.x, point.y, point.z);
    if (!point_in_map.allFinite()) {
      continue;
    }

    if (
      point_in_map.x() < prior_min_pt_.x - prior_bounds_margin ||
      point_in_map.x() > prior_max_pt_.x + prior_bounds_margin ||
      point_in_map.y() < prior_min_pt_.y - prior_bounds_margin ||
      point_in_map.y() > prior_max_pt_.y + prior_bounds_margin) {
      continue;
    }

    if (filter_by_prior_neighbor) {
      pcl::PointXYZ point_in_map_pcl;
      point_in_map_pcl.x = point_in_map.x();
      point_in_map_pcl.y = point_in_map.y();
      point_in_map_pcl.z = point_in_map.z();
      if (
        prior_kdtree_->nearestKSearch(point_in_map_pcl, 1, nearest_indices, nearest_distances_sq) <=
          0 ||
        nearest_distances_sq.front() > max_neighbor_distance_sq) {
        continue;
      }
    }

    filtered_source.push_back(point);
  }

  filtered_source.width = filtered_source.size();
  filtered_source.height = 1;
  filtered_source.is_dense = false;
}

void SmallGicpVerifier::prepareTarget()
{
  updatePriorBounds();
  prior_filter_map_->clear();
  pcl::VoxelGrid<pcl::PointXYZ> prior_filter;
  prior_filter.setInputCloud(global_map_);
  prior_filter.setLeafSize(
    params_.global_leaf_size, params_.global_leaf_size, params_.global_leaf_size);
  prior_filter.filter(*prior_filter_map_);

  const pcl::PointCloud<pcl::PointXYZ>::ConstPtr prior_kdtree_cloud =
    prior_filter_map_->empty() ? global_map_ : prior_filter_map_;
  prior_kdtree_->setInputCloud(prior_kdtree_cloud);

  target_ = small_gicp::voxelgrid_sampling_omp<
    pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(
    *global_map_, params_.global_leaf_size);

  small_gicp::estimate_covariances_omp(*target_, params_.num_neighbors, params_.num_threads);

  target_tree_ = std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(
    target_, small_gicp::KdTreeBuilderOMP(params_.num_threads));
}

VerificationResult SmallGicpVerifier::verify(
  const pcl::PointCloud<pcl::PointXYZ> & accumulated_cloud, const Eigen::Isometry3d & initial_guess)
{
  GicpDeltaGate delta_gate;
  delta_gate.max_delta_xy = params_.max_delta_xy;
  delta_gate.max_delta_z = params_.max_delta_z;
  delta_gate.max_delta_yaw = params_.max_delta_yaw;
  return verify(accumulated_cloud, initial_guess, delta_gate, "local", params_.max_iterations);
}

VerificationResult SmallGicpVerifier::verify(
  const pcl::PointCloud<pcl::PointXYZ> & accumulated_cloud, const Eigen::Isometry3d & initial_guess,
  const GicpDeltaGate & delta_gate, const std::string & delta_gate_label, int max_iterations)
{
  VerificationResult verification;
  verification.transform = initial_guess;

  if (accumulated_cloud.empty()) {
    RCLCPP_WARN(logger_, "No accumulated points to process.");
    return verification;
  }

  if (!target_ || !target_tree_) {
    RCLCPP_WARN(logger_, "Small GICP target map is not prepared.");
    return verification;
  }

  pcl::PointCloud<pcl::PointXYZ> filtered_cloud;
  filterSourceByPriorMap(accumulated_cloud, initial_guess, filtered_cloud);
  const double retained_ratio =
    static_cast<double>(filtered_cloud.size()) / static_cast<double>(accumulated_cloud.size());

  if (filtered_cloud.empty()) {
    RCLCPP_WARN(
      logger_,
      "GICP source filtering removed all points: input_points=%zu "
      "prior_neighbor_max_distance=%.3f",
      accumulated_cloud.size(), params_.prior_neighbor_max_distance);
    return verification;
  }

  auto source = small_gicp::voxelgrid_sampling_omp<
    pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(
    filtered_cloud, params_.registered_leaf_size);

  if (!source || source->empty()) {
    RCLCPP_WARN(logger_, "Downsampled source cloud is empty.");
    return verification;
  }

  pcl::PointXYZ source_min_pt;
  pcl::PointXYZ source_max_pt;
  pcl::getMinMax3D(filtered_cloud, source_min_pt, source_max_pt);
  RCLCPP_INFO(
    logger_,
    "Verifying GICP source: input_points=%zu filtered_points=%zu retained_ratio=%.3f "
    "downsampled_points=%zu "
    "bbox x=[%.3f, %.3f] y=[%.3f, %.3f] z=[%.3f, %.3f] init_xyz=[%.3f, %.3f, %.3f]",
    accumulated_cloud.size(), filtered_cloud.size(), retained_ratio, source->size(), source_min_pt.x,
    source_max_pt.x, source_min_pt.y, source_max_pt.y, source_min_pt.z, source_max_pt.z,
    initial_guess.translation().x(), initial_guess.translation().y(),
    initial_guess.translation().z());

  small_gicp::estimate_covariances_omp(*source, params_.num_neighbors, params_.num_threads);

  register_->reduction.num_threads = params_.num_threads;
  register_->rejector.max_dist_sq = params_.max_dist_sq;
  const int bounded_max_iterations = std::max(1, max_iterations);
  register_->optimizer.max_iterations = bounded_max_iterations;

  const auto result = register_->align(*target_, *source, *target_tree_, initial_guess);

  const double inlier_ratio = source->empty() ? 0.0
                                              : static_cast<double>(result.num_inliers) /
                                                  source->size();  //内点比列，越高越好
  const double mean_error = result.num_inliers == 0 ? std::numeric_limits<double>::infinity()
                                                    : result.error / result.num_inliers;  //平均误差
  verification.converged = result.converged;
  verification.error = result.error;
  verification.mean_error = mean_error;
  verification.inlier_ratio = inlier_ratio;
  verification.num_inliers = result.num_inliers;
  verification.source_points = source->size();
  verification.iterations = result.iterations;

  const Eigen::Vector3d delta_translation =
    result.T_target_source.translation() - initial_guess.translation();
  const double delta_yaw =
    normalizeAngle(yawFromIsometry(result.T_target_source) - yawFromIsometry(initial_guess));

  if (!result.converged) {
    RCLCPP_WARN(
      logger_,
      "GICP did not converge: iterations=%zu error=%.2f mean_error=%.2f "
      "inliers_ratio=%zu/%zu ratio=%.3f max_iterations=%d "
      "delta=[dx=%.3f, dy=%.3f, dz=%.3f, dyaw=%.3f]",
      result.iterations, result.error, mean_error, result.num_inliers, source->size(), inlier_ratio,
      bounded_max_iterations, delta_translation.x(), delta_translation.y(), delta_translation.z(),
      delta_yaw);
    return verification;
  }

  if (
    mean_error > params_.max_mean_error || inlier_ratio < params_.min_inlier_ratio ||
    result.num_inliers < static_cast<std::size_t>(params_.min_inliers)) {
    RCLCPP_WARN(
      logger_,
      "GICP converged but rejected by quality gate: iterations=%zu error=%.2f "
      "mean_error=%.2f max_mean_error=%.2f inliers_ratio=%zu/%zu ratio=%.3f "
      "min_inlier_ratio=%.3f min_inliers=%d",
      result.iterations, result.error, mean_error, params_.max_mean_error, result.num_inliers,
      source->size(), inlier_ratio, params_.min_inlier_ratio, params_.min_inliers);
    return verification;
  }

  if (
    std::abs(delta_translation.x()) > delta_gate.max_delta_xy ||
    std::abs(delta_translation.y()) > delta_gate.max_delta_xy ||
    std::abs(delta_translation.z()) > delta_gate.max_delta_z ||
    std::abs(delta_yaw) > delta_gate.max_delta_yaw) {
    RCLCPP_WARN(
      logger_,
      "GICP converged but rejected by %s pose delta gate: iterations=%zu error=%.2f "
      "mean_error=%.2f inliers_ratio=%zu/%zu ratio=%.3f "
      "max_iterations=%d delta=[dx=%.3f, dy=%.3f, dz=%.3f, dyaw=%.3f] "
      "limit=[xy=%.3f, z=%.3f, yaw=%.3f]",
      delta_gate_label.c_str(), result.iterations, result.error, mean_error, result.num_inliers,
      source->size(), inlier_ratio, bounded_max_iterations, delta_translation.x(),
      delta_translation.y(), delta_translation.z(), delta_yaw, delta_gate.max_delta_xy,
      delta_gate.max_delta_z, delta_gate.max_delta_yaw);
    return verification;
  }

  RCLCPP_INFO(
    logger_,
    "GICP converge successfully with %s pose delta gate!: iterations=%zu error=%.2f "
    "mean_error=%.2f "
    "inliers_ratio=%zu/%zu ratio=%.3f "
    "max_iterations=%d delta=[dx=%.3f, dy=%.3f, dz=%.3f, dyaw=%.3f] "
    "limit=[xy=%.3f, z=%.3f, yaw=%.3f]",
    delta_gate_label.c_str(), result.iterations, result.error, mean_error, result.num_inliers,
    source->size(), inlier_ratio, bounded_max_iterations, delta_translation.x(),
    delta_translation.y(), delta_translation.z(), delta_yaw, delta_gate.max_delta_xy,
    delta_gate.max_delta_z, delta_gate.max_delta_yaw);
  verification.accepted = true;
  verification.transform = result.T_target_source;
  return verification;
}

const char * RelocalizationManagerNode::localizationStateName(LocalizationState state) const
{
  switch (state) {
    case LocalizationState::LOCALIZING:
      return "LOCALIZING";
    case LocalizationState::LOCALIZED:
      return "LOCALIZED";
  }

  return "UNKNOWN";
}

std::string RelocalizationManagerNode::localizationStateLogLabel(LocalizationState state) const
{
  switch (state) {
    case LocalizationState::LOCALIZING:
      return "\033[1;33mLOCALIZING\033[0m";
    case LocalizationState::LOCALIZED:
      return "\033[1;32mLOCALIZED\033[0m";
  }

  return "\033[1;31mUNKNOWN\033[0m";
}

bool RelocalizationManagerNode::isLocalized() const
{
  return localization_state_ == LocalizationState::LOCALIZED;
}

bool RelocalizationManagerNode::isRobotMoving() const { return robot_moving_.load(); }

void RelocalizationManagerNode::motionCommandCallback(
  const geometry_msgs::msg::Twist::SharedPtr msg)
{
  {
    std::lock_guard<std::mutex> lock(motion_command_mutex_);
    last_motion_command_time_ = this->now();
    has_motion_command_ = true;
  }

  const auto & linear_velocity = msg->linear;
  const double linear_speed = std::hypot(linear_velocity.x, linear_velocity.y);
  const bool moving = std::isfinite(linear_speed) && linear_speed > moving_linear_speed_threshold_;
  const bool was_moving = robot_moving_.exchange(moving);

  if (moving && !was_moving) {
    resetAcceptedGicpVerificationStreak();
    reset_accumulated_cloud_.store(true);
    std::ostringstream reason;
    reason << "commanded linear motion detected: linear_speed=" << std::fixed
           << std::setprecision(3) << linear_speed
           << " threshold=" << moving_linear_speed_threshold_ << " topic=" << motion_command_topic_;
    setLocalizationState(LocalizationState::LOCALIZING, reason.str());
  } else if (!moving && was_moving) {
    resetAcceptedGicpVerificationStreak();
    reset_accumulated_cloud_.store(true);
  }
}

void RelocalizationManagerNode::expireStaleMotionCommand()
{
  bool command_is_stale = false;
  {
    std::lock_guard<std::mutex> lock(motion_command_mutex_);
    if (!has_motion_command_ || !robot_moving_.load()) {
      return;
    }

    const double elapsed = (this->now() - last_motion_command_time_).seconds();
    command_is_stale = std::isfinite(elapsed) && elapsed > motion_command_timeout_;
  }

  if (command_is_stale && robot_moving_.exchange(false)) {
    resetAcceptedGicpVerificationStreak();
    reset_accumulated_cloud_.store(true);
  }
}

void RelocalizationManagerNode::setLocalizationState(
  LocalizationState state, const std::string & reason)
{
  if (localization_state_ == state) {
    RCLCPP_DEBUG(
      this->get_logger(), "Localization state remains %s: %s",
      localizationStateLogLabel(localization_state_).c_str(), reason.c_str());
    return;
  }

  const LocalizationState previous_state = localization_state_;
  localization_state_ = state;
  const std::string previous_label = localizationStateLogLabel(previous_state);
  const std::string current_label = localizationStateLogLabel(localization_state_);
  RCLCPP_INFO(
    this->get_logger(), "Localization state transition: %s -> %s (%s)", previous_label.c_str(),
    current_label.c_str(), reason.c_str());
}

void RelocalizationManagerNode::logLocalizationState()
{
  const std::string state_label = localizationStateLogLabel(localization_state_);

  RCLCPP_INFO(this->get_logger(), "Localization state: %s", state_label.c_str());
}

void RelocalizationManagerNode::recordAcceptedGicpVerification(const std::string & reason)
{
  if (isRobotMoving()) {
    resetAcceptedGicpVerificationStreak();
    return;
  }

  if (consecutive_gicp_acceptances_ < required_consecutive_gicp_acceptances_) {
    ++consecutive_gicp_acceptances_;
  }

  if (consecutive_gicp_acceptances_ >= required_consecutive_gicp_acceptances_) {
    setLocalizationState(
      LocalizationState::LOCALIZED, reason + " after required GICP verification streak");
  }
}

void RelocalizationManagerNode::resetAcceptedGicpVerificationStreak()
{
  consecutive_gicp_acceptances_ = 0;
}

RelocalizationManagerNode::RelocalizationManagerNode(const rclcpp::NodeOptions & options)
: Node("relocalization_manager", options),
  current_map_to_odom_(Eigen::Isometry3d::Identity()),
  previous_map_to_odom_(Eigen::Isometry3d::Identity())
{
  this->declare_parameter("num_threads", 4);
  this->declare_parameter("num_neighbors", 20);
  this->declare_parameter("max_iterations", 10);
  this->declare_parameter("global_leaf_size", 0.25);
  this->declare_parameter("registered_leaf_size", 0.25);
  this->declare_parameter("max_dist_sq", 1.0);
  this->declare_parameter("max_mean_error", 0.30);
  this->declare_parameter("min_inlier_ratio", 0.75);
  this->declare_parameter("min_inliers", 1000);
  this->declare_parameter("prior_neighbor_max_distance", 0.5);
  this->declare_parameter("max_delta_xy", 0.25);
  this->declare_parameter("max_delta_z", 0.15);
  this->declare_parameter("max_delta_yaw", 0.17453292519943295);
  this->declare_parameter("map_frame", "map");
  this->declare_parameter("odom_frame", "odom");
  this->declare_parameter("base_frame", "");
  this->declare_parameter("robot_base_frame", "");
  this->declare_parameter("lidar_frame", "");
  this->declare_parameter("prior_pcd_file", "");
  this->declare_parameter("init_pose", std::vector<double>{0., 0., 0., 0., 0., 0.});
  this->declare_parameter("prior_pcd_transform", std::vector<double>{0., 0., 0., 0., 0., 0.});
  this->declare_parameter("input_cloud_topic", "registered_scan");
  this->declare_parameter("initial_pose_topic", "initialpose");
  this->declare_parameter("motion_command_topic", "cmd_vel");
  this->declare_parameter("moving_linear_speed_threshold", 0.05);
  this->declare_parameter("motion_command_timeout", 0.5);
  this->declare_parameter("required_consecutive_gicp_acceptances", 5);
  this->declare_parameter("transform_prior_map_with_lidar_offset", false);
  this->declare_parameter("teaser_enabled", true);
  this->declare_parameter("teaser_query_on_startup", true);
  this->declare_parameter("teaser_query_on_gicp_failure", true);
  this->declare_parameter("teaser_failure_trigger_count", 5);
  this->declare_parameter("teaser_retry_interval", 3.0);
  this->declare_parameter("teaser_num_threads", 4);
  this->declare_parameter("teaser_min_source_points", 1000);
  this->declare_parameter("teaser_min_feature_points", 50);
  this->declare_parameter("teaser_min_correspondences", 30);
  this->declare_parameter("teaser_max_correspondences", 500);
  this->declare_parameter("teaser_voxel_size", 0.5);
  this->declare_parameter("teaser_normal_radius", 1.0);
  this->declare_parameter("teaser_fpfh_radius", 2.0);
  this->declare_parameter("teaser_cross_check", true);
  this->declare_parameter("teaser_feature_ratio_threshold", 0.90);
  this->declare_parameter("teaser_noise_bound", 0.4);
  this->declare_parameter("teaser_cbar2", 2.0);
  this->declare_parameter("teaser_rotation_max_iterations", 100);
  this->declare_parameter("teaser_rotation_gnc_factor", 1.4);
  this->declare_parameter("teaser_rotation_cost_threshold", 0.005);
  this->declare_parameter("teaser_max_clique_time_limit", 1.0);
  this->declare_parameter("teaser_use_exact_max_clique", true);
  this->declare_parameter("teaser_overlap_map_leaf_size", 0.10);
  this->declare_parameter("teaser_overlap_source_leaf_size", 0.20);
  this->declare_parameter("teaser_initial_overlap_max_distance", 0.8);
  this->declare_parameter("teaser_initial_min_overlap_ratio", 0.45);
  this->declare_parameter("teaser_initial_min_overlap_points", 200);
  this->declare_parameter("teaser_initial_max_overlap_rmse", 0.45);
  this->declare_parameter("teaser_coarse_num_neighbors", 20);
  this->declare_parameter("teaser_coarse_global_leaf_size", 0.25);
  this->declare_parameter("teaser_coarse_registered_leaf_size", 0.20);
  this->declare_parameter("teaser_coarse_max_dist_sq", 1.44);
  this->declare_parameter("teaser_coarse_prior_neighbor_max_distance", 1.2);
  this->declare_parameter("teaser_coarse_max_mean_error", 0.50);
  this->declare_parameter("teaser_coarse_min_inlier_ratio", 0.55);
  this->declare_parameter("teaser_coarse_min_inliers", 300);
  this->declare_parameter("teaser_coarse_max_iterations", 60);
  this->declare_parameter("teaser_coarse_max_delta_xy", 1.0);
  this->declare_parameter("teaser_coarse_max_delta_z", 0.40);
  this->declare_parameter("teaser_coarse_max_delta_yaw", 0.52);
  this->declare_parameter("teaser_coarse_overlap_max_distance", 0.40);
  this->declare_parameter("teaser_coarse_min_overlap_ratio", 0.65);
  this->declare_parameter("teaser_coarse_min_overlap_points", 500);
  this->declare_parameter("teaser_coarse_max_overlap_rmse", 0.25);
  this->declare_parameter("scan_context_mode", "off");
  this->declare_parameter("scan_context_database_path", "");
  this->declare_parameter("scan_context_input_frame", "base_footprint");
  this->declare_parameter("scan_context_build_source", "live");
  this->declare_parameter("scan_context_update_interval", 0.1);
  this->declare_parameter("scan_context_num_rings", 20);
  this->declare_parameter("scan_context_num_sectors", 60);
  this->declare_parameter("scan_context_max_radius", 25.0);
  this->declare_parameter("scan_context_min_height", -1.0);
  this->declare_parameter("scan_context_max_height", 3.0);
  this->declare_parameter("scan_context_num_candidates", 10);
  this->declare_parameter("scan_context_score_threshold", 0.15);
  this->declare_parameter("scan_context_keyframe_min_translation", 1.0);
  this->declare_parameter("scan_context_keyframe_min_yaw", 0.17453292519943295);
  this->declare_parameter("scan_context_keyframe_min_interval", 1.0);
  this->declare_parameter("scan_context_min_points", 500);
  this->declare_parameter("scan_context_prior_sample_resolution", 1.0);
  this->declare_parameter("scan_context_prior_leaf_size", 0.25);
  this->declare_parameter("scan_context_query_on_startup", true);
  this->declare_parameter("scan_context_query_on_gicp_failure", true);
  this->declare_parameter("scan_context_failure_trigger_count", 3);
  this->declare_parameter("scan_context_max_gicp_candidates", 5);
  this->declare_parameter("scan_context_max_iterations", 60);
  this->declare_parameter("scan_context_yaw_delta_sign", 1.0);
  this->declare_parameter("scan_context_max_delta_xy", 1.0);
  this->declare_parameter("scan_context_max_delta_z", 0.30);
  this->declare_parameter("scan_context_max_delta_yaw", 0.35);
  this->declare_parameter("scan_context_tf_lookup_timeout", 0.15);
  this->declare_parameter("scan_context_free_space_check", true);
  this->declare_parameter("scan_context_free_space_topic", "map");
  this->declare_parameter("scan_context_free_space_occupied_threshold", 65);
  this->declare_parameter("scan_context_free_space_reject_unknown", true);
  this->declare_parameter("scan_context_free_space_radius", 0.20);

  this->get_parameter("num_threads", small_gicp_params_.num_threads);
  this->get_parameter("num_neighbors", small_gicp_params_.num_neighbors);
  this->get_parameter("max_iterations", small_gicp_params_.max_iterations);
  this->get_parameter("global_leaf_size", small_gicp_params_.global_leaf_size);
  this->get_parameter("registered_leaf_size", small_gicp_params_.registered_leaf_size);
  this->get_parameter("max_dist_sq", small_gicp_params_.max_dist_sq);
  this->get_parameter("max_mean_error", small_gicp_params_.max_mean_error);
  this->get_parameter("min_inlier_ratio", small_gicp_params_.min_inlier_ratio);
  this->get_parameter("min_inliers", small_gicp_params_.min_inliers);
  this->get_parameter(
    "prior_neighbor_max_distance", small_gicp_params_.prior_neighbor_max_distance);
  this->get_parameter("max_delta_xy", small_gicp_params_.max_delta_xy);
  this->get_parameter("max_delta_z", small_gicp_params_.max_delta_z);
  this->get_parameter("max_delta_yaw", small_gicp_params_.max_delta_yaw);
  small_gicp_params_.min_inliers = std::max(0, small_gicp_params_.min_inliers);
  if (
    !std::isfinite(small_gicp_params_.prior_neighbor_max_distance) ||
    small_gicp_params_.prior_neighbor_max_distance < 0.0) {
    small_gicp_params_.prior_neighbor_max_distance = 0.5;
  }
  this->get_parameter("map_frame", map_frame_);
  this->get_parameter("odom_frame", odom_frame_);
  this->get_parameter("base_frame", base_frame_);
  this->get_parameter("robot_base_frame", robot_base_frame_);
  this->get_parameter("lidar_frame", lidar_frame_);
  this->get_parameter("prior_pcd_file", prior_pcd_file_);
  this->get_parameter("init_pose", init_pose_);
  this->get_parameter("prior_pcd_transform", prior_pcd_transform_);
  this->get_parameter("input_cloud_topic", input_cloud_topic_);
  this->get_parameter("initial_pose_topic", initial_pose_topic_);
  this->get_parameter("motion_command_topic", motion_command_topic_);
  this->get_parameter("moving_linear_speed_threshold", moving_linear_speed_threshold_);
  this->get_parameter("motion_command_timeout", motion_command_timeout_);
  this->get_parameter(
    "required_consecutive_gicp_acceptances", required_consecutive_gicp_acceptances_);
  this->get_parameter(
    "transform_prior_map_with_lidar_offset", transform_prior_map_with_lidar_offset_);
  this->get_parameter("teaser_enabled", teaser_params_.enabled);
  this->get_parameter("teaser_query_on_startup", teaser_query_on_startup_);
  this->get_parameter("teaser_query_on_gicp_failure", teaser_query_on_gicp_failure_);
  this->get_parameter("teaser_failure_trigger_count", teaser_failure_trigger_count_);
  this->get_parameter("teaser_retry_interval", teaser_retry_interval_);
  this->get_parameter("teaser_num_threads", teaser_params_.num_threads);
  this->get_parameter("teaser_min_source_points", teaser_params_.min_source_points);
  this->get_parameter("teaser_min_feature_points", teaser_params_.min_feature_points);
  this->get_parameter("teaser_min_correspondences", teaser_params_.min_correspondences);
  this->get_parameter("teaser_max_correspondences", teaser_params_.max_correspondences);
  this->get_parameter("teaser_voxel_size", teaser_params_.voxel_size);
  this->get_parameter("teaser_normal_radius", teaser_params_.normal_radius);
  this->get_parameter("teaser_fpfh_radius", teaser_params_.fpfh_radius);
  this->get_parameter("teaser_cross_check", teaser_params_.cross_check);
  this->get_parameter("teaser_feature_ratio_threshold", teaser_params_.feature_ratio_threshold);
  this->get_parameter("teaser_noise_bound", teaser_params_.noise_bound);
  this->get_parameter("teaser_cbar2", teaser_params_.cbar2);
  this->get_parameter("teaser_rotation_max_iterations", teaser_params_.rotation_max_iterations);
  this->get_parameter("teaser_rotation_gnc_factor", teaser_params_.rotation_gnc_factor);
  this->get_parameter("teaser_rotation_cost_threshold", teaser_params_.rotation_cost_threshold);
  this->get_parameter("teaser_max_clique_time_limit", teaser_params_.max_clique_time_limit);
  this->get_parameter("teaser_use_exact_max_clique", teaser_params_.use_exact_max_clique);
  this->get_parameter("teaser_overlap_map_leaf_size", teaser_params_.overlap_map_leaf_size);
  this->get_parameter("teaser_overlap_source_leaf_size", teaser_params_.overlap_source_leaf_size);
  this->get_parameter(
    "teaser_initial_overlap_max_distance", teaser_params_.initial_overlap_max_distance);
  this->get_parameter("teaser_initial_min_overlap_ratio", teaser_params_.initial_min_overlap_ratio);
  this->get_parameter(
    "teaser_initial_min_overlap_points", teaser_params_.initial_min_overlap_points);
  this->get_parameter("teaser_initial_max_overlap_rmse", teaser_params_.initial_max_overlap_rmse);
  this->get_parameter("teaser_coarse_num_neighbors", teaser_coarse_gicp_params_.num_neighbors);
  this->get_parameter(
    "teaser_coarse_global_leaf_size", teaser_coarse_gicp_params_.global_leaf_size);
  this->get_parameter(
    "teaser_coarse_registered_leaf_size", teaser_coarse_gicp_params_.registered_leaf_size);
  this->get_parameter("teaser_coarse_max_dist_sq", teaser_coarse_gicp_params_.max_dist_sq);
  this->get_parameter(
    "teaser_coarse_prior_neighbor_max_distance",
    teaser_coarse_gicp_params_.prior_neighbor_max_distance);
  this->get_parameter("teaser_coarse_max_mean_error", teaser_coarse_gicp_params_.max_mean_error);
  this->get_parameter(
    "teaser_coarse_min_inlier_ratio", teaser_coarse_gicp_params_.min_inlier_ratio);
  this->get_parameter("teaser_coarse_min_inliers", teaser_coarse_gicp_params_.min_inliers);
  this->get_parameter("teaser_coarse_max_iterations", teaser_coarse_max_iterations_);
  this->get_parameter("teaser_coarse_max_delta_xy", teaser_coarse_delta_gate_.max_delta_xy);
  this->get_parameter("teaser_coarse_max_delta_z", teaser_coarse_delta_gate_.max_delta_z);
  this->get_parameter("teaser_coarse_max_delta_yaw", teaser_coarse_delta_gate_.max_delta_yaw);
  this->get_parameter("teaser_coarse_overlap_max_distance", teaser_coarse_overlap_max_distance_);
  this->get_parameter("teaser_coarse_min_overlap_ratio", teaser_coarse_min_overlap_ratio_);
  this->get_parameter("teaser_coarse_min_overlap_points", teaser_coarse_min_overlap_points_);
  this->get_parameter("teaser_coarse_max_overlap_rmse", teaser_coarse_max_overlap_rmse_);
  this->get_parameter("scan_context_mode", scan_context_params_.mode);
  this->get_parameter("scan_context_database_path", scan_context_params_.database_path);
  this->get_parameter("scan_context_input_frame", scan_context_params_.input_frame);
  this->get_parameter("scan_context_build_source", scan_context_params_.build_source);
  this->get_parameter("scan_context_update_interval", scan_context_params_.update_interval);
  this->get_parameter("scan_context_num_rings", scan_context_params_.num_rings);
  this->get_parameter("scan_context_num_sectors", scan_context_params_.num_sectors);
  this->get_parameter("scan_context_max_radius", scan_context_params_.max_radius);
  this->get_parameter("scan_context_min_height", scan_context_params_.min_height);
  this->get_parameter("scan_context_max_height", scan_context_params_.max_height);
  this->get_parameter("scan_context_num_candidates", scan_context_params_.num_candidates);
  this->get_parameter("scan_context_score_threshold", scan_context_params_.score_threshold);
  this->get_parameter(
    "scan_context_keyframe_min_translation", scan_context_params_.keyframe_min_translation);
  this->get_parameter("scan_context_keyframe_min_yaw", scan_context_params_.keyframe_min_yaw);
  this->get_parameter(
    "scan_context_keyframe_min_interval", scan_context_params_.keyframe_min_interval);
  this->get_parameter("scan_context_min_points", scan_context_params_.min_points);
  this->get_parameter(
    "scan_context_prior_sample_resolution", scan_context_params_.prior_sample_resolution);
  this->get_parameter("scan_context_prior_leaf_size", scan_context_params_.prior_leaf_size);
  this->get_parameter("scan_context_query_on_startup", scan_context_query_on_startup_);
  this->get_parameter("scan_context_query_on_gicp_failure", scan_context_query_on_gicp_failure_);
  this->get_parameter("scan_context_failure_trigger_count", scan_context_failure_trigger_count_);
  this->get_parameter("scan_context_max_gicp_candidates", scan_context_max_gicp_candidates_);
  this->get_parameter("scan_context_max_iterations", scan_context_max_iterations_);
  this->get_parameter("scan_context_yaw_delta_sign", scan_context_yaw_delta_sign_);
  this->get_parameter("scan_context_max_delta_xy", scan_context_max_delta_xy_);
  this->get_parameter("scan_context_max_delta_z", scan_context_max_delta_z_);
  this->get_parameter("scan_context_max_delta_yaw", scan_context_max_delta_yaw_);
  this->get_parameter("scan_context_tf_lookup_timeout", scan_context_tf_lookup_timeout_);
  this->get_parameter("scan_context_free_space_check", scan_context_free_space_check_);
  this->get_parameter("scan_context_free_space_topic", scan_context_free_space_topic_);
  this->get_parameter(
    "scan_context_free_space_occupied_threshold", scan_context_free_space_occupied_threshold_);
  this->get_parameter(
    "scan_context_free_space_reject_unknown", scan_context_free_space_reject_unknown_);
  this->get_parameter("scan_context_free_space_radius", scan_context_free_space_radius_);

  scan_context_params_.mode = lowerCopy(scan_context_params_.mode);
  scan_context_params_.build_source = lowerCopy(scan_context_params_.build_source);
  scan_context_failure_trigger_count_ = std::max(1, scan_context_failure_trigger_count_);
  scan_context_max_gicp_candidates_ = std::max(1, scan_context_max_gicp_candidates_);
  scan_context_max_iterations_ = std::max(1, scan_context_max_iterations_);
  required_consecutive_gicp_acceptances_ = std::max(1, required_consecutive_gicp_acceptances_);
  if (!std::isfinite(moving_linear_speed_threshold_) || moving_linear_speed_threshold_ < 0.0) {
    moving_linear_speed_threshold_ = 0.05;
  }
  if (!std::isfinite(motion_command_timeout_) || motion_command_timeout_ <= 0.0) {
    motion_command_timeout_ = 0.5;
  }
  if (!std::isfinite(scan_context_yaw_delta_sign_) || scan_context_yaw_delta_sign_ == 0.0) {
    scan_context_yaw_delta_sign_ = 1.0;
  }
  scan_context_yaw_delta_sign_ = scan_context_yaw_delta_sign_ > 0.0 ? 1.0 : -1.0;
  if (!std::isfinite(scan_context_max_delta_xy_) || scan_context_max_delta_xy_ < 0.0) {
    scan_context_max_delta_xy_ = 1.0;
  }
  if (!std::isfinite(scan_context_max_delta_z_) || scan_context_max_delta_z_ < 0.0) {
    scan_context_max_delta_z_ = 0.30;
  }
  if (!std::isfinite(scan_context_max_delta_yaw_) || scan_context_max_delta_yaw_ < 0.0) {
    scan_context_max_delta_yaw_ = 0.35;
  }
  if (!std::isfinite(scan_context_tf_lookup_timeout_) || scan_context_tf_lookup_timeout_ < 0.0) {
    scan_context_tf_lookup_timeout_ = 0.15;
  }
  if (scan_context_free_space_topic_.empty()) {
    scan_context_free_space_topic_ = "map";
  }
  scan_context_free_space_occupied_threshold_ =
    std::max(0, std::min(100, scan_context_free_space_occupied_threshold_));
  if (!std::isfinite(scan_context_free_space_radius_) || scan_context_free_space_radius_ < 0.0) {
    scan_context_free_space_radius_ = 0.20;
  }

  teaser_failure_trigger_count_ = std::max(1, teaser_failure_trigger_count_);
  teaser_params_.num_threads = std::max(1, teaser_params_.num_threads);
  teaser_params_.min_source_points = std::max(1, teaser_params_.min_source_points);
  teaser_params_.min_feature_points = std::max(3, teaser_params_.min_feature_points);
  teaser_params_.min_correspondences = std::max(3, teaser_params_.min_correspondences);
  teaser_params_.max_correspondences =
    std::max(teaser_params_.min_correspondences, teaser_params_.max_correspondences);
  teaser_params_.rotation_max_iterations = std::max(1, teaser_params_.rotation_max_iterations);
  teaser_params_.initial_min_overlap_points =
    std::max(1, teaser_params_.initial_min_overlap_points);
  teaser_coarse_gicp_params_.num_threads = teaser_params_.num_threads;
  teaser_coarse_gicp_params_.min_inliers = std::max(1, teaser_coarse_gicp_params_.min_inliers);
  teaser_coarse_gicp_params_.filter_source_by_prior_map = true;
  teaser_coarse_gicp_params_.max_iterations = std::max(1, teaser_coarse_max_iterations_);
  teaser_coarse_gicp_params_.max_delta_xy = teaser_coarse_delta_gate_.max_delta_xy;
  teaser_coarse_gicp_params_.max_delta_z = teaser_coarse_delta_gate_.max_delta_z;
  teaser_coarse_gicp_params_.max_delta_yaw = teaser_coarse_delta_gate_.max_delta_yaw;
  teaser_coarse_max_iterations_ = std::max(1, teaser_coarse_max_iterations_);
  teaser_coarse_min_overlap_points_ = std::max(1, teaser_coarse_min_overlap_points_);

  const bool teaser_parameters_valid =
    std::isfinite(teaser_retry_interval_) && teaser_retry_interval_ >= 0.0 &&
    std::isfinite(teaser_params_.voxel_size) && teaser_params_.voxel_size > 0.0 &&
    std::isfinite(teaser_params_.normal_radius) && teaser_params_.normal_radius > 0.0 &&
    std::isfinite(teaser_params_.fpfh_radius) &&
    teaser_params_.fpfh_radius > teaser_params_.normal_radius &&
    std::isfinite(teaser_params_.feature_ratio_threshold) &&
    teaser_params_.feature_ratio_threshold > 0.0 && teaser_params_.feature_ratio_threshold <= 1.0 &&
    std::isfinite(teaser_params_.noise_bound) && teaser_params_.noise_bound > 0.0 &&
    std::isfinite(teaser_params_.cbar2) && teaser_params_.cbar2 > 0.0 &&
    std::isfinite(teaser_params_.rotation_gnc_factor) && teaser_params_.rotation_gnc_factor > 1.0 &&
    std::isfinite(teaser_params_.rotation_cost_threshold) &&
    teaser_params_.rotation_cost_threshold > 0.0 &&
    std::isfinite(teaser_params_.max_clique_time_limit) &&
    teaser_params_.max_clique_time_limit > 0.0 &&
    std::isfinite(teaser_params_.overlap_map_leaf_size) &&
    teaser_params_.overlap_map_leaf_size > 0.0 &&
    std::isfinite(teaser_params_.overlap_source_leaf_size) &&
    teaser_params_.overlap_source_leaf_size > 0.0 &&
    std::isfinite(teaser_params_.initial_overlap_max_distance) &&
    teaser_params_.initial_overlap_max_distance > 0.0 &&
    std::isfinite(teaser_params_.initial_min_overlap_ratio) &&
    teaser_params_.initial_min_overlap_ratio >= 0.0 &&
    teaser_params_.initial_min_overlap_ratio <= 1.0 &&
    std::isfinite(teaser_params_.initial_max_overlap_rmse) &&
    teaser_params_.initial_max_overlap_rmse > 0.0 && teaser_coarse_gicp_params_.num_neighbors > 0 &&
    teaser_coarse_gicp_params_.global_leaf_size > 0.0F &&
    teaser_coarse_gicp_params_.registered_leaf_size > 0.0F &&
    teaser_coarse_gicp_params_.max_dist_sq > 0.0F &&
    std::isfinite(teaser_coarse_gicp_params_.prior_neighbor_max_distance) &&
    teaser_coarse_gicp_params_.prior_neighbor_max_distance > 0.0 &&
    std::isfinite(teaser_coarse_gicp_params_.max_mean_error) &&
    teaser_coarse_gicp_params_.max_mean_error > 0.0 &&
    std::isfinite(teaser_coarse_gicp_params_.min_inlier_ratio) &&
    teaser_coarse_gicp_params_.min_inlier_ratio >= 0.0 &&
    teaser_coarse_gicp_params_.min_inlier_ratio <= 1.0 &&
    std::isfinite(teaser_coarse_delta_gate_.max_delta_xy) &&
    teaser_coarse_delta_gate_.max_delta_xy >= 0.0 &&
    std::isfinite(teaser_coarse_delta_gate_.max_delta_z) &&
    teaser_coarse_delta_gate_.max_delta_z >= 0.0 &&
    std::isfinite(teaser_coarse_delta_gate_.max_delta_yaw) &&
    teaser_coarse_delta_gate_.max_delta_yaw >= 0.0 &&
    std::isfinite(teaser_coarse_overlap_max_distance_) &&
    teaser_coarse_overlap_max_distance_ > 0.0 && std::isfinite(teaser_coarse_min_overlap_ratio_) &&
    teaser_coarse_min_overlap_ratio_ >= 0.0 && teaser_coarse_min_overlap_ratio_ <= 1.0 &&
    std::isfinite(teaser_coarse_max_overlap_rmse_) && teaser_coarse_max_overlap_rmse_ > 0.0;
  if (teaser_params_.enabled && !teaser_parameters_valid) {
    RCLCPP_ERROR(
      this->get_logger(),
      "TEASER++ parameters are invalid. Global TEASER++ relocalization will be disabled.");
    teaser_params_.enabled = false;
  }

  if (scan_context_params_.database_path.empty()) {
    scan_context_params_.database_path = scanContextDatabasePathFromPriorPcd(prior_pcd_file_);
  }

  if (!init_pose_.empty() && init_pose_.size() >= 6) {
    current_map_to_odom_ = poseVectorToIsometry(init_pose_);
  }
  previous_map_to_odom_ = current_map_to_odom_;

  accumulated_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  latest_scan_context_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
  prior_pcd_map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "prior_pcd_map", rclcpp::QoS(1).transient_local().reliable());
  state_log_timer_ = this->create_wall_timer(
    std::chrono::seconds(5), std::bind(&RelocalizationManagerNode::logLocalizationState, this));

  configureScanContext();

  const bool scan_context_build_mode = isScanContextBuildMode(scan_context_params_);
  const bool scan_context_prior_build_mode = isScanContextPriorBuildMode(scan_context_params_);
  const bool scan_context_query_mode = isScanContextQueryMode(scan_context_params_);

  if (!scan_context_build_mode) {
    small_gicp_verifier_ = std::make_unique<SmallGicpVerifier>(
      this->get_logger(), small_gicp_params_, prior_pcd_transform_);
    if (small_gicp_verifier_->loadGlobalMap(prior_pcd_file_)) {
      maybeApplyLidarOffsetToPriorMap();
      publishPriorPcdMap(small_gicp_verifier_->globalMap());
    }
    configureTeaserRelocalization();
  } else {
    save_scan_context_database_service_ = this->create_service<std_srvs::srv::Trigger>(
      "save_scan_context_database", std::bind(
                                      &RelocalizationManagerNode::saveScanContextDatabaseCallback,
                                      this, std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(
      this->get_logger(),
      "Scan Context build mode is active; GICP verification and map->odom publishing are "
      "disabled.");
  }

  if (scan_context_query_mode) {
    trigger_scan_context_relocalization_service_ = this->create_service<std_srvs::srv::Trigger>(
      "trigger_scan_context_relocalization",
      std::bind(
        &RelocalizationManagerNode::triggerScanContextRelocalizationCallback, this,
        std::placeholders::_1, std::placeholders::_2));
    scan_context_candidates_pub_ =
      this->create_publisher<geometry_msgs::msg::PoseArray>("scan_context_candidates", 10);
    scan_context_best_pose_pub_ =
      this->create_publisher<geometry_msgs::msg::PoseStamped>("scan_context_best_pose", 10);
    if (scan_context_free_space_check_) {
      occupancy_map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        scan_context_free_space_topic_, rclcpp::QoS(1).transient_local().reliable(),
        std::bind(&RelocalizationManagerNode::occupancyMapCallback, this, std::placeholders::_1));
    }
    scan_context_startup_query_pending_.store(scan_context_query_on_startup_);
    RCLCPP_INFO(
      this->get_logger(),
      "Scan Context query mode is active: query_on_startup=%s query_on_gicp_failure=%s "
      "failure_trigger_count=%d max_gicp_candidates=%d max_iterations=%d yaw_delta_sign=%.0f "
      "delta_gate=[xy=%.3f, z=%.3f, yaw=%.3f] tf_lookup_timeout=%.3f "
      "free_space_check=%s free_space_topic=%s free_space_radius=%.3f",
      scan_context_query_on_startup_ ? "true" : "false",
      scan_context_query_on_gicp_failure_ ? "true" : "false", scan_context_failure_trigger_count_,
      scan_context_max_gicp_candidates_, scan_context_max_iterations_, scan_context_yaw_delta_sign_,
      scan_context_max_delta_xy_, scan_context_max_delta_z_, scan_context_max_delta_yaw_,
      scan_context_tf_lookup_timeout_, scan_context_free_space_check_ ? "true" : "false",
      scan_context_free_space_topic_.c_str(), scan_context_free_space_radius_);
  }

  if (scan_context_prior_build_mode) {
    buildScanContextDatabaseFromPriorPcd();
  }

  if (scan_context_manager_ && !scan_context_prior_build_mode) {
    scan_context_timer_ = this->create_wall_timer(
      secondsToChrono(scan_context_params_.update_interval),
      std::bind(&RelocalizationManagerNode::processPendingScanContext, this));
  }

  if (!scan_context_prior_build_mode) {
    pcd_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      input_cloud_topic_, 10,
      std::bind(&RelocalizationManagerNode::registeredPcdCallback, this, std::placeholders::_1));
  }

  if (!scan_context_build_mode) {
    motion_command_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      motion_command_topic_, rclcpp::QoS(10),
      std::bind(&RelocalizationManagerNode::motionCommandCallback, this, std::placeholders::_1));

    initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      initial_pose_topic_, 10,
      std::bind(&RelocalizationManagerNode::initialPoseCallback, this, std::placeholders::_1));

    register_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&RelocalizationManagerNode::performRegistration, this));

    transform_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50), std::bind(&RelocalizationManagerNode::publishTransform, this));
  }
}

RelocalizationManagerNode::~RelocalizationManagerNode()
{
  if (teaser_future_.valid()) {
    teaser_future_.wait();
  }
}

void RelocalizationManagerNode::configureTeaserRelocalization()
{
  if (!teaser_params_.enabled) {
    RCLCPP_INFO(this->get_logger(), "TEASER++ global relocalization is disabled.");
    return;
  }
  if (!small_gicp_verifier_ || small_gicp_verifier_->globalMap().empty()) {
    RCLCPP_ERROR(
      this->get_logger(),
      "TEASER++ global relocalization requires a loaded prior PCD map and will stay disabled.");
    teaser_params_.enabled = false;
    return;
  }

  teaser_relocalizer_ = std::make_unique<TeaserRelocalizer>(teaser_params_);
  std::string setup_reason;
  if (!teaser_relocalizer_->setGlobalMap(small_gicp_verifier_->globalMap(), setup_reason)) {
    RCLCPP_ERROR(
      this->get_logger(), "Failed to prepare TEASER++ target map: reason=%s", setup_reason.c_str());
    teaser_relocalizer_.reset();
    teaser_params_.enabled = false;
    return;
  }

  teaser_coarse_gicp_verifier_ = std::make_unique<SmallGicpVerifier>(
    this->get_logger(), teaser_coarse_gicp_params_, std::vector<double>{});
  if (!teaser_coarse_gicp_verifier_->setGlobalMap(small_gicp_verifier_->globalMap())) {
    RCLCPP_ERROR(this->get_logger(), "Failed to prepare the TEASER++ coarse GICP target map.");
    teaser_coarse_gicp_verifier_.reset();
    teaser_relocalizer_.reset();
    teaser_params_.enabled = false;
    return;
  }

  trigger_teaser_relocalization_service_ = this->create_service<std_srvs::srv::Trigger>(
    "trigger_teaser_relocalization",
    std::bind(
      &RelocalizationManagerNode::triggerTeaserRelocalizationCallback, this, std::placeholders::_1,
      std::placeholders::_2));
  teaser_startup_query_pending_.store(teaser_query_on_startup_);
  next_teaser_attempt_time_ = std::chrono::steady_clock::now();

  RCLCPP_INFO(
    this->get_logger(),
    "TEASER++ global relocalization configured: target_features=%zu voxel=%.3f "
    "normal_radius=%.3f fpfh_radius=%.3f noise_bound=%.3f min_correspondences=%d "
    "coarse_prior_neighbor_max_distance=%.3f query_on_startup=%s "
    "query_on_gicp_failure=%s failure_trigger_count=%d",
    teaser_relocalizer_->targetFeaturePointCount(), teaser_params_.voxel_size,
    teaser_params_.normal_radius, teaser_params_.fpfh_radius, teaser_params_.noise_bound,
    teaser_params_.min_correspondences, teaser_coarse_gicp_params_.prior_neighbor_max_distance,
    teaser_query_on_startup_ ? "true" : "false",
    teaser_query_on_gicp_failure_ ? "true" : "false", teaser_failure_trigger_count_);
}

void RelocalizationManagerNode::triggerTeaserRelocalizationCallback(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  if (!teaser_params_.enabled || !teaser_relocalizer_ || !teaser_coarse_gicp_verifier_) {
    response->success = false;
    response->message = "TEASER++ global relocalization is not available";
    return;
  }
  if (teaser_relocalization_running_.load()) {
    response->success = false;
    response->message = "TEASER++ global relocalization is already running";
    return;
  }

  force_teaser_query_once_.store(true);
  response->success = true;
  response->message = "TEASER++ global relocalization queued";
}

bool RelocalizationManagerNode::startTeaserRelocalization(
  const pcl::PointCloud<pcl::PointXYZ> & source, const std::string & reason, bool bypass_cooldown)
{
  if (
    !teaser_params_.enabled || !teaser_relocalizer_ || !teaser_coarse_gicp_verifier_ ||
    !small_gicp_verifier_ || teaser_relocalization_running_.load()) {
    return false;
  }

  if (source.size() < static_cast<std::size_t>(teaser_params_.min_source_points)) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 3000,
      "TEASER++ relocalization is waiting for more source points: reason=%s points=%zu "
      "required=%d",
      reason.c_str(), source.size(), teaser_params_.min_source_points);
    return false;
  }

  const auto now = std::chrono::steady_clock::now();
  if (!bypass_cooldown && now < next_teaser_attempt_time_) {
    return false;
  }

  active_teaser_reason_ = reason;
  teaser_relocalization_running_.store(true);
  next_teaser_attempt_time_ = now + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                      std::chrono::duration<double>(teaser_retry_interval_));
  try {
    teaser_future_ = std::async(std::launch::async, [this, source, reason]() {
      return runTeaserRelocalization(source, reason);
    });
  } catch (const std::exception & exception) {
    teaser_relocalization_running_.store(false);
    RCLCPP_ERROR(
      this->get_logger(), "Failed to start TEASER++ relocalization task: %s", exception.what());
    return false;
  }

  RCLCPP_INFO(
    this->get_logger(), "Started asynchronous TEASER++ relocalization: reason=%s points=%zu",
    reason.c_str(), source.size());
  return true;
}

bool RelocalizationManagerNode::acceptCoarseOverlap(const NearestNeighborMetrics & metrics) const
{
  return metrics.valid && metrics.overlap_ratio >= teaser_coarse_min_overlap_ratio_ &&
         metrics.inliers >= static_cast<std::size_t>(teaser_coarse_min_overlap_points_) &&
         metrics.rmse <= teaser_coarse_max_overlap_rmse_;
}

TeaserPipelineResult RelocalizationManagerNode::runTeaserRelocalization(
  const pcl::PointCloud<pcl::PointXYZ> & source, const std::string & reason)
{
  TeaserPipelineResult pipeline;
  pipeline.teaser = teaser_relocalizer_->align(source);
  if (!pipeline.teaser.accepted) {
    pipeline.reason = std::string("teaser_") + pipeline.teaser.reason;
    RCLCPP_WARN(
      this->get_logger(),
      "TEASER++ coarse pose rejected: trigger=%s reason=%s source_features=%zu "
      "target_features=%zu correspondences=%zu overlap=%zu/%zu ratio=%.3f rmse=%.3f",
      reason.c_str(), pipeline.teaser.reason.c_str(), pipeline.teaser.source_feature_points,
      pipeline.teaser.target_feature_points, pipeline.teaser.correspondences,
      pipeline.teaser.overlap.inliers, pipeline.teaser.overlap.source_points,
      pipeline.teaser.overlap.overlap_ratio, pipeline.teaser.overlap.rmse);
    return pipeline;
  }

  RCLCPP_INFO(
    this->get_logger(),
    "TEASER++ coarse pose accepted: trigger=%s correspondences=%zu overlap=%zu/%zu "
    "ratio=%.3f rmse=%.3f xyz=[%.3f, %.3f, %.3f]",
    reason.c_str(), pipeline.teaser.correspondences, pipeline.teaser.overlap.inliers,
    pipeline.teaser.overlap.source_points, pipeline.teaser.overlap.overlap_ratio,
    pipeline.teaser.overlap.rmse, pipeline.teaser.transform.translation().x(),
    pipeline.teaser.transform.translation().y(), pipeline.teaser.transform.translation().z());

  pipeline.coarse_verification = teaser_coarse_gicp_verifier_->verify(
    source, pipeline.teaser.transform, teaser_coarse_delta_gate_, "teaser_coarse",
    teaser_coarse_max_iterations_);
  if (!pipeline.coarse_verification.accepted) {
    pipeline.reason = "coarse_gicp_rejected";
    return pipeline;
  }

  pipeline.coarse_overlap = teaser_relocalizer_->evaluateOverlap(
    source, pipeline.coarse_verification.transform, teaser_coarse_overlap_max_distance_);
  if (!acceptCoarseOverlap(pipeline.coarse_overlap)) {
    pipeline.reason = "coarse_overlap_rejected";
    RCLCPP_WARN(
      this->get_logger(),
      "TEASER++ coarse GICP rejected by nearest-neighbor overlap: inliers=%zu/%zu "
      "ratio=%.3f min_ratio=%.3f rmse=%.3f max_rmse=%.3f",
      pipeline.coarse_overlap.inliers, pipeline.coarse_overlap.source_points,
      pipeline.coarse_overlap.overlap_ratio, teaser_coarse_min_overlap_ratio_,
      pipeline.coarse_overlap.rmse, teaser_coarse_max_overlap_rmse_);
    return pipeline;
  }

  pipeline.final_verification =
    small_gicp_verifier_->verify(source, pipeline.coarse_verification.transform);
  if (!pipeline.final_verification.accepted) {
    pipeline.reason = "final_gicp_rejected";
    return pipeline;
  }

  pipeline.accepted = true;
  pipeline.reason = "accepted";
  return pipeline;
}

bool RelocalizationManagerNode::processTeaserRelocalizationResult()
{
  if (!teaser_relocalization_running_.load()) {
    return false;
  }
  if (
    !teaser_future_.valid() ||
    teaser_future_.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) {
    return true;
  }

  TeaserPipelineResult result;
  try {
    result = teaser_future_.get();
  } catch (const std::exception & exception) {
    result.reason = std::string("exception: ") + exception.what();
  }
  teaser_relocalization_running_.store(false);

  const bool startup_attempt = active_teaser_reason_ == "startup";
  if (result.accepted && !isRobotMoving()) {
    current_map_to_odom_ = result.final_verification.transform;
    previous_map_to_odom_ = result.final_verification.transform;
    consecutive_gicp_failures_ = 0;
    recordAcceptedGicpVerification(
      std::string("TEASER++ recovery accepted: ") + active_teaser_reason_);
    RCLCPP_INFO(
      this->get_logger(),
      "TEASER++ relocalization accepted: trigger=%s coarse_overlap=%zu/%zu ratio=%.3f "
      "rmse=%.3f final_mean_error=%.6f final_inlier_ratio=%.3f xyz=[%.3f, %.3f, %.3f]",
      active_teaser_reason_.c_str(), result.coarse_overlap.inliers,
      result.coarse_overlap.source_points, result.coarse_overlap.overlap_ratio,
      result.coarse_overlap.rmse, result.final_verification.mean_error,
      result.final_verification.inlier_ratio, result.final_verification.transform.translation().x(),
      result.final_verification.transform.translation().y(),
      result.final_verification.transform.translation().z());
  } else {
    if (startup_attempt) {
      teaser_startup_query_pending_.store(true);
    }
    resetAcceptedGicpVerificationStreak();
    RCLCPP_WARN(
      this->get_logger(), "TEASER++ relocalization failed: trigger=%s reason=%s moving=%s",
      active_teaser_reason_.c_str(), result.reason.c_str(), isRobotMoving() ? "true" : "false");
  }

  active_teaser_reason_.clear();
  return true;
}

void RelocalizationManagerNode::configureScanContext()
{
  if (scan_context_params_.mode == "off") {
    RCLCPP_INFO(this->get_logger(), "Scan Context is disabled.");
    return;
  }

  if (scan_context_params_.mode != "build" && scan_context_params_.mode != "query") {
    RCLCPP_ERROR(
      this->get_logger(),
      "Unsupported Scan Context mode: %s. Supported modes are: off, build, query. "
      "Scan Context will stay disabled.",
      scan_context_params_.mode.c_str());
    scan_context_params_.mode = "off";
    return;
  }

  if (
    scan_context_params_.mode == "build" && scan_context_params_.build_source != "live" &&
    scan_context_params_.build_source != "prior_pcd") {
    RCLCPP_ERROR(
      this->get_logger(),
      "Unsupported Scan Context build source: %s. Supported build sources are: live, prior_pcd. "
      "Scan Context will stay disabled.",
      scan_context_params_.build_source.c_str());
    scan_context_params_.mode = "off";
    return;
  }

  if (
    scan_context_params_.num_rings <= 0 || scan_context_params_.num_sectors <= 0 ||
    scan_context_params_.max_radius <= 0.0 || !std::isfinite(scan_context_params_.max_radius) ||
    !std::isfinite(scan_context_params_.min_height) ||
    !std::isfinite(scan_context_params_.max_height) ||
    scan_context_params_.min_height > scan_context_params_.max_height ||
    scan_context_params_.num_candidates <= 0 || scan_context_params_.update_interval <= 0.0 ||
    !std::isfinite(scan_context_params_.update_interval) ||
    scan_context_params_.prior_sample_resolution <= 0.0 ||
    !std::isfinite(scan_context_params_.prior_sample_resolution) ||
    scan_context_params_.prior_leaf_size < 0.0 ||
    !std::isfinite(scan_context_params_.prior_leaf_size)) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Scan Context parameters are invalid: rings=%d sectors=%d max_radius=%.3f "
      "height=[%.3f, %.3f] candidates=%d update_interval=%.3f "
      "prior_sample_resolution=%.3f prior_leaf_size=%.3f. "
      "Scan Context will stay disabled.",
      scan_context_params_.num_rings, scan_context_params_.num_sectors,
      scan_context_params_.max_radius, scan_context_params_.min_height,
      scan_context_params_.max_height, scan_context_params_.num_candidates,
      scan_context_params_.update_interval, scan_context_params_.prior_sample_resolution,
      scan_context_params_.prior_leaf_size);
    scan_context_params_.mode = "off";
    return;
  }

  scan_context_manager_ =
    std::make_unique<::scan_context::ScanContextManager>(scan_context_params_);

  RCLCPP_INFO(
    this->get_logger(),
    "Scan Context configured: mode=%s build_source=%s input_frame=%s database=%s "
    "update_interval=%.3f rings=%d sectors=%d max_radius=%.3f",
    scan_context_params_.mode.c_str(), scan_context_params_.build_source.c_str(),
    scan_context_params_.input_frame.c_str(), scan_context_params_.database_path.c_str(),
    scan_context_params_.update_interval, scan_context_params_.num_rings,
    scan_context_params_.num_sectors, scan_context_params_.max_radius);

  if (scan_context_params_.database_path.empty()) {
    RCLCPP_WARN(
      this->get_logger(), "Scan Context is enabled but scan_context_database_path is empty.");
    scan_context_manager_.reset();
    scan_context_params_.mode = "off";
    return;
  }

  if (scan_context_params_.mode == "build") {
    RCLCPP_INFO(
      this->get_logger(), "Scan Context build mode will create a fresh database: path=%s",
      scan_context_params_.database_path.c_str());
    return;
  }

  if (scan_context_manager_->loadDatabase(scan_context_params_.database_path)) {
    RCLCPP_INFO(
      this->get_logger(), "Loaded Scan Context database: path=%s keyframes=%zu",
      scan_context_params_.database_path.c_str(), scan_context_manager_->size());
  } else {
    RCLCPP_WARN(
      this->get_logger(), "Scan Context database was not loaded: path=%s",
      scan_context_params_.database_path.c_str());
    scan_context_manager_.reset();
    scan_context_params_.mode = "off";
  }
}

void RelocalizationManagerNode::publishPriorPcdMap(
  const pcl::PointCloud<pcl::PointXYZ> & prior_cloud)
{
  if (!prior_pcd_map_pub_) {
    return;
  }

  if (prior_cloud.empty()) {
    RCLCPP_WARN(this->get_logger(), "Cannot publish prior_pcd_map because prior cloud is empty.");
    return;
  }

  sensor_msgs::msg::PointCloud2 msg;
  pcl::toROSMsg(prior_cloud, msg);
  msg.header.stamp = this->now();
  msg.header.frame_id = map_frame_;
  prior_pcd_map_pub_->publish(msg);
  RCLCPP_INFO(
    this->get_logger(),
    "Published transformed prior PCD map: topic=prior_pcd_map frame=%s points=%zu",
    map_frame_.c_str(), prior_cloud.size());
}

void RelocalizationManagerNode::buildScanContextDatabaseFromPriorPcd()
{
  if (!scan_context_manager_) {
    return;
  }

  if (prior_pcd_file_.empty()) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Scan Context prior_pcd build source was requested, but prior_pcd_file is empty.");
    return;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr prior_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(prior_pcd_file_, *prior_cloud) == -1) {
    RCLCPP_ERROR(
      this->get_logger(), "Couldn't read prior PCD for Scan Context database: %s",
      prior_pcd_file_.c_str());
    return;
  }

  RCLCPP_INFO(
    this->get_logger(), "Loaded prior PCD for Scan Context database: path=%s points=%zu",
    prior_pcd_file_.c_str(), prior_cloud->size());

  if (!isIdentityPoseVector(prior_pcd_transform_)) {
    pcl::transformPointCloud(*prior_cloud, *prior_cloud, poseVectorToAffine(prior_pcd_transform_));
  }
  publishPriorPcdMap(*prior_cloud);

  pcl::PointCloud<pcl::PointXYZ>::Ptr sampled_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  if (scan_context_params_.prior_leaf_size > 0.0) {
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setInputCloud(prior_cloud);
    const float leaf_size = static_cast<float>(scan_context_params_.prior_leaf_size);
    voxel_filter.setLeafSize(leaf_size, leaf_size, leaf_size);
    voxel_filter.filter(*sampled_cloud);
  } else {
    sampled_cloud = prior_cloud;
  }

  if (sampled_cloud->empty()) {
    RCLCPP_ERROR(this->get_logger(), "Prior PCD for Scan Context is empty after filtering.");
    return;
  }

  pcl::PointXYZ min_pt;
  pcl::PointXYZ max_pt;
  pcl::getMinMax3D(*sampled_cloud, min_pt, max_pt);

  const double resolution = scan_context_params_.prior_sample_resolution;
  const double max_radius_sq = scan_context_params_.max_radius * scan_context_params_.max_radius;
  const double base_z = initPoseZ(init_pose_);
  const double max_z_delta = std::max(
    std::abs(static_cast<double>(min_pt.z) - base_z),
    std::abs(static_cast<double>(max_pt.z) - base_z));
  const double search_radius = std::sqrt(max_radius_sq + max_z_delta * max_z_delta);
  std::size_t attempted_positions = 0;
  std::size_t added_keyframes = 0;
  std::int64_t stamp_nanoseconds = 0;
  pcl::KdTreeFLANN<pcl::PointXYZ> local_search_tree;
  local_search_tree.setInputCloud(sampled_cloud);

  {
    std::lock_guard<std::mutex> lock(scan_context_database_mutex_);
    scan_context_manager_->clear();

    for (double x = min_pt.x; x <= max_pt.x; x += resolution) {
      for (double y = min_pt.y; y <= max_pt.y; y += resolution) {
        ++attempted_positions;

        pcl::PointXYZ query_center;
        query_center.x = static_cast<float>(x);
        query_center.y = static_cast<float>(y);
        query_center.z = static_cast<float>(base_z);
        std::vector<int> local_indices;
        std::vector<float> local_distances;
        if (
          local_search_tree.radiusSearch(
            query_center, search_radius, local_indices, local_distances) == 0) {
          continue;
        }

        pcl::PointCloud<pcl::PointXYZ> local_cloud;
        local_cloud.reserve(local_indices.size());

        for (const int point_index : local_indices) {
          const auto & point = sampled_cloud->points[static_cast<std::size_t>(point_index)];
          if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
            continue;
          }

          const double dx = static_cast<double>(point.x) - x;
          const double dy = static_cast<double>(point.y) - y;
          if (dx * dx + dy * dy > max_radius_sq) {
            continue;
          }

          pcl::PointXYZ local_point;
          local_point.x = static_cast<float>(dx);
          local_point.y = static_cast<float>(dy);
          local_point.z = static_cast<float>(static_cast<double>(point.z) - base_z);
          local_cloud.push_back(local_point);
        }

        const auto descriptor = scan_context_manager_->makeDescriptor(local_cloud);
        if (descriptor.scan_context.size() == 0) {
          continue;
        }

        Eigen::Isometry3d map_to_base = Eigen::Isometry3d::Identity();
        map_to_base.translation() << x, y, base_z;
        if (scan_context_manager_->addKeyframeDescriptor(
              descriptor, local_cloud.size(), map_to_base, stamp_nanoseconds, false)) {
          ++added_keyframes;
          stamp_nanoseconds += 1000000000LL;
        }
      }
    }
    scan_context_manager_->rebuildIndex();
    scan_context_database_dirty_ = added_keyframes > 0;
  }

  RCLCPP_INFO(
    this->get_logger(),
    "Built Scan Context database from prior PCD: source_points=%zu sampled_points=%zu "
    "grid_positions=%zu keyframes=%zu bbox x=[%.3f, %.3f] y=[%.3f, %.3f] base_z=%.3f",
    prior_cloud->size(), sampled_cloud->size(), attempted_positions, added_keyframes, min_pt.x,
    max_pt.x, min_pt.y, max_pt.y, base_z);

  if (added_keyframes == 0) {
    RCLCPP_WARN(
      this->get_logger(),
      "No Scan Context keyframes were generated from prior PCD. Check height limits, "
      "max_radius, min_points, and prior_sample_resolution.");
    return;
  }

  if (saveScanContextDatabase()) {
    RCLCPP_INFO(
      this->get_logger(), "Saved prior-built Scan Context database: path=%s",
      scan_context_params_.database_path.c_str());
  }
}

void RelocalizationManagerNode::maybeApplyLidarOffsetToPriorMap()
{
  if (!transform_prior_map_with_lidar_offset_) {
    RCLCPP_INFO(
      this->get_logger(), "Using prior PCD as loaded; no base->lidar mounting offset is applied.");
    return;
  }

  if (base_frame_.empty() || lidar_frame_.empty()) {
    RCLCPP_ERROR(
      this->get_logger(),
      "transform_prior_map_with_lidar_offset is true, but base_frame or lidar_frame is empty. "
      "Using prior PCD as loaded.");
    return;
  }

  Eigen::Affine3d odom_to_lidar_odom;
  while (rclcpp::ok()) {
    try {
      const auto tf_stamped = tf_buffer_->lookupTransform(
        base_frame_, lidar_frame_, this->now(), rclcpp::Duration::from_seconds(1.0));
      odom_to_lidar_odom = tf2::transformToEigen(tf_stamped.transform);
      RCLCPP_INFO_STREAM(
        this->get_logger(), "odom_to_lidar_odom: translation = "
                              << odom_to_lidar_odom.translation().transpose() << ", rpy = "
                              << odom_to_lidar_odom.rotation().eulerAngles(0, 1, 2).transpose());
      break;
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s Retrying...", ex.what());
      rclcpp::sleep_for(std::chrono::seconds(1));
    }
  }

  small_gicp_verifier_->transformGlobalMap(odom_to_lidar_odom, "Prior PCD after lidar offset");
}

void RelocalizationManagerNode::queueScanContextInput(
  const std_msgs::msg::Header & header, const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & scan)
{
  if (!scan_context_manager_) {
    return;
  }

  std::lock_guard<std::mutex> lock(scan_context_mutex_);
  pending_scan_context_header_ = header;
  pending_scan_context_cloud_ = scan;
  has_pending_scan_context_cloud_ = true;
}

void RelocalizationManagerNode::processPendingScanContext()
{
  if (!scan_context_manager_) {
    return;
  }

  pcl::PointCloud<pcl::PointXYZ>::ConstPtr scan;
  std_msgs::msg::Header header;
  {
    std::lock_guard<std::mutex> lock(scan_context_mutex_);
    if (!has_pending_scan_context_cloud_) {
      return;
    }
    scan = pending_scan_context_cloud_;
    header = pending_scan_context_header_;
    has_pending_scan_context_cloud_ = false;
  }

  if (!scan) {
    return;
  }

  updateScanContextDescriptor(header, *scan);
}

bool RelocalizationManagerNode::transformCloudToScanContextFrame(
  const std_msgs::msg::Header & header, const pcl::PointCloud<pcl::PointXYZ> & scan,
  pcl::PointCloud<pcl::PointXYZ> & cloud_in_scan_context_frame)
{
  if (scan_context_params_.input_frame.empty()) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "scan_context_input_frame is empty; cannot generate Scan Context descriptor.");
    return false;
  }

  if (header.frame_id.empty()) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "Input cloud frame_id is empty; cannot generate Scan Context descriptor.");
    return false;
  }

  if (header.frame_id == scan_context_params_.input_frame) {
    cloud_in_scan_context_frame = scan;
    return true;
  }

  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer_->lookupTransform(
      scan_context_params_.input_frame, header.frame_id, rclcpp::Time(header.stamp),
      rclcpp::Duration::from_seconds(scan_context_tf_lookup_timeout_));
  } catch (const tf2::TransformException & timed_ex) {
    try {
      transform = tf_buffer_->lookupTransform(
        scan_context_params_.input_frame, header.frame_id, tf2::TimePointZero);
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "Non-blocking Scan Context TF lookup at cloud stamp failed (%s -> %s): %s. "
        "Using latest transform instead.",
        header.frame_id.c_str(), scan_context_params_.input_frame.c_str(), timed_ex.what());
    } catch (const tf2::TransformException & latest_ex) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "Scan Context TF lookup failed (%s -> %s): %s", header.frame_id.c_str(),
        scan_context_params_.input_frame.c_str(), latest_ex.what());
      return false;
    }
  }

  const Eigen::Affine3d transform_eigen = tf2::transformToEigen(transform.transform);
  pcl::transformPointCloud(scan, cloud_in_scan_context_frame, transform_eigen);
  return true;
}

void RelocalizationManagerNode::updateScanContextDescriptor(
  const std_msgs::msg::Header & header, const pcl::PointCloud<pcl::PointXYZ> & scan)
{
  if (!scan_context_manager_) {
    return;
  }

  pcl::PointCloud<pcl::PointXYZ> cloud_in_scan_context_frame;
  if (!transformCloudToScanContextFrame(header, scan, cloud_in_scan_context_frame)) {
    return;
  }

  const auto descriptor = scan_context_manager_->makeDescriptor(cloud_in_scan_context_frame);
  if (descriptor.scan_context.size() == 0) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "Scan Context descriptor is empty: frame=%s raw_points=%zu transformed_points=%zu",
      scan_context_params_.input_frame.c_str(), scan.size(), cloud_in_scan_context_frame.size());
    return;
  }

  {
    std::lock_guard<std::mutex> lock(scan_context_mutex_);
    *latest_scan_context_cloud_ = cloud_in_scan_context_frame;
    latest_scan_context_descriptor_ = descriptor;
    latest_scan_context_stamp_ = rclcpp::Time(header.stamp);
    has_latest_scan_context_descriptor_ = true;
  }

  const Eigen::Index occupied_bins = (descriptor.scan_context.array() > 0.0F).count();
  RCLCPP_DEBUG_THROTTLE(
    this->get_logger(), *this->get_clock(), 5000,
    "Generated Scan Context descriptor: source_frame=%s target_frame=%s points=%zu bins=%ld/%ld",
    header.frame_id.c_str(), scan_context_params_.input_frame.c_str(),
    cloud_in_scan_context_frame.size(), static_cast<long>(occupied_bins),
    static_cast<long>(descriptor.scan_context.size()));

  if (isScanContextLiveBuildMode(scan_context_params_)) {
    maybeAddScanContextKeyframe(header, cloud_in_scan_context_frame, descriptor);
  }
}

bool RelocalizationManagerNode::lookupScanContextPoseInMap(
  const std_msgs::msg::Header & header, Eigen::Isometry3d & map_to_scan_context_frame)
{
  if (map_frame_.empty() || scan_context_params_.input_frame.empty()) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "Cannot add Scan Context keyframe because map_frame or scan_context_input_frame is empty.");
    return false;
  }

  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer_->lookupTransform(
      map_frame_, scan_context_params_.input_frame, rclcpp::Time(header.stamp),
      rclcpp::Duration::from_seconds(0.0));
  } catch (const tf2::TransformException & timed_ex) {
    try {
      transform = tf_buffer_->lookupTransform(
        map_frame_, scan_context_params_.input_frame, tf2::TimePointZero);
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "Non-blocking Scan Context keyframe pose lookup at cloud stamp failed (%s -> %s): %s. "
        "Using latest transform instead.",
        scan_context_params_.input_frame.c_str(), map_frame_.c_str(), timed_ex.what());
    } catch (const tf2::TransformException & latest_ex) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "Scan Context keyframe pose lookup failed (%s -> %s): %s",
        scan_context_params_.input_frame.c_str(), map_frame_.c_str(), latest_ex.what());
      return false;
    }
  }

  map_to_scan_context_frame = tf2::transformToEigen(transform.transform);
  return true;
}

bool RelocalizationManagerNode::shouldAddScanContextKeyframe(
  const Eigen::Isometry3d & map_to_scan_context_frame, const rclcpp::Time & stamp) const
{
  if (!has_last_scan_context_keyframe_) {
    return true;
  }

  if (scan_context_params_.keyframe_min_interval > 0.0) {
    const double dt = (stamp - last_scan_context_keyframe_stamp_).seconds();
    if (dt >= 0.0 && dt < scan_context_params_.keyframe_min_interval) {
      return false;
    }
  }

  const Eigen::Vector3d delta =
    map_to_scan_context_frame.translation() - last_scan_context_keyframe_pose_.translation();
  const double delta_xy = std::hypot(delta.x(), delta.y());
  const double delta_yaw = std::abs(normalizeAngle(
    yawFromIsometry(map_to_scan_context_frame) -
    yawFromIsometry(last_scan_context_keyframe_pose_)));

  return delta_xy >= scan_context_params_.keyframe_min_translation ||
         delta_yaw >= scan_context_params_.keyframe_min_yaw;
}

void RelocalizationManagerNode::maybeAddScanContextKeyframe(
  const std_msgs::msg::Header & header, const pcl::PointCloud<pcl::PointXYZ> & cloud,
  const ::scan_context::ScanContextDescriptor & descriptor)
{
  Eigen::Isometry3d map_to_scan_context_frame = Eigen::Isometry3d::Identity();
  if (!lookupScanContextPoseInMap(header, map_to_scan_context_frame)) {
    return;
  }

  const rclcpp::Time stamp(header.stamp);
  if (!shouldAddScanContextKeyframe(map_to_scan_context_frame, stamp)) {
    return;
  }

  bool added = false;
  std::size_t keyframe_count = 0;
  {
    std::lock_guard<std::mutex> lock(scan_context_database_mutex_);
    added = scan_context_manager_->addKeyframeDescriptor(
      descriptor, cloud.size(), map_to_scan_context_frame, stamp.nanoseconds());
    if (added) {
      scan_context_database_dirty_ = true;
      keyframe_count = scan_context_manager_->size();
    }
  }

  if (!added) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "Failed to add Scan Context keyframe: points=%zu descriptor_size=%ld", cloud.size(),
      static_cast<long>(descriptor.scan_context.size()));
    return;
  }

  has_last_scan_context_keyframe_ = true;
  last_scan_context_keyframe_pose_ = map_to_scan_context_frame;
  last_scan_context_keyframe_stamp_ = stamp;

  RCLCPP_INFO(
    this->get_logger(),
    "Added Scan Context keyframe #%zu: stamp=%.3f xyz=[%.3f, %.3f, %.3f] yaw=%.3f "
    "points=%zu",
    keyframe_count, stamp.seconds(), map_to_scan_context_frame.translation().x(),
    map_to_scan_context_frame.translation().y(), map_to_scan_context_frame.translation().z(),
    yawFromIsometry(map_to_scan_context_frame), cloud.size());
}

bool RelocalizationManagerNode::saveScanContextDatabase()
{
  if (!scan_context_manager_) {
    return false;
  }

  if (scan_context_params_.database_path.empty()) {
    RCLCPP_ERROR(
      this->get_logger(), "Cannot save Scan Context database because database path is empty.");
    return false;
  }

  std::lock_guard<std::mutex> lock(scan_context_database_mutex_);
  if (!scan_context_manager_->saveDatabase(scan_context_params_.database_path)) {
    return false;
  }

  scan_context_database_dirty_ = false;
  return true;
}

void RelocalizationManagerNode::saveScanContextDatabaseCallback(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;
  response->success = saveScanContextDatabase();
  if (response->success) {
    response->message = "Saved Scan Context database to " + scan_context_params_.database_path;
  } else {
    response->message =
      "Failed to save Scan Context database to " + scan_context_params_.database_path;
  }
}

void RelocalizationManagerNode::triggerScanContextRelocalizationCallback(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;
  if (!scan_context_manager_ || !isScanContextQueryMode(scan_context_params_)) {
    response->success = false;
    response->message = "Scan Context query mode is not active.";
    return;
  }

  force_scan_context_query_once_.store(true);
  response->success = true;
  response->message = "Scan Context relocalization will be attempted on the next GICP cycle.";
}

void RelocalizationManagerNode::occupancyMapCallback(
  const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  if (!msg) {
    return;
  }

  const std::size_t expected_size =
    static_cast<std::size_t>(msg->info.width) * static_cast<std::size_t>(msg->info.height);
  if (
    msg->info.width == 0 || msg->info.height == 0 ||
    !std::isfinite(static_cast<double>(msg->info.resolution)) || msg->info.resolution <= 0.0 ||
    msg->data.size() != expected_size) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "Ignoring invalid occupancy map for Scan Context free-space check: frame=%s "
      "width=%u height=%u resolution=%.6f data=%zu expected=%zu",
      msg->header.frame_id.c_str(), msg->info.width, msg->info.height, msg->info.resolution,
      msg->data.size(), expected_size);
    return;
  }

  std::lock_guard<std::mutex> lock(occupancy_map_mutex_);
  latest_occupancy_map_ = msg;
}

bool RelocalizationManagerNode::getLatestScanContextCloud(
  pcl::PointCloud<pcl::PointXYZ> & cloud_in_scan_context_frame, rclcpp::Time & stamp)
{
  std::lock_guard<std::mutex> lock(scan_context_mutex_);
  if (
    !has_latest_scan_context_descriptor_ || !latest_scan_context_cloud_ ||
    latest_scan_context_cloud_->empty()) {
    return false;
  }

  cloud_in_scan_context_frame = *latest_scan_context_cloud_;
  stamp = latest_scan_context_stamp_;
  return true;
}

bool RelocalizationManagerNode::makeMapToBaseEstimateFromScanContextCandidate(
  const ::scan_context::ScanContextCandidate & candidate, Eigen::Isometry3d & map_to_base)
{
  Eigen::Isometry3d yaw_correction = Eigen::Isometry3d::Identity();
  yaw_correction.linear() =
    Eigen::AngleAxisd(scan_context_yaw_delta_sign_ * candidate.yaw_delta, Eigen::Vector3d::UnitZ())
      .toRotationMatrix();

  map_to_base = candidate.map_to_base * yaw_correction;
  return map_to_base.matrix().allFinite();
}

bool RelocalizationManagerNode::lookupScanContextOdomToBase(
  const rclcpp::Time & stamp, Eigen::Isometry3d & odom_to_base)
{
  if (odom_frame_.empty() || scan_context_params_.input_frame.empty()) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "Cannot make Scan Context GICP guess because odom_frame or scan_context_input_frame is "
      "empty.");
    return false;
  }

  geometry_msgs::msg::TransformStamped odom_to_base_msg;
  try {
    odom_to_base_msg = tf_buffer_->lookupTransform(
      odom_frame_, scan_context_params_.input_frame, stamp,
      rclcpp::Duration::from_seconds(scan_context_tf_lookup_timeout_));
  } catch (const tf2::TransformException & timed_ex) {
    try {
      odom_to_base_msg = tf_buffer_->lookupTransform(
        odom_frame_, scan_context_params_.input_frame, tf2::TimePointZero);
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "Scan Context odom->base lookup at descriptor stamp failed (%s -> %s): %s. "
        "Using latest transform instead.",
        scan_context_params_.input_frame.c_str(), odom_frame_.c_str(), timed_ex.what());
    } catch (const tf2::TransformException & latest_ex) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "Scan Context odom->base lookup failed (%s -> %s): %s",
        scan_context_params_.input_frame.c_str(), odom_frame_.c_str(), latest_ex.what());
      return false;
    }
  }

  odom_to_base = tf2::transformToEigen(odom_to_base_msg.transform);
  return odom_to_base.matrix().allFinite();
}

bool RelocalizationManagerNode::isScanContextFreeSpaceMapReady() const
{
  if (!scan_context_free_space_check_) {
    return true;
  }

  std::lock_guard<std::mutex> lock(occupancy_map_mutex_);
  return static_cast<bool>(latest_occupancy_map_);
}

bool RelocalizationManagerNode::isScanContextCandidateInFreeSpace(
  const Eigen::Isometry3d & map_to_base_estimate, std::string & reject_reason) const
{
  if (!scan_context_free_space_check_) {
    return true;
  }

  if (!map_to_base_estimate.matrix().allFinite()) {
    reject_reason = "non_finite_candidate_pose";
    return false;
  }

  nav_msgs::msg::OccupancyGrid::SharedPtr occupancy_map;
  {
    std::lock_guard<std::mutex> lock(occupancy_map_mutex_);
    occupancy_map = latest_occupancy_map_;
  }

  if (!occupancy_map) {
    reject_reason = "map_unavailable";
    return false;
  }

  const std::string map_header_frame = normalizedFrameId(occupancy_map->header.frame_id);
  const std::string configured_map_frame = normalizedFrameId(map_frame_);
  if (
    !map_header_frame.empty() && !configured_map_frame.empty() &&
    map_header_frame != configured_map_frame) {
    reject_reason = "map_frame_mismatch";
    return false;
  }

  const auto & info = occupancy_map->info;
  const std::size_t expected_size =
    static_cast<std::size_t>(info.width) * static_cast<std::size_t>(info.height);
  if (
    info.width == 0 || info.height == 0 || !std::isfinite(static_cast<double>(info.resolution)) ||
    info.resolution <= 0.0 || occupancy_map->data.size() != expected_size) {
    reject_reason = "invalid_map";
    return false;
  }

  const auto & origin = info.origin;
  const Eigen::Vector3d origin_translation(origin.position.x, origin.position.y, origin.position.z);
  Eigen::Quaterniond origin_rotation(
    origin.orientation.w, origin.orientation.x, origin.orientation.y, origin.orientation.z);
  if (
    !origin_translation.allFinite() || !std::isfinite(origin_rotation.w()) ||
    !std::isfinite(origin_rotation.x()) || !std::isfinite(origin_rotation.y()) ||
    !std::isfinite(origin_rotation.z()) || origin_rotation.norm() < 1e-9) {
    reject_reason = "invalid_map_origin";
    return false;
  }
  origin_rotation.normalize();

  Eigen::Isometry3d map_origin = Eigen::Isometry3d::Identity();
  map_origin.translation() = origin_translation;
  map_origin.linear() = origin_rotation.toRotationMatrix();

  const Eigen::Vector3d candidate_position = map_to_base_estimate.translation();
  const Eigen::Vector3d grid_position = map_origin.inverse() * candidate_position;
  if (!grid_position.allFinite()) {
    reject_reason = "non_finite_grid_position";
    return false;
  }

  const double resolution = static_cast<double>(info.resolution);
  const int center_x = static_cast<int>(std::floor(grid_position.x() / resolution));
  const int center_y = static_cast<int>(std::floor(grid_position.y() / resolution));
  const int width = static_cast<int>(info.width);
  const int height = static_cast<int>(info.height);

  const auto cell_is_free = [&](int x, int y) {
    if (x < 0 || y < 0 || x >= width || y >= height) {
      reject_reason = "outside_map";
      return false;
    }

    const std::size_t index =
      static_cast<std::size_t>(y) * static_cast<std::size_t>(width) + static_cast<std::size_t>(x);
    const int occupancy = static_cast<int>(occupancy_map->data[index]);
    if (occupancy < 0) {
      if (scan_context_free_space_reject_unknown_) {
        reject_reason = "unknown_cell";
        return false;
      }
      return true;
    }

    if (occupancy >= scan_context_free_space_occupied_threshold_) {
      reject_reason = "occupied_cell";
      return false;
    }
    return true;
  };

  const int radius_cells =
    static_cast<int>(std::ceil(scan_context_free_space_radius_ / resolution));
  const double radius_sq = scan_context_free_space_radius_ * scan_context_free_space_radius_;
  for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
    for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
      const double distance_sq = static_cast<double>(dx * dx + dy * dy) * resolution * resolution;
      if (distance_sq > radius_sq) {
        continue;
      }

      if (!cell_is_free(center_x + dx, center_y + dy)) {
        return false;
      }
    }
  }

  return true;
}

void RelocalizationManagerNode::publishScanContextCandidates(
  const std::vector<::scan_context::ScanContextCandidate> & candidates,
  const std::vector<Eigen::Isometry3d> & map_to_base_estimates, const rclcpp::Time & stamp)
{
  if (!scan_context_candidates_pub_) {
    return;
  }

  geometry_msgs::msg::PoseArray pose_array;
  pose_array.header.stamp = stamp;
  pose_array.header.frame_id = map_frame_;
  const std::size_t pose_count = std::min(candidates.size(), map_to_base_estimates.size());
  pose_array.poses.reserve(pose_count);
  for (std::size_t i = 0; i < pose_count; ++i) {
    pose_array.poses.push_back(poseFromIsometry(map_to_base_estimates[i]));
  }
  scan_context_candidates_pub_->publish(pose_array);
}

void RelocalizationManagerNode::publishAcceptedScanContextPose(
  const Eigen::Isometry3d & map_to_base_estimate, const rclcpp::Time & stamp)
{
  if (!scan_context_best_pose_pub_) {
    return;
  }

  geometry_msgs::msg::PoseStamped pose;
  pose.header.stamp = stamp;
  pose.header.frame_id = map_frame_;
  pose.pose = poseFromIsometry(map_to_base_estimate);
  scan_context_best_pose_pub_->publish(pose);
}

bool RelocalizationManagerNode::verifyWithScanContextCandidates(
  const pcl::PointCloud<pcl::PointXYZ> & accumulated_cloud, const std::string & reason,
  VerificationResult & best_verification)
{
  if (
    !small_gicp_verifier_ || !scan_context_manager_ ||
    !isScanContextQueryMode(scan_context_params_)) {
    return false;
  }

  if (!isScanContextFreeSpaceMapReady()) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "Scan Context relocalization skipped: free-space map is not available yet. reason=%s "
      "topic=%s",
      reason.c_str(), scan_context_free_space_topic_.c_str());
    return false;
  }

  processPendingScanContext();

  pcl::PointCloud<pcl::PointXYZ> cloud_in_scan_context_frame;
  rclcpp::Time stamp;
  if (!getLatestScanContextCloud(cloud_in_scan_context_frame, stamp)) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "Scan Context relocalization skipped: no valid latest descriptor. reason=%s", reason.c_str());
    return false;
  }

  std::vector<::scan_context::ScanContextCandidate> candidates;
  {
    std::lock_guard<std::mutex> lock(scan_context_database_mutex_);
    candidates = scan_context_manager_->query(cloud_in_scan_context_frame);
  }

  if (candidates.empty()) {
    RCLCPP_WARN(
      this->get_logger(),
      "Scan Context relocalization found no candidates: reason=%s points=%zu "
      "score_threshold=%.3f",
      reason.c_str(), cloud_in_scan_context_frame.size(), scan_context_params_.score_threshold);
    return false;
  }

  const int max_candidates =
    std::min(scan_context_max_gicp_candidates_, static_cast<int>(candidates.size()));
  std::vector<::scan_context::ScanContextCandidate> valid_candidates;
  std::vector<Eigen::Isometry3d> map_to_base_estimates;
  valid_candidates.reserve(static_cast<std::size_t>(max_candidates));
  map_to_base_estimates.reserve(static_cast<std::size_t>(max_candidates));

  VerificationResult best_candidate_verification;
  Eigen::Isometry3d best_map_to_base_estimate = Eigen::Isometry3d::Identity();
  int best_candidate_index = -1;
  int free_space_rejected_candidates = 0;
  std::string first_free_space_reject_reason;

  RCLCPP_INFO(
    this->get_logger(),
    "Trying Scan Context relocalization: reason=%s candidates=%zu max_gicp_candidates=%d "
    "free_space_check=%s",
    reason.c_str(), candidates.size(), max_candidates,
    scan_context_free_space_check_ ? "true" : "false");

  Eigen::Isometry3d odom_to_base = Eigen::Isometry3d::Identity();
  if (!lookupScanContextOdomToBase(stamp, odom_to_base)) {
    return false;
  }
  const Eigen::Isometry3d base_to_odom = odom_to_base.inverse();
  GicpDeltaGate scan_context_delta_gate;
  scan_context_delta_gate.max_delta_xy = scan_context_max_delta_xy_;
  scan_context_delta_gate.max_delta_z = scan_context_max_delta_z_;
  scan_context_delta_gate.max_delta_yaw = scan_context_max_delta_yaw_;

  for (int i = 0; i < max_candidates; ++i) {
    const auto & candidate = candidates[static_cast<std::size_t>(i)];
    Eigen::Isometry3d map_to_base_estimate = Eigen::Isometry3d::Identity();
    if (!makeMapToBaseEstimateFromScanContextCandidate(candidate, map_to_base_estimate)) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "Scan Context candidate produced a non-finite map->base estimate.");
      continue;
    }

    std::string free_space_reject_reason;
    if (!isScanContextCandidateInFreeSpace(map_to_base_estimate, free_space_reject_reason)) {
      ++free_space_rejected_candidates;
      if (first_free_space_reject_reason.empty()) {
        first_free_space_reject_reason = free_space_reject_reason;
      }
      RCLCPP_WARN(
        this->get_logger(),
        "Scan Context candidate[%d] rejected by free-space check: keyframe_id=%d reason=%s "
        "map_base_xy=[%.3f, %.3f]",
        i, candidate.keyframe_id, free_space_reject_reason.c_str(),
        map_to_base_estimate.translation().x(), map_to_base_estimate.translation().y());
      continue;
    }

    const Eigen::Isometry3d map_to_odom_guess = map_to_base_estimate * base_to_odom;
    if (!map_to_odom_guess.matrix().allFinite()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "Scan Context candidate produced a non-finite map->odom guess.");
      continue;
    }

    valid_candidates.push_back(candidate);
    map_to_base_estimates.push_back(map_to_base_estimate);

    RCLCPP_INFO(
      this->get_logger(),
      "Scan Context candidate[%d]: keyframe_id=%d score=%.4f yaw_delta=%.3f "
      "map_base_xyz=[%.3f, %.3f, %.3f] guess_map_odom_xyz=[%.3f, %.3f, %.3f]",
      i, candidate.keyframe_id, candidate.score, scan_context_yaw_delta_sign_ * candidate.yaw_delta,
      map_to_base_estimate.translation().x(), map_to_base_estimate.translation().y(),
      map_to_base_estimate.translation().z(), map_to_odom_guess.translation().x(),
      map_to_odom_guess.translation().y(), map_to_odom_guess.translation().z());

    const VerificationResult verification = small_gicp_verifier_->verify(
      accumulated_cloud, map_to_odom_guess, scan_context_delta_gate, "scan_context",
      scan_context_max_iterations_);
    if (!verification.accepted) {
      continue;
    }

    const bool better_than_current =
      !best_candidate_verification.accepted ||
      verification.mean_error < best_candidate_verification.mean_error ||
      (std::abs(verification.mean_error - best_candidate_verification.mean_error) < 1e-9 &&
       verification.inlier_ratio > best_candidate_verification.inlier_ratio);
    if (better_than_current) {
      best_candidate_verification = verification;
      best_map_to_base_estimate = map_to_base_estimate;
      best_candidate_index = i;
    }
  }

  publishScanContextCandidates(valid_candidates, map_to_base_estimates, stamp);

  if (!best_candidate_verification.accepted) {
    RCLCPP_WARN(
      this->get_logger(),
      "Scan Context relocalization did not produce an accepted GICP result: reason=%s "
      "valid_candidates=%zu free_space_rejected=%d first_free_space_reject_reason=%s",
      reason.c_str(), valid_candidates.size(), free_space_rejected_candidates,
      first_free_space_reject_reason.empty() ? "none" : first_free_space_reject_reason.c_str());
    return false;
  }

  best_verification = best_candidate_verification;
  publishAcceptedScanContextPose(best_map_to_base_estimate, stamp);
  RCLCPP_INFO(
    this->get_logger(),
    "Scan Context relocalization accepted candidate[%d]: mean_error=%.6f "
    "inlier_ratio=%.3f transform_xyz=[%.3f, %.3f, %.3f]",
    best_candidate_index, best_verification.mean_error, best_verification.inlier_ratio,
    best_verification.transform.translation().x(), best_verification.transform.translation().y(),
    best_verification.transform.translation().z());
  return true;
}

void RelocalizationManagerNode::registeredPcdCallback(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  expireStaleMotionCommand();

  last_scan_time_ = msg->header.stamp;
  current_scan_frame_id_ = msg->header.frame_id;

  pcl::PointCloud<pcl::PointXYZ>::Ptr scan(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*msg, *scan);
  queueScanContextInput(msg->header, scan);
  if (small_gicp_verifier_) {
    if (reset_accumulated_cloud_.exchange(false)) {
      accumulated_cloud_->clear();
    }
    if (!isRobotMoving()) {
      *accumulated_cloud_ += *scan;
    }
  }
}

void RelocalizationManagerNode::performRegistration()
{
  if (!small_gicp_verifier_) {
    return;
  }

  expireStaleMotionCommand();

  if (processTeaserRelocalizationResult()) {
    accumulated_cloud_->clear();
    return;
  }

  if (reset_accumulated_cloud_.exchange(false)) {
    accumulated_cloud_->clear();
  }

  if (isRobotMoving()) {
    resetAcceptedGicpVerificationStreak();
    accumulated_cloud_->clear();
    if (isLocalized()) {
      setLocalizationState(LocalizationState::LOCALIZING, "commanded linear motion detected");
    }
    return;
  }

  if (accumulated_cloud_->empty()) {
    return;
  }

  const bool manual_teaser_query = force_teaser_query_once_.exchange(false);
  const bool startup_teaser_query = teaser_startup_query_pending_.exchange(false);
  if (manual_teaser_query || startup_teaser_query) {
    const std::string reason = manual_teaser_query ? "manual_trigger" : "startup";
    if (startTeaserRelocalization(*accumulated_cloud_, reason, manual_teaser_query)) {
      accumulated_cloud_->clear();
      return;
    }
    if (manual_teaser_query) {
      force_teaser_query_once_.store(true);
    }
    if (startup_teaser_query) {
      teaser_startup_query_pending_.store(true);
    }
  }

  const bool manual_scan_context_query = force_scan_context_query_once_.exchange(false);
  const bool startup_scan_context_query = scan_context_startup_query_pending_.exchange(false);
  const bool force_scan_context_query = manual_scan_context_query || startup_scan_context_query;
  if (force_scan_context_query) {
    if (!isScanContextFreeSpaceMapReady()) {
      if (manual_scan_context_query) {
        force_scan_context_query_once_.store(true);
      }
      if (startup_scan_context_query) {
        scan_context_startup_query_pending_.store(true);
      }
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "Scan Context query is still pending because free-space map is not available yet. "
        "topic=%s",
        scan_context_free_space_topic_.c_str());
    } else {
      VerificationResult scan_context_verification;
      const std::string reason = manual_scan_context_query ? "manual_trigger" : "startup";
      if (verifyWithScanContextCandidates(*accumulated_cloud_, reason, scan_context_verification)) {
        if (isRobotMoving()) {
          resetAcceptedGicpVerificationStreak();
          accumulated_cloud_->clear();
          return;
        }
        current_map_to_odom_ = scan_context_verification.transform;
        previous_map_to_odom_ = scan_context_verification.transform;
        consecutive_gicp_failures_ = 0;
        recordAcceptedGicpVerification(std::string("Scan Context accepted: ") + reason);
        accumulated_cloud_->clear();
        return;
      }

      if (startup_scan_context_query) {
        pcl::PointCloud<pcl::PointXYZ> latest_cloud;
        rclcpp::Time latest_stamp;
        if (!getLatestScanContextCloud(latest_cloud, latest_stamp)) {
          scan_context_startup_query_pending_.store(true);
          RCLCPP_DEBUG_THROTTLE(
            this->get_logger(), *this->get_clock(), 5000,
            "Startup Scan Context query is still pending because no descriptor is ready yet.");
        }
      }
    }
  }

  if (isLocalized()) {
    RCLCPP_DEBUG_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "Already localized; skipping periodic GICP refinement to keep map->odom stable.");
    accumulated_cloud_->clear();
    return;
  }

  const VerificationResult verification =
    small_gicp_verifier_->verify(*accumulated_cloud_, previous_map_to_odom_);

  if (verification.accepted) {
    if (isRobotMoving()) {
      resetAcceptedGicpVerificationStreak();
      accumulated_cloud_->clear();
      return;
    }
    current_map_to_odom_ = verification.transform;
    previous_map_to_odom_ = verification.transform;
    consecutive_gicp_failures_ = 0;
    recordAcceptedGicpVerification("local GICP accepted");
    accumulated_cloud_->clear();
    return;
  }

  resetAcceptedGicpVerificationStreak();
  ++consecutive_gicp_failures_;
  if (
    teaser_params_.enabled && teaser_query_on_gicp_failure_ &&
    consecutive_gicp_failures_ >= teaser_failure_trigger_count_ &&
    startTeaserRelocalization(*accumulated_cloud_, "gicp_failure", false)) {
    accumulated_cloud_->clear();
    return;
  }

  if (
    scan_context_query_on_gicp_failure_ && isScanContextQueryMode(scan_context_params_) &&
    consecutive_gicp_failures_ >= scan_context_failure_trigger_count_) {
    VerificationResult scan_context_verification;
    if (verifyWithScanContextCandidates(
          *accumulated_cloud_, "gicp_failure", scan_context_verification)) {
      if (isRobotMoving()) {
        resetAcceptedGicpVerificationStreak();
        accumulated_cloud_->clear();
        return;
      }
      current_map_to_odom_ = scan_context_verification.transform;
      previous_map_to_odom_ = scan_context_verification.transform;
      consecutive_gicp_failures_ = 0;
      recordAcceptedGicpVerification("Scan Context recovery accepted");
    }
  }

  accumulated_cloud_->clear();
}

void RelocalizationManagerNode::publishTransform()
{
  if (current_map_to_odom_.matrix().isZero()) {
    return;
  }

  geometry_msgs::msg::TransformStamped transform_stamped;
  transform_stamped.header.stamp = this->now() + rclcpp::Duration::from_seconds(0.1);
  transform_stamped.header.frame_id = map_frame_;
  transform_stamped.child_frame_id = odom_frame_;

  const Eigen::Vector3d translation = current_map_to_odom_.translation();
  const Eigen::Quaterniond rotation(current_map_to_odom_.rotation());

  transform_stamped.transform.translation.x = translation.x();
  transform_stamped.transform.translation.y = translation.y();
  transform_stamped.transform.translation.z = translation.z();
  transform_stamped.transform.rotation.x = rotation.x();
  transform_stamped.transform.rotation.y = rotation.y();
  transform_stamped.transform.rotation.z = rotation.z();
  transform_stamped.transform.rotation.w = rotation.w();

  tf_broadcaster_->sendTransform(transform_stamped);
}

void RelocalizationManagerNode::initialPoseCallback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  RCLCPP_INFO(
    this->get_logger(), "Received initial pose: [x: %f, y: %f, z: %f]", msg->pose.pose.position.x,
    msg->pose.pose.position.y, msg->pose.pose.position.z);

  Eigen::Isometry3d map_to_robot_base = Eigen::Isometry3d::Identity();
  map_to_robot_base.translation() << msg->pose.pose.position.x, msg->pose.pose.position.y,
    msg->pose.pose.position.z;
  map_to_robot_base.linear() = Eigen::Quaterniond(
                                 msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                                 msg->pose.pose.orientation.y, msg->pose.pose.orientation.z)
                                 .toRotationMatrix();

  try {
    const auto transform =
      tf_buffer_->lookupTransform(robot_base_frame_, current_scan_frame_id_, tf2::TimePointZero);
    const Eigen::Isometry3d robot_base_to_odom = tf2::transformToEigen(transform.transform);
    const Eigen::Isometry3d map_to_odom = map_to_robot_base * robot_base_to_odom;

    previous_map_to_odom_ = current_map_to_odom_ = map_to_odom;
    consecutive_gicp_failures_ = 0;
    resetAcceptedGicpVerificationStreak();
    setLocalizationState(LocalizationState::LOCALIZING, "initial pose reset");
    RCLCPP_INFO(
      this->get_logger(),
      "Initial pose reset localization state; the next registration cycle may refine map->odom.");
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(
      this->get_logger(), "Could not transform initial pose from %s to %s: %s",
      robot_base_frame_.c_str(), current_scan_frame_id_.c_str(), ex.what());
  }
}

}  // namespace relocalization_manager

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(relocalization_manager::RelocalizationManagerNode)
