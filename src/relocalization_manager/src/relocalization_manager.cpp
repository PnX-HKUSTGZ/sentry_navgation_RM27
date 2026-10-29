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

double rollFromIsometry(const Eigen::Isometry3d & transform)
{
  const auto rotation = transform.rotation();
  return std::atan2(rotation(2, 1), rotation(2, 2));
}

double pitchFromIsometry(const Eigen::Isometry3d & transform)
{
  const auto rotation = transform.rotation();
  return std::atan2(-rotation(2, 0), std::hypot(rotation(0, 0), rotation(1, 0)));
}

double initPoseZ(const std::vector<double> & init_pose)
{
  return init_pose.size() >= 3 ? init_pose[2] : 0.0;
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
    accumulated_cloud.size(), filtered_cloud.size(), retained_ratio, source->size(),
    source_min_pt.x, source_max_pt.x, source_min_pt.y, source_max_pt.y, source_min_pt.z,
    source_max_pt.z, initial_guess.translation().x(), initial_guess.translation().y(),
    initial_guess.translation().z());

  small_gicp::estimate_covariances_omp(*source, params_.num_neighbors, params_.num_threads);

  register_->reduction.num_threads = params_.num_threads;
  register_->rejector.max_dist_sq = params_.max_dist_sq;
  const int bounded_max_iterations = std::max(1, params_.max_iterations);
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
    !result.T_target_source.matrix().allFinite() || !std::isfinite(mean_error) ||
    !std::isfinite(inlier_ratio) || mean_error > params_.max_mean_error ||
    inlier_ratio < params_.min_inlier_ratio ||
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
    std::abs(delta_translation.x()) > params_.max_delta_xy ||
    std::abs(delta_translation.y()) > params_.max_delta_xy ||
    std::abs(delta_translation.z()) > params_.max_delta_z ||
    std::abs(delta_yaw) > params_.max_delta_yaw ||
    Eigen::AngleAxisd(initial_guess.rotation().transpose() * result.T_target_source.rotation())
        .angle() > params_.max_delta_yaw) {
    RCLCPP_WARN(
      logger_,
      "GICP converged but rejected by local pose delta gate: iterations=%zu error=%.2f "
      "mean_error=%.2f inliers_ratio=%zu/%zu ratio=%.3f "
      "max_iterations=%d delta=[dx=%.3f, dy=%.3f, dz=%.3f, dyaw=%.3f] "
      "limit=[xy=%.3f, z=%.3f, yaw=%.3f]",
      result.iterations, result.error, mean_error, result.num_inliers, source->size(), inlier_ratio,
      bounded_max_iterations, delta_translation.x(), delta_translation.y(), delta_translation.z(),
      delta_yaw, params_.max_delta_xy, params_.max_delta_z, params_.max_delta_yaw);
    return verification;
  }

  RCLCPP_INFO(
    logger_,
    "GICP converge successfully with local pose delta gate!: iterations=%zu error=%.2f "
    "mean_error=%.2f "
    "inliers_ratio=%zu/%zu ratio=%.3f "
    "max_iterations=%d delta=[dx=%.3f, dy=%.3f, dz=%.3f, dyaw=%.3f] "
    "limit=[xy=%.3f, z=%.3f, yaw=%.3f] "
    "initial_rpy=[%.6f, %.6f, %.6f] accepted_rpy=[%.6f, %.6f, %.6f]",
    result.iterations, result.error, mean_error, result.num_inliers, source->size(), inlier_ratio,
    bounded_max_iterations, delta_translation.x(), delta_translation.y(), delta_translation.z(),
    delta_yaw, params_.max_delta_xy, params_.max_delta_z, params_.max_delta_yaw,
    rollFromIsometry(initial_guess), pitchFromIsometry(initial_guess),
    yawFromIsometry(initial_guess), rollFromIsometry(result.T_target_source),
    pitchFromIsometry(result.T_target_source), yawFromIsometry(result.T_target_source));
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
  stationary_gate_.command(
    this->now().seconds(), std::hypot(msg->linear.x, msg->linear.y),
    std::hypot(std::hypot(msg->angular.x, msg->angular.y), msg->angular.z));
  updateMotionState();
}

void RelocalizationManagerNode::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  const auto & twist = msg->twist.twist;
  stationary_gate_.odometry(
    this->now().seconds(), rclcpp::Time(msg->header.stamp).seconds(),
    std::hypot(std::hypot(twist.linear.x, twist.linear.y), twist.linear.z),
    std::hypot(std::hypot(twist.angular.x, twist.angular.y), twist.angular.z),
    msg->header.frame_id == odom_frame_);
  updateMotionState();
}

void RelocalizationManagerNode::updateMotionState()
{
  const auto now = this->now();
  if (last_scan_time_ > now) {
    last_scan_time_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
    ++motion_epoch_;
    reset_accumulated_cloud_.store(true);
    resetAcceptedGicpVerificationStreak();
  }
  const bool moving = !stationary_gate_.stationary(now.seconds());
  const bool was_moving = robot_moving_.exchange(moving);
  if (moving != was_moving) {
    ++motion_epoch_;
    resetAcceptedGicpVerificationStreak();
    reset_accumulated_cloud_.store(true);
    if (moving) {
      setLocalizationState(LocalizationState::LOCALIZING, "motion or stale odometry detected");
    }
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
  wait_for_initial_pose_ = this->declare_parameter("wait_for_initial_pose", false);
  initialized_ = !wait_for_initial_pose_;
  enable_stationary_refinement_ = this->declare_parameter("enable_stationary_refinement", true);
  stationary_gate_.settle_time = this->declare_parameter("stationary_settle_time", 0.75);
  stationary_gate_.angular_threshold =
    this->declare_parameter("moving_angular_speed_threshold", 0.08);
  stationary_gate_.odometry_timeout = this->declare_parameter("odometry_timeout", 0.5);
  candidate_translation_tolerance_ =
    this->declare_parameter("candidate_translation_tolerance", 0.08);
  candidate_rotation_tolerance_ = this->declare_parameter("candidate_rotation_tolerance", 0.035);
  refinement_reference_frame_ = this->declare_parameter("refinement_reference_frame", "");
  max_reference_tilt_ = this->declare_parameter("max_reference_tilt", 0.009);
  const auto odometry_topic = this->declare_parameter("odometry_topic", "odometry");
  const auto ack_topic =
    this->declare_parameter("initial_pose_ack_topic", "relocalization/initialpose_accepted");
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
  if (wait_for_initial_pose_) {
    // RViz /initialpose selects the Gazebo spawn, not a localization seed.
    initial_pose_topic_ = "relocalization/initialpose";
    robot_base_frame_ = base_frame_;
    if (robot_base_frame_.empty()) {
      throw std::invalid_argument("Global initialization requires base_frame");
    }
    // Keep parameter introspection consistent with the effective global handoff contract.
    this->set_parameters(
      {rclcpp::Parameter("initial_pose_topic", initial_pose_topic_),
       rclcpp::Parameter("robot_base_frame", robot_base_frame_)});
  }
  this->get_parameter("moving_linear_speed_threshold", moving_linear_speed_threshold_);
  this->get_parameter("motion_command_timeout", motion_command_timeout_);
  this->get_parameter(
    "required_consecutive_gicp_acceptances", required_consecutive_gicp_acceptances_);
  this->get_parameter(
    "transform_prior_map_with_lidar_offset", transform_prior_map_with_lidar_offset_);
  required_consecutive_gicp_acceptances_ = std::max(1, required_consecutive_gicp_acceptances_);
  if (!std::isfinite(moving_linear_speed_threshold_) || moving_linear_speed_threshold_ < 0.0) {
    moving_linear_speed_threshold_ = 0.05;
  }
  if (!std::isfinite(motion_command_timeout_) || motion_command_timeout_ <= 0.0) {
    motion_command_timeout_ = 0.5;
  }

  stationary_gate_.linear_threshold = moving_linear_speed_threshold_;
  if (
    !stationary_gate_.validParameters() || !std::isfinite(candidate_translation_tolerance_) ||
    candidate_translation_tolerance_ <= 0.0 || !std::isfinite(candidate_rotation_tolerance_) ||
    candidate_rotation_tolerance_ <= 0.0 || !std::isfinite(max_reference_tilt_) ||
    max_reference_tilt_ <= 0.0) {
    throw std::invalid_argument("Invalid stationary relocalization thresholds");
  }

  if (!init_pose_.empty() && init_pose_.size() >= 6) {
    current_map_to_odom_ = poseVectorToIsometry(init_pose_);
  }
  previous_map_to_odom_ = current_map_to_odom_;

  last_scan_time_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
  accumulated_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock(), tf2::durationFromSec(60.0));
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
  prior_pcd_map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "prior_pcd_map", rclcpp::QoS(1).transient_local().reliable());
  state_log_timer_ = this->create_wall_timer(
    std::chrono::seconds(5), std::bind(&RelocalizationManagerNode::logLocalizationState, this));

  small_gicp_verifier_ = std::make_unique<SmallGicpVerifier>(
    this->get_logger(), small_gicp_params_, prior_pcd_transform_);
  if (small_gicp_verifier_->loadGlobalMap(prior_pcd_file_)) {
    maybeApplyLidarOffsetToPriorMap();
    publishPriorPcdMap(small_gicp_verifier_->globalMap());
  }

  pcd_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    input_cloud_topic_, rclcpp::SensorDataQoS().keep_last(1),
    std::bind(&RelocalizationManagerNode::registeredPcdCallback, this, std::placeholders::_1));

  motion_command_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    motion_command_topic_, rclcpp::QoS(10),
    std::bind(&RelocalizationManagerNode::motionCommandCallback, this, std::placeholders::_1));

  odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    odometry_topic, rclcpp::SensorDataQoS().keep_last(5),
    std::bind(&RelocalizationManagerNode::odometryCallback, this, std::placeholders::_1));
  initial_pose_ack_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    ack_topic, rclcpp::QoS(1).transient_local().reliable());

  initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    initial_pose_topic_, 10,
    std::bind(&RelocalizationManagerNode::initialPoseCallback, this, std::placeholders::_1));

  register_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&RelocalizationManagerNode::performRegistration, this));

  transform_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(50), std::bind(&RelocalizationManagerNode::publishTransform, this));
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

void RelocalizationManagerNode::registeredPcdCallback(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  updateMotionState();
  if (reset_accumulated_cloud_.exchange(false)) {
    accumulated_cloud_->clear();
  }
  const rclcpp::Time stamp(msg->header.stamp, this->get_clock()->get_clock_type());
  const double age = (this->now() - stamp).seconds();
  if (
    !initialized_ || !enable_stationary_refinement_ || isRobotMoving() || isLocalized() ||
    msg->header.frame_id != odom_frame_ || age < 0.0 || age > 0.5 ||
    stamp.seconds() < stationary_gate_.settledSince() || stamp <= last_scan_time_) {
    return;
  }
  last_scan_time_ = stamp;
  current_scan_frame_id_ = msg->header.frame_id;
  pcl::PointCloud<pcl::PointXYZ> scan;
  pcl::fromROSMsg(*msg, scan);
  // Bound memory if registration is slower than the incoming lidar stream.
  if (accumulated_cloud_->size() + scan.size() > 300000) {
    accumulated_cloud_->clear();
  }
  if (scan.size() <= 300000) {
    *accumulated_cloud_ += scan;
  }
}

void RelocalizationManagerNode::performRegistration()
{
  updateMotionState();
  if (reset_accumulated_cloud_.exchange(false)) {
    accumulated_cloud_->clear();
  }

  if (registration_future_.valid()) {
    if (registration_future_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
      return;
    }
    VerificationResult verification;
    try {
      verification = registration_future_.get();
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(this->get_logger(), "Stationary GICP failed: %s", ex.what());
    }
    // Motion callbacks and TF publication keep running while GICP works. Never apply
    // an old result after motion, a pose reset, a clock jump, or a sensor outage.
    const double result_age = this->now().seconds() - registration_scan_time_;
    if (
      !initialized_ || isRobotMoving() || registration_epoch_ != motion_epoch_ ||
      result_age < 0.0 || result_age > 2.0 || !verification.accepted) {
      resetAcceptedGicpVerificationStreak();
      accumulated_cloud_->clear();
      return;
    }
    if (!refinement_reference_frame_.empty()) {
      try {
        const auto reference_tf =
          tf_buffer_->lookupTransform(odom_frame_, refinement_reference_frame_, tf2::TimePointZero);
        const Eigen::Isometry3d reference = tf2::transformToEigen(reference_tf.transform);
        const Eigen::Isometry3d map_to_reference = verification.transform * reference;
        const double roll = rollFromIsometry(map_to_reference);
        const double pitch = pitchFromIsometry(map_to_reference);
        const auto reference_stamp = rclcpp::Time(reference_tf.header.stamp);
        const double age = (this->now() - reference_stamp).seconds();
        if (
          !std::isfinite(roll) || !std::isfinite(pitch) ||
          (reference_stamp.nanoseconds() != 0 && (age < -0.1 || age > 0.5)) ||
          std::abs(roll) > max_reference_tilt_ || std::abs(pitch) > max_reference_tilt_) {
          RCLCPP_WARN(
            this->get_logger(),
            "Stationary GICP rejected by navigation tilt/TF gate: frame=%s roll=%.5f "
            "pitch=%.5f limit=%.5f; retaining committed map->odom",
            refinement_reference_frame_.c_str(), roll, pitch, max_reference_tilt_);
          resetAcceptedGicpVerificationStreak();
          accumulated_cloud_->clear();
          return;
        }
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN(this->get_logger(), "Stationary GICP reference TF unavailable: %s", ex.what());
        resetAcceptedGicpVerificationStreak();
        accumulated_cloud_->clear();
        return;
      }
    }
    const auto candidate_delta = previous_map_to_odom_.inverse() * verification.transform;
    if (
      consecutive_gicp_acceptances_ > 0 &&
      (candidate_delta.translation().norm() > candidate_translation_tolerance_ ||
       Eigen::AngleAxisd(candidate_delta.rotation()).angle() > candidate_rotation_tolerance_)) {
      resetAcceptedGicpVerificationStreak();
    }
    previous_map_to_odom_ = verification.transform;
    recordAcceptedGicpVerification("stationary GICP consensus");
    if (isLocalized()) {
      const auto delta = current_map_to_odom_.inverse() * verification.transform;
      current_map_to_odom_ = verification.transform;
      publishTransform();
      RCLCPP_INFO(
        this->get_logger(),
        "STATIONARY_CORRECTION committed: translation=%.4f rotation=%.4f confirmations=%d",
        delta.translation().norm(), Eigen::AngleAxisd(delta.rotation()).angle(),
        consecutive_gicp_acceptances_);
    }
    // Each confirmation must use a new scan window, not a duplicate buffered cloud.
    accumulated_cloud_->clear();
    return;
  }

  if (
    !initialized_ || !enable_stationary_refinement_ || !small_gicp_verifier_ || isRobotMoving() ||
    isLocalized() || accumulated_cloud_->empty()) {
    accumulated_cloud_->clear();
    return;
  }
  if ((this->now() - last_scan_time_).seconds() > 0.5) {
    resetAcceptedGicpVerificationStreak();
    accumulated_cloud_->clear();
    return;
  }
  auto cloud = accumulated_cloud_;
  accumulated_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  registration_epoch_ = motion_epoch_;
  registration_scan_time_ = last_scan_time_.seconds();
  // All candidates are bounded against the committed transform, not the previous
  // candidate: several individually small corrections cannot walk past the gate.
  const auto anchor = current_map_to_odom_;
  registration_future_ = std::async(std::launch::async, [this, cloud, anchor]() {
    return small_gicp_verifier_->verify(*cloud, anchor);
  });
}

void RelocalizationManagerNode::publishTransform()
{
  if (!initialized_ || current_map_to_odom_.matrix().isZero()) {
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
  if (wait_for_initial_pose_ && accepted_initial_pose_) {
    if (*msg == *accepted_initial_pose_) {
      initial_pose_ack_pub_->publish(*accepted_initial_pose_);
    }
    return;
  }
  const auto & pose = msg->pose.pose;
  Eigen::Quaterniond rotation(
    pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
  const Eigen::Vector3d translation(pose.position.x, pose.position.y, pose.position.z);
  if (
    msg->header.frame_id != map_frame_ || !translation.allFinite() ||
    !rotation.coeffs().allFinite() || std::abs(rotation.norm() - 1.0) > 1e-3 ||
    msg->header.stamp.sec < 0 || msg->header.stamp.nanosec >= 1000000000 ||
    (wait_for_initial_pose_ && msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0)) {
    RCLCPP_WARN(this->get_logger(), "Rejected invalid initial pose frame, stamp or geometry");
    return;
  }
  Eigen::Isometry3d map_to_robot_base = Eigen::Isometry3d::Identity();
  map_to_robot_base.translation() = translation;
  map_to_robot_base.linear() = rotation.normalized().toRotationMatrix();
  try {
    // Use exactly the global match timestamp. A later pose (during the scan spin)
    // would introduce a heading/translation error into every subsequent goal.
    const auto transform = tf_buffer_->lookupTransform(
      odom_frame_, robot_base_frame_, rclcpp::Time(msg->header.stamp),
      rclcpp::Duration::from_seconds(0.0));
    const Eigen::Isometry3d odom_to_robot_base = tf2::transformToEigen(transform.transform);
    previous_map_to_odom_ = current_map_to_odom_ = map_to_robot_base * odom_to_robot_base.inverse();
    ++motion_epoch_;
    initialized_ = true;
    accepted_initial_pose_ = msg;
    resetAcceptedGicpVerificationStreak();
    accumulated_cloud_->clear();
    setLocalizationState(LocalizationState::LOCALIZING, "timestamped initial pose accepted");
    publishTransform();
    initial_pose_ack_pub_->publish(*msg);
    RCLCPP_INFO(
      this->get_logger(), "Global pose accepted; stationary GICP refinement is %s",
      enable_stationary_refinement_ ? "enabled" : "disabled");
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Waiting for initial-pose timestamp TF %s <- %s: %s", odom_frame_.c_str(),
      robot_base_frame_.c_str(), ex.what());
  }
}

}  // namespace relocalization_manager

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(relocalization_manager::RelocalizationManagerNode)
