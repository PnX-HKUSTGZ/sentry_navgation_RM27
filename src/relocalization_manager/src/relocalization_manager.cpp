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
#include <limits>
#include <rclcpp/logging.hpp>

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

void SmallGicpVerifier::prepareTarget()
{
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

  auto source = small_gicp::voxelgrid_sampling_omp<
    pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(
    accumulated_cloud, params_.registered_leaf_size);

  if (!source || source->empty()) {
    RCLCPP_WARN(logger_, "Downsampled source cloud is empty.");
    return verification;
  }

  pcl::PointXYZ source_min_pt;
  pcl::PointXYZ source_max_pt;
  pcl::getMinMax3D(accumulated_cloud, source_min_pt, source_max_pt);
  RCLCPP_INFO(
    logger_,
    "Verifying accumulated cloud: raw_points=%zu downsampled_points=%zu "
    "bbox x=[%.3f, %.3f] y=[%.3f, %.3f] z=[%.3f, %.3f] init_xyz=[%.3f, %.3f, %.3f]",
    accumulated_cloud.size(), source->size(), source_min_pt.x, source_max_pt.x, source_min_pt.y,
    source_max_pt.y, source_min_pt.z, source_max_pt.z, initial_guess.translation().x(),
    initial_guess.translation().y(), initial_guess.translation().z());

  small_gicp::estimate_covariances_omp(*source, params_.num_neighbors, params_.num_threads);

  register_->reduction.num_threads = params_.num_threads;
  register_->rejector.max_dist_sq = params_.max_dist_sq;
  register_->optimizer.max_iterations = params_.max_iterations;

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

  if (!result.converged) {
    RCLCPP_WARN(
      logger_,
      "GICP did not converge: iterations=%zu error=%.6f mean_error=%.6f "
      "inliers_ratio=%zu/%zu ratio=%.3f",
      result.iterations, result.error, mean_error, result.num_inliers, source->size(),
      inlier_ratio);
    return verification;
  }

  const Eigen::Vector3d delta_translation =
    result.T_target_source.translation() - initial_guess.translation();
  const double delta_yaw =
    normalizeAngle(yawFromIsometry(result.T_target_source) - yawFromIsometry(initial_guess));

  if (
    std::abs(delta_translation.x()) > params_.max_delta_xy ||
    std::abs(delta_translation.y()) > params_.max_delta_xy ||
    std::abs(delta_translation.z()) > params_.max_delta_z ||
    std::abs(delta_yaw) > params_.max_delta_yaw) {
    RCLCPP_WARN(
      logger_,
      "GICP converged but rejected by pose jump gate: iterations=%zu error=%.6f "
      "mean_error=%.6f inliers_ratio=%zu/%zu ratio=%.3f "
      "delta=[dx=%.3f, dy=%.3f, dz=%.3f, dyaw=%.3f] ",
      result.iterations, result.error, mean_error, result.num_inliers, source->size(), inlier_ratio,
      delta_translation.x(), delta_translation.y(), delta_translation.z(), delta_yaw);
    return verification;
  }

  RCLCPP_INFO(
    logger_,
    "GICP converge successfully!: iterations=%zu error=%.6f mean_error=%.6f "
    "inliers_ratio=%zu/%zu ratio=%.3f "
    "delta=[dx=%.3f, dy=%.3f, dz=%.3f, dyaw=%.3f]",
    result.iterations, result.error, mean_error, result.num_inliers, source->size(), inlier_ratio,
    delta_translation.x(), delta_translation.y(), delta_translation.z(), delta_yaw);
  verification.accepted = true;
  verification.transform = result.T_target_source;
  return verification;
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
  this->declare_parameter("transform_prior_map_with_lidar_offset", false);
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
  this->declare_parameter("scan_context_yaw_delta_sign", 1.0);

  this->get_parameter("num_threads", small_gicp_params_.num_threads);
  this->get_parameter("num_neighbors", small_gicp_params_.num_neighbors);
  this->get_parameter("max_iterations", small_gicp_params_.max_iterations);
  this->get_parameter("global_leaf_size", small_gicp_params_.global_leaf_size);
  this->get_parameter("registered_leaf_size", small_gicp_params_.registered_leaf_size);
  this->get_parameter("max_dist_sq", small_gicp_params_.max_dist_sq);
  this->get_parameter("max_delta_xy", small_gicp_params_.max_delta_xy);
  this->get_parameter("max_delta_z", small_gicp_params_.max_delta_z);
  this->get_parameter("max_delta_yaw", small_gicp_params_.max_delta_yaw);
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
  this->get_parameter(
    "transform_prior_map_with_lidar_offset", transform_prior_map_with_lidar_offset_);
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
  this->get_parameter("scan_context_yaw_delta_sign", scan_context_yaw_delta_sign_);

  scan_context_params_.mode = lowerCopy(scan_context_params_.mode);
  scan_context_params_.build_source = lowerCopy(scan_context_params_.build_source);
  scan_context_failure_trigger_count_ = std::max(1, scan_context_failure_trigger_count_);
  scan_context_max_gicp_candidates_ = std::max(1, scan_context_max_gicp_candidates_);
  if (!std::isfinite(scan_context_yaw_delta_sign_) || scan_context_yaw_delta_sign_ == 0.0) {
    scan_context_yaw_delta_sign_ = 1.0;
  }
  scan_context_yaw_delta_sign_ = scan_context_yaw_delta_sign_ > 0.0 ? 1.0 : -1.0;

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
    scan_context_startup_query_pending_.store(scan_context_query_on_startup_);
    RCLCPP_INFO(
      this->get_logger(),
      "Scan Context query mode is active: query_on_startup=%s query_on_gicp_failure=%s "
      "failure_trigger_count=%d max_gicp_candidates=%d yaw_delta_sign=%.0f",
      scan_context_query_on_startup_ ? "true" : "false",
      scan_context_query_on_gicp_failure_ ? "true" : "false", scan_context_failure_trigger_count_,
      scan_context_max_gicp_candidates_, scan_context_yaw_delta_sign_);
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
      rclcpp::Duration::from_seconds(0.0));
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

bool RelocalizationManagerNode::makeMapToOdomGuessFromScanContextCandidate(
  const ::scan_context::ScanContextCandidate & candidate, const rclcpp::Time & stamp,
  Eigen::Isometry3d & map_to_odom_guess, Eigen::Isometry3d & map_to_base_estimate)
{
  if (odom_frame_.empty() || scan_context_params_.input_frame.empty()) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "Cannot make Scan Context GICP guess because odom_frame or scan_context_input_frame is "
      "empty.");
    return false;
  }

  if (!makeMapToBaseEstimateFromScanContextCandidate(candidate, map_to_base_estimate)) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "Scan Context candidate produced a non-finite map->base estimate.");
    return false;
  }

  geometry_msgs::msg::TransformStamped odom_to_base_msg;
  try {
    odom_to_base_msg = tf_buffer_->lookupTransform(
      odom_frame_, scan_context_params_.input_frame, stamp, rclcpp::Duration::from_seconds(0.0));
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

  const Eigen::Isometry3d odom_to_base = tf2::transformToEigen(odom_to_base_msg.transform);
  map_to_odom_guess = map_to_base_estimate * odom_to_base.inverse();
  return map_to_odom_guess.matrix().allFinite();
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

  RCLCPP_INFO(
    this->get_logger(),
    "Trying Scan Context relocalization: reason=%s candidates=%zu max_gicp_candidates=%d",
    reason.c_str(), candidates.size(), max_candidates);

  for (int i = 0; i < max_candidates; ++i) {
    const auto & candidate = candidates[static_cast<std::size_t>(i)];
    Eigen::Isometry3d map_to_base_estimate = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d map_to_odom_guess = Eigen::Isometry3d::Identity();
    if (!makeMapToOdomGuessFromScanContextCandidate(
          candidate, stamp, map_to_odom_guess, map_to_base_estimate)) {
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

    const VerificationResult verification =
      small_gicp_verifier_->verify(accumulated_cloud, map_to_odom_guess);
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
      "valid_candidates=%zu",
      reason.c_str(), valid_candidates.size());
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
  last_scan_time_ = msg->header.stamp;
  current_scan_frame_id_ = msg->header.frame_id;

  pcl::PointCloud<pcl::PointXYZ>::Ptr scan(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*msg, *scan);
  queueScanContextInput(msg->header, scan);
  if (small_gicp_verifier_) {
    *accumulated_cloud_ += *scan;
  }
}

void RelocalizationManagerNode::performRegistration()
{
  if (!small_gicp_verifier_) {
    return;
  }

  if (accumulated_cloud_->empty()) {
    return;
  }

  const bool manual_scan_context_query = force_scan_context_query_once_.exchange(false);
  const bool startup_scan_context_query = scan_context_startup_query_pending_.exchange(false);
  const bool force_scan_context_query = manual_scan_context_query || startup_scan_context_query;
  if (force_scan_context_query) {
    VerificationResult scan_context_verification;
    const std::string reason = manual_scan_context_query ? "manual_trigger" : "startup";
    if (verifyWithScanContextCandidates(*accumulated_cloud_, reason, scan_context_verification)) {
      current_map_to_odom_ = scan_context_verification.transform;
      previous_map_to_odom_ = scan_context_verification.transform;
      consecutive_gicp_failures_ = 0;
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

  const VerificationResult verification =
    small_gicp_verifier_->verify(*accumulated_cloud_, previous_map_to_odom_);

  if (verification.accepted) {
    current_map_to_odom_ = verification.transform;
    previous_map_to_odom_ = verification.transform;
    consecutive_gicp_failures_ = 0;
    accumulated_cloud_->clear();
    return;
  }

  ++consecutive_gicp_failures_;
  if (
    scan_context_query_on_gicp_failure_ && isScanContextQueryMode(scan_context_params_) &&
    consecutive_gicp_failures_ >= scan_context_failure_trigger_count_) {
    VerificationResult scan_context_verification;
    if (verifyWithScanContextCandidates(
          *accumulated_cloud_, "gicp_failure", scan_context_verification)) {
      current_map_to_odom_ = scan_context_verification.transform;
      previous_map_to_odom_ = scan_context_verification.transform;
      consecutive_gicp_failures_ = 0;
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
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(
      this->get_logger(), "Could not transform initial pose from %s to %s: %s",
      robot_base_frame_.c_str(), current_scan_frame_id_.c_str(), ex.what());
  }
}

}  // namespace relocalization_manager

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(relocalization_manager::RelocalizationManagerNode)
