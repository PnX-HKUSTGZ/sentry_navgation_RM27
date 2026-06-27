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
#include <cmath>
#include <limits>
#include <rclcpp/logging.hpp>

#include "pcl/common/common.h"
#include "pcl/common/transforms.h"
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
  const Eigen::Quaterniond rotation =
    Eigen::AngleAxisd(pose[5], Eigen::Vector3d::UnitZ()) *
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
    logger, "%s cloud: points=%zu bbox x=[%.3f, %.3f] y=[%.3f, %.3f] z=[%.3f, %.3f]",
    label.c_str(), cloud.size(), min_pt.x, max_pt.x, min_pt.y, max_pt.y, min_pt.z, max_pt.z);
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
  } else{
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

  const double inlier_ratio =
    source->empty() ? 0.0 : static_cast<double>(result.num_inliers) / source->size();  //内点比列，越高越好
  const double mean_error =
    result.num_inliers == 0 ? std::numeric_limits<double>::infinity()
                          : result.error / result.num_inliers;  //平均误差
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
    std::abs(delta_yaw) > params_.max_delta_yaw)
  {
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
    result.iterations, result.error, mean_error, result.num_inliers, source->size(),
    inlier_ratio, delta_translation.x(), delta_translation.y(), delta_translation.z(), delta_yaw);
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

  if (!init_pose_.empty() && init_pose_.size() >= 6) {
    current_map_to_odom_ = poseVectorToIsometry(init_pose_);
  }
  previous_map_to_odom_ = current_map_to_odom_;

  accumulated_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

  small_gicp_verifier_ = std::make_unique<SmallGicpVerifier>(
    this->get_logger(), small_gicp_params_, prior_pcd_transform_);
  small_gicp_verifier_->loadGlobalMap(prior_pcd_file_);
  maybeApplyLidarOffsetToPriorMap();

  pcd_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    input_cloud_topic_, 10,
    std::bind(&RelocalizationManagerNode::registeredPcdCallback, this, std::placeholders::_1));

  initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    initial_pose_topic_, 10,
    std::bind(&RelocalizationManagerNode::initialPoseCallback, this, std::placeholders::_1));

  register_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500), std::bind(&RelocalizationManagerNode::performRegistration, this));

  transform_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(50), std::bind(&RelocalizationManagerNode::publishTransform, this));
}

void RelocalizationManagerNode::maybeApplyLidarOffsetToPriorMap()
{
  if (!transform_prior_map_with_lidar_offset_) {
    RCLCPP_INFO(
      this->get_logger(),
      "Using prior PCD as loaded; no base->lidar mounting offset is applied.");
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
  last_scan_time_ = msg->header.stamp;
  current_scan_frame_id_ = msg->header.frame_id;

  pcl::PointCloud<pcl::PointXYZ>::Ptr scan(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*msg, *scan);
  *accumulated_cloud_ += *scan;
}

void RelocalizationManagerNode::performRegistration()
{
  const VerificationResult verification =
    small_gicp_verifier_->verify(*accumulated_cloud_, previous_map_to_odom_);
  
  if (verification.accepted) {
    current_map_to_odom_ = verification.transform;
    previous_map_to_odom_ = verification.transform;
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
