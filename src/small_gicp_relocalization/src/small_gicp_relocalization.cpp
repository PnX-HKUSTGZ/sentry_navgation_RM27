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

#include "small_gicp_relocalization/small_gicp_relocalization.hpp"

#include <algorithm>
#include <cmath>

#include "pcl/common/common.h"
#include "pcl/common/transforms.h"
#include "pcl_conversions/pcl_conversions.h"
#include "small_gicp/pcl/pcl_registration.hpp"
#include "small_gicp/util/downsampling_omp.hpp"
#include "tf2_eigen/tf2_eigen.hpp"

namespace small_gicp_relocalization
{
namespace
{

Eigen::Affine3d poseVectorToAffine(const std::vector<double> & pose)
{
  Eigen::Affine3d transform = Eigen::Affine3d::Identity();
  if (pose.size() < 6) {
    return transform;
  }

  transform.translation() << pose[0], pose[1], pose[2];
  transform.linear() =
    Eigen::AngleAxisd(pose[5], Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(pose[4], Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(pose[3], Eigen::Vector3d::UnitX()).toRotationMatrix();
  return transform;
}

bool isIdentityPoseVector(const std::vector<double> & pose)
{
  if (pose.size() < 6) {
    return true;
  }

  return std::all_of(
    pose.begin(), pose.begin() + 6, [](double value) { return std::abs(value) < 1e-9; });
}

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

}  // namespace

SmallGicpRelocalizationNode::SmallGicpRelocalizationNode(const rclcpp::NodeOptions & options)
: Node("small_gicp_relocalization", options),
  result_t_(Eigen::Isometry3d::Identity()),
  previous_result_t_(Eigen::Isometry3d::Identity())
{
  this->declare_parameter("num_threads", 4);
  this->declare_parameter("num_neighbors", 20);
  this->declare_parameter("max_iterations", 10);
  this->declare_parameter("global_leaf_size", 0.25);
  this->declare_parameter("registered_leaf_size", 0.25);
  this->declare_parameter("max_dist_sq", 1.0);
  this->declare_parameter("map_frame", "map");
  this->declare_parameter("odom_frame", "odom");
  this->declare_parameter("base_frame", "");
  this->declare_parameter("robot_base_frame", "");
  this->declare_parameter("lidar_frame", "");
  this->declare_parameter("prior_pcd_file", "");
  this->declare_parameter("init_pose", std::vector<double>{0., 0., 0., 0., 0., 0.});
  this->declare_parameter("prior_pcd_transform", std::vector<double>{0., 0., 0., 0., 0., 0.});
  this->declare_parameter("input_cloud_topic", "registered_scan");
  this->declare_parameter("transform_prior_map_with_lidar_offset", false);

  this->get_parameter("num_threads", num_threads_);
  this->get_parameter("num_neighbors", num_neighbors_);
  this->get_parameter("max_iterations", max_iterations_);
  this->get_parameter("global_leaf_size", global_leaf_size_);
  this->get_parameter("registered_leaf_size", registered_leaf_size_);
  this->get_parameter("max_dist_sq", max_dist_sq_);
  this->get_parameter("map_frame", map_frame_);
  this->get_parameter("odom_frame", odom_frame_);
  this->get_parameter("base_frame", base_frame_);
  this->get_parameter("robot_base_frame", robot_base_frame_);
  this->get_parameter("lidar_frame", lidar_frame_);
  this->get_parameter("prior_pcd_file", prior_pcd_file_);
  this->get_parameter("init_pose", init_pose_);
  this->get_parameter("prior_pcd_transform", prior_pcd_transform_);
  this->get_parameter("input_cloud_topic", input_cloud_topic_);
  this->get_parameter(
    "transform_prior_map_with_lidar_offset", transform_prior_map_with_lidar_offset_);

  // [x, y, z, roll, pitch, yaw] - init_pose parameters
  if (!init_pose_.empty() && init_pose_.size() >= 6) {
    result_t_.translation() << init_pose_[0], init_pose_[1], init_pose_[2];
    result_t_.linear() =
      Eigen::AngleAxisd(init_pose_[5], Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(init_pose_[4], Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(init_pose_[3], Eigen::Vector3d::UnitX()).toRotationMatrix();
  }
  previous_result_t_ = result_t_;

  accumulated_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  global_map_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  register_ = std::make_shared<
    small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP>>();

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

  loadGlobalMap(prior_pcd_file_);

  // Downsample points and convert them into pcl::PointCloud<pcl::PointCovariance>
  target_ = small_gicp::voxelgrid_sampling_omp<
    pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(
    *global_map_, global_leaf_size_);

  // Estimate covariances of points
  small_gicp::estimate_covariances_omp(*target_, num_neighbors_, num_threads_);

  // Create KdTree for target
  target_tree_ = std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(
    target_, small_gicp::KdTreeBuilderOMP(num_threads_));

  pcd_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    input_cloud_topic_, 10,
    std::bind(&SmallGicpRelocalizationNode::registeredPcdCallback, this, std::placeholders::_1));

  initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "initialpose", 10,
    std::bind(&SmallGicpRelocalizationNode::initialPoseCallback, this, std::placeholders::_1));

  register_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500),  // 2 Hz
    std::bind(&SmallGicpRelocalizationNode::performRegistration, this));

  transform_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(50),  // 20 Hz
    std::bind(&SmallGicpRelocalizationNode::publishTransform, this));
}

void SmallGicpRelocalizationNode::loadGlobalMap(const std::string & file_name)
{
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_name, *global_map_) == -1) {
    RCLCPP_ERROR(this->get_logger(), "Couldn't read PCD file: %s", file_name.c_str());
    return;
  }
  RCLCPP_INFO(this->get_logger(), "Loaded global map with %zu points", global_map_->points.size());
  logCloudBounds(this->get_logger(), "Prior PCD raw", *global_map_);

  if (!isIdentityPoseVector(prior_pcd_transform_)) {
    const Eigen::Affine3d prior_pcd_transform = poseVectorToAffine(prior_pcd_transform_);
    pcl::transformPointCloud(*global_map_, *global_map_, prior_pcd_transform);
    RCLCPP_INFO_STREAM(
      this->get_logger(), "Applied prior_pcd_transform xyz/rpy = ["
                            << prior_pcd_transform_[0] << ", " << prior_pcd_transform_[1] << ", "
                            << prior_pcd_transform_[2] << ", " << prior_pcd_transform_[3] << ", "
                            << prior_pcd_transform_[4] << ", " << prior_pcd_transform_[5] << "]");
    logCloudBounds(this->get_logger(), "Prior PCD transformed", *global_map_);
  }

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

  // NOTE: Transform global pcd_map (based on `lidar_odom` frame) to the `odom` frame
  Eigen::Affine3d odom_to_lidar_odom;
  while (true) {
    try {
      auto tf_stamped = tf_buffer_->lookupTransform(
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
  pcl::transformPointCloud(*global_map_, *global_map_, odom_to_lidar_odom);
  logCloudBounds(this->get_logger(), "Prior PCD after lidar offset", *global_map_);
}

void SmallGicpRelocalizationNode::registeredPcdCallback(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  last_scan_time_ = msg->header.stamp;
  current_scan_frame_id_ = msg->header.frame_id;

  pcl::PointCloud<pcl::PointXYZ>::Ptr scan(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*msg, *scan);
  *accumulated_cloud_ += *scan;
}

void SmallGicpRelocalizationNode::performRegistration()
{
  if (accumulated_cloud_->empty()) {
    RCLCPP_WARN(this->get_logger(), "No accumulated points to process.");
    return;
  }

  source_ = small_gicp::voxelgrid_sampling_omp<
    pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(
    *accumulated_cloud_, registered_leaf_size_);

  pcl::PointXYZ source_min_pt;
  pcl::PointXYZ source_max_pt;
  pcl::getMinMax3D(*accumulated_cloud_, source_min_pt, source_max_pt);
  RCLCPP_INFO_THROTTLE(
    this->get_logger(), *this->get_clock(), 5000,
    "Registering accumulated cloud: raw_points=%zu downsampled_points=%zu "
    "bbox x=[%.3f, %.3f] y=[%.3f, %.3f] z=[%.3f, %.3f] init_xyz=[%.3f, %.3f, %.3f]",
    accumulated_cloud_->size(), source_->size(), source_min_pt.x, source_max_pt.x,
    source_min_pt.y, source_max_pt.y, source_min_pt.z, source_max_pt.z,
    previous_result_t_.translation().x(), previous_result_t_.translation().y(),
    previous_result_t_.translation().z());

  small_gicp::estimate_covariances_omp(*source_, num_neighbors_, num_threads_);

  source_tree_ = std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(
    source_, small_gicp::KdTreeBuilderOMP(num_threads_));

  if (!source_ || !source_tree_) {
    return;
  }

  register_->reduction.num_threads = num_threads_;
  register_->rejector.max_dist_sq = max_dist_sq_;
  register_->optimizer.max_iterations = max_iterations_;

  auto result = register_->align(*target_, *source_, *target_tree_, previous_result_t_);

  if (result.converged) {
    result_t_ = previous_result_t_ = result.T_target_source;
  } else {
    RCLCPP_WARN(this->get_logger(), "GICP did not converge.");
  }

  accumulated_cloud_->clear();
}

void SmallGicpRelocalizationNode::publishTransform()
{
  if (result_t_.matrix().isZero()) {
    return;
  }

  geometry_msgs::msg::TransformStamped transform_stamped;
  // Keep the initial map->odom transform usable before the first registered scan arrives.
  transform_stamped.header.stamp = this->now() + rclcpp::Duration::from_seconds(0.1);
  transform_stamped.header.frame_id = map_frame_;
  transform_stamped.child_frame_id = odom_frame_;

  const Eigen::Vector3d translation = result_t_.translation();
  const Eigen::Quaterniond rotation(result_t_.rotation());

  transform_stamped.transform.translation.x = translation.x();
  transform_stamped.transform.translation.y = translation.y();
  transform_stamped.transform.translation.z = translation.z();
  transform_stamped.transform.rotation.x = rotation.x();
  transform_stamped.transform.rotation.y = rotation.y();
  transform_stamped.transform.rotation.z = rotation.z();
  transform_stamped.transform.rotation.w = rotation.w();

  tf_broadcaster_->sendTransform(transform_stamped);
}

void SmallGicpRelocalizationNode::initialPoseCallback(
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
    auto transform =
      tf_buffer_->lookupTransform(robot_base_frame_, current_scan_frame_id_, tf2::TimePointZero);
    Eigen::Isometry3d robot_base_to_odom = tf2::transformToEigen(transform.transform);
    Eigen::Isometry3d map_to_odom = map_to_robot_base * robot_base_to_odom;

    previous_result_t_ = result_t_ = map_to_odom;
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(
      this->get_logger(), "Could not transform initial pose from %s to %s: %s",
      robot_base_frame_.c_str(), current_scan_frame_id_.c_str(), ex.what());
  }
}

}  // namespace small_gicp_relocalization

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(small_gicp_relocalization::SmallGicpRelocalizationNode)
