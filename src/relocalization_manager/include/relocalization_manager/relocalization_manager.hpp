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

#ifndef RELOCALIZATION_MANAGER__RELOCALIZATION_MANAGER_HPP_
#define RELOCALIZATION_MANAGER__RELOCALIZATION_MANAGER_HPP_

#include <Eigen/Geometry>

#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "pcl/io/pcd_io.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "small_gicp/ann/kdtree_omp.hpp"
#include "small_gicp/factors/gicp_factor.hpp"
#include "small_gicp/pcl/pcl_point.hpp"
#include "small_gicp/registration/reduction_omp.hpp"
#include "small_gicp/registration/registration.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

namespace relocalization_manager
{

struct SmallGicpParams
{
  int num_threads{4};
  int num_neighbors{20};
  int max_iterations{10};
  float global_leaf_size{0.25F};
  float registered_leaf_size{0.25F};
  float max_dist_sq{1.0F};
};

struct VerificationResult
{
  bool accepted{false};
  Eigen::Isometry3d transform{Eigen::Isometry3d::Identity()};
};

class SmallGicpVerifier
{
public:
  SmallGicpVerifier(
    const rclcpp::Logger & logger, const SmallGicpParams & params,
    const std::vector<double> & prior_pcd_transform);

  bool loadGlobalMap(const std::string & file_name);
  void transformGlobalMap(const Eigen::Affine3d & transform, const std::string & label);
  VerificationResult verify(
    const pcl::PointCloud<pcl::PointXYZ> & accumulated_cloud,
    const Eigen::Isometry3d & initial_guess);

private:
  void prepareTarget();

  rclcpp::Logger logger_;
  SmallGicpParams params_;
  std::vector<double> prior_pcd_transform_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr global_map_;
  pcl::PointCloud<pcl::PointCovariance>::Ptr target_;
  std::shared_ptr<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>> target_tree_;
  std::shared_ptr<
    small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP>>
    register_;
};

class RelocalizationManagerNode : public rclcpp::Node
{
public:
  explicit RelocalizationManagerNode(const rclcpp::NodeOptions & options);

private:
  void registeredPcdCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void performRegistration();
  void publishTransform();
  void initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void maybeApplyLidarOffsetToPriorMap();

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcd_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;

  SmallGicpParams small_gicp_params_;
  std::vector<double> init_pose_;
  std::vector<double> prior_pcd_transform_;

  std::string map_frame_;
  std::string odom_frame_;
  std::string prior_pcd_file_;
  std::string base_frame_;
  std::string robot_base_frame_;
  std::string lidar_frame_;
  std::string current_scan_frame_id_;
  std::string input_cloud_topic_;
  std::string initial_pose_topic_;
  bool transform_prior_map_with_lidar_offset_;
  rclcpp::Time last_scan_time_;
  Eigen::Isometry3d current_map_to_odom_;
  Eigen::Isometry3d previous_map_to_odom_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr accumulated_cloud_;
  std::unique_ptr<SmallGicpVerifier> small_gicp_verifier_;

  rclcpp::TimerBase::SharedPtr transform_timer_;
  rclcpp::TimerBase::SharedPtr register_timer_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}  // namespace relocalization_manager

#endif  // RELOCALIZATION_MANAGER__RELOCALIZATION_MANAGER_HPP_
