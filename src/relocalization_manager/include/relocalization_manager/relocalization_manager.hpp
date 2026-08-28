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
#include <atomic>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "pcl/io/pcd_io.h"
#include "pcl/kdtree/kdtree_flann.h"
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
  double max_mean_error{0.30};
  double min_inlier_ratio{0.75};
  int min_inliers{1000};
  double prior_neighbor_max_distance{0.5};
  bool filter_source_by_prior_map{true};
  double max_delta_xy{0.25};
  double max_delta_z{0.15};
  double max_delta_yaw{0.17453292519943295};
};

struct VerificationResult
{
  bool accepted{false};
  bool converged{false};
  Eigen::Isometry3d transform{Eigen::Isometry3d::Identity()};
  double error{std::numeric_limits<double>::infinity()};
  double mean_error{std::numeric_limits<double>::infinity()};
  double inlier_ratio{0.0};
  std::size_t num_inliers{0};
  std::size_t source_points{0};
  std::size_t iterations{0};
};

enum class LocalizationState : std::uint8_t { LOCALIZING, LOCALIZED };

class SmallGicpVerifier
{
public:
  SmallGicpVerifier(
    const rclcpp::Logger & logger, const SmallGicpParams & params,
    const std::vector<double> & prior_pcd_transform);

  bool loadGlobalMap(const std::string & file_name);
  void transformGlobalMap(const Eigen::Affine3d & transform, const std::string & label);
  const pcl::PointCloud<pcl::PointXYZ> & globalMap() const;
  VerificationResult verify(
    const pcl::PointCloud<pcl::PointXYZ> & accumulated_cloud,
    const Eigen::Isometry3d & initial_guess);

private:
  void prepareTarget();
  void updatePriorBounds();
  void filterSourceByPriorMap(
    const pcl::PointCloud<pcl::PointXYZ> & source, const Eigen::Isometry3d & map_to_odom_guess,
    pcl::PointCloud<pcl::PointXYZ> & filtered_source) const;

  rclcpp::Logger logger_;
  SmallGicpParams params_;
  std::vector<double> prior_pcd_transform_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr global_map_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr prior_filter_map_;
  bool has_prior_bounds_{false};
  pcl::PointXYZ prior_min_pt_;
  pcl::PointXYZ prior_max_pt_;
  pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr prior_kdtree_;
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
  ~RelocalizationManagerNode() override = default;

private:
  void registeredPcdCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void motionCommandCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void performRegistration();
  void publishTransform();
  void initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void maybeApplyLidarOffsetToPriorMap();
  void publishPriorPcdMap(const pcl::PointCloud<pcl::PointXYZ> & prior_cloud);
  const char * localizationStateName(LocalizationState state) const;
  std::string localizationStateLogLabel(LocalizationState state) const;
  bool isLocalized() const;
  bool isRobotMoving() const;
  void expireStaleMotionCommand();
  void setLocalizationState(LocalizationState state, const std::string & reason);
  void logLocalizationState();
  void recordAcceptedGicpVerification(const std::string & reason);
  void resetAcceptedGicpVerificationStreak();

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcd_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr motion_command_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr prior_pcd_map_pub_;

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
  std::string motion_command_topic_;
  bool transform_prior_map_with_lidar_offset_;
  double moving_linear_speed_threshold_{0.05};
  double motion_command_timeout_{0.5};
  rclcpp::Time last_scan_time_;
  Eigen::Isometry3d current_map_to_odom_;
  Eigen::Isometry3d previous_map_to_odom_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr accumulated_cloud_;
  std::mutex motion_command_mutex_;
  std::atomic_bool robot_moving_{false};
  std::atomic_bool reset_accumulated_cloud_{false};
  bool has_motion_command_{false};
  rclcpp::Time last_motion_command_time_;
  int consecutive_gicp_acceptances_{0};
  int required_consecutive_gicp_acceptances_{5};
  LocalizationState localization_state_{LocalizationState::LOCALIZING};
  std::unique_ptr<SmallGicpVerifier> small_gicp_verifier_;

  rclcpp::TimerBase::SharedPtr transform_timer_;
  rclcpp::TimerBase::SharedPtr register_timer_;
  rclcpp::TimerBase::SharedPtr state_log_timer_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}  // namespace relocalization_manager

#endif  // RELOCALIZATION_MANAGER__RELOCALIZATION_MANAGER_HPP_
