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
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "pcl/io/pcd_io.h"
#include "pcl/kdtree/kdtree_flann.h"
#include "rclcpp/rclcpp.hpp"
#include "scan_context/scan_context_manager.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "small_gicp/ann/kdtree_omp.hpp"
#include "small_gicp/factors/gicp_factor.hpp"
#include "small_gicp/pcl/pcl_point.hpp"
#include "small_gicp/registration/reduction_omp.hpp"
#include "small_gicp/registration/registration.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_srvs/srv/trigger.hpp"
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

enum class LocalizationState { LOCALIZING, LOCALIZED };

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

private:
  void registeredPcdCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void motionCommandCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void performRegistration();
  void publishTransform();
  void initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void maybeApplyLidarOffsetToPriorMap();
  void publishPriorPcdMap(const pcl::PointCloud<pcl::PointXYZ> & prior_cloud);
  void configureScanContext();
  void buildScanContextDatabaseFromPriorPcd();
  void queueScanContextInput(
    const std_msgs::msg::Header & header, const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & scan);
  void processPendingScanContext();
  bool transformCloudToScanContextFrame(
    const std_msgs::msg::Header & header, const pcl::PointCloud<pcl::PointXYZ> & scan,
    pcl::PointCloud<pcl::PointXYZ> & cloud_in_scan_context_frame);
  void updateScanContextDescriptor(
    const std_msgs::msg::Header & header, const pcl::PointCloud<pcl::PointXYZ> & scan);
  bool lookupScanContextPoseInMap(
    const std_msgs::msg::Header & header, Eigen::Isometry3d & map_to_scan_context_frame);
  bool shouldAddScanContextKeyframe(
    const Eigen::Isometry3d & map_to_scan_context_frame, const rclcpp::Time & stamp) const;
  void maybeAddScanContextKeyframe(
    const std_msgs::msg::Header & header, const pcl::PointCloud<pcl::PointXYZ> & cloud,
    const ::scan_context::ScanContextDescriptor & descriptor);
  bool saveScanContextDatabase();
  void saveScanContextDatabaseCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void triggerScanContextRelocalizationCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  bool getLatestScanContextCloud(
    pcl::PointCloud<pcl::PointXYZ> & cloud_in_scan_context_frame, rclcpp::Time & stamp);
  bool makeMapToBaseEstimateFromScanContextCandidate(
    const ::scan_context::ScanContextCandidate & candidate, Eigen::Isometry3d & map_to_base);
  bool makeMapToOdomGuessFromScanContextCandidate(
    const ::scan_context::ScanContextCandidate & candidate, const rclcpp::Time & stamp,
    Eigen::Isometry3d & map_to_odom_guess, Eigen::Isometry3d & map_to_base_estimate);
  bool verifyWithScanContextCandidates(
    const pcl::PointCloud<pcl::PointXYZ> & accumulated_cloud, const std::string & reason,
    VerificationResult & best_verification);
  void publishScanContextCandidates(
    const std::vector<::scan_context::ScanContextCandidate> & candidates,
    const std::vector<Eigen::Isometry3d> & map_to_base_estimates, const rclcpp::Time & stamp);
  void publishAcceptedScanContextPose(
    const Eigen::Isometry3d & map_to_base_estimate, const rclcpp::Time & stamp);
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
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_scan_context_database_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr trigger_scan_context_relocalization_service_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr prior_pcd_map_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr scan_context_candidates_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr scan_context_best_pose_pub_;

  SmallGicpParams small_gicp_params_;
  ::scan_context::ScanContextParams scan_context_params_;
  bool scan_context_query_on_startup_{true};
  bool scan_context_query_on_gicp_failure_{true};
  int scan_context_failure_trigger_count_{3};
  int scan_context_max_gicp_candidates_{5};
  double scan_context_yaw_delta_sign_{1.0};
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
  pcl::PointCloud<pcl::PointXYZ>::Ptr latest_scan_context_cloud_;
  ::scan_context::ScanContextDescriptor latest_scan_context_descriptor_;
  rclcpp::Time latest_scan_context_stamp_;
  bool has_latest_scan_context_descriptor_{false};
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr pending_scan_context_cloud_;
  std_msgs::msg::Header pending_scan_context_header_;
  bool has_pending_scan_context_cloud_{false};
  std::mutex scan_context_mutex_;
  std::mutex scan_context_database_mutex_;
  std::mutex motion_command_mutex_;
  bool scan_context_database_dirty_{false};
  std::atomic_bool force_scan_context_query_once_{false};
  std::atomic_bool scan_context_startup_query_pending_{false};
  std::atomic_bool robot_moving_{false};
  std::atomic_bool reset_accumulated_cloud_{false};
  bool has_motion_command_{false};
  rclcpp::Time last_motion_command_time_;
  int consecutive_gicp_failures_{0};
  int consecutive_gicp_acceptances_{0};
  int required_consecutive_gicp_acceptances_{5};
  LocalizationState localization_state_{LocalizationState::LOCALIZING};
  bool has_last_scan_context_keyframe_{false};
  Eigen::Isometry3d last_scan_context_keyframe_pose_{Eigen::Isometry3d::Identity()};
  rclcpp::Time last_scan_context_keyframe_stamp_;
  std::unique_ptr<SmallGicpVerifier> small_gicp_verifier_;
  std::unique_ptr<::scan_context::ScanContextManager> scan_context_manager_;

  rclcpp::TimerBase::SharedPtr transform_timer_;
  rclcpp::TimerBase::SharedPtr register_timer_;
  rclcpp::TimerBase::SharedPtr scan_context_timer_;
  rclcpp::TimerBase::SharedPtr state_log_timer_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}  // namespace relocalization_manager

#endif  // RELOCALIZATION_MANAGER__RELOCALIZATION_MANAGER_HPP_
