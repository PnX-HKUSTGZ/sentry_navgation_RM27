#ifndef MINCO_PLANNER__MINCO_PLANNER_HPP_
#define MINCO_PLANNER__MINCO_PLANNER_HPP_

#include "minco_core/header.hpp"
#include "minco_core/components/planning_request_contract.hpp"
#include "minco_core/components/planning_request_lease.hpp"
#include "minco_core/components/planning_session_state.hpp"
#include "minco_core/performance/planner_performance_monitor.hpp"

#include <limits>

namespace rog_map {
class ROGMapROS;
}  // namespace rog_map

namespace minco_planner {

class Visualizer;
class MincoFsm;

class MincoPlanner : public nav2_core::GlobalPlanner
{
public:
  using Ptr = std::shared_ptr<MincoPlanner>;

  // === Constructor & Lifecycle ===
  MincoPlanner();
  ~MincoPlanner();

  void configure(const nav2_util::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void activate() override;
  void deactivate() override;
  void cleanup() override;

  // === Core Planning Interfaces ===
  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    std::function<bool()> cancel_checker) override;

  bool PlanGlobalPath(const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    uint64_t expected_session);

  void setMap(const std::shared_ptr<rog_map::MapQueryInterface> & map);

  bool ReplanLocal(
    const geometry_msgs::msg::PoseStamped & current_pose, uint64_t expected_session);
  bool makePlan(const geometry_msgs::msg::Pose & start,
    const geometry_msgs::msg::Pose & goal,
    double tolerance,
    std::function<bool()> cancel_checker,
    nav_msgs::msg::Path & plan);

  // === Callbacks ===
  void safetyTimerCallback();

  // === Utility & Helper Functions ===
  bool checkCollision();
  bool checkCollision(const geometry_utils::Trajectory & traj);
  bool checkCollision(const geometry_utils::Trajectory & position_traj,
    const geometry_utils::Trajectory & yaw_traj);

  // Accessors for FSM
  bool isTrajSafe() const { return is_traj_safe_.load(); }
  bool ensureTrajectorySafe(const geometry_msgs::msg::PoseStamped & current_pose);
  double nowSeconds() const;
  double getTrajectoryRemainTime() const;
  bool isTrajectoryTimeExpired(double now_s) const;
  double getLookaheadDist() const { return lookahead_dist_; }
  bool getRobotPose(geometry_msgs::msg::PoseStamped & pose) const;
  bool checkGoalReached(const geometry_msgs::msg::PoseStamped & current_pose);
  bool consumePendingGoal(
    geometry_msgs::msg::PoseStamped & goal_out, uint64_t & session_out);
  bool isPlanningSessionCurrent(uint64_t session) const;
  void cancelGoal();
  void completeGoal(uint64_t expected_session);
  Eigen::Vector3d getCurrentSpeed() const;
  double getCurrentYawFromOdom() const;

  // Query ESDF distance at the given position.
  double getEsdfDistance(const Eigen::Vector3d & pos) const;

  bool publishEscapeCommand(const geometry_msgs::msg::PoseStamped & current_pose,
    const Eigen::Vector2d & escape_vel,
    uint64_t expected_session);

  void clearRecoveryDebugVisualization();

  void publishEmergencyStop(const geometry_msgs::msg::PoseStamped & current_pose);

  traj_opt::Trajectory generateBackupTraj(const Eigen::Matrix3d & start_state);
  std::vector<Eigen::Vector3d> extractLocalPath(const Eigen::Vector3d & cur_pos);

private:
  // === Internal Types ===
  enum class PlanningState
  {
    COLD_START,     // Full replanning from measured velocity and zero acceleration.
    HOT_START,      // Replanning with a shifted optimizer seed and measured state.
    EMERGENCY_STOP  // Immediate backup braking with safety priority.
  };

  struct PlanningSessionHandle
  {
    uint64_t session{0U};
    builtin_interfaces::msg::Time planning_stamp;
  };

  // === Utility & Helper Functions ===
  PlanningState determinePlanningState(
    const geometry_msgs::msg::Pose & start_pose, const std::vector<Eigen::Vector3d> & new_path);

  void prepareColdStart(const geometry_msgs::msg::Pose & start_pose,
    Eigen::Matrix3d & start_state,
    const std::vector<Eigen::Vector3d> & sparse_path);

  void prepareHotStart(
    const geometry_msgs::msg::Pose & start_pose, Eigen::Matrix3d & start_state);

  void PTAllocation(const std::vector<Eigen::Vector3d> & sparse_path,
    const Eigen::Matrix3d & start_state,
    bool stop_at_local_end,
    PlanningState state,
    bool has_shifted_seed,
    const vec_Vec3f & shifted_waypoints,
    const VecDf & shifted_durations,
    vec_Vec3f & init_ps,
    VecDf & init_ts,
    VecDf & local_vmaxs) const;

  bool validateTrajectory(const traj_opt::Trajectory & traj, const Eigen::Vector3d & expected_end_pos);

  bool optimizeYaw(const Eigen::Matrix3d & start_state,
    const traj_opt::Trajectory & pos_traj,
    traj_opt::Trajectory & out_yaw_traj,
    PlanningState state,
    const geometry_msgs::msg::Pose & current_pose,
    double goal_yaw);

  rcl_interfaces::msg::SetParametersResult onSetParameters(
    const std::vector<rclcpp::Parameter> & parameters);

  bool configureRogMap(const nav2_util::LifecycleNode::SharedPtr & node, const std::string & plugin_prefix);

  bool ensureMapAvailable();
  void rebuildModeDependentQueries();
  void configureMincoPerfLogging(const nav2_util::LifecycleNode::SharedPtr & node, const std::string & prefix);

  void initPlannerMode(
    const std::string & planner_mode_param,
    const std::string & map_frame,
    const std::string & rog_frame,
    double static_obstacle_clearance_radius);
  bool normalizePoseToFrame(const geometry_msgs::msg::PoseStamped & in,
    const std::string & fallback_frame,
    const std::string & target_frame,
    const std::string & context,
    geometry_msgs::msg::PoseStamped & out) const;
  bool getPhysicalRobotPoseInPlanningFrame(
    geometry_msgs::msg::PoseStamped & pose) const;
  rclcpp::Time rosNow() const;
  bool evaluateCachedTrajectorySafety(
    const geometry_msgs::msg::PoseStamped * fallback_stop_pose);
  bool republishSafeCachedTrajectory(
    const geometry_msgs::msg::PoseStamped & fallback_stop_pose,
    uint64_t expected_session,
    const char * rejection_reason);
  bool checkExecutableTrajectory(
    const geometry_utils::Trajectory & position_traj,
    const geometry_utils::Trajectory & yaw_traj);
  bool checkExecutableTrajectory(
    const geometry_utils::Trajectory & position_traj,
    const geometry_utils::Trajectory & yaw_traj,
    const geometry_msgs::msg::PoseStamped & actual_pose);
  bool checkCollisionFromTime(const geometry_utils::Trajectory & position_traj,
    const geometry_utils::Trajectory & yaw_traj,
    double start_time);
  PlanningSessionHandle acceptPlanningGoal(
    const geometry_msgs::msg::PoseStamped & normalized_goal);
  uint64_t beginPlanningSession();
  void advancePlanningStampLocked();
  void invalidateTrajectoryLocked();
  void publishBlockCommandLocked(bool mirror_to_backup = false);
  bool expirePlanningRequestLeaseLocked(PlanningRequestLease::TimePoint now);
  void requestLeaseTimerCallback();
  bool isLifecycleActive() const;
  void publishEmergencyStopImpl(const geometry_msgs::msg::PoseStamped & current_pose,
    uint64_t expected_generation,
    bool require_generation_match);

  // === ROS 2 Interfaces (Publishers, Subscribers, Timers) ===
  rclcpp::Publisher<ros_interfaces::msg::MpcPositionCommand>::SharedPtr opt_path_pub_;
  rclcpp::Publisher<ros_interfaces::msg::MpcPositionCommand>::SharedPtr backup_path_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr fsm_timer_;
  rclcpp::TimerBase::SharedPtr safety_timer_;
  rclcpp::TimerBase::SharedPtr request_lease_timer_;
  rclcpp::CallbackGroup::SharedPtr odom_callback_group_;
  rclcpp::CallbackGroup::SharedPtr fsm_callback_group_;
  rclcpp::CallbackGroup::SharedPtr safety_callback_group_;
  rclcpp::CallbackGroup::SharedPtr request_lease_callback_group_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_parameters_callback_handle_;

  // === TF & Costmap & Frames ===
  std::shared_ptr<tf2_ros::Buffer> tf_;
  rclcpp::Clock::SharedPtr clock_;
  nav2_util::LifecycleNode::WeakPtr node_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::shared_ptr<rog_map::MapQueryInterface> map_;
  std::shared_ptr<rog_map::MapQueryInterface> rog_query_raw_;
  std::shared_ptr<rog_map::ROGMapROS> rog_map_ros_;
  std::string global_frame_, planning_frame_, output_frame_, map_frame_, rog_frame_, name_;
  std::string physical_base_frame_{"base_link"};
  PlannerModeParams mode_params_;

  // === Configurations & Parameters ===
  double tolerance_;
  bool allow_unknown_;
  bool use_smac_;
  bool use_yaw_opt_{true};
  bool priormap_use_nav2_global_search_{true};
  bool priormap_dynamic_global_obstacle_enable_{false};
  bool priormap_clip_seed_by_rog_boundary_{true};
  bool priormap_ground_edge_avoidance_enable_{false};
  bool exploration_unknown_as_occupied_{true};
  bool exploration_prefer_goal_direction_{true};
  double priormap_rog_boundary_margin_{0.8};
  double priormap_rog_boundary_sample_step_{0.1};
  double priormap_dynamic_global_collision_distance_{0.0};
  double priormap_ground_edge_max_step_{0.06};
  double priormap_ground_edge_max_slope_deg_{28.0};
  double priormap_ground_edge_lethal_clearance_radius_{0.0};
  double priormap_ground_edge_clearance_radius_{0.20};
  int priormap_ground_edge_clearance_cost_{240};
  std::string priormap_ground_edge_yaml_path_{};
  std::string priormap_ground_edge_pgm_path_{};
  std::string priormap_ground_edge_frame_{"map"};
  double exploration_boundary_margin_{0.8};
  double exploration_boundary_sample_step_{0.1};
  double lidar_offset_x_{0.0};
  double lidar_offset_y_{-0.2};
  double opt_freq_;
  double failed_replan_retry_period_{0.25};
  double successful_replan_period_{1.0};
  double lookahead_dist_;
  double traj_goal_tolerance_{0.5};
  // Maximum future time certified against the rolling ROG snapshot.
  double safety_check_horizon_{2.0};
  double collision_cache_reuse_max_duration_{0.75};
  double request_lease_timeout_{0.0};
  MincoOptimizer::Config minco_config;
  RecoverServer::Config recovery_server_config_{};
  PlannerPerformanceMonitor planner_perf_monitor_;

  // === Core Modules (Pointers to FSM, Optimizers, etc.) ===
  std::unique_ptr<Astar> astar_planner_;
  std::unique_ptr<minco_planner::smac::SmacPlanner2DSimple> smac_planner_;
  std::unique_ptr<MincoOptimizer> minco_optimizer_;
  std::unique_ptr<traj_opt::BackupTrajOpt> backup_opt_;
  std::unique_ptr<traj_opt::YawTrajOpt> yaw_opt_;
  std::unique_ptr<PlannerModeContext> mode_context_;
  std::unique_ptr<GlobalPathSearcher> global_path_searcher_;
  std::unique_ptr<LocalPathProcessor> local_path_processor_;
  std::unique_ptr<TrajectorySafetyChecker> safety_checker_;
  std::unique_ptr<MincoFsm> fsm_;
  SimpleCorridorGenerator::Ptr corridor_gen_;
  RecoverServer::Ptr recovery_server_;
  Ptr planner_handle_;

  // === State Variables & Caches ===
  uint32_t opt_trajectory_id_{0};
  geometry_utils::Trajectory last_traj_;
  geometry_utils::Trajectory last_yaw_traj_;
  std::vector<geometry_msgs::msg::PoseStamped> latest_global_path_;
  nav_msgs::msg::Odometry latest_odom_;
  geometry_msgs::msg::PoseStamped pending_goal_;
  geometry_msgs::msg::PoseStamped active_goal_;

  bool has_last_traj_ = false;
  bool has_last_yaw_traj_ = false;
  bool has_pending_goal_{false};
  bool has_active_goal_{false};
  bool has_latest_odom_{false};
  std::atomic_bool is_traj_safe_{true};
  uint64_t trajectory_generation_{0};
  uint64_t pending_goal_session_{0};
  uint64_t active_goal_session_{0};
  uint64_t latest_global_path_session_{0};
  uint64_t last_trajectory_session_{0};
  builtin_interfaces::msg::Time planning_stamp_;
  PlanningSessionState planning_session_;
  PlanningRequestLease request_lease_;
  bool emergency_stop_latched_{false};

  std::mutex path_mutex_;
  std::mutex goal_mutex_;
  std::mutex perf_mutex_;
  std::mutex backup_mutex_;
  std::mutex fsm_execution_mutex_;
  std::mutex safety_execution_mutex_;
  std::mutex request_lease_execution_mutex_;
  mutable std::mutex odom_mutex_;
  mutable std::mutex mutex_;

  std::unique_ptr<Visualizer> visualizer_;
  rclcpp::Logger logger_{rclcpp::get_logger("MincoPlanner")};
  double last_global_search_time_ms_{std::numeric_limits<double>::quiet_NaN()};
  bool has_fresh_global_search_time_{false};
  long long last_minco_perf_stamp_ns_{0};
  std::string last_validation_failure_reason_{"KINEMATIC_VIOLATION"};
};

}  // namespace minco_planner

#endif  // MINCO_PLANNER__MINCO_PLANNER_HPP_
