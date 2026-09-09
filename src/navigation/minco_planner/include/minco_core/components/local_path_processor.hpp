#ifndef MINCO_PLANNER__LOCAL_PATH_PROCESSOR_HPP_
#define MINCO_PLANNER__LOCAL_PATH_PROCESSOR_HPP_

#include "minco_core/header.hpp"

namespace minco_planner {

class PlannerModeContext;

struct LocalPathSeed
{
  bool valid{false};
  bool local_end_is_goal{false};
  bool observed_prefix_clipped{false};
  bool stop_at_local_end{false};
  std::vector<Eigen::Vector3d> dense_path;
  std::vector<Eigen::Vector3d> sparse_waypoints;
  std::vector<double> local_magnitudes;
};

bool shouldOptimizeYawForSeed(
  bool yaw_optimization_enabled, const LocalPathSeed & seed) noexcept;

class LocalPathProcessor
{
public:
  using FootprintSafetyCheck =
    std::function<bool(const Eigen::Vector3d & position, double yaw)>;

  void configure(double lookahead_dist,
    double max_vel,
    double max_acc,
    double traj_goal_tolerance,
    rclcpp::Logger logger,
    rclcpp::Clock::SharedPtr clock);

  void updateLimits(double max_vel, double max_acc, double traj_goal_tolerance);

  LocalPathSeed buildSeed(const std::vector<geometry_msgs::msg::PoseStamped> & global_path,
    const geometry_msgs::msg::PoseStamped & current_pose,
    const PlannerModeContext & mode_context,
    const FootprintSafetyCheck & footprint_is_safe) const;

private:
  std::vector<Eigen::Vector3d> extractLocalPath(
    const std::vector<geometry_msgs::msg::PoseStamped> & global_path,
    const Eigen::Vector3d & cur_pos) const;

  bool clipLocalPathByRogBoundary(
    std::vector<Eigen::Vector3d> & path, const PlannerModeContext & mode_context) const;

  bool clipToObservedSafePrefix(
    std::vector<Eigen::Vector3d> & path,
    const PlannerModeContext & mode_context,
    double initial_yaw,
    const FootprintSafetyCheck & footprint_is_safe,
    bool & clipped) const;

  double lookahead_dist_{5.0};
  double max_vel_{2.0};
  double max_acc_{4.0};
  double traj_goal_tolerance_{0.5};
  rclcpp::Logger logger_{rclcpp::get_logger("LocalPathProcessor")};
  rclcpp::Clock::SharedPtr clock_;
};

}  // namespace minco_planner

#endif  // MINCO_PLANNER__LOCAL_PATH_PROCESSOR_HPP_
