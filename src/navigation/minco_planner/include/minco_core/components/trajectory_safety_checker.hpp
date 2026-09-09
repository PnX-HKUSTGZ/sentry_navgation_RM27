#ifndef MINCO_PLANNER__TRAJECTORY_SAFETY_CHECKER_HPP_
#define MINCO_PLANNER__TRAJECTORY_SAFETY_CHECKER_HPP_

#include "minco_core/header.hpp"

namespace minco_planner {

class TrajectorySafetyChecker {
public:
  enum class FailureReason : uint8_t {
    NONE = 0,
    QUERY_UNAVAILABLE,
    INVALID_TRAJECTORY_DURATION,
    INVALID_YAW_TRAJECTORY,
    NONFINITE_CENTER,
    NONFINITE_YAW,
    INVALID_MAP_RESOLUTION,
    OUT_OF_MAP,
    COSTMAP_UNKNOWN,
    COSTMAP_LETHAL,
    COSTMAP_INSCRIBED,
    QUERY_FAILED,
    INVALID_SNAPSHOT_STAMP,
    INVALID_ROS_TIME,
    STALE_SNAPSHOT,
    FUTURE_SNAPSHOT,
    NONFINITE_DISTANCE,
    INSUFFICIENT_CLEARANCE,
    INVALID_TRAJECTORY_START_TIME,
  };

  struct FailureDiagnostic {
    FailureReason reason{FailureReason::NONE};
    double trajectory_time{std::numeric_limits<double>::quiet_NaN()};
    double trajectory_duration{std::numeric_limits<double>::quiet_NaN()};
    Eigen::Vector3d center{Eigen::Vector3d::Constant(
      std::numeric_limits<double>::quiet_NaN())};
    Eigen::Vector3d query_point{Eigen::Vector3d::Constant(
      std::numeric_limits<double>::quiet_NaN())};
    Eigen::Vector2d footprint_offset{Eigen::Vector2d::Constant(
      std::numeric_limits<double>::quiet_NaN())};
    double yaw{std::numeric_limits<double>::quiet_NaN()};
    double distance{std::numeric_limits<double>::quiet_NaN()};
    double safe_distance{std::numeric_limits<double>::quiet_NaN()};
    double snapshot_age{std::numeric_limits<double>::quiet_NaN()};
    double snapshot_commit_age{std::numeric_limits<double>::quiet_NaN()};
    double snapshot_processing_age_ms{std::numeric_limits<double>::quiet_NaN()};
    uint8_t cost{nav2_costmap_2d::NO_INFORMATION};
    rog_map::ProjectionDiagnostic projection;
    rog_map::QueryStatus query_status{rog_map::QueryStatus::SNAPSHOT_INVALID};
    bool cost_checked{false};
    bool query_attempted{false};
    bool footprint_sample{false};
    std::string planning_frame{"unknown"};
    std::string rog_frame{"unknown"};
  };

  struct Config {
    double safe_dist{0.0};
    double footprint_length{0.30};
    double footprint_width{0.30};
    double footprint_margin{0.05};
    double sample_dt{0.05};
    double map_timeout{0.50};
    double future_tolerance{0.05};
    std::string planning_frame{"unknown"};
    std::string rog_frame{"unknown"};
  };

  void configure(const Config &config, rclcpp::Logger logger,
                 rclcpp::Clock::SharedPtr clock);
  void setQuery(std::shared_ptr<rog_map::MapQueryInterface> dynamic_query);

  bool checkPoint(const Eigen::Vector3d &pos) const;
  bool checkFootprint(const Eigen::Vector3d &pos, double yaw) const;
  bool checkSweptFootprint(const Eigen::Vector3d &from_pos, double from_yaw,
                           const Eigen::Vector3d &to_pos, double to_yaw) const;
  bool checkTrajectory(const traj_opt::Trajectory &traj) const;
  bool checkTrajectoryFromTime(const traj_opt::Trajectory &traj,
                               double start_time) const;
  bool checkTrajectory(const traj_opt::Trajectory &position_traj,
                       const traj_opt::Trajectory &yaw_traj) const;
  bool checkTrajectoryFromTime(const traj_opt::Trajectory &position_traj,
                               const traj_opt::Trajectory &yaw_traj,
                               double start_time) const;
  bool computeSpatialCheckStartTime(const traj_opt::Trajectory &traj,
                                    const Eigen::Vector3d &actual_position,
                                    double &start_time) const;
  double getDistance(const Eigen::Vector3d &pos) const;
  bool projectOutOfObstacle(Eigen::Vector3d &pos, double margin) const;
  FailureDiagnostic lastFailureDiagnostic() const;

  static const char *failureReasonName(FailureReason reason);

private:
  struct PointCheckResult {
    bool safe{false};
    FailureDiagnostic diagnostic;
  };

  std::shared_ptr<rog_map::MapQueryInterface> querySnapshot() const;
  PointCheckResult evaluatePoint(
      const std::shared_ptr<rog_map::MapQueryInterface> &query,
      const Eigen::Vector3d &pos) const;
  PointCheckResult evaluateQueryResult(
      const std::shared_ptr<rog_map::MapQueryInterface> &query,
      const Eigen::Vector3d &pos, const rog_map::QueryResult &result,
      double query_time) const;
  bool checkPoint(const std::shared_ptr<rog_map::MapQueryInterface> &query,
                  const Eigen::Vector3d &pos) const;
  bool evaluateFootprint(
      const std::shared_ptr<rog_map::MapQueryInterface> &query,
      const Eigen::Vector3d &pos, double yaw, double trajectory_time,
      FailureDiagnostic *failure) const;
  bool checkFootprint(const std::shared_ptr<rog_map::MapQueryInterface> &query,
                      const Eigen::Vector3d &pos, double yaw) const;
  bool queryIsFresh(const rog_map::QueryResult &result) const;
  FailureReason freshnessFailure(const rog_map::QueryResult &result,
                                 double &snapshot_age) const;
  FailureReason freshnessFailureAt(const rog_map::QueryResult &result,
                                   double query_time,
                                   double &snapshot_age) const;
  void recordTrajectoryFailure(FailureDiagnostic diagnostic,
                               double trajectory_duration) const;
  void clearTrajectoryFailure() const;

  mutable std::mutex query_mutex_;
  std::shared_ptr<rog_map::MapQueryInterface> dynamic_query_;
  double safe_dist_{0.0};
  double footprint_length_{0.30};
  double footprint_width_{0.30};
  double footprint_margin_{0.05};
  double sample_dt_{0.05};
  double map_timeout_{0.50};
  double future_tolerance_{0.05};
  std::string planning_frame_{"unknown"};
  std::string rog_frame_{"unknown"};
  rclcpp::Logger logger_{rclcpp::get_logger("TrajectorySafetyChecker")};
  rclcpp::Clock::SharedPtr clock_;
  mutable std::mutex failure_mutex_;
  mutable FailureDiagnostic last_failure_;
};

} // namespace minco_planner

#endif // MINCO_PLANNER__TRAJECTORY_SAFETY_CHECKER_HPP_
