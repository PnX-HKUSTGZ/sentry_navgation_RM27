#ifndef MINCO_PLANNER__MINCO_FSM_HPP_
#define MINCO_PLANNER__MINCO_FSM_HPP_

#include "minco_core/header.hpp"

namespace minco_planner {

class MincoPlanner;
class RecoverServer;

class MincoFsm
{
public:
  // === Internal Types ===
  enum class State
  {
    INIT,
    WAIT_GOAL,
    GENERATE_TRAJ,
    FOLLOW_TRAJ,
    RECOVERING,
  };

  using PlannerPtr = std::shared_ptr<MincoPlanner>;
  using RecoveryPtr = std::shared_ptr<RecoverServer>;

  // === Constructor & Lifecycle ===
  MincoFsm(const PlannerPtr & planner, const RecoveryPtr & recovery_server,
    double failed_replan_retry_period, double successful_replan_period);

  // === Core Planning Interfaces ===
  void callMainFsmOnce();
  void cancelGoal();

  // === Utility & Helper Functions ===
  State getState() const { return state_; }

private:
  // === Utility & Helper Functions ===
  // --- State Transition ---
  void changeState(const char * caller, State new_state);
  bool generateRetryDeferred() const;
  void deferGenerateRetry();
  void clearGenerateRetry();

  // === Core Modules (Pointers to FSM, Optimizers, etc.) ===
  PlannerPtr planner_;
  RecoveryPtr recovery_server_;

  // === State Variables & Caches ===
  // --- FSM State ---
  State state_{State::INIT};
  State last_state_{State::INIT};
  using RetryClock = std::chrono::steady_clock;
  RetryClock::duration failed_replan_retry_period_{};
  RetryClock::time_point generate_retry_not_before_{};
  bool has_generate_retry_deadline_{false};
  RetryClock::duration successful_replan_period_{};
  RetryClock::time_point follow_replan_not_before_{};
  bool has_follow_replan_deadline_{false};
  bool follow_replan_retry_deferred_{false};

  // --- Goal Lifecycle ---
  bool has_goal_{false};
  uint64_t goal_session_{0};
  geometry_msgs::msg::PoseStamped goal_;

  // --- Motion Tracking Cache ---
  bool has_last_pose_{false};
  double last_pose_x_{0.0};
  double last_pose_y_{0.0};
  double traveled_dist_{0.0};

  // --- Recovery / Emergency Runtime ---
  bool stop_published_{false};
  bool goal_stop_published_{false};
  double emer_stop_start_time_{0.0};
  Eigen::Vector2d current_escape_vel_{0.0, 0.0};
};

}  // namespace minco_planner

#endif  // MINCO_PLANNER__MINCO_FSM_HPP_
