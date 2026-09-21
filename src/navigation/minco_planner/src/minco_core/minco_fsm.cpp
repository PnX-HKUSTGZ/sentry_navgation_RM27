#include "minco_core/minco_fsm.hpp"

// C++ standard library
#include <cmath>
#include <iostream>

// Project
#include "minco_core/minco_planner.hpp"

namespace minco_planner {

// -----------------------------------------------------------------------------
// 1) Construction / Destruction
// -----------------------------------------------------------------------------

MincoFsm::MincoFsm(const PlannerPtr & planner, const RecoveryPtr & recovery_server,
  double failed_replan_retry_period, double successful_replan_period)
: planner_(planner), recovery_server_(recovery_server)
{
  const double bounded_period =
    std::isfinite(failed_replan_retry_period) ?
    std::max(0.0, failed_replan_retry_period) : 0.25;
  failed_replan_retry_period_ = std::chrono::duration_cast<RetryClock::duration>(
    std::chrono::duration<double>(bounded_period));
  const double bounded_success_period =
    std::isfinite(successful_replan_period) ?
    std::max(0.0, successful_replan_period) : 1.0;
  successful_replan_period_ = std::chrono::duration_cast<RetryClock::duration>(
    std::chrono::duration<double>(bounded_success_period));
}

void MincoFsm::cancelGoal()
{
  clearGenerateRetry();
  has_follow_replan_deadline_ = false;
  follow_replan_retry_deferred_ = false;
  force_global_search_ = true;
  has_goal_ = false;
  goal_session_ = 0U;
  if (recovery_server_) {
    recovery_server_->clearMissionGoal();
  }
  // Disabled to prevent zero-velocity deadlock
  // changeState("CANCEL_GOAL", State::EMER_STOP);
  changeState("CANCEL_GOAL", State::WAIT_GOAL);
}

// -----------------------------------------------------------------------------
// 2) Core business interface
// -----------------------------------------------------------------------------

void MincoFsm::callMainFsmOnce()
{
  if (!planner_ || !recovery_server_) {
    return;
  }

  // A new/cancelled goal or lifecycle transition invalidates the token before
  // any potentially long-running search can publish. Reconcile the FSM on the
  // next tick without sharing mutable goal state across callbacks.
  if (has_goal_ && !planner_->isPlanningSessionCurrent(goal_session_)) {
    clearGenerateRetry();
    has_follow_replan_deadline_ = false;
    follow_replan_retry_deferred_ = false;
    force_global_search_ = true;
    has_goal_ = false;
    goal_session_ = 0U;
    recovery_server_->clearMissionGoal();
    changeState("STALE_SESSION", State::WAIT_GOAL);
  }

  // Consume latest goal (createPlan only sets this flag).
  // Disabled to prevent zero-velocity deadlock
  // if (state_ != State::EMER_STOP && state_ != State::RECOVERING) {
  if (state_ != State::RECOVERING) {
    geometry_msgs::msg::PoseStamped new_goal;
    uint64_t new_goal_session = 0U;
    if (planner_->consumePendingGoal(new_goal, new_goal_session)) {
      clearGenerateRetry();
      has_follow_replan_deadline_ = false;
      follow_replan_retry_deferred_ = false;
      goal_ = new_goal;
      goal_session_ = new_goal_session;
      has_goal_ = true;
      force_global_search_ = true;
      recovery_server_->setMissionGoal(new_goal);
      changeState("NewGoal", State::GENERATE_TRAJ);
    }
  }

  // Get current robot pose.
  geometry_msgs::msg::PoseStamped current_pose;
  const bool has_odom = planner_->getRobotPose(current_pose);

  switch (state_) {
  case State::INIT: {
    if (!has_odom) {
      return;
    }
    changeState("INIT", State::WAIT_GOAL);
    break;
  }

  case State::WAIT_GOAL: {
    if (!has_goal_) {
      return;
    }
    changeState("WAIT_GOAL", State::GENERATE_TRAJ);
    break;
  }

  case State::GENERATE_TRAJ: {
    if (!has_goal_) {
      changeState("GENERATE_TRAJ", State::WAIT_GOAL);
      return;
    }
    if (!has_odom) {
      changeState("GENERATE_TRAJ", State::INIT);
      return;
    }
    if (generateRetryDeferred()) {
      return;
    }

    auto handle_generate_replan_failure = [this, &current_pose](
                                            const char * escape_reason, const char * emer_reason) {
      if (!planner_->ensureTrajectorySafe(current_pose)) {
        force_global_search_ = true;
        changeState("UNSAFE_OLD_TRAJECTORY_STOP", State::GENERATE_TRAJ);
        return;
      }

      Eigen::Vector2d escape_vel;
      const auto decision = recovery_server_->handleReplanFailure(
        planner_->nowSeconds(),
        current_pose,
        [this](const Eigen::Vector3d & p) {
          return planner_->getEsdfDistance(p);
        },
        escape_vel);

      if (decision == RecoverServer::RecoveryDecision::DO_ESCAPE) {
        current_escape_vel_ = escape_vel;
        changeState(escape_reason, State::RECOVERING);
        return;
      }

      if (decision == RecoverServer::RecoveryDecision::ENTER_EMER_STOP) {
        // Disabled to prevent zero-velocity deadlock
        // changeState(emer_reason, State::EMER_STOP);
        force_global_search_ = true;
        changeState(emer_reason, State::GENERATE_TRAJ);
        return;
      }

      // Recovery threshold not reached yet: stay in GENERATE_TRAJ and keep accumulating failures.
      // Disabled to prevent zero-velocity deadlock
      // if (!stop_published_) {
      //   planner_->publishEmergencyStop(current_pose);
      //   stop_published_ = true;
      // }
    };

    if (force_global_search_ || !planner_->hasGlobalPath(goal_session_)) {
      if (!planner_->PlanGlobalPath(current_pose, goal_, goal_session_)) {
        force_global_search_ = true;
        handle_generate_replan_failure(
          "GLOBAL_SEARCH_FAIL_TRIGGER_RECOVERING", "GLOBAL_SEARCH_FAIL_RECOVERY_FAIL");
        if (state_ == State::GENERATE_TRAJ) {
          deferGenerateRetry();
        }
        return;
      }
      force_global_search_ = false;
    }
    if (!planner_->ReplanLocal(current_pose, goal_session_)) {
      Eigen::Vector3d cur_p(current_pose.pose.position.x, current_pose.pose.position.y, 0.0);
      if (!planner_->lastLocalReplanWasOptimizerFailure()) {
        force_global_search_ = true;
      }
      // double dist = planner_->getEsdfDistance(cur_p);
      // if (dist < 0.25) {
      handle_generate_replan_failure("GEN_STUCK_TRIGGER_RECOVERING", "GENERATE_RECOVERY_FAIL");
      if (state_ == State::GENERATE_TRAJ) {
        deferGenerateRetry();
      }
      // return;
      // }
      // recovery_server_->onReplanSuccess();
      return;
    }

    traveled_dist_ = 0.0;
    clearGenerateRetry();
    follow_replan_not_before_ = RetryClock::now() + successful_replan_period_;
    has_follow_replan_deadline_ = true;
    follow_replan_retry_deferred_ = false;
    changeState("GENERATE_TRAJ", State::FOLLOW_TRAJ);
    break;
  }

  case State::FOLLOW_TRAJ: {
    if (!has_goal_) {
      changeState("FOLLOW_TRAJ", State::WAIT_GOAL);
      return;
    }
    if (!has_odom) {
      changeState("FOLLOW_TRAJ", State::INIT);
      return;
    }

    // 容差限停检测：到达终点且速度足够低
    if (planner_->checkGoalReached(current_pose)) {
      // if (!goal_stop_published_) {
      //   planner_->publishEmergencyStop(current_pose);
      //   goal_stop_published_ = true;
      // }

      if (planner_->getCurrentSpeed().head<2>().norm() < 0.3) {
        planner_->completeGoal(goal_session_);
        has_goal_ = false;
        goal_session_ = 0U;
        has_follow_replan_deadline_ = false;
        follow_replan_retry_deferred_ = false;
        recovery_server_->clearMissionGoal();
        changeState("GOAL_REACHED", State::WAIT_GOAL);
      }

      return;
    }

    // Disabled to prevent zero-velocity deadlock
    // goal_stop_published_ = false;

    const double now_s = planner_->nowSeconds();
    const auto steady_now = RetryClock::now();
    const bool urgent_replan =
      planner_->isTrajectoryTimeExpired(now_s) || !planner_->isTrajSafe();
    const bool periodic_replan =
      !has_follow_replan_deadline_ || steady_now >= follow_replan_not_before_;
    const bool need_replan = urgent_replan || periodic_replan;

    if (!need_replan ||
      (follow_replan_retry_deferred_ &&
      steady_now < follow_replan_not_before_))
    {
      return;
    }

    if (!planner_->ReplanLocal(current_pose, goal_session_)) {
      follow_replan_not_before_ = steady_now + failed_replan_retry_period_;
      has_follow_replan_deadline_ = true;
      follow_replan_retry_deferred_ = true;
      const bool optimizer_failure =
        planner_->lastLocalReplanWasOptimizerFailure();
      // Reuse only after checking the robot's actual footprint, its swept join
      // to the controller's spatial pickup point, and the remaining trajectory
      // through the endpoint. An unsafe cache has already emitted BLOCK here.
      if (!planner_->ensureTrajectorySafe(current_pose)) {
        // The local seed follows latest_global_path_. If that route is now
        // obstructed, retrying only ReplanLocal() can never discover another
        // homotopy. Regenerate the global path so the latest ROG costmap
        // overlay and dynamic hard mask can route around the obstruction.
        force_global_search_ = true;
        clearGenerateRetry();
        changeState("UNSAFE_LOCAL_REPLAN_GLOBAL_SEARCH", State::GENERATE_TRAJ);
        return;
      }
      if (!optimizer_failure) {
        // A collision/invalid seed means the cached homotopy is no longer a
        // useful local route, even if its short certified remainder is safe.
        force_global_search_ = true;
        clearGenerateRetry();
        changeState("UNSAFE_LOCAL_SEED_GLOBAL_SEARCH", State::GENERATE_TRAJ);
        return;
      }
      if (!planner_->isTrajectoryTimeExpired(now_s)) {
        return;
      }

      // 1. 失败诊断：区分是”前方路被挡”还是”自身被卡死”
      Eigen::Vector3d cur_p(current_pose.pose.position.x, current_pose.pose.position.y, 0.0);
      double dist = planner_->getEsdfDistance(cur_p);

      // 2. 诊断为安全 (ESDF >= 0.25m)：纯粹前方路障，立即绕路
      if (dist >= 0.25) {
        // recovery_server_->onReplanSuccess();  // 清空失败计数
        force_global_search_ = true;
        changeState("PATH_BLOCKED_DETOUR", State::GENERATE_TRAJ);
        return;
      }

      // 3. 诊断为危险 (ESDF < 0.25m)：陷入死角，请求推离自救
      Eigen::Vector2d escape_vel;
      const auto decision = recovery_server_->handleReplanFailure(
        planner_->nowSeconds(),
        current_pose,
        [this](const Eigen::Vector3d & p) {
          return planner_->getEsdfDistance(p);
        },
        escape_vel);

      if (decision == RecoverServer::RecoveryDecision::DO_ESCAPE) {
        current_escape_vel_ = escape_vel;
        changeState("STUCK_TRIGGER_RECOVERING", State::RECOVERING);
        return;
      }

      if (decision == RecoverServer::RecoveryDecision::ENTER_EMER_STOP) {
        // Disabled to prevent zero-velocity deadlock
        // changeState("FOLLOW_REPLAN_RECOVERY", State::EMER_STOP);
        force_global_search_ = true;
        changeState("FOLLOW_REPLAN_RECOVERY", State::GENERATE_TRAJ);
        return;
      }

      // 4. 处于 NONE 状态
      // Disabled to prevent zero-velocity deadlock
      // if (!stop_published_) {
      //   planner_->publishEmergencyStop(current_pose);
      //   stop_published_ = true;
      // }
      return;
    }

    follow_replan_not_before_ = steady_now + successful_replan_period_;
    has_follow_replan_deadline_ = true;
    follow_replan_retry_deferred_ = false;
    recovery_server_->onReplanSuccess();

    traveled_dist_ = 0.0;
    return;
  }

  case State::RECOVERING: {
    if (!has_odom) {
      return;
    }

    const double now_s = planner_->nowSeconds();
    Eigen::Vector3d cur_p(current_pose.pose.position.x, current_pose.pose.position.y, 0.0);
    double dist = planner_->getEsdfDistance(cur_p);

    // 条件1: 成功挤出泥坑 (ESDF 距离恢复安全)
    if (dist > 0.40) {
      recovery_server_->finishRecovery(true, now_s);
      force_global_search_ = true;
      changeState("ESCAPE_SUCCESS", State::GENERATE_TRAJ);
      return;
    }

    // 条件2: 挣扎超时保护
    if (!recovery_server_->inRecovery(now_s)) {
      recovery_server_->finishRecovery(false, now_s);
      // Disabled to prevent zero-velocity deadlock
      // changeState("ESCAPE_TIMEOUT", State::EMER_STOP);
      force_global_search_ = true;
      changeState("ESCAPE_TIMEOUT", State::GENERATE_TRAJ);
      return;
    }

    // 条件3: 持续生成短时脱困轨迹；每次下发前都重新执行完整安全检查。
    if (!planner_->publishEscapeCommand(current_pose, current_escape_vel_, goal_session_)) {
      recovery_server_->finishRecovery(false, now_s);
      changeState("ESCAPE_SAFETY_REJECTED", State::GENERATE_TRAJ);
    }
    return;
  }

    // Disabled to prevent zero-velocity deadlock
    /*
    case State::EMER_STOP: {
      if (!has_odom) {
        return;
      }

      // 1) First run: publish independent brake trajectory.
      if (!stop_published_) {
        planner_->publishEmergencyStop(current_pose);
        stop_published_ = true;
        emer_stop_start_time_ = planner_->nowSeconds();
      }

      // 2) Timeout protection: avoid deadlock.
      const double now_s = planner_->nowSeconds();
      if (std::isfinite(now_s) && std::isfinite(emer_stop_start_time_) &&
          (now_s - emer_stop_start_time_) > 2.0) {
        // Keep mission goal so FSM can retry planning automatically after emergency stop timeout.
        changeState("EMER_TIMEOUT", has_goal_ ? State::GENERATE_TRAJ : State::WAIT_GOAL);
        return;
      }

      // 3) Still try to find safe path to recover without fully stopping.
      bool recover_possible = false;
      if (planner_->PlanGlobalPath(current_pose, goal_, goal_session_)) {
        if (planner_->ReplanLocal(current_pose, goal_session_)) {
          recover_possible = true;
        }
      }

      if (recover_possible) {
        recovery_server_->finishRecovery(true, now_s);
        changeState("EMER_RECOVER", State::FOLLOW_TRAJ);
        return;
      }

      // 4) Blocking wait until fully stopped.
      const Eigen::Vector3d speed = planner_->getCurrentSpeed();
      if (std::isfinite(speed.head<2>().norm()) && speed.head<2>().norm() > 0.1) {
        return;
      }

      // 5) Recovery: stopped, check safety before leaving EMER_STOP.
      recovery_server_->finishRecovery(false, now_s);
      // Keep mission goal so robot can continue navigating once it is safe/stopped.
      changeState("EMER_SAFE", has_goal_ ? State::GENERATE_TRAJ : State::WAIT_GOAL);
      return;
    }
    */

  default:
    break;
  }
}

// -----------------------------------------------------------------------------
// 3) Helpers
// -----------------------------------------------------------------------------

namespace {
[[maybe_unused]] const char * StateToString(MincoFsm::State s)
{
  switch (s) {
  case MincoFsm::State::INIT:
    return "INIT";
  case MincoFsm::State::WAIT_GOAL:
    return "WAIT_GOAL";
  case MincoFsm::State::GENERATE_TRAJ:
    return "GENERATE_TRAJ";
  case MincoFsm::State::FOLLOW_TRAJ:
    return "FOLLOW_TRAJ";
  case MincoFsm::State::RECOVERING:
    return "RECOVERING";
  // Disabled to prevent zero-velocity deadlock
  // case MincoFsm::State::EMER_STOP:
  //   return "EMER_STOP";
  default:
    return "UNKNOWN";
  }
}

}  // namespace

void MincoFsm::changeState(const char * caller, State new_state)
{
  if (state_ == new_state) {
    return;
  }

  // Leaving recovery-related states: clear debug visualization and reset recovery runtime.
  // Disabled to prevent zero-velocity deadlock
  // if ((state_ == State::RECOVERING || state_ == State::EMER_STOP) && new_state != State::RECOVERING &&
  //     new_state != State::EMER_STOP) {
  if (state_ == State::RECOVERING && new_state != State::RECOVERING) {
    planner_->clearRecoveryDebugVisualization();
    recovery_server_->reset();
  }

  (void)caller;

  // std::cout << "[MincoFSM] [" << (caller ? caller : "?") << "] change state from ["
  //           << StateToString(state_) << "] to [" << StateToString(new_state) << "]" << std::endl;

  last_state_ = state_;
  state_ = new_state;
  // Disabled to prevent zero-velocity deadlock
  // stop_published_ = false;
  // goal_stop_published_ = false;
}

bool MincoFsm::generateRetryDeferred() const
{
  return has_generate_retry_deadline_ &&
         RetryClock::now() < generate_retry_not_before_;
}

void MincoFsm::deferGenerateRetry()
{
  if (failed_replan_retry_period_ <= RetryClock::duration::zero()) {
    clearGenerateRetry();
    return;
  }
  generate_retry_not_before_ = RetryClock::now() + failed_replan_retry_period_;
  has_generate_retry_deadline_ = true;
}

void MincoFsm::clearGenerateRetry()
{
  has_generate_retry_deadline_ = false;
  generate_retry_not_before_ = RetryClock::time_point{};
}

}  // namespace minco_planner
