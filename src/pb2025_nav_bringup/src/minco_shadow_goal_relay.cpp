// Copyright 2026 RM Navigation Contributors
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

#include "pb2025_nav_bringup/minco_shadow_goal_relay.hpp"

#include <cmath>
#include <cstdint>
#include <exception>
#include <memory>
#include <optional>
#include <string>
#include <utility>

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/json_export.h"
#include "lifecycle_msgs/msg/state.hpp"
#include "nav2_behavior_tree/bt_utils.hpp"
#include "nav2_behavior_tree/json_utils.hpp"

namespace pb2025_nav_bringup
{
namespace
{

constexpr char kToPoseActionName[] = "/minco_shadow/compute_path_to_pose";
constexpr char kThroughPosesActionName[] = "/minco_shadow/compute_path_through_poses";
constexpr char kDefaultLifecycleTransitionTopic[] = "/minco_shadow/planner_server/transition_event";
constexpr double kPoseComparisonTolerance = 1.0e-9;

bool nearlyEqual(double lhs, double rhs) { return std::abs(lhs - rhs) <= kPoseComparisonTolerance; }

bool sameGoal(
  const geometry_msgs::msg::PoseStamped & lhs, const geometry_msgs::msg::PoseStamped & rhs)
{
  return lhs.header.frame_id == rhs.header.frame_id &&
         nearlyEqual(lhs.pose.position.x, rhs.pose.position.x) &&
         nearlyEqual(lhs.pose.position.y, rhs.pose.position.y) &&
         nearlyEqual(lhs.pose.position.z, rhs.pose.position.z) &&
         nearlyEqual(lhs.pose.orientation.x, rhs.pose.orientation.x) &&
         nearlyEqual(lhs.pose.orientation.y, rhs.pose.orientation.y) &&
         nearlyEqual(lhs.pose.orientation.z, rhs.pose.orientation.z) &&
         nearlyEqual(lhs.pose.orientation.w, rhs.pose.orientation.w);
}

bool sameGoals(
  const std::vector<geometry_msgs::msg::PoseStamped> & lhs,
  const std::vector<geometry_msgs::msg::PoseStamped> & rhs)
{
  if (lhs.size() != rhs.size()) {
    return false;
  }
  for (std::size_t index = 0; index < lhs.size(); ++index) {
    if (!sameGoal(lhs[index], rhs[index])) {
      return false;
    }
  }
  return true;
}

template <typename GoalHandleT>
void logShadowResult(
  const rclcpp::Logger & logger, const typename GoalHandleT::WrappedResult & result)
{
  if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_DEBUG(logger, "MINCO shadow planning request completed.");
    return;
  }
  RCLCPP_WARN(
    logger,
    "MINCO shadow planning request ended with action result code %d; legacy navigation "
    "is unaffected.",
    static_cast<int>(result.code));
}

template <typename ActionT>
void cancelGoalBestEffort(
  const std::weak_ptr<rclcpp_action::Client<ActionT>> & weak_client,
  const typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr & goal_handle,
  const rclcpp::Logger & logger)
{
  if (!goal_handle) {
    return;
  }
  const auto client = weak_client.lock();
  if (!client) {
    return;
  }
  try {
    (void)client->async_cancel_goal(goal_handle);
  } catch (const std::exception & exception) {
    RCLCPP_DEBUG(logger, "Could not cancel a superseded MINCO shadow goal: %s", exception.what());
  }
}

template <typename ActionT, typename DispatchStateT>
rclcpp::Subscription<lifecycle_msgs::msg::TransitionEvent>::SharedPtr createLifecycleSubscription(
  const rclcpp::Node::SharedPtr & node,
  const std::shared_ptr<rclcpp_action::Client<ActionT>> & client,
  const std::shared_ptr<DispatchStateT> & state, const std::string & topic)
{
  const auto weak_state = std::weak_ptr<DispatchStateT>(state);
  const auto weak_client = std::weak_ptr<rclcpp_action::Client<ActionT>>(client);
  const auto logger = node->get_logger();
  return node->create_subscription<lifecycle_msgs::msg::TransitionEvent>(
    topic, rclcpp::QoS(10).reliable(),
    [weak_state, weak_client,
     logger](const lifecycle_msgs::msg::TransitionEvent::ConstSharedPtr event) {
      const auto state = weak_state.lock();
      if (!state) {
        return;
      }

      const bool active = event->goal_state.id == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
      typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr superseded_goal;
      {
        std::lock_guard<std::mutex> lock(state->mutex);
        superseded_goal = state->goal_handle;
        ++state->generation;
        state->in_flight_goal.reset();
        state->last_accepted_goal.reset();
        state->goal_handle.reset();
        state->server_was_ready = false;
        state->lifecycle_state_known = true;
        state->lifecycle_active = active;
      }

      cancelGoalBestEffort<ActionT>(weak_client, superseded_goal, logger);
      RCLCPP_DEBUG(
        logger, "MINCO shadow sidecar lifecycle changed to '%s' (%u); dispatch cache invalidated.",
        event->goal_state.label.c_str(), event->goal_state.id);
    });
}

template <typename ActionT, typename DispatchStateT>
void handleGoalResponse(
  const std::weak_ptr<DispatchStateT> & weak_state,
  const std::weak_ptr<rclcpp_action::Client<ActionT>> & weak_client, std::uint64_t generation,
  const typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr & goal_handle,
  const rclcpp::Logger & logger)
{
  const auto state = weak_state.lock();
  if (!state) {
    cancelGoalBestEffort<ActionT>(weak_client, goal_handle, logger);
    return;
  }

  bool stale = false;
  {
    std::lock_guard<std::mutex> lock(state->mutex);
    stale = generation != state->generation || !state->in_flight_goal.has_value();
    if (!stale && goal_handle) {
      state->goal_handle = goal_handle;
      state->last_accepted_goal = state->in_flight_goal;
    } else if (!stale) {
      state->in_flight_goal.reset();
      state->last_accepted_goal.reset();
      state->goal_handle.reset();
    }
  }

  if (stale) {
    cancelGoalBestEffort<ActionT>(weak_client, goal_handle, logger);
    return;
  }
  if (!goal_handle) {
    RCLCPP_WARN(
      logger,
      "MINCO shadow sidecar rejected the planning request; the next BT tick will retry. "
      "Legacy navigation is unaffected.");
  }
}

template <typename GoalHandleT, typename DispatchStateT>
void handleGoalResult(
  const std::weak_ptr<DispatchStateT> & weak_state, std::uint64_t generation,
  const typename GoalHandleT::WrappedResult & result, const rclcpp::Logger & logger)
{
  const auto state = weak_state.lock();
  if (!state) {
    return;
  }

  bool current = false;
  {
    std::lock_guard<std::mutex> lock(state->mutex);
    current = generation == state->generation;
    if (current) {
      state->in_flight_goal.reset();
      state->goal_handle.reset();
      if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
        state->last_accepted_goal.reset();
      }
    }
  }

  if (current) {
    logShadowResult<GoalHandleT>(logger, result);
  }
}

template <typename DispatchStateT>
void rollBackFailedSend(const std::shared_ptr<DispatchStateT> & state, std::uint64_t generation)
{
  std::lock_guard<std::mutex> lock(state->mutex);
  if (generation == state->generation) {
    state->in_flight_goal.reset();
    state->last_accepted_goal.reset();
    state->goal_handle.reset();
  }
}

}  // namespace

MincoShadowGoalToPoseRelay::MincoShadowGoalToPoseRelay(
  const std::string & name, const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config),
  node_(config.blackboard->get<rclcpp::Node::SharedPtr>("node")),
  client_(rclcpp_action::create_client<Action>(node_, kToPoseActionName)),
  dispatch_state_(std::make_shared<DispatchState>())
{
  const auto lifecycle_topic =
    getInput<std::string>("lifecycle_transition_topic").value_or(kDefaultLifecycleTransitionTopic);
  lifecycle_subscription_ =
    createLifecycleSubscription<Action>(node_, client_, dispatch_state_, lifecycle_topic);
}

BT::PortsList MincoShadowGoalToPoseRelay::providedPorts()
{
  BT::RegisterJsonDefinition<geometry_msgs::msg::PoseStamped>();
  return {
    BT::InputPort<geometry_msgs::msg::PoseStamped>("goal", "Navigation goal to mirror"),
    BT::InputPort<std::string>("planner_id", "MincoPlanner", "Shadow planner plugin ID"),
    BT::InputPort<std::string>(
      "lifecycle_transition_topic", kDefaultLifecycleTransitionTopic,
      "Shadow planner lifecycle transition topic"),
  };
}

BT::NodeStatus MincoShadowGoalToPoseRelay::tick()
{
  const auto goal_input = getInput<geometry_msgs::msg::PoseStamped>("goal");
  if (!goal_input) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000,
      "MINCO shadow goal relay has no valid goal: %s", goal_input.error().c_str());
    return BT::NodeStatus::SUCCESS;
  }

  const auto & goal_pose = goal_input.value();
  const bool server_ready = client_->action_server_is_ready();
  GoalHandle::SharedPtr superseded_goal;
  std::optional<std::uint64_t> generation;
  bool lifecycle_blocked = false;
  {
    std::lock_guard<std::mutex> lock(dispatch_state_->mutex);
    if (!server_ready) {
      if (
        dispatch_state_->server_was_ready || dispatch_state_->in_flight_goal ||
        dispatch_state_->last_accepted_goal) {
        superseded_goal = dispatch_state_->goal_handle;
        ++dispatch_state_->generation;
        dispatch_state_->in_flight_goal.reset();
        dispatch_state_->last_accepted_goal.reset();
        dispatch_state_->goal_handle.reset();
      }
      dispatch_state_->server_was_ready = false;
      dispatch_state_->lifecycle_state_known = false;
      dispatch_state_->lifecycle_active = false;
    } else {
      dispatch_state_->server_was_ready = true;
      lifecycle_blocked =
        dispatch_state_->lifecycle_state_known && !dispatch_state_->lifecycle_active;
      if (
        !lifecycle_blocked && !dispatch_state_->in_flight_goal &&
        (!dispatch_state_->last_accepted_goal ||
         !sameGoal(*dispatch_state_->last_accepted_goal, goal_pose))) {
        dispatch_state_->in_flight_goal = goal_pose;
        dispatch_state_->goal_handle.reset();
        generation = ++dispatch_state_->generation;
      }
    }
  }

  cancelGoalBestEffort<Action>(client_, superseded_goal, node_->get_logger());
  if (!server_ready) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000,
      "%s is unavailable; skipping this shadow tick. Legacy navigation remains authoritative.",
      kToPoseActionName);
    return BT::NodeStatus::SUCCESS;
  }
  if (lifecycle_blocked) {
    RCLCPP_DEBUG_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000,
      "MINCO shadow sidecar is not active; deferring goal dispatch.");
    return BT::NodeStatus::SUCCESS;
  }
  if (!generation) {
    return BT::NodeStatus::SUCCESS;
  }

  Action::Goal action_goal;
  action_goal.goal = goal_pose;
  action_goal.planner_id = getInput<std::string>("planner_id").value_or("MincoPlanner");
  action_goal.use_start = false;

  typename rclcpp_action::Client<Action>::SendGoalOptions options;
  const auto logger = node_->get_logger();
  const auto weak_state = std::weak_ptr<DispatchState>(dispatch_state_);
  const auto weak_client = std::weak_ptr<rclcpp_action::Client<Action>>(client_);
  options.goal_response_callback = [weak_state, weak_client, generation = *generation,
                                    logger](const GoalHandle::SharedPtr & goal_handle) {
    handleGoalResponse<Action>(weak_state, weak_client, generation, goal_handle, logger);
  };
  options.result_callback = [weak_state, generation = *generation,
                             logger](const GoalHandle::WrappedResult & result) {
    handleGoalResult<GoalHandle>(weak_state, generation, result, logger);
  };

  try {
    (void)client_->async_send_goal(action_goal, options);
  } catch (const std::exception & exception) {
    rollBackFailedSend(dispatch_state_, *generation);
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000,
      "Failed to dispatch MINCO shadow goal: %s. Legacy navigation is unaffected.",
      exception.what());
  }
  return BT::NodeStatus::SUCCESS;
}

MincoShadowGoalsThroughPosesRelay::MincoShadowGoalsThroughPosesRelay(
  const std::string & name, const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config),
  node_(config.blackboard->get<rclcpp::Node::SharedPtr>("node")),
  client_(rclcpp_action::create_client<Action>(node_, kThroughPosesActionName)),
  dispatch_state_(std::make_shared<DispatchState>())
{
  const auto lifecycle_topic =
    getInput<std::string>("lifecycle_transition_topic").value_or(kDefaultLifecycleTransitionTopic);
  lifecycle_subscription_ =
    createLifecycleSubscription<Action>(node_, client_, dispatch_state_, lifecycle_topic);
}

BT::PortsList MincoShadowGoalsThroughPosesRelay::providedPorts()
{
  BT::RegisterJsonDefinition<std::vector<geometry_msgs::msg::PoseStamped>>();
  return {
    BT::InputPort<std::vector<geometry_msgs::msg::PoseStamped>>(
      "goals", "Navigation goals to mirror"),
    BT::InputPort<std::string>("planner_id", "MincoPlanner", "Shadow planner plugin ID"),
    BT::InputPort<std::string>(
      "lifecycle_transition_topic", kDefaultLifecycleTransitionTopic,
      "Shadow planner lifecycle transition topic"),
  };
}

BT::NodeStatus MincoShadowGoalsThroughPosesRelay::tick()
{
  const auto goals_input = getInput<std::vector<geometry_msgs::msg::PoseStamped>>("goals");
  if (!goals_input || goals_input.value().empty()) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000,
      "MINCO shadow goals relay has no valid poses; legacy navigation continues.");
    return BT::NodeStatus::SUCCESS;
  }

  const auto & goal_poses = goals_input.value();
  const bool server_ready = client_->action_server_is_ready();
  GoalHandle::SharedPtr superseded_goal;
  std::optional<std::uint64_t> generation;
  bool lifecycle_blocked = false;
  {
    std::lock_guard<std::mutex> lock(dispatch_state_->mutex);
    if (!server_ready) {
      if (
        dispatch_state_->server_was_ready || dispatch_state_->in_flight_goal ||
        dispatch_state_->last_accepted_goal) {
        superseded_goal = dispatch_state_->goal_handle;
        ++dispatch_state_->generation;
        dispatch_state_->in_flight_goal.reset();
        dispatch_state_->last_accepted_goal.reset();
        dispatch_state_->goal_handle.reset();
      }
      dispatch_state_->server_was_ready = false;
      dispatch_state_->lifecycle_state_known = false;
      dispatch_state_->lifecycle_active = false;
    } else {
      dispatch_state_->server_was_ready = true;
      lifecycle_blocked =
        dispatch_state_->lifecycle_state_known && !dispatch_state_->lifecycle_active;
      if (
        !lifecycle_blocked && !dispatch_state_->in_flight_goal &&
        (!dispatch_state_->last_accepted_goal ||
         !sameGoals(*dispatch_state_->last_accepted_goal, goal_poses))) {
        dispatch_state_->in_flight_goal = goal_poses;
        dispatch_state_->goal_handle.reset();
        generation = ++dispatch_state_->generation;
      }
    }
  }

  cancelGoalBestEffort<Action>(client_, superseded_goal, node_->get_logger());
  if (!server_ready) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000,
      "%s is unavailable; skipping this shadow tick. Legacy navigation remains authoritative.",
      kThroughPosesActionName);
    return BT::NodeStatus::SUCCESS;
  }
  if (lifecycle_blocked) {
    RCLCPP_DEBUG_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000,
      "MINCO shadow sidecar is not active; deferring goals dispatch.");
    return BT::NodeStatus::SUCCESS;
  }
  if (!generation) {
    return BT::NodeStatus::SUCCESS;
  }

  Action::Goal action_goal;
  action_goal.goals = goal_poses;
  action_goal.planner_id = getInput<std::string>("planner_id").value_or("MincoPlanner");
  action_goal.use_start = false;

  typename rclcpp_action::Client<Action>::SendGoalOptions options;
  const auto logger = node_->get_logger();
  const auto weak_state = std::weak_ptr<DispatchState>(dispatch_state_);
  const auto weak_client = std::weak_ptr<rclcpp_action::Client<Action>>(client_);
  options.goal_response_callback = [weak_state, weak_client, generation = *generation,
                                    logger](const GoalHandle::SharedPtr & goal_handle) {
    handleGoalResponse<Action>(weak_state, weak_client, generation, goal_handle, logger);
  };
  options.result_callback = [weak_state, generation = *generation,
                             logger](const GoalHandle::WrappedResult & result) {
    handleGoalResult<GoalHandle>(weak_state, generation, result, logger);
  };

  try {
    (void)client_->async_send_goal(action_goal, options);
  } catch (const std::exception & exception) {
    rollBackFailedSend(dispatch_state_, *generation);
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000,
      "Failed to dispatch MINCO shadow goals: %s. Legacy navigation is unaffected.",
      exception.what());
  }
  return BT::NodeStatus::SUCCESS;
}

}  // namespace pb2025_nav_bringup

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<pb2025_nav_bringup::MincoShadowGoalToPoseRelay>(
    "SendMincoShadowGoalToPose");
  factory.registerNodeType<pb2025_nav_bringup::MincoShadowGoalsThroughPosesRelay>(
    "SendMincoShadowGoalsThroughPoses");
}
