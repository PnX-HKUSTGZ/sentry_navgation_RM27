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

#ifndef PB2025_NAV_BRINGUP__MINCO_SHADOW_GOAL_RELAY_HPP_
#define PB2025_NAV_BRINGUP__MINCO_SHADOW_GOAL_RELAY_HPP_

#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "behaviortree_cpp/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "lifecycle_msgs/msg/transition_event.hpp"
#include "nav2_msgs/action/compute_path_through_poses.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace pb2025_nav_bringup
{

namespace detail
{

template <typename GoalT, typename GoalHandleT>
struct ShadowDispatchState
{
  std::mutex mutex;
  std::optional<GoalT> in_flight_goal;
  std::optional<GoalT> last_accepted_goal;
  typename GoalHandleT::SharedPtr goal_handle;
  std::uint64_t generation{0U};
  bool server_was_ready{false};
  bool lifecycle_state_known{false};
  bool lifecycle_active{false};
};

}  // namespace detail

class MincoShadowGoalToPoseRelay : public BT::SyncActionNode
{
public:
  MincoShadowGoalToPoseRelay(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();

private:
  BT::NodeStatus tick() override;

  using Action = nav2_msgs::action::ComputePathToPose;
  using GoalHandle = rclcpp_action::ClientGoalHandle<Action>;
  using DispatchState = detail::ShadowDispatchState<geometry_msgs::msg::PoseStamped, GoalHandle>;

  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<Action>::SharedPtr client_;
  std::shared_ptr<DispatchState> dispatch_state_;
  rclcpp::Subscription<lifecycle_msgs::msg::TransitionEvent>::SharedPtr lifecycle_subscription_;
};

class MincoShadowGoalsThroughPosesRelay : public BT::SyncActionNode
{
public:
  MincoShadowGoalsThroughPosesRelay(const std::string & name, const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();

private:
  BT::NodeStatus tick() override;

  using Action = nav2_msgs::action::ComputePathThroughPoses;
  using GoalHandle = rclcpp_action::ClientGoalHandle<Action>;
  using Goals = std::vector<geometry_msgs::msg::PoseStamped>;
  using DispatchState = detail::ShadowDispatchState<Goals, GoalHandle>;

  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<Action>::SharedPtr client_;
  std::shared_ptr<DispatchState> dispatch_state_;
  rclcpp::Subscription<lifecycle_msgs::msg::TransitionEvent>::SharedPtr lifecycle_subscription_;
};

}  // namespace pb2025_nav_bringup

#endif  // PB2025_NAV_BRINGUP__MINCO_SHADOW_GOAL_RELAY_HPP_
