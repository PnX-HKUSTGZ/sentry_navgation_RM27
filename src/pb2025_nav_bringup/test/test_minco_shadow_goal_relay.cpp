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

#include <algorithm>
#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "behaviortree_cpp/bt_factory.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "gtest/gtest.h"
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition_event.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "pb2025_nav_bringup/minco_shadow_goal_relay.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace
{

using namespace std::chrono_literals;

class TestToPoseActionServer
{
public:
  enum class Behavior
  {
    SUCCEED,
    REJECT_FIRST,
    HOLD,
  };

  using Action = nav2_msgs::action::ComputePathToPose;
  using GoalHandle = rclcpp_action::ServerGoalHandle<Action>;

  explicit TestToPoseActionServer(const rclcpp::Node::SharedPtr & node, Behavior behavior)
  : node_(node), behavior_(behavior)
  {
    lifecycle_publisher_ =
      node_->create_publisher<lifecycle_msgs::msg::TransitionEvent>("~/transition_event", 10);
    server_ = rclcpp_action::create_server<Action>(
      node_, "/minco_shadow/compute_path_to_pose",
      [this](const rclcpp_action::GoalUUID &, const std::shared_ptr<const Action::Goal>) {
        const int request_number = ++request_count_;
        if (behavior_ == Behavior::REJECT_FIRST && request_number == 1) {
          return rclcpp_action::GoalResponse::REJECT;
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [this](const std::shared_ptr<GoalHandle>) {
        ++cancel_count_;
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](const std::shared_ptr<GoalHandle> goal_handle) {
        if (behavior_ == Behavior::HOLD) {
          std::lock_guard<std::mutex> lock(goal_handles_mutex_);
          held_goal_handles_.push_back(goal_handle);
          return;
        }
        goal_handle->succeed(std::make_shared<Action::Result>());
      });
  }

  int requestCount() const { return request_count_.load(); }

  int cancelCount() const { return cancel_count_.load(); }

  rclcpp::Node::SharedPtr node() const { return node_; }

  std::string transitionTopic() const
  {
    return node_->get_fully_qualified_name() + std::string("/transition_event");
  }

  bool hasLifecycleSubscriber() const
  {
    return lifecycle_publisher_->get_subscription_count() > 0U;
  }

  void publishLifecycleState(std::uint8_t state_id, const std::string & label)
  {
    lifecycle_msgs::msg::TransitionEvent event;
    event.goal_state.id = state_id;
    event.goal_state.label = label;
    lifecycle_publisher_->publish(event);
  }

private:
  rclcpp::Node::SharedPtr node_;
  Behavior behavior_;
  std::atomic<int> request_count_{0};
  std::atomic<int> cancel_count_{0};
  std::mutex goal_handles_mutex_;
  std::vector<std::shared_ptr<GoalHandle>> held_goal_handles_;
  rclcpp_action::Server<Action>::SharedPtr server_;
  rclcpp::Publisher<lifecycle_msgs::msg::TransitionEvent>::SharedPtr lifecycle_publisher_;
};

class MincoShadowGoalRelayTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }

  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void SetUp() override
  {
    const auto test_index = test_count_.fetch_add(1);
    node_ =
      std::make_shared<rclcpp::Node>("minco_shadow_goal_relay_test_" + std::to_string(test_index));
    blackboard_ = BT::Blackboard::create();
    blackboard_->set("node", node_);
    factory_.registerNodeType<pb2025_nav_bringup::MincoShadowGoalToPoseRelay>(
      "SendMincoShadowGoalToPose");
    factory_.registerNodeType<pb2025_nav_bringup::MincoShadowGoalsThroughPosesRelay>(
      "SendMincoShadowGoalsThroughPoses");

    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
    executor_thread_ = std::thread([this]() { executor_->spin(); });
  }

  void TearDown() override
  {
    executor_->cancel();
    executor_thread_.join();
    executor_->remove_node(node_);
    for (const auto & server_node : server_nodes_) {
      executor_->remove_node(server_node);
    }
  }

  BT::Tree makeToPoseTree(const std::string & lifecycle_topic = "")
  {
    geometry_msgs::msg::PoseStamped goal;
    goal.header.frame_id = "map";
    goal.pose.orientation.w = 1.0;
    blackboard_->set("goal", goal);
    const std::string topic_port =
      lifecycle_topic.empty() ? "" : " lifecycle_transition_topic=\"" + lifecycle_topic + "\"";
    return factory_.createTreeFromText(
      "<root BTCPP_format=\"3\"><BehaviorTree ID=\"MainTree\">"
      "<SendMincoShadowGoalToPose goal=\"{goal}\"" +
        topic_port +
        "/>"
        "</BehaviorTree></root>",
      blackboard_);
  }

  template <typename PredicateT>
  bool tickUntil(BT::Tree & tree, PredicateT predicate, std::chrono::milliseconds timeout)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      if (predicate()) {
        return true;
      }
      EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
      std::this_thread::sleep_for(10ms);
    }
    return predicate();
  }

  template <typename PredicateT>
  bool waitUntil(PredicateT predicate, std::chrono::milliseconds timeout)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      if (predicate()) {
        return true;
      }
      std::this_thread::sleep_for(10ms);
    }
    return predicate();
  }

  std::shared_ptr<TestToPoseActionServer> addToPoseServer(TestToPoseActionServer::Behavior behavior)
  {
    auto server_node = std::make_shared<rclcpp::Node>(
      "minco_shadow_sidecar_test_" + std::to_string(test_count_.fetch_add(1)), "/minco_shadow");
    executor_->add_node(server_node);
    server_nodes_.push_back(server_node);
    return std::make_shared<TestToPoseActionServer>(server_node, behavior);
  }

  void removeToPoseServer(std::shared_ptr<TestToPoseActionServer> & server)
  {
    const auto server_node = server->node();
    executor_->remove_node(server_node);
    server.reset();
    server_nodes_.erase(
      std::remove(server_nodes_.begin(), server_nodes_.end(), server_node), server_nodes_.end());
  }

  rclcpp::Node::SharedPtr node_;
  BT::Blackboard::Ptr blackboard_;
  BT::BehaviorTreeFactory factory_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread executor_thread_;
  std::vector<rclcpp::Node::SharedPtr> server_nodes_;
  static std::atomic<unsigned int> test_count_;
};

std::atomic<unsigned int> MincoShadowGoalRelayTest::test_count_{0U};

TEST_F(MincoShadowGoalRelayTest, ToPoseSucceedsImmediatelyWithoutSidecar)
{
  geometry_msgs::msg::PoseStamped goal;
  goal.header.frame_id = "map";
  goal.pose.orientation.w = 1.0;
  blackboard_->set("goal", goal);
  auto tree = factory_.createTreeFromText(
    R"(<root BTCPP_format="3"><BehaviorTree ID="MainTree">
         <SendMincoShadowGoalToPose goal="{goal}"/>
       </BehaviorTree></root>)",
    blackboard_);

  const auto start = std::chrono::steady_clock::now();
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  const auto elapsed = std::chrono::steady_clock::now() - start;
  EXPECT_LT(elapsed, std::chrono::milliseconds(50));
}

TEST_F(MincoShadowGoalRelayTest, ThroughPosesSucceedsImmediatelyWithoutSidecar)
{
  std::vector<geometry_msgs::msg::PoseStamped> goals(2);
  goals[0].header.frame_id = "map";
  goals[0].pose.orientation.w = 1.0;
  goals[1] = goals[0];
  goals[1].pose.position.x = 1.0;
  blackboard_->set("goals", goals);
  auto tree = factory_.createTreeFromText(
    R"(<root BTCPP_format="3"><BehaviorTree ID="MainTree">
         <SendMincoShadowGoalsThroughPoses goals="{goals}"/>
       </BehaviorTree></root>)",
    blackboard_);

  const auto start = std::chrono::steady_clock::now();
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  const auto elapsed = std::chrono::steady_clock::now() - start;
  EXPECT_LT(elapsed, std::chrono::milliseconds(50));
}

TEST_F(MincoShadowGoalRelayTest, EmptyThroughPosesIsFailOpen)
{
  blackboard_->set("goals", std::vector<geometry_msgs::msg::PoseStamped>{});
  auto tree = factory_.createTreeFromText(
    R"(<root BTCPP_format="3"><BehaviorTree ID="MainTree">
         <SendMincoShadowGoalsThroughPoses goals="{goals}"/>
       </BehaviorTree></root>)",
    blackboard_);

  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
}

TEST_F(MincoShadowGoalRelayTest, GoalUnavailableOnFirstTickIsSentWhenServerAppears)
{
  auto tree = makeToPoseTree();
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);

  const auto server = addToPoseServer(TestToPoseActionServer::Behavior::SUCCEED);
  ASSERT_TRUE(tickUntil(tree, [&server]() { return server->requestCount() == 1; }, 2s));
  ASSERT_TRUE(waitUntil([&server]() { return server->requestCount() == 1; }, 100ms));

  for (int index = 0; index < 10; ++index) {
    EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
    std::this_thread::sleep_for(10ms);
  }
  EXPECT_EQ(server->requestCount(), 1);
}

TEST_F(MincoShadowGoalRelayTest, RejectedGoalIsRetriedButAcceptedGoalIsDeduplicated)
{
  const auto server = addToPoseServer(TestToPoseActionServer::Behavior::REJECT_FIRST);
  auto tree = makeToPoseTree();

  ASSERT_TRUE(tickUntil(tree, [&server]() { return server->requestCount() >= 2; }, 2s));
  EXPECT_EQ(server->requestCount(), 2);

  for (int index = 0; index < 10; ++index) {
    EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
    std::this_thread::sleep_for(10ms);
  }
  EXPECT_EQ(server->requestCount(), 2);
}

TEST_F(MincoShadowGoalRelayTest, ObservedServerReadinessLossRedispatchesTheSameGoal)
{
  using Action = nav2_msgs::action::ComputePathToPose;
  const auto readiness_probe =
    rclcpp_action::create_client<Action>(node_, "/minco_shadow/compute_path_to_pose");
  auto first_server = addToPoseServer(TestToPoseActionServer::Behavior::SUCCEED);
  auto tree = makeToPoseTree();

  ASSERT_TRUE(tickUntil(tree, [&first_server]() { return first_server->requestCount() == 1; }, 2s));
  removeToPoseServer(first_server);

  // Action discovery exposes readiness, not a server identity. Prove the false edge before
  // replacement; restarts whose endpoints overlap are detected by lifecycle transition events.
  ASSERT_TRUE(
    waitUntil([&readiness_probe]() { return !readiness_probe->action_server_is_ready(); }, 5s));
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);

  const auto replacement_server = addToPoseServer(TestToPoseActionServer::Behavior::SUCCEED);
  ASSERT_TRUE(
    waitUntil([&readiness_probe]() { return readiness_probe->action_server_is_ready(); }, 5s));
  EXPECT_TRUE(tickUntil(
    tree, [&replacement_server]() { return replacement_server->requestCount() == 1; }, 2s));
}

TEST_F(MincoShadowGoalRelayTest, RepeatedTicksDoNotStackAnExecutingGoal)
{
  const auto server = addToPoseServer(TestToPoseActionServer::Behavior::HOLD);
  auto tree = makeToPoseTree();

  ASSERT_TRUE(tickUntil(tree, [&server]() { return server->requestCount() == 1; }, 2s));
  for (int index = 0; index < 20; ++index) {
    EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
    std::this_thread::sleep_for(10ms);
  }
  EXPECT_EQ(server->requestCount(), 1);
}

TEST_F(MincoShadowGoalRelayTest, LifecycleReactivationRedispatchesTheSameGoal)
{
  const auto server = addToPoseServer(TestToPoseActionServer::Behavior::SUCCEED);
  auto tree = makeToPoseTree(server->transitionTopic());

  ASSERT_TRUE(waitUntil([&server]() { return server->hasLifecycleSubscriber(); }, 2s));
  ASSERT_TRUE(tickUntil(tree, [&server]() { return server->requestCount() == 1; }, 2s));

  server->publishLifecycleState(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, "inactive");
  std::this_thread::sleep_for(50ms);
  for (int index = 0; index < 5; ++index) {
    EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
  }
  EXPECT_EQ(server->requestCount(), 1);

  server->publishLifecycleState(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, "active");
  ASSERT_TRUE(tickUntil(tree, [&server]() { return server->requestCount() == 2; }, 2s));

  for (int index = 0; index < 10; ++index) {
    EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::SUCCESS);
    std::this_thread::sleep_for(10ms);
  }
  EXPECT_EQ(server->requestCount(), 2);
}

}  // namespace
