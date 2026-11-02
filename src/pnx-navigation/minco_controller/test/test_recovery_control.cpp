#include <gtest/gtest.h>
#include <thread>
#include "minco_controller/minco_mpc_controller.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "tf2/LinearMath/Quaternion.h"

namespace {
class TestCostmap : public nav2_costmap_2d::Costmap2DROS {
public:
  TestCostmap() : Costmap2DROS("recovery_control_costmap") {
    global_frame_ = "odom";
    robot_base_frame_ = "base_link";
  }
};

TEST(RecoveryControl, BoundedOutputBypassesAssistanceAndRejectsUnsafeCommand) {
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
    "recovery_control_test", rclcpp::NodeOptions().parameter_overrides({
      rclcpp::Parameter("FollowPath.performance.enable", false),
      rclcpp::Parameter("FollowPath.reference_startup_min_command_speed", 0.6),
      rclcpp::Parameter("FollowPath.uphill_startup_min_command_speed", 1.0),
      rclcpp::Parameter("FollowPath.fixed_wz", 2.0),
      rclcpp::Parameter("FollowPath.opt_path_topic", "/recovery_control_test/path"),
      rclcpp::Parameter("FollowPath.odom_topic", "/recovery_control_test/odom")
    }));
  auto costmap = std::make_shared<TestCostmap>();
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  minco_controller::MincoMpcController controller;
  controller.configure(node, "FollowPath", tf, costmap);
  controller.activate();
  auto publisher_node = std::make_shared<rclcpp::Node>("recovery_control_inputs");
  auto path_pub = publisher_node->create_publisher<ros_interfaces::msg::MpcPositionCommand>(
    "/recovery_control_test/path", 1);
  auto odom_pub = publisher_node->create_publisher<nav_msgs::msg::Odometry>(
    "/recovery_control_test/odom", rclcpp::SensorDataQoS());
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.add_node(publisher_node);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while ((path_pub->get_subscription_count() == 0U || odom_pub->get_subscription_count() == 0U) &&
    std::chrono::steady_clock::now() < deadline) {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  EXPECT_GT(path_pub->get_subscription_count(), 0U);
  nav_msgs::msg::Path plan;
  plan.header.frame_id = "odom";
  plan.header.stamp = node->now();
  plan.poses.resize(1U);
  plan.poses.front().header = plan.header;
  plan.poses.front().pose.position.x = -2.0;
  plan.poses.front().pose.orientation.w = 1.0;
  controller.setPlan(plan);
  ros_interfaces::msg::MpcPositionCommand command;
  command.header = plan.header;
  command.planning_stamp = plan.header.stamp;
  command.command_flag = ros_interfaces::msg::MpcPositionCommand::RECOVERY_COMMAND;
  command.mpc_horizon = 21U;
  command.cmds.resize(21U);
  for (size_t i = 0U; i < command.cmds.size(); ++i) {
    command.cmds[i].position.x = -0.01 * i;
    command.cmds[i].velocity.x = -0.2;
    command.cmds[i].trajectory_id = 1U;
  }
  nav_msgs::msg::Odometry odom;
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_link";
  tf2::Quaternion q;
  q.setRPY(0.0, 0.20, 0.0);
  odom.pose.pose.orientation.x = q.x();
  odom.pose.pose.orientation.y = q.y();
  odom.pose.pose.orientation.z = q.z();
  odom.pose.pose.orientation.w = q.w();
  const auto deliver = [&]() {
    command.header.stamp = node->now();
    odom.header.stamp = command.header.stamp;
    path_pub->publish(command);
    odom_pub->publish(odom);
    for (int i = 0; i < 4; ++i) {
      executor.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  };
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "odom";
  pose.pose.orientation.w = 1.0;
  deliver();
  auto output = controller.computeVelocityCommands(pose, geometry_msgs::msg::Twist{}, nullptr);
  EXPECT_NEAR(output.twist.linear.x, -0.2, 1.0e-6);
  EXPECT_NEAR(output.twist.linear.y, 0.0, 1.0e-6);
  EXPECT_NEAR(output.twist.angular.z, 0.0, 1.0e-6);
  // A rotating lidar reports a different yaw. Recovery follows chassis TF.
  odom.child_frame_id = "lidar_link";
  q.setRPY(0.0, 0.20, 0.60);
  odom.pose.pose.orientation.x = q.x();
  odom.pose.pose.orientation.y = q.y();
  odom.pose.pose.orientation.z = q.z();
  odom.pose.pose.orientation.w = q.w();
  geometry_msgs::msg::TransformStamped physical;
  physical.header.frame_id = "odom";
  physical.child_frame_id = "base_link";
  physical.header.stamp = node->now();
  physical.transform.rotation.w = 1.0;
  ASSERT_TRUE(tf->setTransform(physical, "recovery_test", false));
  deliver();
  output = controller.computeVelocityCommands(pose, geometry_msgs::msg::Twist{}, nullptr);
  EXPECT_NEAR(output.twist.linear.x, -0.2, 1.0e-6);
  EXPECT_NEAR(output.twist.angular.z, 0.0, 1.0e-6);
  odom.child_frame_id = "base_link";
  q.setRPY(0.0, 0.20, 0.0);
  odom.pose.pose.orientation.x = q.x();
  odom.pose.pose.orientation.y = q.y();
  odom.pose.pose.orientation.z = q.z();
  odom.pose.pose.orientation.w = q.w();
  for (auto & point : command.cmds) { point.velocity.x = -0.6; }
  deliver();
  output = controller.computeVelocityCommands(pose, geometry_msgs::msg::Twist{}, nullptr);
  EXPECT_DOUBLE_EQ(output.twist.linear.x, 0.0);
  for (auto & point : command.cmds) { point.velocity.x = -0.2; }
  odom.pose.pose.position.x = -0.201;
  deliver();
  output = controller.computeVelocityCommands(pose, geometry_msgs::msg::Twist{}, nullptr);
  EXPECT_DOUBLE_EQ(output.twist.linear.x, 0.0);
  // A verified braking replacement must not be accelerated back to the
  // startup/uphill floors or inherit the configured gyro rotation.
  odom.pose.pose.position.x = 0.0;
  command.command_flag = ros_interfaces::msg::MpcPositionCommand::BRAKING_COMMAND;
  for (size_t i = 0U; i < command.cmds.size(); ++i) {
    const double t = 0.05 * i;
    command.cmds[i].trajectory_id = 2U;
    command.cmds[i].position.x = -0.4 * (t - 0.5 * t * t);
    command.cmds[i].velocity.x = -0.4 * (1.0 - t);
    command.cmds[i].acceleration.x = 0.4;
  }
  deliver();
  output = controller.computeVelocityCommands(pose, geometry_msgs::msg::Twist{}, nullptr);
  EXPECT_GT(std::abs(output.twist.linear.x), 0.01);
  EXPECT_LT(std::hypot(output.twist.linear.x, output.twist.linear.y), 0.6);
  EXPECT_DOUBLE_EQ(output.twist.angular.z, 0.0);
  command.command_flag = ros_interfaces::msg::MpcPositionCommand::BLOCK_COMMAND;
  deliver();
  output = controller.computeVelocityCommands(pose, geometry_msgs::msg::Twist{}, nullptr);
  EXPECT_DOUBLE_EQ(output.twist.linear.x, 0.0);
  controller.deactivate();
  controller.cleanup();
  executor.remove_node(node->get_node_base_interface());
  executor.remove_node(publisher_node);
  rclcpp::shutdown();
}
}  // namespace
