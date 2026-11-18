#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <future>
#include <thread>

#include "loam_interface/loam_interface.hpp"
#include "sensor_scan_generation/sensor_scan_generation.hpp"
#include "tf2_ros/static_transform_broadcaster.h"

namespace
{
using namespace std::chrono_literals;

class CallbackIsolation : public ::testing::TestWithParam<bool>
{
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
};

TEST_P(CallbackIsolation, OdometryProgressesWhileCloudCallbackGroupIsBlocked)
{
  const bool scan_generation = GetParam();
  rclcpp::NodeOptions options;
  options.parameter_overrides(
    {rclcpp::Parameter("lidar_frame", "isolation_lidar"),
     rclcpp::Parameter("base_frame", "isolation_base"),
     rclcpp::Parameter("robot_base_frame", "isolation_base"),
     rclcpp::Parameter("state_estimation_topic", "isolation_input"),
     rclcpp::Parameter("registered_scan_topic", "isolation_cloud")});
  rclcpp::Node::SharedPtr subject;
  if (scan_generation) {
    subject = std::make_shared<sensor_scan_generation::SensorScanGenerationNode>(options);
  } else {
    subject = std::make_shared<loam_interface::LoamInterfaceNode>(options);
  }
  auto helper = std::make_shared<rclcpp::Node>("callback_isolation_test");
  auto input = helper->create_publisher<nav_msgs::msg::Odometry>(
    scan_generation ? "lidar_odometry" : "isolation_input", 10);
  std::atomic<int> outputs{0};
  auto output = helper->create_subscription<nav_msgs::msg::Odometry>(
    scan_generation ? "odometry" : "lidar_odometry", 10,
    [&](nav_msgs::msg::Odometry::ConstSharedPtr) { ++outputs; });
  tf2_ros::StaticTransformBroadcaster transforms(helper);
  geometry_msgs::msg::TransformStamped mechanical;
  mechanical.header.stamp = helper->now();
  mechanical.header.frame_id = "isolation_base";
  mechanical.child_frame_id = "isolation_lidar";
  mechanical.transform.rotation.w = 1.;
  transforms.sendTransform(mechanical);

  rclcpp::CallbackGroup::SharedPtr cloud_group;
  const std::string cloud_topic = scan_generation ? "/registered_scan" : "/isolation_cloud";
  subject->get_node_base_interface()->for_each_callback_group([&](const auto & group) {
    if (group->find_subscription_ptrs_if(
          [&](const auto & sub) { return sub->get_topic_name() == cloud_topic; })) {
      cloud_group = group;
    }
  });
  ASSERT_NE(cloud_group, nullptr);

  // 阻塞实际点云订阅所属回调组，避免用不稳定的大点云耗时模拟争抢。
  std::promise<void> release;
  auto released = release.get_future().share();
  std::atomic<bool> blocked{false};
  rclcpp::TimerBase::SharedPtr block_timer;
  block_timer = subject->create_wall_timer(
    1ms,
    [&] {
      block_timer->cancel();
      blocked = true;
      released.wait();
    },
    cloud_group);
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(subject);
  executor.add_node(helper);
  std::thread spinner([&] { executor.spin(); });
  struct Cleanup
  {
    std::promise<void> & release;
    rclcpp::Executor & executor;
    std::thread & spinner;
    ~Cleanup()
    {
      release.set_value();
      executor.cancel();
      spinner.join();
    }
  } cleanup{release, executor, spinner};
  const auto wait_for = [](auto predicate) {
    const auto deadline = std::chrono::steady_clock::now() + 3s;
    while (!predicate() && std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(2ms);
    }
    return predicate();
  };
  ASSERT_TRUE(wait_for([&] { return blocked.load() && input->get_subscription_count() > 0; }));
  nav_msgs::msg::Odometry pose;
  pose.header.stamp = helper->now();
  pose.header.frame_id = "isolation_world";
  pose.child_frame_id = "isolation_lidar";
  pose.pose.pose.orientation.w = 1.;
  const auto deadline = std::chrono::steady_clock::now() + 2s;
  while (outputs.load() == 0 && std::chrono::steady_clock::now() < deadline) {
    input->publish(pose);
    std::this_thread::sleep_for(20ms);
  }
  EXPECT_GT(outputs.load(), 0) << "Cloud work blocked chassis feedback";
}

INSTANTIATE_TEST_SUITE_P(Adapters, CallbackIsolation, ::testing::Values(false, true));
}  // namespace
