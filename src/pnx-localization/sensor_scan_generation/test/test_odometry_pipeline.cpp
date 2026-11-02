#include <gtest/gtest.h>

#include <chrono>
#include <functional>
#include <limits>
#include <thread>
#include <vector>

#include "sensor_scan_generation/sensor_scan_generation.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace
{
using namespace std::chrono_literals;

class OdometryPipeline : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {rclcpp::init(0, nullptr);}
  }

  static void TearDownTestSuite() {rclcpp::shutdown();}

  void SetUp() override
  {
    if (!rclcpp::ok()) {rclcpp::init(0, nullptr);}
    helper_ = std::make_shared<rclcpp::Node>("scan_pipeline_test");
    rclcpp::NodeOptions options;
    options.parameter_overrides({
      rclcpp::Parameter("lidar_frame", "test_lidar"),
      rclcpp::Parameter("base_frame", "test_chassis"),
      rclcpp::Parameter("robot_base_frame", "test_chassis")});
    subject_ = std::make_shared<sensor_scan_generation::SensorScanGenerationNode>(options);
    input_ = helper_->create_publisher<nav_msgs::msg::Odometry>("lidar_odometry", 10);
    cloud_ = helper_->create_publisher<sensor_msgs::msg::PointCloud2>("registered_scan", 10);
    output_ = helper_->create_subscription<nav_msgs::msg::Odometry>(
      "odometry", 10, [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        received_.push_back(*msg);
      });
    transforms_ = std::make_unique<tf2_ros::TransformBroadcaster>(*helper_);
    executor_.add_node(helper_);
    executor_.add_node(subject_);
    ASSERT_TRUE(waitFor([this] {return input_->get_subscription_count() > 0;}, 2s));
  }

  void TearDown() override
  {
    executor_.remove_node(subject_);
    executor_.remove_node(helper_);
    subject_.reset();
    helper_.reset();
  }

  bool waitFor(const std::function<bool()> & predicate, std::chrono::milliseconds timeout)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      executor_.spin_some();
      if (predicate()) {return true;}
      std::this_thread::sleep_for(2ms);
    }
    return predicate();
  }

  nav_msgs::msg::Odometry sensorPose(double yaw, bool publish_transform = true)
  {
    // A stationary chassis with an offset, rotating, tilted sensor.
    tf2::Quaternion turn, tilt;
    turn.setRPY(0.0, 0.0, yaw);
    tilt.setRPY(-0.523598775598, 0.0, 0.0);
    const tf2::Transform chassis_from_sensor(
      turn * tilt, tf2::quatRotate(turn, tf2::Vector3(0.0, 0.126, 0.13)));
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = helper_->now();
    odom.header.frame_id = "test_world";
    odom.child_frame_id = "test_lidar";
    const tf2::Transform world_from_chassis(tf2::Quaternion::getIdentity(), {2.0, 3.0, 0.08});
    tf2::toMsg(world_from_chassis * chassis_from_sensor, odom.pose.pose);
    if (publish_transform) {
      geometry_msgs::msg::TransformStamped transform;
      transform.header.stamp = odom.header.stamp;
      transform.header.frame_id = "test_chassis";
      transform.child_frame_id = "test_lidar";
      transform.transform = tf2::toMsg(chassis_from_sensor);
      transforms_->sendTransform(transform);
      // Let the independent TF listener receive this exact-time joint sample.
      waitFor([] {return false;}, 25ms);
    }
    return odom;
  }

  rclcpp::Node::SharedPtr helper_;
  std::shared_ptr<sensor_scan_generation::SensorScanGenerationNode> subject_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr input_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr output_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> transforms_;
  std::vector<nav_msgs::msg::Odometry> received_;
};

TEST_F(OdometryPipeline, RotatingSensorPublishesStationaryChassisWithoutAnyCloud)
{
  for (double yaw : {0.0, 0.5, 1.0, 1.5}) {
    const auto odom = sensorPose(yaw);
    const auto before = received_.size();
    input_->publish(odom);
    ASSERT_TRUE(waitFor([&] {return received_.size() > before;}, 1s));
    const auto & out = received_.back();
    EXPECT_EQ(out.header.stamp, odom.header.stamp);
    EXPECT_EQ(out.child_frame_id, "test_chassis");
    EXPECT_NEAR(out.pose.pose.position.x, 2.0, 1.0e-6);
    EXPECT_NEAR(out.pose.pose.position.y, 3.0, 1.0e-6);
    EXPECT_NEAR(out.twist.twist.linear.x, 0.0, 1.0e-5);
    EXPECT_NEAR(out.twist.twist.linear.y, 0.0, 1.0e-5);
    EXPECT_NEAR(out.twist.twist.angular.z, 0.0, 1.0e-5);
  }
}

TEST_F(OdometryPipeline, DelayedCloudAndDuplicatePoseCannotRewindChassisFeedback)
{
  const auto first = sensorPose(0.0);
  input_->publish(first);
  ASSERT_TRUE(waitFor([&] {return received_.size() == 1;}, 1s));
  const auto second = sensorPose(0.5);
  input_->publish(second);
  ASSERT_TRUE(waitFor([&] {return received_.size() == 2;}, 1s));
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header = first.header;
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(1);
  cloud_->publish(cloud);
  input_->publish(first);
  input_->publish(second);
  waitFor([] {return false;}, 150ms);
  ASSERT_EQ(received_.size(), 2U);
  auto invalid = second;
  invalid.header.stamp = helper_->now();
  invalid.pose.pose.position.x = std::numeric_limits<double>::quiet_NaN();
  input_->publish(invalid);
  waitFor([] {return false;}, 30ms);
  input_->publish(first);
  waitFor([] {return false;}, 30ms);
  EXPECT_EQ(received_.size(), 2U);
  EXPECT_EQ(received_.back().header.stamp, second.header.stamp);
}

TEST_F(OdometryPipeline, MissingJointTransformDoesNotPublishFabricatedFeedback)
{
  input_->publish(sensorPose(0.5, false));
  waitFor([] {return false;}, 600ms);
  EXPECT_TRUE(received_.empty());
  const auto valid = sensorPose(0.8);
  input_->publish(valid);
  ASSERT_TRUE(waitFor([&] {return !received_.empty();}, 1s));
  EXPECT_EQ(received_.back().header.stamp, valid.header.stamp);
  EXPECT_DOUBLE_EQ(received_.back().twist.twist.linear.x, 0.0);
}
}  // namespace
