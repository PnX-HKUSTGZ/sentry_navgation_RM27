#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "example_interfaces/msg/float32.hpp"
#include "fake_vel_transform/fake_vel_transform.hpp"
#include "fake_vel_transform/odom_validation.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace
{

builtin_interfaces::msg::Time timeFromSeconds(double seconds)
{
  return rclcpp::Time(static_cast<int64_t>(seconds * 1e9), RCL_ROS_TIME);
}

nav_msgs::msg::Odometry makeOdometry(double yaw, const builtin_interfaces::msg::Time & stamp)
{
  nav_msgs::msg::Odometry odometry;
  odometry.header.stamp = stamp;
  odometry.pose.pose.orientation.z = std::sin(yaw * 0.5);
  odometry.pose.pose.orientation.w = std::cos(yaw * 0.5);
  return odometry;
}

class FakeVelTransformFixture : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void createNodes(
    bool use_latest, double odom_timeout = 0.5,
    const std::vector<rclcpp::Parameter> & extra_overrides = {})
  {
    static std::atomic<unsigned int> sequence{0U};
    const std::string suffix = std::to_string(sequence.fetch_add(1U));
    odom_topic_ = "/fake_vel_test/odom_" + suffix;
    path_topic_ = "/fake_vel_test/path_" + suffix;
    input_topic_ = "/fake_vel_test/input_" + suffix;
    output_topic_ = "/fake_vel_test/output_" + suffix;
    spin_topic_ = "/fake_vel_test/spin_" + suffix;
    terrain_topic_ = "/fake_vel_test/terrain_" + suffix;

    rclcpp::NodeOptions options;
    std::vector<rclcpp::Parameter> parameter_overrides{
      rclcpp::Parameter("use_latest_odom_for_cmd", use_latest),
      rclcpp::Parameter("odom_timeout", odom_timeout),
      rclcpp::Parameter("odom_future_tolerance", 0.05),
      rclcpp::Parameter("odom_topic", odom_topic_),
      rclcpp::Parameter("local_plan_topic", path_topic_),
      rclcpp::Parameter("terrain_map_topic", terrain_topic_),
      rclcpp::Parameter("cmd_spin_topic", spin_topic_),
      rclcpp::Parameter("input_cmd_vel_topic", input_topic_),
      rclcpp::Parameter("output_cmd_vel_topic", output_topic_)};
    parameter_overrides.insert(
      parameter_overrides.end(), extra_overrides.begin(), extra_overrides.end());
    options.parameter_overrides(parameter_overrides);
    transform_node_ = std::make_shared<fake_vel_transform::FakeVelTransform>(options);
    test_node_ = std::make_shared<rclcpp::Node>("fake_vel_transform_test_" + suffix);

    odom_pub_ = test_node_->create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 10);
    path_pub_ = test_node_->create_publisher<nav_msgs::msg::Path>(path_topic_, 10);
    cmd_pub_ = test_node_->create_publisher<geometry_msgs::msg::Twist>(input_topic_, 10);
    spin_pub_ = test_node_->create_publisher<example_interfaces::msg::Float32>(spin_topic_, 10);
    terrain_pub_ = test_node_->create_publisher<sensor_msgs::msg::PointCloud2>(
      terrain_topic_, rclcpp::SensorDataQoS().keep_last(1));
    output_sub_ = test_node_->create_subscription<geometry_msgs::msg::Twist>(
      output_topic_, 10,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) { outputs_.push_back(*msg); });

    executor_.add_node(transform_node_);
    executor_.add_node(test_node_);
    ASSERT_TRUE(spinUntil(
      [this]() {
        return odom_pub_->get_subscription_count() > 0U &&
               path_pub_->get_subscription_count() > 0U &&
               cmd_pub_->get_subscription_count() > 0U && output_sub_->get_publisher_count() > 0U;
      },
      std::chrono::seconds(2)));
  }

  void TearDown() override
  {
    executor_.cancel();
    if (transform_node_) {
      executor_.remove_node(transform_node_);
    }
    if (test_node_) {
      executor_.remove_node(test_node_);
    }
    output_sub_.reset();
    odom_pub_.reset();
    path_pub_.reset();
    cmd_pub_.reset();
    spin_pub_.reset();
    terrain_pub_.reset();
    transform_node_.reset();
    test_node_.reset();
  }

  template <typename PredicateT, typename DurationT>
  bool spinUntil(PredicateT predicate, DurationT timeout)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      executor_.spin_some();
      if (predicate()) {
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    executor_.spin_some();
    return predicate();
  }

  void spinFor(std::chrono::milliseconds duration)
  {
    ASSERT_TRUE(spinUntil([]() { return false; }, duration) == false);
  }

  rclcpp::executors::SingleThreadedExecutor executor_;
  std::shared_ptr<fake_vel_transform::FakeVelTransform> transform_node_;
  rclcpp::Node::SharedPtr test_node_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<example_interfaces::msg::Float32>::SharedPtr spin_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr terrain_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr output_sub_;
  std::vector<geometry_msgs::msg::Twist> outputs_;
  std::string odom_topic_;
  std::string path_topic_;
  std::string input_topic_;
  std::string output_topic_;
  std::string spin_topic_;
  std::string terrain_topic_;
};

TEST(OdomValidation, RejectsNonfiniteAndZeroQuaternionOdometry)
{
  auto odometry = makeOdometry(0.0, timeFromSeconds(10.0));
  EXPECT_TRUE(fake_vel_transform::odom_validation::finiteOdometry(odometry));

  odometry.pose.pose.position.x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(fake_vel_transform::odom_validation::finiteOdometry(odometry));

  odometry = makeOdometry(0.0, timeFromSeconds(10.0));
  odometry.pose.pose.orientation.z = 0.0;
  odometry.pose.pose.orientation.w = 0.0;
  EXPECT_FALSE(fake_vel_transform::odom_validation::finiteOdometry(odometry));

  odometry = makeOdometry(0.0, timeFromSeconds(10.0));
  odometry.twist.twist.angular.z = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(fake_vel_transform::odom_validation::finiteOdometry(odometry));
}

TEST(OdomValidation, RejectsZeroInvalidStaleAndFutureStamps)
{
  using fake_vel_transform::odom_validation::freshStamp;
  constexpr int64_t NOW_NANOSECONDS = 10000000000LL;
  EXPECT_TRUE(freshStamp(timeFromSeconds(9.8), NOW_NANOSECONDS, 0.25, 0.05));
  EXPECT_TRUE(freshStamp(timeFromSeconds(10.05), NOW_NANOSECONDS, 0.25, 0.05));
  EXPECT_FALSE(freshStamp(builtin_interfaces::msg::Time{}, NOW_NANOSECONDS, 0.25, 0.05));
  EXPECT_FALSE(freshStamp(timeFromSeconds(9.7), NOW_NANOSECONDS, 0.25, 0.05));
  EXPECT_FALSE(freshStamp(timeFromSeconds(10.051), NOW_NANOSECONDS, 0.25, 0.05));

  builtin_interfaces::msg::Time invalid_stamp;
  invalid_stamp.sec = 10;
  invalid_stamp.nanosec = 1000000000U;
  EXPECT_FALSE(freshStamp(invalid_stamp, NOW_NANOSECONDS, 0.25, 0.05));
}

TEST_F(FakeVelTransformFixture, LatestModeUsesFirstValidOdometryImmediately)
{
  createNodes(true);
  const auto stamp = test_node_->now();
  odom_pub_->publish(makeOdometry(M_PI_2, stamp));
  spinFor(std::chrono::milliseconds(30));

  geometry_msgs::msg::Twist command;
  command.linear.x = 1.0;
  cmd_pub_->publish(command);
  ASSERT_TRUE(spinUntil([this]() { return !outputs_.empty(); }, std::chrono::seconds(1)));

  EXPECT_NEAR(outputs_.back().linear.x, 0.0, 1e-6);
  EXPECT_NEAR(outputs_.back().linear.y, -1.0, 1e-6);
}

TEST_F(FakeVelTransformFixture, LatestModeStopsWithoutValidFreshOdometry)
{
  createNodes(true);
  auto invalid_odometry = makeOdometry(M_PI_2, builtin_interfaces::msg::Time{});
  odom_pub_->publish(invalid_odometry);
  spinFor(std::chrono::milliseconds(30));

  geometry_msgs::msg::Twist command;
  command.linear.x = 1.0;
  cmd_pub_->publish(command);
  ASSERT_TRUE(spinUntil([this]() { return !outputs_.empty(); }, std::chrono::seconds(1)));

  EXPECT_DOUBLE_EQ(outputs_.back().linear.x, 0.0);
  EXPECT_DOUBLE_EQ(outputs_.back().linear.y, 0.0);
  EXPECT_DOUBLE_EQ(outputs_.back().angular.z, 0.0);
}

TEST_F(FakeVelTransformFixture, LatestModeWatchdogStopsWhenOdometryExpires)
{
  createNodes(true, 0.08);
  odom_pub_->publish(makeOdometry(0.0, test_node_->now()));
  spinFor(std::chrono::milliseconds(20));

  geometry_msgs::msg::Twist command;
  command.linear.x = 0.4;
  cmd_pub_->publish(command);
  ASSERT_TRUE(spinUntil([this]() { return !outputs_.empty(); }, std::chrono::seconds(1)));
  ASSERT_NEAR(outputs_.back().linear.x, 0.4, 1e-6);

  ASSERT_TRUE(
    spinUntil([this]() { return outputs_.size() >= 2U; }, std::chrono::milliseconds(300)));
  EXPECT_DOUBLE_EQ(outputs_.back().linear.x, 0.0);
  EXPECT_DOUBLE_EQ(outputs_.back().linear.y, 0.0);
  EXPECT_DOUBLE_EQ(outputs_.back().angular.z, 0.0);
}

TEST_F(FakeVelTransformFixture, CmdSpinIsEnabledByDefault)
{
  createNodes(true);
  EXPECT_TRUE(transform_node_->get_parameter("enable_cmd_spin").as_bool());
  ASSERT_TRUE(spinUntil(
    [this]() { return spin_pub_->get_subscription_count() > 0U; }, std::chrono::seconds(1)));

  odom_pub_->publish(makeOdometry(0.0, test_node_->now()));
  spinFor(std::chrono::milliseconds(20));
  example_interfaces::msg::Float32 spin;
  spin.data = 0.6F;
  spin_pub_->publish(spin);
  spinFor(std::chrono::milliseconds(20));

  cmd_pub_->publish(geometry_msgs::msg::Twist{});
  ASSERT_TRUE(spinUntil([this]() { return !outputs_.empty(); }, std::chrono::seconds(1)));
  EXPECT_NEAR(outputs_.back().angular.z, 0.6, 1e-6);
}

TEST_F(FakeVelTransformFixture, DisabledCmdSpinCannotOverrideZeroCommand)
{
  createNodes(
    true, 0.5,
    {rclcpp::Parameter("enable_cmd_spin", false), rclcpp::Parameter("init_spin_speed", 0.8)});
  EXPECT_FALSE(transform_node_->get_parameter("enable_cmd_spin").as_bool());
  spinFor(std::chrono::milliseconds(20));
  EXPECT_EQ(spin_pub_->get_subscription_count(), 0U);

  odom_pub_->publish(makeOdometry(0.0, test_node_->now()));
  spinFor(std::chrono::milliseconds(20));
  example_interfaces::msg::Float32 spin;
  spin.data = 1.2F;
  spin_pub_->publish(spin);
  spinFor(std::chrono::milliseconds(20));

  cmd_pub_->publish(geometry_msgs::msg::Twist{});
  ASSERT_TRUE(spinUntil([this]() { return !outputs_.empty(); }, std::chrono::seconds(1)));
  EXPECT_DOUBLE_EQ(outputs_.back().angular.z, 0.0);
}

TEST_F(FakeVelTransformFixture, TerrainGuardRejectsStartsAndStopsStalePerception)
{
  createNodes(
    true, 0.5,
    {rclcpp::Parameter("terrain_guard_enabled", true), rclcpp::Parameter("terrain_timeout", 0.08)});
  ASSERT_TRUE(spinUntil(
    [this]() { return terrain_pub_->get_subscription_count() > 0U; }, std::chrono::seconds(1)));

  odom_pub_->publish(makeOdometry(0.0, test_node_->now()));
  spinFor(std::chrono::milliseconds(20));

  geometry_msgs::msg::Twist command;
  command.linear.x = 0.4;
  cmd_pub_->publish(command);
  ASSERT_TRUE(spinUntil([this]() { return !outputs_.empty(); }, std::chrono::seconds(1)));
  EXPECT_DOUBLE_EQ(outputs_.back().linear.x, 0.0);

  const std::size_t blocked_output_count = outputs_.size();
  terrain_pub_->publish(sensor_msgs::msg::PointCloud2{});
  odom_pub_->publish(makeOdometry(0.0, test_node_->now()));
  spinFor(std::chrono::milliseconds(20));
  cmd_pub_->publish(command);
  ASSERT_TRUE(spinUntil(
    [this, blocked_output_count]() { return outputs_.size() > blocked_output_count; },
    std::chrono::seconds(1)));
  EXPECT_NEAR(outputs_.back().linear.x, 0.4, 1e-6);

  const std::size_t moving_output_count = outputs_.size();
  ASSERT_TRUE(spinUntil(
    [this, moving_output_count]() {
      return outputs_.size() > moving_output_count && outputs_.back().linear.x == 0.0;
    },
    std::chrono::milliseconds(300)));
  EXPECT_DOUBLE_EQ(outputs_.back().linear.y, 0.0);
  EXPECT_DOUBLE_EQ(outputs_.back().angular.z, 0.0);
}

TEST_F(FakeVelTransformFixture, LegacyModeStillWaitsForSynchronizedLocalPlan)
{
  createNodes(false);
  nav_msgs::msg::Path path;
  path.header.stamp = test_node_->now();
  path_pub_->publish(path);
  spinFor(std::chrono::milliseconds(20));

  geometry_msgs::msg::Twist command;
  command.linear.x = 1.0;
  cmd_pub_->publish(command);
  spinFor(std::chrono::milliseconds(30));
  EXPECT_TRUE(outputs_.empty());

  const auto stamp = test_node_->now();
  path.header.stamp = stamp;
  odom_pub_->publish(makeOdometry(M_PI_2, stamp));
  path_pub_->publish(path);
  ASSERT_TRUE(spinUntil([this]() { return !outputs_.empty(); }, std::chrono::seconds(1)));
  EXPECT_NEAR(outputs_.back().linear.x, 0.0, 1e-6);
  EXPECT_NEAR(outputs_.back().linear.y, -1.0, 1e-6);
}

}  // namespace
