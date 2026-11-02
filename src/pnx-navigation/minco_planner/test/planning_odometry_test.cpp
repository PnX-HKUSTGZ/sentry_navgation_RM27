#include <gtest/gtest.h>

#include "minco_core/components/planning_odometry.hpp"

namespace
{
using minco_planner::planning_odometry::planarVelocity;

geometry_msgs::msg::Quaternion messageQuaternion(const Eigen::Quaterniond & quaternion)
{
  geometry_msgs::msg::Quaternion result;
  result.w = quaternion.w();
  result.x = quaternion.x();
  result.y = quaternion.y();
  result.z = quaternion.z();
  return result;
}

nav_msgs::msg::Odometry odometry()
{
  nav_msgs::msg::Odometry result;
  result.header.frame_id = "camera_init";
  result.header.stamp.sec = 10;
  result.child_frame_id = "body";
  result.pose.pose.orientation.w = 1.0;
  return result;
}

geometry_msgs::msg::TransformStamped frameRotation(double yaw, int seconds = 10)
{
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "map";
  transform.child_frame_id = "camera_init";
  transform.header.stamp.sec = seconds;
  transform.transform.translation.x = 20.0;
  transform.transform.rotation = messageQuaternion(
    Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())));
  return transform;
}

TEST(PlanningOdometry, RotatesVelocityIntoMapWithoutAddingTranslation)
{
  tf2_ros::Buffer transforms(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME));
  ASSERT_TRUE(transforms.setTransform(frameRotation(M_PI_2), "test", true));
  auto odom = odometry();
  odom.twist.twist.linear.x = 1.0;
  Eigen::Vector3d velocity;
  std::string reason;
  ASSERT_TRUE(planarVelocity(odom, "map", &transforms, 0.0, 0.0, velocity, reason)) << reason;
  EXPECT_NEAR(velocity.x(), 0.0, 1e-9);
  EXPECT_NEAR(velocity.y(), 1.0, 1e-9);
  EXPECT_DOUBLE_EQ(velocity.z(), 0.0);
}

TEST(PlanningOdometry, TiltedBodyVelocityUsesAllThreeComponents)
{
  auto odom = odometry();
  odom.pose.pose.orientation = messageQuaternion(
    Eigen::Quaterniond(Eigen::AngleAxisd(M_PI / 3.0, Eigen::Vector3d::UnitX())));
  odom.twist.twist.linear.y = 0.5;
  odom.twist.twist.linear.z = -std::sqrt(3.0) / 2.0;
  Eigen::Vector3d velocity;
  std::string reason;
  ASSERT_TRUE(planarVelocity(odom, "camera_init", nullptr, 0.0, 0.0, velocity, reason));
  EXPECT_NEAR(velocity.y(), 1.0, 1e-9);
  EXPECT_DOUBLE_EQ(velocity.z(), 0.0);
}

TEST(PlanningOdometry, CancelsRigidLeverArmVelocityBeforeFrameRotation)
{
  tf2_ros::Buffer transforms(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME));
  ASSERT_TRUE(transforms.setTransform(frameRotation(M_PI_2), "test", true));
  auto odom = odometry();
  odom.twist.twist.linear.x = 0.2;
  odom.twist.twist.angular.z = 1.0;
  Eigen::Vector3d velocity;
  std::string reason;
  ASSERT_TRUE(planarVelocity(odom, "map", &transforms, 0.0, -0.2, velocity, reason));
  EXPECT_NEAR(velocity.norm(), 0.0, 1e-9);
}

TEST(PlanningOdometry, KeepsVerticalComponentUntilPlanningFrameRotation)
{
  tf2_ros::Buffer transforms(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME));
  auto transform = frameRotation(0.0);
  transform.transform.rotation = messageQuaternion(
    Eigen::Quaterniond(Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitX())));
  ASSERT_TRUE(transforms.setTransform(transform, "test", true));
  auto odom = odometry();
  odom.twist.twist.linear.z = 1.0;
  Eigen::Vector3d velocity;
  std::string reason;
  ASSERT_TRUE(planarVelocity(odom, "map", &transforms, 0.0, 0.0, velocity, reason));
  EXPECT_NEAR(velocity.y(), 1.0, 1e-9);
  EXPECT_DOUBLE_EQ(velocity.z(), 0.0);
}

TEST(PlanningOdometry, DoesNotUseLatestTransformWhenSampleIsOutsideHistory)
{
  tf2_ros::Buffer transforms(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME));
  ASSERT_TRUE(transforms.setTransform(frameRotation(0.0, 12), "test", false));
  auto odom = odometry();
  Eigen::Vector3d velocity;
  std::string reason;
  EXPECT_FALSE(planarVelocity(odom, "map", &transforms, 0.0, 0.0, velocity, reason));
  EXPECT_FALSE(velocity.allFinite());
}

TEST(PlanningOdometry, UsesTransformAtOdometryStampNotLatest)
{
  tf2_ros::Buffer transforms(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME));
  ASSERT_TRUE(transforms.setTransform(frameRotation(0.0, 10), "test", false));
  ASSERT_TRUE(transforms.setTransform(frameRotation(M_PI_2, 12), "test", false));
  auto odom = odometry();
  odom.twist.twist.linear.x = 1.0;
  Eigen::Vector3d velocity;
  std::string reason;
  ASSERT_TRUE(planarVelocity(odom, "map", &transforms, 0.0, 0.0, velocity, reason));
  EXPECT_NEAR(velocity.x(), 1.0, 1e-9);
  EXPECT_NEAR(velocity.y(), 0.0, 1e-9);
}

TEST(PlanningOdometry, MissingTransformDoesNotPretendRobotIsStationary)
{
  tf2_ros::Buffer transforms(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME));
  auto odom = odometry();
  Eigen::Vector3d velocity;
  std::string reason;
  EXPECT_FALSE(planarVelocity(odom, "map", &transforms, 0.0, 0.0, velocity, reason));
  EXPECT_FALSE(velocity.allFinite());
  EXPECT_NE(reason.find("ODOMETRY_TRANSFORM_UNAVAILABLE"), std::string::npos);
}

TEST(PlanningOdometry, RejectsInvalidFramesStampsAndVelocities)
{
  Eigen::Vector3d velocity;
  std::string reason;
  auto odom = odometry();
  odom.header.frame_id.clear();
  EXPECT_FALSE(planarVelocity(odom, "camera_init", nullptr, 0.0, 0.0, velocity, reason));
  odom = odometry();
  odom.header.stamp.sec = 0;
  EXPECT_FALSE(planarVelocity(odom, "camera_init", nullptr, 0.0, 0.0, velocity, reason));
  odom = odometry();
  odom.pose.pose.orientation.w = 0.0;
  EXPECT_FALSE(planarVelocity(odom, "camera_init", nullptr, 0.0, 0.0, velocity, reason));
  odom = odometry();
  odom.twist.twist.linear.z = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(planarVelocity(odom, "camera_init", nullptr, 0.0, 0.0, velocity, reason));
}
}  // namespace
