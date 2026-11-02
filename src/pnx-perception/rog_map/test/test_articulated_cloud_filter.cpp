#include <gtest/gtest.h>
#include <rog_map_ros/cloud_registered_crop_filter.hpp>

namespace {

geometry_msgs::msg::TransformStamped joint(const std::string & child, int second,
                                          double yaw, double x, double y)
{
  geometry_msgs::msg::TransformStamped t;
  t.header.frame_id = "base";
  t.child_frame_id = child;
  t.header.stamp.sec = second;
  t.transform.translation.x = x;
  t.transform.translation.y = y;
  t.transform.rotation.z = std::sin(yaw/2);
  t.transform.rotation.w = std::cos(yaw/2);
  return t;
}

rog_map::Config config()
{
  rog_map::Config c;
  c.cloud_filter_position_frame = "base";
  c.cloud_filter_mode = "transform_cloud";
  c.cloud_filter_size_x = c.cloud_filter_size_y = c.cloud_filter_size_z = .4;
  c.cloud_filter_z_offset = -1;
  c.cloud_filter_transform_timeout = 0;
  return c;
}

TEST(ArticulatedCloudFilter, UsesMountAtEachScanInsteadOfCachingFirstAngle)
{
  auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  auto tf = std::make_shared<tf2_ros::Buffer>(clock);
  tf->setTransform(joint("lidar", 1, 0, 1, 0), "test");
  tf->setTransform(joint("lidar", 2, M_PI/2, 0, 1), "test");
  rog_map::CloudRegisteredCropFilter filter(config(), tf, clock, rclcpp::get_logger("test"), nullptr);
  for (int second : {1, 2}) {
    rog_map::PointCloud cloud;
    cloud.push_back(rog_map::PclPoint(.1f, 0, 0, 1));
    cloud.push_back(rog_map::PclPoint(.6f, 0, 0, 1));
    std_msgs::msg::Header h;
    h.frame_id = "world";
    h.stamp.sec = second;
    const double yaw = second == 1 ? 0 : M_PI/2;
    super_utils::Pose pose{super_utils::Vec3f(second == 1 ? 1 : 0, second == 1 ? 0 : 1, 0),
      super_utils::Quatf(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()))};
    ASSERT_TRUE(filter.filter(cloud, h, pose, "world", "lidar"));
    ASSERT_EQ(cloud.size(), 1U);
    EXPECT_NEAR(cloud[0].x, .6, 1e-6);
  }
}

TEST(ArticulatedCloudFilter, RotatedHeadBoxDoesNotEraseObstacleAtItsOldPosition)
{
  auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  auto tf = std::make_shared<tf2_ros::Buffer>(clock);
  tf->setTransform(joint("head", 1, M_PI/2, 0, 0), "test");
  auto c = config();
  c.cloud_filter_center_x = .2;
  c.cloud_filter_size_x = .12;
  c.cloud_filter_size_y = c.cloud_filter_size_z = .06;
  c.cloud_filter_box_frames = {"head"};
  rog_map::CloudRegisteredCropFilter filter(c, tf, clock, rclcpp::get_logger("test"), nullptr);
  rog_map::PointCloud cloud;
  cloud.push_back(rog_map::PclPoint(0, .2f, 0, 1));
  cloud.push_back(rog_map::PclPoint(.2f, 0, 0, 1));
  std_msgs::msg::Header h;
  h.frame_id = "world";
  h.stamp.sec = 1;
  super_utils::Pose pose{super_utils::Vec3f::Zero(), super_utils::Quatf::Identity()};
  ASSERT_TRUE(filter.filter(cloud, h, pose, "world", "base"));
  ASSERT_EQ(cloud.size(), 1U);
  EXPECT_NEAR(cloud[0].x, .2, 1e-6);
}
}  // namespace
