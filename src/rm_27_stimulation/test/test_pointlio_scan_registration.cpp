#include <gtest/gtest.h>
#include <limits>
#include "rm_27_stimulation/pointlio_scan_registration.hpp"

namespace rm_27_stimulation
{
namespace
{
struct ScanFixture : testing::Test
{
  sensor_msgs::msg::PointCloud2 scan;
  nav_msgs::msg::Odometry odom;
  geometry_msgs::msg::Transform extrinsic;
  OrganizedLidarMissRayConfig config;
  ScanFixture()
  {
    scan.header.frame_id = "left_mid360";
    scan.header.stamp.sec = 10;
    scan.width = 2; scan.height = 2; scan.point_step = 16; scan.row_step = 32;
    scan.data.resize(64);
    for (const auto & entry : std::vector<std::pair<std::string, uint32_t>>{
      {"x", 0}, {"y", 4}, {"z", 8}, {"ring", 12}})
    {
      sensor_msgs::msg::PointField f;
      f.name = entry.first; f.offset = entry.second; f.count = 1;
      f.datatype = entry.first == "ring" ? f.UINT16 : f.FLOAT32;
      scan.fields.push_back(f);
    }
    for (int i = 0; i < 4; ++i) {
      auto * p = scan.data.data() + 16*i;
      writePointValue<float>(p, 0, 1.f);
      writePointValue<float>(p, 4, 0.f);
      writePointValue<float>(p, 8, i < 2 ? 0.f : .25f);
      writePointValue<uint16_t>(p, 12, i/2);
    }
    odom.header = scan.header; odom.header.frame_id = "camera_init";
    odom.child_frame_id = "body"; odom.pose.pose.orientation.w = 1;
    extrinsic.rotation.w = 1;
    config.horizontal_samples = 2; config.vertical_samples = 2;
    config.horizontal_max_angle = 1.; config.vertical_max_angle = .5;
    config.miss_ray_length = 10.5; config.rog_raycast_max_range = 10.;
    config.rog_map_resolution = .05;
  }
};
TEST_F(ScanFixture, RetainsEveryFloorAndRoofReturnWithoutCentroidDownsampling)
{
  const auto out = registerPointlioScan(scan, odom, extrinsic, config);
  ASSERT_EQ(out.width * out.height, 4u);
  EXPECT_FLOAT_EQ(readPointValue<float>(out.data.data(), 8), 0.f);
  EXPECT_FLOAT_EQ(readPointValue<float>(out.data.data()+32, 8), .25f);
  EXPECT_EQ(out.header, odom.header);
}
TEST_F(ScanFixture, ComposesSensorExtrinsicOnRightWithoutApplyingBaseMountTwice)
{
  tf2::Quaternion q; q.setRPY(-.523598775598, 0., 1.570796326795);
  odom.pose.pose.orientation = tf2::toMsg(q);
  odom.pose.pose.position.x = 2.;
  extrinsic.translation.x = .1;
  const auto out = registerPointlioScan(scan, odom, extrinsic, config);
  EXPECT_NEAR(readPointValue<float>(out.data.data(), 0), 2., 1.e-6);
  EXPECT_NEAR(readPointValue<float>(out.data.data(), 4), 1.1, 1.e-6);
  // With co-located IMU/lidar, -30 degrees belongs only in odometry.
  EXPECT_NEAR(readPointValue<float>(out.data.data()+32, 0), 1.875, 1.e-6);
  EXPECT_NEAR(readPointValue<float>(out.data.data()+32, 8), .25*std::cos(.523598775598), 1.e-6);
}
TEST_F(ScanFixture, AcceptsOnlyNumericalStampRoundingAndRejectsNextScan)
{
  odom.header.stamp.nanosec = 1;
  EXPECT_NO_THROW(registerPointlioScan(scan, odom, extrinsic, config));
  odom.header.stamp.nanosec = 1000000;
  EXPECT_THROW(registerPointlioScan(scan, odom, extrinsic, config), std::invalid_argument);
}
TEST_F(ScanFixture, RejectsWrongFramesInvalidPoseAndMalformedRayLayout)
{
  odom.child_frame_id = "base_link";
  EXPECT_THROW(registerPointlioScan(scan, odom, extrinsic, config), std::invalid_argument);
  odom.child_frame_id = "body"; odom.pose.pose.orientation.w = 0.;
  EXPECT_THROW(registerPointlioScan(scan, odom, extrinsic, config), std::invalid_argument);
  odom.pose.pose.orientation.w = 1.; odom.pose.pose.position.x = NAN;
  EXPECT_THROW(registerPointlioScan(scan, odom, extrinsic, config), std::invalid_argument);
  odom.pose.pose.position.x = 0.; scan.height = 1;
  EXPECT_THROW(registerPointlioScan(scan, odom, extrinsic, config), std::invalid_argument);
}
TEST_F(ScanFixture, ReconstructsMissButPreservesLowObstacleAndDiscardsUndirectedNaN)
{
  auto * p = scan.data.data();
  writePointValue<float>(p, 0, INFINITY); writePointValue<float>(p, 4, NAN);
  writePointValue<float>(p, 8, INFINITY);
  p += 16;
  for (int offset : {0, 4, 8}) writePointValue<float>(p, offset, NAN);
  writePointValue<float>(scan.data.data()+32, 8, .10f);
  const auto out = registerPointlioScan(scan, odom, extrinsic, config);
  ASSERT_EQ(out.width * out.height, 3u);
  EXPECT_FLOAT_EQ(readPointValue<float>(out.data.data(), 0), 10.5f);
  EXPECT_FLOAT_EQ(readPointValue<float>(out.data.data()+16, 8), .10f);
}
}  // namespace
}  // namespace rm_27_stimulation
