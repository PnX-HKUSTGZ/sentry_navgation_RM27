#include "mapping_output.hpp"
#include "mapping_scan_worker.hpp"
#include <gtest/gtest.h>
using namespace raw_gicp_mapping;
TEST(MappingOutput,RejectsFailedFrontendResult) {
  global_relocalization::RawFrontendResult result;
  EXPECT_FALSE(mappingOutput(result,builtin_interfaces::msg::Time{}));
}
TEST(MappingWorker,MappingGapBudgetUsesActualStampsAndRejectsExcessiveGap) {
  using Worker=MappingScanWorker<int,int>;
  Worker worker([](const Worker::Item & item){return item.payload;},[]{return 0.;},5.,1.);
  worker.begin();
  EXPECT_TRUE(worker.enqueue(1,4,10.,10.));
  EXPECT_TRUE(worker.enqueue(2,4,10.5,10.5));
  EXPECT_DOUBLE_EQ(worker.stats().max_stamp_gap,.5);
  EXPECT_FALSE(worker.enqueue(3,4,11.6,11.6));
  EXPECT_NEAR(worker.stats().max_stamp_gap,1.1,1e-12);
  EXPECT_EQ(worker.stats().error,"raw source scan coverage gap");
  worker.close();
}
TEST(MappingOutput,SensorOdometryAndRegisteredCloudShareExactlyOneTransform) {
  global_relocalization::RawFrontendResult result;result.accepted=true;
  result.reference_from_sensor.translation()<<1.,-2.,.4;
  result.reference_from_sensor.linear()=(Eigen::AngleAxisd(.8,Eigen::Vector3d::UnitZ())*Eigen::AngleAxisd(-.5,Eigen::Vector3d::UnitX())).toRotationMatrix();
  result.reference_from_base=result.reference_from_sensor;
  result.reference_from_base.translation().z()-=.13;
  result.filtered_sensor_cloud.emplace_back(2.,.3,1.);
  builtin_interfaces::msg::Time stamp;stamp.sec=57;stamp.nanosec=123456789;
  const auto output=mappingOutput(result,stamp);ASSERT_TRUE(output);
  EXPECT_EQ(output->odometry.header.frame_id,"camera_init");EXPECT_EQ(output->odometry.child_frame_id,"body");
  EXPECT_EQ(output->registered_cloud.header,output->odometry.header);
  EXPECT_EQ(output->odometry.header.stamp,stamp);
  EXPECT_DOUBLE_EQ(output->odometry.pose.pose.position.z,.4);
  global_relocalization::Cloud cloud;pcl::fromROSMsg(output->registered_cloud,cloud);ASSERT_EQ(cloud.size(),1u);
  const Eigen::Vector3d expected=result.reference_from_sensor*Eigen::Vector3d(2.,.3,1.);
  EXPECT_LT((cloud[0].getVector3fMap().cast<double>()-expected).norm(),1e-6);
}
