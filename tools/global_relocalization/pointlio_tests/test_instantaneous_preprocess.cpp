#include "preprocess.h"
#include <iostream>
#include <stdexcept>
int main() {
  pcl::PointCloud<velodyne_ros::Point> cloud;
  for (int i=0; i<16; ++i) {
    velodyne_ros::Point p{};
    double angle = i*.15;
    p.x=3*std::cos(angle); p.y=3*std::sin(angle); p.z=.1;
    p.ring=0; p.time=0;
    cloud.push_back(p);
  }
  auto message=std::make_shared<sensor_msgs::msg::PointCloud2>();
  pcl::toROSMsg(cloud,*message);
  Preprocess preprocess;
  preprocess.lidar_type=VELO16; preprocess.time_unit=SEC;
  preprocess.N_SCANS=32; preprocess.SCAN_RATE=10;
  preprocess.point_filter_num=1; preprocess.blind=.5;
  PointCloudXYZI::Ptr result(new PointCloudXYZI);
  preprocess.instantaneous_scan=true;
  preprocess.process(message,result);
  if(result->size()!=cloud.size()) throw std::runtime_error("instantaneous frame lost points");
  for(const auto &p:*result) if(p.curvature!=0) throw std::runtime_error("instantaneous frame got invented timestamps");
  preprocess.instantaneous_scan=false;
  preprocess.process(message,result);
  bool inferred=false;
  for(const auto &p:*result) inferred |= p.curvature>0;
  if(!inferred) throw std::runtime_error("legacy azimuth fallback changed");
  for(auto &p:cloud) p.time=.025;
  pcl::toROSMsg(cloud,*message);
  preprocess.process(message,result);
  for(const auto &p:*result) if(std::abs(p.curvature-25)>.001) throw std::runtime_error("real scan seconds conversion changed");
  std::cout<<"instantaneous, missing-time fallback, and real per-point timing passed\n";
}
