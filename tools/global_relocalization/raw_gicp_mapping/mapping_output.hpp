// Copyright 2026 RM27 contributors. SPDX-License-Identifier: Apache-2.0
#pragma once
#include "global_relocalization/stationary_raw_frontend.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <optional>

namespace raw_gicp_mapping {
struct MappingOutput {
  nav_msgs::msg::Odometry odometry;
  sensor_msgs::msg::PointCloud2 registered_cloud;
};
inline std::optional<MappingOutput> mappingOutput(
  const global_relocalization::RawFrontendResult & result,const builtin_interfaces::msg::Time & stamp) {
  if(!result.accepted)return {};
  MappingOutput output;
  // Match Point-LIO's contract: camera_init is the gravity-aligned initial
  // sensor origin, and body is the current sensor/IMU frame. loam_interface
  // applies the fixed base mount exactly once downstream.
  output.odometry.header.stamp=stamp;output.odometry.header.frame_id="camera_init";
  output.odometry.child_frame_id="body";
  const auto & pose=result.reference_from_sensor;
  auto & p=output.odometry.pose.pose;
  p.position.x=pose.translation().x();p.position.y=pose.translation().y();p.position.z=pose.translation().z();
  const Eigen::Quaterniond q(pose.rotation());
  p.orientation.x=q.x();p.orientation.y=q.y();p.orientation.z=q.z();p.orientation.w=q.w();
  // Covariance and twist are unavailable. The mapping status declares this;
  // this compatibility output is for loam_interface, not covariance fusion.
  global_relocalization::Cloud transformed;
  pcl::transformPointCloud(result.filtered_sensor_cloud,transformed,pose.matrix().cast<float>());
  pcl::toROSMsg(transformed,output.registered_cloud);
  output.registered_cloud.header=output.odometry.header;
  return output;
}
}
