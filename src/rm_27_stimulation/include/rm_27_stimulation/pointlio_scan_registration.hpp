#pragma once

#include <cmath>
#include <stdexcept>
#include <string>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl_ros/transforms.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "rm_27_stimulation/organized_lidar_miss_rays.hpp"

namespace rm_27_stimulation
{
constexpr int64_t kPointlioStampRoundingNanoseconds = 1000;
// Simulation scans are instantaneous. Never use the latest pose or interpolate
// across a missing Point-LIO update: each ray belongs to this exact estimate.
inline sensor_msgs::msg::PointCloud2 registerPointlioScan(
  const sensor_msgs::msg::PointCloud2 & scan,
  const nav_msgs::msg::Odometry & odom,
  const geometry_msgs::msg::Transform & body_to_lidar,
  const OrganizedLidarMissRayConfig & config,
  const std::string & world_frame = "camera_init",
  const std::string & body_frame = "body",
  const std::string & lidar_frame = "left_mid360")
{
  const auto ns = [](const auto & t) {
    return int64_t(t.sec) * 1000000000LL + t.nanosec;
  };
  // Point-LIO converts stamps through double seconds (observed rounding: 1 ns).
  // This numerical tolerance is far below one sensor/IMU period.
  if (std::abs(ns(scan.header.stamp) - ns(odom.header.stamp)) >
    kPointlioStampRoundingNanoseconds ||
    odom.header.frame_id != world_frame || odom.child_frame_id != body_frame ||
    scan.header.frame_id != lidar_frame)
  {
    throw std::invalid_argument("scan/Point-LIO timestamp or frame mismatch");
  }
  const auto valid_rotation = [](const auto & q) {
    const double n = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
    return std::isfinite(n) && n > 1.e-12;
  };
  const auto valid_translation = [](const auto & p) {
    return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
  };
  if (!valid_rotation(odom.pose.pose.orientation) ||
    !valid_translation(odom.pose.pose.position) ||
    !valid_rotation(body_to_lidar.rotation) ||
    !valid_translation(body_to_lidar.translation))
  {
    throw std::invalid_argument("invalid Point-LIO pose or sensor extrinsic");
  }
  tf2::Transform world_to_body, extrinsic;
  tf2::fromMsg(odom.pose.pose, world_to_body);
  tf2::fromMsg(body_to_lidar, extrinsic);
  auto q = world_to_body.getRotation(); q.normalize(); world_to_body.setRotation(q);
  q = extrinsic.getRotation(); q.normalize(); extrinsic.setRotation(q);
  auto rays = scan;
  const auto filled = reconstructOrganizedLidarMissRays(rays, config);
  if (filled.status != MissRayFillStatus::OK) {
    throw std::invalid_argument(std::string("invalid organized scan: ") +
      missRayFillStatusName(filled.status));
  }
  sensor_msgs::msg::PointCloud2 compact;
  const auto compacted = compactFiniteXyzPoints(rays, compact);
  if (compacted.status != FinitePointCompactionStatus::OK) {
    throw std::invalid_argument("invalid scan fields");
  }
  geometry_msgs::msg::TransformStamped transform;
  transform.header = odom.header;
  transform.child_frame_id = lidar_frame;
  transform.transform = tf2::toMsg(world_to_body * extrinsic);
  sensor_msgs::msg::PointCloud2 registered;
  pcl_ros::transformPointCloud(world_frame, transform, compact, registered);
  registered.header = odom.header;
  return registered;
}
}  // namespace rm_27_stimulation
