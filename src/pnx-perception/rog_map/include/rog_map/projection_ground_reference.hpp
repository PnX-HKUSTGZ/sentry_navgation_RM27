#pragma once

#include <cmath>
#include <Eigen/Geometry>
#include <rog_map/projection_layer.hpp>

namespace rog_map {

inline void setProjectionRobotPose(
    ProjectionLayerConfig &config, const Eigen::Vector3d &sensor_position,
    const Eigen::Quaterniond &sensor_orientation,
    const Eigen::Vector3d &sensor_mount_rpy,
    const Eigen::Vector2d &body_center_offset, bool pose_received) {
  const Eigen::Quaterniond body_from_sensor =
      Eigen::AngleAxisd(sensor_mount_rpy.z(), Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(sensor_mount_rpy.y(), Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(sensor_mount_rpy.x(), Eigen::Vector3d::UnitX());
  config.reference_ground_plane_valid = false;
  config.reference_ground_z_abs =
      sensor_position.z() - config.robot_origin_to_ground;
  config.robot_x = sensor_position.x();
  config.robot_y = sensor_position.y();
  config.robot_yaw = 0.0;
  if (!pose_received || !sensor_orientation.coeffs().allFinite() ||
      sensor_orientation.norm() < 1.0e-6) {
    return;
  }

  // Raycasting still starts at the sensor. Only terrain and footprint axes
  // remove the fixed mount rotation to recover the physical chassis attitude.
  const Eigen::Quaterniond body_orientation =
      (sensor_orientation.normalized() * body_from_sensor.conjugate()).normalized();
  const Eigen::Matrix3d rotation = body_orientation.toRotationMatrix();
  config.robot_yaw = std::atan2(rotation(1, 0), rotation(0, 0));
  const double c = std::cos(config.robot_yaw);
  const double s = std::sin(config.robot_yaw);
  config.robot_x += c * body_center_offset.x() - s * body_center_offset.y();
  config.robot_y += s * body_center_offset.x() + c * body_center_offset.y();

  const Eigen::Vector3d normal = rotation.col(2);
  if (!normal.allFinite() || normal.z() <= 1.0e-6) {
    return;
  }
  const double slope_x = -normal.x() / normal.z();
  const double slope_y = -normal.y() / normal.z();
  constexpr double kPi = 3.14159265358979323846;
  if (std::hypot(slope_x, slope_y) >
      std::tan(config.max_ground_slope_deg * kPi / 180.0) + 1.0e-6) {
    return;
  }
  const double plane_constant =
      normal.dot(sensor_position) - config.robot_origin_to_ground;
  config.reference_ground_z_abs =
      (plane_constant - normal.x() * config.robot_x - normal.y() * config.robot_y) /
      normal.z();
  config.reference_ground_plane_valid = true;
  config.reference_ground_slope_x = slope_x;
  config.reference_ground_slope_y = slope_y;
}

} // namespace rog_map
