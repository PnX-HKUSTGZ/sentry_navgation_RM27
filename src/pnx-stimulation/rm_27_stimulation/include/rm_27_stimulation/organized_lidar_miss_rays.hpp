#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

namespace rm_27_stimulation
{

struct OrganizedLidarMissRayConfig
{
  uint32_t horizontal_samples{0};
  uint32_t vertical_samples{0};
  double horizontal_min_angle{0.0};
  double horizontal_max_angle{0.0};
  double vertical_min_angle{0.0};
  double vertical_max_angle{0.0};
  double miss_ray_length{0.0};
  double rog_raycast_max_range{0.0};
  double rog_map_resolution{0.0};
  uint32_t horizontal_stride{1U};
  uint32_t vertical_stride{1U};
  uint32_t horizontal_phase{0U};
  uint32_t vertical_phase{0U};
};

enum class MissRayFillStatus
{
  OK,
  INVALID_CONFIG,
  INVALID_LAYOUT,
  INVALID_FIELDS,
  INVALID_RING_LAYOUT
};

struct MissRayFillResult
{
  MissRayFillStatus status{MissRayFillStatus::OK};
  std::size_t synthesized{0};
  std::size_t finite_returns{0};
  std::size_t unsupported_nonfinite{0};
  std::size_t skipped_by_stride{0};
};

enum class FinitePointCompactionStatus
{
  OK,
  INVALID_LAYOUT,
  INVALID_FIELDS
};

struct FinitePointCompactionResult
{
  FinitePointCompactionStatus status{FinitePointCompactionStatus::OK};
  std::size_t kept{0};
  std::size_t removed{0};
};

inline const char * missRayFillStatusName(MissRayFillStatus status)
{
  switch (status) {
    case MissRayFillStatus::OK:
      return "OK";
    case MissRayFillStatus::INVALID_CONFIG:
      return "INVALID_CONFIG";
    case MissRayFillStatus::INVALID_LAYOUT:
      return "INVALID_LAYOUT";
    case MissRayFillStatus::INVALID_FIELDS:
      return "INVALID_FIELDS";
    case MissRayFillStatus::INVALID_RING_LAYOUT:
      return "INVALID_RING_LAYOUT";
  }
  return "UNKNOWN";
}

inline const char * finitePointCompactionStatusName(
  FinitePointCompactionStatus status)
{
  switch (status) {
    case FinitePointCompactionStatus::OK:
      return "OK";
    case FinitePointCompactionStatus::INVALID_LAYOUT:
      return "INVALID_LAYOUT";
    case FinitePointCompactionStatus::INVALID_FIELDS:
      return "INVALID_FIELDS";
  }
  return "UNKNOWN";
}

inline bool validMissRayConfig(const OrganizedLidarMissRayConfig & config)
{
  return config.horizontal_samples >= 2U && config.vertical_samples >= 2U &&
         std::isfinite(config.horizontal_min_angle) &&
         std::isfinite(config.horizontal_max_angle) &&
         config.horizontal_max_angle > config.horizontal_min_angle &&
         std::isfinite(config.vertical_min_angle) &&
         std::isfinite(config.vertical_max_angle) &&
         config.vertical_max_angle > config.vertical_min_angle &&
         std::isfinite(config.miss_ray_length) &&
         std::isfinite(config.rog_raycast_max_range) &&
         config.rog_raycast_max_range > 0.0 &&
         std::isfinite(config.rog_map_resolution) &&
         config.rog_map_resolution > 0.0 &&
         config.miss_ray_length >=
         config.rog_raycast_max_range + 2.0 * config.rog_map_resolution &&
         config.horizontal_stride >= 1U && config.vertical_stride >= 1U &&
         config.horizontal_phase < config.horizontal_stride &&
         config.vertical_phase < config.vertical_stride;
}

inline const sensor_msgs::msg::PointField * findPointField(
  const sensor_msgs::msg::PointCloud2 & cloud, const std::string & name)
{
  for (const auto & field : cloud.fields) {
    if (field.name == name) {
      return &field;
    }
  }
  return nullptr;
}

inline bool fieldFitsPoint(
  const sensor_msgs::msg::PointField * field, uint8_t datatype,
  uint32_t datatype_size, uint32_t point_step)
{
  return field != nullptr && field->datatype == datatype && field->count == 1U &&
         field->offset <= point_step &&
         datatype_size <= point_step - field->offset;
}

template<typename T>
inline T readPointValue(const uint8_t * point, uint32_t offset)
{
  T value{};
  std::memcpy(&value, point + offset, sizeof(T));
  return value;
}

template<typename T>
inline void writePointValue(uint8_t * point, uint32_t offset, T value)
{
  std::memcpy(point + offset, &value, sizeof(T));
}

// Gazebo's organized GPU lidar preserves each ray's row and column even when
// its range is infinite. Only those directional +/-inf samples are rebuilt.
// Plain NaNs carry no trustworthy miss direction and remain non-finite.
inline MissRayFillResult reconstructOrganizedLidarMissRays(
  sensor_msgs::msg::PointCloud2 & cloud,
  const OrganizedLidarMissRayConfig & config)
{
  MissRayFillResult result;
  if (!validMissRayConfig(config)) {
    result.status = MissRayFillStatus::INVALID_CONFIG;
    return result;
  }

  const std::size_t row_payload =
    static_cast<std::size_t>(cloud.width) * cloud.point_step;
  const std::size_t required_data_size = cloud.height == 0U ? 0U :
    static_cast<std::size_t>(cloud.height - 1U) * cloud.row_step + row_payload;
  if (cloud.is_bigendian || cloud.width != config.horizontal_samples ||
    cloud.height != config.vertical_samples || cloud.point_step == 0U ||
    static_cast<std::size_t>(cloud.row_step) < row_payload ||
    cloud.data.size() < required_data_size)
  {
    result.status = MissRayFillStatus::INVALID_LAYOUT;
    return result;
  }

  const auto * x_field = findPointField(cloud, "x");
  const auto * y_field = findPointField(cloud, "y");
  const auto * z_field = findPointField(cloud, "z");
  const auto * ring_field = findPointField(cloud, "ring");
  if (!fieldFitsPoint(
      x_field, sensor_msgs::msg::PointField::FLOAT32, sizeof(float), cloud.point_step) ||
    !fieldFitsPoint(
      y_field, sensor_msgs::msg::PointField::FLOAT32, sizeof(float), cloud.point_step) ||
    !fieldFitsPoint(
      z_field, sensor_msgs::msg::PointField::FLOAT32, sizeof(float), cloud.point_step) ||
    !fieldFitsPoint(
      ring_field, sensor_msgs::msg::PointField::UINT16, sizeof(uint16_t), cloud.point_step))
  {
    result.status = MissRayFillStatus::INVALID_FIELDS;
    return result;
  }

  // Validate the complete raster before changing any bytes. A transposed or
  // reordered cloud must fail closed instead of receiving incorrect rays.
  for (uint32_t row = 0; row < cloud.height; ++row) {
    const uint8_t * row_data = cloud.data.data() +
      static_cast<std::size_t>(row) * cloud.row_step;
    for (uint32_t column = 0; column < cloud.width; ++column) {
      const uint8_t * point = row_data +
        static_cast<std::size_t>(column) * cloud.point_step;
      if (readPointValue<uint16_t>(point, ring_field->offset) != row) {
        result.status = MissRayFillStatus::INVALID_RING_LAYOUT;
        return result;
      }
    }
  }

  const double horizontal_increment =
    (config.horizontal_max_angle - config.horizontal_min_angle) /
    static_cast<double>(config.horizontal_samples - 1U);
  const double vertical_increment =
    (config.vertical_max_angle - config.vertical_min_angle) /
    static_cast<double>(config.vertical_samples - 1U);

  for (uint32_t row = 0; row < cloud.height; ++row) {
    const double elevation = config.vertical_min_angle +
      static_cast<double>(row) * vertical_increment;
    const double horizontal_scale = config.miss_ray_length * std::cos(elevation);
    const float z = static_cast<float>(config.miss_ray_length * std::sin(elevation));
    uint8_t * row_data = cloud.data.data() +
      static_cast<std::size_t>(row) * cloud.row_step;

    for (uint32_t column = 0; column < cloud.width; ++column) {
      uint8_t * point = row_data +
        static_cast<std::size_t>(column) * cloud.point_step;
      const float old_x = readPointValue<float>(point, x_field->offset);
      const float old_y = readPointValue<float>(point, y_field->offset);
      const float old_z = readPointValue<float>(point, z_field->offset);
      if (std::isfinite(old_x) && std::isfinite(old_y) && std::isfinite(old_z)) {
        ++result.finite_returns;
        continue;
      }
      if (!std::isinf(old_x) && !std::isinf(old_y) && !std::isinf(old_z)) {
        ++result.unsupported_nonfinite;
        continue;
      }
      if (row % config.vertical_stride != config.vertical_phase ||
        column % config.horizontal_stride != config.horizontal_phase)
      {
        ++result.skipped_by_stride;
        continue;
      }

      const double azimuth = config.horizontal_min_angle +
        static_cast<double>(column) * horizontal_increment;
      writePointValue<float>(
        point, x_field->offset,
        static_cast<float>(horizontal_scale * std::cos(azimuth)));
      writePointValue<float>(
        point, y_field->offset,
        static_cast<float>(horizontal_scale * std::sin(azimuth)));
      writePointValue<float>(point, z_field->offset, z);
      ++result.synthesized;
    }
  }

  return result;
}

// Keep the complete point record for finite XYZ samples while removing the
// organized raster holes. This reduces transform and DDS cost without losing
// intensity, ring, or any other fields consumed downstream.
inline FinitePointCompactionResult compactFiniteXyzPoints(
  const sensor_msgs::msg::PointCloud2 & input,
  sensor_msgs::msg::PointCloud2 & output)
{
  FinitePointCompactionResult result;
  const std::size_t row_payload =
    static_cast<std::size_t>(input.width) * input.point_step;
  const std::size_t required_data_size = input.height == 0U ? 0U :
    static_cast<std::size_t>(input.height - 1U) * input.row_step + row_payload;
  const std::size_t point_count =
    static_cast<std::size_t>(input.width) * input.height;
  if (input.is_bigendian || input.point_step == 0U ||
    static_cast<std::size_t>(input.row_step) < row_payload ||
    input.data.size() < required_data_size ||
    point_count > std::numeric_limits<uint32_t>::max() ||
    point_count > std::numeric_limits<std::size_t>::max() / input.point_step)
  {
    result.status = FinitePointCompactionStatus::INVALID_LAYOUT;
    return result;
  }

  const auto * x_field = findPointField(input, "x");
  const auto * y_field = findPointField(input, "y");
  const auto * z_field = findPointField(input, "z");
  if (!fieldFitsPoint(
      x_field, sensor_msgs::msg::PointField::FLOAT32, sizeof(float), input.point_step) ||
    !fieldFitsPoint(
      y_field, sensor_msgs::msg::PointField::FLOAT32, sizeof(float), input.point_step) ||
    !fieldFitsPoint(
      z_field, sensor_msgs::msg::PointField::FLOAT32, sizeof(float), input.point_step))
  {
    result.status = FinitePointCompactionStatus::INVALID_FIELDS;
    return result;
  }

  output.header = input.header;
  output.height = 1U;
  output.width = 0U;
  output.fields = input.fields;
  output.is_bigendian = false;
  output.point_step = input.point_step;
  output.row_step = 0U;
  output.is_dense = true;
  output.data.resize(point_count * input.point_step);

  std::size_t write_offset = 0U;
  for (uint32_t row = 0; row < input.height; ++row) {
    const uint8_t * row_data = input.data.data() +
      static_cast<std::size_t>(row) * input.row_step;
    for (uint32_t column = 0; column < input.width; ++column) {
      const uint8_t * point = row_data +
        static_cast<std::size_t>(column) * input.point_step;
      const float x = readPointValue<float>(point, x_field->offset);
      const float y = readPointValue<float>(point, y_field->offset);
      const float z = readPointValue<float>(point, z_field->offset);
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        ++result.removed;
        continue;
      }
      std::memcpy(output.data.data() + write_offset, point, input.point_step);
      write_offset += input.point_step;
      ++result.kept;
    }
  }

  output.data.resize(write_offset);
  output.width = static_cast<uint32_t>(result.kept);
  output.row_step = output.width * output.point_step;
  return result;
}

}  // namespace rm_27_stimulation
