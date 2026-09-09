#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>

#include <gtest/gtest.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include "rm_27_stimulation/organized_lidar_miss_rays.hpp"

namespace rm_27_stimulation
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

sensor_msgs::msg::PointField field(
  const char * name, uint32_t offset, uint8_t datatype)
{
  sensor_msgs::msg::PointField output;
  output.name = name;
  output.offset = offset;
  output.datatype = datatype;
  output.count = 1U;
  return output;
}

sensor_msgs::msg::PointCloud2 makeCloud(uint32_t width = 3U, uint32_t height = 2U)
{
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.width = width;
  cloud.height = height;
  cloud.point_step = 16U;
  cloud.row_step = width * cloud.point_step + 4U;
  cloud.is_bigendian = false;
  cloud.is_dense = false;
  cloud.fields = {
    field("x", 0U, sensor_msgs::msg::PointField::FLOAT32),
    field("y", 4U, sensor_msgs::msg::PointField::FLOAT32),
    field("z", 8U, sensor_msgs::msg::PointField::FLOAT32),
    field("ring", 12U, sensor_msgs::msg::PointField::UINT16)};
  cloud.data.resize(static_cast<std::size_t>(cloud.row_step) * cloud.height, 0U);

  const float infinity = std::numeric_limits<float>::infinity();
  const float nan = std::numeric_limits<float>::quiet_NaN();
  for (uint32_t row = 0; row < height; ++row) {
    for (uint32_t column = 0; column < width; ++column) {
      uint8_t * point = cloud.data.data() +
        static_cast<std::size_t>(row) * cloud.row_step +
        static_cast<std::size_t>(column) * cloud.point_step;
      writePointValue<float>(point, 0U, infinity);
      writePointValue<float>(point, 4U, nan);
      writePointValue<float>(point, 8U, infinity);
      writePointValue<uint16_t>(point, 12U, static_cast<uint16_t>(row));
    }
  }
  return cloud;
}

OrganizedLidarMissRayConfig makeConfig()
{
  OrganizedLidarMissRayConfig config;
  config.horizontal_samples = 3U;
  config.vertical_samples = 2U;
  config.horizontal_min_angle = 0.0;
  config.horizontal_max_angle = 2.0 * kPi;
  config.vertical_min_angle = 0.0;
  config.vertical_max_angle = kPi / 2.0;
  config.miss_ray_length = 10.5;
  config.rog_raycast_max_range = 10.0;
  config.rog_map_resolution = 0.05;
  return config;
}

uint8_t * pointAt(
  sensor_msgs::msg::PointCloud2 & cloud, uint32_t row, uint32_t column)
{
  return cloud.data.data() + static_cast<std::size_t>(row) * cloud.row_step +
         static_cast<std::size_t>(column) * cloud.point_step;
}

TEST(OrganizedLidarMissRays, RebuildsOnlyDirectionalInfiniteReturns)
{
  auto cloud = makeCloud();
  uint8_t * finite = pointAt(cloud, 0U, 1U);
  writePointValue<float>(finite, 0U, 1.0F);
  writePointValue<float>(finite, 4U, 2.0F);
  writePointValue<float>(finite, 8U, 3.0F);
  uint8_t * unsupported_nan = pointAt(cloud, 1U, 1U);
  const float nan = std::numeric_limits<float>::quiet_NaN();
  writePointValue<float>(unsupported_nan, 0U, nan);
  writePointValue<float>(unsupported_nan, 4U, nan);
  writePointValue<float>(unsupported_nan, 8U, nan);

  const auto result = reconstructOrganizedLidarMissRays(cloud, makeConfig());

  EXPECT_EQ(result.status, MissRayFillStatus::OK);
  EXPECT_EQ(result.synthesized, 4U);
  EXPECT_EQ(result.finite_returns, 1U);
  EXPECT_EQ(result.unsupported_nonfinite, 1U);
  EXPECT_FLOAT_EQ(readPointValue<float>(finite, 0U), 1.0F);
  EXPECT_FLOAT_EQ(readPointValue<float>(finite, 1U * 4U), 2.0F);
  EXPECT_FLOAT_EQ(readPointValue<float>(finite, 2U * 4U), 3.0F);
  EXPECT_NEAR(readPointValue<float>(pointAt(cloud, 0U, 0U), 0U), 10.5F, 1.0e-5F);
  EXPECT_NEAR(readPointValue<float>(pointAt(cloud, 0U, 2U), 0U), 10.5F, 1.0e-5F);
  EXPECT_NEAR(readPointValue<float>(pointAt(cloud, 1U, 0U), 8U), 10.5F, 1.0e-5F);
  EXPECT_TRUE(std::isnan(readPointValue<float>(unsupported_nan, 0U)));
}

TEST(OrganizedLidarMissRays, UsesInclusiveRasterAnglesAndPreservesSeam)
{
  auto cloud = makeCloud(360U, 720U);
  OrganizedLidarMissRayConfig config;
  config.horizontal_samples = 360U;
  config.vertical_samples = 720U;
  config.horizontal_min_angle = 0.0;
  config.horizontal_max_angle = 2.0 * kPi;
  config.vertical_min_angle = -7.22 * kPi / 180.0;
  config.vertical_max_angle = 55.22 * kPi / 180.0;
  config.miss_ray_length = 10.5;
  config.rog_raycast_max_range = 10.0;
  config.rog_map_resolution = 0.05;

  const auto result = reconstructOrganizedLidarMissRays(cloud, config);

  ASSERT_EQ(result.status, MissRayFillStatus::OK);
  EXPECT_EQ(result.synthesized, 360U * 720U);
  const uint8_t * first = pointAt(cloud, 0U, 0U);
  EXPECT_NEAR(
    std::atan2(readPointValue<float>(first, 8U),
      std::hypot(readPointValue<float>(first, 0U),
        readPointValue<float>(first, 4U))),
    config.vertical_min_angle, 1.0e-6);

  const uint8_t * upper_quadrant = pointAt(cloud, 719U, 90U);
  EXPECT_NEAR(
    std::atan2(readPointValue<float>(upper_quadrant, 4U),
      readPointValue<float>(upper_quadrant, 0U)),
    90.0 * 2.0 * kPi / 359.0, 1.0e-6);
  EXPECT_NEAR(
    std::atan2(readPointValue<float>(upper_quadrant, 8U),
      std::hypot(readPointValue<float>(upper_quadrant, 0U),
        readPointValue<float>(upper_quadrant, 4U))),
    config.vertical_max_angle, 1.0e-6);

  const uint8_t * seam = pointAt(cloud, 0U, 359U);
  EXPECT_NEAR(readPointValue<float>(seam, 0U), readPointValue<float>(first, 0U), 1.0e-5F);
  EXPECT_NEAR(readPointValue<float>(seam, 4U), readPointValue<float>(first, 4U), 1.0e-5F);
  EXPECT_NEAR(readPointValue<float>(seam, 8U), readPointValue<float>(first, 8U), 1.0e-5F);
}

TEST(OrganizedLidarMissRays, StridesOnlyMissesAndKeepsEveryFiniteHit)
{
  auto cloud = makeCloud(4U, 4U);
  auto config = makeConfig();
  config.horizontal_samples = 4U;
  config.vertical_samples = 4U;
  config.horizontal_stride = 2U;
  config.vertical_stride = 2U;
  uint8_t * finite_on_skipped_row = pointAt(cloud, 1U, 1U);
  writePointValue<float>(finite_on_skipped_row, 0U, 1.0F);
  writePointValue<float>(finite_on_skipped_row, 4U, 2.0F);
  writePointValue<float>(finite_on_skipped_row, 8U, 3.0F);

  const auto result = reconstructOrganizedLidarMissRays(cloud, config);

  EXPECT_EQ(result.status, MissRayFillStatus::OK);
  EXPECT_EQ(result.synthesized, 4U);
  EXPECT_EQ(result.finite_returns, 1U);
  EXPECT_EQ(result.skipped_by_stride, 11U);
  EXPECT_FLOAT_EQ(readPointValue<float>(finite_on_skipped_row, 0U), 1.0F);
  EXPECT_TRUE(std::isinf(readPointValue<float>(pointAt(cloud, 1U, 0U), 0U)));
}

TEST(OrganizedLidarMissRays, RotatingStridePhasesCoverEveryMissDirection)
{
  std::size_t synthesized_total = 0U;
  for (uint32_t vertical_phase = 0U; vertical_phase < 2U; ++vertical_phase) {
    for (uint32_t horizontal_phase = 0U; horizontal_phase < 2U;
      ++horizontal_phase)
    {
      auto cloud = makeCloud(4U, 4U);
      auto config = makeConfig();
      config.horizontal_samples = 4U;
      config.vertical_samples = 4U;
      config.horizontal_stride = 2U;
      config.vertical_stride = 2U;
      config.horizontal_phase = horizontal_phase;
      config.vertical_phase = vertical_phase;

      const auto result = reconstructOrganizedLidarMissRays(cloud, config);

      ASSERT_EQ(result.status, MissRayFillStatus::OK);
      EXPECT_EQ(result.synthesized, 4U);
      EXPECT_EQ(result.skipped_by_stride, 12U);
      synthesized_total += result.synthesized;
      for (uint32_t row = 0U; row < 4U; ++row) {
        for (uint32_t column = 0U; column < 4U; ++column) {
          const bool selected = row % 2U == vertical_phase &&
            column % 2U == horizontal_phase;
          EXPECT_EQ(
            std::isfinite(readPointValue<float>(pointAt(cloud, row, column), 0U)),
            selected);
        }
      }
    }
  }
  EXPECT_EQ(synthesized_total, 16U);
}

TEST(OrganizedLidarMissRays, RejectsRingMismatchBeforeMutatingCloud)
{
  auto cloud = makeCloud();
  const auto original_data = cloud.data;
  writePointValue<uint16_t>(pointAt(cloud, 1U, 2U), 12U, 0U);
  const auto mismatched_data = cloud.data;

  const auto result = reconstructOrganizedLidarMissRays(cloud, makeConfig());

  EXPECT_EQ(result.status, MissRayFillStatus::INVALID_RING_LAYOUT);
  EXPECT_EQ(result.synthesized, 0U);
  EXPECT_EQ(cloud.data, mismatched_data);
  EXPECT_NE(cloud.data, original_data);
}

TEST(OrganizedLidarMissRays, RejectsLayoutAndUnsafeRange)
{
  auto wrong_size = makeCloud(4U, 2U);
  EXPECT_EQ(
    reconstructOrganizedLidarMissRays(wrong_size, makeConfig()).status,
    MissRayFillStatus::INVALID_LAYOUT);

  auto truncated_row = makeCloud();
  truncated_row.row_step = truncated_row.width * truncated_row.point_step - 1U;
  EXPECT_EQ(
    reconstructOrganizedLidarMissRays(truncated_row, makeConfig()).status,
    MissRayFillStatus::INVALID_LAYOUT);

  auto truncated_data = makeCloud();
  truncated_data.data.resize(truncated_data.data.size() - truncated_data.point_step);
  EXPECT_EQ(
    reconstructOrganizedLidarMissRays(truncated_data, makeConfig()).status,
    MissRayFillStatus::INVALID_LAYOUT);

  auto big_endian = makeCloud();
  big_endian.is_bigendian = true;
  EXPECT_EQ(
    reconstructOrganizedLidarMissRays(big_endian, makeConfig()).status,
    MissRayFillStatus::INVALID_LAYOUT);

  auto cloud = makeCloud();
  auto unsafe_config = makeConfig();
  unsafe_config.miss_ray_length = unsafe_config.rog_raycast_max_range;
  EXPECT_EQ(
    reconstructOrganizedLidarMissRays(cloud, unsafe_config).status,
    MissRayFillStatus::INVALID_CONFIG);

  auto insufficient_margin = makeConfig();
  insufficient_margin.miss_ray_length = 10.05;
  EXPECT_EQ(
    reconstructOrganizedLidarMissRays(cloud, insufficient_margin).status,
    MissRayFillStatus::INVALID_CONFIG);

  auto zero_stride = makeConfig();
  zero_stride.vertical_stride = 0U;
  EXPECT_EQ(
    reconstructOrganizedLidarMissRays(cloud, zero_stride).status,
    MissRayFillStatus::INVALID_CONFIG);

  auto invalid_phase = makeConfig();
  invalid_phase.horizontal_stride = 2U;
  invalid_phase.horizontal_phase = 2U;
  EXPECT_EQ(
    reconstructOrganizedLidarMissRays(cloud, invalid_phase).status,
    MissRayFillStatus::INVALID_CONFIG);
}

TEST(OrganizedLidarMissRays, RejectsMissingOrWrongFields)
{
  auto cloud = makeCloud();
  cloud.fields.back().datatype = sensor_msgs::msg::PointField::UINT32;

  EXPECT_EQ(
    reconstructOrganizedLidarMissRays(cloud, makeConfig()).status,
    MissRayFillStatus::INVALID_FIELDS);

  cloud = makeCloud();
  cloud.fields.front().offset = cloud.point_step - 2U;
  EXPECT_EQ(
    reconstructOrganizedLidarMissRays(cloud, makeConfig()).status,
    MissRayFillStatus::INVALID_FIELDS);
}

TEST(FinitePointCompaction, RemovesNonFinitePointsAndPreservesRecords)
{
  auto cloud = makeCloud();
  cloud.header.frame_id = "left_mid360";
  uint8_t * first = pointAt(cloud, 0U, 1U);
  writePointValue<float>(first, 0U, 1.0F);
  writePointValue<float>(first, 4U, 2.0F);
  writePointValue<float>(first, 8U, 3.0F);
  writePointValue<uint16_t>(first, 12U, 7U);
  uint8_t * second = pointAt(cloud, 1U, 2U);
  writePointValue<float>(second, 0U, 4.0F);
  writePointValue<float>(second, 4U, 5.0F);
  writePointValue<float>(second, 8U, 6.0F);
  writePointValue<uint16_t>(second, 12U, 8U);

  sensor_msgs::msg::PointCloud2 compact;
  const auto result = compactFiniteXyzPoints(cloud, compact);

  ASSERT_EQ(result.status, FinitePointCompactionStatus::OK);
  EXPECT_EQ(result.kept, 2U);
  EXPECT_EQ(result.removed, 4U);
  EXPECT_EQ(compact.header.frame_id, cloud.header.frame_id);
  EXPECT_EQ(compact.height, 1U);
  EXPECT_EQ(compact.width, 2U);
  EXPECT_EQ(compact.row_step, 2U * cloud.point_step);
  EXPECT_EQ(compact.data.size(), compact.row_step);
  EXPECT_TRUE(compact.is_dense);
  EXPECT_FLOAT_EQ(readPointValue<float>(compact.data.data(), 0U), 1.0F);
  EXPECT_EQ(readPointValue<uint16_t>(compact.data.data(), 12U), 7U);
  EXPECT_FLOAT_EQ(
    readPointValue<float>(compact.data.data() + cloud.point_step, 8U), 6.0F);
  EXPECT_EQ(
    readPointValue<uint16_t>(compact.data.data() + cloud.point_step, 12U), 8U);
}

TEST(FinitePointCompaction, RejectsInvalidLayoutAndFields)
{
  sensor_msgs::msg::PointCloud2 compact;
  auto malformed = makeCloud();
  malformed.data.resize(malformed.data.size() - malformed.point_step);
  EXPECT_EQ(
    compactFiniteXyzPoints(malformed, compact).status,
    FinitePointCompactionStatus::INVALID_LAYOUT);

  auto wrong_fields = makeCloud();
  wrong_fields.fields.front().datatype = sensor_msgs::msg::PointField::FLOAT64;
  EXPECT_EQ(
    compactFiniteXyzPoints(wrong_fields, compact).status,
    FinitePointCompactionStatus::INVALID_FIELDS);
}

}  // namespace
}  // namespace rm_27_stimulation
