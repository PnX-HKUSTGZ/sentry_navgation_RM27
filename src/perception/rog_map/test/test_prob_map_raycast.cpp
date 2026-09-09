#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <rog_map/prob_map.h>

namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kSensorCoordinate = 0.1;
constexpr double kRaycastMaxRange = 2.0;

class TestProbMap : public rog_map::ProbMap
{
public:
  void configure(const std::shared_ptr<rclcpp::Node> & node)
  {
    cfg_.loadFromRosNode(node, "prob_map_test");
    initProbMap();
  }
};

class RclcppContextGuard
{
public:
  RclcppContextGuard()
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
      owns_context_ = true;
    }
  }

  ~RclcppContextGuard()
  {
    if (owns_context_ && rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

private:
  bool owns_context_{false};
};

rog_map::PclPoint pointAt(const rog_map::Vec3f & position)
{
  rog_map::PclPoint point;
  point.x = static_cast<float>(position.x());
  point.y = static_cast<float>(position.y());
  point.z = static_cast<float>(position.z());
  point.intensity = 1.0F;
  return point;
}

rog_map::Vec3f horizontalPoint(
  const rog_map::Vec3f & sensor,
  double range, double azimuth_degrees)
{
  const double azimuth = azimuth_degrees * kPi / 180.0;
  return sensor +
         range * rog_map::Vec3f(std::cos(azimuth), std::sin(azimuth), 0.0);
}

std::shared_ptr<rclcpp::Node> makeConfigNode()
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({
      rclcpp::Parameter("prob_map_test.resolution", 0.2),
      rclcpp::Parameter("prob_map_test.inflation_resolution", 0.2),
      rclcpp::Parameter("prob_map_test.inflation_step", 1),
      rclcpp::Parameter("prob_map_test.unk_inflation_en", false),
      rclcpp::Parameter("prob_map_test.map_size",
                      std::vector<double>{8.0, 8.0, 4.0}),
      rclcpp::Parameter("prob_map_test.map_sliding.enable", false),
      rclcpp::Parameter("prob_map_test.fix_map_origin",
                      std::vector<double>{0.0, 0.0, 0.0}),
      rclcpp::Parameter("prob_map_test.point_filt_num", 1),
      rclcpp::Parameter("prob_map_test.raycasting.enable", true),
      rclcpp::Parameter("prob_map_test.raycasting.batch_update_size", 1),
      rclcpp::Parameter("prob_map_test.raycasting.ray_range",
                      std::vector<double>{0.2, kRaycastMaxRange}),
      rclcpp::Parameter("prob_map_test.raycasting.local_update_box",
                      std::vector<double>{5.0, 5.0, 3.0}),
      rclcpp::Parameter("prob_map_test.raycasting.p_hit", 0.9),
      rclcpp::Parameter("prob_map_test.raycasting.p_miss", 0.45),
      rclcpp::Parameter("prob_map_test.raycasting.p_min", 0.12),
      rclcpp::Parameter("prob_map_test.raycasting.p_max", 0.97),
      rclcpp::Parameter("prob_map_test.raycasting.p_occ", 0.85),
      rclcpp::Parameter("prob_map_test.raycasting.p_free", 0.499),
      rclcpp::Parameter("prob_map_test.raycasting.parallel_enable", true),
      rclcpp::Parameter("prob_map_test.raycasting.num_threads", 2),
      rclcpp::Parameter("prob_map_test.performance.parallel_raycast_enable",
                      true),
      rclcpp::Parameter("prob_map_test.performance.raycast_num_threads", 2),
      rclcpp::Parameter("prob_map_test.decay.enable", false),
      rclcpp::Parameter("prob_map_test.decay.keep_time", 0.4),
      rclcpp::Parameter("prob_map_test.decay.clear_time", 1.0),
      rclcpp::Parameter("prob_map_test.frontier_extraction_en", false),
      rclcpp::Parameter("prob_map_test.esdf.enable", false),
      rclcpp::Parameter("prob_map_test.field.enable", false),
      rclcpp::Parameter("prob_map_test.projection.enable", false),
      rclcpp::Parameter("prob_map_test.performance.enable", false),
      rclcpp::Parameter("prob_map_test.visualization.enable", false),
      rclcpp::Parameter("prob_map_test.ros_callback.enable", false),
      rclcpp::Parameter("prob_map_test.virtual_ground_height", -1.8),
      rclcpp::Parameter("prob_map_test.virtual_ceil_height", 1.8),
  });
  return std::make_shared<rclcpp::Node>("prob_map_raycast_test", options);
}

TEST(ProbMapRaycast,
     MissOnlyEndpointsNeverBecomeOccupiedAndHitsWinTheirVoxel)
{
  RclcppContextGuard context;
  const auto node = makeConfigNode();
  TestProbMap map;
  map.configure(node);

  const rog_map::Vec3f sensor(kSensorCoordinate, kSensorCoordinate,
    kSensorCoordinate);
  const rog_map::Pose sensor_pose(sensor, super_utils::Quatf::Identity());

  // A no-return endpoint is deliberately beyond max range. ROGMap must clip
  // it into a miss-only ray, never an occupied endpoint on the range shell.
  rog_map::PointCloud miss_only_cloud;
  constexpr int kMissesPerDirection = 5;
  const std::vector<double> miss_azimuths{60.0, 90.0, 120.0,
    240.0, 270.0, 300.0};
  for (const double azimuth : miss_azimuths) {
    const auto endpoint = horizontalPoint(sensor, 3.0, azimuth);
    for (int repeat = 0; repeat < kMissesPerDirection; ++repeat) {
      miss_only_cloud.push_back(pointAt(endpoint));
    }
  }

  map.updateProbMap(miss_only_cloud, sensor_pose, sensor);

  const auto miss_only_stats = map.runtimeStats();
  EXPECT_DOUBLE_EQ(miss_only_stats.hit_count, 0.0);
  EXPECT_DOUBLE_EQ(miss_only_stats.raycast_skipped_far_count,
                   static_cast<double>(miss_only_cloud.size()));
  EXPECT_GT(miss_only_stats.miss_count, 0.0);

  rog_map::vec_E<rog_map::Vec3f> occupied_points;
  map.rawOccupiedBoxSearch(map.localMapMinPosition(), map.localMapMaxPosition(),
                           occupied_points);
  EXPECT_TRUE(occupied_points.empty());

  for (const double azimuth : miss_azimuths) {
    const auto clipped_endpoint =
      horizontalPoint(sensor, kRaycastMaxRange, azimuth);
    EXPECT_NE(map.getGridType(clipped_endpoint), super_utils::OCCUPIED)
      << "azimuth=" << azimuth;
  }
  EXPECT_EQ(map.getGridType(horizontalPoint(sensor, 1.0, 90.0)),
            super_utils::KNOWN_FREE);

  // Keep this cloud below the parallel dispatch threshold. The far rays pass
  // through the hit voxel often enough that applying their misses would make
  // it free; the expected occupied state therefore proves hit precedence.
  const rog_map::Vec3f serial_hit = horizontalPoint(sensor, 1.0, 0.0);
  rog_map::PointCloud serial_mixed_cloud;
  for (int repeat = 0; repeat < 12; ++repeat) {
    serial_mixed_cloud.push_back(
      pointAt(horizontalPoint(sensor, 3.0, 0.0)));
  }
  serial_mixed_cloud.push_back(pointAt(serial_hit));

  map.updateProbMap(serial_mixed_cloud, sensor_pose, sensor);

  const auto serial_stats = map.runtimeStats();
  EXPECT_DOUBLE_EQ(serial_stats.hit_count, 1.0);
  EXPECT_GT(serial_stats.miss_count, 0.0);
#ifdef _OPENMP
  EXPECT_DOUBLE_EQ(serial_stats.raycast_parallel_time, 0.0);
#endif
  EXPECT_EQ(map.getGridType(serial_hit), super_utils::OCCUPIED);
  EXPECT_NE(map.getGridType(horizontalPoint(sensor, kRaycastMaxRange, 0.0)),
            super_utils::OCCUPIED);

  // More than num_threads * 16 points selects the OpenMP implementation when
  // available. It must preserve the same hit-over-miss contract during merge.
  const rog_map::Vec3f parallel_hit = horizontalPoint(sensor, 1.0, 180.0);
  rog_map::PointCloud parallel_mixed_cloud;
  for (int repeat = 0; repeat < 64; ++repeat) {
    parallel_mixed_cloud.push_back(
      pointAt(horizontalPoint(sensor, 3.0, 180.0)));
  }
  parallel_mixed_cloud.push_back(pointAt(parallel_hit));

  map.updateProbMap(parallel_mixed_cloud, sensor_pose, sensor);

  const auto parallel_stats = map.runtimeStats();
  EXPECT_DOUBLE_EQ(parallel_stats.hit_count, 1.0);
  EXPECT_GT(parallel_stats.miss_count, 0.0);
#ifdef _OPENMP
  EXPECT_GT(parallel_stats.raycast_parallel_time, 0.0);
  EXPECT_DOUBLE_EQ(parallel_stats.raycast_unique_endpoint_count, 2.0);
  EXPECT_DOUBLE_EQ(
    parallel_stats.raycast_endpoint_dedup_count,
    static_cast<double>(parallel_mixed_cloud.size() - 2U));
  EXPECT_GE(parallel_stats.raycast_time, parallel_stats.raycast_parallel_time);
#endif
  EXPECT_EQ(map.getGridType(parallel_hit), super_utils::OCCUPIED);
  EXPECT_NE(
    map.getGridType(horizontalPoint(sensor, kRaycastMaxRange, 180.0)),
    super_utils::OCCUPIED);
}

}  // namespace
