#include <gtest/gtest.h>

#include "minco_core/components/map_query_adapters.hpp"

#include <chrono>
#include <cmath>
#include <future>
#include <limits>
#include <mutex>
#include <utility>
#include <vector>

namespace minco_planner
{
namespace
{

using namespace std::chrono_literals;

class RecordingMapQuery : public rog_map::MapQueryInterface
{
public:
  bool worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my) const override
  {
    last_world_to_map = Eigen::Vector2d(wx, wy);
    mx = 4U;
    my = 7U;
    return true;
  }

  void mapToWorld(unsigned int mx, unsigned int my, double & wx, double & wy) const override
  {
    last_map_to_world = Eigen::Vector2i(static_cast<int>(mx), static_cast<int>(my));
    wx = map_cell_center.x();
    wy = map_cell_center.y();
  }

  unsigned int sizeX() const override {return 1U;}
  unsigned int sizeY() const override {return 1U;}
  double resolution() const override {return 0.05;}
  double originX() const override {return 0.0;}
  double originY() const override {return 0.0;}
  uint8_t value(unsigned int, unsigned int) const override {return 0U;}
  const unsigned char * values() const override {return values_.data();}
  bool isValid(unsigned int, unsigned int) const override {return true;}
  bool isFree(unsigned int, unsigned int) const override {return true;}

  rog_map::QueryResult query(const Eigen::Vector3d & pos) const override
  {
    last_query = pos;
    rog_map::QueryResult result;
    result.ok = true;
    result.status = rog_map::QueryStatus::OK;
    result.distance = 0.75;
    result.gradient = query_gradient;
    result.snapshot_stamp = 123.0;
    return result;
  }

  bool evaluate(
    const Eigen::Vector3d & pos, double & distance,
    Eigen::Vector3d & gradient) const override
  {
    const auto result = query(pos);
    distance = result.distance;
    gradient = result.gradient;
    return result.ok;
  }

  Eigen::Vector2d map_cell_center{8.0, -1.0};
  Eigen::Vector3d query_gradient{1.0, 0.0, 2.0};
  mutable Eigen::Vector2d last_world_to_map{
    Eigen::Vector2d::Constant(std::numeric_limits<double>::quiet_NaN())};
  mutable Eigen::Vector2i last_map_to_world{Eigen::Vector2i::Constant(-1)};
  mutable Eigen::Vector3d last_query{
    Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN())};

private:
  std::vector<unsigned char> values_{0U};
};


TEST(Nav2CostmapQueryTest, CopyValuesUsesCostmapMutexAndCopiesCompleteMap)
{
  nav2_costmap_2d::Costmap2D costmap(3U, 2U, 0.05, -1.0, -2.0, 17U);
  costmap.setCost(2U, 1U, 99U);
  Nav2CostmapQuery query(&costmap);

  std::promise<void> worker_entered;
  auto entered = worker_entered.get_future();
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> map_lock(*costmap.getMutex());
  auto worker = std::async(
    std::launch::async,
    [&query, entered_promise = std::move(worker_entered)]() mutable {
      entered_promise.set_value();
      std::vector<unsigned char> values;
      const bool copied = query.copyValues(values);
      return std::make_pair(copied, std::move(values));
    });

  ASSERT_EQ(entered.wait_for(1s), std::future_status::ready);
  EXPECT_EQ(worker.wait_for(50ms), std::future_status::timeout);
  map_lock.unlock();

  ASSERT_EQ(worker.wait_for(1s), std::future_status::ready);
  const auto [copied, values] = worker.get();
  ASSERT_TRUE(copied);
  ASSERT_EQ(values.size(), 6U);
  EXPECT_EQ(values[0], 17U);
  EXPECT_EQ(values[5], 99U);
}

TEST(Nav2CostmapQueryTest, CellReadUsesCostmapMutex)
{
  nav2_costmap_2d::Costmap2D costmap(2U, 2U, 0.05, 0.0, 0.0, 23U);
  Nav2CostmapQuery query(&costmap);

  std::promise<void> worker_entered;
  auto entered = worker_entered.get_future();
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> map_lock(*costmap.getMutex());
  auto worker = std::async(
    std::launch::async,
    [&query, entered_promise = std::move(worker_entered)]() mutable {
      entered_promise.set_value();
      return query.value(1U, 1U);
    });

  ASSERT_EQ(entered.wait_for(1s), std::future_status::ready);
  EXPECT_EQ(worker.wait_for(50ms), std::future_status::timeout);
  map_lock.unlock();

  ASSERT_EQ(worker.wait_for(1s), std::future_status::ready);
  EXPECT_EQ(worker.get(), 23U);
}

TEST(Nav2CostmapQueryTest, QueryReadsOneConsistentCostmapState)
{
  nav2_costmap_2d::Costmap2D costmap(
    2U, 2U, 0.25, -0.5, -0.5, nav2_costmap_2d::FREE_SPACE);
  costmap.setCost(1U, 1U, nav2_costmap_2d::LETHAL_OBSTACLE);
  Nav2CostmapQuery query(&costmap);

  const auto free_result = query.query(Eigen::Vector3d(-0.375, -0.375, 0.0));
  ASSERT_TRUE(free_result.ok);
  ASSERT_TRUE(free_result.projected_cost_valid);
  EXPECT_EQ(free_result.projected_cost, nav2_costmap_2d::FREE_SPACE);
  EXPECT_DOUBLE_EQ(free_result.distance, 0.25);

  const auto occupied_result = query.query(Eigen::Vector3d(-0.125, -0.125, 0.0));
  ASSERT_TRUE(occupied_result.ok);
  ASSERT_TRUE(occupied_result.projected_cost_valid);
  EXPECT_EQ(occupied_result.projected_cost, nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_DOUBLE_EQ(occupied_result.distance, -1.0);
}

TEST(StaticObstacleClearanceQueryTest, HardensOnlyTrueLethalObstacleNeighborhood)
{
  constexpr unsigned int kSize = 9U;
  nav2_costmap_2d::Costmap2D costmap(
    kSize, kSize, 1.0, 0.0, 0.0, nav2_costmap_2d::FREE_SPACE);
  costmap.setCost(4U, 4U, nav2_costmap_2d::LETHAL_OBSTACLE);
  costmap.setCost(0U, 0U, nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
  costmap.setCost(8U, 8U, nav2_costmap_2d::NO_INFORMATION);
  auto base = std::make_shared<Nav2CostmapQuery>(&costmap);

  StaticObstacleClearanceQuery query(base, 2.01);

  EXPECT_EQ(query.hardenedCellCount(), 12U);
  EXPECT_EQ(query.value(4U, 4U), nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_EQ(query.value(6U, 4U), nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_EQ(query.value(6U, 6U), nav2_costmap_2d::FREE_SPACE);
  EXPECT_EQ(query.value(0U, 0U), nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
  EXPECT_EQ(query.value(1U, 0U), nav2_costmap_2d::FREE_SPACE);
  EXPECT_EQ(query.value(8U, 8U), nav2_costmap_2d::NO_INFORMATION);
  EXPECT_FALSE(query.isFree(6U, 4U));

  const auto result = query.query(Eigen::Vector3d(6.5, 4.5, 0.0));
  ASSERT_TRUE(result.ok);
  ASSERT_TRUE(result.projected_cost_valid);
  EXPECT_EQ(result.projected_cost, nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_DOUBLE_EQ(result.distance, -1.0);

  std::vector<unsigned char> values;
  ASSERT_TRUE(query.copyValues(values));
  ASSERT_EQ(values.size(), static_cast<size_t>(kSize) * kSize);
  EXPECT_EQ(values[4U * kSize + 6U], nav2_costmap_2d::LETHAL_OBSTACLE);
}

TEST(StaticObstacleClearanceQueryTest, GuardsKnownFreeSideOfUnknownObstacleBoundary)
{
  constexpr unsigned int kSize = 9U;
  nav2_costmap_2d::Costmap2D costmap(
    kSize, kSize, 1.0, 0.0, 0.0, nav2_costmap_2d::FREE_SPACE);
  costmap.setCost(4U, 4U, nav2_costmap_2d::LETHAL_OBSTACLE);
  costmap.setCost(5U, 4U, nav2_costmap_2d::NO_INFORMATION);
  auto base = std::make_shared<Nav2CostmapQuery>(&costmap);

  StaticObstacleClearanceQuery query(base, 1.01);

  EXPECT_EQ(query.value(5U, 4U), nav2_costmap_2d::NO_INFORMATION);
  EXPECT_FALSE(query.isFree(5U, 4U));
  EXPECT_EQ(query.value(6U, 4U),
            nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE - 1U);
  EXPECT_TRUE(query.isFree(6U, 4U));
  EXPECT_EQ(query.value(8U, 8U), nav2_costmap_2d::FREE_SPACE);
  EXPECT_GE(query.unknownBoundaryGuardCellCount(), 1U);
}

TEST(StaticObstacleClearanceQueryTest, DoesNotBakeMergedDynamicObstacleIntoStaticOverlay)
{
  constexpr unsigned int kSize = 9U;
  nav2_costmap_2d::Costmap2D merged(
    kSize, kSize, 1.0, 0.0, 0.0, nav2_costmap_2d::FREE_SPACE);
  nav2_costmap_2d::Costmap2D static_layer(
    kSize, kSize, 1.0, 0.0, 0.0, nav2_costmap_2d::FREE_SPACE);
  merged.setCost(1U, 1U, nav2_costmap_2d::LETHAL_OBSTACLE);
  merged.setCost(4U, 4U, nav2_costmap_2d::LETHAL_OBSTACLE);
  static_layer.setCost(4U, 4U, nav2_costmap_2d::LETHAL_OBSTACLE);
  auto base = std::make_shared<Nav2CostmapQuery>(&merged);
  auto static_source = std::make_shared<Nav2CostmapQuery>(&static_layer);

  StaticObstacleClearanceQuery query(base, static_source, 1.01);

  EXPECT_EQ(query.value(1U, 1U), nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_EQ(query.value(2U, 1U), nav2_costmap_2d::FREE_SPACE);
  EXPECT_EQ(query.value(5U, 4U), nav2_costmap_2d::LETHAL_OBSTACLE);

  merged.setCost(1U, 1U, nav2_costmap_2d::FREE_SPACE);
  EXPECT_EQ(query.value(1U, 1U), nav2_costmap_2d::FREE_SPACE);
  EXPECT_EQ(query.value(2U, 1U), nav2_costmap_2d::FREE_SPACE);
}

TEST(StaticObstacleClearanceQueryTest, RejectsInvalidRadius)
{
  nav2_costmap_2d::Costmap2D costmap(
    2U, 2U, 1.0, 0.0, 0.0, nav2_costmap_2d::FREE_SPACE);
  auto base = std::make_shared<Nav2CostmapQuery>(&costmap);

  EXPECT_THROW(StaticObstacleClearanceQuery(base, -0.01), std::invalid_argument);
  EXPECT_THROW(
    StaticObstacleClearanceQuery(
      base, std::numeric_limits<double>::quiet_NaN()),
    std::invalid_argument);
}

TEST(StaticObstacleClearanceQueryTest, OrientedPolygonOpens800mmCorridor)
{
  nav2_costmap_2d::Costmap2D costmap(80U, 80U, 0.05, -2.0, -2.0,
    nav2_costmap_2d::LETHAL_OBSTACLE);
  for (unsigned int y = 32U; y < 48U; ++y) {
    for (unsigned int x = 0U; x < 80U; ++x) {
      costmap.setCost(x, y, nav2_costmap_2d::FREE_SPACE);
    }
  }
  auto base = std::make_shared<Nav2CostmapQuery>(&costmap);
  const std::vector<Eigen::Vector2d> footprint{
    {0.311, -0.221}, {0.311, 0.220}, {0.296, 0.257}, {0.258, 0.296},
    {0.216, 0.313}, {-0.216, 0.313}, {-0.258, 0.296}, {-0.296, 0.257},
    {-0.311, 0.220}, {-0.311, -0.221}, {-0.296, -0.258}, {-0.258, -0.295},
    {-0.219, -0.311}, {0.218, -0.311}, {0.259, -0.294}, {0.296, -0.258}};
  StaticObstacleClearanceQuery circular(base, 0.443);
  EXPECT_FALSE(circular.isFree(40U, 40U));

  StaticObstacleClearanceQuery polygon(base, base, footprint, 0.05);
  EXPECT_TRUE(polygon.isFree(40U, 40U));
  EXPECT_FALSE(polygon.isFree(40U, 43U));
  polygon.setFootprintYaw(std::acos(-1.0) / 4.0);
  EXPECT_FALSE(polygon.isFree(40U, 40U));
  polygon.setFootprintYaw(std::acos(-1.0) / 2.0);
  EXPECT_TRUE(polygon.isFree(40U, 40U));
  polygon.setFootprintYaw(0.0);
  EXPECT_TRUE(polygon.isFree(40U, 40U));

  costmap.setCost(40U, 40U, nav2_costmap_2d::LETHAL_OBSTACLE);
  polygon.setFootprintYaw(0.0);
  EXPECT_FALSE(polygon.isFree(40U, 40U));
}

TEST(StaticObstacleClearanceQueryTest, Static253IsSoftOnlyWithIndependentDynamicLayerEvidence)
{
  nav2_costmap_2d::Costmap2D master(40U,40U,0.05,-1,-1,0U);
  nav2_costmap_2d::Costmap2D raw(40U,40U,0.05,-1,-1,0U);
  nav2_costmap_2d::Costmap2D measured(40U,40U,0.05,-1,-1,255U);
  for (unsigned int x=0; x<40; ++x) {
    for (unsigned int y=0; y<40; ++y) {
      if (y<14 || y>=27) {raw.setCost(x,y,254U);}
    }
  }
  master.setCost(20,20,253U);
  auto base=std::make_shared<Nav2CostmapQuery>(&master);
  auto prior=std::make_shared<Nav2CostmapQuery>(&raw);
  auto dynamic=std::make_shared<Nav2CostmapQuery>(&measured);
  const std::vector<Eigen::Vector2d> body={{-.311,-.312},{.311,-.312},{.311,.312},{-.311,.312}};
  StaticObstacleClearanceQuery query(base,prior,body,0.0,dynamic,0.312);
  EXPECT_EQ(query.value(20,20),252U);
  EXPECT_GT(query.query({.025,.025,0}).distance, 0.0);
  std::vector<uint8_t> values;
  ASSERT_TRUE(query.copyValues(values));
  EXPECT_EQ(values[20*40+20],252U);
  measured.setCost(20,26,254U);
  query.setFootprintYaw(0);
  EXPECT_EQ(query.value(20,20),254U);
  measured.setCost(20,26,255U);
  raw.setCost(20,20,254U);
  query.setFootprintYaw(0);
  EXPECT_EQ(query.value(20,20),254U);
  raw.setCost(20,20,0U);
  StaticObstacleClearanceQuery unseparated(base,prior,body,0.0);
  EXPECT_EQ(unseparated.value(20,20),253U);
}

TEST(StaticObstacleClearanceQueryTest, MeasuredPillarBlocksBodyCornerAndRotatesWithChassis)
{
  nav2_costmap_2d::Costmap2D master(40U,40U,0.05,-1,-1,0U);
  nav2_costmap_2d::Costmap2D raw(40U,40U,0.05,-1,-1,0U);
  nav2_costmap_2d::Costmap2D measured(40U,40U,0.05,-1,-1,255U);
  measured.setCost(20,20,254U);
  auto base = std::make_shared<Nav2CostmapQuery>(&master);
  auto prior = std::make_shared<Nav2CostmapQuery>(&raw);
  auto dynamic = std::make_shared<Nav2CostmapQuery>(&measured);
  const std::vector<Eigen::Vector2d> body = {
    {-.31,-.12},{.31,-.12},{.31,.12},{-.31,.12}};
  StaticObstacleClearanceQuery query(base, prior, body, 0.0, dynamic, .12, {.04,.03});
  EXPECT_EQ(query.value(14,18),254U);
  EXPECT_LT(query.value(18,14),253U);
  EXPECT_LT(query.value(13,20),253U);
  query.setFootprintYaw(std::acos(-1.0) / 2.0);
  EXPECT_LT(query.value(14,18),253U);
  EXPECT_EQ(query.value(18,14),254U);
  measured.setCost(20,20,255U);
  query.setFootprintYaw(0.0);
  EXPECT_LT(query.value(14,18),253U);
}

TEST(StaticObstacleClearanceQueryTest, PolygonPreservesUnknownAndSoftInflation)
{
  nav2_costmap_2d::Costmap2D master(60U, 60U, 0.05, 0.0, 0.0,
    nav2_costmap_2d::FREE_SPACE);
  nav2_costmap_2d::Costmap2D source(master);
  source.setCost(35U, 30U, nav2_costmap_2d::NO_INFORMATION);
  master.setCost(35U, 30U, nav2_costmap_2d::NO_INFORMATION);
  master.setCost(20U, 20U, 100U);
  master.setCost(20U, 22U, nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
  master.setCost(50U, 50U, nav2_costmap_2d::LETHAL_OBSTACLE);
  auto base = std::make_shared<Nav2CostmapQuery>(&master);
  auto static_source = std::make_shared<Nav2CostmapQuery>(&source);
  const std::vector<Eigen::Vector2d> footprint{
    {-0.30, -0.30}, {0.30, -0.30}, {0.30, 0.30}, {-0.30, 0.30}};
  StaticObstacleClearanceQuery polygon(base, static_source, footprint, 0.05);
  EXPECT_EQ(polygon.value(35U, 30U), nav2_costmap_2d::NO_INFORMATION);
  EXPECT_FALSE(polygon.isFree(30U, 30U));
  EXPECT_EQ(polygon.value(20U, 20U), 100U);
  EXPECT_EQ(polygon.value(20U, 22U), nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
  EXPECT_EQ(polygon.value(50U, 50U), nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_TRUE(polygon.isFree(45U, 50U));
  EXPECT_FALSE(polygon.isFree(1U, 20U));
  EXPECT_THROW(polygon.setFootprintYaw(std::numeric_limits<double>::quiet_NaN()),
    std::invalid_argument);
  const std::vector<Eigen::Vector2d> concave{
    {-0.3, -0.3}, {0.3, -0.3}, {0.0, 0.0}, {0.3, 0.3}, {-0.3, 0.3}};
  EXPECT_THROW(StaticObstacleClearanceQuery(base, static_source, concave, 0.05),
    std::invalid_argument);
}

rog_map::PriorMapData makeGroundPrior(unsigned int width, unsigned int height)
{
  rog_map::PriorMapData prior;
  prior.loaded = true;
  prior.width = static_cast<int>(width);
  prior.height = static_cast<int>(height);
  prior.resolution = 1.0;
  prior.origin_x = 0.0;
  prior.origin_y = 0.0;
  prior.occupied.assign(static_cast<size_t>(width) * height, 0U);
  prior.known_free.assign(static_cast<size_t>(width) * height, 1U);
  prior.ground_elevation_loaded = true;
  prior.ground_elevation_default_height = 0.0;
  return prior;
}

TEST(SurveyedGroundEdgeQueryTest, MakesStepLethalAndAddsTraversableClearanceCost)
{
  constexpr unsigned int kWidth = 7U;
  constexpr unsigned int kHeight = 5U;
  nav2_costmap_2d::Costmap2D costmap(
    kWidth, kHeight, 1.0, 0.0, 0.0, nav2_costmap_2d::FREE_SPACE);
  auto base = std::make_shared<Nav2CostmapQuery>(&costmap);
  auto prior = makeGroundPrior(kWidth, kHeight);
  rog_map::GroundElevationPatch upper;
  upper.min_x = 3.0;
  upper.min_y = 0.0;
  upper.max_x = 7.0;
  upper.max_y = 5.0;
  upper.reference_z = 1.0;
  prior.ground_elevation_patches.push_back(upper);

  SurveyedGroundEdgeConfig config;
  config.max_step = 0.20;
  config.max_slope_deg = 20.0;
  config.clearance_radius = 1.01;
  config.clearance_cost = 200U;
  SurveyedGroundEdgeQuery query(base, prior, config);

  EXPECT_EQ(query.edgeCellCount(), 2U * kHeight);
  EXPECT_EQ(query.value(2U, 2U), nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_EQ(query.value(3U, 2U), nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_EQ(query.value(1U, 2U), 200U);
  EXPECT_EQ(query.value(4U, 2U), 200U);
  EXPECT_TRUE(query.isFree(1U, 2U));
  EXPECT_FALSE(query.isFree(2U, 2U));
  EXPECT_EQ(query.value(0U, 2U), nav2_costmap_2d::FREE_SPACE);

  std::vector<unsigned char> values;
  ASSERT_TRUE(query.copyValues(values));
  ASSERT_EQ(values.size(), static_cast<size_t>(kWidth) * kHeight);
  EXPECT_EQ(values[2U * kWidth + 2U], nav2_costmap_2d::LETHAL_OBSTACLE);
}

TEST(SurveyedGroundEdgeQueryTest, PreservesGradualSlopeAndBaseObstacles)
{
  constexpr unsigned int kWidth = 6U;
  constexpr unsigned int kHeight = 3U;
  nav2_costmap_2d::Costmap2D costmap(
    kWidth, kHeight, 1.0, 0.0, 0.0, nav2_costmap_2d::FREE_SPACE);
  costmap.setCost(0U, 0U, nav2_costmap_2d::NO_INFORMATION);
  costmap.setCost(5U, 2U, nav2_costmap_2d::LETHAL_OBSTACLE);
  auto base = std::make_shared<Nav2CostmapQuery>(&costmap);
  auto prior = makeGroundPrior(kWidth, kHeight);
  rog_map::GroundElevationPatch slope;
  slope.min_x = 0.0;
  slope.min_y = 0.0;
  slope.max_x = 6.0;
  slope.max_y = 3.0;
  slope.reference_z = 0.0;
  slope.slope_x = 0.10;
  prior.ground_elevation_patches.push_back(slope);

  SurveyedGroundEdgeConfig config;
  config.max_step = 0.20;
  config.max_slope_deg = 20.0;
  SurveyedGroundEdgeQuery query(base, prior, config);

  EXPECT_EQ(query.edgeCellCount(), 0U);
  EXPECT_EQ(query.clearanceCellCount(), 0U);
  EXPECT_EQ(query.value(2U, 1U), nav2_costmap_2d::FREE_SPACE);
  EXPECT_EQ(query.value(0U, 0U), nav2_costmap_2d::NO_INFORMATION);
  EXPECT_EQ(query.value(5U, 2U), nav2_costmap_2d::LETHAL_OBSTACLE);
}

TEST(SurveyedGroundEdgeQueryTest, MakesConfiguredInnerClearanceLethal)
{
  constexpr unsigned int kWidth = 7U;
  constexpr unsigned int kHeight = 5U;
  nav2_costmap_2d::Costmap2D costmap(
    kWidth, kHeight, 1.0, 0.0, 0.0, nav2_costmap_2d::FREE_SPACE);
  auto base = std::make_shared<Nav2CostmapQuery>(&costmap);
  auto prior = makeGroundPrior(kWidth, kHeight);
  rog_map::GroundElevationPatch upper;
  upper.min_x = 3.0;
  upper.min_y = 0.0;
  upper.max_x = 7.0;
  upper.max_y = 5.0;
  upper.reference_z = 1.0;
  prior.ground_elevation_patches.push_back(upper);

  SurveyedGroundEdgeConfig config;
  config.max_step = 0.20;
  config.max_slope_deg = 20.0;
  config.lethal_clearance_radius = 1.01;
  config.clearance_radius = 2.01;
  config.clearance_cost = 200U;
  SurveyedGroundEdgeQuery query(base, prior, config);

  EXPECT_EQ(query.value(1U, 2U), nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_EQ(query.value(4U, 2U), nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_EQ(query.value(0U, 2U), 200U);
  EXPECT_EQ(query.value(5U, 2U), 200U);
  EXPECT_FALSE(query.isFree(1U, 2U));
}

TEST(SurveyedGroundEdgeQueryTest, ClosesFreeCellsWithoutSurveyedSupport)
{
  constexpr unsigned int kWidth = 5U;
  constexpr unsigned int kHeight = 3U;
  nav2_costmap_2d::Costmap2D costmap(
    kWidth, kHeight, 1.0, 0.0, 0.0, nav2_costmap_2d::FREE_SPACE);
  auto base = std::make_shared<Nav2CostmapQuery>(&costmap);
  auto prior = makeGroundPrior(kWidth, kHeight);
  // Prior-map rasters use image-row order. Map row 1 is also image row 1 for
  // this three-row fixture.
  prior.known_free[1U * kWidth + 2U] = 0U;

  SurveyedGroundEdgeConfig config;
  config.clearance_radius = 0.0;
  SurveyedGroundEdgeQuery query(base, prior, config);

  EXPECT_EQ(query.unsupportedCellCount(), 1U);
  EXPECT_EQ(query.edgeCellCount(), 8U);
  EXPECT_EQ(query.value(2U, 1U), nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_EQ(query.value(1U, 1U), nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_EQ(query.value(0U, 1U), nav2_costmap_2d::FREE_SPACE);
  EXPECT_FALSE(query.isFree(2U, 1U));
}

TEST(FrameAwareRogQueryTest, AppliesNonIdentityTransformInEachRequiredDirection)
{
  auto clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(clock);

  geometry_msgs::msg::TransformStamped rog_from_planning;
  rog_from_planning.header.frame_id = "odom";
  rog_from_planning.child_frame_id = "map";
  rog_from_planning.transform.translation.x = 10.0;
  rog_from_planning.transform.translation.y = -2.0;
  rog_from_planning.transform.translation.z = 0.5;
  constexpr double kSqrtHalf = 0.7071067811865476;
  rog_from_planning.transform.rotation.z = kSqrtHalf;
  rog_from_planning.transform.rotation.w = kSqrtHalf;
  ASSERT_TRUE(tf_buffer->setTransform(rog_from_planning, "frame_aware_query_test", true));

  auto raw = std::make_shared<RecordingMapQuery>();
  FrameAwareRogQuery query(
    raw, tf_buffer, "map", "odom", rclcpp::get_logger("frame_aware_query_test"), clock);

  const auto result = query.query(Eigen::Vector3d(1.0, 2.0, 3.0));
  ASSERT_TRUE(result.ok);
  EXPECT_TRUE(raw->last_query.isApprox(Eigen::Vector3d(8.0, -1.0, 3.5), 1.0e-12));
  EXPECT_TRUE(result.gradient.isApprox(Eigen::Vector3d(0.0, -1.0, 2.0), 1.0e-12));
  EXPECT_DOUBLE_EQ(result.distance, 0.75);
  EXPECT_DOUBLE_EQ(result.snapshot_stamp, 123.0);

  unsigned int mx = 0U;
  unsigned int my = 0U;
  ASSERT_TRUE(query.worldToMap(1.0, 2.0, mx, my));
  EXPECT_TRUE(raw->last_world_to_map.isApprox(Eigen::Vector2d(8.0, -1.0), 1.0e-12));
  EXPECT_EQ(mx, 4U);
  EXPECT_EQ(my, 7U);

  double wx = 0.0;
  double wy = 0.0;
  query.mapToWorld(4U, 7U, wx, wy);
  EXPECT_EQ(raw->last_map_to_world, Eigen::Vector2i(4, 7));
  EXPECT_NEAR(wx, 1.0, 1.0e-12);
  EXPECT_NEAR(wy, 2.0, 1.0e-12);
}

TEST(FrameAwareRogQueryTest, BatchTransformsEveryPointAndGradient)
{
  auto clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(clock);

  geometry_msgs::msg::TransformStamped rog_from_planning;
  rog_from_planning.header.frame_id = "odom";
  rog_from_planning.child_frame_id = "map";
  rog_from_planning.transform.translation.x = 2.0;
  const double quarter_turn = 0.25 * std::acos(-1.0);
  rog_from_planning.transform.rotation.z = std::sin(quarter_turn);
  rog_from_planning.transform.rotation.w = std::cos(quarter_turn);
  ASSERT_TRUE(tf_buffer->setTransform(
    rog_from_planning, "frame_aware_batch_query_test", true));

  auto raw = std::make_shared<RecordingMapQuery>();
  FrameAwareRogQuery query(
    raw, tf_buffer, "map", "odom",
    rclcpp::get_logger("frame_aware_batch_query_test"), clock);
  const auto results = query.queryBatch(
    {Eigen::Vector3d(1.0, 0.0, 0.0), Eigen::Vector3d(0.0, 1.0, 0.0)});

  ASSERT_EQ(results.size(), 2U);
  ASSERT_TRUE(results[0].ok);
  ASSERT_TRUE(results[1].ok);
  EXPECT_TRUE(results[0].gradient.isApprox(Eigen::Vector3d(0.0, -1.0, 2.0), 1e-12));
  EXPECT_TRUE(results[1].gradient.isApprox(Eigen::Vector3d(0.0, -1.0, 2.0), 1e-12));
  EXPECT_DOUBLE_EQ(results[0].snapshot_stamp, 123.0);
  EXPECT_DOUBLE_EQ(results[1].snapshot_stamp, 123.0);
}

}  // namespace
}  // namespace minco_planner
