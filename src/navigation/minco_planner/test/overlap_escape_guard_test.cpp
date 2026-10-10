#include <gtest/gtest.h>
#include "minco_core/components/overlap_escape_guard.hpp"
#include "minco_core/components/map_query_adapters.hpp"
#include "minco_core/minco_utils.hpp"

namespace minco_planner {
namespace {
class FreshGrid : public Nav2CostmapQuery {
public:
  explicit FreshGrid(nav2_costmap_2d::Costmap2D * map) : Nav2CostmapQuery(map) {}
  rog_map::QueryResult query(const Eigen::Vector3d & position) const override {
    auto result = Nav2CostmapQuery::queryBatch({position}).front();
    result.snapshot_stamp = stamp;
    result.distance = result.projected_cost >= 253U ? penetration : 1.0;
    result.projection.valid = true;
    result.projection.dynamic_cost = result.projected_cost;
    return result;
  }
  std::vector<rog_map::QueryResult> queryBatch(const std::vector<Eigen::Vector3d> & positions) const override {
    return rog_map::MapQueryInterface::queryBatch(positions);
  }
  double stamp{10.0};
  double penetration{-0.025};
};

class OverlapEscapeTest : public ::testing::Test {
protected:
  OverlapEscapeTest() : map(40U, 40U, 0.05, -1.0, -1.0, 0U),
    query(std::make_shared<FreshGrid>(&map)), guard(configuration()) {
    for (unsigned int y = 0U; y < 40U; ++y) {
      for (unsigned int x = 26U; x < 40U; ++x) {
        map.setCost(x, y, 254U);
      }
    }
  }
  static OverlapEscapeGuard::Config configuration() {
    OverlapEscapeGuard::Config config;
    config.footprint = {{-0.30, -0.30}, {0.30, -0.30}, {0.30, 0.30}, {-0.30, 0.30}};
    return config;
  }
  nav2_costmap_2d::Costmap2D map;
  std::shared_ptr<FreshGrid> query;
  OverlapEscapeGuard guard;
};

TEST_F(OverlapEscapeTest, SmallOverlapCanRetreatButCannotAdvanceIntoWall) {
  const auto context = guard.begin({0.02, 0.0, 0.0}, 0.0, {-0.20, 0.0}, query, 10.0);
  ASSERT_TRUE(context);
  EXPECT_GT(context->last_overlap_area, 0.0);
  EXPECT_TRUE(guard.check(*context, {0.01, 0.0, 0.0}, 0.0, query, 10.1));
  EXPECT_FALSE(guard.check(*context, {0.02, 0.0, 0.0}, 0.0, query, 10.2));
  EXPECT_FALSE(guard.begin({0.02, 0.0, 0.0}, 0.0, {0.20, 0.0}, query, 10.0));
}

TEST_F(OverlapEscapeTest, StaticOverlapRetreatUsesOriginalCellsAndRechecksBothSources) {
  nav2_costmap_2d::Costmap2D observed(40U, 40U, 0.05, -1.025, -1.025, 0U);
  auto dynamic = std::make_shared<FreshGrid>(&observed);
  auto prior = std::make_shared<Nav2CostmapQuery>(&map);
  std::string reason;
  auto context = guard.begin({0.02,0,0}, 0, {-0.2,0}, dynamic, 10.0, &reason, prior);
  ASSERT_TRUE(context) << reason;
  ASSERT_TRUE(context->static_context);
  EXPECT_GT(context->static_context->last_overlap_area, 0.0);
  EXPECT_TRUE(guard.check(*context, {0.01,0,0}, 0, dynamic, 10.1));
  EXPECT_FALSE(guard.check(*context, {0.02,0,0}, 0, dynamic, 10.1));
  EXPECT_FALSE(guard.begin({0.02,0,0}, 0, {0.2,0}, dynamic, 10.0, &reason, prior));
  observed.setCost(16U, 20U, 254U);
  EXPECT_FALSE(guard.begin({0.02,0,0}, 0, {-0.2,0}, dynamic, 10.0, &reason, prior));
  observed.setCost(16U, 20U, 255U);
  EXPECT_FALSE(guard.begin({0.02,0,0}, 0, {-0.2,0}, dynamic, 10.0, &reason, prior));
}

TEST_F(OverlapEscapeTest, ShortensStaticEscapeInsideNarrowCorridor) {
  for (unsigned int y = 0U; y < 40U; ++y) {
    for (unsigned int x = 0U; x < 40U; ++x) {
      map.setCost(x, y, 0U);
    }
  }
  // Leave a 0.65 m corridor. A 0.60 m footprint starts with a small overlap
  // at the lower raster row; the configured 0.20 m retreat would hit the
  // upper row, while a short upward correction clears the lower row.
  for (unsigned int x = 0U; x < 40U; ++x) {
    for (unsigned int y = 14U; y <= 14U; ++y) {
      map.setCost(x, y, 254U);
    }
    for (unsigned int y = 28U; y < 40U; ++y) {
      map.setCost(x, y, 254U);
    }
  }
  auto prior = std::make_shared<Nav2CostmapQuery>(&map);
  std::string reason;
  const auto context = guard.begin({0.02, 0.03, 0.0}, 0.0, {0.0, 0.20},
    query, 10.0, &reason, prior);
  ASSERT_TRUE(context) << reason;
  EXPECT_GT(context->last_overlap_area, 0.0);
  EXPECT_LT(context->duration, 1.0);
  EXPECT_LT(context->distance, 0.20);
}

TEST_F(OverlapEscapeTest, StaticExitCanUseDrivingToleranceButMeasuredExitMustBeClear) {
  nav2_costmap_2d::Costmap2D observed(40U, 40U, 0.05, -1.0, -1.0, 0U);
  auto dynamic = std::make_shared<FreshGrid>(&observed);
  auto prior = std::make_shared<Nav2CostmapQuery>(&map);
  auto config = configuration();
  config.max_duration = 0.1;
  config.static_overlap = {0.04, 0.03};
  OverlapEscapeGuard tolerant(config);
  std::string reason;
  const auto context = tolerant.begin({0.04,0,0}, 0, {-0.2,0}, dynamic, 10.0, &reason, prior);
  ASSERT_TRUE(context) << reason;
  EXPECT_TRUE(tolerant.check(*context, {0.02,0,0}, 0, dynamic, 10.0));
  config.static_overlap.max_ratio = 0.0;
  EXPECT_FALSE(OverlapEscapeGuard(config).begin(
    {0.04,0,0}, 0, {-0.2,0}, dynamic, 10.0, &reason, prior));
  EXPECT_FALSE(tolerant.begin({0.04,0,0}, 0, {-0.2,0}, query, 10.0, &reason, prior));
}

TEST_F(OverlapEscapeTest, EscapeSearchIncludesWallNormalBetweenAngularSamples) {
  Eigen::Vector2d selected;
  const Eigen::Vector2d requested = Eigen::Vector2d(-1.0, 1.0).normalized() * 0.2;
  EXPECT_FALSE(guard.begin({0.02, 0.0, 0.0}, 0.0, requested, query, 10.0));
  ASSERT_TRUE(utils::selectSafeEscapeVelocity(requested,
    [&](const Eigen::Vector2d & velocity) {
      return static_cast<bool>(guard.begin({0.02, 0.0, 0.0}, 0.0, velocity, query, 10.0));
    }, selected));
  EXPECT_NEAR(selected.x(), -0.2, 1.0e-8);
  EXPECT_NEAR(selected.y(), 0.0, 1.0e-8);
}

TEST_F(OverlapEscapeTest, RejectsNewObstacleEvenWhileOriginalOverlapDecreases) {
  for (unsigned int y = 0U; y < 40U; ++y) {
    map.setCost(10U, y, 254U);
  }
  EXPECT_FALSE(guard.begin({0.02, 0.0, 0.0}, 0.0, {-0.20, 0.0}, query, 10.0));
}

TEST_F(OverlapEscapeTest, RejectsUnknownDestinationAndStaleOrInvalidMap) {
  map.setCost(10U, 20U, 255U);
  EXPECT_FALSE(guard.begin({0.02, 0.0, 0.0}, 0.0, {-0.20, 0.0}, query, 10.0));
  map.setCost(10U, 20U, 0U);
  query->stamp = 9.0;
  EXPECT_FALSE(guard.begin({0.02, 0.0, 0.0}, 0.0, {-0.20, 0.0}, query, 10.0));
  query->stamp = 10.1;
  EXPECT_FALSE(guard.begin({0.02, 0.0, 0.0}, 0.0, {-0.20, 0.0}, query, 10.0));
  query->stamp = 10.0;
  query->penetration = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(guard.begin({0.02, 0.0, 0.0}, 0.0, {-0.20, 0.0}, query, 10.0));
}

TEST_F(OverlapEscapeTest, RejectsDeepLargeOrUnresolvedOverlapAndExcessiveSpeed) {
  std::string reason;
  EXPECT_FALSE(guard.begin({0.15, 0.0, 0.0}, 0.0, {-0.20, 0.0}, query, 10.0, &reason));
  EXPECT_EQ(reason, "OVERLAP_AREA_LIMIT");
  EXPECT_FALSE(guard.begin({0.02, 0.0, 0.0}, 0.0, {-0.01, 0.0}, query, 10.0));
  EXPECT_FALSE(guard.begin({0.02, 0.0, 0.0}, 0.0, {-0.21, 0.0}, query, 10.0));
  query->penetration = -0.10;
  EXPECT_FALSE(guard.begin({0.02, 0.0, 0.0}, 0.0, {-0.20, 0.0}, query, 10.0));
}

TEST_F(OverlapEscapeTest, RechecksRotationTrackingAndNewlyOccupiedCells) {
  auto context = guard.begin({0.02, 0.0, 0.0}, 0.0, {-0.20, 0.0}, query, 10.0);
  ASSERT_TRUE(context);
  EXPECT_FALSE(guard.check(*context, {0.02, 0.0, 0.0}, 0.04, query, 10.0));
  EXPECT_FALSE(guard.check(*context, {0.02, 0.04, 0.0}, 0.0, query, 10.0));
  map.setCost(17U, 20U, 254U);
  EXPECT_FALSE(guard.check(*context, {0.02, 0.0, 0.0}, 0.0, query, 10.0));
}

TEST_F(OverlapEscapeTest, CompletedRetreatIsClearAndDoesNotRefreshOriginalAllowance) {
  auto context = guard.begin({0.02, 0.0, 0.0}, 0.0, {-0.20, 0.0}, query, 10.0);
  ASSERT_TRUE(context);
  EXPECT_TRUE(guard.check(*context, {-0.18, 0.0, 0.0}, 0.0, query, 10.0));
  EXPECT_NEAR(context->last_overlap_area, 0.0, 1.0e-8);
  EXPECT_FALSE(guard.check(*context, {0.01, 0.0, 0.0}, 0.0, query, 10.0));
}
}  // namespace
}  // namespace minco_planner
