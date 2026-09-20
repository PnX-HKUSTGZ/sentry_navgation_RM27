#include <gtest/gtest.h>

#include "smac_search/smac_planner_2d_simple.hpp"
#include "minco_core/components/dynamic_obstacle_evidence.hpp"

#include "nav2_costmap_2d/cost_values.hpp"
#include "rog_map/projection_layer.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <vector>

namespace minco_planner
{
namespace smac
{
namespace
{

class GridQuery : public rog_map::MapQueryInterface
{
public:
  GridQuery(
    unsigned int width, unsigned int height, double resolution, double distance,
    double query_max_x = std::numeric_limits<double>::infinity())
  : width_(width), height_(height), resolution_(resolution), distance_(distance),
    query_max_x_(query_max_x),
    values_(static_cast<size_t>(width) * static_cast<size_t>(height),
      nav2_costmap_2d::FREE_SPACE)
  {
  }

  bool worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my) const override
  {
    if (!std::isfinite(wx) || !std::isfinite(wy) || wx < 0.0 || wy < 0.0) {
      return false;
    }
    mx = static_cast<unsigned int>(wx / resolution_);
    my = static_cast<unsigned int>(wy / resolution_);
    return isValid(mx, my);
  }

  void mapToWorld(unsigned int mx, unsigned int my, double & wx, double & wy) const override
  {
    wx = (static_cast<double>(mx) + 0.5) * resolution_;
    wy = (static_cast<double>(my) + 0.5) * resolution_;
  }

  unsigned int sizeX() const override {return width_;}
  unsigned int sizeY() const override {return height_;}
  double resolution() const override {return resolution_;}
  double originX() const override {return 0.0;}
  double originY() const override {return 0.0;}

  uint8_t value(unsigned int mx, unsigned int my) const override
  {
    return isValid(mx, my) ? values_[static_cast<size_t>(my) * width_ + mx] :
           nav2_costmap_2d::LETHAL_OBSTACLE;
  }

  const unsigned char * values() const override {return values_.data();}

  bool copyValues(std::vector<unsigned char> & out) const override
  {
    out = values_;
    return true;
  }

  bool isValid(unsigned int mx, unsigned int my) const override
  {
    return mx < width_ && my < height_;
  }

  bool isFree(unsigned int mx, unsigned int my) const override
  {
    return isValid(mx, my) && value(mx, my) < nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
  }

  void setValue(unsigned int mx, unsigned int my, uint8_t value)
  {
    if (isValid(mx, my)) {
      values_[static_cast<size_t>(my) * width_ + mx] = value;
    }
  }

  rog_map::QueryResult query(const Eigen::Vector3d & pos) const override
  {
    rog_map::QueryResult result;
    if (!pos.allFinite()) {
      result.status = rog_map::QueryStatus::NONFINITE_INPUT;
      return result;
    }
    if (pos.x() > query_max_x_) {
      result.status = rog_map::QueryStatus::OUT_OF_MAP;
      return result;
    }
    result.ok = true;
    result.status = rog_map::QueryStatus::OK;
    result.distance = distance_;
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

private:
  unsigned int width_;
  unsigned int height_;
  double resolution_;
  double distance_;
  double query_max_x_;
  std::vector<unsigned char> values_;
};

class UnverifiedOccupiedQuery : public GridQuery
{
public:
  using GridQuery::GridQuery;
  mutable size_t query_count{0U};

  rog_map::QueryResult query(const Eigen::Vector3d & pos) const override
  {
    ++query_count;
    auto result = GridQuery::query(pos);
    if (!result.ok) {
      return result;
    }
    result.distance = 0.0;
    result.projected_cost_valid = true;
    result.projected_cost = nav2_costmap_2d::LETHAL_OBSTACLE;
    result.projection.valid = true;
    result.projection.cost_source =
      rog_map::ProjectedCostSource::DYNAMIC_PROJECTION;
    result.projection.cost_cause = rog_map::ProjectedCostCause::RAW_OCCUPIED;
    result.projection.dynamic_cost = nav2_costmap_2d::LETHAL_OBSTACLE;
    const auto occupied = static_cast<uint8_t>(rog_map::CellType::OCCUPIED);
    result.projection.cell_type = occupied;
    result.projection.raw_type = occupied;
    result.projection.candidate_type = occupied;
    result.projection.base_type = occupied;
    result.projection.raw_reason = static_cast<uint8_t>(
      rog_map::ProjectionClassReason::HEADROOM_UNVERIFIED);
    result.projection.candidate_reason = result.projection.raw_reason;
    return result;
  }
};

class MeasuredWallQuery : public GridQuery
{
public:
  using GridQuery::GridQuery;

  rog_map::QueryResult query(const Eigen::Vector3d & pos) const override
  {
    auto result = GridQuery::query(pos);
    if (!result.ok) {
      return result;
    }
    const unsigned int mx = static_cast<unsigned int>(pos.x() / resolution());
    const unsigned int my = static_cast<unsigned int>(pos.y() / resolution());
    if (mx != 10U || my < 4U || my > 15U) {
      return result;
    }
    result.distance = 0.0;
    result.projected_cost_valid = true;
    result.projected_cost = nav2_costmap_2d::LETHAL_OBSTACLE;
    result.projection.valid = true;
    result.projection.cost_source =
      rog_map::ProjectedCostSource::DYNAMIC_PROJECTION;
    result.projection.cost_cause = rog_map::ProjectedCostCause::RAW_OCCUPIED;
    result.projection.dynamic_cost = nav2_costmap_2d::LETHAL_OBSTACLE;
    const auto occupied = static_cast<uint8_t>(rog_map::CellType::OCCUPIED);
    result.projection.cell_type = occupied;
    result.projection.raw_type = occupied;
    result.projection.candidate_type = occupied;
    result.projection.base_type = occupied;
    result.projection.raw_reason =
      static_cast<uint8_t>(rog_map::ProjectionClassReason::HEADROOM_BLOCKED);
    result.projection.candidate_reason = result.projection.raw_reason;
    result.projection.occupied_z_min = 0.0F;
    result.projection.occupied_z_max = 0.5F;
    return result;
  }
};

TEST(SmacGlobalSearchPolicyTest, FreeGlobalMapEscapesBlockedLocalEsdfStartRegion)
{
  auto global_map = std::make_shared<GridQuery>(100U, 70U, 0.1, 10.0);
  auto blocked_local_esdf = std::make_shared<GridQuery>(100U, 70U, 0.1, 0.0, 5.3);

  SmacPlanner2DSimple planner;
  planner.setMap(global_map);
  planner.setESDFQuery(blocked_local_esdf);
  planner.setCollisionDistance(0.0);
  planner.setParameters(false, 1000000, 0.0F);

  SmacPlanner2DSimple::CoordinateVector path;
  ASSERT_TRUE(planner.createPath(50U, 35U, 80U, 35U, path));
  ASSERT_GE(path.size(), 2U);
  EXPECT_FLOAT_EQ(path.front().x, 80.0F);
  EXPECT_FLOAT_EQ(path.front().y, 35.0F);
  EXPECT_FLOAT_EQ(path.back().x, 50.0F);
  EXPECT_FLOAT_EQ(path.back().y, 35.0F);
}

TEST(SmacGlobalSearchPolicyTest, ReproducesIterationOneTrapWhenLocalEsdfIsHard)
{
  auto global_map = std::make_shared<GridQuery>(100U, 70U, 0.1, 10.0);
  auto blocked_local_esdf = std::make_shared<GridQuery>(100U, 70U, 0.1, 0.0, 5.3);

  SmacPlanner2DSimple planner;
  planner.setMap(global_map);
  planner.setESDFQuery(blocked_local_esdf);
  planner.setCollisionDistance(0.22);
  planner.setParameters(false, 1000000, 0.0F);

  SmacPlanner2DSimple::CoordinateVector path;
  EXPECT_FALSE(planner.createPath(50U, 35U, 80U, 35U, path));
  EXPECT_TRUE(path.empty());
}

TEST(SmacGlobalSearchPolicyTest,
  FailClosedUnverifiedColumnsDoNotBecomeGlobalTopologyObstacles)
{
  auto global_map = std::make_shared<GridQuery>(30U, 20U, 0.1, 10.0);
  auto unverified =
    std::make_shared<UnverifiedOccupiedQuery>(30U, 20U, 0.1, 0.0);

  SmacPlanner2DSimple planner;
  planner.setMap(global_map);
  planner.setESDFQuery(unverified);
  planner.setCollisionDistance(0.26);
  planner.setParameters(false, 1000000, 0.0F);

  SmacPlanner2DSimple::CoordinateVector path;
  ASSERT_TRUE(planner.createPath(2U, 10U, 25U, 10U, path));
  ASSERT_GE(path.size(), 2U);
  EXPECT_LE(unverified->query_count, 30U * 20U);
  unverified->query_count = 0U;
  ASSERT_TRUE(planner.createPath(2U, 10U, 25U, 10U, path));
  EXPECT_GT(unverified->query_count, 0U);
  EXPECT_LE(unverified->query_count, 30U * 20U);
}

TEST(SmacGlobalSearchPolicyTest, MeasuredWallForcesGlobalDetour)
{
  auto global_map = std::make_shared<GridQuery>(20U, 20U, 0.1, 10.0);
  auto measured_wall =
    std::make_shared<MeasuredWallQuery>(20U, 20U, 0.1, 10.0);

  SmacPlanner2DSimple planner;
  planner.setMap(global_map);
  planner.setESDFQuery(measured_wall);
  planner.setCollisionDistance(0.05);
  planner.setParameters(false, 1000000, 0.0F);

  SmacPlanner2DSimple::CoordinateVector path;
  ASSERT_TRUE(planner.createPath(2U, 10U, 17U, 10U, path));
  ASSERT_GE(path.size(), 2U);
  for (const auto & point : path) {
    EXPECT_FALSE(
      static_cast<unsigned int>(point.x) == 10U && point.y >= 4.0F && point.y <= 15.0F);
  }
}

TEST(SmacGlobalSearchPolicyTest, LinearInflationPenaltyAvoidsSoftCostBand)
{
  auto global_map = std::make_shared<GridQuery>(40U, 20U, 0.1, 10.0);
  for (unsigned int y = 7U; y <= 12U; ++y) {
    for (unsigned int x = 5U; x <= 34U; ++x) {
      global_map->setValue(x, y, 45U);
    }
  }

  SmacPlanner2DSimple planner;
  planner.setMap(global_map);
  planner.setCollisionDistance(0.0);
  planner.setParameters(false, 1000000, 0.0F);

  SmacPlanner2DSimple::CoordinateVector path;
  planner.setInflationCostParameters(0.0F, false);
  ASSERT_TRUE(planner.createPath(2U, 9U, 37U, 9U, path));
  const auto unweighted_soft_cells = std::count_if(
    path.begin(), path.end(), [](const auto & point) {
      return point.x >= 5.0F && point.x <= 34.0F &&
             point.y >= 7.0F && point.y <= 12.0F;
    });
  EXPECT_GE(unweighted_soft_cells, 25);

  planner.setInflationCostParameters(2.0F, false);
  ASSERT_TRUE(planner.createPath(2U, 9U, 37U, 9U, path));
  const auto weighted_soft_cells = std::count_if(
    path.begin(), path.end(), [](const auto & point) {
      return point.x >= 5.0F && point.x <= 34.0F &&
             point.y >= 7.0F && point.y <= 12.0F;
    });
  EXPECT_LE(weighted_soft_cells, 2);
}

TEST(SmacGlobalSearchPolicyTest, EvidenceGateRequiresMeasuredSpanAndObstacleReason)
{
  MeasuredWallQuery measured_wall(20U, 20U, 0.1, 10.0);
  auto result = measured_wall.query(Eigen::Vector3d(1.05, 1.05, 0.0));
  ASSERT_TRUE(dynamic_obstacle::hasMeasuredOccupiedEvidence(result));

  for (const auto reason : {rog_map::ProjectionClassReason::HEADROOM_UNVERIFIED,
      rog_map::ProjectionClassReason::GROUND_UNVERIFIED,
      rog_map::ProjectionClassReason::OVERHEAD_CLEARANCE_OK})
  {
    auto unverified = result;
    unverified.projection.raw_reason = static_cast<uint8_t>(reason);
    unverified.projection.candidate_reason = static_cast<uint8_t>(reason);
    EXPECT_FALSE(dynamic_obstacle::hasMeasuredOccupiedEvidence(unverified));
  }
  for (const auto cause : {rog_map::ProjectedCostCause::UNKNOWN_AS_OCCUPIED,
      rog_map::ProjectedCostCause::MASK_DENOISE_TO_UNKNOWN,
      rog_map::ProjectedCostCause::MASK_HOLE_FILL})
  {
    auto filtered = result;
    filtered.projection.cost_cause = cause;
    EXPECT_FALSE(dynamic_obstacle::hasMeasuredOccupiedEvidence(filtered));
  }
  result.projection.occupied_z_min = std::numeric_limits<float>::quiet_NaN();
  EXPECT_FALSE(dynamic_obstacle::hasMeasuredOccupiedEvidence(result));
}

TEST(SmacGlobalSearchPolicyTest,
     BuildsShortSegmentWhenStartIsInsideGoalTolerance) {
  auto global_map = std::make_shared<GridQuery>(30U, 30U, 0.1, 10.0);

  SmacPlanner2DSimple planner;
  planner.setMap(global_map);
  planner.setCollisionDistance(0.0);
  planner.setParameters(false, 1000000, 0.30F);

  SmacPlanner2DSimple::CoordinateVector path;
  ASSERT_TRUE(planner.createPath(10U, 10U, 12U, 12U, path));
  ASSERT_EQ(path.size(), 2U);
  EXPECT_FLOAT_EQ(path.front().x, 12.0F);
  EXPECT_FLOAT_EQ(path.front().y, 12.0F);
  EXPECT_FLOAT_EQ(path.back().x, 10.0F);
  EXPECT_FLOAT_EQ(path.back().y, 10.0F);
}

} // namespace
} // namespace smac
}  // namespace minco_planner
