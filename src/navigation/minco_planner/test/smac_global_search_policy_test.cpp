#include <gtest/gtest.h>

#include "smac_search/smac_planner_2d_simple.hpp"

#include "nav2_costmap_2d/cost_values.hpp"

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
