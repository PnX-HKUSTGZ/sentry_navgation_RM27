#include <gtest/gtest.h>

#include "minco_core/components/local_path_processor.hpp"
#include "minco_core/components/planner_mode_context.hpp"
#include "minco_core/components/trajectory_safety_checker.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <utility>
#include <vector>

namespace minco_planner
{
namespace
{

class TestGridQuery : public rog_map::MapQueryInterface
{
public:
  using CostAt = std::function<uint8_t(double, double)>;

  explicit TestGridQuery(CostAt cost_at, double resolution = 0.05)
  : cost_at_(std::move(cost_at)), resolution_(resolution),
    values_(120U * 80U, nav2_costmap_2d::FREE_SPACE)
  {
  }

  bool worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my) const override
  {
    if (!std::isfinite(wx) || !std::isfinite(wy) || wx < originX() || wy < originY()) {
      return false;
    }
    mx = static_cast<unsigned int>((wx - originX()) / resolution_);
    my = static_cast<unsigned int>((wy - originY()) / resolution_);
    return isValid(mx, my);
  }

  void mapToWorld(unsigned int mx, unsigned int my, double & wx, double & wy) const override
  {
    wx = originX() + (static_cast<double>(mx) + 0.5) * resolution_;
    wy = originY() + (static_cast<double>(my) + 0.5) * resolution_;
  }

  unsigned int sizeX() const override {return 120U;}
  unsigned int sizeY() const override {return 80U;}
  double resolution() const override {return resolution_;}
  double originX() const override {return -2.0;}
  double originY() const override {return -2.0;}

  uint8_t value(unsigned int mx, unsigned int my) const override
  {
    if (!isValid(mx, my)) {
      return nav2_costmap_2d::NO_INFORMATION;
    }
    double wx = 0.0;
    double wy = 0.0;
    mapToWorld(mx, my, wx, wy);
    return cost_at_(wx, wy);
  }

  const unsigned char * values() const override {return values_.data();}
  bool isValid(unsigned int mx, unsigned int my) const override
  {
    return mx < sizeX() && my < sizeY();
  }
  bool isFree(unsigned int mx, unsigned int my) const override
  {
    const uint8_t cost = value(mx, my);
    return cost != nav2_costmap_2d::NO_INFORMATION &&
           cost < nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
  }

  rog_map::QueryResult query(const Eigen::Vector3d & position) const override
  {
    rog_map::QueryResult result;
    unsigned int mx = 0U;
    unsigned int my = 0U;
    if (!position.allFinite()) {
      result.status = rog_map::QueryStatus::NONFINITE_INPUT;
      return result;
    }
    if (!worldToMap(position.x(), position.y(), mx, my)) {
      result.status = rog_map::QueryStatus::OUT_OF_MAP;
      return result;
    }
    result.ok = true;
    result.status = rog_map::QueryStatus::OK;
    result.distance = 1.0;
    result.snapshot_stamp = rclcpp::Clock(RCL_SYSTEM_TIME).now().seconds();
    return result;
  }

  bool evaluate(
    const Eigen::Vector3d & position, double & distance,
    Eigen::Vector3d & gradient) const override
  {
    const auto result = query(position);
    distance = result.distance;
    gradient = result.gradient;
    return result.ok;
  }

private:
  CostAt cost_at_;
  double resolution_;
  std::vector<unsigned char> values_;
};

geometry_msgs::msg::PoseStamped pose(double x, double y, double yaw = 0.0)
{
  geometry_msgs::msg::PoseStamped result;
  result.pose.position.x = x;
  result.pose.position.y = y;
  result.pose.orientation.z = std::sin(0.5 * yaw);
  result.pose.orientation.w = std::cos(0.5 * yaw);
  return result;
}

PlannerModeContext makeContext(const std::shared_ptr<TestGridQuery> & query)
{
  PlannerModeParams params;
  params.planner_mode = "EXPLORATION";
  params.exploration_boundary_margin = 0.0;
  params.exploration_boundary_sample_step = 0.05;
  PlannerModeContext context;
  context.configure(
    params, query, nullptr, nullptr, rclcpp::get_logger("local_path_processor_test"),
    std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME));
  return context;
}

LocalPathProcessor makeProcessor(double lookahead = 5.0)
{
  LocalPathProcessor processor;
  processor.configure(
    lookahead, 0.5, 1.0, 0.05, 10.0, 5.0,
    rclcpp::get_logger("local_path_processor_test"),
    std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME));
  return processor;
}

std::unique_ptr<TrajectorySafetyChecker> makeSafetyChecker(
  const std::shared_ptr<TestGridQuery> & query)
{
  TrajectorySafetyChecker::Config config;
  config.safe_dist = 0.0;
  config.footprint_length = 0.30;
  config.footprint_width = 0.20;
  config.footprint_margin = 0.05;
  config.map_timeout = 0.50;
  auto checker = std::make_unique<TrajectorySafetyChecker>();
  checker->configure(
    config, rclcpp::get_logger("local_path_processor_test"),
    std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME));
  checker->setQuery(query);
  return checker;
}

TEST(LocalPathProcessorTest, FullFootprintClipsBeforeUnknownObservedBoundary)
{
  auto query = std::make_shared<TestGridQuery>(
    [](double x, double) {
      return x >= 0.80 ? nav2_costmap_2d::NO_INFORMATION : nav2_costmap_2d::FREE_SPACE;
    });
  auto context = makeContext(query);
  auto processor = makeProcessor();
  auto checker = makeSafetyChecker(query);

  const std::vector<geometry_msgs::msg::PoseStamped> path{
    pose(0.0, 0.0), pose(1.0, 0.0), pose(2.0, 0.0)};
  const auto seed = processor.buildSeed(
    path, pose(0.0, 0.0), context,
    [&checker](const Eigen::Vector3d & position, double yaw) {
      return checker->checkFootprint(position, yaw);
    });

  ASSERT_TRUE(seed.valid);
  ASSERT_TRUE(seed.observed_prefix_clipped);
  EXPECT_TRUE(seed.stop_at_local_end);
  EXPECT_FALSE(seed.local_end_is_goal);
  ASSERT_GE(seed.dense_path.size(), 2U);
  EXPECT_LT(seed.dense_path.back().x(), 0.65);
  EXPECT_GT(seed.dense_path.back().x(), 0.50);
}

TEST(LocalPathProcessorTest, ObservedPrefixStopKeepsMeasuredOmniYaw)
{
  LocalPathSeed rolling_seed;
  EXPECT_TRUE(shouldOptimizeYawForSeed(true, rolling_seed));

  rolling_seed.observed_prefix_clipped = true;
  EXPECT_FALSE(shouldOptimizeYawForSeed(true, rolling_seed));
  EXPECT_FALSE(shouldOptimizeYawForSeed(false, rolling_seed));
}

TEST(LocalPathProcessorTest, UnsafeStartingFootprintFailsClosed)
{
  auto query = std::make_shared<TestGridQuery>(
    [](double, double) {return nav2_costmap_2d::NO_INFORMATION;});
  auto context = makeContext(query);
  auto processor = makeProcessor();
  auto checker = makeSafetyChecker(query);

  const auto seed = processor.buildSeed(
    {pose(0.0, 0.0), pose(1.0, 0.0)}, pose(0.0, 0.0), context,
    [&checker](const Eigen::Vector3d & position, double yaw) {
      return checker->checkFootprint(position, yaw);
    });

  EXPECT_FALSE(seed.valid);
  EXPECT_TRUE(seed.dense_path.empty());
  EXPECT_TRUE(seed.sparse_waypoints.empty());
}

TEST(LocalPathProcessorTest, NoSafeProgressAlongFirstSegmentFailsClosed)
{
  auto query = std::make_shared<TestGridQuery>(
    [](double, double) {return nav2_costmap_2d::FREE_SPACE;});
  auto context = makeContext(query);
  auto processor = makeProcessor();
  size_t checks = 0U;

  const auto seed = processor.buildSeed(
    {pose(0.0, 0.0), pose(1.0, 0.0)}, pose(0.0, 0.0), context,
    [&checks](const Eigen::Vector3d &, double) {return checks++ < 2U;});

  EXPECT_FALSE(seed.valid);
  EXPECT_TRUE(seed.observed_prefix_clipped);
  ASSERT_EQ(seed.dense_path.size(), 1U);
  EXPECT_DOUBLE_EQ(seed.dense_path.front().x(), 0.0);
}

TEST(LocalPathProcessorTest, InvalidCurrentYawFailsClosedBeforeSafetyCallback)
{
  auto query = std::make_shared<TestGridQuery>(
    [](double, double) {return nav2_costmap_2d::FREE_SPACE;});
  auto context = makeContext(query);
  auto processor = makeProcessor();
  geometry_msgs::msg::PoseStamped invalid_current_pose;
  invalid_current_pose.pose.orientation.x = 0.0;
  invalid_current_pose.pose.orientation.y = 0.0;
  invalid_current_pose.pose.orientation.z = 0.0;
  invalid_current_pose.pose.orientation.w = 0.0;
  size_t checks = 0U;

  const auto seed = processor.buildSeed(
    {pose(0.0, 0.0), pose(1.0, 0.0)}, invalid_current_pose, context,
    [&checks](const Eigen::Vector3d &, double) {
      ++checks;
      return true;
    });

  EXPECT_FALSE(seed.valid);
  EXPECT_EQ(checks, 0U);
}

TEST(LocalPathProcessorTest, FullyObservedGoalPreservesPathAndUsesMeasuredOmniYaw)
{
  constexpr double kMeasuredYaw = 0.305;
  auto query = std::make_shared<TestGridQuery>(
    [](double, double) {return nav2_costmap_2d::FREE_SPACE;});
  auto context = makeContext(query);
  auto processor = makeProcessor();
  std::vector<double> sampled_yaws;
  std::vector<Eigen::Vector3d> sampled_positions;

  const std::vector<geometry_msgs::msg::PoseStamped> path{
    pose(0.0, 0.0), pose(0.25, 0.0), pose(0.25, 0.25)};
  const auto seed = processor.buildSeed(
    path, pose(0.0, 0.0, kMeasuredYaw), context,
    [&sampled_yaws, &sampled_positions](const Eigen::Vector3d & position, double yaw) {
      sampled_positions.push_back(position);
      sampled_yaws.push_back(yaw);
      return true;
    });

  ASSERT_TRUE(seed.valid);
  EXPECT_FALSE(seed.observed_prefix_clipped);
  EXPECT_TRUE(seed.local_end_is_goal);
  EXPECT_TRUE(seed.stop_at_local_end);
  ASSERT_EQ(seed.dense_path.size(), path.size());
  EXPECT_TRUE(seed.dense_path.back().isApprox(Eigen::Vector3d(0.25, 0.25, 0.0)));

  ASSERT_FALSE(sampled_yaws.empty());
  for (const double yaw : sampled_yaws) {
    EXPECT_NEAR(yaw, kMeasuredYaw, 1e-9);
  }

  // Inspect the dense-prefix pass. Sparsification then rechecks independent
  // shortcuts and can legitimately restart sampling from earlier positions.
  const auto dense_end = std::find_if(
    sampled_positions.begin(), sampled_positions.end(),
    [](const Eigen::Vector3d & p) {return p.isApprox(Eigen::Vector3d(0.25, 0.25, 0.0));});
  ASSERT_NE(dense_end, sampled_positions.end());
  const size_t dense_checks = static_cast<size_t>(std::distance(sampled_positions.begin(), dense_end)) + 1U;
  for (size_t i = 1U; i < dense_checks; ++i) {
    const double step = (sampled_positions[i] - sampled_positions[i - 1U]).head<2>().norm();
    if (step > 1e-9) {
      EXPECT_LE(step, 0.025 + 1e-9);
    }
  }
}

TEST(LocalPathProcessorTest, DynamicFootprintVetoPreservesDetourDuringSparsification)
{
  auto query = std::make_shared<TestGridQuery>(
    [](double, double) {return nav2_costmap_2d::FREE_SPACE;});
  auto context = makeContext(query);
  auto processor = makeProcessor();
  processor.updateLimits(2.0, 4.0, 0.05);
  std::vector<geometry_msgs::msg::PoseStamped> path;
  for (int i = 0; i <= 16; ++i) {
    path.push_back(pose(0.0, i * 0.05));
  }
  for (int i = 1; i <= 32; ++i) {
    path.push_back(pose(i * 0.05, 0.80));
  }
  for (int i = 1; i <= 16; ++i) {
    path.push_back(pose(1.60, 0.80 - i * 0.05));
  }
  const auto footprint_is_safe = [](const Eigen::Vector3d & p, double) {
      return !(p.x() > 0.05 && p.x() < 1.55 && p.y() < 0.78);
    };
  const auto seed = processor.buildSeed(path, pose(0.0, 0.0), context, footprint_is_safe);

  ASSERT_TRUE(seed.valid);
  EXPECT_FALSE(seed.observed_prefix_clipped);
  ASSERT_GE(seed.sparse_waypoints.size(), 4U);
  for (size_t i = 1U; i < seed.sparse_waypoints.size(); ++i) {
    const auto & a = seed.sparse_waypoints[i - 1U];
    const auto & b = seed.sparse_waypoints[i];
    const int samples = std::max(1, static_cast<int>(std::ceil((b - a).norm() / 0.01)));
    for (int sample = 0; sample <= samples; ++sample) {
      EXPECT_TRUE(footprint_is_safe(a + (b - a) * (static_cast<double>(sample) / samples), 0.0));
    }
  }
}

TEST(LocalPathProcessorTest, InflationCostEnvelopePreservesLowerCostDetour)
{
  auto query = std::make_shared<TestGridQuery>(
    [](double x, double y) {
      const bool inside_soft_band = x > 0.20 && x < 1.40 && std::abs(y) < 0.25;
      return inside_soft_band ? static_cast<uint8_t>(180U) : nav2_costmap_2d::FREE_SPACE;
    });
  auto context = makeContext(query);
  auto processor = makeProcessor();
  processor.updateLimits(2.0, 4.0, 0.05);

  std::vector<geometry_msgs::msg::PoseStamped> path;
  for (int i = 0; i <= 8; ++i) {
    path.push_back(pose(0.0, i * 0.05));
  }
  for (int i = 1; i <= 32; ++i) {
    path.push_back(pose(i * 0.05, 0.40));
  }
  for (int i = 1; i <= 8; ++i) {
    path.push_back(pose(1.60, 0.40 - i * 0.05));
  }

  const auto seed = processor.buildSeed(
    path, pose(0.0, 0.0), context,
    [](const Eigen::Vector3d &, double) {return true;});

  ASSERT_TRUE(seed.valid);
  ASSERT_GE(seed.sparse_waypoints.size(), 4U);
  for (size_t index = 1U; index < seed.sparse_waypoints.size(); ++index) {
    const auto & a = seed.sparse_waypoints[index - 1U];
    const auto & b = seed.sparse_waypoints[index];
    const int samples = std::max(1, static_cast<int>(std::ceil((b - a).norm() / 0.02)));
    for (int sample = 0; sample <= samples; ++sample) {
      const Eigen::Vector3d point =
        a + (b - a) * (static_cast<double>(sample) / samples);
      if (point.x() > 0.20 && point.x() < 1.40) {
        EXPECT_GE(point.y(), 0.25 - 1e-9);
      }
    }
  }
}

TEST(LocalPathProcessorTest, UniformlyInflatedNarrowPassageStillAllowsShortcut)
{
  auto query = std::make_shared<TestGridQuery>(
    [](double, double) {return static_cast<uint8_t>(180U);});
  auto context = makeContext(query);
  auto processor = makeProcessor();
  processor.updateLimits(2.0, 4.0, 0.05);
  const std::vector<geometry_msgs::msg::PoseStamped> path{
    pose(0.0, 0.0), pose(0.20, 0.02), pose(0.40, 0.0), pose(0.60, 0.02),
    pose(0.80, 0.0), pose(1.00, 0.02), pose(1.20, 0.0)};

  const auto seed = processor.buildSeed(
    path, pose(0.0, 0.0), context,
    [](const Eigen::Vector3d &, double) {return true;});

  ASSERT_TRUE(seed.valid);
  EXPECT_LT(seed.sparse_waypoints.size(), seed.dense_path.size());
  EXPECT_TRUE(seed.sparse_waypoints.front().isApprox(Eigen::Vector3d(0.0, 0.0, 0.0)));
  EXPECT_TRUE(seed.sparse_waypoints.back().isApprox(Eigen::Vector3d(1.20, 0.0, 0.0)));
}

TEST(LocalPathProcessorTest, SparseRepairNeverAppendsUncheckedCornerOrGoal)
{
  const std::vector<Eigen::Vector3d> path{
    {0.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {1.0, 1.0, 0.0}, {1.0, 0.0, 0.0}};
  const auto sparse = utils::getSparseWaypoints(
    path, 2.0, 4.0, true,
    [](const Eigen::Vector3d &, const Eigen::Vector3d &) {return false;});
  EXPECT_TRUE(sparse.empty());
}

TEST(LocalPathProcessorTest, SafeRollingHorizonKeepsCruiseEndBehavior)
{
  auto query = std::make_shared<TestGridQuery>(
    [](double, double) {return nav2_costmap_2d::FREE_SPACE;});
  auto context = makeContext(query);
  auto processor = makeProcessor(0.50);
  const std::vector<geometry_msgs::msg::PoseStamped> path{
    pose(0.0, 0.0), pose(0.25, 0.0), pose(0.50, 0.0), pose(0.75, 0.0), pose(1.0, 0.0)};

  const auto seed = processor.buildSeed(
    path, pose(0.0, 0.0), context,
    [](const Eigen::Vector3d &, double) {return true;});

  ASSERT_TRUE(seed.valid);
  EXPECT_FALSE(seed.observed_prefix_clipped);
  EXPECT_FALSE(seed.local_end_is_goal);
  EXPECT_FALSE(seed.stop_at_local_end);
  EXPECT_TRUE(seed.dense_path.back().isApprox(Eigen::Vector3d(0.50, 0.0, 0.0)));
}

TEST(LocalPathProcessorTest, RollingReplanSeedStartsAtMeasuredPoseNotOldGlobalWaypoint)
{
  auto query = std::make_shared<TestGridQuery>(
    [](double, double) {return nav2_costmap_2d::FREE_SPACE;});
  auto context = makeContext(query);
  auto processor = makeProcessor();
  std::vector<Eigen::Vector3d> checked_positions;
  std::vector<double> checked_yaws;
  const std::vector<geometry_msgs::msg::PoseStamped> path{
    pose(0.0, 0.10), pose(0.25, 0.10), pose(0.50, 0.10), pose(0.75, 0.10)};

  const auto seed = processor.buildSeed(
    path, pose(0.35, 0.0), context,
    [&checked_positions, &checked_yaws](const Eigen::Vector3d & position, double yaw) {
      checked_positions.push_back(position);
      checked_yaws.push_back(yaw);
      return true;
    });

  ASSERT_TRUE(seed.valid);
  ASSERT_GE(seed.dense_path.size(), 2U);
  EXPECT_TRUE(seed.dense_path.front().isApprox(Eigen::Vector3d(0.35, 0.0, 0.0)));
  EXPECT_DOUBLE_EQ(seed.dense_path[1].x(), 0.50);
  EXPECT_DOUBLE_EQ(seed.dense_path[1].y(), 0.10);
  ASSERT_FALSE(checked_positions.empty());
  EXPECT_TRUE(checked_positions.front().isApprox(Eigen::Vector3d(0.35, 0.0, 0.0)));
  ASSERT_EQ(checked_positions.size(), checked_yaws.size());
  for (size_t i = 0U; i < checked_positions.size(); ++i) {
    const auto & checked = checked_positions[i];
    EXPECT_GE(checked.x(), 0.35 - 1e-9);
    EXPECT_NEAR(checked_yaws[i], 0.0, 1e-9);
  }
}

TEST(LocalPathProcessorTest, ChoosesNearestPolylineSegmentOverNearbyCornerVertex)
{
  auto query = std::make_shared<TestGridQuery>(
    [](double, double) {return nav2_costmap_2d::FREE_SPACE;});
  auto context = makeContext(query);
  auto processor = makeProcessor();
  const std::vector<geometry_msgs::msg::PoseStamped> path{
    pose(0.0, 0.0), pose(3.5, 0.0), pose(1.8, 1.0), pose(1.8, 1.8)};

  // The nearest stored vertex is (5, 1), but the robot is approaching the
  // long horizontal segment at y=0 and should continue through its endpoint.
  const auto seed = processor.buildSeed(
    path, pose(1.8, 0.2), context,
    [](const Eigen::Vector3d &, double) {return true;});

  ASSERT_TRUE(seed.valid);
  ASSERT_GE(seed.dense_path.size(), 2U);
  EXPECT_NEAR(seed.dense_path.front().x(), 1.8, 1e-9);
  EXPECT_NEAR(seed.dense_path.front().y(), 0.2, 1e-9);
  EXPECT_NEAR(seed.dense_path[1].x(), 3.5, 1e-9);
  EXPECT_NEAR(seed.dense_path[1].y(), 0.0, 1e-9);
}

}  // namespace
}  // namespace minco_planner
