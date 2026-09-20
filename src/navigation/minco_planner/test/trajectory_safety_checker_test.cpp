#include <gtest/gtest.h>

#include "minco_core/components/trajectory_safety_checker.hpp"
#include "minco_core/minco_utils.hpp"
#include "rog_map/projection_layer.hpp"

#include <functional>
#include <utility>

namespace minco_planner {
namespace {

double systemNowSeconds() {
  return rclcpp::Clock(RCL_SYSTEM_TIME).now().seconds();
}

class FakeMapQuery : public rog_map::MapQueryInterface {
public:
  using ObstaclePredicate = std::function<bool(const Eigen::Vector3d &)>;

  explicit FakeMapQuery(ObstaclePredicate obstacle, double resolution = 0.02,
                        double snapshot_stamp = systemNowSeconds())
      : obstacle_(std::move(obstacle)), resolution_(resolution),
        snapshot_stamp_(snapshot_stamp) {}

  bool worldToMap(double wx, double wy, unsigned int &mx,
                  unsigned int &my) const override {
    if (!std::isfinite(wx) || !std::isfinite(wy) || wx < -10.0 || wx >= 10.0 ||
        wy < -10.0 || wy >= 10.0) {
      return false;
    }
    mx = static_cast<unsigned int>((wx + 10.0) / resolution_);
    my = static_cast<unsigned int>((wy + 10.0) / resolution_);
    return true;
  }

  void mapToWorld(unsigned int mx, unsigned int my, double &wx,
                  double &wy) const override {
    wx = -10.0 + (static_cast<double>(mx) + 0.5) * resolution_;
    wy = -10.0 + (static_cast<double>(my) + 0.5) * resolution_;
  }

  unsigned int sizeX() const override {
    return static_cast<unsigned int>(20.0 / resolution_);
  }
  unsigned int sizeY() const override {
    return static_cast<unsigned int>(20.0 / resolution_);
  }
  double resolution() const override { return resolution_; }
  double originX() const override { return -10.0; }
  double originY() const override { return -10.0; }
  uint8_t value(unsigned int, unsigned int) const override {
    return nav2_costmap_2d::FREE_SPACE;
  }
  const unsigned char *values() const override { return nullptr; }
  bool isValid(unsigned int mx, unsigned int my) const override {
    return mx < sizeX() && my < sizeY();
  }
  bool isFree(unsigned int mx, unsigned int my) const override {
    return isValid(mx, my);
  }

  rog_map::QueryResult query(const Eigen::Vector3d &pos) const override {
    rog_map::QueryResult result;
    if (!pos.allFinite()) {
      result.status = rog_map::QueryStatus::NONFINITE_INPUT;
      return result;
    }
    result.ok = true;
    result.status = rog_map::QueryStatus::OK;
    result.distance = obstacle_(pos) ? 0.0 : 10.0;
    result.snapshot_stamp = snapshot_stamp_;
    return result;
  }

  bool evaluate(const Eigen::Vector3d &pos, double &dist,
                Eigen::Vector3d &grad) const override {
    const auto result = query(pos);
    dist = result.distance;
    grad = result.gradient;
    return result.ok;
  }

private:
  ObstaclePredicate obstacle_;
  double resolution_;
  double snapshot_stamp_;
};

class FailingMapQuery : public FakeMapQuery {
public:
  explicit FailingMapQuery(rog_map::QueryStatus status)
      : FakeMapQuery([](const Eigen::Vector3d &) { return false; }),
        status_(status) {}

  rog_map::QueryResult query(const Eigen::Vector3d &) const override {
    rog_map::QueryResult result;
    result.status = status_;
    return result;
  }

private:
  rog_map::QueryStatus status_;
};

class NegativeInterpolatedDistanceQuery : public FakeMapQuery {
public:
  NegativeInterpolatedDistanceQuery()
      : FakeMapQuery([](const Eigen::Vector3d &) { return false; }) {}

  rog_map::QueryResult query(const Eigen::Vector3d &pos) const override {
    auto result = FakeMapQuery::query(pos);
    if (result.ok) {
      result.distance = -0.033;
    }
    return result;
  }
};

class LethalMetadataQuery : public FakeMapQuery {
public:
  LethalMetadataQuery()
      : FakeMapQuery([](const Eigen::Vector3d &) { return false; }) {}

  rog_map::QueryResult query(const Eigen::Vector3d &pos) const override {
    auto result = FakeMapQuery::query(pos);
    result.projected_cost_valid = true;
    result.projected_cost = nav2_costmap_2d::LETHAL_OBSTACLE;
    result.snapshot_commit_stamp = result.snapshot_stamp;
    result.snapshot_processing_age_ms = 12.0;
    result.projection.valid = true;
    result.projection.cost_source =
        rog_map::ProjectedCostSource::DYNAMIC_PROJECTION;
    result.projection.dynamic_cost = nav2_costmap_2d::LETHAL_OBSTACLE;
    result.projection.cell_type =
        static_cast<uint8_t>(rog_map::CellType::OCCUPIED);
    result.projection.raw_reason = static_cast<uint8_t>(
        rog_map::ProjectionClassReason::HEADROOM_BLOCKED);
    result.projection.candidate_reason = result.projection.raw_reason;
    result.projection.headroom = 0.18F;
    return result;
  }
};

traj_opt::Trajectory makeLinearTrajectory(double duration,
                                          const Eigen::Vector3d &start,
                                          const Eigen::Vector3d &velocity) {
  Eigen::MatrixXd coefficients(3, 6);
  coefficients.setZero();
  coefficients.col(4) = velocity;
  coefficients.col(5) = start;
  traj_opt::Trajectory trajectory;
  trajectory.emplace_back(duration, coefficients);
  return trajectory;
}

traj_opt::Trajectory makeYawTrajectory(double duration, double yaw) {
  Eigen::MatrixXd coefficients(3, 6);
  coefficients.setZero();
  coefficients(0, 5) = yaw;
  traj_opt::Trajectory trajectory;
  trajectory.emplace_back(duration, coefficients);
  return trajectory;
}

std::unique_ptr<TrajectorySafetyChecker>
makeChecker(const TrajectorySafetyChecker::Config &config,
            const std::shared_ptr<rog_map::MapQueryInterface> &query) {
  auto checker = std::make_unique<TrajectorySafetyChecker>();
  checker->configure(config,
                     rclcpp::get_logger("trajectory_safety_checker_test"),
                     std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME));
  checker->setQuery(query);
  return checker;
}

TEST(TrajectorySafetyCheckerTest, RejectsObstacleAtMandatoryEndpoint) {
  TrajectorySafetyChecker::Config config;
  config.safe_dist = 0.01;
  config.footprint_length = 0.10;
  config.footprint_width = 0.10;
  config.footprint_margin = 0.001;
  config.sample_dt = 0.60;
  auto map = std::make_shared<FakeMapQuery>([](const Eigen::Vector3d &pos) {
    return std::abs(pos.x() - 1.0) < 0.005 && std::abs(pos.y()) < 0.005;
  });
  auto checker = makeChecker(config, map);

  const auto position = makeLinearTrajectory(1.0, Eigen::Vector3d::Zero(),
                                             Eigen::Vector3d::UnitX());
  const auto yaw = makeYawTrajectory(1.0, 0.0);
  EXPECT_FALSE(checker->checkTrajectory(position, yaw));

  const auto diagnostic = checker->lastFailureDiagnostic();
  EXPECT_EQ(diagnostic.reason,
            TrajectorySafetyChecker::FailureReason::INSUFFICIENT_CLEARANCE);
  EXPECT_DOUBLE_EQ(diagnostic.trajectory_time, 1.0);
  EXPECT_TRUE(diagnostic.footprint_sample);
  EXPECT_NEAR(diagnostic.center.x(), 1.0, 1e-12);
  EXPECT_NEAR(diagnostic.query_point.x(), 1.0, 1e-12);
  EXPECT_NEAR(diagnostic.footprint_offset.norm(), 0.0, 1e-12);
}

TEST(TrajectorySafetyCheckerTest, PreservesLethalProjectionEvidence) {
  TrajectorySafetyChecker::Config config;
  config.safe_dist = 0.0;
  config.footprint_length = 0.10;
  config.footprint_width = 0.10;
  config.footprint_margin = 0.001;
  auto checker = makeChecker(config, std::make_shared<LethalMetadataQuery>());

  ASSERT_FALSE(checker->checkTrajectory(makeLinearTrajectory(
      0.2, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero())));
  const auto diagnostic = checker->lastFailureDiagnostic();
  EXPECT_EQ(diagnostic.reason,
            TrajectorySafetyChecker::FailureReason::COSTMAP_LETHAL);
  ASSERT_TRUE(diagnostic.projection.valid);
  EXPECT_EQ(diagnostic.projection.cost_source,
            rog_map::ProjectedCostSource::DYNAMIC_PROJECTION);
  EXPECT_EQ(diagnostic.projection.raw_reason,
            static_cast<uint8_t>(
              rog_map::ProjectionClassReason::HEADROOM_BLOCKED));
  EXPECT_FLOAT_EQ(diagnostic.projection.headroom, 0.18F);
  EXPECT_DOUBLE_EQ(diagnostic.snapshot_processing_age_ms, 12.0);
}

TEST(TrajectorySafetyCheckerTest, ReportsFirstCenterlineCollisionSample) {
  TrajectorySafetyChecker::Config config;
  config.safe_dist = 0.01;
  config.sample_dt = 0.25;
  config.planning_frame = "map";
  config.rog_frame = "camera_init";
  auto map = std::make_shared<FakeMapQuery>([](const Eigen::Vector3d &pos) {
    return pos.x() >= 0.49;
  });
  auto checker = makeChecker(config, map);

  const auto collision_trajectory = makeLinearTrajectory(
      1.0, Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitX());
  ASSERT_FALSE(checker->checkTrajectory(collision_trajectory));

  const auto diagnostic = checker->lastFailureDiagnostic();
  EXPECT_EQ(diagnostic.reason,
            TrajectorySafetyChecker::FailureReason::INSUFFICIENT_CLEARANCE);
  EXPECT_DOUBLE_EQ(diagnostic.trajectory_time, 0.5);
  EXPECT_DOUBLE_EQ(diagnostic.trajectory_duration, 1.0);
  EXPECT_FALSE(diagnostic.footprint_sample);
  EXPECT_TRUE(diagnostic.query_attempted);
  EXPECT_EQ(diagnostic.query_status, rog_map::QueryStatus::OK);
  EXPECT_DOUBLE_EQ(diagnostic.center.x(), 0.5);
  EXPECT_DOUBLE_EQ(diagnostic.query_point.x(), 0.5);
  EXPECT_DOUBLE_EQ(diagnostic.distance, 0.0);
  EXPECT_DOUBLE_EQ(diagnostic.safe_distance, 0.01);
  EXPECT_EQ(diagnostic.planning_frame, "map");
  EXPECT_EQ(diagnostic.rog_frame, "camera_init");

  const auto safe_trajectory = makeLinearTrajectory(
      0.5, Eigen::Vector3d(-1.0, 0.0, 0.0), Eigen::Vector3d::Zero());
  ASSERT_TRUE(checker->checkTrajectory(safe_trajectory));
  EXPECT_EQ(checker->lastFailureDiagnostic().reason,
            TrajectorySafetyChecker::FailureReason::NONE);
}

TEST(TrajectorySafetyCheckerTest,
     RemainingTrajectoryDoesNotRecheckAlreadyExecutedSamples) {
  TrajectorySafetyChecker::Config config;
  config.safe_dist = 0.01;
  config.footprint_length = 0.02;
  config.footprint_width = 0.02;
  config.footprint_margin = 0.001;
  config.sample_dt = 0.10;
  auto map = std::make_shared<FakeMapQuery>([](const Eigen::Vector3d &pos) {
    return pos.x() < 0.25;
  });
  auto checker = makeChecker(config, map);
  const auto position = makeLinearTrajectory(
      1.0, Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitX());
  const auto yaw = makeYawTrajectory(1.0, 0.0);

  // Newly generated candidates retain the original full validation contract.
  EXPECT_FALSE(checker->checkTrajectory(position, yaw));
  EXPECT_TRUE(checker->checkTrajectoryFromTime(position, 0.5));
  EXPECT_TRUE(checker->checkTrajectoryFromTime(position, yaw, 0.5));

  double start_time = std::numeric_limits<double>::quiet_NaN();
  ASSERT_TRUE(checker->computeSpatialCheckStartTime(
      position, Eigen::Vector3d(0.5, 0.0, 0.0), start_time));
  EXPECT_NEAR(start_time, 0.5, 1e-9);
  EXPECT_TRUE(checker->checkTrajectoryFromTime(position, yaw, start_time));
}

TEST(TrajectorySafetyCheckerTest,
     SpatialProgressCoversWhatNearestPointMpcCanStillExecute) {
  TrajectorySafetyChecker::Config config;
  config.safe_dist = 0.01;
  config.footprint_length = 0.02;
  config.footprint_width = 0.02;
  config.footprint_margin = 0.001;
  config.sample_dt = 0.10;
  auto map = std::make_shared<FakeMapQuery>([](const Eigen::Vector3d &pos) {
    return pos.x() >= 0.39 && pos.x() <= 0.41;
  });
  auto checker = makeChecker(config, map);
  const auto position = makeLinearTrajectory(
      1.0, Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitX());
  const auto yaw = makeYawTrajectory(1.0, 0.0);

  double start_time = std::numeric_limits<double>::quiet_NaN();
  ASSERT_TRUE(checker->computeSpatialCheckStartTime(
      position, Eigen::Vector3d(0.2, 0.0, 0.0), start_time));
  EXPECT_NEAR(start_time, 0.2, 1e-9);

  // A wall-clock-only start misses this obstacle. Spatial indexing retains it
  // because it is still ahead of the controller's nearest-point pickup.
  EXPECT_TRUE(checker->checkTrajectoryFromTime(position, yaw, 0.8));
  EXPECT_FALSE(checker->checkTrajectoryFromTime(position, yaw, start_time));
}

TEST(TrajectorySafetyCheckerTest, SpatialProgressMatchesForwardProjection) {
  TrajectorySafetyChecker::Config config;
  config.sample_dt = 0.10;
  auto map = std::make_shared<FakeMapQuery>(
      [](const Eigen::Vector3d &) { return false; });
  auto checker = makeChecker(config, map);
  const auto position = makeLinearTrajectory(
      1.0, Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitX());
  double start_time = std::numeric_limits<double>::quiet_NaN();

  ASSERT_TRUE(checker->computeSpatialCheckStartTime(
      position, Eigen::Vector3d(0.26, 0.0, 0.0), start_time));
  EXPECT_NEAR(start_time, 0.26, 1e-9);

  ASSERT_TRUE(checker->computeSpatialCheckStartTime(
      position, Eigen::Vector3d(1.0, 0.0, 0.0), start_time));
  EXPECT_NEAR(start_time, 1.0, 1e-9);
}

TEST(TrajectorySafetyCheckerTest, InvalidProgressInputFailsClosed) {
  auto map = std::make_shared<FakeMapQuery>(
      [](const Eigen::Vector3d &) { return false; });
  auto checker = makeChecker(TrajectorySafetyChecker::Config{}, map);
  const auto position = makeLinearTrajectory(
      1.0, Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitX());
  double start_time = 0.0;

  EXPECT_FALSE(checker->computeSpatialCheckStartTime(
      position,
      Eigen::Vector3d(std::numeric_limits<double>::infinity(), 0.0, 0.0),
      start_time));
  EXPECT_TRUE(std::isnan(start_time));
}

TEST(TrajectorySafetyCheckerTest, SweptJoinRejectsObstacleBetweenRobotAndPickup) {
  TrajectorySafetyChecker::Config config;
  config.safe_dist = 0.01;
  config.footprint_length = 0.02;
  config.footprint_width = 0.02;
  config.footprint_margin = 0.001;
  auto map = std::make_shared<FakeMapQuery>([](const Eigen::Vector3d &pos) {
    return pos.x() >= 0.24 && pos.x() <= 0.26;
  });
  auto checker = makeChecker(config, map);

  EXPECT_FALSE(checker->checkSweptFootprint(
      Eigen::Vector3d::Zero(), 0.0, Eigen::Vector3d(0.5, 0.0, 0.0), 0.0));
  EXPECT_EQ(checker->lastFailureDiagnostic().reason,
            TrajectorySafetyChecker::FailureReason::INSUFFICIENT_CLEARANCE);
}

TEST(TrajectorySafetyCheckerTest, SweptJoinChecksInPlaceRotation) {
  TrajectorySafetyChecker::Config config;
  config.safe_dist = 0.01;
  config.footprint_length = 0.40;
  config.footprint_width = 0.10;
  config.footprint_margin = 0.001;
  auto map = std::make_shared<FakeMapQuery>([](const Eigen::Vector3d &pos) {
    return pos.x() > 0.13 && pos.x() < 0.17 && pos.y() > 0.13 && pos.y() < 0.17;
  });
  auto checker = makeChecker(config, map);

  EXPECT_FALSE(checker->checkSweptFootprint(
      Eigen::Vector3d::Zero(), 0.0, Eigen::Vector3d::Zero(), 1.5707963267948966));
}

TEST(TrajectorySafetyCheckerTest, RemainingTrajectoryStillChecksFutureSamples) {
  TrajectorySafetyChecker::Config config;
  config.safe_dist = 0.01;
  config.footprint_length = 0.02;
  config.footprint_width = 0.02;
  config.footprint_margin = 0.001;
  config.sample_dt = 0.10;
  auto map = std::make_shared<FakeMapQuery>([](const Eigen::Vector3d &pos) {
    return pos.x() >= 0.69;
  });
  auto checker = makeChecker(config, map);
  const auto position = makeLinearTrajectory(
      1.0, Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitX());
  const auto yaw = makeYawTrajectory(1.0, 0.0);

  ASSERT_FALSE(checker->checkTrajectoryFromTime(position, yaw, 0.5));
  const auto diagnostic = checker->lastFailureDiagnostic();
  EXPECT_EQ(diagnostic.reason,
            TrajectorySafetyChecker::FailureReason::INSUFFICIENT_CLEARANCE);
  EXPECT_GE(diagnostic.trajectory_time, 0.5);
  EXPECT_LE(diagnostic.trajectory_time, 1.0);
}

TEST(TrajectorySafetyCheckerTest, StartPastDurationStillChecksEndpoint) {
  TrajectorySafetyChecker::Config config;
  config.safe_dist = 0.01;
  config.footprint_length = 0.02;
  config.footprint_width = 0.02;
  config.footprint_margin = 0.001;
  auto map = std::make_shared<FakeMapQuery>([](const Eigen::Vector3d &pos) {
    return pos.x() >= 0.98;
  });
  auto checker = makeChecker(config, map);
  const auto position = makeLinearTrajectory(
      1.0, Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitX());
  const auto yaw = makeYawTrajectory(1.0, 0.0);

  ASSERT_FALSE(checker->checkTrajectoryFromTime(position, yaw, 5.0));
  EXPECT_DOUBLE_EQ(checker->lastFailureDiagnostic().trajectory_time, 1.0);
}

TEST(TrajectorySafetyCheckerTest, BoundedEndTimeDoesNotQueryBeyondSafetyWindow) {
  TrajectorySafetyChecker::Config config;
  config.safe_dist = 0.01;
  config.footprint_length = 0.02;
  config.footprint_width = 0.02;
  config.footprint_margin = 0.001;
  config.sample_dt = 0.10;
  auto map = std::make_shared<FakeMapQuery>([](const Eigen::Vector3d &pos) {
    return pos.x() >= 0.80;
  });
  auto checker = makeChecker(config, map);
  const auto position = makeLinearTrajectory(
      2.0, Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitX());
  const auto yaw = makeYawTrajectory(2.0, 0.0);

  ASSERT_TRUE(checker->checkTrajectoryFromTime(position, yaw, 0.0, 0.50));
  ASSERT_FALSE(checker->checkTrajectoryFromTime(position, yaw, 0.0, 1.0));
  EXPECT_GE(checker->lastFailureDiagnostic().trajectory_time, 0.79);
}

TEST(TrajectorySafetyCheckerTest, NonfiniteStartTimeFailsClosed) {
  auto map = std::make_shared<FakeMapQuery>(
      [](const Eigen::Vector3d &) { return false; });
  auto checker = makeChecker(TrajectorySafetyChecker::Config{}, map);
  const auto position = makeLinearTrajectory(
      1.0, Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitX());
  const auto yaw = makeYawTrajectory(1.0, 0.0);

  ASSERT_FALSE(checker->checkTrajectoryFromTime(
      position, yaw, std::numeric_limits<double>::quiet_NaN()));
  EXPECT_EQ(checker->lastFailureDiagnostic().reason,
            TrajectorySafetyChecker::FailureReason::INVALID_TRAJECTORY_START_TIME);
}

TEST(TrajectorySafetyCheckerTest, AppliesYawToRectangularFootprint) {
  TrajectorySafetyChecker::Config config;
  config.safe_dist = 0.01;
  config.footprint_length = 1.0;
  config.footprint_width = 0.10;
  config.footprint_margin = 0.001;
  config.sample_dt = 0.10;
  auto map = std::make_shared<FakeMapQuery>([](const Eigen::Vector3d &pos) {
    return std::abs(pos.x()) < 0.005 && std::abs(pos.y() - 0.5) < 0.005;
  });
  auto checker = makeChecker(config, map);
  const auto position = makeLinearTrajectory(0.2, Eigen::Vector3d::Zero(),
                                             Eigen::Vector3d::Zero());

  EXPECT_TRUE(checker->checkTrajectory(position, makeYawTrajectory(0.2, 0.0)));
  EXPECT_FALSE(checker->checkTrajectory(
      position, makeYawTrajectory(0.2, 1.5707963267948966)));

  const auto diagnostic = checker->lastFailureDiagnostic();
  EXPECT_EQ(diagnostic.reason,
            TrajectorySafetyChecker::FailureReason::INSUFFICIENT_CLEARANCE);
  EXPECT_TRUE(diagnostic.footprint_sample);
  EXPECT_NEAR(diagnostic.center.x(), 0.0, 1e-12);
  EXPECT_NEAR(diagnostic.center.y(), 0.0, 1e-12);
  EXPECT_NEAR(diagnostic.query_point.x(), 0.0, 0.005);
  EXPECT_NEAR(diagnostic.query_point.y(), 0.5, 0.005);
  EXPECT_NEAR(diagnostic.footprint_offset.x(), 0.5, 0.005);
  EXPECT_NEAR(diagnostic.footprint_offset.y(), 0.0, 0.005);
}

TEST(TrajectorySafetyCheckerTest, ReportsRogQueryFailureStatus) {
  TrajectorySafetyChecker::Config config;
  config.safe_dist = 0.0;
  config.sample_dt = 0.10;
  config.planning_frame = "map";
  config.rog_frame = "camera_init";
  auto checker = makeChecker(
      config, std::make_shared<FailingMapQuery>(rog_map::QueryStatus::TF_FAILED));

  const auto position = makeLinearTrajectory(
      0.2, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
  ASSERT_FALSE(checker->checkTrajectory(position));

  const auto diagnostic = checker->lastFailureDiagnostic();
  EXPECT_EQ(diagnostic.reason,
            TrajectorySafetyChecker::FailureReason::QUERY_FAILED);
  EXPECT_EQ(diagnostic.query_status, rog_map::QueryStatus::TF_FAILED);
  EXPECT_TRUE(diagnostic.query_attempted);
  EXPECT_DOUBLE_EQ(diagnostic.trajectory_time, 0.0);
  EXPECT_TRUE(std::isnan(diagnostic.distance));
  EXPECT_EQ(diagnostic.planning_frame, "map");
  EXPECT_EQ(diagnostic.rog_frame, "camera_init");
}

TEST(TrajectorySafetyCheckerTest,
     ZeroClearanceUsesExactFreeCellAfterValidFreshQuery) {
  TrajectorySafetyChecker::Config config;
  config.safe_dist = 0.0;
  auto map = std::make_shared<NegativeInterpolatedDistanceQuery>();
  auto checker = makeChecker(config, map);

  EXPECT_TRUE(checker->checkPoint(Eigen::Vector3d::Zero()));

  config.safe_dist = 0.001;
  checker = makeChecker(config, map);
  EXPECT_FALSE(checker->checkPoint(Eigen::Vector3d::Zero()));
}

TEST(TrajectorySafetyCheckerTest, ExpandsMarginOnEachSide) {
  TrajectorySafetyChecker::Config config;
  config.safe_dist = 0.01;
  config.footprint_length = 0.20;
  config.footprint_width = 0.10;
  config.footprint_margin = 0.001;
  config.sample_dt = 0.10;
  auto map = std::make_shared<FakeMapQuery>([](const Eigen::Vector3d &pos) {
    return std::abs(pos.x() - 0.2) < 0.005 && std::abs(pos.y()) < 0.005;
  });
  const auto position = makeLinearTrajectory(0.2, Eigen::Vector3d::Zero(),
                                             Eigen::Vector3d::Zero());
  const auto yaw = makeYawTrajectory(0.2, 0.0);

  EXPECT_TRUE(makeChecker(config, map)->checkTrajectory(position, yaw));
  config.footprint_margin = 0.10;
  EXPECT_FALSE(makeChecker(config, map)->checkTrajectory(position, yaw));
}

TEST(TrajectorySafetyCheckerTest,
     RejectsYawTrajectoryShorterThanPositionTrajectory) {
  TrajectorySafetyChecker::Config config;
  config.safe_dist = 0.01;
  config.footprint_margin = 0.001;
  auto map = std::make_shared<FakeMapQuery>(
      [](const Eigen::Vector3d &) { return false; });
  auto checker = makeChecker(config, map);

  const auto position = makeLinearTrajectory(1.0, Eigen::Vector3d::Zero(),
                                             Eigen::Vector3d::UnitX());
  EXPECT_FALSE(checker->checkTrajectory(position, makeYawTrajectory(0.5, 0.0)));
}

TEST(TrajectorySafetyCheckerTest, RejectsInvalidConfiguration) {
  TrajectorySafetyChecker checker;
  TrajectorySafetyChecker::Config config;
  config.footprint_length = 0.0;
  EXPECT_THROW(checker.configure(
                   config, rclcpp::get_logger("trajectory_safety_checker_test"),
                   std::make_shared<rclcpp::Clock>(RCL_ROS_TIME)),
               std::invalid_argument);
}

TEST(TrajectorySafetyCheckerTest, RejectsStaleMapSnapshot) {
  TrajectorySafetyChecker::Config config;
  config.map_timeout = 0.10;
  auto map = std::make_shared<FakeMapQuery>(
      [](const Eigen::Vector3d &) { return false; }, 0.02,
      systemNowSeconds() - 1.0);

  EXPECT_FALSE(makeChecker(config, map)->checkPoint(Eigen::Vector3d::Zero()));
}

TEST(TrajectorySafetyCheckerTest, RejectsFutureMapSnapshot) {
  TrajectorySafetyChecker::Config config;
  config.future_tolerance = 0.05;
  auto map = std::make_shared<FakeMapQuery>(
      [](const Eigen::Vector3d &) { return false; }, 0.02,
      systemNowSeconds() + 1.0);

  EXPECT_FALSE(makeChecker(config, map)->checkPoint(Eigen::Vector3d::Zero()));
}

TEST(TrajectorySafetyCheckerTest, RejectsMissingMapSnapshotTimestamp) {
  auto map = std::make_shared<FakeMapQuery>(
      [](const Eigen::Vector3d &) { return false; }, 0.02,
      std::numeric_limits<double>::quiet_NaN());

  EXPECT_FALSE(makeChecker(TrajectorySafetyChecker::Config{}, map)
                   ->checkPoint(Eigen::Vector3d::Zero()));
}

TEST(TrajectorySafetyCheckerTest, MissingQueryHasNoUsableDistance) {
  TrajectorySafetyChecker checker;
  checker.configure(TrajectorySafetyChecker::Config{},
                    rclcpp::get_logger("trajectory_safety_checker_test"),
                    std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME));

  EXPECT_TRUE(std::isnan(checker.getDistance(Eigen::Vector3d::Zero())));
}

TEST(TrajectorySafetyCheckerTest, FrameAwareQueryPreservesSnapshotTimestamp) {
  const double stamp = systemNowSeconds();
  auto raw = std::make_shared<FakeMapQuery>(
      [](const Eigen::Vector3d &) { return false; }, 0.02, stamp);
  auto clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
  FrameAwareRogQuery framed(raw, nullptr, "map", "map",
                            rclcpp::get_logger("frame_query_test"), clock);

  const auto result = framed.query(Eigen::Vector3d::Zero());
  ASSERT_TRUE(result.ok);
  EXPECT_DOUBLE_EQ(result.snapshot_stamp, stamp);
}

TEST(TrajectorySafetyCheckerTest,
     EscapeCandidateChecksPredictedEndpointWithYawedFootprint) {
  TrajectorySafetyChecker::Config config;
  config.safe_dist = 0.01;
  config.footprint_length = 1.0;
  config.footprint_width = 0.10;
  config.footprint_margin = 0.001;
  config.sample_dt = 0.10;
  auto map = std::make_shared<FakeMapQuery>([](const Eigen::Vector3d &pos) {
    return std::abs(pos.x() - 0.251) < 0.006 &&
           std::abs(pos.y() - 0.501) < 0.006;
  });
  auto checker = makeChecker(config, map);

  geometry_msgs::msg::PoseStamped pose;
  pose.pose.position.x = 0.0;
  pose.pose.position.y = 0.0;
  traj_opt::Trajectory position;
  traj_opt::Trajectory yaw;
  ASSERT_TRUE(utils::makeEscapeTrajectories(
      pose, Eigen::Vector2d(0.4, 0.0), 0.0, 0.5, position, yaw));
  EXPECT_TRUE(checker->checkTrajectory(position, yaw));

  ASSERT_TRUE(utils::makeEscapeTrajectories(
      pose, Eigen::Vector2d(0.4, 0.0), 1.5707963267948966, 0.5,
      position, yaw));
  ASSERT_FALSE(checker->checkTrajectory(position, yaw));
  const auto diagnostic = checker->lastFailureDiagnostic();
  EXPECT_EQ(diagnostic.reason,
            TrajectorySafetyChecker::FailureReason::INSUFFICIENT_CLEARANCE);
  EXPECT_DOUBLE_EQ(diagnostic.trajectory_time, 0.5);
  EXPECT_TRUE(diagnostic.footprint_sample);
  EXPECT_NEAR(diagnostic.center.x(), 0.2, 1e-12);
}

TEST(TrajectorySafetyCheckerTest, EscapeCandidateChecksCurrentFootprint) {
  TrajectorySafetyChecker::Config config;
  config.safe_dist = 0.01;
  config.footprint_length = 0.30;
  config.footprint_width = 0.20;
  config.footprint_margin = 0.001;
  config.sample_dt = 0.10;
  auto map = std::make_shared<FakeMapQuery>([](const Eigen::Vector3d &pos) {
    return pos.x() < -0.145;
  });
  auto checker = makeChecker(config, map);

  geometry_msgs::msg::PoseStamped pose;
  traj_opt::Trajectory position;
  traj_opt::Trajectory yaw;
  ASSERT_TRUE(utils::makeEscapeTrajectories(
      pose, Eigen::Vector2d(0.4, 0.0), 0.0, 0.5, position, yaw));
  ASSERT_FALSE(checker->checkTrajectory(position, yaw));
  const auto diagnostic = checker->lastFailureDiagnostic();
  EXPECT_DOUBLE_EQ(diagnostic.trajectory_time, 0.0);
  EXPECT_TRUE(diagnostic.footprint_sample);
  EXPECT_NEAR(diagnostic.center.x(), 0.0, 1e-12);
}

TEST(TrajectorySafetyCheckerTest, EscapeCandidateRejectsStaleRogSnapshot) {
  TrajectorySafetyChecker::Config config;
  config.map_timeout = 0.10;
  config.sample_dt = 0.10;
  auto map = std::make_shared<FakeMapQuery>(
      [](const Eigen::Vector3d &) { return false; }, 0.02,
      systemNowSeconds() - 1.0);
  auto checker = makeChecker(config, map);

  geometry_msgs::msg::PoseStamped pose;
  traj_opt::Trajectory position;
  traj_opt::Trajectory yaw;
  ASSERT_TRUE(utils::makeEscapeTrajectories(
      pose, Eigen::Vector2d(0.4, 0.0), 0.0, 0.5, position, yaw));
  ASSERT_FALSE(checker->checkTrajectory(position, yaw));
  EXPECT_EQ(checker->lastFailureDiagnostic().reason,
            TrajectorySafetyChecker::FailureReason::STALE_SNAPSHOT);
}

TEST(TrajectorySafetyCheckerTest, EscapeBuilderRejectsNonfiniteMotion) {
  geometry_msgs::msg::PoseStamped pose;
  traj_opt::Trajectory position;
  traj_opt::Trajectory yaw;

  EXPECT_FALSE(utils::makeEscapeTrajectories(
      pose,
      Eigen::Vector2d(std::numeric_limits<double>::quiet_NaN(), 0.0),
      0.0, 0.5, position, yaw));
  EXPECT_FALSE(utils::makeEscapeTrajectories(
      pose, Eigen::Vector2d(0.4, 0.0),
      std::numeric_limits<double>::quiet_NaN(), 0.5, position, yaw));
  pose.pose.position.x = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(utils::makeEscapeTrajectories(
      pose, Eigen::Vector2d(0.4, 0.0), 0.0, 0.5, position, yaw));
}

TEST(TrajectorySafetyCheckerTest, BackupCommandPreservesOneTokenAndTrajectoryId) {
  const auto backup = makeLinearTrajectory(
      0.4, Eigen::Vector3d(1.0, 2.0, 0.0), Eigen::Vector3d::Zero());
  std_msgs::msg::Header header;
  header.frame_id = "map";
  header.stamp.sec = 12;
  header.stamp.nanosec = 34U;
  builtin_interfaces::msg::Time planning_stamp;
  planning_stamp.sec = 9;
  planning_stamp.nanosec = 87U;

  const auto command = utils::makeBackupTrajectoryCommand(
      backup, 77U, header, planning_stamp, 3, 0.1, 0.25);
  EXPECT_EQ(command.command_flag,
            ros_interfaces::msg::MpcPositionCommand::BLOCK_COMMAND);
  EXPECT_EQ(command.header.frame_id, "map");
  EXPECT_EQ(command.header.stamp.sec, 12);
  EXPECT_EQ(command.header.stamp.nanosec, 34U);
  EXPECT_EQ(command.planning_stamp.sec, 9);
  EXPECT_EQ(command.planning_stamp.nanosec, 87U);
  ASSERT_EQ(command.mpc_horizon, 3U);
  ASSERT_EQ(command.cmds.size(), 3U);
  for (const auto &cmd : command.cmds) {
    EXPECT_EQ(cmd.trajectory_id, 77U);
    EXPECT_EQ(cmd.header.stamp.sec, command.header.stamp.sec);
    EXPECT_EQ(cmd.header.stamp.nanosec, command.header.stamp.nanosec);
  }
}

TEST(TrajectorySafetyCheckerTest, CheckedYawRejectsInvalidQuaternion) {
  geometry_msgs::msg::Quaternion invalid;
  invalid.x = 0.0;
  invalid.y = 0.0;
  invalid.z = 0.0;
  invalid.w = 0.0;
  double yaw = 0.0;
  EXPECT_FALSE(utils::quaternionToYawChecked(invalid, yaw));
  EXPECT_TRUE(std::isnan(yaw));

  geometry_msgs::msg::Quaternion valid;
  valid.z = std::sin(0.25);
  valid.w = std::cos(0.25);
  ASSERT_TRUE(utils::quaternionToYawChecked(valid, yaw));
  EXPECT_NEAR(yaw, 0.5, 1e-12);
}

} // namespace
} // namespace minco_planner
