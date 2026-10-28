#include <gtest/gtest.h>

#include <pcl/point_types.h>

#include "adaptive_lio.hpp"
#include "common_lib.h"
#include "ivox/ivox3d.h"

namespace
{
using EigenPointVector =
  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>;

TEST(AdaptiveLio, FitsPlanarSurface)
{
  EigenPointVector points;
  for (int x = -2; x <= 2; ++x) {
    for (int y = -2; y <= 2; ++y) {
      points.emplace_back(0.1 * x, 0.1 * y, 0.001 * ((x + y) % 2));
    }
  }

  point_lio::PlaneModel model;
  ASSERT_TRUE(point_lio::fitPlane(points, points.size(), 0.03, 0.02, model));
  EXPECT_LT(model.curvature, 0.001);
  EXPECT_GT(std::abs(model.normal.z()), 0.99);
  EXPECT_GT(model.quality, 0.5);
}

TEST(AdaptiveLio, RejectsLinearGeometry)
{
  EigenPointVector points;
  for (int i = 0; i < 20; ++i) {
    points.emplace_back(0.1 * i, 0.0, 0.0);
  }

  point_lio::PlaneModel model;
  EXPECT_FALSE(point_lio::fitPlane(points, points.size(), 0.03, 0.02, model));
}

TEST(AdaptiveLio, ResponsibilitiesPreferConsistentScale)
{
  point_lio::PlaneModelVector models(3);
  models[0].residual = 0.01;
  models[1].residual = 0.10;
  models[2].residual = 0.20;
  for (auto & model : models) {
    model.innovation_variance = 0.01;
    model.quality = 1.0;
  }

  point_lio::computeResponsibilities(models, 0.05, 3.0, 5.0, 0.0);
  const double responsibility_sum =
    models[0].responsibility + models[1].responsibility + models[2].responsibility;
  EXPECT_LE(responsibility_sum, 1.0);
  EXPECT_GT(models[0].responsibility, models[1].responsibility);
  EXPECT_GT(models[1].responsibility, models[2].responsibility);
}

TEST(AdaptiveLio, RejectsAllScalesOutsideGate)
{
  point_lio::PlaneModelVector models(3);
  for (auto & model : models) {
    model.residual = 1.0;
    model.innovation_variance = 0.01;
    model.quality = 1.0;
  }
  point_lio::computeResponsibilities(models, 0.05, 3.0, 5.0, 0.05);
  for (const auto & model : models) {
    EXPECT_DOUBLE_EQ(model.responsibility, 0.0);
  }
}

TEST(AdaptiveLio, EmptyPointCloudHasNoTimeGroups)
{
  PointCloudXYZI::Ptr cloud(new PointCloudXYZI());
  EXPECT_TRUE(time_compressing<int>(cloud).empty());
}

TEST(AdaptiveLio, TemporalSchedulerHonorsInformationAndHardLimits)
{
  point_lio::TemporalAdaptiveConfig config;
  config.enabled = true;
  config.min_points = 8;
  config.initial_target_points = 32;
  config.max_points = 128;
  config.min_duration = 0.001;
  config.max_duration = 0.010;
  config.max_translation = 0.02;
  config.max_pose_covariance_trace = 10.0;
  config.latency_penalty = 0.02;
  config.information_ema_alpha = 1.0;
  point_lio::TemporalUpdateScheduler scheduler(config);

  const Eigen::Vector3d origin = Eigen::Vector3d::Zero();
  const Eigen::Matrix3d identity = Eigen::Matrix3d::Identity();
  EXPECT_FALSE(scheduler.shouldUpdate(1.0, 1, origin, identity, 0.1, false));
  ASSERT_TRUE(scheduler.shouldUpdate(1.002, 31, origin, identity, 0.1, false));
  scheduler.recordUpdate(1.002, origin, identity, 0.32, 32.0);
  EXPECT_EQ(scheduler.commandedPoints(), 71);

  EXPECT_FALSE(scheduler.shouldUpdate(1.0025, 4, origin, identity, 0.1, false));
  EXPECT_TRUE(scheduler.shouldUpdate(1.013, 4, origin, identity, 0.1, false));
  scheduler.recordUpdate(1.013, origin, identity, 0.8, 8.0);
  EXPECT_EQ(scheduler.commandedPoints(), 22);
}

TEST(AdaptiveLio, TemporalSchedulerForcesUpdateOnMotion)
{
  point_lio::TemporalAdaptiveConfig config;
  config.enabled = true;
  config.initial_target_points = 100;
  config.max_translation = 0.02;
  config.max_pose_covariance_trace = 10.0;
  point_lio::TemporalUpdateScheduler scheduler(config);
  const Eigen::Matrix3d identity = Eigen::Matrix3d::Identity();

  EXPECT_FALSE(scheduler.shouldUpdate(1.0, 1, Eigen::Vector3d::Zero(), identity, 0.1, false));
  EXPECT_TRUE(scheduler.shouldUpdate(1.001, 1, Eigen::Vector3d(0.03, 0.0, 0.0), identity, 0.1, false));
}

TEST(AdaptiveLio, StableLogDetHandlesPositiveCovariance)
{
  Eigen::Matrix<double, 6, 6> covariance = Eigen::Matrix<double, 6, 6>::Identity();
  covariance.diagonal() << 1.0, 2.0, 4.0, 0.5, 0.5, 0.5;
  EXPECT_NEAR(point_lio::stableLogDet(covariance), 0.0, 1.0e-12);
}

TEST(AdaptiveLio, IVoxBoundsPointsPerVoxel)
{
  using IVox = faster_lio::IVox<3, faster_lio::IVoxNodeType::DEFAULT, pcl::PointXYZINormal>;
  IVox::Options options;
  options.resolution_ = 1.0;
  options.max_points_per_voxel_ = 4;
  IVox ivox(options);
  IVox::PointVector points;
  for (int i = 0; i < 20; ++i) {
    pcl::PointXYZINormal point;
    point.x = 0.01f * i;
    point.y = 0.0f;
    point.z = 0.0f;
    points.emplace_back(point);
  }
  ivox.AddPoints(points);
  EXPECT_EQ(ivox.NumValidGrids(), 1U);
  EXPECT_EQ(ivox.NumPoints(), 4U);
}
}  // namespace
