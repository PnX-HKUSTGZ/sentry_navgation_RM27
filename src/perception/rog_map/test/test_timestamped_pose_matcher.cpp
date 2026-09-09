#include <gtest/gtest.h>

#include <rog_map_ros/timestamped_pose_matcher.hpp>

#include <memory>

namespace
{

using Matcher = rog_map::detail::TimestampedPoseMatcher<int, std::unique_ptr<int>>;

Matcher makeMatcher(
  double tolerance = 0.1, double pending_timeout = 0.25,
  double pose_timeout = 0.25, std::size_t pose_capacity = 8U,
  std::size_t cloud_capacity = 4U)
{
  return Matcher({tolerance, pending_timeout, pose_timeout, pose_capacity, cloud_capacity});
}

TEST(TimestampedPoseMatcher, MatchesExactStampForEitherCallbackOrder)
{
  auto odom_first = makeMatcher();
  ASSERT_TRUE(odom_first.addPose(10.0, 10.01, 100));
  EXPECT_EQ(
    odom_first.addCloud(10.0, 10.02, std::make_unique<int>(1)).status,
    Matcher::CloudInsertStatus::STORED);
  auto matched = odom_first.takeNext(10.02);
  ASSERT_EQ(matched.status, Matcher::TakeStatus::MATCHED);
  ASSERT_TRUE(matched.match.has_value());
  EXPECT_EQ(matched.match->pose, 100);
  EXPECT_EQ(*matched.match->payload, 1);

  auto cloud_first = makeMatcher();
  ASSERT_TRUE(cloud_first.addPose(19.90, 20.00, 199));
  ASSERT_EQ(
    cloud_first.addCloud(20.0, 20.01, std::make_unique<int>(2)).status,
    Matcher::CloudInsertStatus::STORED);
  // Do not consume the previous 10 Hz pose just because DDS delivered the
  // equal-stamp cloud before its equal-stamp odometry.
  EXPECT_EQ(cloud_first.takeNext(20.01).status, Matcher::TakeStatus::NONE);
  ASSERT_TRUE(cloud_first.addPose(20.0, 20.02, 200));
  matched = cloud_first.takeNext(20.02);
  ASSERT_EQ(matched.status, Matcher::TakeStatus::MATCHED);
  ASSERT_TRUE(matched.match.has_value());
  EXPECT_EQ(matched.match->pose, 200);
  EXPECT_EQ(*matched.match->payload, 2);
}

TEST(TimestampedPoseMatcher, WaitsForOdomTimelineThenChoosesClosestApproximateStamp)
{
  auto matcher = makeMatcher();
  ASSERT_TRUE(matcher.addPose(9.90, 10.00, 90));
  ASSERT_EQ(
    matcher.addCloud(10.00, 10.01, std::make_unique<int>(7)).status,
    Matcher::CloudInsertStatus::STORED);

  // The previous 10 Hz odometry is within the nominal tolerance, but matching
  // it now would make the result depend on DDS callback order.
  EXPECT_EQ(matcher.takeNext(10.01).status, Matcher::TakeStatus::NONE);
  ASSERT_TRUE(matcher.addPose(10.06, 10.02, 106));

  const auto result = matcher.takeNext(10.02);
  ASSERT_EQ(result.status, Matcher::TakeStatus::MATCHED);
  ASSERT_TRUE(result.match.has_value());
  EXPECT_EQ(result.match->pose, 106);
  EXPECT_NEAR(result.match->stamp_delta, 0.06, 1.0e-12);
}

TEST(TimestampedPoseMatcher, UsesValidEarlierApproximatePoseAfterReorderWindow)
{
  auto matcher = makeMatcher(0.1, 0.05, 0.05);
  ASSERT_TRUE(matcher.addPose(9.98, 10.00, 98));
  ASSERT_EQ(
    matcher.addCloud(10.00, 10.01, std::make_unique<int>(9)).status,
    Matcher::CloudInsertStatus::STORED);

  EXPECT_EQ(matcher.takeNext(10.05).status, Matcher::TakeStatus::NONE);
  const auto result = matcher.takeNext(10.07);
  ASSERT_EQ(result.status, Matcher::TakeStatus::MATCHED);
  ASSERT_TRUE(result.match.has_value());
  EXPECT_EQ(result.match->pose, 98);
  EXPECT_NEAR(result.match->stamp_delta, 0.02, 1.0e-12);
}

TEST(TimestampedPoseMatcher, ChoosesClosestFreshPoseFromHistory)
{
  auto matcher = makeMatcher();
  ASSERT_TRUE(matcher.addPose(9.96, 10.00, 96));
  ASSERT_TRUE(matcher.addPose(10.08, 10.01, 108));
  ASSERT_EQ(
    matcher.addCloud(10.00, 10.02, std::make_unique<int>(3)).status,
    Matcher::CloudInsertStatus::STORED);

  const auto result = matcher.takeNext(10.02);
  ASSERT_EQ(result.status, Matcher::TakeStatus::MATCHED);
  ASSERT_TRUE(result.match.has_value());
  EXPECT_EQ(result.match->pose, 96);
  EXPECT_NEAR(result.match->stamp_delta, 0.04, 1.0e-12);
}

TEST(TimestampedPoseMatcher, AcceptsFloatingPointToleranceBoundary)
{
  auto matcher = makeMatcher(0.1);
  ASSERT_TRUE(matcher.addPose(10.1, 10.11, 101));
  ASSERT_EQ(
    matcher.addCloud(10.0, 10.12, std::make_unique<int>(4)).status,
    Matcher::CloudInsertStatus::STORED);

  EXPECT_EQ(matcher.takeNext(10.12).status, Matcher::TakeStatus::MATCHED);
}

TEST(TimestampedPoseMatcher, FailsClosedWhenNoAcceptableOdomArrives)
{
  auto no_odom = makeMatcher(0.1, 0.25);
  ASSERT_EQ(
    no_odom.addCloud(10.0, 10.01, std::make_unique<int>(5)).status,
    Matcher::CloudInsertStatus::STORED);
  EXPECT_EQ(no_odom.takeNext(10.25).status, Matcher::TakeStatus::NONE);
  EXPECT_EQ(no_odom.takeNext(10.27).status, Matcher::TakeStatus::DROPPED_NO_ODOMETRY);

  auto unsynchronized = makeMatcher(0.05, 0.25);
  ASSERT_TRUE(unsynchronized.addPose(10.20, 10.20, 102));
  ASSERT_EQ(
    unsynchronized.addCloud(10.0, 10.21, std::make_unique<int>(6)).status,
    Matcher::CloudInsertStatus::STORED);
  EXPECT_EQ(
    unsynchronized.takeNext(10.21).status,
    Matcher::TakeStatus::DROPPED_SYNC_TOLERANCE);

  auto stale = makeMatcher(0.1, 0.25, 0.05);
  ASSERT_TRUE(stale.addPose(10.0, 10.0, 100));
  ASSERT_EQ(
    stale.addCloud(10.0, 10.10, std::make_unique<int>(8)).status,
    Matcher::CloudInsertStatus::STORED);
  EXPECT_EQ(stale.takeNext(10.10).status, Matcher::TakeStatus::DROPPED_ODOMETRY_TIMEOUT);
}

TEST(TimestampedPoseMatcher, EnforcesMonotonicInputsAndBoundedStorage)
{
  auto matcher = makeMatcher(0.1, 0.25, 0.25, 2U, 2U);
  EXPECT_TRUE(matcher.addPose(1.0, 1.0, 1));
  EXPECT_TRUE(matcher.addPose(2.0, 2.0, 2));
  EXPECT_TRUE(matcher.addPose(3.0, 3.0, 3));
  EXPECT_EQ(matcher.poseCount(), 2U);
  EXPECT_FALSE(matcher.addPose(2.5, 3.0, 25));

  EXPECT_EQ(
    matcher.addCloud(4.0, 4.0, std::make_unique<int>(4)).status,
    Matcher::CloudInsertStatus::STORED);
  EXPECT_EQ(
    matcher.addCloud(5.0, 5.0, std::make_unique<int>(5)).status,
    Matcher::CloudInsertStatus::STORED);
  const auto third = matcher.addCloud(6.0, 6.0, std::make_unique<int>(6));
  EXPECT_EQ(third.status, Matcher::CloudInsertStatus::STORED);
  EXPECT_EQ(third.evicted_count, 1U);
  EXPECT_EQ(matcher.pendingCount(), 2U);
  EXPECT_EQ(
    matcher.addCloud(6.0, 6.1, std::make_unique<int>(60)).status,
    Matcher::CloudInsertStatus::DUPLICATE_OR_OUT_OF_ORDER);
  EXPECT_EQ(
    matcher.addCloud(5.5, 6.1, std::make_unique<int>(55)).status,
    Matcher::CloudInsertStatus::DUPLICATE_OR_OUT_OF_ORDER);
}

}  // namespace
