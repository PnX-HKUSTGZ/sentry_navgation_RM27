#include "minco_core/components/planning_request_contract.hpp"
#include "minco_core/components/planning_request_lease.hpp"
#include "minco_core/components/planning_session_state.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <type_traits>

namespace minco_planner {
namespace {

using namespace std::chrono_literals;

TEST(PlanningRequestLeaseTest, TenHertzHeartbeatDoesNotExpire)
{
  PlanningRequestLease lease;
  lease.configure(0.75);
  const auto start = PlanningRequestLease::TimePoint{};

  for (int heartbeat = 0; heartbeat < 50; ++heartbeat) {
    const auto now = start + heartbeat * 100ms;
    EXPECT_FALSE(lease.consumeExpired(now).has_value());
    lease.refresh(17U, now);
    EXPECT_FALSE(lease.consumeExpired(now + 99ms).has_value());
  }
}

TEST(PlanningRequestLeaseTest, StopsHeartbeatAndExpiresExactlyOnce)
{
  PlanningRequestLease lease;
  lease.configure(0.75);
  const auto start = PlanningRequestLease::TimePoint{};
  lease.refresh(23U, start);

  EXPECT_FALSE(lease.consumeExpired(start + 749ms).has_value());
  const auto expired = lease.consumeExpired(start + 750ms);
  ASSERT_TRUE(expired.has_value());
  EXPECT_EQ(*expired, 23U);
  EXPECT_FALSE(lease.consumeExpired(start + 10s).has_value());
}

TEST(PlanningRequestLeaseTest, NonPositiveTimeoutDisablesLease)
{
  PlanningRequestLease lease;
  const auto start = PlanningRequestLease::TimePoint{};

  lease.configure(0.0);
  lease.refresh(1U, start);
  EXPECT_FALSE(lease.enabled());
  EXPECT_FALSE(lease.consumeExpired(start + 24h).has_value());

  lease.configure(-1.0);
  lease.refresh(2U, start);
  EXPECT_FALSE(lease.enabled());
  EXPECT_FALSE(lease.consumeExpired(start + 24h).has_value());
}

TEST(PlanningRequestLeaseTest, ResetDisarmsAnActiveLease)
{
  PlanningRequestLease lease;
  lease.configure(0.75);
  const auto start = PlanningRequestLease::TimePoint{};
  lease.refresh(7U, start);

  lease.reset();

  EXPECT_FALSE(lease.consumeExpired(start + 24h).has_value());
}

TEST(PlanningRequestLeaseTest, RefreshExtendsDeadlineAndTracksCurrentSession)
{
  PlanningRequestLease lease;
  lease.configure(0.75);
  const auto start = PlanningRequestLease::TimePoint{};
  lease.refresh(31U, start);
  lease.refresh(32U, start + 700ms);

  EXPECT_FALSE(lease.consumeExpired(start + 1449ms).has_value());
  const auto expired = lease.consumeExpired(start + 1450ms);
  ASSERT_TRUE(expired.has_value());
  EXPECT_EQ(*expired, 32U);
}

TEST(PlanningRequestLeaseTest, RosClockChangesCannotAffectSteadyDeadline)
{
  static_assert(std::is_same_v<PlanningRequestLease::Clock, std::chrono::steady_clock>);

  PlanningRequestLease lease;
  lease.configure(0.75);
  const auto steady_start = PlanningRequestLease::TimePoint{} + 10s;
  lease.refresh(41U, steady_start);

  // ROS timestamps are intentionally absent from the lease API and cannot
  // extend or shorten its monotonic deadline.
  EXPECT_FALSE(lease.consumeExpired(steady_start + 500ms).has_value());

  const auto expired = lease.consumeExpired(steady_start + 750ms);
  ASSERT_TRUE(expired.has_value());
  EXPECT_EQ(*expired, 41U);
}

TEST(PlanningRequestLeaseTest, StaleLeaseCannotFenceAReplacementSession)
{
  PlanningSessionState sessions;
  PlanningRequestLease lease;
  lease.configure(0.75);
  sessions.activate();
  const uint64_t old_session = sessions.beginSession();
  const auto start = PlanningRequestLease::TimePoint{};
  lease.refresh(old_session, start);

  const uint64_t replacement_session = sessions.beginSession();
  const auto expired = lease.consumeExpired(start + 750ms);
  ASSERT_TRUE(expired.has_value());
  EXPECT_FALSE(sessions.accepts(*expired));
  EXPECT_TRUE(sessions.accepts(replacement_session));
}

TEST(PlanningRequestLeaseTest, ExpiryFencesIntegratedPlanningSession)
{
  PlanningSessionState sessions;
  PlanningRequestLease lease;
  builtin_interfaces::msg::Time planning_stamp;
  const builtin_interfaces::msg::Time stalled_ros_time;
  lease.configure(0.75);
  sessions.activate();
  const uint64_t goal_session = sessions.beginSession();
  planning_stamp = planning_contract::monotonicPlanningStamp(stalled_ros_time, planning_stamp);
  const auto goal_stamp = planning_stamp;
  const auto start = PlanningRequestLease::TimePoint{};
  lease.refresh(goal_session, start);

  const auto expired = lease.consumeExpired(start + 750ms);
  ASSERT_TRUE(expired.has_value());
  ASSERT_TRUE(sessions.accepts(*expired));
  const uint64_t timeout_fence_session = sessions.beginSession();
  planning_stamp = planning_contract::monotonicPlanningStamp(stalled_ros_time, planning_stamp);
  const auto timeout_fence_stamp = planning_stamp;

  EXPECT_FALSE(sessions.accepts(goal_session));
  EXPECT_TRUE(sessions.accepts(timeout_fence_session));
  EXPECT_GT(
    planning_contract::planningStampNanoseconds(timeout_fence_stamp),
    planning_contract::planningStampNanoseconds(goal_stamp));

  // A request for the same semantic goal after timeout must begin a fresh session.
  const uint64_t replacement_goal_session = sessions.beginSession();
  planning_stamp = planning_contract::monotonicPlanningStamp(stalled_ros_time, planning_stamp);
  lease.refresh(replacement_goal_session, start + 800ms);
  EXPECT_NE(replacement_goal_session, goal_session);
  EXPECT_FALSE(sessions.accepts(timeout_fence_session));
  EXPECT_TRUE(sessions.accepts(replacement_goal_session));
  EXPECT_GT(
    planning_contract::planningStampNanoseconds(planning_stamp),
    planning_contract::planningStampNanoseconds(timeout_fence_stamp));
}

}  // namespace
}  // namespace minco_planner
