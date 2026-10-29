#include <gtest/gtest.h>

#include "minco_core/components/planning_session_state.hpp"
#include "minco_core/components/route_refresh_policy.hpp"
#include <limits>

namespace minco_planner {

TEST(PlanningSessionStateTest, RejectsWorkUntilActivated) {
  PlanningSessionState state;
  const auto configured_generation = state.beginSession();

  EXPECT_FALSE(state.active());
  EXPECT_FALSE(state.accepts(configured_generation));
}

TEST(PlanningSessionStateTest, ReplacingGoalInvalidatesInFlightWork) {
  PlanningSessionState state;
  const auto activation_generation = state.activate();
  const auto first_goal_generation = state.beginSession();

  EXPECT_FALSE(state.accepts(activation_generation));
  EXPECT_TRUE(state.accepts(first_goal_generation));

  const auto replacement_generation = state.beginSession();
  EXPECT_FALSE(state.accepts(first_goal_generation));
  EXPECT_TRUE(state.accepts(replacement_generation));
}

TEST(PlanningSessionStateTest, DeactivateAndReactivateRejectOldResults) {
  PlanningSessionState state;
  state.activate();
  const auto old_goal_generation = state.beginSession();

  state.deactivate();
  EXPECT_FALSE(state.accepts(old_goal_generation));

  const auto new_activation_generation = state.activate();
  EXPECT_FALSE(state.accepts(old_goal_generation));
  EXPECT_TRUE(state.accepts(new_activation_generation));
}

TEST(PlanningSessionStateTest, RollingReplansRestoreHeadingInsteadOfIntegratingDrift) {
  PlanningSessionState state;
  state.activate();
  const auto first = state.beginSession();
  ASSERT_TRUE(state.cruiseHeading(first, .02));
  for (int i = 1; i <= 100; ++i) {
    ASSERT_TRUE(state.cruiseHeading(first, .02 + i*.01));
    EXPECT_DOUBLE_EQ(*state.cruiseHeading(first, .02 + i*.01), .02);
  }
  const auto second = state.beginSession();
  EXPECT_FALSE(state.cruiseHeading(first, -2.));
  EXPECT_DOUBLE_EQ(*state.cruiseHeading(second, .7), .7);
  state.deactivate();
  EXPECT_FALSE(state.cruiseHeading(second, 0.));
}

TEST(RouteRefreshPolicyTest, MovingThroughVisibilityFrontierKeepsCorridor) {
  RouteRefreshPolicy policy;
  policy.reset(1, 0.0, 0.0);
  for (int i = 1; i <= 30; ++i) {
    // Continued progress across short stopping prefixes must not turn the
    // robot around just because a new scan temporarily clips the guide.
    EXPECT_TRUE(policy.defer(1, i * .25, i * .5, 1.5));
  }
  // Once a real obstruction prevents forward progress, rerouting is bounded.
  EXPECT_TRUE(policy.defer(1, 7.5, 16.4, 1.5));
  EXPECT_FALSE(policy.defer(1, 7.5, 16.5, 1.5));
}

TEST(RouteRefreshPolicyTest, BacktrackingAndSmallCorrectionsCannotDelayDetourForever) {
  RouteRefreshPolicy policy;
  policy.reset(8, 10.0, 3.0);
  policy.observe(8, 10.1, 3.4);
  policy.observe(8, 9.8, 3.8);
  policy.observe(8, 10.1, 4.4);
  EXPECT_FALSE(policy.defer(8, 10.0, 4.5, 1.5));
}

TEST(RouteRefreshPolicyTest, NewGoalAndInvalidClockCannotInheritWait) {
  RouteRefreshPolicy policy;
  policy.reset(3, 0.0, 10.0);
  EXPECT_FALSE(policy.defer(4, 0.0, 10.1, 1.5));
  policy.reset(4, 0.0, 10.1);
  policy.observe(3, 100.0, 12.0);
  EXPECT_FALSE(policy.defer(4, 0.0, 11.7, 1.5));
  EXPECT_FALSE(policy.defer(4, 0.0, 10.2, 0.0));
  EXPECT_FALSE(policy.defer(4, 0.0, 9.0, 1.5));
  EXPECT_FALSE(policy.defer(4, std::numeric_limits<double>::quiet_NaN(), 10.2, 1.5));
}

TEST(RouteRefreshPolicyTest, MapEvidenceWindowIsBoundedInTheSuppliedClock) {
  RouteRefreshPolicy policy;
  policy.reset(12, 2.0, 30.0);
  // Repeated wall-clock callbacks with paused simulation must not consume
  // the map's evidence window or invent forward progress.
  for (int i = 0; i < 100; ++i) {
    EXPECT_TRUE(policy.defer(12, 2.0, 30.0, 4.0));
  }
  EXPECT_TRUE(policy.defer(12, 2.0, 33.99, 4.0));
  EXPECT_FALSE(policy.defer(12, 2.0, 34.0, 4.0));
  EXPECT_FALSE(policy.defer(12, 2.0, 1.0, 4.0));
}

} // namespace minco_planner
