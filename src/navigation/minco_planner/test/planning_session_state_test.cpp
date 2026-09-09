#include <gtest/gtest.h>

#include "minco_core/components/planning_session_state.hpp"

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

} // namespace minco_planner
