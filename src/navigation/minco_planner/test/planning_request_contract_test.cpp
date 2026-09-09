#include "minco_core/components/planning_request_contract.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <string>

namespace minco_planner::planning_contract {
namespace {

geometry_msgs::msg::PoseStamped makeGoal(
  const std::string & frame, double x, double y, double z, double yaw)
{
  geometry_msgs::msg::PoseStamped goal;
  goal.header.frame_id = frame;
  goal.pose.position.x = x;
  goal.pose.position.y = y;
  goal.pose.position.z = z;
  goal.pose.orientation.z = std::sin(0.5 * yaw);
  goal.pose.orientation.w = std::cos(0.5 * yaw);
  return goal;
}

builtin_interfaces::msg::Time makeStamp(int32_t sec, uint32_t nanosec)
{
  builtin_interfaces::msg::Time stamp;
  stamp.sec = sec;
  stamp.nanosec = nanosec;
  return stamp;
}

TEST(PlanningRequestContract, ReusesEquivalentGoalWithinPositionAndYawTolerance)
{
  auto lhs = makeGoal("map", 1.0, -2.0, 0.4, 0.75);
  auto rhs = makeGoal("map", 1.0006, -1.9994, 0.4, 0.7509);
  lhs.header.stamp = makeStamp(10, 0U);
  rhs.header.stamp = makeStamp(20, 0U);

  EXPECT_TRUE(samePlanningGoal(lhs, rhs));

  rhs.pose.position.x = 1.0008;
  rhs.pose.position.y = -1.9992;
  EXPECT_FALSE(samePlanningGoal(lhs, rhs));
}

TEST(PlanningRequestContract, RequiresExactlyMatchingNonemptyFrame)
{
  const auto map_goal = makeGoal("map", 1.0, 2.0, 0.0, 0.0);
  auto other_goal = map_goal;
  other_goal.header.frame_id = "odom";
  EXPECT_FALSE(samePlanningGoal(map_goal, other_goal));

  auto empty_frame_goal = map_goal;
  empty_frame_goal.header.frame_id.clear();
  EXPECT_FALSE(samePlanningGoal(empty_frame_goal, empty_frame_goal));
}

TEST(PlanningRequestContract, TreatsYawAcrossPiBoundaryAsEquivalent)
{
  constexpr double kPi = 3.14159265358979323846;
  const auto lhs = makeGoal("map", 0.0, 0.0, 0.0, kPi - 2.0e-4);
  auto rhs = makeGoal("map", 0.0, 0.0, 0.0, -kPi + 2.0e-4);
  EXPECT_TRUE(samePlanningGoal(lhs, rhs));

  rhs = makeGoal("map", 0.0, 0.0, 0.0, -kPi + 2.0e-3);
  EXPECT_FALSE(samePlanningGoal(lhs, rhs));
}

TEST(PlanningRequestContract, RejectsInvalidPoseOrQuaternion)
{
  const auto valid = makeGoal("map", 0.0, 0.0, 0.0, 0.0);
  auto invalid_position = valid;
  invalid_position.pose.position.z = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(samePlanningGoal(valid, invalid_position));

  auto invalid_quaternion = valid;
  invalid_quaternion.pose.orientation.x = 0.0;
  invalid_quaternion.pose.orientation.y = 0.0;
  invalid_quaternion.pose.orientation.z = 0.0;
  invalid_quaternion.pose.orientation.w = 0.0;
  EXPECT_FALSE(samePlanningGoal(valid, invalid_quaternion));
}

TEST(PlanningRequestContract, AdvancesWhenRosTimeStallsOrMovesBackward)
{
  const auto previous = makeStamp(12, 345U);

  const auto stalled = monotonicPlanningStamp(previous, previous);
  EXPECT_EQ(planningStampNanoseconds(stalled), planningStampNanoseconds(previous) + 1);

  const auto backward = monotonicPlanningStamp(makeStamp(1, 0U), stalled);
  EXPECT_EQ(planningStampNanoseconds(backward), planningStampNanoseconds(stalled) + 1);
}

TEST(PlanningRequestContract, PreservesAForwardRosTimeCandidate)
{
  const auto previous = makeStamp(12, 345U);
  const auto candidate = makeStamp(15, 678U);
  const auto result = monotonicPlanningStamp(candidate, previous);

  EXPECT_EQ(planningStampNanoseconds(result), planningStampNanoseconds(candidate));
}

TEST(PlanningRequestContract, NeverGeneratesTheReservedZeroToken)
{
  const auto result = monotonicPlanningStamp(makeStamp(0, 0U), makeStamp(0, 0U));
  EXPECT_EQ(planningStampNanoseconds(result), 1);
}

}  // namespace
}  // namespace minco_planner::planning_contract
