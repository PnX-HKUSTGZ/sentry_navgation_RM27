#include <gtest/gtest.h>

#include "minco_controller/trajectory_session_gate.hpp"

namespace minco_controller {

TEST(TrajectorySessionGateTest, RequiresActivationAndMatchingPlanToken) {
  TrajectorySessionGate gate;
  gate.configure();

  EXPECT_EQ(gate.classifyNormal(10U),
            TrajectorySessionGate::NormalDisposition::REJECT);
  gate.activate();
  EXPECT_TRUE(gate.announceBlock(10U));
  EXPECT_EQ(gate.classifyNormal(10U),
            TrajectorySessionGate::NormalDisposition::WAIT_FOR_PLAN);

  EXPECT_EQ(gate.setPlan(10U, true),
            TrajectorySessionGate::PlanUpdate::AUTHORIZED);
  EXPECT_EQ(gate.classifyNormal(9U),
            TrajectorySessionGate::NormalDisposition::REJECT);
  EXPECT_EQ(gate.classifyNormal(10U),
            TrajectorySessionGate::NormalDisposition::ACCEPT);
}

TEST(TrajectorySessionGateTest, SameTokenRefreshPreservesAuthorization) {
  TrajectorySessionGate gate;
  gate.activate();
  EXPECT_TRUE(gate.announceBlock(5U));
  EXPECT_EQ(gate.setPlan(5U, true),
            TrajectorySessionGate::PlanUpdate::AUTHORIZED);
  EXPECT_EQ(gate.setPlan(5U, true),
            TrajectorySessionGate::PlanUpdate::REFRESHED);
  EXPECT_EQ(gate.classifyNormal(5U),
            TrajectorySessionGate::NormalDisposition::ACCEPT);
}

TEST(TrajectorySessionGateTest, NewBlockRevokesOldSession) {
  TrajectorySessionGate gate;
  gate.activate();
  ASSERT_TRUE(gate.announceBlock(5U));
  ASSERT_EQ(gate.setPlan(5U, true),
            TrajectorySessionGate::PlanUpdate::AUTHORIZED);

  EXPECT_TRUE(gate.announceBlock(6U));
  EXPECT_FALSE(gate.hasPlan());
  EXPECT_EQ(gate.classifyNormal(5U),
            TrajectorySessionGate::NormalDisposition::REJECT);
  EXPECT_EQ(gate.classifyNormal(6U),
            TrajectorySessionGate::NormalDisposition::WAIT_FOR_PLAN);
  EXPECT_FALSE(gate.announceBlock(5U));
}

TEST(TrajectorySessionGateTest, ReactivationCannotReuseOldPlanOrTrajectory) {
  TrajectorySessionGate gate;
  gate.activate();
  gate.announceBlock(2U);
  gate.setPlan(2U, true);
  EXPECT_EQ(gate.classifyNormal(2U),
            TrajectorySessionGate::NormalDisposition::ACCEPT);

  gate.deactivate();
  EXPECT_EQ(gate.classifyNormal(2U),
            TrajectorySessionGate::NormalDisposition::REJECT);

  gate.activate();
  EXPECT_EQ(gate.classifyNormal(2U),
            TrajectorySessionGate::NormalDisposition::REJECT);
  gate.announceBlock(4U);
  gate.setPlan(4U, true);
  EXPECT_EQ(gate.classifyNormal(4U),
            TrajectorySessionGate::NormalDisposition::ACCEPT);
}

TEST(TrajectorySessionGateTest, EmptyPlanKeepsGateClosed) {
  TrajectorySessionGate gate;
  gate.activate();
  gate.announceBlock(1U);
  EXPECT_EQ(gate.setPlan(1U, false),
            TrajectorySessionGate::PlanUpdate::CLEARED);

  EXPECT_FALSE(gate.hasPlan());
  EXPECT_EQ(gate.classifyNormal(1U),
            TrajectorySessionGate::NormalDisposition::REJECT);
}

TEST(TrajectorySessionGateTest, PathMayArriveBeforeMatchingBlock) {
  TrajectorySessionGate gate;
  gate.activate();
  EXPECT_EQ(gate.setPlan(7U, true),
            TrajectorySessionGate::PlanUpdate::AUTHORIZED);
  EXPECT_TRUE(gate.announceBlock(7U));
  EXPECT_TRUE(gate.hasPlan());
  EXPECT_EQ(gate.classifyNormal(7U),
            TrajectorySessionGate::NormalDisposition::ACCEPT);
}

TEST(TrajectorySessionGateTest, SafetyBlockKeepsMatchingPlanAuthorization) {
  TrajectorySessionGate gate;
  gate.activate();
  ASSERT_TRUE(gate.announceBlock(11U));
  ASSERT_EQ(gate.setPlan(11U, true),
            TrajectorySessionGate::PlanUpdate::AUTHORIZED);

  EXPECT_TRUE(gate.announceBlock(11U));
  EXPECT_TRUE(gate.hasPlan());
  EXPECT_EQ(gate.classifyNormal(11U),
            TrajectorySessionGate::NormalDisposition::ACCEPT);
}

TEST(TrajectorySessionGateTest, StalePlanAndBlockCannotReplaceNewSession) {
  TrajectorySessionGate gate;
  gate.activate();
  ASSERT_TRUE(gate.announceBlock(20U));
  ASSERT_EQ(gate.setPlan(20U, true),
            TrajectorySessionGate::PlanUpdate::AUTHORIZED);

  ASSERT_TRUE(gate.announceBlock(30U));
  EXPECT_EQ(gate.setPlan(20U, true),
            TrajectorySessionGate::PlanUpdate::REJECTED);
  EXPECT_FALSE(gate.announceBlock(20U));
  EXPECT_EQ(gate.classifyNormal(20U),
            TrajectorySessionGate::NormalDisposition::REJECT);
  EXPECT_EQ(gate.classifyNormal(30U),
            TrajectorySessionGate::NormalDisposition::WAIT_FOR_PLAN);
}

} // namespace minco_controller
