// Copyright 2026 RM27 Contributors
// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>

#include <limits>

#include "relocalization_manager/stationary_gate.hpp"

using relocalization_manager::StationaryGate;

static void feed(
  StationaryGate & gate, double from, double to, double linear = 0.0, double angular = 0.0)
{
  for (double t = from; t < to; t += 0.05) {
    gate.odometry(t, t, linear, angular, true);
    gate.stationary(t);
  }
}

TEST(StationaryGate, RequiresExplicitStopAndFreshMeasuredStandstill)
{
  StationaryGate gate;
  feed(gate, 1.0, 2.0);
  EXPECT_FALSE(gate.stationary(2.0));
  gate.command(2.0, 0.0, 0.0);
  feed(gate, 2.01, 2.5);
  EXPECT_FALSE(gate.stationary(2.5));
  feed(gate, 2.51, 3.0);
  EXPECT_TRUE(gate.stationary(3.0));
  EXPECT_FALSE(gate.stationary(4.0));
  feed(gate, 4.01, 4.5);
  EXPECT_FALSE(gate.stationary(4.5));
  feed(gate, 4.51, 5.0);
  EXPECT_TRUE(gate.stationary(5.0));
}

TEST(StationaryGate, RotationAndCoastingPreventCorrection)
{
  StationaryGate gate;
  gate.command(1.0, 0.0, 0.3);
  feed(gate, 1.0, 3.0);
  EXPECT_FALSE(gate.stationary(3.0));  // Stale nonzero command never implies stop.
  gate.command(3.0, 0.0, 0.0);
  feed(gate, 3.0, 4.0, 0.2);
  EXPECT_FALSE(gate.stationary(4.0));
  feed(gate, 4.0, 5.0, 0.0, 0.2);
  EXPECT_FALSE(gate.stationary(5.0));
  feed(gate, 5.0, 6.0);
  EXPECT_TRUE(gate.stationary(6.0));
}

TEST(StationaryGate, InvalidSamplesAndClockResetInvalidateSettlement)
{
  StationaryGate gate;
  gate.command(1.0, 0.0, 0.0);
  feed(gate, 1.0, 2.0);
  ASSERT_TRUE(gate.stationary(2.0));
  gate.odometry(2.1, 2.1, 0.0, 0.0, false);
  EXPECT_FALSE(gate.stationary(2.1));
  gate.odometry(2.2, 2.3, 0.0, 0.0, true);
  EXPECT_FALSE(gate.stationary(2.2));
  gate.odometry(2.3, 2.1, std::numeric_limits<double>::quiet_NaN(), 0.0, true);
  EXPECT_FALSE(gate.stationary(2.3));
  feed(gate, 2.4, 3.5);
  ASSERT_TRUE(gate.stationary(3.5));
  EXPECT_FALSE(gate.stationary(0.0));
  feed(gate, 0.01, 1.0);
  EXPECT_FALSE(gate.stationary(1.0));  // Must receive a new stop command.
  gate.command(1.0, 0.0, 0.0);
  feed(gate, 1.01, 2.0);
  EXPECT_TRUE(gate.stationary(2.0));
}
