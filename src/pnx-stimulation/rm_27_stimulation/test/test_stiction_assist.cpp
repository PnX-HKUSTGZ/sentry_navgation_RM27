#include <limits>

#include <gtest/gtest.h>

#include "rm_27_stimulation/stiction_assist.hpp"

namespace rm_27_stimulation {
namespace {

StictionAssistConfig TestConfig() {
  return {0.08, 0.02, 0.10, 0.20, 100.0, 45.0};
}

TEST(StictionAssist, WaitsForDelayThenRampsToLimit) {
  StictionAssist assist;
  const auto config = TestConfig();

  EXPECT_DOUBLE_EQ(assist.Update(0.5, 0.0, 0.0, 0.0, 0.1, config).x, 0.0);
  EXPECT_DOUBLE_EQ(assist.Update(0.5, 0.0, 0.0, 0.0, 0.1, config).x, 0.0);
  EXPECT_NEAR(assist.Update(0.5, 0.0, 0.0, 0.0, 0.1, config).x, 10.0, 1e-12);
  for (int i = 0; i < 10; ++i) {
    assist.Update(0.5, 0.0, 0.0, 0.0, 0.1, config);
  }
  EXPECT_NEAR(assist.Force(), 45.0, 1e-12);
}

TEST(StictionAssist, UsesProgressAlongCommandAndReleasesWithoutStoredImpulse) {
  StictionAssist assist;
  const auto config = TestConfig();
  for (int i = 0; i < 10; ++i) {
    assist.Update(0.0, -0.5, 0.2, 0.05, 0.1, config);
  }
  ASSERT_GT(assist.Force(), 0.0);

  const auto released = assist.Update(0.0, -0.5, 0.0, -0.11, 0.1, config);
  EXPECT_DOUBLE_EQ(released.x, 0.0);
  EXPECT_DOUBLE_EQ(released.y, 0.0);
  EXPECT_DOUBLE_EQ(assist.Force(), 0.0);
}

TEST(StictionAssist, ResetsForSmallCommandDirectionChangeAndInvalidInput) {
  StictionAssist assist;
  const auto config = TestConfig();
  for (int i = 0; i < 10; ++i) {
    assist.Update(0.5, 0.0, 0.0, 0.0, 0.1, config);
  }
  ASSERT_GT(assist.Force(), 0.0);

  EXPECT_DOUBLE_EQ(assist.Update(-0.5, 0.0, 0.0, 0.0, 0.1, config).x, 0.0);
  EXPECT_DOUBLE_EQ(assist.Force(), 0.0);
  EXPECT_DOUBLE_EQ(assist.Update(0.01, 0.0, 0.0, 0.0, 0.1, config).x, 0.0);
  EXPECT_DOUBLE_EQ(assist
                       .Update(0.5, 0.0,
                               std::numeric_limits<double>::quiet_NaN(), 0.0,
                               0.1, config)
                       .x,
                   0.0);
}

} // namespace
} // namespace rm_27_stimulation
