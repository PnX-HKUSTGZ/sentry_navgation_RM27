// Copyright 2026 RM27 contributors. SPDX-License-Identifier: Apache-2.0
#include "global_relocalization/competition_state.hpp"
#include <gtest/gtest.h>
using global_relocalization::CompetitionState;
TEST(CompetitionState, InactiveAndEarlyStartNeverAuthorizeSuccess) {
  CompetitionState state;EXPECT_EQ(state.start(),CompetitionState::Start::Inactive);
  auto generation=state.begin();EXPECT_EQ(state.start(),CompetitionState::Start::TooEarly);
  EXPECT_FALSE(state.prepared(generation));EXPECT_FALSE(state.finish(true));
  EXPECT_FALSE(state.releaseControl());
}
TEST(CompetitionState, PreparedStartIsOnceOnlyAndSuccessReleasesControl) {
  CompetitionState state;auto generation=state.begin();ASSERT_TRUE(state.prepared(generation));
  EXPECT_TRUE(state.ready());EXPECT_EQ(state.start(),CompetitionState::Start::Accepted);
  EXPECT_EQ(state.start(),CompetitionState::Start::Duplicate);EXPECT_TRUE(state.finish(true));
  EXPECT_EQ(state.start(),CompetitionState::Start::Inactive);
}
TEST(CompetitionState, CancellationAndOldPreparationCannotContaminateNextRun) {
  CompetitionState state;auto previous=state.begin();ASSERT_TRUE(state.prepared(previous));
  EXPECT_FALSE(state.finish(false));auto current=state.begin();EXPECT_NE(previous,current);
  EXPECT_FALSE(state.prepared(previous));EXPECT_FALSE(state.ready());EXPECT_FALSE(state.started());
  EXPECT_EQ(state.start(),CompetitionState::Start::TooEarly);EXPECT_FALSE(state.finish(false));
}
TEST(CompetitionState, UnstartedPreparationCannotPublishSuccess) {
  CompetitionState state;ASSERT_TRUE(state.prepared(state.begin()));EXPECT_FALSE(state.finish(true));
}
