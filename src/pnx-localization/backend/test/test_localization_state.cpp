#include "rm27_backend/localization_state.hpp"
#include <gtest/gtest.h>
using namespace rm27_backend;
/// 普通观测不能绕过初始化和独立恢复验收。
TEST(LocalizationState, RegularCorrectionCannotRecover) {
  LocalizationStateMachine s;
  s.accepted(false, "regular");
  EXPECT_FALSE(s.trusted());
  s.accepted(true, "anchor");
  EXPECT_TRUE(s.trusted());
  s.suspect("jump");
  s.accepted(false, "regular");
  EXPECT_FALSE(s.trusted());
  s.accepted(true, "recovered");
  EXPECT_TRUE(s.trusted());
}
/// 七次独立有效冲突进入可疑；恢复失败三次才扩大搜索。
TEST(LocalizationState, EvidenceAndGenerations) {
  LocalizationStateMachine s;
  s.accepted(true, "anchor");
  const auto initial = s.generation();
  for (int i = 0; i < 6; ++i)
    s.conflict(7, "NIS");
  EXPECT_EQ(s.status(), LocalizationStatus::ObsDegraded);
  EXPECT_TRUE(s.trusted());
  s.conflict(7, "NIS");
  EXPECT_EQ(s.status(), LocalizationStatus::Suspect);
  EXPECT_GT(s.generation(), initial);
  s.recoveryFailed(3, "failed");
  s.recoveryFailed(3, "failed");
  EXPECT_EQ(s.status(), LocalizationStatus::Suspect);
  s.recoveryFailed(3, "failed");
  EXPECT_EQ(s.status(), LocalizationStatus::Lost);
  const auto lost = s.generation();
  s.lost("another fault");
  EXPECT_GT(s.generation(), lost);
}
/// 初始化的失配保持等待；人工请求使同一枚举下的旧任务失效。
TEST(LocalizationState, InitialFailureAndManualInvalidation) {
  LocalizationStateMachine s;
  for (int i = 0; i < 10; ++i)
    s.recoveryFailed(3, "no candidate");
  EXPECT_EQ(s.status(), LocalizationStatus::Initializing);
  s.requestRecovery();
  const auto generation = s.generation();
  s.requestRecovery();
  EXPECT_GT(s.generation(), generation);
}
