#include "rm27_backend/localization_state.hpp"
#include <algorithm>
namespace rm27_backend {
/// 状态名用于消息和日志，保持稳定。
const char *LocalizationStateMachine::name() const {
  switch (status_) {
  case LocalizationStatus::Initializing:
    return "INITIALIZING";
  case LocalizationStatus::Normal:
    return "NORMAL";
  case LocalizationStatus::ObsDegraded:
    return "OBS_DEGRADED";
  case LocalizationStatus::Suspect:
    return "SUSPECT";
  case LocalizationStatus::Lost:
    return "LOST";
  }
  return "LOST";
}
/// 降级状态仍可传播，暂停提交的冲突观测不污染预测。
bool LocalizationStateMachine::trusted() const {
  return status_ == LocalizationStatus::Normal ||
         status_ == LocalizationStatus::ObsDegraded;
}
/// 切换阶段并记录原因。
void LocalizationStateMachine::set(LocalizationStatus s,
                                   const std::string &reason) {
  if (status_ != s)
    ++generation_;
  status_ = s;
  reason_ = reason;
}
/// 普通校正不能使可疑或丢失状态自行恢复。
void LocalizationStateMachine::accepted(bool recovery,
                                        const std::string &reason) {
  if (!recovery && !trusted())
    return;
  conflicts_ = failures_ = 0;
  set(LocalizationStatus::Normal, reason);
}
/// 连续真实冲突达到阈值后冻结全局可信输出。
void LocalizationStateMachine::conflict(int limit, const std::string &reason) {
  if (!trusted())
    return;
  set(++conflicts_ >= std::max(1, limit) ? LocalizationStatus::Suspect
                                         : LocalizationStatus::ObsDegraded,
      reason);
}
/// 运动或模型异常作废当前跟踪阶段。
void LocalizationStateMachine::suspect(const std::string &reason) {
  if (!trusted())
    return;
  conflicts_ = failures_ = 0;
  set(LocalizationStatus::Suspect, reason);
}
/// 初始化继续等待，可疑阶段连续失败升级为全局搜索。
void LocalizationStateMachine::recoveryFailed(int limit,
                                              const std::string &reason) {
  if (status_ == LocalizationStatus::Suspect &&
      ++failures_ >= std::max(1, limit))
    set(LocalizationStatus::Lost, reason);
  else
    reason_ = reason;
}
/// 故障总是建立新身份，即使已经 LOST。
void LocalizationStateMachine::lost(const std::string &reason) {
  ++generation_;
  conflicts_ = failures_ = 0;
  set(LocalizationStatus::Lost, reason);
}
/// 人工请求允许在任意状态进入独立恢复。
void LocalizationStateMachine::requestRecovery() {
  lost("manual recovery requested");
}
} // 命名空间 rm27_backend
