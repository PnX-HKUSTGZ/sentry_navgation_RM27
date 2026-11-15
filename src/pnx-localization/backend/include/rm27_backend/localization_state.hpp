#pragma once
#include <cstdint>
#include <string>
namespace rm27_backend {
/// 全局定位可信状态；地图加载状态单独维护，未找到候选不等于观测冲突。
enum class LocalizationStatus {
  Initializing,
  Normal,
  ObsDegraded,
  Suspect,
  Lost
};
/// 状态机仅管理证据和任务代次，图写入与耗时计算由调度层负责。
class LocalizationStateMachine {
public:
  /// 返回稳定状态名称，供结构化诊断及测试使用。
  const char *name() const;
  /// 当前状态是否允许可信位姿传播和普通地图因子提交。
  bool trusted() const;
  /// 读取主状态。
  LocalizationStatus status() const { return status_; }
  /// 读取任务代次；过期后台结果必须丢弃。
  std::uint64_t generation() const { return generation_; }
  /// 最近转移或验收原因。
  const std::string &reason() const { return reason_; }
  /// 恢复锚或正常观测提交成功后清除冲突证据。
  void accepted(bool recovery, const std::string &reason);
  /// 一个合法观测事务与预测冲突，只累计一次。
  void conflict(int limit, const std::string &reason);
  /// 明确的跟踪假设异常立即进入可疑状态。
  void suspect(const std::string &reason);
  /// 真实恢复尝试失败才计数，等待/预算耗尽/过期结果不计数。
  void recoveryFailed(int limit, const std::string &reason);
  /// 强制隔离故障代次；重复 LOST 也使旧任务失效。
  void lost(const std::string &reason);
  /// 人工恢复请求作废全部在途计算，保留最后可信输出。
  void requestRecovery();

private:
  /// 更改状态时推进代次，防止同名状态的旧结果混入。
  void set(LocalizationStatus value, const std::string &reason);
  LocalizationStatus status_ = LocalizationStatus::Initializing;
  std::uint64_t generation_ = 1;
  int conflicts_ = 0, failures_ = 0;
  std::string reason_ = "waiting for first verified map anchor";
};
} // 命名空间 rm27_backend
