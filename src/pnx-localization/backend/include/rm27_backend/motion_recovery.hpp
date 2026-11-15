#pragma once
#include "relocalization_manager/recovery_engine.hpp"
#include <chrono>

namespace rm27_backend {
/// 健康前端连续性提供的保守恢复范围；线性增长项是工程误差包络，并非滤波协方差。
struct MotionRecoveryConfig {
  bool enabled = true;
  double max_age = 15.; // 最后可信锚的源时间和墙钟年龄上限，s。
  double base_radius = 1.5, max_radius = 8.; // 恢复位置包络初值与最大值，m。
  double drift_per_second = .15, drift_per_meter = .08;
  double base_angle = .2617993878, max_angle = 1.0471975512;
  double angle_per_second = .01745329252, angle_per_meter = .03490658504;
  double angle_per_radian = .1;
  double max_gap = 1., max_speed = 12., max_angular_speed = 4.;
};
/// 保存最后可信地图锚及其同刻前端位姿；异常后不可凭后续健康帧复活旧锚。
class MotionRecovery {
public:
  /// 校验误差包络、年龄和连续性门限。
  explicit MotionRecovery(MotionRecoveryConfig config = {});
  /// 检查同身份时间序列和运动合理性，并累计路径长度及转角。
  void observe(const Frame &frame);
  /// 只有新验证的可信状态可刷新锚；必须与最近 observe 的时刻、身份完全一致。
  void anchor(const Pose &map_pose, const Frame &frame);
  /// 作废旧锚和运动历史；重置、人工位置提示和图异常应调用。
  void invalidate(const std::string &reason);
  /// 生成 map←IMU 预测及有界位置、姿态范围，超龄或不连续时返回空。
  std::optional<RecoveryRegion> predict(const Frame &frame) const;
  /// 返回无约束或当前有效约束的诊断原因。
  std::string reason(const Frame &frame) const;

private:
  MotionRecoveryConfig config_;
  std::optional<Frame> previous_, anchor_frame_;
  Pose map_anchor_ = Pose::Identity();
  double distance_ = 0., rotation_ = 0.;
  std::chrono::steady_clock::time_point anchored_at_;
  std::string invalid_reason_ = "no verified continuity anchor";
};
} // namespace rm27_backend
