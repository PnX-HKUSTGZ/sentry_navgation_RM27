#include "rm27_backend/motion_recovery.hpp"
#include <algorithm>

namespace rm27_backend {
/// 连续性约束不能比基础包络更小，也不能接受无限年龄或非有限阈值。
MotionRecovery::MotionRecovery(MotionRecoveryConfig c) : config_(c) {
  for (double v : {c.max_age, c.base_radius, c.max_radius, c.base_angle,
                   c.max_angle, c.max_gap, c.max_speed, c.max_angular_speed})
    if (!std::isfinite(v) || v <= 0)
      throw std::invalid_argument("invalid motion recovery limits");
  for (double v : {c.drift_per_second, c.drift_per_meter, c.angle_per_second,
                   c.angle_per_meter, c.angle_per_radian})
    if (!std::isfinite(v) || v < 0)
      throw std::invalid_argument("invalid motion recovery growth");
  if (c.base_radius > c.max_radius || c.base_angle > c.max_angle ||
      c.max_angle >= M_PI)
    throw std::invalid_argument("invalid motion recovery envelope");
}
/// 同帧重复观察不累计路程；回退、断流、速度跳变和低质量都会永久作废旧锚。
void MotionRecovery::observe(const Frame &f) {
  if (!validPose(f.odom) || f.degraded || f.stamp_ns <= 0) {
    invalidate("frontend invalid or degraded");
    return;
  }
  if (previous_) {
    const auto &p = *previous_;
    if (p.identity == f.identity && p.sequence == f.sequence &&
        p.stamp_ns == f.stamp_ns)
      return;
    const double dt = (f.stamp_ns - p.stamp_ns) * 1e-9;
    const Pose delta = p.odom.inverse() * f.odom;
    if (p.identity != f.identity || dt <= 0 || dt > config_.max_gap ||
        f.sequence <= p.sequence ||
        delta.translation().norm() > config_.max_speed * dt ||
        angle(delta) > config_.max_angular_speed * dt) {
      invalidate("frontend identity, time or motion discontinuity");
    } else if (anchor_frame_) {
      distance_ += delta.translation().norm();
      rotation_ += angle(delta);
    }
  }
  previous_ = f;
}
/// 更新可信锚时保留 IMU 参考点，不能混用底盘位姿或最新无时间关联的前端状态。
void MotionRecovery::anchor(const Pose &pose, const Frame &f) {
  if (!config_.enabled || !validPose(pose) || f.degraded || !previous_ ||
      previous_->identity != f.identity || previous_->stamp_ns != f.stamp_ns ||
      previous_->sequence != f.sequence)
    return;
  anchor_frame_ = f;
  map_anchor_ = pose;
  distance_ = rotation_ = 0.;
  anchored_at_ = std::chrono::steady_clock::now();
  invalid_reason_.clear();
}
/// 解除约束后只允许新的可信配准结果建立锚。
void MotionRecovery::invalidate(const std::string &reason) {
  anchor_frame_.reset();
  previous_.reset();
  distance_ = rotation_ = 0.;
  invalid_reason_ = reason;
}
/// 最后可信地图位姿乘以前端相对运动，误差随经过时间、累计路程及转角增长。
std::optional<RecoveryRegion> MotionRecovery::predict(const Frame &f) const {
  if (!config_.enabled || !anchor_frame_ || !previous_ || f.degraded ||
      previous_->identity != f.identity || previous_->stamp_ns != f.stamp_ns ||
      previous_->sequence != f.sequence ||
      f.identity != anchor_frame_->identity)
    return std::nullopt;
  const double age = (f.stamp_ns - anchor_frame_->stamp_ns) * 1e-9;
  const double wall = std::chrono::duration<double>(
                          std::chrono::steady_clock::now() - anchored_at_)
                          .count();
  const double radius = config_.base_radius + config_.drift_per_second * age +
                        config_.drift_per_meter * distance_;
  const double angle_limit = config_.base_angle +
                             config_.angle_per_second * age +
                             config_.angle_per_meter * distance_ +
                             config_.angle_per_radian * rotation_;
  if (age < 0 || age > config_.max_age || wall > config_.max_age ||
      radius > config_.max_radius || angle_limit > config_.max_angle)
    return std::nullopt;
  RecoveryRegion region;
  region.pose = map_anchor_ * anchor_frame_->odom.inverse() * f.odom;
  region.radius = radius;
  region.angle = angle_limit;
  return region;
}
/// 诊断区分无可信锚、连续性失效和包络过期，方便解释为什么扩大到全图。
std::string MotionRecovery::reason(const Frame &f) const {
  if (!config_.enabled)
    return "motion recovery disabled";
  if (predict(f))
    return "healthy frontend motion envelope";
  if (!anchor_frame_)
    return invalid_reason_;
  return "motion envelope expired or frontend sample mismatch";
}
} // namespace rm27_backend
