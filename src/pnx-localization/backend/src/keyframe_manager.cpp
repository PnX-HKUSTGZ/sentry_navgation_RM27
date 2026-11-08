#include "rm27_backend/keyframe_manager.hpp"

namespace rm27_backend {
/// 校验运动触发、连续性和噪声参数，保证时间门限及过程方差模型可用。
KeyframeManager::KeyframeManager(KeyframeConfig c, GraphConfig g)
    : config_(c), graph_(g) {
  for (double x :
       {c.distance, c.angle_rad, c.min_interval, c.max_interval, c.max_gap,
        c.max_linear_speed, c.max_angular_speed,
        c.translation_variance_per_second, c.translation_variance_per_meter,
        c.rotation_variance_per_second, c.rotation_variance_per_radian,
        c.degraded_noise_multiplier})
    if (!std::isfinite(x) || x < 0)
      throw std::invalid_argument("invalid keyframe/noise parameter");
  if (c.max_gap <= 0 || c.max_linear_speed <= 0 || c.max_angular_speed <= 0 ||
      c.degraded_noise_multiplier < 1 ||
      c.translation_variance_per_second <= 0 ||
      c.rotation_variance_per_second <= 0 ||
      (c.max_interval > 0 && c.max_interval < c.min_interval))
    throw std::invalid_argument("invalid keyframe limits");
}
/// 开始独立轨迹段并丢弃旧图和活动元数据，保留关键帧 ID 单调性。
/// 保留已接收帧的身份与序号水位，防止估计器重置后重新接纳延迟旧消息。
void KeyframeManager::breakSegment() {
  ++segment_;
  graph_.reset(segment_);
  keyframes_.clear();
  latest_.reset();
  // 估计器失败后仍保留输入身份与序号水位；
  // 避免清空图状态后重新接纳延迟到达的旧帧。
  last_edge_.reset();
  interval_degraded_ = false;
}
/// 按实例、epoch、序号和时间验收输入，并记录区间降级或传输缺帧。
/// 进程/代次变化、长时间间隔或不合理速度会断段；旧实例与同代乱序帧直接拒绝。
Admission KeyframeManager::observe(const Frame &f) {
  if (!validPose(f.odom) || f.stamp_ns < 0 || f.sequence == 0 ||
      f.identity.instance.empty()) {
    reason_ = "invalid frame";
    return Admission::Rejected;
  }
  bool fresh = !previous_ || !latest_;
  if (retired_instances_.count(f.identity.instance)) {
    reason_ = "retired instance";
    return Admission::Rejected;
  }
  if (previous_) {
    if (f.identity.instance == previous_->identity.instance &&
        f.identity.epoch < previous_->identity.epoch) {
      reason_ = "retired epoch";
      return Admission::Rejected;
    }
    if (f.identity != previous_->identity) {
      if (f.identity.instance != previous_->identity.instance) {
        if (retired_instances_.size() >= 64) {
          reason_ = "session limit reached; restart backend";
          return Admission::Rejected;
        }
        retired_instances_.insert(previous_->identity.instance);
      }
      breakSegment();
      fresh = true;
      reason_ = "frontend identity changed";
    } else {
      if (f.sequence <= previous_->sequence ||
          f.stamp_ns <= previous_->stamp_ns) {
        reason_ = "duplicate or out-of-order frame";
        return Admission::Rejected;
      }
      const auto delta = previous_->odom.inverse() * f.odom;
      const double dt = (f.stamp_ns - previous_->stamp_ns) * 1e-9;
      if (f.sequence > previous_->sequence + 1) {
        sequence_gaps_ += f.sequence - previous_->sequence - 1;
        interval_degraded_ = true;
      }
      if (dt > config_.max_gap ||
          delta.translation().norm() / dt > config_.max_linear_speed ||
          angle(delta) / dt > config_.max_angular_speed) {
        breakSegment();
        fresh = true;
        reason_ = "motion/time continuity lost";
      }
    }
  }
  previous_ = f;
  interval_degraded_ = interval_degraded_ || f.degraded;
  if (!fresh)
    reason_ = "accepted";
  return fresh ? Admission::NewSegment : Admission::Accepted;
}
/// 只对最近接纳帧判定：首帧直接建帧，否则先满足最小间隔。
/// 平移、三维转角和最大间隔按“或”触发，全部依据原始前端位姿。
bool KeyframeManager::shouldCreate(const Frame &f) const {
  if (!previous_ || f.identity != previous_->identity ||
      f.sequence != previous_->sequence)
    return false;
  if (!latest_)
    return true;
  const double dt = (f.stamp_ns - latest_->frame.stamp_ns) * 1e-9;
  if (dt < config_.min_interval || dt <= 0)
    return false;
  const Pose delta = latest_->frame.odom.inverse() * f.odom;
  return (config_.distance > 0 &&
          delta.translation().norm() >= config_.distance) ||
         (config_.angle_rad > 0 && angle(delta) >= config_.angle_rad) ||
         (config_.max_interval > 0 && dt >= config_.max_interval);
}
/// 构造首帧先验或相邻里程计增量，并根据区间时间、运动和质量生成过程协方差。
/// 完成图更新后登记关键帧及最近测量；图更新异常由上层捕获并开启新段。
Keyframe KeyframeManager::commit(const Frame &f,
                                 const std::optional<PosePrior> &first_prior) {
  if (!shouldCreate(f))
    throw std::invalid_argument("frame is not an eligible keyframe");
  Keyframe k;
  k.id = next_id_;
  k.segment = segment_;
  k.frame = f;
  std::optional<OdometryMeasurement> edge;
  if (latest_) {
    OdometryMeasurement e;
    e.from = latest_->id;
    e.to = k.id;
    e.relative = latest_->frame.odom.inverse() * f.odom;
    e.seconds = (f.stamp_ns - latest_->frame.stamp_ns) * 1e-9;
    e.distance = e.relative.translation().norm();
    e.angle = angle(e.relative);
    const double scale =
        interval_degraded_ ? config_.degraded_noise_multiplier : 1.;
    e.covariance.setZero();
    e.covariance.diagonal().head<3>().setConstant(
        scale *
        std::max(1e-12,
                 config_.translation_variance_per_second * e.seconds +
                     config_.translation_variance_per_meter * e.distance));
    e.covariance.diagonal().tail<3>().setConstant(
        scale *
        std::max(1e-12, config_.rotation_variance_per_second * e.seconds +
                            config_.rotation_variance_per_radian * e.angle));
    graph_.addOdometry(k, e);
    edge = e;
  } else if (first_prior) {
    graph_.addFirst(k, *first_prior);
  } else {
    graph_.addFirst(k);
  }
  k.estimate = graph_.estimate(k.id);
  if (!validPose(k.estimate))
    throw std::runtime_error("graph returned invalid pose");
  ++next_id_;
  latest_ = k;
  keyframes_.emplace(k.id, k);
  synchronizeEstimates();
  last_edge_ = edge;
  interval_degraded_ = f.degraded;
  return k;
}
/// 从活动图刷新关键帧估计，移除已边缘化元数据，原始前端位姿保持不变。
/// 全图模式节点多，一次取全部估计，避免逐节点回代。
void KeyframeManager::synchronizeEstimates() {
  const auto estimates = graph_.estimates();
  for (auto it = keyframes_.begin(); it != keyframes_.end();) {
    const auto found = estimates.find(it->first);
    if (found == estimates.end())
      it = keyframes_.erase(it);
    else {
      it->second.estimate = found->second;
      ++it;
    }
  }
  if (latest_)
    if (const auto found = estimates.find(latest_->id); found != estimates.end())
      latest_->estimate = found->second;
}
/// 将最近关键帧协方差通过相对运动的逆 Adjoint 传播到当前传感器系。
/// 保留平移与旋转的耦合，再添加区间过程噪声；它是模型预测，不是新的独立测量。
Covariance
KeyframeManager::predictionCovariance(const Frame &f,
                                      const Covariance &anchor) const {
  if (!latest_ || !positiveCovariance(anchor))
    throw std::invalid_argument("invalid covariance anchor");
  const Pose delta = latest_->frame.odom.inverse() * f.odom;
  const double dt = (f.stamp_ns - latest_->frame.stamp_ns) * 1e-9;
  if (dt < 0 || f.identity != latest_->frame.identity)
    throw std::invalid_argument("invalid prediction interval");
  if (dt == 0)
    return anchor;
  const Pose inverse = delta.inverse();
  const auto t = inverse.translation();
  Eigen::Matrix3d skew;
  skew << 0, -t.z(), t.y(), t.z(), 0, -t.x(), -t.y(), t.x(), 0;
  Covariance adj = Covariance::Zero();
  adj.topLeftCorner<3, 3>() = inverse.linear();
  adj.topRightCorner<3, 3>() = skew * inverse.linear();
  adj.bottomRightCorner<3, 3>() = inverse.linear();
  Covariance p = adj * anchor * adj.transpose();
  const double scale =
      interval_degraded_ ? config_.degraded_noise_multiplier : 1.;
  p.diagonal().head<3>().array() +=
      scale *
      (config_.translation_variance_per_second * dt +
       config_.translation_variance_per_meter * delta.translation().norm());
  p.diagonal().tail<3>().array() +=
      scale * (config_.rotation_variance_per_second * dt +
               config_.rotation_variance_per_radian * angle(delta));
  return .5 * (p + p.transpose()).eval();
}
} // 命名空间 rm27_backend
