#include "rm27_backend/localization_controller.hpp"
#include <chrono>
#include <pcl/common/transforms.h>
namespace rm27_backend {
/// 校验恢复源和时间限制后启动单后台线程。
LocalizationController::LocalizationController(ControllerConfig c,
                                               MatcherConfig m,
                                               ScanContextConfig sc,
                                               std::string bundle,
                                               Pose alignment)
    : config_(c), motion_(c.motion) {
  if (c.source_min_frames < 1 || c.source_max_frames < c.source_min_frames ||
      c.source_max_frames > 20 || c.failures_before_lost < 1 ||
      c.degraded_frames_before_suspect < 1 ||
      c.rmse_failures_before_suspect < 1 || !std::isfinite(c.source_span) ||
      c.source_span <= 0 || !std::isfinite(c.result_max_age) ||
      c.result_max_age <= 0 || !std::isfinite(c.input_timeout) ||
      c.input_timeout <= 0)
    throw std::invalid_argument("invalid recovery controller limits");
  for (double value :
       {c.tracking.correction_period, c.tracking.acquisition_period,
        c.tracking.max_translation_correction,
        c.tracking.max_rotation_correction})
    if (!std::isfinite(value) || value <= 0)
      throw std::invalid_argument("invalid tracking limits");
  if (c.tracking.max_conflicts < 1 ||
      !std::isfinite(c.tracking.rejection_probability) ||
      c.tracking.rejection_probability <= 0 ||
      c.tracking.rejection_probability >= 1)
    throw std::invalid_argument("invalid innovation gate");
  for (double value :
       {c.ambiguity_retry_period, c.retry_translation, c.retry_rotation})
    if (!std::isfinite(value) || value <= 0)
      throw std::invalid_argument("invalid recovery retry limits");
  thread_ = std::thread(&LocalizationController::worker, this,
                        std::move(bundle), alignment, m, sc);
}
/// 退出时不再启动排队任务，等待当前有界搜索结束。
LocalizationController::~LocalizationController() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stop_ = true;
    ++cancellation_;
    pending_.reset();
  }
  wake_.notify_all();
  if (thread_.joinable())
    thread_.join();
}
/// 地图加载与几何计算完全在后台，异常转为可诊断失败结果。
void LocalizationController::worker(std::string bundle, Pose alignment,
                                    MatcherConfig m, ScanContextConfig sc) {
  std::unique_ptr<RecoveryEngine> engine;
  try {
    engine = std::make_unique<RecoveryEngine>(bundle, alignment, m, sc,
                                              config_.recovery);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stats_.map_points = engine->size();
      stats_.planar_points = engine->planarPoints();
      background_ = engine->background();
    }
    loaded_ = true;
  } catch (const std::exception &e) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      error_ = e.what();
    }
    failed_ = true;
    idle_.notify_all();
    return;
  }
  idle_.notify_all();
  for (;;) {
    Job job;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      wake_.wait(lock, [&] { return stop_ || pending_.has_value(); });
      if (stop_)
        return;
      job = std::move(*pending_);
      pending_.reset();
      busy_ = true;
    }
    const auto start = std::chrono::steady_clock::now();
    RecoveryEstimate estimate;
    /// 只读取原子版本，不跨线程读取主线程维护的状态机。
    const auto cancelled = [&] {
      return cancellation_.load() != job.cancellation;
    };
    try {
      if (job.recovery)
        estimate =
            engine->recover(*job.cloud, job.frame.odom, job.frames, job.hint,
                            job.near, cancelled, job.hint_limits, job.region);
      else {
        estimate.match =
            engine->track(*job.cloud, job.prediction, job.frames, cancelled);
        estimate.found = estimate.match.valid;
        estimate.reason = estimate.match.reason;
      }
    } catch (const std::exception &e) {
      estimate.reason = e.what();
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stats_.last_ms = std::chrono::duration<double, std::milli>(
                           std::chrono::steady_clock::now() - start)
                           .count();
      stats_.max_ms = std::max(stats_.max_ms, stats_.last_ms);
      stats_.total_ms += stats_.last_ms;
      ++stats_.completed;
      stats_.last_attempts = estimate.attempts;
      if (cancelled())
        ++stats_.cancelled;
      results_.push_back({std::move(job), std::move(estimate)});
      busy_ = false;
    }
    idle_.notify_all();
  }
}
/// 单任务请求队列不允许超过一个未消费结果。
bool LocalizationController::enqueue(Job job) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!ready() || stop_ || busy_ || pending_ || !results_.empty()) {
    ++stats_.busy_skips;
    return false;
  }
  job.cancellation = cancellation_.load();
  pending_ = std::move(job);
  ++stats_.requests;
  wake_.notify_one();
  return true;
}
/// 新轨迹段不能继承旧图锚或旧扫描累积；首次注册保持 INITIALIZING。
bool LocalizationController::syncSegment(std::uint64_t segment) {
  if (segment_ && *segment_ == segment)
    return false;
  if (segment_) {
    state_.lost("frontend continuity changed");
    hint_.reset();
    hint_odom_.reset();
  }
  cancelWork();
  motion_.invalidate("frontend segment changed");
  waiting_hint_ = recovery_paused_ = false;
  retry_pose_.reset();
  segment_ = segment;
  anchored_ = false;
  history_.clear();
  anchor_.reset();
  identity_.reset();
  last_request_ns_ = 0;
  return true;
}
/// 图故障保持最后可信位置作为诊断，禁止继续输出正常定位。
void LocalizationController::resetSegment() {
  cancelWork();
  state_.lost("estimator failure");
  motion_.invalidate("graph estimator failure");
  anchored_ = false;
  anchor_.reset();
  history_.clear();
}
/// 有界按源时间累计；所有点云保持各自局部 IMU 坐标。
void LocalizationController::observe(const Frame &f, Cloud::ConstPtr cloud) {
  motion_.observe(f);
  if (!history_.empty() && history_.back().first.identity == f.identity &&
      history_.back().first.stamp_ns == f.stamp_ns &&
      history_.back().first.sequence == f.sequence) {
    history_.back().second = std::move(cloud);
    return;
  }
  timeout_ = false;
  degraded_frames_ = f.degraded ? degraded_frames_ + 1 : 0;
  if (degraded_frames_ >= config_.degraded_frames_before_suspect)
    state_.suspect("persistent frontend quality degradation");
  identity_ = f.identity;
  history_.emplace_back(f, std::move(cloud));
  while (history_.size() > std::size_t(config_.source_max_frames) ||
         (!history_.empty() &&
          (f.stamp_ns - history_.front().first.stamp_ns) * 1e-9 >
              config_.source_span))
    history_.pop_front();
}
/// 从同代近期帧累积恢复源；配准期间允许后续帧继续进入。
bool LocalizationController::requestRecovery(const Frame &f) {
  if (f.degraded || waiting_hint_ || recovery_paused_ ||
      degraded_frames_ >= config_.degraded_frames_before_suspect ||
      state_.trusted() || !ready() || !segment_ || anchor_ ||
      history_.size() < std::size_t(config_.source_min_frames))
    return false;
  if (last_request_ns_ && (f.stamp_ns - last_request_ns_) * 1e-9 <
                              config_.tracking.acquisition_period)
    return false;
  if (retry_pose_ && std::chrono::steady_clock::now() < retry_after_) {
    const Pose delta = retry_pose_->inverse() * f.odom;
    if (delta.translation().norm() < config_.retry_translation &&
        angle(delta) < config_.retry_rotation)
      return false;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (busy_ || pending_ || !results_.empty()) {
      ++stats_.busy_skips;
      return false;
    }
  }
  auto source = std::make_shared<Cloud>();
  int count = 0;
  for (const auto &[frame, cloud] : history_) {
    if (frame.identity != f.identity || frame.degraded)
      continue;
    Cloud moved;
    pcl::transformPointCloud(
        *cloud, moved, (f.odom.inverse() * frame.odom).matrix().cast<float>());
    *source += moved;
    ++count;
  }
  if (count < config_.source_min_frames)
    return false;
  Job job;
  job.frame = f;
  job.cloud = source;
  job.frames = count;
  job.recovery = true;
  job.generation = state_.generation();
  job.segment = *segment_;
  job.hint = hint_;
  job.hint_limits = hint_limits_;
  if (hint_ && hint_odom_)
    job.hint = *hint_ * hint_odom_->inverse() * f.odom;
  // 提示来自人工或启动入口时由其独立负责；自动恢复只使用健康连续性。
  if (!hint_)
    job.region = motion_.predict(f);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stats_.motion_recovery_active = job.region.has_value();
    stats_.motion_recovery_reason = motion_.reason(f);
    stats_.motion_recovery_radius = job.region ? job.region->radius : 0.;
    stats_.motion_recovery_angle = job.region ? job.region->angle : 0.;
    if (!job.hint && !job.region && !manual_global_recovery_ &&
        !config_.allow_unbounded_auto_recovery) {
      recovery_blocked_reason_ = "unbounded automatic recovery blocked: " +
                                 stats_.motion_recovery_reason;
      return false;
    }
    recovery_blocked_reason_.clear();
  }
  if (!enqueue(std::move(job)))
    return false;
  last_request_ns_ = f.stamp_ns;
  return true;
}
/// 可信状态才允许普通地图跟踪，原子任务身份与当前状态一致。
bool LocalizationController::requestTracking(const Keyframe &k,
                                             const Pose &prediction,
                                             Cloud::ConstPtr cloud,
                                             int frames) {
  if (!state_.trusted() || !anchored_)
    return false;
  if (last_request_ns_ && (k.frame.stamp_ns - last_request_ns_) * 1e-9 <
                              config_.tracking.correction_period)
    return false;
  Job j;
  j.frame = k.frame;
  j.key = k.id;
  j.cloud = std::move(cloud);
  j.prediction = prediction;
  j.frames = frames;
  j.generation = state_.generation();
  j.segment = k.segment;
  if (!enqueue(std::move(j)))
    return false;
  last_request_ns_ = k.frame.stamp_ns;
  return true;
}
/// 提交前再次核对代次、段号、源身份及新鲜度，失败事务不修改可信状态。
ApplyOutcome LocalizationController::apply(KeyframeManager &manager) {
  ApplyOutcome out;
  std::deque<Result> results;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    results.swap(results_);
  }
  for (auto &r : results) {
    const bool stale =
        r.job.cancellation != cancellation_.load() || !segment_ ||
        r.job.segment != *segment_ || r.job.generation != state_.generation() ||
        !identity_ || r.job.frame.identity != *identity_ || history_.empty() ||
        (history_.back().first.stamp_ns - r.job.frame.stamp_ns) * 1e-9 >
            config_.result_max_age;
    if (stale) {
      std::lock_guard<std::mutex> lock(mutex_);
      ++stats_.stale;
      continue;
    }
    if (r.job.recovery) {
      // 在途恢复期间连续性可能失效，不能仅凭发起任务时的有效范围提交结果。
      if (r.job.region && !motion_.predict(history_.back().first)) {
        std::lock_guard<std::mutex> lock(mutex_);
        ++stats_.stale;
        continue;
      }
      if (r.estimate.deferred)
        continue;
      if (!r.estimate.found) {
        state_.recoveryFailed(config_.failures_before_lost, r.estimate.reason);
        std::lock_guard<std::mutex> lock(mutex_);
        ++stats_.acquisition_failures;
        if (r.estimate.reason.find("ambiguous") != std::string::npos) {
          retry_pose_ = r.job.frame.odom;
          retry_after_ =
              std::chrono::steady_clock::now() +
              std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                  std::chrono::duration<double>(
                      config_.ambiguity_retry_period));
        }
        continue;
      }
      // 恢复后建立独立图分量，不跨不可信区间补里程计边。
      manager.breakSegment();
      segment_ = manager.segment();
      anchored_ = false;
      anchor_ = std::move(r);
      continue;
    }
    if (!state_.trusted() || !manager.graph().contains(r.job.key))
      continue;
    if (!r.estimate.found) {
      std::lock_guard<std::mutex> lock(mutex_);
      if (r.estimate.match.reason == "point-to-plane RMSE above threshold" &&
          ++stats_.consecutive_rmse_failures >=
              static_cast<std::uint64_t>(config_.rmse_failures_before_suspect))
        state_.suspect("tracking residual outside valid basin");
      ++stats_.quality_rejects;
      continue;
    }
    const auto &m = r.estimate.match;
    AbsolutePoseMeasurement measurement;
    measurement.measurement_id = "map:" + std::to_string(r.job.generation) +
                                 ":" + std::to_string(r.job.key);
    measurement.key = r.job.key;
    measurement.segment = manager.segment();
    measurement.pose = m.pose;
    measurement.sqrt_information = m.sqrt_information;
    const Pose delta = manager.graph().estimate(r.job.key).inverse() * m.pose;
    if (delta.translation().norm() >
            config_.tracking.max_translation_correction ||
        angle(delta) > config_.tracking.max_rotation_correction) {
      state_.suspect("map correction outside tracking basin");
      {
        std::lock_guard<std::mutex> lock(mutex_);
        ++stats_.correction_rejects;
      }
      continue;
    }
    try {
      const auto innovation = manager.graph().assessAbsolute(measurement);
      {
        std::lock_guard<std::mutex> lock(mutex_);
        stats_.last_nis = innovation.nis;
        stats_.last_rmse = m.rmse;
        stats_.last_rank = m.rank;
        stats_.last_inlier_ratio = m.inlier_ratio;
        stats_.last_correction = delta.translation().norm();
      }
      if (!innovation.valid ||
          innovation.probability < config_.tracking.rejection_probability) {
        state_.conflict(config_.tracking.max_conflicts,
                        "map measurement conflicts with graph");
        std::lock_guard<std::mutex> lock(mutex_);
        ++stats_.nis_rejects;
        ++stats_.consecutive_conflicts;
        continue;
      }
      if (manager.graph().addAbsolute(measurement)) {
        manager.synchronizeEstimates();
        state_.accepted(false, "map tracking accepted");
        ++out.accepted;
        std::lock_guard<std::mutex> lock(mutex_);
        ++stats_.accepted;
        stats_.consecutive_conflicts = 0;
        stats_.consecutive_rmse_failures = 0;
      }
    } catch (const std::exception &e) {
      state_.lost(e.what());
      out.graph_failure = true;
    }
  }
  return out;
}
/// 用恢复源帧到最新帧的里程计增量传播先验，保留右扰动交叉项。
std::optional<PosePrior> LocalizationController::takeAnchor(const Frame &f) {
  if (!anchor_)
    return std::nullopt;
  const auto &r = *anchor_;
  if (f.identity != r.job.frame.identity || f.stamp_ns < r.job.frame.stamp_ns ||
      (f.stamp_ns - r.job.frame.stamp_ns) * 1e-9 > config_.result_max_age) {
    anchor_.reset();
    return std::nullopt;
  }
  const auto covariance =
      covarianceFromSqrtInformation(r.estimate.match.sqrt_information);
  if (!covariance) {
    anchor_.reset();
    return std::nullopt;
  }
  const Pose delta = r.job.frame.odom.inverse() * f.odom;
  // 由现有关键帧预测合同传播协方差；恢复跨帧的运动不确定度按时间保守补入。
  const Pose inv = delta.inverse();
  Covariance adj = Covariance::Zero();
  const Eigen::Vector3d t = inv.translation();
  Eigen::Matrix3d skew;
  skew << 0, -t.z(), t.y(), t.z(), 0, -t.x(), -t.y(), t.x(), 0;
  adj.topLeftCorner<3, 3>() = inv.linear();
  adj.bottomRightCorner<3, 3>() = inv.linear();
  adj.topRightCorner<3, 3>() = skew * inv.linear();
  Covariance p = adj * (*covariance) * adj.transpose();
  const double dt = (f.stamp_ns - r.job.frame.stamp_ns) * 1e-9;
  p.diagonal().head<3>().array() += .002 * dt;
  p.diagonal().tail<3>().array() += .0001 * dt;
  return PosePrior{r.estimate.match.pose * delta, p};
}
/// 首帧图成功提交后才宣布恢复成功。
void LocalizationController::confirmAnchor() {
  if (!anchor_)
    return;
  state_.accepted(true, anchor_->estimate.reason);
  anchor_.reset();
  hint_.reset();
  hint_odom_.reset();
  anchored_ = true;
  manual_global_recovery_ = false;
  std::lock_guard<std::mutex> lock(mutex_);
  stats_.consecutive_rmse_failures = 0;
  recovery_blocked_reason_.clear();
  if (stats_.anchors > 0)
    ++stats_.recoveries;
  ++stats_.anchors;
  stats_.consecutive_conflicts = 0;
}
/// 仅可信输出能更新附近搜索中心。
void LocalizationController::rememberPose(const Pose &pose,
                                          std::int64_t stamp) {
  if (!state_.trusted())
    return;
  for (auto it = history_.rbegin(); it != history_.rend(); ++it) {
    if (it->first.stamp_ns == stamp && identity_ &&
        it->first.identity == *identity_) {
      // 人工提示取消了旧锚，但缓存同刻原子帧仍可作为新可信锚的参考。
      if (it == history_.rbegin())
        motion_.observe(it->first);
      motion_.anchor(pose, it->first);
      return;
    }
  }
}
/// 提示和无提示请求都建立独立恢复代次。
void LocalizationController::manual(const std::optional<Pose> &hint,
                                    const std::optional<Pose> &hint_odom,
                                    bool wait_for_hint) {
  cancelWork();
  if (hint || wait_for_hint)
    motion_.invalidate("explicit manual pose overrides continuity");
  waiting_hint_ = wait_for_hint;
  recovery_paused_ = false;
  manual_global_recovery_ = !hint && !wait_for_hint;
  retry_pose_.reset();
  hint_limits_ = HintLimits{};
  state_.requestRecovery();
  hint_ = hint;
  hint_odom_ = hint_odom;
  anchor_.reset();
  last_request_ns_ = 0;
}
/// 取消几何任务时只更新原子身份，配准在迭代边界自行退出。
void LocalizationController::cancelWork() {
  ++cancellation_;
  manual_global_recovery_ = false;
  std::lock_guard<std::mutex> lock(mutex_);
  stats_.consecutive_rmse_failures = 0;
  recovery_blocked_reason_.clear();
  stats_.stale += results_.size();
  pending_.reset();
  results_.clear();
}
/// 同刻提示到齐后立即使用已缓存的有效扫描，不等待固定采集周期。
void LocalizationController::updateHint(const Pose &hint, const Pose &hint_odom,
                                        HintLimits limits) {
  if (!validPose(hint) || !validPose(hint_odom) ||
      !std::isfinite(limits.translation) || !std::isfinite(limits.rotation) ||
      limits.translation <= 0 || limits.rotation <= 0)
    throw std::invalid_argument("invalid manual recovery hint");
  cancelWork();
  motion_.invalidate("explicit manual pose overrides continuity");
  hint_ = hint;
  hint_odom_ = hint_odom;
  hint_limits_ = limits;
  waiting_hint_ = false;
  last_request_ns_ = 0;
}
/// 截止后保持未定位；重新点击会显式解除暂停。
void LocalizationController::cancelRecovery(const std::string &reason) {
  cancelWork();
  waiting_hint_ = false;
  recovery_paused_ = true;
  anchor_.reset();
  state_.lost(reason);
}
/// 超时不可因定时器反复调用而无限递增工作身份。
void LocalizationController::inputTimedOut() {
  if (!timeout_) {
    cancelWork();
    state_.lost("frontend input timeout");
    motion_.invalidate("frontend input timed out");
    anchor_.reset();
    history_.clear();
    timeout_ = true;
  }
}
/// 背景构造后不可变，短锁复制共享指针。
Cloud::ConstPtr LocalizationController::background() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return background_;
}
/// 复制线程共享统计。
LocalizerStats LocalizationController::stats() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return stats_;
}
/// 读取加载失败或状态机原因。
std::string LocalizationController::reason() const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!error_.empty())
    return error_;
  return recovery_blocked_reason_.empty() ? state_.reason()
                                          : recovery_blocked_reason_;
}
/// 测试等待加载和当前任务完成。
void LocalizationController::waitIdle() const {
  std::unique_lock<std::mutex> lock(mutex_);
  idle_.wait(lock, [&] { return failed_ || (loaded_ && !busy_ && !pending_); });
}
} // namespace rm27_backend
