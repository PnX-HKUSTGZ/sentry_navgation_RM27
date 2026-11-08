#include "rm27_backend/map_localizer.hpp"
#include <climits>
#include <cmath>

namespace rm27_backend {
namespace {
/// 判断相对位姿是否超过平移或旋转上限；上限为 0 表示不限制。
bool exceeds(const Pose &delta, double translation, double rotation) {
  return (translation > 0 && delta.translation().norm() > translation) ||
         (rotation > 0 && angle(delta) > rotation);
}
} // 匿名命名空间
/// 状态枚举到诊断字符串。
const char *toString(MapState state) {
  switch (state) {
  case MapState::Loading:
    return "loading";
  case MapState::Failed:
    return "failed";
  case MapState::Waiting:
    return "waiting_anchor";
  case MapState::Tracking:
    return "tracking";
  case MapState::Suspect:
    return "suspect";
  }
  return "unknown";
}
/// 校验调度和门限参数后启动后台线程。
MapLocalizer::MapLocalizer(LocalizerConfig c, MatcherConfig m, std::string path,
                           Pose transform)
    : config_(c), matcher_(m) {
  for (double x : {c.correction_period, c.acquisition_period,
                   c.source_max_distance, c.max_translation_correction,
                   c.max_rotation_correction, c.acquisition_max_translation,
                   c.acquisition_max_rotation, c.recovery_translation,
                   c.recovery_rotation, c.anchor_max_age, c.continuity_timeout})
    if (!std::isfinite(x) || x < 0)
      throw std::invalid_argument("invalid prior map localizer parameter");
  if (c.source_keyframes < 1 || c.max_conflicts < 1 ||
      !std::isfinite(c.rejection_probability) || c.rejection_probability <= 0 ||
      c.rejection_probability >= 1)
    throw std::invalid_argument("invalid prior map localizer limits");
  thread_ = std::thread(&MapLocalizer::worker, this, std::move(path), transform);
}
/// 通知后台退出并回收线程。
MapLocalizer::~MapLocalizer() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stop_ = true;
  }
  wake_.notify_all();
  if (thread_.joinable())
    thread_.join();
}
/// 地图加载成功后状态离开 Loading/Failed。
bool MapLocalizer::ready() const {
  const auto s = state();
  return s != MapState::Loading && s != MapState::Failed;
}
/// 加载失败只影响地图因子，调用方应退回纯里程计图；随后按单槽队列串行配准。
void MapLocalizer::worker(std::string path, Pose transform) {
  try {
    auto map = PriorMap::load(path, transform, matcher_);
    std::lock_guard<std::mutex> lock(mutex_);
    map_ = map;
    stats_.map_points = map->size();
    stats_.planar_points = map->planarPoints();
    reason_ = "waiting for anchor";
    state_ = MapState::Waiting;
  } catch (const std::exception &e) {
    std::lock_guard<std::mutex> lock(mutex_);
    reason_ = e.what();
    state_ = MapState::Failed;
    idle_.notify_all();
    return;
  }
  idle_.notify_all();
  while (true) {
    Job job;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      wake_.wait(lock, [&] { return stop_ || pending_; });
      if (stop_)
        return;
      job = std::move(*pending_);
      pending_.reset();
      busy_ = true;
    }
    MatchResult match;
    try {
      match = map_->match(*job.cloud, job.initial, job.frames);
    } catch (const std::exception &e) {
      match.reason = e.what();
    }
    std::lock_guard<std::mutex> lock(mutex_);
    results_.push_back({std::move(job), std::move(match)});
    while (results_.size() > 4)
      results_.pop_front();
    busy_ = false;
    idle_.notify_all();
  }
}
/// 单任务：后台正在计算或已有待处理任务时跳过，保证延迟和 CPU 有界。
bool MapLocalizer::enqueue(Job job) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!map_ || busy_ || pending_) {
      ++stats_.busy_skips;
      return false;
    }
    pending_ = std::move(job);
    ++stats_.requests;
  }
  wake_.notify_one();
  return true;
}
/// 递增代次使在途结果失效，清除锚定和恢复候选；已加载地图保持可用。
void MapLocalizer::resetSegment() {
  ++generation_;
  anchored_ = false;
  anchor_.reset();
  recovery_candidate_.reset();
  last_acquisition_ns_ = last_track_ns_ = INT64_MIN / 2;
  std::lock_guard<std::mutex> lock(mutex_);
  pending_.reset();
  stats_.consecutive_conflicts = 0;
  if (map_) {
    state_ = MapState::Waiting;
    reason_ = "segment changed; waiting for anchor";
  }
}
/// 记录已同步的段号，段号变化时才使锚定和在途结果失效。
bool MapLocalizer::syncSegment(std::uint64_t segment) {
  if (segment_ && *segment_ == segment)
    return false;
  const bool changed = segment_.has_value();
  segment_ = segment;
  if (changed)
    resetSegment();
  return changed;
}
/// 保存最近输出的 map 位姿和时间戳。
void MapLocalizer::rememberPose(const Pose &pose, std::int64_t stamp) {
  last_pose_ = std::make_pair(pose, stamp);
}
/// 断段后若时差在容许范围内，机器人位置近似连续，可作为新段锚定初值。
std::optional<Pose> MapLocalizer::continuitySeed(std::int64_t stamp) const {
  if (!last_pose_ || stamp < last_pose_->second ||
      (stamp - last_pose_->second) * 1e-9 > config_.continuity_timeout)
    return std::nullopt;
  return last_pose_->first;
}
/// 按锚定周期提交当前帧点云和 map←IMU 种子。
bool MapLocalizer::requestAcquisition(const Frame &f, Cloud::ConstPtr cloud,
                                      const Pose &seed) {
  if (anchored_ || !cloud || !validPose(seed) ||
      (f.stamp_ns - last_acquisition_ns_) * 1e-9 < config_.acquisition_period)
    return false;
  Job job;
  job.kind = Kind::Acquire;
  job.generation = generation_;
  job.frame = f;
  job.initial = seed;
  job.cloud = std::move(cloud);
  if (!enqueue(std::move(job)))
    return false;
  last_acquisition_ns_ = f.stamp_ns;
  return true;
}
/// 用同一前端身份内的原始里程计增量把锚定位姿外推到本帧，协方差沿用配准结果。
std::optional<PosePrior> MapLocalizer::takeAnchor(const Frame &f) {
  if (anchored_ || !anchor_)
    return std::nullopt;
  const auto &a = *anchor_;
  const double age = (f.stamp_ns - a.job.frame.stamp_ns) * 1e-9;
  if (f.identity != a.job.frame.identity || age < 0 ||
      age > config_.anchor_max_age) {
    anchor_.reset();
    return std::nullopt;
  }
  const auto covariance = covarianceFromSqrtInformation(a.match.sqrt_information);
  if (!covariance) {
    anchor_.reset();
    return std::nullopt;
  }
  PosePrior prior;
  prior.pose = a.match.pose * a.job.frame.odom.inverse() * f.odom;
  prior.covariance = *covariance;
  anchor_.reset();
  anchored_ = true;
  state_ = MapState::Tracking;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    ++stats_.anchors;
    reason_ = "anchored to prior map";
  }
  return prior;
}
/// 按校正周期为新关键帧排队；初值取当前图估计，避免使用漂移的原始里程计。
bool MapLocalizer::requestTracking(const Keyframe &k, const Pose &predicted,
                                   Cloud::ConstPtr source, int frames) {
  if (!anchored_ || !source || frames < 1 ||
      (k.frame.stamp_ns - last_track_ns_) * 1e-9 < config_.correction_period)
    return false;
  Job job;
  job.kind = Kind::Track;
  job.generation = generation_;
  job.key = k.id;
  job.frame = k.frame;
  job.initial = predicted;
  job.cloud = std::move(source);
  job.frames = frames;
  if (!enqueue(std::move(job)))
    return false;
  last_track_ns_ = k.frame.stamp_ns;
  return true;
}
/// 冲突计数达到门限后进入 Suspect，此后需要两次一致的满秩结果才恢复。
void MapLocalizer::conflict(const std::string &why) {
  std::lock_guard<std::mutex> lock(mutex_);
  ++stats_.consecutive_conflicts;
  reason_ = why;
  if (stats_.consecutive_conflicts >= std::uint64_t(config_.max_conflicts) &&
      state_ == MapState::Tracking) {
    state_ = MapState::Suspect;
    reason_ = "repeated prior map conflicts; waiting for consistent recovery";
  }
}
/// 依次处理后台结果。常规跟踪：质量 → 校正上限 → NIS；Suspect：满秩且连续两次
/// 推得的 map←world 对齐一致才跳过 NIS 提交，用于纠正已偏离地图的图估计。
ApplyOutcome MapLocalizer::apply(KeyframeManager &manager) {
  ApplyOutcome outcome;
  std::deque<Result> results;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    results.swap(results_);
  }
  for (auto &r : results) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      ++stats_.completed;
      stats_.last_ms = r.match.milliseconds;
      stats_.max_ms = std::max(stats_.max_ms, r.match.milliseconds);
      stats_.total_ms += r.match.milliseconds;
      stats_.last_rmse = r.match.rmse;
      stats_.last_inlier_ratio = r.match.inlier_ratio;
      stats_.last_rank = r.match.rank;
      if (r.job.generation != generation_) {
        ++stats_.stale;
        continue;
      }
    }
    if (r.job.kind == Kind::Acquire) {
      std::string why;
      if (!r.match.valid)
        why = "anchor rejected: " + r.match.reason;
      else if (r.match.rank < 6)
        why = "anchor rejected: registration is not full rank";
      else if (exceeds(r.job.initial.inverse() * r.match.pose,
                       config_.acquisition_max_translation,
                       config_.acquisition_max_rotation))
        why = "anchor rejected: too far from seed";
      std::lock_guard<std::mutex> lock(mutex_);
      if (!why.empty()) {
        ++stats_.acquisition_failures;
        reason_ = why;
      } else {
        anchor_ = std::move(r);
      }
      continue;
    }
    PoseGraph &graph = manager.graph();
    if (!anchored_ || !graph.contains(r.job.key)) {
      std::lock_guard<std::mutex> lock(mutex_);
      ++stats_.stale;
      continue;
    }
    if (!r.match.valid) {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        ++stats_.quality_rejects;
      }
      conflict("map match rejected: " + r.match.reason);
      continue;
    }
    AbsolutePoseMeasurement m;
    m.measurement_id = "map:" + std::to_string(generation_) + ":" +
                       std::to_string(r.job.key);
    m.key = r.job.key;
    m.segment = manager.segment();
    m.pose = r.match.pose;
    m.sqrt_information = r.match.sqrt_information;
    const Pose correction = graph.estimate(r.job.key).inverse() * m.pose;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stats_.last_correction = correction.translation().norm();
    }
    bool recovered = false;
    if (state() == MapState::Suspect) {
      if (r.match.rank < 6)
        continue;
      const Pose alignment = m.pose * r.job.frame.odom.inverse();
      if (!recovery_candidate_ ||
          exceeds(recovery_candidate_->inverse() * alignment,
                  config_.recovery_translation, config_.recovery_rotation)) {
        recovery_candidate_ = alignment;
        continue;
      }
      recovered = true;
    } else {
      if (exceeds(correction, config_.max_translation_correction,
                  config_.max_rotation_correction)) {
        {
          std::lock_guard<std::mutex> lock(mutex_);
          ++stats_.correction_rejects;
        }
        conflict("map correction exceeds limit");
        continue;
      }
      const Innovation innovation = graph.assessAbsolute(m);
      {
        std::lock_guard<std::mutex> lock(mutex_);
        stats_.last_nis = innovation.nis;
      }
      if (!innovation.valid ||
          innovation.probability < config_.rejection_probability) {
        {
          std::lock_guard<std::mutex> lock(mutex_);
          ++stats_.nis_rejects;
        }
        conflict("map innovation rejected by NIS gate");
        continue;
      }
    }
    try {
      if (!graph.addAbsolute(m)) {
        std::lock_guard<std::mutex> lock(mutex_);
        ++stats_.stale;
        continue;
      }
      manager.synchronizeEstimates();
    } catch (const std::exception &e) {
      std::lock_guard<std::mutex> lock(mutex_);
      reason_ = std::string("graph update failed: ") + e.what();
      outcome.graph_failure = true;
      return outcome;
    }
    ++outcome.accepted;
    recovery_candidate_.reset();
    std::lock_guard<std::mutex> lock(mutex_);
    ++stats_.accepted;
    stats_.consecutive_conflicts = 0;
    if (recovered) {
      ++stats_.recoveries;
      reason_ = "recovered from consistent prior map matches";
    } else {
      reason_ = "tracking prior map";
    }
    state_ = MapState::Tracking;
  }
  return outcome;
}
/// 在锁内复制统计。
LocalizerStats MapLocalizer::stats() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return stats_;
}
/// 在锁内复制原因字符串。
std::string MapLocalizer::reason() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return reason_;
}
/// 等待后台加载完成且队列为空。
void MapLocalizer::waitIdle() const {
  std::unique_lock<std::mutex> lock(mutex_);
  idle_.wait(lock, [&] {
    return state_ == MapState::Failed ||
           (map_ && !busy_ && !pending_);
  });
}
} // 命名空间 rm27_backend
