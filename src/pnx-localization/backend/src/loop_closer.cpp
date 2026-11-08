#include "rm27_backend/loop_closer.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <tbb/task_arena.h>

namespace rm27_backend {
namespace {
/// 角度归一化到 (−π, π]。
double normalize(double a) { return std::atan2(std::sin(a), std::cos(a)); }
/// 旋转残差的 roll/pitch 绝对值较大者。
double maxRollPitch(const Eigen::Matrix3d &r) {
  const double roll = std::atan2(r(2, 1), r(2, 2));
  const double pitch = std::asin(std::clamp(-r(2, 0), -1., 1.));
  return std::max(std::abs(roll), std::abs(pitch));
}
/// 判断相对位姿是否在平移和旋转容差内。
bool within(const Pose &residual, double translation, double rotation) {
  return residual.translation().norm() <= translation && angle(residual) <= rotation;
}
} // 匿名命名空间
/// 校验门限并启动后台线程。
LoopCloser::LoopCloser(LoopConfig c, ScanContextConfig d, MatcherConfig m)
    : config_(c), matcher_(m), database_(d) {
  for (double x : {c.target_radius, c.candidate_max_distance, c.query_distance,
                   c.query_interval,
                   c.descriptor_ratio_threshold, c.cloud_voxel, c.max_translation,
                   c.max_z, c.max_roll_pitch, c.max_yaw_residual, c.min_overlap,
                   c.pcm_translation, c.pcm_rotation})
    if (!std::isfinite(x) || x < 0)
      throw std::invalid_argument("invalid loop closure parameter");
  if (c.target_keyframes < 1 || c.source_keyframes < 1 || c.max_keyframes < 2 || c.threads < 0 ||
      c.cooldown_keyframes < 0 || c.min_clique < 1 || c.cloud_voxel <= 0 ||
      !std::isfinite(c.rejection_probability) || c.rejection_probability <= 0 ||
      c.rejection_probability >= 1)
    throw std::invalid_argument("invalid loop closure limits");
  thread_ = std::thread(&LoopCloser::worker, this);
}
/// 通知后台退出并回收线程。
LoopCloser::~LoopCloser() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stop_ = true;
  }
  wake_.notify_all();
  if (thread_.joinable())
    thread_.join();
}
/// 段号变化说明图已重建，旧关键帧不能再作为回环端点。
bool LoopCloser::syncSegment(std::uint64_t segment) {
  if (segment_ && *segment_ == segment)
    return false;
  const bool changed = segment_.has_value();
  segment_ = segment;
  if (!changed)
    return false;
  ++generation_;
  database_.clear();
  entries_.clear();
  accepted_.clear();
  records_.clear();
  last_checked_ = last_accepted_ = -1;
  last_query_travel_ = -1;
  last_query_stamp_ = 0;
  std::lock_guard<std::mutex> lock(mutex_);
  pending_.reset();
  stats_.keyframes = stats_.descriptor_bytes = stats_.cloud_bytes = 0;
  reason_ = "segment changed; loop database cleared";
  return true;
}
/// 描述子使用整平后的单帧点云；回环库另存体素降采样点云供后续配准。
void LoopCloser::addKeyframe(const Keyframe &k, const Cloud &cloud) {
  if (entries_.size() >= config_.max_keyframes) {
    std::lock_guard<std::mutex> lock(mutex_);
    reason_ = "loop database reached max_keyframes";
    return;
  }
  Cloud leveled;
  pcl::transformPointCloud(cloud, leveled,
                           levelFromBody(k.frame.odom).matrix().cast<float>());
  database_.add(leveled);
  auto sampled = std::make_shared<Cloud>();
  pcl::VoxelGrid<pcl::PointXYZI> grid;
  grid.setInputCloud(cloud.makeShared());
  const float leaf = float(config_.cloud_voxel);
  grid.setLeafSize(leaf, leaf, leaf);
  grid.filter(*sampled);
  const double travel =
      entries_.empty() ? 0.
                       : entries_.back().travel +
                             (k.frame.odom.translation() - entries_.back().odom.translation()).norm();
  entries_.push_back({k.id, k.frame.stamp_ns, k.frame.odom, travel, sampled});
  const auto &d = database_.config();
  std::lock_guard<std::mutex> lock(mutex_);
  stats_.keyframes = entries_.size();
  stats_.descriptor_bytes = entries_.size() * std::size_t(d.augment_count) *
                            std::size_t(d.num_rings * d.num_sectors) * sizeof(float);
  stats_.cloud_bytes += sampled->size() * sizeof(pcl::PointXYZI);
}
/// 照搬 z-localization 的 captureLoop：单任务、冷却、去重、空间预筛与描述子检索，
/// 再冻结候选目标子图、源帧和已接受回环，后台计算期间不再访问可变状态。
bool LoopCloser::request(const KeyframeManager &manager) {
  if (entries_.size() < 2)
    return false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (busy_ || pending_) {
      ++stats_.busy_skips;
      return false;
    }
  }
  const int latest = int(entries_.size()) - 1;
  if (latest == last_checked_ ||
      (last_accepted_ >= 0 && latest - last_accepted_ < config_.cooldown_keyframes))
    return false;
  // 按行程限频：静止或低速时不在同一处密集检索，避免相关回环被当作独立信息反复叠加；
  // 行程不足时再按时间间隔低频检索，使静止期间的里程计漂移仍可被回环约束。
  const double travel = entries_[latest].travel;
  if (last_query_travel_ >= 0 && travel - last_query_travel_ < config_.query_distance &&
      (config_.query_interval <= 0 ||
       (entries_[latest].stamp_ns - last_query_stamp_) * 1e-9 < config_.query_interval))
    return false;
  const auto &active = manager.keyframes();
  /// 读取条目在当前图中的估计；已边缘化或不存在时为空。
  const auto estimate = [&](const Entry &e) -> std::optional<Pose> {
    const auto it = active.find(e.id);
    if (it == active.end())
      return std::nullopt;
    return it->second.estimate;
  };
  const auto latest_estimate = estimate(entries_[latest]);
  if (!latest_estimate)
    return false;
  std::vector<int> eligible;
  std::vector<std::optional<Pose>> estimates(entries_.size());
  for (int i = 0; i < latest; ++i) {
    estimates[i] = estimate(entries_[i]);
    if (estimates[i] &&
        (config_.candidate_max_distance <= 0 ||
         (estimates[i]->translation() - latest_estimate->translation()).norm() <=
             config_.candidate_max_distance))
      eligible.push_back(i);
  }
  auto candidates = database_.detectLatest(eligible);
  last_checked_ = latest;
  last_query_travel_ = travel;
  last_query_stamp_ = entries_[latest].stamp_ns;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    ++stats_.queries;
    stats_.candidates += candidates.size();
  }
  if (candidates.empty())
    return false;
  if (config_.descriptor_ratio_threshold < 1 && candidates.size() > 1 &&
      candidates[0].distance / std::max(1e-9, candidates[1].distance) >
          config_.descriptor_ratio_threshold)
    return false;
  Job job;
  job.generation = generation_;
  job.latest_index = latest;
  job.latest = entries_[latest];
  job.latest_estimate = *latest_estimate;
  for (int i = latest - 1; i >= 0 && int(job.source.size()) + 1 < config_.source_keyframes; --i)
    job.source.push_back(entries_[i]);
  for (const auto &candidate : candidates) {
    Target target;
    target.descriptor = candidate;
    target.entry = entries_[candidate.index];
    target.estimate = *estimates[candidate.index];
    std::vector<std::pair<double, int>> neighbors;
    const Pose center = target.estimate.inverse();
    if (config_.target_same_pass) {
      // 同一趟子图：只取候选前后相邻关键帧，用原始里程计拼接；局部里程计一致性高，
      // 避免不同趟之间尚未校正的偏差把子图拼花。上界不越过检索排除区，防止与源重叠。
      const int upper = latest - database_.config().exclude_recent;
      const Pose reference = target.entry.odom.inverse();
      std::vector<int> order{candidate.index};
      for (int o = 1; o <= config_.target_keyframes; ++o) {
        order.push_back(candidate.index - o);
        order.push_back(candidate.index + o);
      }
      for (const int j : order) {
        if (int(target.submap.size()) >= config_.target_keyframes)
          break;
        if (j < 0 || j > upper)
          continue;
        const Pose relative = reference * entries_[j].odom;
        if (config_.target_radius <= 0 || relative.translation().norm() <= config_.target_radius)
          target.submap.emplace_back(entries_[j].cloud, relative);
      }
      job.targets.push_back(std::move(target));
      continue;
    }
    for (int j = 0; j <= candidate.index; ++j)
      if (estimates[j]) {
        const double d =
            (estimates[j]->translation() - target.estimate.translation()).norm();
        if (config_.target_radius <= 0 || d <= config_.target_radius)
          neighbors.emplace_back(d, j);
      }
    std::sort(neighbors.begin(), neighbors.end());
    if (int(neighbors.size()) > config_.target_keyframes)
      neighbors.resize(config_.target_keyframes);
    for (const auto &[d, j] : neighbors)
      target.submap.emplace_back(entries_[j].cloud, center * *estimates[j]);
    job.targets.push_back(std::move(target));
  }
  for (const auto &a : accepted_) {
    job.accepted.push_back(a);
    job.accepted_entries.push_back(entries_[a.latest]);
    job.accepted_entries.push_back(entries_[a.candidate]);
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    pending_ = std::move(job);
  }
  wake_.notify_one();
  return true;
}
/// 后台串行处理单槽任务，结果队列有界。
void LoopCloser::worker() {
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
    // 验证在独立线程池中运行，限制并发避免与主线程图更新争抢 CPU。
    Result result;
    if (config_.threads > 0) {
      tbb::task_arena arena(config_.threads);
      arena.execute([&] { result = compute(job); });
    } else {
      result = compute(job);
    }
    std::lock_guard<std::mutex> lock(mutex_);
    results_.push_back(std::move(result));
    while (results_.size() > 4)
      results_.pop_front();
    busy_ = false;
    idle_.notify_all();
  }
}
/// 源点云为最新帧及之前最多 source_keyframes−1 帧，按原始里程计拼到最新帧 IMU 系；
/// 目标为候选帧附近子图，按图估计拼到候选帧系；信息按源帧数折减。
LoopCloser::Result LoopCloser::compute(const Job &job) const {
  const auto start = std::chrono::steady_clock::now();
  Result result;
  result.generation = job.generation;
  result.latest_index = job.latest_index;
  result.latest_id = job.latest.id;
  Cloud source = *job.latest.cloud;
  const Pose reference = job.latest.odom.inverse();
  for (const auto &e : job.source) {
    Cloud moved;
    pcl::transformPointCloud(*e.cloud, moved, (reference * e.odom).matrix().cast<float>());
    source += moved;
  }
  const int frames = 1 + int(job.source.size());
  std::vector<Verified> verified;
  for (const auto &target : job.targets) {
    std::vector<Eigen::Vector3d> points;
    for (const auto &[cloud, pose] : target.submap)
      for (const auto &p : *cloud)
        points.push_back(pose * Eigen::Vector3d(p.x, p.y, p.z));
    MatchResult match;
    try {
      const auto map = PriorMap::fromPoints(points, matcher_);
      const Pose predicted = target.estimate.inverse() * job.latest_estimate;
      const Pose initial = descriptorYawToSourceTarget(
          target.descriptor.yaw, predicted.translation(), job.latest.odom,
          target.entry.odom);
      match = map->match(source, initial, frames);
    } catch (const std::exception &) {
      match.valid = false;
    }
    if (!match.valid) {
      ++result.registration_rejects;
      continue;
    }
    const Pose between = match.pose.inverse();
    if (!validate(job, target, match, between)) {
      ++result.geometry_rejects;
      continue;
    }
    if (!consistentWithHistory(job, target, between)) {
      ++result.pcm_rejects;
      continue;
    }
    verified.push_back({target, match, between});
  }
  if (!verified.empty()) {
    auto members = clique(verified);
    if (int(members.size()) >= config_.min_clique) {
      // 一致团成员共享同一源，只提交 RMSE 最小者，避免重复计数。
      std::sort(members.begin(), members.end(), [&](int a, int b) {
        const auto &l = verified[a], &r = verified[b];
        if (l.match.rmse != r.match.rmse)
          return l.match.rmse < r.match.rmse;
        if (l.target.descriptor.distance != r.target.descriptor.distance)
          return l.target.descriptor.distance < r.target.descriptor.distance;
        return l.target.descriptor.index < r.target.descriptor.index;
      });
      result.selected = verified[members.front()];
    }
  }
  result.milliseconds = std::chrono::duration<double, std::milli>(
                            std::chrono::steady_clock::now() - start)
                            .count();
  return result;
}
/// 同一前端段内使用原始里程计做世界系检验，避免以优化结果自证回环。
bool LoopCloser::validate(const Job &job, const Target &target,
                          const MatchResult &match, const Pose &between) const {
  if (match.overlap_ratio < config_.min_overlap ||
      between.translation().norm() > config_.max_translation)
    return false;
  const Pose predicted = job.latest.odom * between;
  if (config_.max_z > 0 &&
      std::abs(predicted.translation().z() - target.entry.odom.translation().z()) >
          config_.max_z)
    return false;
  if (config_.max_roll_pitch > 0 &&
      maxRollPitch(predicted.linear() * target.entry.odom.linear().transpose()) >
          config_.max_roll_pitch)
    return false;
  if (config_.max_yaw_residual > 0) {
    const double geometric =
        normalize(yawOf(job.latest.odom.linear()) - yawOf(predicted.linear()));
    if (std::abs(normalize(geometric - target.descriptor.yaw)) > config_.max_yaw_residual)
      return false;
  }
  return true;
}
/// 历史回环 a: old_latest←old_candidate，经原始里程计闭合到当前候选，残差在容差内即一致。
bool LoopCloser::consistentWithHistory(const Job &job, const Target &target,
                                       const Pose &between) const {
  if (!config_.pcm || config_.min_consistent_loops <= 0 || job.accepted.empty())
    return true;
  int consistent = 0;
  for (std::size_t i = 0; i < job.accepted.size(); ++i) {
    const Entry &old_latest = job.accepted_entries[2 * i];
    const Entry &old_candidate = job.accepted_entries[2 * i + 1];
    const Pose expected = job.latest.odom.inverse() * old_latest.odom *
                          job.accepted[i].between * old_candidate.odom.inverse() *
                          target.entry.odom;
    if (within(expected.inverse() * between, config_.pcm_translation,
               config_.pcm_rotation) &&
        ++consistent >= config_.min_consistent_loops)
      return true;
  }
  return false;
}
/// 两条同源回环推出的候选间相对位姿应与候选间原始里程计一致。
bool LoopCloser::sameSourceConsistent(const Verified &a, const Verified &b) const {
  if (!config_.pcm)
    return true;
  const Pose measured = a.between.inverse() * b.between;
  const Pose odom = a.target.entry.odom.inverse() * b.target.entry.odom;
  return within(odom.inverse() * measured, config_.pcm_translation, config_.pcm_rotation);
}
/// 候选数不超过 20 时分支限界求最大一致团；平局依次比较 RMSE 和、描述子距离和。
std::vector<int> LoopCloser::clique(const std::vector<Verified> &v) const {
  const int count = std::min<int>(int(v.size()), 20);
  if (!config_.pcm || count <= 1) {
    std::vector<int> all(count);
    for (int i = 0; i < count; ++i)
      all[i] = i;
    return all;
  }
  std::vector<std::vector<bool>> ok(count, std::vector<bool>(count, true));
  for (int i = 0; i < count; ++i)
    for (int j = i + 1; j < count; ++j)
      ok[i][j] = ok[j][i] = sameSourceConsistent(v[i], v[j]);
  std::vector<int> best, current;
  double best_rmse = std::numeric_limits<double>::max();
  double best_descriptor = std::numeric_limits<double>::max();
  std::function<void(std::vector<int>, double, double)> expand =
      [&](std::vector<int> available, double rmse, double descriptor) {
        while (!available.empty()) {
          if (current.size() + available.size() < best.size())
            return;
          const int vertex = available.back();
          available.pop_back();
          current.push_back(vertex);
          const double next_rmse = rmse + v[vertex].match.rmse;
          const double next_descriptor = descriptor + v[vertex].target.descriptor.distance;
          if (current.size() > best.size() ||
              (current.size() == best.size() &&
               (next_rmse < best_rmse ||
                (next_rmse == best_rmse && next_descriptor < best_descriptor)))) {
            best = current;
            best_rmse = next_rmse;
            best_descriptor = next_descriptor;
          }
          std::vector<int> next;
          for (int c : available)
            if (ok[vertex][c])
              next.push_back(c);
          if (!next.empty())
            expand(std::move(next), next_rmse, next_descriptor);
          current.pop_back();
        }
      };
  std::vector<int> all(count);
  for (int i = 0; i < count; ++i)
    all[i] = i;
  expand(all, 0., 0.);
  return best;
}
/// 联合 NIS 通过后以 candidate←latest 方向写入回环因子，并记录 PCM 历史与冷却起点。
ApplyOutcome LoopCloser::apply(KeyframeManager &manager) {
  ApplyOutcome outcome;
  std::deque<Result> results;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    results.swap(results_);
  }
  for (auto &r : results) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stats_.last_ms = r.milliseconds;
      stats_.max_ms = std::max(stats_.max_ms, r.milliseconds);
      stats_.total_ms += r.milliseconds;
      stats_.registration_rejects += r.registration_rejects;
      stats_.geometry_rejects += r.geometry_rejects;
      stats_.pcm_rejects += r.pcm_rejects;
      if (r.generation != generation_) {
        ++stats_.stale;
        continue;
      }
      if (!r.selected) {
        reason_ = "no verified loop candidate";
        continue;
      }
      ++stats_.verified;
      stats_.last_rmse = r.selected->match.rmse;
      stats_.last_overlap = r.selected->match.overlap_ratio;
    }
    const auto &s = *r.selected;
    PoseGraph &graph = manager.graph();
    LoopMeasurement m;
    m.measurement_id = "loop:" + std::to_string(generation_) + ":" +
                       std::to_string(r.latest_id) + ":" + std::to_string(s.target.entry.id);
    m.from = s.target.entry.id;
    m.to = r.latest_id;
    m.segment = manager.segment();
    m.relative = s.match.pose; // T_candidate←latest
    m.sqrt_information = s.match.sqrt_information;
    if (!graph.contains(m.from) || !graph.contains(m.to)) {
      std::lock_guard<std::mutex> lock(mutex_);
      ++stats_.stale;
      continue;
    }
    const auto nis_start = std::chrono::steady_clock::now();
    const Innovation innovation = graph.assessLoop(m);
    const auto update_start = std::chrono::steady_clock::now();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stats_.nis_ms_max = std::max(
          stats_.nis_ms_max,
          std::chrono::duration<double, std::milli>(update_start - nis_start).count());
      stats_.last_nis = innovation.nis;
      if (!innovation.valid || innovation.probability < config_.rejection_probability) {
        ++stats_.nis_rejects;
        reason_ = "loop rejected by pre-update NIS";
        continue;
      }
    }
    try {
      if (!graph.addLoop(m)) {
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
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stats_.update_ms_max = std::max(
          stats_.update_ms_max, std::chrono::duration<double, std::milli>(
                                    std::chrono::steady_clock::now() - update_start)
                                    .count());
    }
    accepted_.push_back({r.latest_index, s.target.descriptor.index, s.between});
    if (config_.history_size > 0 && accepted_.size() > config_.history_size)
      accepted_.erase(accepted_.begin(),
                      accepted_.end() - std::ptrdiff_t(config_.history_size));
    last_accepted_ = r.latest_index;
    if (records_.size() < 1024)
      records_.push_back({m, s.target.entry.stamp_ns, entries_[r.latest_index].stamp_ns,
                          innovation.nis, s.match.rmse, s.match.overlap_ratio});
    std::lock_guard<std::mutex> lock(mutex_);
    ++stats_.accepted;
    stats_.last_from = m.from;
    stats_.last_to = m.to;
    reason_ = "loop accepted";
  }
  return outcome;
}
/// 在锁内复制统计。
LoopStats LoopCloser::stats() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return stats_;
}
/// 在锁内复制原因字符串。
std::string LoopCloser::reason() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return reason_;
}
/// 交出已入图回环记录，调用方线程访问。
std::vector<LoopRecord> LoopCloser::takeAccepted() {
  std::vector<LoopRecord> out;
  out.swap(records_);
  return out;
}
/// 等待后台空闲且无待处理任务。
void LoopCloser::waitIdle() const {
  std::unique_lock<std::mutex> lock(mutex_);
  idle_.wait(lock, [&] { return !busy_ && !pending_; });
}
} // 命名空间 rm27_backend
