#pragma once
#include "rm27_backend/keyframe_manager.hpp"
#include "rm27_backend/map_localizer.hpp"
#include "rm27_backend/map_matcher.hpp"
#include "rm27_backend/scan_context.hpp"
#include <condition_variable>
#include <deque>
#include <mutex>
#include <thread>

namespace rm27_backend {
/// 回环检索、验证和入图参数；默认值照搬 z-localization 的 loop 配置，角度单位 rad。
struct LoopConfig {
  int target_keyframes = 17;         // 目标子图最多关键帧数
  double target_radius = 12.;        // 目标子图空间半径，m
  bool target_same_pass = false;     // true 只用候选前后相邻关键帧按原始里程计拼子图；false 照搬 z-localization 取空间近邻
  int cooldown_keyframes = 5;        // 接受回环后再次检索的最少关键帧间隔
  double query_distance = 0.;        // 两次检索间原始里程计最少行程，m；0 每个关键帧都检索
  double query_interval = 0.;        // 行程不足时两次检索的最少间隔，s；0 只按行程（静止不检索）
  double candidate_max_distance = 7.;  // 候选相对最新帧的最大图估计距离，m；0 不限
  double descriptor_ratio_threshold = 1.;  // 最佳/次佳描述子距离比上限；≥1 关闭
  int source_keyframes = 4;          // 源点云最多累计的因果关键帧数
  double cloud_voxel = .25;          // 回环库保存与配准使用的点云体素，m
  int threads = 0;                   // 后台验证可用的 TBB 线程数；0 沿用进程上限
  std::size_t max_keyframes = 3000;  // 回环库关键帧上限，超出后停止增长
  std::size_t history_size = 200;    // PCM 保留的已接受回环数
  double max_translation = 3.;       // 回环测量平移上限，m
  double max_z = .5;                 // 原始里程计下的垂直闭合差上限，m
  double max_roll_pitch = .1745;     // roll/pitch 闭合残差上限
  double max_yaw_residual = 1.047;   // 几何偏航与描述子偏航差上限
  double min_overlap = .45;          // 最小独立几何重叠率
  bool pcm = true;                   // 是否启用 PCM 一致性与最大一致团
  double pcm_translation = 1.;       // PCM 闭合平移误差上限，m
  double pcm_rotation = .2618;       // PCM 闭合旋转误差上限
  int min_consistent_loops = 1;      // 要求的历史一致回环数；无可比历史时放行
  int min_clique = 1;                // 同源候选一致团的最小规模
  double rejection_probability = 1e-3;  // 联合 NIS 拒绝概率
};
/// 已入图回环的记录，供诊断日志和离线真值评估。
struct LoopRecord {
  LoopMeasurement measurement;       // from=候选，to=最新，relative=T_from←to
  std::int64_t from_stamp_ns = 0, to_stamp_ns = 0;
  double nis = 0, rmse = 0, overlap = 0;
};
/// 回环统计；耗时单位 ms。
struct LoopStats {
  std::size_t keyframes = 0, descriptor_bytes = 0, cloud_bytes = 0;
  std::uint64_t queries = 0, candidates = 0, verified = 0, accepted = 0;
  std::uint64_t busy_skips = 0, stale = 0, nis_rejects = 0;
  std::uint64_t registration_rejects = 0, geometry_rejects = 0, pcm_rejects = 0;
  double last_nis = 0, last_rmse = 0, last_overlap = 0, last_ms = 0, max_ms = 0,
         total_ms = 0;
  double nis_ms_max = 0, update_ms_max = 0; // 调用方线程：联合 NIS 与入图更新的最长耗时
  Key last_from = 0, last_to = 0;
};
/// Scan Context 回环：调用方线程登记关键帧、采集快照并提交；后台单线程完成配准与一致性验证。
/// 回环库只覆盖当前轨迹段，段号变化时清空并使在途结果失效。
class LoopCloser {
public:
  /// 校验参数并启动后台线程。
  LoopCloser(LoopConfig config, ScanContextConfig descriptor, MatcherConfig matcher);
  /// 停止后台线程。
  ~LoopCloser();
  /// 段号变化时清空回环库、历史与冷却状态；首次调用只记录段号。
  bool syncSegment(std::uint64_t segment);
  /// 登记新关键帧：按原始里程计 roll/pitch 整平生成描述子，并保存降采样局部点云。
  void addKeyframe(const Keyframe &keyframe, const Cloud &cloud);
  /// 为最新关键帧检索候选并排队验证；后台忙、冷却中或无候选时返回 false。
  bool request(const KeyframeManager &manager);
  /// 提交已完成的验证结果：身份检查、联合 NIS，通过后以鲁棒回环因子入图。
  ApplyOutcome apply(KeyframeManager &manager);
  /// 读取统计快照。
  LoopStats stats() const;
  /// 最近一次拒绝或接受原因。
  std::string reason() const;
  /// 取出并清空自上次调用以来入图的回环记录。
  std::vector<LoopRecord> takeAccepted();
  /// 等待后台空闲，仅用于测试与离线工具。
  void waitIdle() const;

private:
  /// 回环库条目：关键帧身份、时间戳、原始里程计位姿、累计行程和只读降采样点云。
  struct Entry {
    Key id = 0;
    std::int64_t stamp_ns = 0;
    Pose odom = Pose::Identity();
    double travel = 0; // 本段原始里程计累计行程，m
    std::shared_ptr<const Cloud> cloud;
  };
  /// 已接受回环：最新帧编号、候选编号和 T_latest←candidate 测量。
  struct Accepted {
    int latest = -1, candidate = -1;
    Pose between = Pose::Identity();
  };
  /// 单个候选的验证输入：描述子结果、候选条目、图估计和目标子图组成。
  struct Target {
    ScanContextCandidate descriptor;
    Entry entry;
    Pose estimate = Pose::Identity();
    std::vector<std::pair<std::shared_ptr<const Cloud>, Pose>> submap;
  };
  /// 后台任务快照。
  struct Job {
    std::uint64_t generation = 0;
    int latest_index = -1;
    Entry latest;
    Pose latest_estimate = Pose::Identity();
    std::vector<Entry> source;
    std::vector<Target> targets;
    std::vector<Accepted> accepted;
    std::vector<Entry> accepted_entries; // 与 accepted 对应的 (latest, candidate) 原始位姿
  };
  /// 通过全部几何验证的候选。
  struct Verified {
    Target target;
    MatchResult match;
    Pose between = Pose::Identity(); // T_latest←candidate
  };
  /// 后台验证结果及各类拒绝计数。
  struct Result {
    std::uint64_t generation = 0;
    int latest_index = -1;
    Key latest_id = 0;
    std::optional<Verified> selected;
    std::uint64_t registration_rejects = 0, geometry_rejects = 0, pcm_rejects = 0;
    double milliseconds = 0;
  };
  /// 后台主循环。
  void worker();
  /// 执行一次完整验证：累计源点云，逐候选配准与检验，最后按一致团择优。
  Result compute(const Job &job) const;
  /// 照搬 z-localization 的 validateMeasurement：平移、z、roll/pitch、描述子偏航与重叠率。
  bool validate(const Job &job, const Target &target, const MatchResult &match,
                const Pose &between) const;
  /// PCM：用原始里程计把历史回环推到当前候选，至少一条一致或无可比历史时通过。
  bool consistentWithHistory(const Job &job, const Target &target,
                             const Pose &between) const;
  /// 同源候选两两一致性，用于最大一致团。
  bool sameSourceConsistent(const Verified &a, const Verified &b) const;
  /// 分支限界求最大一致团，按团大小、RMSE 和、描述子距离和择优。
  std::vector<int> clique(const std::vector<Verified> &verified) const;

  LoopConfig config_;
  MatcherConfig matcher_;
  ScanContextDatabase database_;
  mutable std::mutex mutex_;
  mutable std::condition_variable wake_, idle_;
  std::optional<Job> pending_;
  std::deque<Result> results_;
  bool busy_ = false, stop_ = false;
  LoopStats stats_;
  std::string reason_ = "waiting for keyframes";
  // 以下成员仅在调用方线程访问。
  std::vector<Entry> entries_;
  std::vector<Accepted> accepted_;
  std::vector<LoopRecord> records_;
  std::optional<std::uint64_t> segment_;
  std::uint64_t generation_ = 0;
  int last_checked_ = -1, last_accepted_ = -1;
  double last_query_travel_ = -1; // 上次检索时的累计行程；负值表示尚未检索
  std::int64_t last_query_stamp_ = 0;
  std::thread thread_;
};
} // 命名空间 rm27_backend
