#pragma once
#include "rm27_backend/keyframe_manager.hpp"
#include "rm27_backend/map_matcher.hpp"
#include <atomic>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <thread>

namespace rm27_backend {
/// 先验地图定位调度与验收参数；时间单位 s，距离 m，角度 rad。
struct LocalizerConfig {
  double correction_period = .5;          // 跟踪配准最小间隔
  double acquisition_period = 1.;         // 初始锚定尝试最小间隔
  int source_keyframes = 3;               // 跟踪源点云合并的最多关键帧数
  double source_max_distance = 1.5;       // 参与合并的关键帧相对当前帧最大距离
  double max_translation_correction = 1.; // 常规校正平移上限
  double max_rotation_correction = .2618; // 常规校正旋转上限
  double acquisition_max_translation = 1.5; // 锚定结果相对种子的平移上限
  double acquisition_max_rotation = .349;   // 锚定结果相对种子的旋转上限
  double rejection_probability = 1e-3;      // NIS 卡方上尾概率拒绝门限
  int max_conflicts = 7;                     // 连续冲突后进入 Suspect
  double recovery_translation = .1;          // Suspect 下连续两次对齐一致的平移容差
  double recovery_rotation = .0349;          // Suspect 下连续两次对齐一致的旋转容差
  double anchor_max_age = .5;                // 锚定结果外推到新帧的最大时差
  double continuity_timeout = 2.;            // 断段后沿用上一段 map 位姿作种子的最长时差
};
/// 先验地图定位状态：加载、失败、等待锚定、正常跟踪、可疑（等待一致恢复）。
enum class MapState { Loading, Failed, Waiting, Tracking, Suspect };
/// 返回状态的英文短名，用于诊断输出。
const char *toString(MapState state);
/// 先验地图定位统计，单位：时间 ms，距离 m。
struct LocalizerStats {
  std::size_t map_points = 0, planar_points = 0;
  std::uint64_t requests = 0, completed = 0, busy_skips = 0, stale = 0;
  std::uint64_t accepted = 0, anchors = 0, acquisition_failures = 0;
  std::uint64_t quality_rejects = 0, correction_rejects = 0, nis_rejects = 0;
  std::uint64_t recoveries = 0, consecutive_conflicts = 0;
  double last_rmse = 0, last_inlier_ratio = 0, last_nis = 0;
  double last_correction = 0, last_ms = 0, max_ms = 0, total_ms = 0;
  int last_rank = 0;
};
/// 调用 apply 的结果：本次接入的地图因子数，以及是否发生需要断段的图异常。
struct ApplyOutcome {
  std::size_t accepted = 0;
  bool graph_failure = false;
};
/// 先验地图定位器：后台单线程执行加载与配准，调用方线程负责全部图写入。
/// 采用“采集快照 → 无锁计算 → 回到调用方提交”流程，并以代次丢弃过期结果。
class MapLocalizer {
public:
  /// 校验参数并启动后台线程；线程先加载地图，失败时进入 Failed 状态。
  MapLocalizer(LocalizerConfig config, MatcherConfig matcher, std::string path,
               Pose map_transform);
  /// 停止后台线程并等待当前配准结束。
  ~MapLocalizer();
  /// 线程安全地读取当前状态。
  MapState state() const { return state_.load(); }
  /// 判断本段图是否已在 map 系建立；仅在调用方线程访问。
  bool anchored() const { return anchored_; }
  /// 判断地图可用，即后台加载已成功。
  bool ready() const;
  /// 新轨迹段或图重建时调用：清除锚定与待用结果，并递增代次使在途结果失效。
  void resetSegment();
  /// 每帧传入管理器当前段号，仅在段号真正变化时调用 resetSegment 并返回 true。
  /// 未锚定时管理器每帧都报告“新段”接纳结果，不能据此重置，否则锚定结果永远过期。
  bool syncSegment(std::uint64_t segment);
  /// 记录最近一次输出的 map 位姿，供断段后的连续性种子使用。
  void rememberPose(const Pose &map_pose, std::int64_t stamp_ns);
  /// 若断段间隔未超时，返回上一段最后的 map 位姿作为新段初值。
  std::optional<Pose> continuitySeed(std::int64_t stamp_ns) const;
  /// 在周期与单任务约束下提交锚定配准；源点云为该帧 IMU 局部云。
  bool requestAcquisition(const Frame &frame, Cloud::ConstPtr cloud,
                          const Pose &seed);
  /// 若已有锚定结果且与本帧同身份、时差合法，返回外推到本帧的 map 先验并标记已锚定。
  std::optional<PosePrior> takeAnchor(const Frame &frame);
  /// 在周期与单任务约束下为已提交关键帧排队跟踪配准；frames 为源云合并帧数。
  bool requestTracking(const Keyframe &keyframe, const Pose &predicted,
                       Cloud::ConstPtr source, int frames);
  /// 处理已完成结果：做身份、质量、校正上限和 NIS 检验后提交地图因子。
  ApplyOutcome apply(KeyframeManager &manager);
  /// 读取统计快照。
  LocalizerStats stats() const;
  /// 返回最近一次拒绝或状态变化的原因。
  std::string reason() const;
  /// 阻塞等待后台空闲且无待处理任务，仅用于测试和离线工具。
  void waitIdle() const;

private:
  /// 后台任务类型：初始锚定或常规跟踪。
  enum class Kind { Acquire, Track };
  /// 后台任务快照，包含提交时代次、关键帧身份、原始里程计位姿和源点云。
  struct Job {
    Kind kind = Kind::Track;
    std::uint64_t generation = 0;
    Key key = 0;
    Frame frame;
    Pose initial = Pose::Identity();
    Cloud::ConstPtr cloud;
    int frames = 1;
  };
  /// 任务与对应配准结果。
  struct Result {
    Job job;
    MatchResult match;
  };
  /// 后台线程主循环：先加载地图，再串行处理单槽任务。
  void worker(std::string path, Pose transform);
  /// 在锁内放入任务；后台忙或已有待处理任务时返回 false。
  bool enqueue(Job job);
  /// 记录一次冲突，连续冲突达到门限时转入 Suspect。
  void conflict(const std::string &why);

  LocalizerConfig config_;
  MatcherConfig matcher_;
  std::shared_ptr<const PriorMap> map_;
  std::atomic<MapState> state_{MapState::Loading};
  mutable std::mutex mutex_;
  mutable std::condition_variable wake_, idle_;
  std::optional<Job> pending_;
  std::deque<Result> results_;
  bool busy_ = false, stop_ = false;
  LocalizerStats stats_;
  std::string reason_ = "loading prior map";
  // 以下成员只在调用方线程访问。
  std::uint64_t generation_ = 0;
  std::optional<std::uint64_t> segment_;
  bool anchored_ = false;
  std::optional<Result> anchor_;
  std::optional<Pose> recovery_candidate_;
  std::optional<std::pair<Pose, std::int64_t>> last_pose_;
  std::int64_t last_acquisition_ns_ = INT64_MIN / 2, last_track_ns_ = INT64_MIN / 2;
  std::thread thread_;
};
} // 命名空间 rm27_backend
