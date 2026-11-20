#pragma once
#include "relocalization_manager/recovery_engine.hpp"
#include "rm27_backend/localization_state.hpp"
#include "rm27_backend/map_localizer.hpp"
#include "rm27_backend/motion_recovery.hpp"

namespace rm27_backend {
/// 统一恢复调度参数；时间门限分别用于传感器数据积累与结果新鲜度。
struct ControllerConfig {
  MotionRecoveryConfig motion;
  LocalizerConfig tracking;
  RecoveryConfig recovery;
  int source_min_frames = 2, source_max_frames = 5, failures_before_lost = 3;
  int degraded_frames_before_suspect = 10; // 持续前端低质量撤销可信传播。
  int rmse_failures_before_suspect =
      3; // 成功地图因子提交之间的 RMSE 超限次数。
  bool allow_unbounded_auto_recovery =
      false; // 无运动包络时仅允许显式人工恢复。
  double source_span = .5, result_max_age = 8., input_timeout = 1.;
  double ambiguity_retry_period = 3.; // 静止歧义场景的墙钟重试间隔，s。
  double retry_translation = .3,
         retry_rotation = .15; // 新视角触发提前重试的运动量。
};
/// 后端唯一定位协调器；后台只读几何计算，调用方串行决定状态和图提交。
class LocalizationController {
public:
  /// 启动同进程后台线程，地图包加载失败保持不可用并报告错误。
  LocalizationController(ControllerConfig config, MatcherConfig matcher,
                         ScanContextConfig sc, std::string bundle,
                         Pose alignment);
  /// 取消未开始任务并等待当前有界计算退出。
  ~LocalizationController();
  /// 同步新段，保留最后可信位姿用于附近搜索，作废在途结果。
  bool syncSegment(std::uint64_t segment);
  /// 数值故障后使当前定位失效。
  void resetSegment();
  /// 接收有界近期局部云，恢复不依赖已有关键帧图。
  void observe(const Frame &frame, Cloud::ConstPtr cloud);
  /// 从工作结果中验收跟踪因子或暂存恢复锚。
  ApplyOutcome apply(KeyframeManager &manager);
  /// 将恢复源时刻的锚传播到当前帧；该方法只准备先验，不宣布成功。
  std::optional<PosePrior> takeAnchor(const Frame &frame);
  /// 首帧图先验提交成功后安装可信锚。
  void confirmAnchor();
  /// 普通地图跟踪仅在可信状态调度。
  bool requestTracking(const Keyframe &frame, const Pose &prediction,
                       Cloud::ConstPtr source, int frames);
  /// 根据主状态调度附近或全局恢复，提示仅作为待几何验证的初值。
  bool requestRecovery(const Frame &frame);
  /// 保存最后可信 IMU 位姿及其同刻原子前端状态，作为运动约束的恢复锚。
  void rememberPose(const Pose &pose, std::int64_t stamp);
  /// 人工触发全局恢复，可选提示需属于当前帧时间与坐标合同。
  void manual(const std::optional<Pose> &hint,
              const std::optional<Pose> &hint_odom = std::nullopt,
              bool wait_for_hint = false);
  /// 同一人工请求的同刻 TF 到齐后安装提示，不重复清空扫描或递增状态代次。
  void updateHint(const Pose &hint, const Pose &hint_odom,
                  HintLimits limits = {});
  /// 请求截止或输入非法时停止后台恢复，等待新的人工请求或轨迹段。
  void cancelRecovery(const std::string &reason);
  /// 输入超时只触发一次故障代次，恢复后由实际结果解除。
  void inputTimedOut();
  /// 地图是否已成功加载。
  bool ready() const { return loaded_.load(); }
  /// 地图是否加载失败。
  bool failed() const { return failed_.load(); }
  /// 当前是否具有有效地图锚。
  bool anchored() const { return anchored_; }
  /// 统一状态机快照只由调用线程读取。
  const LocalizationStateMachine &state() const { return state_; }
  /// 取得只读地图背景，用于保留原导航背景点云接口。
  Cloud::ConstPtr background() const;
  /// 读取统计快照。
  LocalizerStats stats() const;
  /// 返回状态原因或加载错误。
  std::string reason() const;
  /// 等待后台空闲，仅供测试使用。
  void waitIdle() const;

private:
  /// 一个不可变计算请求，绑定前端身份、源时间、段号和状态代次。
  struct Job {
    Frame frame;
    Cloud::ConstPtr cloud;
    Pose prediction = Pose::Identity();
    std::optional<Pose> hint, near;
    Key key = 0;
    std::uint64_t generation = 0, segment = 0;
    int frames = 1;
    bool recovery = false;
    std::uint64_t cancellation = 0;
    HintLimits hint_limits;
    std::optional<RecoveryRegion> region;
  };
  /// 工作线程输出，包含等待与失败的明确区别。
  struct Result {
    Job job;
    RecoveryEstimate estimate;
  };
  /// 后台加载及单任务循环；计算过程中不持有调度锁。
  void worker(std::string bundle, Pose alignment, MatcherConfig matcher,
              ScanContextConfig sc);
  /// 单槽准入，忙时丢弃新请求并记录，不积压点云。
  bool enqueue(Job job);
  /// 取消在途迭代并清除尚未执行或消费的工作，不等待几何线程。
  void cancelWork();
  ControllerConfig config_;
  MotionRecovery motion_;
  LocalizationStateMachine state_;
  std::optional<std::uint64_t> segment_;
  std::optional<Identity> identity_;
  std::optional<Pose> hint_, hint_odom_;
  std::deque<std::pair<Frame, Cloud::ConstPtr>> history_;
  std::optional<Result> anchor_;
  std::int64_t last_request_ns_ = 0;
  bool anchored_ = false, timeout_ = false;
  int degraded_frames_ = 0;
  std::atomic<bool> loaded_{false}, failed_{false};
  std::atomic<std::uint64_t> cancellation_{0};
  bool waiting_hint_ = false, recovery_paused_ = false;
  bool manual_global_recovery_ = false;
  std::string recovery_blocked_reason_;
  HintLimits hint_limits_;
  std::optional<Pose> retry_pose_;
  std::chrono::steady_clock::time_point retry_after_;
  mutable std::mutex mutex_;
  mutable std::condition_variable wake_, idle_;
  std::optional<Job> pending_;
  std::deque<Result> results_;
  LocalizerStats stats_;
  Cloud::ConstPtr background_;
  std::string error_;
  bool busy_ = false, stop_ = false;
  std::thread thread_;
};
} // namespace rm27_backend
