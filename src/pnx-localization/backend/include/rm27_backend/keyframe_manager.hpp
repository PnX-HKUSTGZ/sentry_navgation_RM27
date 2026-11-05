#pragma once
#include "rm27_backend/pose_graph.hpp"
#include <optional>
#include <set>

namespace rm27_backend {
/// 关键帧触发、输入连续性和里程计过程噪声参数；噪声系数均为方差系数。
struct KeyframeConfig {
  double distance = .5, angle_rad = .1745329252, min_interval = .2,
         max_interval = .5;
  double max_gap = 1., max_linear_speed = 12., max_angular_speed = 35.;
  double translation_variance_per_second = .0005,
         translation_variance_per_meter = .0002;
  double rotation_variance_per_second = 1e-6,
         rotation_variance_per_radian = 3e-7;
  double degraded_noise_multiplier = 10.;
};
/// 输入接纳结果：拒绝、同段有效帧或新独立轨迹段。
enum class Admission { Rejected, Accepted, NewSegment };
/// 管理输入身份、关键帧、里程计测量和活动图元数据；调用方负责串行访问。
/// 原始位姿保持不变，优化结果单独保存；重置或连续性丢失时禁止跨段连边。
class KeyframeManager {
public:
  /// 校验关键帧与噪声参数，构造图管理对象。
  KeyframeManager(KeyframeConfig config = {}, GraphConfig graph = {});
  /// 验收身份、序号和时间，累计区间质量，必要时开始新轨迹段。
  Admission observe(const Frame &frame);
  /// 判断已接纳帧是否满足首帧或运动/时间触发条件，不修改状态。
  bool shouldCreate(const Frame &frame) const;
  /// 为候选帧生成首帧先验或里程计边，提交图后登记关键帧；点云应由调用方预先准备。
  Keyframe commit(const Frame &frame);
  /// 将最近关键帧的局部右扰动协方差传播到当前帧，并加入配置的过程噪声。
  Covariance predictionCovariance(const Frame &frame,
                                  const Covariance &anchor) const;
  /// 清空图和活动关键帧并递增段号，保留输入水位以拒绝延迟旧帧。
  void breakSegment();
  /// 获取最近一次关键帧提交的里程计边；本段首帧没有前驱边。
  const std::optional<OdometryMeasurement> &lastMeasurement() const {
    return last_edge_;
  }
  /// 获取最新关键帧，尚未建帧时为空。
  const std::optional<Keyframe> &latest() const { return latest_; }
  /// 获取最近一次输入判定的原因。
  const std::string &reason() const { return reason_; }
  /// 访问图接口，供调用方提交外部约束或查询估计。
  PoseGraph &graph() { return graph_; }
  /// 只读访问活动关键帧元数据，不加载点云。
  const std::map<Key, Keyframe> &keyframes() const { return keyframes_; }
  /// 刷新活动关键帧估计并移除过期元数据；外部图更新后应调用。
  void synchronizeEstimates();
  /// 返回当前轨迹段编号。
  std::uint64_t segment() const { return segment_; }
  /// 返回本管理器已创建的关键帧总数，也是下一帧待分配的 ID。
  std::uint64_t totalKeyframes() const { return next_id_; }
  /// 返回已检测到的同代序号缺口总数。
  std::uint64_t sequenceGaps() const { return sequence_gaps_; }

private:
  KeyframeConfig config_;
  PoseGraph graph_;
  std::map<Key, Keyframe> keyframes_;
  std::optional<Frame> previous_;
  std::optional<Keyframe> latest_;
  std::optional<OdometryMeasurement> last_edge_;
  std::set<std::string> retired_instances_;
  std::string reason_ = "waiting";
  Key next_id_ = 0;
  std::uint64_t segment_ = 0, sequence_gaps_ = 0;
  bool interval_degraded_ = false;
};
} // namespace rm27_backend
