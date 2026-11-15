#pragma once
#include <deque>
#include <optional>

#include "relocalization_manager/map_matcher.hpp"
#include "relocalization_manager/scan_context.hpp"

namespace rm27_backend
{
/// 有限位置提示的验收范围，由消息协方差与部署上限共同确定。
struct HintLimits
{
  double translation = 2.;  // 相对提示允许的最大平移，m
  double rotation = .6;     // 相对提示允许的最大旋转，rad
};
/// 有效连续性下的 map←IMU 预测；只约束候选，不替代几何验收。
struct RecoveryRegion
{
  Pose pose = Pose::Identity();
  double radius = 1.5;         // 当前 IMU 位置相对预测的位置误差上限，m
  double angle = .2617993878;  // 相对预测的完整旋转误差上限，rad
  /// 几何结果必须满足位置与姿态范围，不能只约束检索参考视点。
  bool contains(const Pose & candidate) const
  {
    const Pose delta = pose.inverse() * candidate;
    return validPose(candidate) && delta.translation().norm() <= radius &&
           rm27_backend::angle(delta) <= angle;
  }
};
/// 全局恢复计算参数；源点云、候选数及子图缓存均有明确上限。
struct RecoveryConfig
{
  std::string source_pcd, navigation_grid;  // 可选部署源文件，存在时核对地图包来源哈希。
  int candidates = 8;                       // 最多几何候选数
  int yaw_samples = 6;                      // 无可靠航向时整周采样数
  int cache_targets = 8;                    // 最多保留的已准备子地图数
  double suspect_radius = 8.;   // 可疑状态在最后可信位置附近搜索半径，m
  double target_radius = 12.;   // 配准子地图半径，m
  double ambiguity_ratio = .9;  // 不同地点最佳/次佳 RMSE 比上限
  double ambiguity_angle_rad = .5235987756;  // 相同地点不同航向也属于独立假设。
  double ambiguity_distance = 2.;            // 区分不同地点假设的距离，m
  double min_overlap = .45;                  // 恢复最低重叠比例
  bool hint_global_fallback = true;          // 提示附近失败后是否允许全地图搜索
  int hint_yaw_samples = 3;              // 提示附近航向假设数，奇数且包含原航向
  double hint_yaw_step = .1745329252;    // 提示附近航向扰动步长，rad
  double duplicate_distance = .15;       // 同一配准收敛位置的复验去重距离，m
  double duplicate_angle = .0872664626;  // 同一配准收敛姿态的复验去重角度，rad
  double max_seconds = 5.;               // 单次搜索墙钟预算，在假设间检查
};
/// 几何计算结果：失配不修改图，调用方必须复核任务身份后提交。
struct RecoveryEstimate
{
  MatchResult match;
  bool found = false, deferred = false;
  std::string reason;
  int attempts = 0, candidate = -1;
};
/// 不可变先验地图与描述子库；由同一后台线程串行调用，完全不访问 ROS、状态机和图。
class RecoveryEngine
{
public:
  /// 加载离线地图包，核对格式、地图坐标变换及资源边界。
  RecoveryEngine(
    const std::string & bundle, const Pose & map_from_pcd, MatcherConfig matcher,
    ScanContextConfig descriptor, RecoveryConfig config);
  /// 在预测位置附近准备子图并进行正常跟踪，允许部分秩观测。
  MatchResult track(
    const Cloud & source, const Pose & prediction, int frames,
    const std::function<bool()> & cancel = {});
  /// 优先验证可选提示；失败后按状态范围执行描述子和航向搜索，要求完整位姿可观。
  RecoveryEstimate recover(
    const Cloud & source, const Pose & odom, int frames, const std::optional<Pose> & hint,
    const std::optional<Pose> & near, const std::function<bool()> & cancel = {},
    HintLimits hint_limits = {}, const std::optional<RecoveryRegion> & region = std::nullopt);
  /// 导出已对齐背景点云，供导航动态障碍检测使用；不含实时扫描。
  Cloud::ConstPtr background() const;
  /// 返回地图点数。
  std::size_t size() const { return points_.size(); }
  /// 返回公共验证地图中有效平面点数。
  std::size_t planarPoints() const { return verification_map_->planarPoints(); }
  /// 返回参考视点数。
  std::size_t views() const { return views_.size(); }
  /// 返回地图版本，作为诊断和输出身份。
  const std::string & version() const { return version_; }

private:
  /// 根据参考视点索引构建并缓存有界目标子图。
  std::shared_ptr<const PriorMap> target(int index);
  /// 查找距离给定位置最近的参考视点。
  int nearest(const Eigen::Vector3d & position) const;
  MatcherConfig matcher_;
  RecoveryConfig config_;
  ScanContextDatabase descriptors_;
  std::vector<Eigen::Vector3d> points_;
  std::vector<Pose> views_;
  std::deque<std::pair<int, std::shared_ptr<const PriorMap>>> cache_;
  std::string version_;
  std::shared_ptr<const PriorMap> verification_map_;
};
}  // namespace rm27_backend
