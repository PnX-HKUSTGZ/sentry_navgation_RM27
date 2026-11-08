#pragma once
#include "rm27_backend/types.hpp"
#include <deque>
#include <gtsam/nonlinear/ISAM2.h>
#include <map>
#include <memory>

namespace rm27_backend {
/// 位姿图配置：时间/数量窗口、首帧规范先验标准差及重新线性化阈值。
/// 外部观测入图前的新息检验结果；nis 为白化新息的马氏距离平方。
struct Innovation {
  bool valid = false;
  double nis = 0;
  int dof = 0;
  double probability = 0; // 卡方上尾概率，越小越不一致
};
struct GraphConfig {
  double lag_seconds = 30.;
  std::size_t max_states = 120;
  double prior_translation_std = .01, prior_rotation_std = .01;
  double relinearize_threshold = .01;
};
/// 封装 iSAM2 位姿图及有界历史管理；调用方必须串行执行读写。
/// 只含里程计链时复现前端相对运动；绝对位姿与回环约束由外部验收后提交。
class PoseGraph {
public:
  /// 校验图配置并创建空求解器。
  explicit PoseGraph(GraphConfig config = {});
  /// 清空图、时间索引和观测去重记录，开始指定轨迹段。
  void reset(std::uint64_t segment);
  /// 以原始首帧位姿建立规范先验；该先验不代表已获得赛场绝对定位。
  void addFirst(const Keyframe &keyframe);
  /// 以外部提供的位姿先验建立本段坐标基准，例如先验地图配准得到的 map 位姿。
  void addFirst(const Keyframe &keyframe, const PosePrior &prior);
  /// 添加相邻里程计边和目标节点，完成增量更新后裁剪过期状态。
  void addOdometry(const Keyframe &keyframe,
                   const OdometryMeasurement &measurement);
  /// 接入已验收的地图位姿观测；身份不符、重复或节点过期返回 false，非法测量抛异常。
  bool addAbsolute(const AbsolutePoseMeasurement &measurement);
  /// 不修改图，用当前边缘协方差计算绝对位姿观测的新息卡方统计；节点无效时 valid=false。
  Innovation assessAbsolute(const AbsolutePoseMeasurement &measurement) const;
  /// 接入已验收的回环观测并使用鲁棒核；不支持连接已边缘化的节点。
  bool addLoop(const LoopMeasurement &measurement);
  /// 查询指定活动节点的优化位姿；无效键由求解器报告异常。
  Pose estimate(Key key) const;
  /// 查询节点的边缘协方差，返回局部右扰动 [平移, 旋转] 顺序。
  Covariance covariance(Key key) const;
  /// 获取全部活动节点的位姿快照。
  std::map<Key, Pose> estimates() const;
  /// 查询活动节点的原始纳秒时间戳，过期键抛异常。
  std::int64_t stamp(Key key) const;
  /// 判断图中是否仍保留该节点。
  bool contains(Key key) const;
  /// 返回当前活动位姿节点数。
  std::size_t size() const { return times_.size(); }
  /// 返回因子容器槽数，包含空槽，用于监测长期资源增长。
  std::size_t factorSlots() const;
  /// 返回本轨迹段已边缘化的位姿数量。
  std::uint64_t marginalized() const { return marginalized_; }

private:
  /// 提交一批因子与初值；求解异常后调用方应丢弃本段估计器状态。
  void submit(const gtsam::NonlinearFactorGraph &graph,
              const gtsam::Values &values);
  /// 按时间和数量边缘化旧节点，并压缩空因子槽；保留存活变量的联合线性信息。
  void trim();
  GraphConfig config_;
  std::unique_ptr<gtsam::ISAM2> isam_;
  std::deque<std::pair<Key, std::int64_t>> times_;
  std::map<std::string, std::pair<Key, Key>> observations_;
  std::uint64_t segment_ = 0, marginalized_ = 0;
};
} // 命名空间 rm27_backend
