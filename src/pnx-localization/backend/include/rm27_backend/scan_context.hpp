#pragma once
#include "rm27_backend/types.hpp"
#include <optional>
#include <vector>

namespace rm27_backend {
/// Scan Context 参数，默认值照搬 z-localization 的 scan_context 配置。
struct ScanContextConfig {
  int num_rings = 30;              // 径向环数
  int num_sectors = 90;            // 方位扇区数
  double max_radius = 80.;         // 最大水平半径，m
  double downsample = .6;          // 输入体素边长，m；0 关闭
  double lidar_height = .35;       // 高度编码时加到 z 上的传感器高度，m
  int min_bin_points = 1;          // 栅格有效的最少点数
  int augment_count = 3;           // 每帧描述子数（含原始），横向平移增广
  double lateral_shift = 2.;       // 增广平移步长，m
  double search_ratio = 1.;        // 粗航向附近精搜索范围占整周比例
  int exclude_recent = 15;         // 检索时排除的最近关键帧数
  int candidates = 15;             // 返回的最大候选数
  double distance_threshold = .8;  // 进入几何验证的最大描述子距离
};
/// 单个 Scan Context：环×扇区的最大高度图及其扇区均值键。
struct ScanContext {
  Eigen::MatrixXf image;
  Eigen::VectorXf sector_key;
};
/// 描述子检索候选；yaw 为 source(最新)→target(候选) 的有符号偏航初值。
struct ScanContextCandidate {
  int index = -1;
  double distance = 1.;
  double yaw = 0.;
};
/// 保存按插入顺序编号的关键帧描述子集合，并检索最新帧的历史候选。
/// 编码、航向搜索与候选排序逐项对应 z-localization 的 ScanContextDescriptor。
class ScanContextDatabase {
public:
  /// 规范化并保存参数。
  explicit ScanContextDatabase(ScanContextConfig config = {});
  /// 以已整平的局部点云构造原始及横向平移增广描述子并追加，返回其编号。
  int add(const Cloud &leveled);
  /// 清空全部描述子。
  void clear() { entries_.clear(); }
  /// 返回已保存的关键帧数。
  std::size_t size() const { return entries_.size(); }
  /// 在 eligible 编号中检索最新一帧的候选；排除最近 exclude_recent 帧，按距离升序截断。
  std::vector<ScanContextCandidate>
  detectLatest(const std::vector<int> &eligible) const;
  /// 计算两个描述子的列余弦距离及最佳列移，供测试和诊断使用。
  std::pair<double, int> distance(const ScanContext &query,
                                  const ScanContext &candidate) const;
  /// 将点云编码为指定横向平移下的描述子。
  ScanContext make(const Cloud &leveled,
                   const Eigen::Vector2d &shift = Eigen::Vector2d::Zero()) const;
  /// 返回生效参数。
  const ScanContextConfig &config() const { return config_; }

private:
  ScanContextConfig config_;
  std::vector<std::vector<ScanContext>> entries_;
};
/// 去除机体偏航后的整平变换：T_level←body 的旋转为 R_yawᵀ·R_world←body，平移为零。
Pose levelFromBody(const Pose &world_from_body);
/// 从世界系旋转提取偏航角。
double yawOf(const Eigen::Matrix3d &rotation);
/// 把整平描述子的 source→target 偏航转换为原始 IMU 局部云之间的配准初值 T_target←source。
/// translation 为预测的 target 系平移，照搬 z-localization 的 descriptorYawToRawSourceTarget。
Pose descriptorYawToSourceTarget(double yaw, const Eigen::Vector3d &translation,
                                 const Pose &source_world,
                                 const Pose &target_world);
} // 命名空间 rm27_backend
