#pragma once
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "relocalization_manager/measurement.hpp"

namespace rm27_backend
{
/// 先验地图扫描配准参数；距离单位为 m，角度为 rad。
/// 质量门限和信息构造参照 z-localization 的点到面验收与平方根信息流程。
struct MatcherConfig
{
  bool gravity_aligned = false;  // 地图与前端世界重力对齐时，优化中固定初值 roll/pitch。
  bool point_to_plane = false;  // 新地图跟踪/恢复启用点到面；既有回环保持已验收的 GICP。
  double map_voxel = .1;        // 先验地图体素边长
  int normal_neighbors = 10;    // 地图与源点云局部协方差近邻数
  double max_plane_ratio = .2;  // 地图点最小/最大特征值比上限，超出不作平面
  double source_voxel = .2;     // 源点云体素边长
  int max_iterations = 20;      // 点到面最大迭代次数
  double max_correspondence_distance = 1.;  // 对应点最大距离
  double huber_delta = .1;                  // 点到面 Hessian 的 Huber 转折残差
  int min_inliers = 300;                    // 有效平面对应点下限
  double min_inlier_ratio = .5;             // 有效平面对应点占源点比例下限
  double max_rmse = .15;                    // 点到面 RMSE 上限
  int min_coverage_sectors = 6;             // 36 个水平扇区中有对应点的最少扇区数
  double residual_std_floor = .03;          // 残差标准差下限
  double information_scale = 1.;            // 附加信息缩放；还会再除以源帧数
  double translation_std_floor = .01;       // 共模平移标准差下限
  double rotation_std_floor = .001745;      // 共模旋转标准差下限
  double max_condition = 100.;   // 平衡 Hessian 条件数门限，更弱方向不提供信息
  double overlap_distance = .5;  // 重叠率统计的最近点距离
};
/// 单次配准结果；pose 为 map←IMU，sqrt_information 为 rank×6 局部右扰动 [平移, 旋转]。
struct MatchResult
{
  bool valid = false;
  std::string reason;
  Pose pose = Pose::Identity();
  Eigen::MatrixXd sqrt_information;
  int rank = 0;
  bool converged = false;
  int iterations = 0;
  std::size_t source_points = 0, inliers = 0;
  double inlier_ratio = 0, rmse = 0, condition = 0;
  double overlap_ratio = 0;  // 对齐后最近点距离不超过 overlap_distance 的源点比例
  int coverage_sectors = 0;
  double milliseconds = 0;
};
/// 只读先验地图：体素化点云、KD 树、法向/协方差与平面标记；加载后可跨线程共享。
/// small_gicp 仅出现在实现文件中，不进入导出头文件。
class PriorMap
{
public:
  /// 读取 PCD，先施加 map←PCD 变换再体素化并估计局部几何；失败抛异常。
  static std::shared_ptr<const PriorMap> load(
    const std::string & path, const Pose & transform, const MatcherConfig & config);
  /// 由已在 map 系的点构造地图，主要用于测试与离线工具。
  static std::shared_ptr<const PriorMap> fromPoints(
    const std::vector<Eigen::Vector3d> & points, const MatcherConfig & config);
  /// 返回体素化后的地图点数。
  std::size_t size() const;
  /// 返回可作为点到面目标的平面点数。
  std::size_t planarPoints() const;
  /// 以 map←IMU 初值配准 IMU 局部点云；frames 为源点云合并的关键帧数，用于缩放信息。
  /// 结果总是返回，质量不达标时 valid=false 并给出原因。
  MatchResult match(const Cloud & source, const Pose & initial, int frames) const;
  /// 不透明源点云；一次体素化和协方差计算后供多个候选只读复用。
  struct Source;
  /// 预处理扫描；参数必须与使用该源的地图配准参数一致。
  static std::shared_ptr<const Source> prepareSource(
    const Cloud & source, const MatcherConfig & config);
  /// 使用已准备源配准；取消回调在点到面迭代及点批次之间检查。
  MatchResult match(
    const Source & source, const Pose & initial, int frames,
    const std::function<bool()> & cancel = {}) const;
  /// 不透明实现类型，仅在实现文件内定义。
  struct Impl;

private:
  /// 保存不可变目标结构与验收参数。
  PriorMap(std::shared_ptr<const Impl> impl, MatcherConfig config);
  std::shared_ptr<const Impl> impl_;
  MatcherConfig config_;
};
}  // namespace rm27_backend
