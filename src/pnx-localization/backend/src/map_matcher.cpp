#include "rm27_backend/map_matcher.hpp"
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <pcl/io/pcd_io.h>
#include <small_gicp/ann/kdtree.hpp>
#include <small_gicp/ann/kdtree_tbb.hpp>
#include <small_gicp/factors/gicp_factor.hpp>
#include <small_gicp/points/point_cloud.hpp>
#include <small_gicp/registration/reduction_tbb.hpp>
#include <small_gicp/registration/registration.hpp>
#include <small_gicp/util/downsampling_tbb.hpp>
#include <small_gicp/util/normal_estimation_tbb.hpp>
#include <stdexcept>

namespace rm27_backend {
using GicpCloud = small_gicp::PointCloud;
using GicpTree = small_gicp::KdTree<GicpCloud>;
/// 地图实现细节：点云、KD 树和逐点平面标记，构造后只读。
struct PriorMap::Impl {
  std::shared_ptr<GicpCloud> cloud;
  std::shared_ptr<GicpTree> tree;
  std::vector<std::uint8_t> planar;
  std::size_t planar_count = 0;
};
namespace {
/// 校验配准参数的有限性和正值范围，避免运行期出现无意义门限。
void validate(const MatcherConfig &c) {
  for (double x : {c.map_voxel, c.max_plane_ratio, c.source_voxel,
                   c.max_correspondence_distance, c.huber_delta,
                   c.min_inlier_ratio, c.max_rmse, c.residual_std_floor,
                   c.overlap_distance,
                   c.information_scale})
    if (!std::isfinite(x) || x <= 0)
      throw std::invalid_argument("invalid prior map matcher parameter");
  if (c.normal_neighbors < 5 || c.max_iterations < 1 || c.min_inliers < 10 ||
      c.min_coverage_sectors < 0 || c.min_coverage_sectors > 36 ||
      !std::isfinite(c.translation_std_floor) || c.translation_std_floor < 0 ||
      !std::isfinite(c.rotation_std_floor) || c.rotation_std_floor < 0 ||
      !std::isfinite(c.max_condition) || c.min_inlier_ratio > 1)
    throw std::invalid_argument("invalid prior map matcher limits");
}
/// 体素化并建立 KD 树和法向/协方差；平面标记按协方差特征值比判定。
std::shared_ptr<PriorMap::Impl> build(std::vector<Eigen::Vector4d> points,
                                      const MatcherConfig &c) {
  if (points.size() < 100)
    throw std::runtime_error("prior map has too few finite points");
  GicpCloud raw(points);
  auto impl = std::make_shared<PriorMap::Impl>();
  impl->cloud = small_gicp::voxelgrid_sampling_tbb(raw, c.map_voxel);
  impl->tree = std::make_shared<GicpTree>(impl->cloud,
                                          small_gicp::KdTreeBuilderTBB());
  small_gicp::estimate_normals_covariances_tbb(*impl->cloud, *impl->tree,
                                               c.normal_neighbors);
  impl->planar.assign(impl->cloud->size(), 0);
  for (std::size_t i = 0; i < impl->cloud->size(); ++i) {
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> s(
        impl->cloud->cov(i).topLeftCorner<3, 3>());
    const auto l = s.eigenvalues();
    if (s.info() == Eigen::Success && l(2) > 0 &&
        l(0) <= c.max_plane_ratio * l(2) &&
        impl->cloud->normal(i).head<3>().squaredNorm() > .5) {
      impl->planar[i] = 1;
      ++impl->planar_count;
    }
  }
  if (impl->planar_count < 100)
    throw std::runtime_error("prior map has too few planar points");
  return impl;
}
/// 反对称矩阵 [v]×。
Eigen::Matrix3d skew(const Eigen::Vector3d &v) {
  Eigen::Matrix3d m;
  m << 0, -v.z(), v.y(), v.z(), 0, -v.x(), -v.y(), v.x(), 0;
  return m;
}
} // 匿名命名空间
/// 保存只读实现和参数。
PriorMap::PriorMap(std::shared_ptr<const Impl> impl, MatcherConfig config)
    : impl_(std::move(impl)), config_(config) {}
/// 读取 PCD 的 XYZ，丢弃非有限点，施加地图变换后建立目标结构。
std::shared_ptr<const PriorMap> PriorMap::load(const std::string &path,
                                               const Pose &transform,
                                               const MatcherConfig &config) {
  validate(config);
  if (!validPose(transform))
    throw std::invalid_argument("invalid prior map transform");
  pcl::PointCloud<pcl::PointXYZ> cloud;
  if (path.empty() || pcl::io::loadPCDFile(path, cloud) != 0)
    throw std::runtime_error("failed to load prior map: " + path);
  std::vector<Eigen::Vector4d> points;
  points.reserve(cloud.size());
  for (const auto &p : cloud)
    if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z)) {
      const Eigen::Vector3d q = transform * Eigen::Vector3d(p.x, p.y, p.z);
      points.emplace_back(q.x(), q.y(), q.z(), 1.);
    }
  return std::shared_ptr<const PriorMap>(
      new PriorMap(build(std::move(points), config), config));
}
/// 由 map 系点直接建立地图。
std::shared_ptr<const PriorMap>
PriorMap::fromPoints(const std::vector<Eigen::Vector3d> &input,
                     const MatcherConfig &config) {
  validate(config);
  std::vector<Eigen::Vector4d> points;
  points.reserve(input.size());
  for (const auto &p : input)
    if (p.allFinite())
      points.emplace_back(p.x(), p.y(), p.z(), 1.);
  return std::shared_ptr<const PriorMap>(
      new PriorMap(build(std::move(points), config), config));
}
/// 返回体素化后的地图点数。
std::size_t PriorMap::size() const { return impl_->cloud->size(); }
/// 返回平面点数。
std::size_t PriorMap::planarPoints() const { return impl_->planar_count; }
/// GICP 求位姿后，用平面对应重建点到面 Hessian、残差和覆盖度，再生成部分秩信息。
/// 雅可比对应 x = z·Exp([ρ,φ])：∂r/∂ρ = nᵀR，∂r/∂φ = −nᵀR[p]×。
MatchResult PriorMap::match(const Cloud &source, const Pose &initial,
                            int frames) const {
  const auto start = std::chrono::steady_clock::now();
  MatchResult out;
  /// 记录耗时并返回带原因的无效结果。
  const auto finish = [&](const char *reason) {
    out.reason = reason ? reason : "";
    out.valid = reason == nullptr;
    out.milliseconds = std::chrono::duration<double, std::milli>(
                           std::chrono::steady_clock::now() - start)
                           .count();
    return out;
  };
  if (!validPose(initial) || frames < 1)
    return finish("invalid registration initial value");
  std::vector<Eigen::Vector4d> points;
  points.reserve(source.size());
  for (const auto &p : source)
    if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z))
      points.emplace_back(p.x, p.y, p.z, 1.);
  GicpCloud raw(points);
  auto src = small_gicp::voxelgrid_sampling_tbb(raw, config_.source_voxel);
  out.source_points = src->size();
  if (src->size() < std::size_t(config_.min_inliers))
    return finish("too few source points");
  GicpTree source_tree(src, small_gicp::KdTreeBuilderTBB());
  small_gicp::estimate_covariances_tbb(*src, source_tree,
                                       config_.normal_neighbors);
  small_gicp::Registration<small_gicp::GICPFactor,
                           small_gicp::ParallelReductionTBB>
      registration;
  registration.rejector.max_dist_sq = std::pow(config_.max_correspondence_distance, 2);
  registration.optimizer.max_iterations = config_.max_iterations;
  const auto gicp = registration.align(*impl_->cloud, *src, *impl_->tree, initial);
  out.converged = gicp.converged;
  out.iterations = int(gicp.iterations);
  out.pose = gicp.T_target_source;
  out.pose.linear() = Eigen::Quaterniond(out.pose.linear()).normalized().toRotationMatrix();
  if (!validPose(out.pose))
    return finish("registration returned an invalid pose");

  const Eigen::Matrix3d r = out.pose.linear();
  const Eigen::Vector3d origin = out.pose.translation();
  const double max_sq = std::pow(config_.max_correspondence_distance, 2);
  Matrix6 h = Matrix6::Zero();
  double squared = 0;
  std::array<int, 36> sectors{};
  std::size_t overlapping = 0;
  const double overlap_sq = config_.overlap_distance * config_.overlap_distance;
  for (std::size_t i = 0; i < src->size(); ++i) {
    const Eigen::Vector3d p = src->point(i).head<3>();
    const Eigen::Vector3d q = out.pose * p;
    std::size_t index = 0;
    double distance = 0;
    if (!impl_->tree->nearest_neighbor_search(
            Eigen::Vector4d(q.x(), q.y(), q.z(), 1.), &index, &distance))
      continue;
    overlapping += distance <= overlap_sq;
    if (distance > max_sq || !impl_->planar[index])
      continue;
    const Eigen::Vector3d n = impl_->cloud->normal(index).head<3>().normalized();
    const double e = n.dot(q - impl_->cloud->point(index).head<3>());
    if (std::abs(e) > config_.max_correspondence_distance)
      continue;
    Eigen::Matrix<double, 1, 6> j;
    j.leftCols<3>() = n.transpose() * r;
    j.rightCols<3>() = -n.transpose() * r * skew(p);
    const double w = std::abs(e) <= config_.huber_delta
                         ? 1.
                         : config_.huber_delta / std::abs(e);
    h += w * j.transpose() * j;
    squared += e * e;
    ++out.inliers;
    const double azimuth = std::atan2(q.y() - origin.y(), q.x() - origin.x());
    const int sector = std::clamp(int((azimuth + M_PI) / (2 * M_PI) * 36), 0, 35);
    ++sectors[sector];
  }
  for (int count : sectors)
    out.coverage_sectors += count >= 3;
  out.inlier_ratio = double(out.inliers) / double(src->size());
  out.overlap_ratio = double(overlapping) / double(src->size());
  out.rmse = out.inliers ? std::sqrt(squared / double(out.inliers))
                         : std::numeric_limits<double>::infinity();
  if (out.inliers < std::size_t(config_.min_inliers))
    return finish("too few planar correspondences");
  if (out.inlier_ratio < config_.min_inlier_ratio)
    return finish("planar correspondence ratio below threshold");
  if (out.rmse > config_.max_rmse)
    return finish("point-to-plane RMSE above threshold");
  if (out.coverage_sectors < config_.min_coverage_sectors)
    return finish("insufficient horizontal coverage");

  const auto analysis = analyzeHessian(h, config_.max_condition);
  if (!analysis.valid)
    return finish("point-to-plane Hessian is not usable");
  const double n = double(out.inliers);
  const double variance =
      std::max(config_.residual_std_floor * config_.residual_std_floor,
               out.rmse * out.rmse * n / std::max(1., n - analysis.rank));
  auto l = sqrtInformationFromHessian(
      analysis, config_.information_scale / (variance * double(frames)));
  if (!l.size())
    return finish("failed to build square-root information");
  Covariance common = Covariance::Zero();
  common.diagonal().head<3>().setConstant(std::pow(config_.translation_std_floor, 2));
  common.diagonal().tail<3>().setConstant(std::pow(config_.rotation_std_floor, 2));
  const auto bounded = addMeasurementCovariance(l, common);
  if (!bounded)
    return finish("failed to apply common-mode covariance");
  out.sqrt_information = *bounded;
  out.rank = analysis.rank;
  out.condition = analysis.condition;
  return finish(nullptr);
}
} // 命名空间 rm27_backend
