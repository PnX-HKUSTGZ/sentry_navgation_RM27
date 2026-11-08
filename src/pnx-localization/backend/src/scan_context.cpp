#include "rm27_backend/scan_context.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <pcl/filters/voxel_grid.h>

namespace rm27_backend {
namespace {
/// 生成照搬 z-localization 顺序的横向平移增广序列。
std::vector<Eigen::Vector2d> shifts(const ScanContextConfig &c) {
  const double s = std::max(0., c.lateral_shift);
  std::vector<Eigen::Vector2d> out{{0, 0},  {0, -s}, {0, s}, {-s, -s},
                                   {s, -s}, {-s, s}, {s, s}};
  out.resize(std::clamp(c.augment_count, 1, int(out.size())));
  return out;
}
/// 用扇区键的循环余弦相似度确定粗列移。
int bestSectorAlignment(const Eigen::VectorXf &q, const Eigen::VectorXf &c) {
  const int n = int(q.size());
  if (n <= 0 || c.size() != q.size())
    return 0;
  int best = 0;
  double score_best = -std::numeric_limits<double>::infinity();
  for (int shift = 0; shift < n; ++shift) {
    double dot = 0, qn = 0, cn = 0;
    for (int i = 0; i < n; ++i) {
      const double a = q((i + shift) % n), b = c(i);
      dot += a * b;
      qn += a * a;
      cn += b * b;
    }
    const double score = qn > 0 && cn > 0 ? dot / std::sqrt(qn * cn) : 0.;
    if (score > score_best) {
      score_best = score;
      best = shift;
    }
  }
  return best;
}
/// 角度归一化到 (−π, π]。
double normalize(double a) { return std::atan2(std::sin(a), std::cos(a)); }
} // 匿名命名空间
/// 按 z-localization 的下限规范化参数。
ScanContextDatabase::ScanContextDatabase(ScanContextConfig c) : config_(c) {
  config_.num_rings = std::max(1, c.num_rings);
  config_.num_sectors = std::max(1, c.num_sectors);
  config_.max_radius = std::max(1., c.max_radius);
  config_.downsample = std::max(0., c.downsample);
  config_.min_bin_points = std::max(1, c.min_bin_points);
  config_.augment_count = std::clamp(c.augment_count, 1, 7);
  config_.search_ratio = std::clamp(c.search_ratio, 0., 1.);
  config_.exclude_recent = std::max(1, c.exclude_recent);
  config_.candidates = std::max(1, c.candidates);
  config_.distance_threshold = std::max(0., c.distance_threshold);
}
/// 环按水平半径、扇区按方位角划分，每格记录 z+lidar_height 的最大值，空格为 0。
ScanContext ScanContextDatabase::make(const Cloud &cloud,
                                      const Eigen::Vector2d &shift) const {
  ScanContext d;
  d.image = Eigen::MatrixXf::Zero(config_.num_rings, config_.num_sectors);
  Eigen::MatrixXi counts =
      Eigen::MatrixXi::Zero(config_.num_rings, config_.num_sectors);
  Eigen::MatrixXf max_z = Eigen::MatrixXf::Constant(
      config_.num_rings, config_.num_sectors, -std::numeric_limits<float>::max());
  const double ring_gap = config_.max_radius / config_.num_rings;
  const double sector_gap = 2 * M_PI / config_.num_sectors;
  for (const auto &p : cloud) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
      continue;
    const double x = p.x - shift.x(), y = p.y - shift.y();
    const double r = std::hypot(x, y);
    if (r <= 0 || r > config_.max_radius)
      continue;
    const int ring = std::clamp(int(std::floor(r / ring_gap)), 0, config_.num_rings - 1);
    double theta = std::atan2(y, x);
    if (theta < 0)
      theta += 2 * M_PI;
    const int sector =
        std::clamp(int(std::floor(theta / sector_gap)), 0, config_.num_sectors - 1);
    ++counts(ring, sector);
    max_z(ring, sector) =
        std::max(max_z(ring, sector), float(p.z + config_.lidar_height));
  }
  for (int r = 0; r < config_.num_rings; ++r)
    for (int s = 0; s < config_.num_sectors; ++s)
      if (counts(r, s) >= config_.min_bin_points)
        d.image(r, s) = max_z(r, s);
  d.sector_key = d.image.colwise().mean().transpose();
  return d;
}
/// 体素化一次后生成全部增广描述子并追加。
int ScanContextDatabase::add(const Cloud &leveled) {
  Cloud sampled;
  if (config_.downsample > 0) {
    pcl::VoxelGrid<pcl::PointXYZI> grid;
    grid.setInputCloud(leveled.makeShared());
    const float leaf = float(config_.downsample);
    grid.setLeafSize(leaf, leaf, leaf);
    grid.filter(sampled);
  } else {
    sampled = leveled;
  }
  std::vector<ScanContext> variants;
  for (const auto &shift : shifts(config_))
    variants.push_back(make(sampled, shift));
  entries_.push_back(std::move(variants));
  return int(entries_.size()) - 1;
}
/// 先用扇区键定粗列移，再在 search_ratio 范围内逐列移比较有效列的平均余弦相似度。
std::pair<double, int> ScanContextDatabase::distance(const ScanContext &q,
                                                     const ScanContext &c) const {
  const int sectors = config_.num_sectors, rings = config_.num_rings;
  if (q.image.rows() != rings || c.image.rows() != rings ||
      q.image.cols() != sectors || c.image.cols() != sectors)
    return {std::numeric_limits<double>::max(), 0};
  const int init = bestSectorAlignment(q.sector_key, c.sector_key);
  // 搜索半径覆盖整周时只遍历一次全部列移，避免重复。
  const int radius = std::min(sectors / 2, std::max(0, int(std::ceil(.5 * config_.search_ratio * sectors))));
  const Eigen::VectorXf qn = q.image.colwise().norm().transpose();
  const Eigen::VectorXf cn = c.image.colwise().norm().transpose();
  double best = -std::numeric_limits<double>::infinity();
  int best_shift = init;
  const int first = -radius, last = 2 * radius + 1 > sectors ? first + sectors - 1 : radius;
  for (int offset = first; offset <= last; ++offset) {
    const int shift = ((init + offset) % sectors + sectors) % sectors;
    double sum = 0;
    int engaged = 0;
    // 只计算本列移需要的列点积，不再预先求完整的扇区×扇区点积矩阵。
    for (int s = 0; s < sectors; ++s) {
      const int qs = (s + shift) % sectors;
      if (qn[qs] <= 0 || cn[s] <= 0)
        continue;
      sum += double(q.image.col(qs).dot(c.image.col(s))) / (double(qn[qs]) * double(cn[s]));
      ++engaged;
    }
    const double sim = engaged ? sum / engaged : 0.;
    if (sim > best) {
      best = sim;
      best_shift = shift;
    }
  }
  return {1. - best, best_shift};
}
/// 最新帧只用其原始描述子查询，候选取所有增广中的最小距离；列移取反即 source→target 偏航。
std::vector<ScanContextCandidate>
ScanContextDatabase::detectLatest(const std::vector<int> &eligible) const {
  std::vector<ScanContextCandidate> out;
  if (entries_.size() < 2 || entries_.back().empty())
    return out;
  const int latest = int(entries_.size()) - 1;
  const int cutoff = std::max(0, latest - config_.exclude_recent + 1);
  const ScanContext &query = entries_.back().front();
  for (int index : eligible) {
    if (index < 0 || index >= latest || index >= cutoff)
      continue;
    ScanContextCandidate best;
    best.index = index;
    best.distance = std::numeric_limits<double>::infinity();
    for (const auto &variant : entries_[index]) {
      const auto [d, shift] = distance(query, variant);
      if (d < best.distance) {
        best.distance = d;
        best.yaw = normalize(-double(shift) * 2 * M_PI / config_.num_sectors);
      }
    }
    if (best.distance < config_.distance_threshold)
      out.push_back(best);
  }
  std::sort(out.begin(), out.end(), [](const auto &a, const auto &b) {
    return a.distance == b.distance ? a.index < b.index : a.distance < b.distance;
  });
  if (int(out.size()) > config_.candidates)
    out.resize(config_.candidates);
  return out;
}
/// 由旋转矩阵第一列求偏航。
double yawOf(const Eigen::Matrix3d &r) { return std::atan2(r(1, 0), r(0, 0)); }
/// 保留重力方向、去掉偏航，使描述子在水平面内编码。
Pose levelFromBody(const Pose &world_from_body) {
  const Eigen::Matrix3d r = world_from_body.linear();
  Pose out = Pose::Identity();
  out.linear() =
      Eigen::AngleAxisd(yawOf(r), Eigen::Vector3d::UnitZ()).toRotationMatrix().transpose() * r;
  return out;
}
/// 偏航初值作用在整平坐标中，再用两端整平旋转换回原始 IMU 局部系。
Pose descriptorYawToSourceTarget(double yaw, const Eigen::Vector3d &translation,
                                 const Pose &source_world, const Pose &target_world) {
  Pose guess = Pose::Identity();
  guess.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  guess.translation() = translation;
  guess.linear() = levelFromBody(target_world).linear().transpose() * guess.linear() *
                   levelFromBody(source_world).linear();
  return guess;
}
} // 命名空间 rm27_backend
