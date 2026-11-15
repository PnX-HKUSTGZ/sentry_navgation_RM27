#include "relocalization_manager/recovery_engine.hpp"

#include <openssl/evp.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <yaml-cpp/yaml.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <numeric>
#include <sstream>

namespace rm27_backend
{
namespace
{
/// 校验产物内容哈希，避免地图与局部云不同版本但编号仍相同。
void verifyDigest(const std::filesystem::path & path, const std::string & expected)
{
  std::ifstream stream(path, std::ios::binary);
  if (!stream || expected.size() != 64) throw std::runtime_error("missing bundle member or digest");
  std::unique_ptr<EVP_MD_CTX, decltype(&EVP_MD_CTX_free)> ctx(EVP_MD_CTX_new(), EVP_MD_CTX_free);
  if (!ctx || EVP_DigestInit_ex(ctx.get(), EVP_sha256(), nullptr) != 1)
    throw std::runtime_error("SHA256 initialization failed");
  char buffer[65536];
  while (stream) {
    stream.read(buffer, sizeof(buffer));
    EVP_DigestUpdate(ctx.get(), buffer, stream.gcount());
  }
  unsigned char digest[EVP_MAX_MD_SIZE];
  unsigned int size = 0;
  EVP_DigestFinal_ex(ctx.get(), digest, &size);
  std::ostringstream out;
  for (unsigned int i = 0; i < size; ++i)
    out << std::hex << std::setw(2) << std::setfill('0') << int(digest[i]);
  if (out.str() != expected) throw std::runtime_error("bundle digest mismatch: " + path.string());
}
/// 拒绝包外路径，防止配置意外读取其他地图的几何。
std::filesystem::path member(const std::filesystem::path & root, const std::string & name)
{
  const std::filesystem::path p(name);
  if (p.empty() || p.is_absolute()) throw std::runtime_error("invalid bundle member");
  for (const auto & part : p)
    if (part == "..") throw std::runtime_error("bundle path escapes root");
  return root / p;
}
}  // namespace
/// 加载地图包；每个参考视点的局部云用于 SC，配准目标从同版全局地图裁剪。
RecoveryEngine::RecoveryEngine(
  const std::string & bundle, const Pose & alignment, MatcherConfig m, ScanContextConfig sc,
  RecoveryConfig c)
: matcher_(m), config_(c), descriptors_(sc)
{
  if (
    c.hint_yaw_samples < 1 || c.hint_yaw_samples > 9 || c.hint_yaw_samples % 2 == 0 ||
    !std::isfinite(c.hint_yaw_step) || c.hint_yaw_step <= 0 ||
    !std::isfinite(c.duplicate_distance) || c.duplicate_distance <= 0 ||
    c.duplicate_distance >= c.ambiguity_distance || !std::isfinite(c.duplicate_angle) ||
    c.duplicate_angle <= 0 || c.duplicate_angle >= c.ambiguity_angle_rad)
    throw std::invalid_argument("invalid recovery hint or duplicate limits");

  if (
    c.candidates < 1 || c.candidates > 64 || c.yaw_samples < 1 || c.yaw_samples > 36 ||
    c.cache_targets < 1 || c.cache_targets > 64 || !std::isfinite(c.target_radius) ||
    c.target_radius <= 0 || !std::isfinite(c.suspect_radius) || c.suspect_radius <= 0 ||
    !std::isfinite(c.max_seconds) || c.max_seconds <= 0 || !std::isfinite(c.ambiguity_ratio) ||
    c.ambiguity_ratio <= 0 || c.ambiguity_ratio >= 1 || !std::isfinite(c.ambiguity_angle_rad) ||
    c.ambiguity_angle_rad <= 0 || !std::isfinite(c.ambiguity_distance) ||
    c.ambiguity_distance <= 0 || !std::isfinite(c.min_overlap) || c.min_overlap <= 0 ||
    c.min_overlap > 1)
    throw std::invalid_argument("invalid recovery parameters");
  matcher_.point_to_plane = true;
  const std::filesystem::path root(bundle);
  const auto manifest = YAML::LoadFile((root / "manifest.yaml").string());
  if (
    manifest["format"].as<std::string>() != "rm27_reference_map_v1" ||
    manifest["frame"].as<std::string>() != "map")
    throw std::runtime_error("unsupported bundle format/frame");
  if (!c.source_pcd.empty())
    verifyDigest(c.source_pcd, manifest["source_pcd_sha256"].as<std::string>());
  if (!c.navigation_grid.empty()) {
    verifyDigest(c.navigation_grid, manifest["grid_yaml_sha256"].as<std::string>());
    const auto grid = YAML::LoadFile(c.navigation_grid);
    verifyDigest(
      std::filesystem::path(c.navigation_grid).parent_path() / grid["image"].as<std::string>(),
      manifest["grid_image_sha256"].as<std::string>());
  }
  const Pose recorded = poseFromVector(manifest["map_from_pcd"].as<std::vector<double>>());
  if (!recorded.matrix().isApprox(alignment.matrix(), 1e-7))
    throw std::runtime_error("bundle and runtime map_from_pcd differ");
  const double coverage = manifest["coverage"]["p95_m"].as<double>();
  if (!std::isfinite(coverage) || coverage > matcher_.max_correspondence_distance)
    throw std::runtime_error(
      "reference map undercovers navigable space; reduce view spacing and regenerate");
  version_ = manifest["version"].as<std::string>();
  verifyDigest(
    member(root, manifest["map"].as<std::string>()), manifest["map_sha256"].as<std::string>());
  pcl::PointCloud<pcl::PointXYZ> map;
  if (
    pcl::io::loadPCDFile(member(root, manifest["map"].as<std::string>()).string(), map) != 0 ||
    map.size() > 10000000)
    throw std::runtime_error("invalid bundle map");
  for (const auto & p : map)
    if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z))
      points_.emplace_back(p.x, p.y, p.z);
  verification_map_ = PriorMap::fromPoints(points_, matcher_);
  const auto views = manifest["views"];
  if (!views.IsSequence() || views.size() == 0 || views.size() > 5000 || points_.size() < 100)
    throw std::runtime_error("invalid bundle resources");
  for (const auto & view : views) {
    views_.push_back(poseFromVector(view["pose"].as<std::vector<double>>()));
    verifyDigest(member(root, view["cloud"].as<std::string>()), view["sha256"].as<std::string>());
    Cloud cloud;
    if (
      pcl::io::loadPCDFile(member(root, view["cloud"].as<std::string>()).string(), cloud) != 0 ||
      cloud.size() < 100 || cloud.size() > 200000)
      throw std::runtime_error("invalid reference cloud");
    Cloud level;
    pcl::transformPointCloud(cloud, level, levelFromBody(views_.back()).matrix().cast<float>());
    descriptors_.add(level);
  }
}
/// 背景云保持地图包的 map 坐标和体素精度，由后台构造一次。
Cloud::ConstPtr RecoveryEngine::background() const
{
  auto cloud = std::make_shared<Cloud>();
  cloud->reserve(points_.size());
  for (const auto & p : points_) {
    pcl::PointXYZI q;
    q.x = p.x();
    q.y = p.y();
    q.z = p.z();
    q.intensity = 0;
    cloud->push_back(q);
  }
  return cloud;
}
/// 欧氏距离选择最近视点，索引顺序用于等距离时稳定排序。
int RecoveryEngine::nearest(const Eigen::Vector3d & p) const
{
  int best = 0;
  for (int i = 1; i < int(views_.size()); ++i)
    if (
      (views_[i].translation() - p).squaredNorm() < (views_[best].translation() - p).squaredNorm())
      best = i;
  return best;
}
/// 缓存按 FIFO 限制，目标点保持 map 系，因此跟踪和恢复返回 map←IMU。
std::shared_ptr<const PriorMap> RecoveryEngine::target(int index)
{
  for (const auto & entry : cache_)
    if (entry.first == index) return entry.second;
  std::vector<Eigen::Vector3d> points;
  const double sq = config_.target_radius * config_.target_radius;
  for (const auto & p : points_)
    if ((p - views_.at(index).translation()).squaredNorm() <= sq) points.push_back(p);
  auto out = PriorMap::fromPoints(points, matcher_);
  cache_.emplace_back(index, out);
  while (cache_.size() > std::size_t(config_.cache_targets)) cache_.pop_front();
  return out;
}
/// 连续跟踪只使用预测位置附近目标，不查询全图描述子。
MatchResult RecoveryEngine::track(
  const Cloud & source, const Pose & prediction, int frames, const std::function<bool()> & cancel)
{
  return target(nearest(prediction.translation()))
    ->match(*PriorMap::prepareSource(source, matcher_), prediction, frames, cancel);
}
/// 独立恢复：提示优先，SC 候选逐一验证；不同地点的近似等优解拒绝自动提交。
RecoveryEstimate RecoveryEngine::recover(
  const Cloud & source, const Pose & odom, int frames, const std::optional<Pose> & hint,
  const std::optional<Pose> & near, const std::function<bool()> & cancel, HintLimits hint_limits,
  const std::optional<RecoveryRegion> & region)
{
  RecoveryEstimate out;
  if (
    region &&
    (!validPose(region->pose) || !std::isfinite(region->radius) || !std::isfinite(region->angle) ||
     region->radius <= 0 || region->angle <= 0 || region->angle >= M_PI))
    throw std::invalid_argument("invalid motion recovery region");
  const auto start = std::chrono::steady_clock::now();
  /// 超时和新代次都尽快停止当前迭代，不能把尚未搜索的候选当作已经排除。
  const auto stop = [&] {
    return (cancel && cancel()) ||
           std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count() >=
             config_.max_seconds;
  };
  /// 终止结果只作调度反馈，不允许提交已搜索到的局部最佳值。
  const auto interrupted = [&] {
    out.found = false;
    out.deferred = true;
    out.reason = cancel && cancel() ? "recovery cancelled" : "recovery budget exhausted";
    return out;
  };
  if (stop()) return interrupted();
  const auto prepared = PriorMap::prepareSource(source, matcher_);
  if (stop()) return interrupted();
  /// 验收完整六维锚，保留方差信息，不能以部分秩观测宣布恢复。
  const auto acceptable = [&](const MatchResult & m) {
    return m.valid && m.converged && m.rank == 6 && (!region || region->contains(m.pose)) &&
           m.overlap_ratio >= config_.min_overlap &&
           covarianceFromSqrtInformation(m.sqrt_information).has_value();
  };
  if (region && !hint) {
    // 连续性提供了有限全局位置与航向，先验只用于初值和验收范围，不写图。
    auto local =
      target(nearest(region->pose.translation()))->match(*prepared, region->pose, frames, stop);
    ++out.attempts;
    if (stop()) return interrupted();
    if (acceptable(local)) {
      auto verified = verification_map_->match(*prepared, local.pose, frames, stop);
      ++out.attempts;
      if (stop()) return interrupted();
      if (acceptable(verified)) {
        out.match = std::move(verified);
        out.found = true;
        out.reason = "motion continuity + verified point-to-plane";
        return out;
      }
    }
  }
  if (hint) {
    for (int i = 0; i < config_.hint_yaw_samples; ++i) {
      if (stop()) return interrupted();
      const int offset = i == 0 ? 0 : (i % 2 ? (i + 1) / 2 : -i / 2);
      Pose seed = *hint;
      seed.linear() = Eigen::AngleAxisd(offset * config_.hint_yaw_step, Eigen::Vector3d::UnitZ())
                        .toRotationMatrix() *
                      hint->linear();
      auto m = target(nearest(seed.translation()))->match(*prepared, seed, frames, stop);
      ++out.attempts;
      if (stop()) return interrupted();
      const Pose delta = hint->inverse() * m.pose;
      if (
        acceptable(m) && delta.translation().norm() < hint_limits.translation &&
        angle(delta) < hint_limits.rotation) {
        out.match = m;
        out.found = true;
        out.reason = "validated initial pose";
        return out;
      }
    }
    if (!config_.hint_global_fallback) {
      out.reason = "initial pose rejected: no valid match inside hint bounds";
      return out;
    }
  }
  Cloud level;
  pcl::transformPointCloud(source, level, levelFromBody(odom).matrix().cast<float>());
  std::vector<int> eligible;
  for (int i = 0; i < int(views_.size()); ++i)
    if (
      (!near || (views_[i].translation() - near->translation()).norm() <= config_.suspect_radius) &&
      (!region ||
       (views_[i].translation() - region->pose.translation()).norm() <= region->radius + 1.))
      eligible.push_back(i);
  const auto ranked = descriptors_.query(level, eligible, int(eligible.size()));
  std::vector<ScanContextCandidate> candidates;
  // 地点覆盖优先，避免前 N 个相邻视点挤掉对称地点，造成虚假的唯一解。
  for (const auto & hit : ranked) {
    bool separated = true;
    for (const auto & chosen : candidates)
      if (
        (views_[hit.index].translation() - views_[chosen.index].translation()).norm() <
        config_.ambiguity_distance)
        separated = false;
    if (separated) candidates.push_back(hit);
    if (candidates.size() >= std::size_t(config_.candidates)) break;
  }
  for (const auto & hit : ranked) {
    if (candidates.size() >= std::size_t(config_.candidates)) break;
    if (std::none_of(candidates.begin(), candidates.end(), [&](const auto & x) {
          return x.index == hit.index;
        }))
      candidates.push_back(hit);
  }
  std::vector<std::pair<int, MatchResult>> good;
  // 优先覆盖各地点的 SC 航向，随后再扩展整周假设；保留全部竞争地点的验收。
  for (int yaw = 0; yaw < config_.yaw_samples; ++yaw) {
    for (const auto & hit : candidates) {
      if (stop()) return interrupted();
      Pose seed = views_[hit.index] * descriptorYawToSourceTarget(
                                        hit.yaw + 2 * M_PI * yaw / config_.yaw_samples,
                                        Eigen::Vector3d::Zero(), odom, views_[hit.index]);
      // 参考视点允许覆盖边缘余量；进入配准前把位置初值收回真实包络内。
      if (region) {
        const Pose rotation_delta = region->pose.inverse() * seed;
        if (angle(rotation_delta) > region->angle) continue;
        Eigen::Vector3d delta = seed.translation() - region->pose.translation();
        if (delta.norm() > region->radius)
          seed.translation() = region->pose.translation() + delta.normalized() * region->radius;
      }
      auto m = target(hit.index)->match(*prepared, seed, frames, stop);
      ++out.attempts;
      if (stop()) return interrupted();
      if (!acceptable(m)) continue;
      // 已收敛到同一几何盆地的候选无需反复访问完整地图。
      const bool duplicate = std::any_of(good.begin(), good.end(), [&](const auto & v) {
        const Pose delta = v.second.pose.inverse() * m.pose;
        return delta.translation().norm() < config_.duplicate_distance &&
               angle(delta) < config_.duplicate_angle;
      });
      if (duplicate) continue;
      auto refined = verification_map_->match(*prepared, m.pose, frames, stop);
      if (stop()) return interrupted();
      if (!acceptable(refined)) continue;
      good.emplace_back(hit.index, std::move(refined));
      // 两个不同位置均达到噪声下限时，继续搜索也无法通过既有歧义门限。
      // 只有这一充分条件允许提前拒绝，不能因单个好候选提前接受。
      for (std::size_t i = 0; i + 1 < good.size(); ++i) {
        const auto & a = good[i].second;
        const auto & b = good.back().second;
        const Pose delta = a.pose.inverse() * b.pose;
        if (
          (delta.translation().norm() > config_.ambiguity_distance ||
           angle(delta) > config_.ambiguity_angle_rad) &&
          a.rmse <= matcher_.residual_std_floor && b.rmse <= matcher_.residual_std_floor) {
          out.reason = "ambiguous distinct map locations at residual noise floor";
          return out;
        }
      }
    }
  }
  std::sort(good.begin(), good.end(), [](const auto & a, const auto & b) {
    return a.second.rmse < b.second.rmse;
  });
  if (good.empty()) {
    out.reason =
      region ? "no verified candidate inside motion envelope" : "no verified global candidate";
    return out;
  }
  const auto & best = good.front();
  std::ostringstream evidence;
  for (const auto & candidate : good)
    evidence << candidate.first << ":" << candidate.second.pose.translation().transpose() << ":"
             << candidate.second.rmse << ";";
  out.reason = evidence.str();
  for (std::size_t i = 1; i < good.size(); ++i) {
    const Pose separation = best.second.pose.inverse() * good[i].second.pose;
    const bool distinct = separation.translation().norm() > config_.ambiguity_distance ||
                          angle(separation) > config_.ambiguity_angle_rad;
    if (
      distinct && (best.second.rmse >= config_.ambiguity_ratio * good[i].second.rmse ||
                   good[i].second.rmse - best.second.rmse <= matcher_.residual_std_floor)) {
      out.reason = "ambiguous distinct map locations: " + evidence.str();
      return out;
    }
  }
  out.found = true;
  out.match = best.second;
  out.candidate = best.first;
  out.reason = "scan context + point-to-plane: " + evidence.str();
  return out;
}
}  // namespace rm27_backend
