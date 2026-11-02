#include <rog_map/prior_dynamic_obstacles.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <algorithm>
#include <array>
#include <Eigen/Eigenvalues>
#include <atomic>
#include <cmath>
#include <stdexcept>
#include <unordered_set>

namespace rog_map {
namespace {
struct Key {
  int x, y, z;
  bool operator==(const Key &o) const { return x == o.x && y == o.y && z == o.z; }
};
struct Hash {
  size_t operator()(const Key &k) const {
    return std::hash<int>{}(k.x) ^ (std::hash<int>{}(k.y) << 1U) ^
           (std::hash<int>{}(k.z) << 2U);
  }
};
Eigen::Vector2f center(const PriorDynamicObstacles::Track &t) {
  return 0.5F * (t.minimum + t.maximum).head<2>();
}
// A measured ray must pass through the old 3D target and end behind it.
// Absence from the current FOV is not clearing evidence.
bool clearsBox(const Eigen::Vector3f &origin, const Eigen::Vector3f &end,
               const Eigen::Vector3f &low, const Eigen::Vector3f &high) {
  const Eigen::Vector3f delta = end - origin;
  float enter = 0.0F, leave = 1.0F;
  for (int axis = 0; axis < 3; ++axis) {
    if (std::abs(delta[axis]) < 1e-6F) {
      if (origin[axis] < low[axis] || origin[axis] > high[axis]) return false;
      continue;
    }
    float a = (low[axis] - origin[axis]) / delta[axis];
    float b = (high[axis] - origin[axis]) / delta[axis];
    if (a > b) std::swap(a, b);
    enter = std::max(enter, a); leave = std::min(leave, b);
    if (enter > leave) return false;
  }
  return enter > 0.0F && leave < 1.0F && (1.0F - leave) * delta.norm() > 0.15F;
}
// Occupancy is completed only along the normal of a measured vertical face.
// The measured box remains unchanged for association and clearing history.
PriorDynamicObstacles::Box horizontalBox(const PriorDynamicObstacles::Track &track,
    const PriorDynamicConfig &config, double half_cell) {
  const Eigen::Vector2f c = center(track);
  const Eigen::Vector2f extent = (.5F * (track.maximum-track.minimum).head<2>()).cwiseMax(
      static_cast<float>(.5*config.min_extent));
  const Eigen::Vector2f pad = Eigen::Vector2f::Constant(static_cast<float>(half_cell+config.padding));
  return {c+track.completion_shift-extent-pad,
          c+track.completion_shift+extent+pad};
}

bool observedFreeVolume(const PriorDynamicObstacles::Track &track,
    const PriorDynamicConfig &config, const Eigen::Vector3f &origin,
    const std::vector<Eigen::Vector3f> &rays) {
  const auto box = horizontalBox(track,config,0.0);
  const Eigen::Vector3f low(box.minimum.x(),box.minimum.y(),track.minimum.z()-.025F);
  const Eigen::Vector3f high(box.maximum.x(),box.maximum.y(),track.maximum.z()+.025F);
  const Eigen::Vector2f cell = (box.maximum-box.minimum)/3.0F;
  std::array<bool,9> covered{};
  int count=0, coverage=0;
  for(const auto &ray:rays) {
    if(!clearsBox(origin,ray,low,high)) continue;
    ++count;
    for(int i=0;i<9;++i) {
      if(covered[i]) continue;
      Eigen::Vector3f a=low,b=high;
      a.x()+=cell.x()*(i%3); a.y()+=cell.y()*(i/3);
      b.x()=a.x()+cell.x();b.y()=a.y()+cell.y();
      if(clearsBox(origin,ray,a,b)){covered[i]=true;++coverage;}
    }
    // A few rays through one former face do not clear a partially seen car.
    if(count>=config.clear_ray_count && coverage>=7) return true;
  }
  return false;
}
} // namespace

struct PriorDynamicObstacles::Background {
  Cloud::Ptr points{new Cloud};
  pcl::KdTreeFLANN<pcl::PointXYZ> tree;
  pcl::PointCloud<pcl::Normal> normals;
};

void PriorDynamicConfig::validate() const {
  for (double x : {background_voxel, background_distance, live_voxel, cluster_distance,
                   min_span, max_range, association_distance, hold_time,
                   max_speed, min_extent, surface_distance, static_surface_radius, static_surface_distance}) {
    if (!std::isfinite(x) || x <= 0.0) throw std::invalid_argument("vehicle obstacle dimensions/times must be positive");
  }
  for (double x : {padding, prior_boundary_distance, ground_surface_tolerance}) {
    if (!std::isfinite(x) || x < 0.0) throw std::invalid_argument("vehicle obstacle margins must be nonnegative");
  }
  if (!std::isfinite(static_surface_max_curvature) || static_surface_max_curvature <= 0.0 ||
      static_surface_max_curvature >= 1.0/3.0 || normal_neighbors < 3 || !std::isfinite(normal_angle_deg) || normal_angle_deg <= 0 || normal_angle_deg >= 90 ||
      surface_distance > background_distance || static_surface_radius < background_distance ||
      static_surface_distance > background_distance || static_surface_min_support < 3 ||
      static_surface_min_support > 32 || min_points < 3 || clear_ray_count < 1 || clear_frames < 1 ||
      !std::isfinite(min_relative_z) || !std::isfinite(max_relative_z) ||
      max_relative_z <= min_relative_z) throw std::invalid_argument("invalid vehicle obstacle evidence configuration");
}

PriorDynamicObstacles::PriorDynamicObstacles(PriorDynamicConfig config) : config_(config) {
  config_.validate();
}

void PriorDynamicObstacles::setBackground(const Cloud &map) {
  if (map.empty()) throw std::invalid_argument("empty fixed obstacle background");
  auto next = std::make_shared<Background>();
  pcl::VoxelGrid<pcl::PointXYZ> voxel;
  auto finite = Cloud::Ptr(new Cloud);
  finite->reserve(map.size());
  for (const auto &point : map) {
    if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) finite->push_back(point);
  }
  if (finite->empty()) throw std::invalid_argument("fixed background contains no finite points");
  voxel.setInputCloud(finite);
  voxel.setLeafSize(config_.background_voxel, config_.background_voxel, config_.background_voxel);
  voxel.filter(*next->points);
  if (next->points->empty()) throw std::invalid_argument("fixed background contains no valid points");
  next->tree.setInputCloud(next->points);
  pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> normals(4);
  normals.setInputCloud(next->points);
  normals.setKSearch(config_.normal_neighbors);
  normals.compute(next->normals);
  std::atomic_store(&background_, std::shared_ptr<const Background>(next));
}

bool PriorDynamicObstacles::backgroundReady() const {
  return static_cast<bool>(std::atomic_load(&background_));
}

void PriorDynamicObstacles::update(const pcl::PointCloud<pcl::PointXYZI> &cloud,
    const Eigen::Isometry3d &map_from_cloud, const Eigen::Vector3d &sensor_map,
    const Eigen::Vector3d &robot_ground_map, double stamp,
    const std::function<bool(double, double)> &candidate_region,
    const std::function<bool(double, double, double &)> &ground_reference) {
  auto background = std::atomic_load(&background_);
  stats_ = {};
  if (!background || !map_from_cloud.matrix().allFinite() || !sensor_map.allFinite() ||
      !robot_ground_map.allFinite() || !std::isfinite(stamp) || stamp <= 0.0) {
    throw std::invalid_argument("vehicle detection requires a fixed map, valid pose and timestamp");
  }
  if (stamp < last_stamp_ || background != last_background_) tracks_.clear();
  if (stamp == last_stamp_ && background == last_background_) return;
  last_stamp_ = stamp; last_background_ = background;
  auto novel = Cloud::Ptr(new Cloud);
  std::vector<Eigen::Vector3f> rays;
  std::unordered_set<Key, Hash> sampled;
  auto sampled_cloud = Cloud::Ptr(new Cloud);
  std::vector<int> nearest(32); std::vector<float> distances(32);
  stats_.input = cloud.size();
  for (const auto &point : cloud) {
    const Eigen::Vector3d raw(point.x, point.y, point.z);
    if (!raw.allFinite()) continue;
    const Eigen::Vector3d p = map_from_cloud * raw;
    if ((p - sensor_map).norm() > config_.max_range ||
        p.z() < robot_ground_map.z() + config_.min_relative_z ||
        p.z() > robot_ground_map.z() + config_.max_relative_z) continue;
    const Key key{static_cast<int>(std::floor(p.x() / config_.live_voxel)),
                  static_cast<int>(std::floor(p.y() / config_.live_voxel)),
                  static_cast<int>(std::floor(p.z() / config_.live_voxel))};
    if (!sampled.insert(key).second) continue;
    ++stats_.sampled; rays.push_back(p.cast<float>());
    sampled_cloud->emplace_back(p.x(), p.y(), p.z());
  }
  pcl::PointCloud<pcl::Normal> live_normals;
  if (sampled_cloud->size() >= static_cast<size_t>(config_.normal_neighbors)) {
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> estimate(2);
    estimate.setInputCloud(sampled_cloud);
    estimate.setKSearch(config_.normal_neighbors);
    estimate.compute(live_normals);
  }
  const float cos_angle = std::cos(config_.normal_angle_deg * std::acos(-1.0) / 180.0);
  for (size_t i = 0; i < sampled_cloud->size(); ++i) {
    const auto &q = (*sampled_cloud)[i];
    // Surveyed ramp/platform faces below its driving surface are static.
    // This is positive static evidence, never a requirement to observe ground.
    double ground_z = 0.0;
    if (ground_reference && ground_reference(q.x, q.y, ground_z) && std::isfinite(ground_z) &&
        q.z <= ground_z + config_.ground_surface_tolerance) {
      ++stats_.ground_filtered; continue;
    }
    const int found = background->tree.nearestKSearch(q, 32, nearest, distances);
    bool fixed = false;
    int plane_support = 0;
    Eigen::Vector3f live_normal = Eigen::Vector3f::Zero();
    bool live_normal_valid = false;
    if (i < live_normals.size()) {
      const auto &n = live_normals[i];
      live_normal = {n.normal_x, n.normal_y, n.normal_z};
      live_normal_valid = live_normal.allFinite() && live_normal.norm() > .5F && n.curvature < .10F;
      if (live_normal_valid) live_normal.normalize();
    }
    for (int j = 0; j < found; ++j) {
      if (distances[j] > config_.static_surface_radius * config_.static_surface_radius) break;
      // Close coincidences are static even if sparse live normals disagree at
      // a tunnel edge. This replaces the overly strict hardcoded 1.5 cm test.
      if (distances[j] <= config_.surface_distance * config_.surface_distance) {
        fixed = true; break;
      }
      const size_t index = static_cast<size_t>(nearest[j]);
      const auto &normal = background->normals[index];
      Eigen::Vector3f bn(normal.normal_x, normal.normal_y, normal.normal_z);
      if (!bn.allFinite() || bn.norm() < .5F || !std::isfinite(normal.curvature) || normal.curvature > config_.static_surface_max_curvature) continue;
      bn.normalize();
      const auto &bp = (*background->points)[index];
      const Eigen::Vector3f delta(q.x-bp.x, q.y-bp.y, q.z-bp.z);
      const float plane_distance = std::abs(bn.dot(delta));
      // Use distance NORMAL to the fixed surface. A nearby parallel vehicle
      // roof is not background merely because its normals agree with the floor.
      const bool normal_agrees = !live_normal_valid || std::abs(live_normal.dot(bn)) >= cos_angle;
      if (distances[j] <= config_.background_distance * config_.background_distance &&
          live_normal_valid && normal_agrees && plane_distance <= config_.static_surface_distance) {
        fixed = true; ++stats_.surface_filtered; break;
      }
      if (plane_distance <= config_.static_surface_distance &&
          (normal_agrees || plane_distance <= config_.surface_distance) &&
          ++plane_support >= config_.static_surface_min_support) {
        fixed = true; ++stats_.surface_filtered; break;
      }
    }
    if (fixed) { ++stats_.background; continue; }
    if (candidate_region && !candidate_region(q.x, q.y)) { ++stats_.boundary_filtered; continue; }
    novel->push_back(q);
  }

  stats_.novel = novel->size();
  std::vector<Track> detections;
  if (novel->size() >= static_cast<size_t>(config_.min_points)) {
    auto tree = pcl::search::KdTree<pcl::PointXYZ>::Ptr(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(novel);
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> extraction;
    extraction.setClusterTolerance(config_.cluster_distance);
    extraction.setMinClusterSize(config_.min_points);
    extraction.setMaxClusterSize(static_cast<int>(novel->size()));
    extraction.setSearchMethod(tree); extraction.setInputCloud(novel);
    std::vector<pcl::PointIndices> groups; extraction.extract(groups);
    for (const auto &group : groups) {
      Track t;
      t.minimum.setConstant(std::numeric_limits<float>::infinity());
      t.maximum.setConstant(-std::numeric_limits<float>::infinity());
      for (int i : group.indices) {
        const auto &p = (*novel)[i]; const Eigen::Vector3f v(p.x, p.y, p.z);
        t.minimum = t.minimum.cwiseMin(v); t.maximum = t.maximum.cwiseMax(v);
      }
      if ((t.maximum - t.minimum).norm() < config_.min_span) continue;
      // A roof patch or changing tangential view cannot locate the hidden
      // body center. Complete only the normal direction of a resolved vertical
      // face; never translate both axes merely because the view is diagonal.
      Eigen::Vector3f mean=Eigen::Vector3f::Zero();
      for(int i:group.indices) mean += (*novel)[i].getVector3fMap();
      mean /= static_cast<float>(group.indices.size());
      Eigen::Matrix3f covariance=Eigen::Matrix3f::Zero();
      for(int i:group.indices) {
        const Eigen::Vector3f delta=(*novel)[i].getVector3fMap()-mean;
        covariance.noalias() += delta*delta.transpose();
      }
      const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> fit(covariance);
      if(fit.info()==Eigen::Success && fit.eigenvalues().allFinite() &&
          fit.eigenvalues()[1] > 1e-6F &&
          fit.eigenvalues()[0] < .05F*fit.eigenvalues().sum()) {
        const Eigen::Vector3f normal=fit.eigenvectors().col(0);
        if(std::abs(normal.z()) < .5F) {
          int axis=std::abs(normal.x())>=std::abs(normal.y()) ? 0 : 1;
          const float span=t.maximum[axis]-t.minimum[axis];
          const Eigen::Vector2f view=center(t)-sensor_map.head<2>().cast<float>();
          if(std::abs(normal[axis])>.85F && span < .5*config_.min_extent &&
              std::abs(view[axis])>.05F) {
            t.completion_shift[axis]=.5F*(config_.min_extent-span)*(view[axis]>0 ? 1.0F : -1.0F);
          }
        }
      }
      // A broad vertical fascia plus its top mixes two planes, so PCA may
      // pick the top normal. If a complete long face and appreciable height
      // are observed, anchor completion to its measured near edge instead of
      // inventing half a vehicle towards the robot. Thin roof strips do not
      // satisfy the height condition and keep the centered footprint.
      const Eigen::Vector3f span=t.maximum-t.minimum;
      if(t.completion_shift.isZero(1e-6F) && span.z()>=.06F) {
        const int axis=span.x()<span.y() ? 0 : 1;
        const int tangent=1-axis;
        const Eigen::Vector2f view=center(t)-sensor_map.head<2>().cast<float>();
        if(span[axis]<.5*config_.min_extent && span[tangent]>=config_.min_extent &&
            std::abs(view[axis])>.05F) {
          t.completion_shift[axis]=.5F*(config_.min_extent-span[axis])*(view[axis]>0 ? 1.0F : -1.0F);
        }
      }
      t.seen = stamp; t.observations = 1; detections.push_back(t);
    }
  }
  stats_.clusters = detections.size();
  const size_t previous_count = tracks_.size();
  std::vector<bool> matched(previous_count, false);
  for (auto &detection : detections) {
    size_t best = previous_count;
    double best_distance = config_.association_distance;
    for (size_t i = 0; i < previous_count; ++i) {
      if (matched[i] || stamp - tracks_[i].seen > config_.hold_time) continue;
      const double distance = (center(tracks_[i]) - center(detection)).norm();
      const double reachable = config_.max_speed * (stamp - tracks_[i].seen) + .15;
      if (distance > reachable) continue;
      if (distance < best_distance) { best = i; best_distance = distance; }
    }
    if (best == previous_count) {
      detection.id = next_id_++; tracks_.push_back(detection); continue;
    }
    auto &old = tracks_[best]; matched[best] = true;
    detection.id = old.id; detection.observations = old.observations + 1;
    old = detection;
  }
  for (size_t i = 0; i < previous_count; ++i) {
    auto &track = tracks_[i];
    if (matched[i]) continue;
    track.clear_frames = observedFreeVolume(track, config_, sensor_map.cast<float>(), rays) ?
        track.clear_frames + 1 : 0;
  }
  tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(), [&](const Track &t) {
    if (t.clear_frames >= config_.clear_frames) { ++stats_.ray_cleared; return true; }
    return stamp - t.seen > config_.hold_time;
  }), tracks_.end());
  stats_.tracks = tracks_.size();
}

bool PriorDynamicObstacles::occupied(const Eigen::Vector2d &p, double half_cell,
    double stamp, float *low_z, float *high_z) const {
  for (const auto &track : tracks_) {
    const double age = stamp - track.seen;
    if (age < 0.0 || age > config_.hold_time) continue;
    const auto box = horizontalBox(track,config_,half_cell);
    if ((p.cast<float>().array() >= box.minimum.array()).all() &&
        (p.cast<float>().array() <= box.maximum.array()).all()) {
      if (low_z) *low_z = track.minimum.z();
      if (high_z) *high_z = track.maximum.z();
      return true;
    }
  }
  return false;
}
std::vector<PriorDynamicObstacles::Box> PriorDynamicObstacles::boxes(double stamp, double half_cell) const {
  std::vector<Box> result;
  for (const auto &track : tracks_) {
    const double age = stamp - track.seen;
    if (age < 0.0 || age > config_.hold_time) continue;
    result.push_back(horizontalBox(track,config_,half_cell));
  }
  return result;
}
} // namespace rog_map
