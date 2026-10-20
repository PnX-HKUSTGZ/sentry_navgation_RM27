// Copyright 2026 RM27 contributors. SPDX-License-Identifier: Apache-2.0
#pragma once
#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace global_relocalization {
using Cloud = pcl::PointCloud<pcl::PointXYZ>;
struct Params {
  std::string search_side{"all"};
  std::string diagnostic_output_dir;
  double side_boundary_x{0.0};
  int sc_rings{20}, sc_sectors{60}, top_k{5}, ring_key_candidates{0};
  bool retrieval_all_views{false};
  bool deterministic_preprocessing{false};
  double db_grid_resolution{2.0}, db_base_z{0.0}, db_max_radius{25.0};
  int db_min_points{80};
  double db_min_elevation_deg{-40.0}, db_max_elevation_deg{85.0};
  double gicp_leaf_size{0.15}, gicp_max_correspondence{1.0};
  int gicp_max_iterations{40};
  // At most two candidate tasks, each using four OpenMP registration threads.
  int candidate_workers{1};
  size_t minimum_confirmation_scans{1};
  bool bounded_global_correspondences{false};
  double max_fitness{0.20}, min_inlier_ratio{0.65}, ambiguity_margin{0.15};
  double confirmation_max_translation{0.15}, confirmation_max_rotation{0.0872664626};
};
// Coarse experiment prior only; tests the base pose, not the lidar origin.
bool poseInSearchSide(const Eigen::Isometry3d & base_pose,const Params & params);
struct Descriptor { Eigen::MatrixXd cells; Eigen::VectorXd ring_key; };
Descriptor scanContext(const Cloud & cloud, int rings, int sectors, double radius);
// Positive yaw maps query coordinates into target coordinates.
std::pair<double, double> descriptorMatch(const Descriptor & query, const Descriptor & target);
Eigen::Isometry3d basePoseFromSensorView(const Eigen::Vector3d & sensor_in_map,
  const Eigen::Vector3d & sensor_in_base, double base_yaw);
Eigen::Isometry3d composeCurrentBasePose(
  const Eigen::Isometry3d & map_from_reference, const Eigen::Isometry3d & odom_from_reference,
  const Eigen::Isometry3d & odom_from_current);
std::string sha256File(const std::string & path);
struct Candidate {
  Eigen::Isometry3d map_from_reference{Eigen::Isometry3d::Identity()};
  double descriptor_distance{1.0}, fitness{1e9}, inlier_ratio{0.0};
  size_t ring_key_rank{0}, scan_context_rank{0};
  bool from_scan_context{false}, from_ring_key{false};
  Eigen::Vector3d seed_translation{Eigen::Vector3d::Zero()};
  double seed_yaw{0.0};
  bool converged{false};
  bool within_search_side{true};
  size_t inliers{0};
  size_t source_points{0}, coarse_source_points{0};
  size_t coarse_iterations{0}, fine_iterations{0};
  double coarse_sec{0.0}, fine_sec{0.0}, quality_sec{0.0}, total_sec{0.0};
  bool canceled{false};
};
std::vector<Candidate> selectRetrievalCandidates(std::vector<Candidate> candidates,
  int scan_context_count, int ring_key_count);
struct SearchResult {
  bool success{false};
  std::string reason;
  Candidate candidate;
  double retrieval_sec{0.0}, gicp_sec{0.0}, confirmation_sec{0.0};
  size_t database_size{0}, candidates_tested{0}, retrieval_scored_views{0}, retrieval_candidate_count{0}, retrieval_ring_added{0};
  // Preparation counts exclude target-map preprocessing and never depend on candidate count.
  size_t query_preparations{0}, independent_preparations{0};
  int candidate_workers{1};
  std::string map_hash;
  std::vector<Candidate> hypotheses;
  std::vector<Candidate> confirmations;
};
// Stage-boundary observation only: small_gicp exposes no per-iteration callback.
// Calls are serialized, synchronous, and exceptions are isolated from the solver.
struct SearchEvent {
  std::string stage;
  size_t index{0}, total{0};
  Candidate candidate;
  std::vector<Candidate> seeds;
  const Cloud * scan{nullptr}; // Valid only for the duration of the callback.
  double best_translation_delta{0.0},best_rotation_delta{0.0};
  double previous_translation_delta{0.0},previous_rotation_delta{0.0};
};
using SearchObserver = std::function<void(const SearchEvent &)>;
// An opaque, single-use continuation. Readiness never represents localization
// success; only confirm() may produce a successful SearchResult. Copies share
// consumption and provenance. A new preparation or map/database invalidates it.
class PreparedSearch {
public:
  PreparedSearch() = default;
  bool ready() const;
  const SearchResult & diagnostics() const;
private:
  struct State;
  std::shared_ptr<State> state_;
  friend class Core;
};
class Core {
public:
  explicit Core(const Params & params);
  ~Core();
  void loadMap(const std::string & path);
  void buildDatabase(const Eigen::Vector3d & base_to_lidar_translation);
  // Calls on one Core must be serialized, including loadMap/buildDatabase.
  PreparedSearch prepare(const Cloud & query_reference,
    const Eigen::Vector3d & base_to_lidar_translation, const std::function<bool()> & canceled,
    const SearchObserver & observer = {});
  SearchResult confirm(const PreparedSearch & prepared,
    const std::vector<Cloud> & independent_scans_reference, const std::function<bool()> & canceled,
    const SearchObserver & observer = {});
  SearchResult search(const Cloud & query_reference, const Cloud & independent_scan_reference,
    const Eigen::Vector3d & base_to_lidar_translation, const std::function<bool()> & canceled, const SearchObserver & observer = {});
  SearchResult search(const Cloud & query_reference, const std::vector<Cloud> & independent_scans_reference,
    const Eigen::Vector3d & base_to_lidar_translation, const std::function<bool()> & canceled, const SearchObserver & observer = {});
  const std::string & mapHash() const;
  size_t databaseSize() const;
private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}
