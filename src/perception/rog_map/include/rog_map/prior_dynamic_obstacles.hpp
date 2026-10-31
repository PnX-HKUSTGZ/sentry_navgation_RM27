#pragma once

#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <memory>
#include <functional>
#include <vector>

namespace rog_map {
struct PriorDynamicConfig {
  double background_voxel{0.04};
  double background_distance{0.15};
  double surface_distance{0.08};
  double normal_angle_deg{45.0};
  int normal_neighbors{12};
  // Static surface association tolerates sparse sampling along a surface,
  // without using the same large tolerance normal to it (a parked car roof).
  double prior_boundary_distance{0.10};
  double static_surface_radius{0.50};
  double static_surface_distance{0.10};
  int static_surface_min_support{3};
  double static_surface_max_curvature{0.10};
  double ground_surface_tolerance{0.05};
  double live_voxel{0.05};
  double cluster_distance{0.22};
  int min_points{5};
  double min_span{0.08};
  double max_range{8.0};
  double min_relative_z{-0.50};
  double max_relative_z{1.50};
  double association_distance{0.80};
  double hold_time{6.0};
  double max_speed{3.0};  // Association gate only; never extrapolates occupancy.
  double padding{0.10};
  double min_extent{0.60};
  int clear_ray_count{4};
  int clear_frames{2};
  void validate() const;
};

// A target is a spatial cluster absent from the fixed map, not a velocity
// threshold. A parked vehicle is therefore never learned into the background.
class PriorDynamicObstacles {
public:
  using Cloud = pcl::PointCloud<pcl::PointXYZ>;
  struct Track {
    uint64_t id{0};
    Eigen::Vector3f minimum{Eigen::Vector3f::Zero()};
    Eigen::Vector3f maximum{Eigen::Vector3f::Zero()};
    // Occupancy completion never changes the measured box used for association.
    Eigen::Vector2f completion_shift{Eigen::Vector2f::Zero()};
    double seen{0.0};
    int observations{0};
    int clear_frames{0};
  };
  struct Stats {
    size_t input{0}, sampled{0}, background{0}, novel{0}, clusters{0}, tracks{0}, ray_cleared{0};
    size_t surface_filtered{0}, boundary_filtered{0}, ground_filtered{0};
  };
  explicit PriorDynamicObstacles(PriorDynamicConfig config = {});
  void setBackground(const Cloud &map);
  bool backgroundReady() const;
  void update(const pcl::PointCloud<pcl::PointXYZI> &cloud,
              const Eigen::Isometry3d &map_from_cloud,
              const Eigen::Vector3d &sensor_map,
              const Eigen::Vector3d &robot_ground_map, double stamp,
              const std::function<bool(double, double)> &candidate_region = {},
              const std::function<bool(double, double, double &)> &ground_reference = {});
  bool occupied(const Eigen::Vector2d &point_map, double half_cell,
                double stamp, float *low_z = nullptr, float *high_z = nullptr) const;
  struct Box { Eigen::Vector2f minimum, maximum; };
  std::vector<Box> boxes(double stamp, double half_cell = 0.0) const;
  const std::vector<Track> &tracks() const { return tracks_; }
  Stats stats() const { return stats_; }
  const PriorDynamicConfig &config() const { return config_; }
private:
  struct Background;
  PriorDynamicConfig config_;
  std::shared_ptr<const Background> background_;
  std::shared_ptr<const Background> last_background_;
  std::vector<Track> tracks_;
  Stats stats_;
  double last_stamp_{0.0};
  uint64_t next_id_{1};
};
} // namespace rog_map
