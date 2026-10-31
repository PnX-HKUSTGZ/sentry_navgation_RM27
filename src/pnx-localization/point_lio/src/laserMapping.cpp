// #include <so3_math.h>
#include <algorithm>
#include <cmath>
#include <chrono>
#include <cstdint>
#include <malloc.h>
#include <memory>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <limits>
#include <sstream>

#include "li_initialization.h"
#include "instantaneous_scan.hpp"

using namespace std;

#define PUBFRAME_PERIOD (20)

const float MOV_THRESHOLD = 1.5f;

string root_dir = ROOT_DIR;

int time_log_counter = 0;

bool init_map = false, flg_first_scan = true;

// Time Log Variables
double match_time = 0, solve_time = 0, propag_time = 0, update_time = 0;

bool flg_reset = false, flg_exit = false;

//surf feature in map
PointCloudXYZI::Ptr feats_undistort(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_body_space(new PointCloudXYZI());
PointCloudXYZI::Ptr init_feats_world(new PointCloudXYZI());
std::deque<PointCloudXYZI::Ptr> depth_feats_world;
pcl::VoxelGrid<PointType> downSizeFilterSurf;
pcl::VoxelGrid<PointType> downSizeFilterMap;

V3D euler_cur;

nav_msgs::msg::Path path;
nav_msgs::msg::Odometry odomAftMapped;
double last_published_odom_stamp = 0.0;
std::chrono::steady_clock::time_point last_odom_publish_wall;
geometry_msgs::msg::PoseStamped msg_body_pose;

int sleep_time = 0;

auto LOGGER = rclcpp::get_logger("laserMapping");

struct FullCloudPoint
{
  PointType point;
  double stamp = 0.0;
};

struct StateSnapshot
{
  double stamp = 0.0;
  V3D pos = V3D::Zero();
  M3D rot = M3D::Identity();
  V3D vel = V3D::Zero();
  V3D acc = V3D::Zero();
  V3D omg = V3D::Zero();
  M3D lidar_to_imu_rotation = M3D::Identity();
  V3D lidar_to_imu_translation = V3D::Zero();
};

constexpr double kFullCloudMaxQueueAge = 0.20;
constexpr double kFullCloudStampEpsilon = 1.0e-6;

std::deque<FullCloudPoint> full_cloud_queue;
bool has_full_cloud_anchor = false;
StateSnapshot full_cloud_anchor;
PointCloudXYZI::Ptr full_cloud_world(new PointCloudXYZI());

namespace
{
struct PendingTemporalPoint
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  std::size_t source_index = 0;
  V3D point_lidar = V3D::Zero();
  V3D point_imu = V3D::Zero();
  V3D predicted_world = V3D::Zero();
};

using PendingTemporalPointVector =
  std::vector<PendingTemporalPoint, Eigen::aligned_allocator<PendingTemporalPoint>>;

struct PendingMapUpdate
{
  double stamp = 0.0;
  PointVector points;
};

PendingTemporalPointVector pending_temporal_points;
std::deque<PendingMapUpdate> pending_map_updates;
point_lio::TemporalUpdateScheduler temporal_scheduler;
double map_pause_until = -std::numeric_limits<double>::infinity();
std::size_t committed_map_frames = 0;
std::size_t rejected_map_frames = 0;

template <typename Covariance>
Eigen::Matrix<double, 6, 6> poseCovariance(const Covariance & covariance)
{
  Eigen::Matrix<double, 6, 6> result;
  result.topLeftCorner<3, 3>() = covariance.template block<3, 3>(0, 0);
  result.topRightCorner<3, 3>() = covariance.template block<3, 3>(0, 3);
  result.bottomLeftCorner<3, 3>() = covariance.template block<3, 3>(3, 0);
  result.bottomRightCorner<3, 3>() = covariance.template block<3, 3>(3, 3);
  return 0.5 * (result + result.transpose());
}

template <typename State>
void appendTemporalGroup(const State & state, int first_index, int count)
{
  for (int offset = 0; offset < count; ++offset) {
    const std::size_t source_index = static_cast<std::size_t>(first_index + offset);
    const auto & point = feats_down_body->points[source_index];
    PendingTemporalPoint pending;
    pending.source_index = source_index;
    pending.point_lidar = V3D(point.x, point.y, point.z);
    pending.point_imu = extrinsic_est_en ?
      state.offset_R_L_I * pending.point_lidar + state.offset_T_L_I : pbody_list[source_index];
    pending.predicted_world = state.rot * pending.point_imu + state.pos;
    pending_temporal_points.emplace_back(pending);

    auto & world = feats_down_world->points[source_index];
    world.x = pending.predicted_world.x();
    world.y = pending.predicted_world.y();
    world.z = pending.predicted_world.z();
    world.intensity = point.intensity;
  }
}

template <typename State>
void prepareActiveMeasurements(const State & state)
{
  active_lidar_points.clear();
  active_lidar_points.reserve(pending_temporal_points.size());
  for (const auto & pending : pending_temporal_points) {
    ActiveLidarPoint active;
    active.source_index = pending.source_index;
    active.point_lidar = pending.point_lidar;
    if (temporal_adaptive_config.enabled) {
      active.point_imu = state.rot.transpose() * (pending.predicted_world - state.pos);
      active.reanchored = true;
    } else {
      active.point_imu = pending.point_imu;
      active.reanchored = false;
    }
    active_lidar_points.emplace_back(active);
  }
}

template <typename State>
void transformActiveMeasurements(const State & state)
{
  for (const auto & active : active_lidar_points) {
    if (active.source_index >= feats_down_world->size()) {
      continue;
    }
    const V3D point_imu = (extrinsic_est_en && !active.reanchored) ?
      state.offset_R_L_I * active.point_lidar + state.offset_T_L_I : active.point_imu;
    const V3D world = state.rot * point_imu + state.pos;
    auto & output = feats_down_world->points[active.source_index];
    output.x = world.x();
    output.y = world.y();
    output.z = world.z();
    output.intensity = feats_down_body->points[active.source_index].intensity;
  }
}

template <typename T>
diagnostic_msgs::msg::KeyValue diagnosticValue(const std::string & key, const T & value)
{
  diagnostic_msgs::msg::KeyValue output;
  output.key = key;
  std::ostringstream stream;
  stream << value;
  output.value = stream.str();
  return output;
}
}  // namespace

void SigHandle(int sig)
{
  flg_exit = true;
  RCLCPP_WARN(LOGGER, "catch sig %d", sig);
  sig_buffer.notify_all();
}

PointCloudXYZI::Ptr loadPointcloudFromPcd(const std::string & file_path)
{
  auto pcd_ptr = std::make_shared<PointCloudXYZI>();

  if (pcl::io::loadPCDFile(file_path, *pcd_ptr) == -1) {
    RCLCPP_ERROR(LOGGER, "Couldn't read pcd file %s", file_path.c_str());
    return nullptr;
  }

  RCLCPP_INFO(LOGGER, "Loaded %zu points from %s", pcd_ptr->size(), file_path.c_str());
  return pcd_ptr;
}

inline void dump_lio_state_to_log(FILE * fp)
{
  V3D rot_ang;
  if (!use_imu_as_input) {
    rot_ang = SO3ToEuler(kf_output.x_.rot);
  } else {
    rot_ang = SO3ToEuler(kf_input.x_.rot);
  }

  fprintf(fp, "%lf ", Measures.lidar_beg_time - first_lidar_time);
  fprintf(fp, "%lf %lf %lf ", rot_ang(0), rot_ang(1), rot_ang(2));  // Angle
  if (use_imu_as_input) {
    fprintf(fp, "%lf %lf %lf ", kf_input.x_.pos(0), kf_input.x_.pos(1), kf_input.x_.pos(2));  // Pos
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);  // omega
    fprintf(fp, "%lf %lf %lf ", kf_input.x_.vel(0), kf_input.x_.vel(1), kf_input.x_.vel(2));  // Vel
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                               // Acc
    fprintf(fp, "%lf %lf %lf ", kf_input.x_.bg(0), kf_input.x_.bg(1), kf_input.x_.bg(2));  // Bias_g
    fprintf(fp, "%lf %lf %lf ", kf_input.x_.ba(0), kf_input.x_.ba(1), kf_input.x_.ba(2));  // Bias_a
    fprintf(
      fp, "%lf %lf %lf ", kf_input.x_.gravity(0), kf_input.x_.gravity(1),
      kf_input.x_.gravity(2));  // Bias_a
  } else {
    fprintf(
      fp, "%lf %lf %lf ", kf_output.x_.pos(0), kf_output.x_.pos(1), kf_output.x_.pos(2));  // Pos
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                            // omega
    fprintf(
      fp, "%lf %lf %lf ", kf_output.x_.vel(0), kf_output.x_.vel(1), kf_output.x_.vel(2));  // Vel
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                            // Acc
    fprintf(
      fp, "%lf %lf %lf ", kf_output.x_.bg(0), kf_output.x_.bg(1), kf_output.x_.bg(2));  // Bias_g
    fprintf(
      fp, "%lf %lf %lf ", kf_output.x_.ba(0), kf_output.x_.ba(1), kf_output.x_.ba(2));  // Bias_a
    fprintf(
      fp, "%lf %lf %lf ", kf_output.x_.gravity(0), kf_output.x_.gravity(1),
      kf_output.x_.gravity(2));  // Bias_a
  }
  fprintf(fp, "\r\n");
  fflush(fp);
}

void pointBodyLidarToIMU(PointType const * const pi, PointType * const po)
{
  V3D p_body_lidar(pi->x, pi->y, pi->z);
  V3D p_body_imu;
  if (extrinsic_est_en) {
    if (!use_imu_as_input) {
      p_body_imu = kf_output.x_.offset_R_L_I * p_body_lidar + kf_output.x_.offset_T_L_I;
    } else {
      p_body_imu = kf_input.x_.offset_R_L_I * p_body_lidar + kf_input.x_.offset_T_L_I;
    }
  } else {
    p_body_imu = Lidar_R_wrt_IMU * p_body_lidar + Lidar_T_wrt_IMU;
  }
  po->x = p_body_imu(0);
  po->y = p_body_imu(1);
  po->z = p_body_imu(2);
  po->intensity = pi->intensity;
}

bool isFinitePoint(const PointType & point)
{
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

void enqueueFullCloud(const PointCloudXYZI::Ptr & cloud, double lidar_beg_time)
{
  if (!cloud) {
    return;
  }

  std::vector<FullCloudPoint> batch;
  batch.reserve(cloud->size());
  for (const auto & point : cloud->points) {
    if (!isFinitePoint(point)) {
      continue;
    }

    FullCloudPoint full_point;
    full_point.point = point;
    full_point.stamp = lidar_beg_time + static_cast<double>(point.curvature) / 1000.0;
    if (std::isfinite(full_point.stamp)) {
      batch.emplace_back(std::move(full_point));
    }
  }

  std::stable_sort(
    batch.begin(), batch.end(),
    [](const FullCloudPoint & lhs, const FullCloudPoint & rhs) { return lhs.stamp < rhs.stamp; });
  for (auto & point : batch) {
    full_cloud_queue.emplace_back(std::move(point));
  }

  // A previous scan can leave future points in the queue, so keep the complete queue ordered.
  std::stable_sort(
    full_cloud_queue.begin(), full_cloud_queue.end(),
    [](const FullCloudPoint & lhs, const FullCloudPoint & rhs) { return lhs.stamp < rhs.stamp; });
}

StateSnapshot makeStateSnapshot(double stamp)
{
  StateSnapshot snapshot;
  snapshot.stamp = stamp;

  if (!use_imu_as_input) {
    snapshot.pos = kf_output.x_.pos;
    snapshot.rot = kf_output.x_.rot;
    snapshot.vel = kf_output.x_.vel;
    snapshot.acc = kf_output.x_.rot * kf_output.x_.acc + kf_output.x_.gravity;
    snapshot.omg = kf_output.x_.omg;
    if (extrinsic_est_en) {
      snapshot.lidar_to_imu_rotation = kf_output.x_.offset_R_L_I;
      snapshot.lidar_to_imu_translation = kf_output.x_.offset_T_L_I;
    }
  } else {
    snapshot.pos = kf_input.x_.pos;
    snapshot.rot = kf_input.x_.rot;
    snapshot.vel = kf_input.x_.vel;
    snapshot.acc =
      kf_input.x_.rot * (input_in.acc - kf_input.x_.ba) + kf_input.x_.gravity;
    snapshot.omg = input_in.gyro - kf_input.x_.bg;
    if (extrinsic_est_en) {
      snapshot.lidar_to_imu_rotation = kf_input.x_.offset_R_L_I;
      snapshot.lidar_to_imu_translation = kf_input.x_.offset_T_L_I;
    }
  }

  if (!extrinsic_est_en) {
    snapshot.lidar_to_imu_rotation = Lidar_R_wrt_IMU;
    snapshot.lidar_to_imu_translation = Lidar_T_wrt_IMU;
  }
  return snapshot;
}

PointType transformFullPointForward(
  const StateSnapshot & anchor, const FullCloudPoint & full_point)
{
  const double dt = full_point.stamp - anchor.stamp;
  const M3D predicted_rotation = anchor.rot * Exp(anchor.omg, dt);
  const V3D predicted_position = anchor.pos + anchor.vel * dt + 0.5 * anchor.acc * dt * dt;
  const V3D point_lidar(full_point.point.x, full_point.point.y, full_point.point.z);
  const V3D point_imu =
    anchor.lidar_to_imu_rotation * point_lidar + anchor.lidar_to_imu_translation;
  const V3D point_world = predicted_rotation * point_imu + predicted_position;

  PointType output = full_point.point;
  output.x = point_world.x();
  output.y = point_world.y();
  output.z = point_world.z();
  return output;
}

void processFullCloudSegmentWithSnapshot(const StateSnapshot & current_snapshot)
{
  if (!std::isfinite(current_snapshot.stamp)) {
    return;
  }
  if (!has_full_cloud_anchor) {
    full_cloud_anchor = current_snapshot;
    has_full_cloud_anchor = true;
    return;
  }
  if (current_snapshot.stamp <= full_cloud_anchor.stamp + kFullCloudStampEpsilon) {
    full_cloud_anchor = current_snapshot;
    return;
  }

  while (
    !full_cloud_queue.empty() &&
    current_snapshot.stamp - full_cloud_queue.front().stamp > kFullCloudMaxQueueAge)
  {
    full_cloud_queue.pop_front();
  }

  while (!full_cloud_queue.empty()) {
    const auto & full_point = full_cloud_queue.front();
    if (full_point.stamp + kFullCloudStampEpsilon < full_cloud_anchor.stamp) {
      full_cloud_queue.pop_front();
      continue;
    }
    if (full_point.stamp > current_snapshot.stamp + kFullCloudStampEpsilon) {
      break;
    }

    PointType point_world = transformFullPointForward(full_cloud_anchor, full_point);
    if (isFinitePoint(point_world)) {
      full_cloud_world->push_back(point_world);
    }
    full_cloud_queue.pop_front();
  }
  full_cloud_anchor = current_snapshot;
}

void publishAccumulatedFullCloudWorld(
  const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & publisher, double stamp)
{
  if (!publisher || !full_cloud_world || full_cloud_world->empty()) {
    return;
  }

  full_cloud_world->width = static_cast<std::uint32_t>(full_cloud_world->size());
  full_cloud_world->height = 1;
  full_cloud_world->is_dense = false;

  auto message = std::make_unique<sensor_msgs::msg::PointCloud2>();
  pcl::toROSMsg(*full_cloud_world, *message);
  message->header.stamp = get_ros_time(stamp);
  message->header.frame_id = "camera_init";
  publisher->publish(std::move(message));
  full_cloud_world->clear();
}

PointVector collectMapPoints()
{
  PointVector points_to_add;
  int cur_pts = feats_down_world->size();
  points_to_add.reserve(cur_pts);

  for (size_t i = 0; i < cur_pts; ++i) {
    /* decide if need add to map */
    PointType & point_world = feats_down_world->points[i];
    if (!Nearest_Points[i].empty()) {
      const PointVector & points_near = Nearest_Points[i];

      Eigen::Vector3f center =
        ((point_world.getVector3fMap() / filter_size_map_min).array().floor() + 0.5) *
        filter_size_map_min;
      bool need_add = true;
      for (int readd_i = 0; readd_i < points_near.size(); readd_i++) {
        Eigen::Vector3f dis_2_center = points_near[readd_i].getVector3fMap() - center;
        if (
          fabs(dis_2_center.x()) < 0.5 * filter_size_map_min &&
          fabs(dis_2_center.y()) < 0.5 * filter_size_map_min &&
          fabs(dis_2_center.z()) < 0.5 * filter_size_map_min) {
          need_add = false;
          break;
        }
      }
      if (need_add) {
        points_to_add.emplace_back(point_world);
      }
    } else {
      points_to_add.emplace_back(point_world);
    }
  }
  return points_to_add;
}

void processPendingMapUpdates(double current_stamp)
{
  if (frame_diagnostics.imu_saturated || imu_gap_detected) {
    map_pause_until = std::max(map_pause_until, current_stamp + map_impact_pause_duration);
    rejected_map_frames += pending_map_updates.size();
    pending_map_updates.clear();
    imu_gap_detected = false;
  }
  if (current_stamp < map_pause_until) {
    return;
  }

  while (!pending_map_updates.empty() &&
         current_stamp - pending_map_updates.front().stamp >= map_insertion_delay) {
    ivox_->AddPoints(pending_map_updates.front().points);
    pending_map_updates.pop_front();
    ++committed_map_frames;
  }
}

void queueMapIncremental(double current_stamp, bool quality_ok)
{
  if (!quality_ok || current_stamp < map_pause_until) {
    ++rejected_map_frames;
    processPendingMapUpdates(current_stamp);
    return;
  }

  PendingMapUpdate update;
  update.stamp = current_stamp;
  update.points = collectMapPoints();
  if (!update.points.empty()) {
    pending_map_updates.emplace_back(std::move(update));
  }
  processPendingMapUpdates(current_stamp);
}

void publish_init_map(
  const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & pubLaserCloudFullRes)
{
  int size_init_map = init_feats_world->size();

  sensor_msgs::msg::PointCloud2 laserCloudmsg;

  pcl::toROSMsg(*init_feats_world, laserCloudmsg);

  laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
  laserCloudmsg.header.frame_id = "camera_init";
  pubLaserCloudFullRes->publish(laserCloudmsg);
}

PointCloudXYZI::Ptr pcl_wait_pub(new PointCloudXYZI(500000, 1));
PointCloudXYZI::Ptr pcl_wait_save(new PointCloudXYZI());
void publish_frame_world(
  const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & pubLaserCloudFullRes)
{
  if (scan_pub_en) {
    sensor_msgs::msg::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*feats_down_world, laserCloudmsg);

    laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
    laserCloudmsg.header.frame_id = "camera_init";
    pubLaserCloudFullRes->publish(laserCloudmsg);

    //--------------------------save map-----------------------------------
    // 1. make sure you have enough memories
    // 2. noted that pcd save will influence the real-time performances
    if (pcd_save_en) {
      *pcl_wait_save += *feats_down_world;

      static int scan_wait_num = 0;
      scan_wait_num++;
      if (!pcl_wait_save->empty() && pcd_save_interval > 0 && scan_wait_num >= pcd_save_interval) {
        pcd_index++;
        string all_points_dir(
          string(string(ROOT_DIR) + "PCD/scans_") + to_string(pcd_index) + string(".pcd"));
        pcl::PCDWriter pcd_writer;
        std::cout << "current scan saved to /PCD/" << all_points_dir << '\n';
        pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
        pcl_wait_save->clear();
        scan_wait_num = 0;
      }
    }
  }
}

void publish_frame_body(
  const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & pubLaserCloudFull_body)
{
  int size = feats_undistort->points.size();
  PointCloudXYZI::Ptr laserCloudIMUBody(new PointCloudXYZI(size, 1));

  for (int i = 0; i < size; i++) {
    pointBodyLidarToIMU(&feats_undistort->points[i], &laserCloudIMUBody->points[i]);
  }

  sensor_msgs::msg::PointCloud2 laserCloudmsg;
  pcl::toROSMsg(*laserCloudIMUBody, laserCloudmsg);
  laserCloudmsg.header.stamp = get_ros_time(lidar_end_time);
  laserCloudmsg.header.frame_id = "body";
  pubLaserCloudFull_body->publish(laserCloudmsg);
}

template <typename T>
void set_posestamp(T & out)
{
  // Static variable, initialized to true, only effective on the first call
  static bool is_first_kf = true;

  auto set_output_from_kf = [&](const auto & kf) {
    out.position.x = kf.x_.pos(0);
    out.position.y = kf.x_.pos(1);
    out.position.z = kf.x_.pos(2);
    Eigen::Quaterniond q(kf.x_.rot);
    out.orientation.x = q.coeffs()[0];
    out.orientation.y = q.coeffs()[1];
    out.orientation.z = q.coeffs()[2];
    out.orientation.w = q.coeffs()[3];
  };

  if (!use_imu_as_input) {
    if (enable_prior_pcd && is_first_kf && init_pose.size() >= 3) {
      // Execute only on the first call
      kf_output.x_.pos(0) = init_pose[0];
      kf_output.x_.pos(1) = init_pose[1];
      kf_output.x_.pos(2) = init_pose[2];
      set_output_from_kf(kf_output);
      is_first_kf = false;  // Set is_first_kf to false after the first call
    } else {
      set_output_from_kf(kf_output);
    }
  } else {
    set_output_from_kf(kf_input);
  }
}

void publish_odometry(
  const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr & pubOdomAftMapped,
  std::shared_ptr<tf2_ros::TransformBroadcaster> & tf_br)
{
  odomAftMapped.header.frame_id = "camera_init";
  odomAftMapped.child_frame_id = "body";
  if (publish_odometry_without_downsample) {
    odomAftMapped.header.stamp = get_ros_time(time_current);
  } else {
    odomAftMapped.header.stamp = get_ros_time(lidar_end_time);
  }
  set_posestamp(odomAftMapped.pose.pose);
  const Eigen::Matrix<double, 6, 6> pose_covariance = use_imu_as_input ?
    poseCovariance(kf_input.get_P()) : poseCovariance(kf_output.get_P());
  for (int row = 0; row < 6; ++row) {
    for (int column = 0; column < 6; ++column) {
      odomAftMapped.pose.covariance[row * 6 + column] = pose_covariance(row, column);
    }
  }

  // nav_msgs/Odometry defines twist in child_frame_id. Filter velocity is in camera_init,
  // while the angular-rate sources below are already expressed in the IMU/body frame.
  auto set_body_linear_velocity = [&](const auto & filter) {
    const V3D velocity_body = filter.x_.rot.transpose() * filter.x_.vel;
    odomAftMapped.twist.twist.linear.x = velocity_body.x();
    odomAftMapped.twist.twist.linear.y = velocity_body.y();
    odomAftMapped.twist.twist.linear.z = velocity_body.z();
  };
  if (!use_imu_as_input) {
    set_body_linear_velocity(kf_output);
    odomAftMapped.twist.twist.angular.x = kf_output.x_.omg.x();
    odomAftMapped.twist.twist.angular.y = kf_output.x_.omg.y();
    odomAftMapped.twist.twist.angular.z = kf_output.x_.omg.z();
  } else {
    set_body_linear_velocity(kf_input);
    const V3D angular_velocity_body = input_in.gyro - kf_input.x_.bg;
    odomAftMapped.twist.twist.angular.x = angular_velocity_body.x();
    odomAftMapped.twist.twist.angular.y = angular_velocity_body.y();
    odomAftMapped.twist.twist.angular.z = angular_velocity_body.z();
  }

  const auto publish_wall = std::chrono::steady_clock::now();
  const double published_stamp = get_time_sec(odomAftMapped.header.stamp);
  if (last_published_odom_stamp > 0.0) {
    const double gap = std::chrono::duration<double>(publish_wall - last_odom_publish_wall).count();
    if (gap > 0.35) {
      RCLCPP_WARN(LOGGER,
        "Odometry publication gap: wall=%.3f s scan_delta=%.3f s "
        "lidar_queue=%zu imu_queue=%zu dropped_lidar=%zu dropped_imu=%zu",
        gap, published_stamp - last_published_odom_stamp,
        lidar_buffer.size(), imu_deque.size(), dropped_lidar_frames, dropped_imu_messages);
    }
  }
  last_published_odom_stamp = published_stamp;
  last_odom_publish_wall = publish_wall;
  pubOdomAftMapped->publish(odomAftMapped);

  if (tf_send_en) {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.frame_id = "camera_init";
    transform.child_frame_id = "aft_mapped";
    transform.transform.translation.x = odomAftMapped.pose.pose.position.x;
    transform.transform.translation.y = odomAftMapped.pose.pose.position.y;
    transform.transform.translation.z = odomAftMapped.pose.pose.position.z;
    transform.transform.rotation.w = odomAftMapped.pose.pose.orientation.w;
    transform.transform.rotation.x = odomAftMapped.pose.pose.orientation.x;
    transform.transform.rotation.y = odomAftMapped.pose.pose.orientation.y;
    transform.transform.rotation.z = odomAftMapped.pose.pose.orientation.z;
    transform.header.stamp = odomAftMapped.header.stamp;
    tf_br->sendTransform(transform);
  }
}

void publish_path(const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath)
{
  set_posestamp(msg_body_pose.pose);
  // msg_body_pose.header.stamp = ros::Time::now();
  msg_body_pose.header.stamp = get_ros_time(lidar_end_time);
  msg_body_pose.header.frame_id = "camera_init";
  static int jjj = 0;
  jjj++;
  // if (jjj % 2 == 0) // if path is too large, the rvis will crash
  {
    path.poses.emplace_back(msg_body_pose);
    pubPath->publish(path);
  }
}

void publish_diagnostics(
  const rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr & publisher,
  double stamp, double processing_ms)
{
  diagnostic_msgs::msg::DiagnosticArray message;
  message.header.stamp = get_ros_time(stamp);
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "point_lio/estimator";
  status.hardware_id = "point_lio";
  const bool low_geometry =
    frame_diagnostics.effective_points < static_cast<double>(map_min_effective_points);
  status.level = (frame_diagnostics.imu_saturated || low_geometry) ?
    diagnostic_msgs::msg::DiagnosticStatus::WARN : diagnostic_msgs::msg::DiagnosticStatus::OK;
  status.message = frame_diagnostics.imu_saturated ? "imu_saturated" :
    (low_geometry ? "low_geometry" : "ok");

  const Eigen::Matrix<double, 6, 6> pose_covariance = use_imu_as_input ?
    poseCovariance(kf_input.get_P()) : poseCovariance(kf_output.get_P());
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> information_solver(
    frame_diagnostics.pose_information, Eigen::EigenvaluesOnly);
  double min_information_eigenvalue = 0.0;
  double information_condition = std::numeric_limits<double>::infinity();
  if (information_solver.info() == Eigen::Success) {
    min_information_eigenvalue = information_solver.eigenvalues().minCoeff();
    information_condition = information_solver.eigenvalues().maxCoeff() /
      std::max(min_information_eigenvalue, 1.0e-12);
  }

  const double row_count = std::max<std::size_t>(frame_diagnostics.effective_rows, 1);
  status.values = {
    diagnosticValue("raw_points", frame_diagnostics.raw_points),
    diagnosticValue("filtered_points", frame_diagnostics.filtered_points),
    diagnosticValue("associated_points", frame_diagnostics.associated_points),
    diagnosticValue("effective_rows", frame_diagnostics.effective_rows),
    diagnosticValue("effective_points", frame_diagnostics.effective_points),
    diagnosticValue("lidar_updates", frame_diagnostics.lidar_updates),
    diagnosticValue("skipped_updates", frame_diagnostics.skipped_updates),
    diagnosticValue("mean_abs_residual", frame_diagnostics.residual_abs_sum / row_count),
    diagnosticValue("mean_nis", frame_diagnostics.nis_sum / row_count),
    diagnosticValue("information_gain", frame_diagnostics.information_gain_sum),
    diagnosticValue("pose_cov_logdet", point_lio::stableLogDet(pose_covariance)),
    diagnosticValue("min_information_eigenvalue", min_information_eigenvalue),
    diagnosticValue("information_condition", information_condition),
    diagnosticValue("scale_0_responsibility", frame_diagnostics.scale_responsibility[0]),
    diagnosticValue("scale_1_responsibility", frame_diagnostics.scale_responsibility[1]),
    diagnosticValue("scale_2_responsibility", frame_diagnostics.scale_responsibility[2]),
    diagnosticValue("scheduler_target_points", temporal_scheduler.commandedPoints()),
    diagnosticValue("information_per_point", temporal_scheduler.informationPerPoint()),
    diagnosticValue("processing_ms", processing_ms),
    diagnosticValue("lidar_queue", lidar_buffer.size()),
    diagnosticValue("imu_queue", imu_deque.size()),
    diagnosticValue("dropped_lidar_frames", dropped_lidar_frames),
    diagnosticValue("dropped_imu_messages", dropped_imu_messages),
    diagnosticValue("pending_map_frames", pending_map_updates.size()),
    diagnosticValue("committed_map_frames", committed_map_frames),
    diagnosticValue("rejected_map_frames", rejected_map_frames),
    diagnosticValue("map_voxels", ivox_->NumValidGrids()),
    diagnosticValue("map_points", ivox_->NumPoints())};
  message.status.emplace_back(std::move(status));
  publisher->publish(message);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto nh = std::make_shared<rclcpp::Node>("laserMapping");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(nh);

  readParameters(nh);
  if (temporal_adaptive_config.enabled && extrinsic_est_en) {
    RCLCPP_WARN(
      LOGGER,
      "temporal adaptation requires fixed LiDAR-IMU extrinsics; disabling temporal adaptation");
    temporal_adaptive_config.enabled = false;
  }
  temporal_scheduler.setConfig(temporal_adaptive_config);
  std::cout << "lidar_type: " << lidar_type << '\n';
  ivox_ = std::make_shared<IVoxType>(ivox_options_);

  path.header.stamp = get_ros_time(lidar_end_time);
  path.header.frame_id = "camera_init";

  /*** variables definition for counting ***/
  int frame_num = 0;
  double aver_time_consu = 0, aver_time_icp = 0, aver_time_match = 0, aver_time_incre = 0,
         aver_time_solve = 0, aver_time_propag = 0;

  downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
  downSizeFilterMap.setLeafSize(filter_size_map_min, filter_size_map_min, filter_size_map_min);

  Lidar_T_wrt_IMU << VEC_FROM_ARRAY(extrinT);
  Lidar_R_wrt_IMU << MAT_FROM_ARRAY(extrinR);

  if (extrinsic_est_en) {
    if (!use_imu_as_input) {
      kf_output.x_.offset_R_L_I = Lidar_R_wrt_IMU;
      kf_output.x_.offset_T_L_I = Lidar_T_wrt_IMU;
    } else {
      kf_input.x_.offset_R_L_I = Lidar_R_wrt_IMU;
      kf_input.x_.offset_T_L_I = Lidar_T_wrt_IMU;
    }
  }

  p_imu->lidar_type = p_pre->lidar_type = lidar_type;
  p_imu->imu_en = imu_en;

  kf_input.init_dyn_share_modified_2h(get_f_input, df_dx_input, h_model_input);
  kf_output.init_dyn_share_modified_3h(
    get_f_output, df_dx_output, h_model_output, h_model_IMU_output);
  kf_input.set_maximum_iterations(ekf_max_iterations);
  kf_output.set_maximum_iterations(ekf_max_iterations);
  Eigen::Matrix<double, 24, 24> P_init;  // = MD(18, 18)::Identity() * 0.1;
  reset_cov(P_init);
  kf_input.change_P(P_init);
  Eigen::Matrix<double, 30, 30> P_init_output;  // = MD(24, 24)::Identity() * 0.01;
  reset_cov_output(P_init_output);
  kf_output.change_P(P_init_output);
  Eigen::Matrix<double, 24, 24> Q_input = process_noise_cov_input();
  Eigen::Matrix<double, 30, 30> Q_output = process_noise_cov_output();
  /*** debug record ***/
  FILE * fp;
  string pos_log_dir = root_dir + "/Log/pos_log.txt";
  fp = fopen(pos_log_dir.c_str(), "w");
  open_file();

  /*** ROS subscribe initialization ***/
  const bool reliable_lidar = nh->declare_parameter<bool>("common.reliable_lidar", false);
  const int imu_receive_depth = static_cast<int>(std::clamp<int64_t>(
    nh->declare_parameter<int64_t>("common.imu_receive_depth", 5), 1, 500));
  auto lidar_qos = rclcpp::SensorDataQoS();
  if (reliable_lidar) {lidar_qos.keep_last(2).reliable();}
  RCLCPP_INFO(LOGGER, "LiDAR transport: %s, depth=%zu; IMU receive depth=%d",
    reliable_lidar ? "reliable" : "best_effort", lidar_qos.depth(), imu_receive_depth);
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pcl_pc;
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr sub_pcl_livox;
  if (p_pre->lidar_type == AVIA) {
    sub_pcl_livox = nh->create_subscription<livox_ros_driver2::msg::CustomMsg>(
      lid_topic, lidar_qos,
      [](const livox_ros_driver2::msg::CustomMsg::SharedPtr msg) { livox_pcl_cbk(msg); });
  } else {
    sub_pcl_pc = nh->create_subscription<sensor_msgs::msg::PointCloud2>(
      lid_topic, lidar_qos,
      [](const sensor_msgs::msg::PointCloud2::SharedPtr msg) { standard_pcl_cbk(msg); });
  }
  auto sub_imu =
    nh->create_subscription<sensor_msgs::msg::Imu>(
      imu_topic, rclcpp::SensorDataQoS().keep_last(imu_receive_depth), imu_cbk);
  auto pub_laser_cloud_full_res =
    nh->create_publisher<sensor_msgs::msg::PointCloud2>("cloud_registered", 20);
  auto pub_laser_cloud_full_world = nh->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/cloud_registered_full", rclcpp::SensorDataQoS().keep_last(1));
  auto pub_laser_cloud_full_res_body =
    nh->create_publisher<sensor_msgs::msg::PointCloud2>("cloud_registered_body", 20);
  auto pub_laser_cloud_effect =
    nh->create_publisher<sensor_msgs::msg::PointCloud2>("cloud_effected", 20);
  auto pub_laser_cloud_map = nh->create_publisher<sensor_msgs::msg::PointCloud2>("Laser_map", 20);
  auto pub_odom_aft_mapped =
    nh->create_publisher<nav_msgs::msg::Odometry>("aft_mapped_to_init", 20);
  auto pub_path = nh->create_publisher<nav_msgs::msg::Path>("path", 20);
  auto pub_diagnostics =
    nh->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("point_lio/diagnostics", 10);
  auto tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(nh);

  //------------------------------------------------------------------------------------------------------
  signal(SIGINT, SigHandle);
  rclcpp::Rate rate(500);
  while (rclcpp::ok()) {
    if (flg_exit) break;
    executor.spin_some();
    if (init_map && last_published_odom_stamp > 0.0) {
      const double gap = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - last_odom_publish_wall).count();
      if (gap > 0.40) {
        const double now = nh->now().seconds();
        RCLCPP_WARN_THROTTLE(LOGGER, *nh->get_clock(), 1000,
          "Odometry waiting: wall_gap=%.3f s odom_age=%.3f s "
          "lidar_input_age=%.3f s imu_input_age=%.3f s lidar_queue=%zu imu_queue=%zu",
          gap, now - last_published_odom_stamp, now - last_timestamp_lidar,
          now - last_timestamp_imu, lidar_buffer.size(), imu_deque.size());
      }
    }
    if (sync_packages(Measures)) {
      frame_diagnostics.reset();
      frame_diagnostics.raw_points = Measures.lidar ? Measures.lidar->size() : 0;
      if (flg_reset) {
        RCLCPP_WARN(LOGGER, "reset when rosbag play back");
        p_imu->Reset();
        feats_undistort.reset(new PointCloudXYZI());
        if (use_imu_as_input) {
          // state_in = kf_input.get_x();
          state_in = state_input();
          kf_input.change_P(P_init);
        } else {
          // state_out = kf_output.get_x();
          state_out = state_output();
          kf_output.change_P(P_init_output);
        }
        flg_first_scan = true;
        is_first_frame = true;
        flg_reset = false;
        init_map = false;
        pending_temporal_points.clear();
        active_lidar_points.clear();
        pending_map_updates.clear();
        temporal_scheduler.reset();
        map_pause_until = -std::numeric_limits<double>::infinity();
        full_cloud_queue.clear();
        has_full_cloud_anchor = false;
        full_cloud_world->clear();

        {
          ivox_.reset(new IVoxType(ivox_options_));
        }
      }

      if (flg_first_scan) {
        first_lidar_time = Measures.lidar_beg_time;
        flg_first_scan = false;
        if (first_imu_time < 1) {
          first_imu_time = get_time_sec(imu_next.header.stamp);
          printf("first imu time: %f\n", first_imu_time);
        }
        time_current = 0.0;
        if (imu_en) {
          // imu_next = *(imu_deque.front());
          kf_input.x_.gravity << VEC_FROM_ARRAY(gravity);
          kf_output.x_.gravity << VEC_FROM_ARRAY(gravity);
          // kf_output.x_.acc << VEC_FROM_ARRAY(gravity);
          // kf_output.x_.acc *= -1;

          {
            while (Measures.lidar_beg_time >
                   get_time_sec(imu_next.header.stamp))  // if it is needed for the new map?
            {
              imu_deque.pop_front();
              if (imu_deque.empty()) {
                break;
              }
              imu_last = imu_next;
              imu_next = *(imu_deque.front());
              // imu_deque.pop();
            }
          }
        } else {
          kf_input.x_.gravity << VEC_FROM_ARRAY(gravity);   // _init);
          kf_output.x_.gravity << VEC_FROM_ARRAY(gravity);  //_init);
          kf_output.x_.acc << VEC_FROM_ARRAY(gravity);      //_init);
          kf_output.x_.acc *= -1;
          p_imu->imu_need_init_ = false;
          // p_imu->after_imu_init_ = true;
        }
        G_m_s2 =
          std::sqrt(gravity[0] * gravity[0] + gravity[1] * gravity[1] + gravity[2] * gravity[2]);
      }

      double t0, t1, t2, t3, t4, t5, match_start, solve_start;
      match_time = 0;
      solve_time = 0;
      propag_time = 0;
      update_time = 0;
      t0 = omp_get_wtime();

      /*** downsample the feature points in a scan ***/
      t1 = omp_get_wtime();
      p_imu->Process(Measures, feats_undistort);
      const bool instantaneous_full_scan = point_lio::isInstantaneousScan(feats_undistort->points);
      if (!instantaneous_full_scan) {
        enqueueFullCloud(feats_undistort, Measures.lidar_beg_time);
      }
      if (!has_full_cloud_anchor) {
        full_cloud_anchor = makeStateSnapshot(Measures.lidar_beg_time);
        has_full_cloud_anchor = true;
      }
      if (space_down_sample) {
        downSizeFilterSurf.setInputCloud(feats_undistort);
        downSizeFilterSurf.filter(*feats_down_body);
        sort(feats_down_body->points.begin(), feats_down_body->points.end(), time_list);
      } else {
        feats_down_body = Measures.lidar;
        sort(feats_down_body->points.begin(), feats_down_body->points.end(), time_list);
      }
      {
        time_seq = time_compressing<int>(feats_down_body);
        feats_down_size = feats_down_body->points.size();
        frame_diagnostics.filtered_points = feats_down_size;
      }

      if (!p_imu->after_imu_init_)  // !p_imu->UseLIInit &&
      {
        if (!p_imu->imu_need_init_) {
          V3D tmp_gravity;
          if (imu_en) {
            tmp_gravity = -p_imu->mean_acc / p_imu->mean_acc.norm() * G_m_s2;
          } else {
            tmp_gravity << VEC_FROM_ARRAY(gravity_init);
            p_imu->after_imu_init_ = true;
          }
          // V3D tmp_gravity << VEC_FROM_ARRAY(gravity_init);
          M3D rot_init;
          p_imu->Set_init(tmp_gravity, rot_init);
          kf_input.x_.rot = rot_init;
          kf_output.x_.rot = rot_init;
          // kf_input.x_.rot; //.normalize();
          // kf_output.x_.rot; //.normalize();
          kf_output.x_.acc = -rot_init.transpose() * kf_output.x_.gravity;
        } else {
          continue;
        }
      }
      if (feats_down_body->empty()) {
        RCLCPP_WARN(LOGGER, "No point after downsampling, skip this scan!");
        continue;
      }
      /*** initialize the map ***/
      if (!init_map) {
        feats_down_world->resize(feats_undistort->size());
        for (int i = 0; i < feats_undistort->size(); i++) {
          {
            pointBodyToWorld(&(feats_undistort->points[i]), &(feats_down_world->points[i]));
          }
        }
        for (const auto & point : *feats_down_world) {
          init_feats_world->points.emplace_back(point);
        }

        if (init_feats_world->size() >= init_map_size) {
          if (enable_prior_pcd) {
            auto map_cloud = loadPointcloudFromPcd(prior_pcd_map_path);
            if (!map_cloud || map_cloud->empty()) {
              RCLCPP_ERROR(LOGGER, "prior PCD is unavailable or empty; skip map initialization");
              continue;
            }
            ivox_->AddPoints(map_cloud->points);
          } else {
            ivox_->AddPoints(init_feats_world->points);
          }
          publish_init_map(pub_laser_cloud_map);
          init_feats_world.reset(new PointCloudXYZI());
          init_map = true;
        } else {
          init_map = false;
        }
        continue;
      }

      /*** ICP and Kalman filter update ***/
      normvec->resize(feats_down_size);
      feats_down_world->resize(feats_down_size);

      Nearest_Points.resize(feats_down_size);

      t2 = omp_get_wtime();

      /*** iterated state estimation ***/
      crossmat_list.resize(feats_down_size);
      pbody_list.resize(feats_down_size);
      point_selected_surf.assign(feats_down_size, 0);
      pending_temporal_points.clear();
      active_lidar_points.clear();
      // pbody_ext_list.reserve(feats_down_size);

      for (size_t i = 0; i < feats_down_body->size(); i++) {
        V3D point_this(
          feats_down_body->points[i].x, feats_down_body->points[i].y, feats_down_body->points[i].z);
        pbody_list[i] = point_this;
        if (!extrinsic_est_en)
        // {
        //     if (!use_imu_as_input)
        //     {
        //         point_this = kf_output.x_.offset_R_L_I * point_this + kf_output.x_.offset_T_L_I;
        //     }
        //     else
        //     {
        //         point_this = kf_input.x_.offset_R_L_I * point_this + kf_input.x_.offset_T_L_I;
        //     }
        // }
        // else
        {
          point_this = Lidar_R_wrt_IMU * point_this + Lidar_T_wrt_IMU;
          M3D point_crossmat;
          point_crossmat << SKEW_SYM_MATRX(point_this);
          crossmat_list[i] = point_crossmat;
        }
      }
      if (!use_imu_as_input) {
        bool imu_upda_cov = false;
        effct_feat_num = 0;
        /**** point by point update ****/
        if (!time_seq.empty()) {
          double pcl_beg_time = Measures.lidar_beg_time;
          idx = -1;
          for (k = 0; k < time_seq.size(); k++) {
            PointType & point_body = feats_down_body->points[idx + time_seq[k]];

            time_current = point_body.curvature / 1000.0 + pcl_beg_time;

            if (is_first_frame) {
              if (imu_en) {
                while (time_current > get_time_sec(imu_next.header.stamp)) {
                  imu_deque.pop_front();
                  if (imu_deque.empty()) break;
                  imu_last = imu_next;
                  imu_next = *(imu_deque.front());
                }
                angvel_avr << imu_last.angular_velocity.x, imu_last.angular_velocity.y,
                  imu_last.angular_velocity.z;
                acc_avr << imu_last.linear_acceleration.x, imu_last.linear_acceleration.y,
                  imu_last.linear_acceleration.z;
              }
              is_first_frame = false;
              imu_upda_cov = true;
              time_update_last = time_current;
              time_predict_last_const = time_current;
            }
            if (imu_en && !imu_deque.empty()) {
              bool last_imu = get_time_sec(imu_next.header.stamp) ==
                              get_time_sec(imu_deque.front()->header.stamp);
              while (get_time_sec(imu_next.header.stamp) < time_predict_last_const &&
                     !imu_deque.empty()) {
                if (!last_imu) {
                  imu_last = imu_next;
                  imu_next = *(imu_deque.front());
                  break;
                } else {
                  imu_deque.pop_front();
                  if (imu_deque.empty()) break;
                  imu_last = imu_next;
                  imu_next = *(imu_deque.front());
                }
              }
              bool imu_comes = time_current > get_time_sec(imu_next.header.stamp);
              while (imu_comes) {
                imu_upda_cov = true;
                angvel_avr << imu_next.angular_velocity.x, imu_next.angular_velocity.y,
                  imu_next.angular_velocity.z;
                acc_avr << imu_next.linear_acceleration.x, imu_next.linear_acceleration.y,
                  imu_next.linear_acceleration.z;

                /*** covariance update ***/
                double dt = get_time_sec(imu_next.header.stamp) - time_predict_last_const;
                kf_output.predict(dt, Q_output, input_in, true, false);
                time_predict_last_const = get_time_sec(imu_next.header.stamp);  // big problem

                {
                  double dt_cov = get_time_sec(imu_next.header.stamp) - time_update_last;

                  if (dt_cov > 0.0) {
                    time_update_last = get_time_sec(imu_next.header.stamp);
                    double propag_imu_start = omp_get_wtime();

                    kf_output.predict(dt_cov, Q_output, input_in, false, true);

                    propag_time += omp_get_wtime() - propag_imu_start;
                    double solve_imu_start = omp_get_wtime();
                    kf_output.update_iterated_dyn_share_IMU();
                    solve_time += omp_get_wtime() - solve_imu_start;
                  }
                }
                imu_deque.pop_front();
                if (imu_deque.empty()) break;
                imu_last = imu_next;
                imu_next = *(imu_deque.front());
                imu_comes = time_current > get_time_sec(imu_next.header.stamp);
              }
            }
            if (flg_reset) {
              break;
            }

            double dt = time_current - time_predict_last_const;
            double propag_state_start = omp_get_wtime();
            if (!prop_at_freq_of_imu) {
              double dt_cov = time_current - time_update_last;
              if (dt_cov > 0.0) {
                kf_output.predict(dt_cov, Q_output, input_in, false, true);
                time_update_last = time_current;
              }
            }
            kf_output.predict(dt, Q_output, input_in, true, false);
            propag_time += omp_get_wtime() - propag_state_start;
            time_predict_last_const = time_current;
            double t_update_start = omp_get_wtime();

            if (feats_down_size < 1) {
              RCLCPP_WARN(LOGGER, "No point, skip this scan!\n");
              idx += time_seq[k];
              continue;
            }
            appendTemporalGroup(kf_output.x_, idx + 1, time_seq[k]);
            const auto covariance_before = poseCovariance(kf_output.get_P());
            const bool force_update = (k + 1 == static_cast<int>(time_seq.size()));
            const bool should_update = temporal_scheduler.shouldUpdate(
              time_current, time_seq[k], kf_output.x_.pos, kf_output.x_.rot,
              covariance_before.trace(), force_update);
            if (!should_update) {
              ++frame_diagnostics.skipped_updates;
              idx += time_seq[k];
              continue;
            }

            prepareActiveMeasurements(kf_output.x_);
            const double logdet_before = point_lio::stableLogDet(covariance_before);
            const double effective_before = frame_diagnostics.effective_points;
            solve_start = omp_get_wtime();
            if (!kf_output.update_iterated_dyn_share_modified()) {
              if (force_update) {
                temporal_scheduler.recordUpdate(
                  time_current, kf_output.x_.pos, kf_output.x_.rot, 0.0, 0.0);
                pending_temporal_points.clear();
                active_lidar_points.clear();
              }
              idx += time_seq[k];
              continue;
            }
            transformActiveMeasurements(kf_output.x_);
            const double logdet_after =
              point_lio::stableLogDet(poseCovariance(kf_output.get_P()));
            const double information_gain =
              (std::isfinite(logdet_before) && std::isfinite(logdet_after)) ?
              std::max(0.0, 0.5 * (logdet_before - logdet_after)) : 0.0;
            const double effective_update_points =
              std::max(0.0, frame_diagnostics.effective_points - effective_before);
            temporal_scheduler.recordUpdate(
              time_current, kf_output.x_.pos, kf_output.x_.rot, information_gain,
              effective_update_points);
            frame_diagnostics.information_gain_sum += information_gain;
            ++frame_diagnostics.lidar_updates;
            pending_temporal_points.clear();
            active_lidar_points.clear();
            processFullCloudSegmentWithSnapshot(makeStateSnapshot(time_current));

            if (publish_odometry_without_downsample) {
              /******* Publish odometry *******/

              publish_odometry(pub_odom_aft_mapped, tf_broadcaster);
              if (runtime_pos_log) {
                euler_cur = SO3ToEuler(kf_output.x_.rot);
                fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " "
                         << euler_cur.transpose() << " " << kf_output.x_.pos.transpose() << " "
                         << kf_output.x_.vel.transpose() << " " << kf_output.x_.omg.transpose()
                         << " " << kf_output.x_.acc.transpose() << " "
                         << kf_output.x_.gravity.transpose() << " " << kf_output.x_.bg.transpose()
                         << " " << kf_output.x_.ba.transpose() << " "
                         << feats_undistort->points.size() << '\n';
              }
            }

            solve_time += omp_get_wtime() - solve_start;

            update_time += omp_get_wtime() - t_update_start;
            idx += time_seq[k];
            // std::cout << "pbp output effect feat num:" << effct_feat_num << '\n';
          }
        } else {
          if (!imu_deque.empty()) {
            imu_last = imu_next;
            imu_next = *(imu_deque.front());

            while (get_time_sec(imu_next.header.stamp) > time_current &&
                   ((get_time_sec(imu_next.header.stamp) <
                     Measures.lidar_beg_time + lidar_time_inte))) {  // >= ?
              if (is_first_frame) {
                {
                  {
                    while (get_time_sec(imu_next.header.stamp) <
                           Measures.lidar_beg_time + lidar_time_inte) {
                      // meas.imu.emplace_back(imu_deque.front()); should add to initialization
                      imu_deque.pop_front();
                      if (imu_deque.empty()) break;
                      imu_last = imu_next;
                      imu_next = *(imu_deque.front());
                    }
                  }
                  break;
                }
                angvel_avr << imu_last.angular_velocity.x, imu_last.angular_velocity.y,
                  imu_last.angular_velocity.z;

                acc_avr << imu_last.linear_acceleration.x, imu_last.linear_acceleration.y,
                  imu_last.linear_acceleration.z;

                imu_upda_cov = true;
                time_update_last = time_current;
                time_predict_last_const = time_current;

                is_first_frame = false;
              }
              time_current = get_time_sec(imu_next.header.stamp);

              if (!is_first_frame) {
                double dt = time_current - time_predict_last_const;
                {
                  double dt_cov = time_current - time_update_last;
                  if (dt_cov > 0.0) {
                    kf_output.predict(dt_cov, Q_output, input_in, false, true);
                    time_update_last = time_current;
                  }
                  kf_output.predict(dt, Q_output, input_in, true, false);
                }

                time_predict_last_const = time_current;

                angvel_avr << imu_next.angular_velocity.x, imu_next.angular_velocity.y,
                  imu_next.angular_velocity.z;
                acc_avr << imu_next.linear_acceleration.x, imu_next.linear_acceleration.y,
                  imu_next.linear_acceleration.z;
                // acc_avr_norm = acc_avr * G_m_s2 / acc_norm;
                kf_output.update_iterated_dyn_share_IMU();
                imu_deque.pop_front();
                if (imu_deque.empty()) break;
                imu_last = imu_next;
                imu_next = *(imu_deque.front());
              } else {
                imu_deque.pop_front();
                if (imu_deque.empty()) break;
                imu_last = imu_next;
                imu_next = *(imu_deque.front());
              }
            }
          }
        }
      } else {
        bool imu_prop_cov = false;
        effct_feat_num = 0;
        if (!time_seq.empty()) {
          double pcl_beg_time = Measures.lidar_beg_time;
          idx = -1;
          for (k = 0; k < time_seq.size(); k++) {
            PointType & point_body = feats_down_body->points[idx + time_seq[k]];
            time_current = point_body.curvature / 1000.0 + pcl_beg_time;
            if (is_first_frame) {
              while (time_current > get_time_sec(imu_next.header.stamp)) {
                imu_deque.pop_front();
                if (imu_deque.empty()) break;
                imu_last = imu_next;
                imu_next = *(imu_deque.front());
              }
              imu_prop_cov = true;

              is_first_frame = false;
              t_last = time_current;
              time_update_last = time_current;
              {
                input_in.gyro << imu_last.angular_velocity.x, imu_last.angular_velocity.y,
                  imu_last.angular_velocity.z;
                input_in.acc << imu_last.linear_acceleration.x, imu_last.linear_acceleration.y,
                  imu_last.linear_acceleration.z;
                input_in.acc = input_in.acc * G_m_s2 / acc_norm;
              }
            }

            while (time_current > get_time_sec(imu_next.header.stamp))  // && !imu_deque.empty())
            {
              imu_deque.pop_front();

              input_in.gyro << imu_last.angular_velocity.x, imu_last.angular_velocity.y,
                imu_last.angular_velocity.z;
              input_in.acc << imu_last.linear_acceleration.x, imu_last.linear_acceleration.y,
                imu_last.linear_acceleration.z;
              input_in.acc = input_in.acc * G_m_s2 / acc_norm;
              double dt = get_time_sec(imu_last.header.stamp) - t_last;

              double dt_cov = get_time_sec(imu_last.header.stamp) - time_update_last;
              if (dt_cov > 0.0) {
                kf_input.predict(dt_cov, Q_input, input_in, false, true);
                time_update_last = get_time_sec(imu_last.header.stamp);  //time_current;
              }
              kf_input.predict(dt, Q_input, input_in, true, false);
              t_last = get_time_sec(imu_last.header.stamp);
              imu_prop_cov = true;

              if (imu_deque.empty()) break;
              imu_last = imu_next;
              imu_next = *(imu_deque.front());
              // imu_upda_cov = true;
            }
            if (flg_reset) {
              break;
            }
            double dt = time_current - t_last;
            t_last = time_current;
            double propag_start = omp_get_wtime();

            if (!prop_at_freq_of_imu) {
              double dt_cov = time_current - time_update_last;
              if (dt_cov > 0.0) {
                kf_input.predict(dt_cov, Q_input, input_in, false, true);
                time_update_last = time_current;
              }
            }
            kf_input.predict(dt, Q_input, input_in, true, false);

            propag_time += omp_get_wtime() - propag_start;

            double t_update_start = omp_get_wtime();

            if (feats_down_size < 1) {
              RCLCPP_WARN(LOGGER, "No point, skip this scan!\n");

              idx += time_seq[k];
              continue;
            }
            appendTemporalGroup(kf_input.x_, idx + 1, time_seq[k]);
            const auto covariance_before = poseCovariance(kf_input.get_P());
            const bool force_update = (k + 1 == static_cast<int>(time_seq.size()));
            const bool should_update = temporal_scheduler.shouldUpdate(
              time_current, time_seq[k], kf_input.x_.pos, kf_input.x_.rot,
              covariance_before.trace(), force_update);
            if (!should_update) {
              ++frame_diagnostics.skipped_updates;
              idx += time_seq[k];
              continue;
            }

            prepareActiveMeasurements(kf_input.x_);
            const double logdet_before = point_lio::stableLogDet(covariance_before);
            const double effective_before = frame_diagnostics.effective_points;
            solve_start = omp_get_wtime();
            if (!kf_input.update_iterated_dyn_share_modified()) {
              if (force_update) {
                temporal_scheduler.recordUpdate(
                  time_current, kf_input.x_.pos, kf_input.x_.rot, 0.0, 0.0);
                pending_temporal_points.clear();
                active_lidar_points.clear();
              }
              idx += time_seq[k];
              continue;
            }
            transformActiveMeasurements(kf_input.x_);
            const double logdet_after =
              point_lio::stableLogDet(poseCovariance(kf_input.get_P()));
            const double information_gain =
              (std::isfinite(logdet_before) && std::isfinite(logdet_after)) ?
              std::max(0.0, 0.5 * (logdet_before - logdet_after)) : 0.0;
            const double effective_update_points =
              std::max(0.0, frame_diagnostics.effective_points - effective_before);
            temporal_scheduler.recordUpdate(
              time_current, kf_input.x_.pos, kf_input.x_.rot, information_gain,
              effective_update_points);
            frame_diagnostics.information_gain_sum += information_gain;
            ++frame_diagnostics.lidar_updates;
            pending_temporal_points.clear();
            active_lidar_points.clear();
            processFullCloudSegmentWithSnapshot(makeStateSnapshot(time_current));

            if (publish_odometry_without_downsample) {
              /******* Publish odometry *******/

              publish_odometry(pub_odom_aft_mapped, tf_broadcaster);
              if (runtime_pos_log) {
                euler_cur = SO3ToEuler(kf_input.x_.rot);
                fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " "
                         << euler_cur.transpose() << " " << kf_input.x_.pos.transpose() << " "
                         << kf_input.x_.vel.transpose() << " " << kf_input.x_.bg.transpose() << " "
                         << kf_input.x_.ba.transpose() << " " << kf_input.x_.gravity.transpose()
                         << " " << feats_undistort->points.size() << '\n';
              }
            }

            solve_time += omp_get_wtime() - solve_start;

            update_time += omp_get_wtime() - t_update_start;
            idx = idx + time_seq[k];
          }
        } else {
          if (!imu_deque.empty()) {
            imu_last = imu_next;
            imu_next = *(imu_deque.front());
            while (get_time_sec(imu_next.header.stamp) > time_current &&
                   ((get_time_sec(imu_next.header.stamp) <
                     Measures.lidar_beg_time + lidar_time_inte))) {  // >= ?
              if (is_first_frame) {
                {
                  {
                    while (get_time_sec(imu_next.header.stamp) <
                           Measures.lidar_beg_time + lidar_time_inte) {
                      imu_deque.pop_front();
                      if (imu_deque.empty()) break;
                      imu_last = imu_next;
                      imu_next = *(imu_deque.front());
                    }
                  }

                  break;
                }
                imu_prop_cov = true;

                t_last = time_current;
                time_update_last = time_current;
                input_in.gyro << imu_last.angular_velocity.x, imu_last.angular_velocity.y,
                  imu_last.angular_velocity.z;
                input_in.acc << imu_last.linear_acceleration.x, imu_last.linear_acceleration.y,
                  imu_last.linear_acceleration.z;
                input_in.acc = input_in.acc * G_m_s2 / acc_norm;

                is_first_frame = false;
              }
              time_current = get_time_sec(imu_next.header.stamp);

              if (!is_first_frame) {
                double dt = time_current - t_last;

                double dt_cov = time_current - time_update_last;
                if (dt_cov > 0.0) {
                  // kf_input.predict(dt_cov, Q_input, input_in, false, true);
                  time_update_last = get_time_sec(imu_next.header.stamp);  //time_current;
                }
                // kf_input.predict(dt, Q_input, input_in, true, false);

                t_last = get_time_sec(imu_next.header.stamp);

                input_in.gyro << imu_next.angular_velocity.x, imu_next.angular_velocity.y,
                  imu_next.angular_velocity.z;
                input_in.acc << imu_next.linear_acceleration.x, imu_next.linear_acceleration.y,
                  imu_next.linear_acceleration.z;
                input_in.acc = input_in.acc * G_m_s2 / acc_norm;
                imu_deque.pop_front();
                if (imu_deque.empty()) break;
                imu_last = imu_next;
                imu_next = *(imu_deque.front());
              } else {
                imu_deque.pop_front();
                if (imu_deque.empty()) break;
                imu_last = imu_next;
                imu_next = *(imu_deque.front());
              }
            }
          }
        }
      }
      // M3D rot_cur_lidar;
      // {
      //     rot_cur_lidar = state.rot_end;
      // }
      // euler_cur = RotMtoEuler(rot_cur_lidar);
      // geoQuat = tf::createQuaternionMsgFromRollPitchYaw
      //                     (euler_cur(0), euler_cur(1), euler_cur(2));
      /******* Publish odometry downsample *******/
      if (!publish_odometry_without_downsample) {
        publish_odometry(pub_odom_aft_mapped, tf_broadcaster);
      }

      /*** add the feature points to map ***/
      t3 = omp_get_wtime();
      if (feats_down_size > 4) {
        if (enable_prior_pcd) {
          sleep_time++;
          if (sleep_time > 200) {
            queueMapIncremental(
              lidar_end_time,
              frame_diagnostics.effective_points >= static_cast<double>(map_min_effective_points));
          }
        } else {
          queueMapIncremental(
            lidar_end_time,
            frame_diagnostics.effective_points >= static_cast<double>(map_min_effective_points));
        }
      }
      processPendingMapUpdates(lidar_end_time);
      t5 = omp_get_wtime();
      /******* Publish points *******/
      if (path_en) publish_path(pub_path);
      if (scan_pub_en || pcd_save_en) publish_frame_world(pub_laser_cloud_full_res);
      if (instantaneous_full_scan) {
        // Every ray belongs to the same acquisition time. Transform the whole
        // snapshot with the FINAL corrected scan pose, matching scan-end odom.
        // The streaming path uses the preceding state to propagate individual
        // points; consuming a snapshot on the first partial update instead
        // leaves the published cloud inconsistent with later same-stamp poses.
        full_cloud_world->clear();
        const StateSnapshot corrected = makeStateSnapshot(lidar_end_time);
        for (const auto & point : feats_undistort->points) {
          if (!isFinitePoint(point)) {continue;}
          FullCloudPoint acquired;
          acquired.point = point;
          acquired.stamp = corrected.stamp;
          const PointType world = transformFullPointForward(corrected, acquired);
          if (isFinitePoint(world)) {full_cloud_world->push_back(world);}
        }
        full_cloud_anchor = corrected;
        has_full_cloud_anchor = true;
      } else if (has_full_cloud_anchor && lidar_end_time > full_cloud_anchor.stamp) {
        processFullCloudSegmentWithSnapshot(makeStateSnapshot(lidar_end_time));
      }
      publishAccumulatedFullCloudWorld(pub_laser_cloud_full_world, lidar_end_time);
      if (scan_pub_en && scan_body_pub_en) publish_frame_body(pub_laser_cloud_full_res_body);
      publish_diagnostics(pub_diagnostics, lidar_end_time, (t5 - t0) * 1000.0);

      /*** Debug variables Logging ***/
      if (runtime_pos_log) {
        frame_num++;
        aver_time_consu = aver_time_consu * (frame_num - 1) / frame_num + (t5 - t0) / frame_num;
        {
          aver_time_icp = aver_time_icp * (frame_num - 1) / frame_num + update_time / frame_num;
        }
        aver_time_match = aver_time_match * (frame_num - 1) / frame_num + (match_time) / frame_num;
        aver_time_solve = aver_time_solve * (frame_num - 1) / frame_num + solve_time / frame_num;
        aver_time_propag = aver_time_propag * (frame_num - 1) / frame_num + propag_time / frame_num;
        if (time_log_counter < MAXN) {
          T1[time_log_counter] = Measures.lidar_beg_time;
          s_plot[time_log_counter] = t5 - t0;
          s_plot2[time_log_counter] = feats_undistort->points.size();
          s_plot3[time_log_counter] = aver_time_consu;
          time_log_counter++;
        }
        printf(
          "[ mapping ]: time: IMU + Map + Input Downsample: %0.6f ave match: %0.6f ave solve: "
          "%0.6f  ave ICP: %0.6f  map incre: %0.6f ave total: %0.6f icp: %0.6f propogate: %0.6f \n",
          t1 - t0, aver_time_match, aver_time_solve, t3 - t1, t5 - t3, aver_time_consu,
          aver_time_icp, aver_time_propag);
        if (!publish_odometry_without_downsample) {
          if (!use_imu_as_input) {
            euler_cur = SO3ToEuler(kf_output.x_.rot);
            fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " "
                     << euler_cur.transpose() << " " << kf_output.x_.pos.transpose() << " "
                     << kf_output.x_.vel.transpose() << " " << kf_output.x_.omg.transpose() << " "
                     << kf_output.x_.acc.transpose() << " " << kf_output.x_.gravity.transpose()
                     << " " << kf_output.x_.bg.transpose() << " " << kf_output.x_.ba.transpose()
                     << " " << feats_undistort->points.size() << '\n';
          } else {
            euler_cur = SO3ToEuler(kf_input.x_.rot);
            fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " "
                     << euler_cur.transpose() << " " << kf_input.x_.pos.transpose() << " "
                     << kf_input.x_.vel.transpose() << " " << kf_input.x_.bg.transpose() << " "
                     << kf_input.x_.ba.transpose() << " " << kf_input.x_.gravity.transpose() << " "
                     << feats_undistort->points.size() << '\n';
          }
        }
        dump_lio_state_to_log(fp);
      }
    }
    rate.sleep();
  }
  //--------------------------save map-----------------------------------
  // 1. make sure you have enough memories
  // 2. noted that pcd save will influence the real-time performances
  if (!pcl_wait_save->empty() && pcd_save_en) {
    string file_name = string("scans.pcd");
    string all_points_dir(string(string(ROOT_DIR) + "PCD/") + file_name);
    pcl::PCDWriter pcd_writer;
    pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
  }
  fout_out.close();
  fout_imu_pbp.close();
  return 0;
}
