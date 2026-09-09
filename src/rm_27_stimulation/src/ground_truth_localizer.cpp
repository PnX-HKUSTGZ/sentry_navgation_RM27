#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl_ros/transforms.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include "rm_27_stimulation/ground_truth_state_buffer.hpp"
#include "rm_27_stimulation/organized_lidar_miss_rays.hpp"
#include "rm_27_stimulation/twist_transform.hpp"

namespace rm_27_stimulation
{

namespace
{

constexpr double kPi = 3.14159265358979323846;

}  // namespace

class GroundTruthLocalizer : public rclcpp::Node {
public:
  GroundTruthLocalizer()
  : Node("rm27_ground_truth_localizer")
  {
    const auto ground_truth_topic = this->declare_parameter<std::string>(
        "ground_truth_odom_topic", "ground_truth/odometry");
    const auto lidar_cloud_topic = this->declare_parameter<std::string>(
        "lidar_cloud_topic", "livox/lidar");
    const auto odometry_topic =
      this->declare_parameter<std::string>("odometry_topic", "odometry");
    const auto lidar_odometry_topic = this->declare_parameter<std::string>(
        "lidar_odometry_topic", "lidar_odometry");
    const auto registered_scan_topic = this->declare_parameter<std::string>(
        "registered_scan_topic", "registered_scan");

    map_frame_ = this->declare_parameter<std::string>("map_frame", "map");
    odom_frame_ = this->declare_parameter<std::string>("odom_frame", "odom");
    robot_base_frame_ = this->declare_parameter<std::string>("robot_base_frame",
                                                             "base_footprint");
    lidar_frame_ =
      this->declare_parameter<std::string>("lidar_frame", "left_mid360");

    const auto lidar_xyz = this->declare_parameter<std::vector<double>>(
        "base_to_lidar_xyz", {0.0, 0.18, 0.14});
    const auto lidar_rpy = this->declare_parameter<std::vector<double>>(
        "base_to_lidar_rpy", {0.0, 0.0, 0.0});
    if (lidar_xyz.size() != 3 || lidar_rpy.size() != 3) {
      throw std::runtime_error(
          "base_to_lidar_xyz and base_to_lidar_rpy must contain 3 values");
    }

    odom_buffer_duration_ =
      this->declare_parameter<double>("odom_buffer_duration", 2.0);
    max_interpolation_gap_ =
      this->declare_parameter<double>("max_interpolation_gap", 0.10);
    scan_match_timeout_ =
      this->declare_parameter<double>("scan_match_timeout", 0.20);
    const auto max_pending_scans =
      this->declare_parameter<int>("max_pending_scans", 20);
    if (!std::isfinite(odom_buffer_duration_) || odom_buffer_duration_ <= 0.0 ||
      !std::isfinite(max_interpolation_gap_) ||
      max_interpolation_gap_ <= 0.0 || !std::isfinite(scan_match_timeout_) ||
      scan_match_timeout_ <= 0.0 || max_pending_scans <= 0)
    {
      throw std::runtime_error("odom_buffer_duration, max_interpolation_gap, "
                               "scan_match_timeout, and "
                               "max_pending_scans must be positive");
    }
    max_pending_scans_ = static_cast<std::size_t>(max_pending_scans);
    state_buffer_.setHistoryDuration(odom_buffer_duration_);

    reconstruct_no_return_rays_ =
      this->declare_parameter<bool>("reconstruct_no_return_rays", false);
    const int horizontal_samples =
      this->declare_parameter<int>("lidar_horizontal_samples", 360);
    const int vertical_samples =
      this->declare_parameter<int>("lidar_vertical_samples", 320);
    const int horizontal_stride =
      this->declare_parameter<int>("no_return_horizontal_stride", 1);
    const int vertical_stride =
      this->declare_parameter<int>("no_return_vertical_stride", 1);
    cycle_no_return_stride_phase_ =
      this->declare_parameter<bool>("cycle_no_return_stride_phase", false);
    if (reconstruct_no_return_rays_ &&
      (horizontal_samples < 2 || vertical_samples < 2 ||
      horizontal_stride < 1 || vertical_stride < 1))
    {
      throw std::runtime_error(
              "organized lidar samples must be >= 2 and miss-ray strides must be >= 1");
    }
    miss_ray_config_.horizontal_samples =
      horizontal_samples >= 0 ? static_cast<uint32_t>(horizontal_samples) : 0U;
    miss_ray_config_.vertical_samples =
      vertical_samples >= 0 ? static_cast<uint32_t>(vertical_samples) : 0U;
    miss_ray_config_.horizontal_stride =
      horizontal_stride >= 0 ? static_cast<uint32_t>(horizontal_stride) : 0U;
    miss_ray_config_.vertical_stride =
      vertical_stride >= 0 ? static_cast<uint32_t>(vertical_stride) : 0U;
    miss_ray_config_.horizontal_min_angle =
      this->declare_parameter<double>("lidar_horizontal_min_angle", 0.0);
    miss_ray_config_.horizontal_max_angle =
      this->declare_parameter<double>("lidar_horizontal_max_angle", 2.0 * kPi);
    miss_ray_config_.vertical_min_angle = this->declare_parameter<double>(
      "lidar_vertical_min_angle", -7.22 * kPi / 180.0);
    miss_ray_config_.vertical_max_angle = this->declare_parameter<double>(
      "lidar_vertical_max_angle", 55.22 * kPi / 180.0);
    miss_ray_config_.miss_ray_length =
      this->declare_parameter<double>("no_return_ray_length", 10.5);
    miss_ray_config_.rog_raycast_max_range =
      this->declare_parameter<double>("rog_raycast_max_range", 10.0);
    miss_ray_config_.rog_map_resolution =
      this->declare_parameter<double>("rog_map_resolution", 0.05);
    if (reconstruct_no_return_rays_ && !validMissRayConfig(miss_ray_config_)) {
      throw std::runtime_error(
              "invalid organized lidar miss-ray parameters; no_return_ray_length "
              "must exceed rog_raycast_max_range by at least two ROG voxels");
    }

    tf2::Quaternion base_to_lidar_rotation;
    base_to_lidar_rotation.setRPY(lidar_rpy[0], lidar_rpy[1], lidar_rpy[2]);
    base_to_lidar_.setOrigin(
        tf2::Vector3(lidar_xyz[0], lidar_xyz[1], lidar_xyz[2]));
    base_to_lidar_.setRotation(base_to_lidar_rotation);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    static_tf_broadcaster_ =
      std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);

    odometry_pub_ =
      this->create_publisher<nav_msgs::msg::Odometry>(odometry_topic, 10);
    lidar_odometry_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
        lidar_odometry_topic, 10);
    registered_scan_pub_ =
      this->create_publisher<sensor_msgs::msg::PointCloud2>(
            registered_scan_topic, rclcpp::SensorDataQoS().keep_last(1));

    ground_truth_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        ground_truth_topic, rclcpp::QoS(10),
        std::bind(&GroundTruthLocalizer::groundTruthCallback, this,
                  std::placeholders::_1));
    lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        lidar_cloud_topic, rclcpp::SensorDataQoS(),
        std::bind(&GroundTruthLocalizer::lidarCallback, this,
                  std::placeholders::_1));
    pending_scan_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&GroundTruthLocalizer::pendingScanTimerCallback, this));
    scan_worker_ = std::thread(&GroundTruthLocalizer::scanWorkerLoop, this);

    RCLCPP_INFO(this->get_logger(),
                "Using Gazebo ground truth from [%s] with %.3fs interpolation "
                "gap and %.3fs scan timeout; organized no-return rays=%s",
                ground_truth_topic.c_str(), max_interpolation_gap_,
                scan_match_timeout_, reconstruct_no_return_rays_ ? "enabled" : "disabled");
  }

  ~GroundTruthLocalizer() override
  {
    {
      std::lock_guard<std::mutex> lock(scan_worker_mutex_);
      stop_scan_worker_ = true;
      pending_ready_scan_.reset();
    }
    scan_worker_cv_.notify_one();
    if (scan_worker_.joinable()) {
      scan_worker_.join();
    }
  }

private:
  using SteadyClock = std::chrono::steady_clock;

  struct PendingScan
  {
    sensor_msgs::msg::PointCloud2::ConstSharedPtr message;
    SteadyClock::time_point received_at;
    int64_t stamp_nanoseconds{0};
  };

  struct ReadyScan
  {
    sensor_msgs::msg::PointCloud2::ConstSharedPtr message;
    GroundTruthState state;
  };

  struct DropStatistics
  {
    std::size_t timed_out{0};
    std::size_t overflow{0};
    std::size_t stale{0};
    std::size_t duplicate{0};
    std::size_t invalid{0};
  };

  static int64_t stampNanoseconds(const builtin_interfaces::msg::Time & stamp)
  {
    constexpr int64_t kNanosecondsPerSecond = 1000000000LL;
    if (stamp.sec < 0 || stamp.nanosec >= kNanosecondsPerSecond) {
      return -1;
    }
    return static_cast<int64_t>(stamp.sec) * kNanosecondsPerSecond +
           static_cast<int64_t>(stamp.nanosec);
  }

  static bool transformFromPose(
    const geometry_msgs::msg::Pose & pose,
    tf2::Transform & transform)
  {
    const auto & position = pose.position;
    const auto & orientation = pose.orientation;
    if (!std::isfinite(position.x) || !std::isfinite(position.y) ||
      !std::isfinite(position.z) || !std::isfinite(orientation.x) ||
      !std::isfinite(orientation.y) || !std::isfinite(orientation.z) ||
      !std::isfinite(orientation.w))
    {
      return false;
    }

    tf2::Quaternion rotation(orientation.x, orientation.y, orientation.z,
      orientation.w);
    if (!std::isfinite(rotation.length2()) || rotation.length2() <= 1.0e-12) {
      return false;
    }
    rotation.normalize();
    transform = tf2::Transform(
        rotation, tf2::Vector3(position.x, position.y, position.z));
    return true;
  }

  static void setPose(
    const tf2::Transform & transform,
    geometry_msgs::msg::Pose & pose)
  {
    pose.position.x = transform.getOrigin().x();
    pose.position.y = transform.getOrigin().y();
    pose.position.z = transform.getOrigin().z();
    pose.orientation = tf2::toMsg(transform.getRotation());
  }

  void groundTruthCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
  {
    const int64_t stamp_nanoseconds = stampNanoseconds(msg->header.stamp);
    tf2::Transform world_to_base;
    if (stamp_nanoseconds < 0 || msg->child_frame_id != robot_base_frame_ ||
      !transformFromPose(msg->pose.pose, world_to_base))
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "Discarding invalid ground-truth odometry "
                           "(stamp/frame/pose contract failed)");
      return;
    }

    tf2::Transform map_to_odom;
    tf2::Transform odom_to_base;
    bool initialized_now = false;
    bool publish_raw_odometry = false;
    bool state_valid = false;
    std::vector<ReadyScan> ready_scans;
    DropStatistics drops;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      map_to_odom = initialized_ ? initial_world_to_base_ : world_to_base;
      odom_to_base = map_to_odom.inverse() * world_to_base;

      GroundTruthState state;
      state.stamp_nanoseconds = stamp_nanoseconds;
      state.odom_to_base = odom_to_base;
      state.base_twist = msg->twist;
      state.pose_covariance = msg->pose.covariance;
      state_valid = state_buffer_.insert(std::move(state));
      if (state_valid) {
        if (!initialized_) {
          initial_world_to_base_ = world_to_base;
          initialized_ = true;
          initialized_now = true;
        }
        if (stamp_nanoseconds > last_raw_odom_stamp_nanoseconds_) {
          last_raw_odom_stamp_nanoseconds_ = stamp_nanoseconds;
          publish_raw_odometry = true;
        }
        collectReadyScansLocked(SteadyClock::now(), ready_scans, drops);
      }
    }

    if (!state_valid) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "Discarding ground-truth odometry containing "
                           "non-finite twist or covariance");
      return;
    }

    if (initialized_now) {
      geometry_msgs::msg::TransformStamped map_transform;
      map_transform.header.stamp = msg->header.stamp;
      map_transform.header.frame_id = map_frame_;
      map_transform.child_frame_id = odom_frame_;
      map_transform.transform = tf2::toMsg(map_to_odom);
      static_tf_broadcaster_->sendTransform(map_transform);
      RCLCPP_INFO(
          this->get_logger(),
          "Initialized map->odom from Gazebo pose: x=%.3f y=%.3f z=%.3f",
          map_to_odom.getOrigin().x(), map_to_odom.getOrigin().y(),
          map_to_odom.getOrigin().z());
    }

    if (publish_raw_odometry) {
      publishRawGroundTruth(*msg, odom_to_base);
    }
    enqueueLatestReadyScan(ready_scans);
    logDrops(drops);
  }

  void lidarCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
  {
    const int64_t stamp_nanoseconds = stampNanoseconds(msg->header.stamp);
    if (stamp_nanoseconds < 0 || msg->header.frame_id != lidar_frame_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "Discarding lidar scan with invalid stamp or "
                           "unexpected frame (expected [%s])",
                           lidar_frame_.c_str());
      return;
    }

    std::vector<ReadyScan> ready_scans;
    DropStatistics drops;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (stamp_nanoseconds <= last_lidar_output_stamp_nanoseconds_) {
        ++drops.stale;
      } else {
        const auto position = std::lower_bound(
            pending_scans_.begin(), pending_scans_.end(), stamp_nanoseconds,
          [](const PendingScan & pending, int64_t stamp) {
            return pending.stamp_nanoseconds < stamp;
            });
        if (position != pending_scans_.end() &&
          position->stamp_nanoseconds == stamp_nanoseconds)
        {
          ++drops.duplicate;
        } else {
          pending_scans_.insert(position, PendingScan{msg, SteadyClock::now(),
              stamp_nanoseconds});
          while (pending_scans_.size() > max_pending_scans_) {
            pending_scans_.pop_front();
            ++drops.overflow;
          }
        }
      }
      collectReadyScansLocked(SteadyClock::now(), ready_scans, drops);
    }

    enqueueLatestReadyScan(ready_scans);
    logDrops(drops);
  }

  void pendingScanTimerCallback()
  {
    std::vector<ReadyScan> ready_scans;
    DropStatistics drops;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      collectReadyScansLocked(SteadyClock::now(), ready_scans, drops);
    }
    enqueueLatestReadyScan(ready_scans);
    logDrops(drops);
  }

  void collectReadyScansLocked(
    const SteadyClock::time_point & now,
    std::vector<ReadyScan> & ready_scans,
    DropStatistics & drops)
  {
    auto pending = pending_scans_.begin();
    while (pending != pending_scans_.end()) {
      if (scanMatchExpired(pending->received_at, now, scan_match_timeout_)) {
        pending = pending_scans_.erase(pending);
        ++drops.timed_out;
      } else {
        ++pending;
      }
    }

    while (!pending_scans_.empty()) {
      const PendingScan & scan = pending_scans_.front();
      if (scan.stamp_nanoseconds <= last_lidar_output_stamp_nanoseconds_) {
        pending_scans_.pop_front();
        ++drops.stale;
        continue;
      }

      GroundTruthState interpolated_state;
      const auto status = state_buffer_.interpolate(
          scan.stamp_nanoseconds, max_interpolation_gap_, interpolated_state);
      if (status == GroundTruthLookupStatus::READY) {
        ready_scans.push_back(
            ReadyScan{scan.message, std::move(interpolated_state)});
        last_lidar_output_stamp_nanoseconds_ = scan.stamp_nanoseconds;
        pending_scans_.pop_front();
        continue;
      }
      if (status == GroundTruthLookupStatus::INVALID_QUERY) {
        pending_scans_.pop_front();
        ++drops.invalid;
        continue;
      }

      // NEED_BRACKET and GAP_TOO_LARGE can still recover from
      // delayed/out-of-order odometry.
      break;
    }
  }

  void publishRawGroundTruth(
    const nav_msgs::msg::Odometry & msg,
    const tf2::Transform & odom_to_base)
  {
    nav_msgs::msg::Odometry odometry = msg;
    odometry.header.frame_id = odom_frame_;
    odometry.child_frame_id = robot_base_frame_;
    setPose(odom_to_base, odometry.pose.pose);
    odometry_pub_->publish(odometry);

    geometry_msgs::msg::TransformStamped base_transform;
    base_transform.header.stamp = msg.header.stamp;
    base_transform.header.frame_id = odom_frame_;
    base_transform.child_frame_id = robot_base_frame_;
    base_transform.transform = tf2::toMsg(odom_to_base);
    tf_broadcaster_->sendTransform(base_transform);
  }

  void enqueueLatestReadyScan(const std::vector<ReadyScan> & ready_scans)
  {
    if (ready_scans.empty()) {
      return;
    }

    const ReadyScan & ready = ready_scans.back();
    // Seed remote TF consumers before reconstruction. The ROG cloud filter
    // normally uses the paired odometry pose directly, but other consumers and
    // its fail-closed compatibility fallback may still request scan-time TF.
    // A transform for a superseded scan remains valid acquisition-time history.
    publishScanTimeBaseTransform(ready);
    bool replaced_pending_scan = false;
    {
      std::lock_guard<std::mutex> lock(scan_worker_mutex_);
      if (stop_scan_worker_) {
        return;
      }
      replaced_pending_scan = pending_ready_scan_.has_value();
      pending_ready_scan_ = ready;
    }
    scan_worker_cv_.notify_one();

    if (ready_scans.size() > 1U || replaced_pending_scan) {
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "Dropping superseded registered scans to keep the ROG-map input current");
    }
  }

  void scanWorkerLoop()
  {
    while (true) {
      std::optional<ReadyScan> ready;
      {
        std::unique_lock<std::mutex> lock(scan_worker_mutex_);
        scan_worker_cv_.wait(lock, [this]() {
          return stop_scan_worker_ || pending_ready_scan_.has_value();
        });
        if (stop_scan_worker_) {
          return;
        }
        ready = std::move(pending_ready_scan_);
        pending_ready_scan_.reset();
      }
      processReadyScan(*ready);
    }
  }

  void processReadyScan(const ReadyScan & ready)
  {
    const tf2::Transform odom_to_lidar =
      ready.state.odom_to_base * base_to_lidar_;

    sensor_msgs::msg::PointCloud2 source_scan;
    const sensor_msgs::msg::PointCloud2 * scan_to_transform = ready.message.get();
    if (reconstruct_no_return_rays_) {
      source_scan = *ready.message;
      auto active_miss_ray_config = miss_ray_config_;
      if (cycle_no_return_stride_phase_) {
        const uint64_t phase_count =
          static_cast<uint64_t>(miss_ray_config_.horizontal_stride) *
          static_cast<uint64_t>(miss_ray_config_.vertical_stride);
        const uint64_t phase_index = miss_ray_phase_index_++ % phase_count;
        active_miss_ray_config.horizontal_phase = static_cast<uint32_t>(
          phase_index % miss_ray_config_.horizontal_stride);
        active_miss_ray_config.vertical_phase = static_cast<uint32_t>(
          (phase_index / miss_ray_config_.horizontal_stride) %
          miss_ray_config_.vertical_stride);
      }
      const auto result = reconstructOrganizedLidarMissRays(
        source_scan, active_miss_ray_config);
      if (result.status != MissRayFillStatus::OK) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "No-return ray reconstruction rejected scan: %s; preserving "
          "non-finite returns so ROG-map remains UNKNOWN",
          missRayFillStatusName(result.status));
      } else {
        RCLCPP_INFO_THROTTLE(
          this->get_logger(), *this->get_clock(), 5000,
          "No-return rays: synthesized=%zu finite_returns=%zu "
          "unsupported_nonfinite=%zu stride_skipped=%zu length=%.2fm "
          "stride=%ux%u phase=%u,%u",
          result.synthesized, result.finite_returns,
          result.unsupported_nonfinite, result.skipped_by_stride,
          active_miss_ray_config.miss_ray_length,
          active_miss_ray_config.horizontal_stride,
          active_miss_ray_config.vertical_stride,
          active_miss_ray_config.horizontal_phase,
          active_miss_ray_config.vertical_phase);
      }
      scan_to_transform = &source_scan;
    }

    sensor_msgs::msg::PointCloud2 compact_scan;
    const auto compaction = compactFiniteXyzPoints(
      *scan_to_transform, compact_scan);
    if (compaction.status == FinitePointCompactionStatus::OK) {
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "Compacted registered scan: kept=%zu removed=%zu",
        compaction.kept, compaction.removed);
      scan_to_transform = &compact_scan;
    } else {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Finite-point compaction rejected scan: %s; publishing the original "
        "layout",
        finitePointCompactionStatusName(compaction.status));
    }

    sensor_msgs::msg::PointCloud2 registered_scan;
    pcl_ros::transformPointCloud(odom_frame_, odom_to_lidar, *scan_to_transform,
                                 registered_scan);
    registered_scan.header.stamp = ready.message->header.stamp;
    registered_scan.header.frame_id = odom_frame_;

    // Publish the acquisition-time TF, lidar odometry, and registered cloud
    // together after all expensive reconstruction/compaction work. Publishing
    // TF and odometry at enqueue time let the worker delay separate this trio
    // by hundreds of milliseconds under load, and it also emitted poses for
    // scans later superseded in the single-slot worker mailbox.
    publishScanTimeBaseTransform(ready);

    nav_msgs::msg::Odometry lidar_odometry;
    lidar_odometry.header.stamp = ready.message->header.stamp;
    lidar_odometry.header.frame_id = odom_frame_;
    lidar_odometry.child_frame_id = lidar_frame_;
    setPose(odom_to_lidar, lidar_odometry.pose.pose);
    lidar_odometry.pose.covariance = ready.state.pose_covariance;
    lidar_odometry.twist =
      transformTwistFromBaseToChild(ready.state.base_twist, base_to_lidar_);
    lidar_odometry_pub_->publish(lidar_odometry);
    registered_scan_pub_->publish(registered_scan);
  }

  void publishScanTimeBaseTransform(const ReadyScan & ready)
  {
    geometry_msgs::msg::TransformStamped base_transform;
    base_transform.header.stamp = ready.message->header.stamp;
    base_transform.header.frame_id = odom_frame_;
    base_transform.child_frame_id = robot_base_frame_;
    base_transform.transform = tf2::toMsg(ready.state.odom_to_base);
    tf_broadcaster_->sendTransform(base_transform);
  }

  void logDrops(const DropStatistics & drops)
  {
    const std::size_t total = drops.timed_out + drops.overflow + drops.stale +
      drops.duplicate + drops.invalid;
    if (total == 0) {
      return;
    }
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "Dropped lidar scans: timeout=%zu overflow=%zu "
                         "stale=%zu duplicate=%zu invalid=%zu",
                         drops.timed_out, drops.overflow, drops.stale,
                         drops.duplicate, drops.invalid);
  }

  std::string map_frame_;
  std::string odom_frame_;
  std::string robot_base_frame_;
  std::string lidar_frame_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster>
    static_tf_broadcaster_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr lidar_odometry_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
    registered_scan_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ground_truth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
  rclcpp::TimerBase::SharedPtr pending_scan_timer_;

  std::mutex scan_worker_mutex_;
  std::condition_variable scan_worker_cv_;
  std::optional<ReadyScan> pending_ready_scan_;
  std::thread scan_worker_;
  bool stop_scan_worker_{false};

  std::mutex state_mutex_;
  GroundTruthStateBuffer state_buffer_;
  std::deque<PendingScan> pending_scans_;
  tf2::Transform initial_world_to_base_;
  tf2::Transform base_to_lidar_;
  double odom_buffer_duration_{2.0};
  double max_interpolation_gap_{0.10};
  double scan_match_timeout_{0.20};
  OrganizedLidarMissRayConfig miss_ray_config_;
  std::size_t max_pending_scans_{20};
  int64_t last_raw_odom_stamp_nanoseconds_{-1};
  int64_t last_lidar_output_stamp_nanoseconds_{-1};
  uint64_t miss_ray_phase_index_{0U};
  bool initialized_{false};
  bool reconstruct_no_return_rays_{false};
  bool cycle_no_return_stride_phase_{false};
};

} // namespace rm_27_stimulation

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rm_27_stimulation::GroundTruthLocalizer>());
  rclcpp::shutdown();
  return 0;
}
