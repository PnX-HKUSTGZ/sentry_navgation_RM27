#include <chrono>
#include <memory>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_epsilon_time.h>
#include <message_filters/synchronizer.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include "rm_27_stimulation/pointlio_scan_registration.hpp"

namespace rm_27_stimulation
{
class PointlioScanRegistrar : public rclcpp::Node
{
  using Cloud = sensor_msgs::msg::PointCloud2;
  using Odom = nav_msgs::msg::Odometry;
  using Policy = message_filters::sync_policies::ApproximateEpsilonTime<Cloud, Odom>;
  using Synchronizer = message_filters::Synchronizer<Policy>;
public:
  PointlioScanRegistrar() : Node("pointlio_scan_registrar")
  {
    const auto cloud_topic = declare_parameter<std::string>("cloud_topic", "livox/lidar");
    const auto odom_topic = declare_parameter<std::string>("odom_topic", "aft_mapped_to_init");
    const auto output_topic = declare_parameter<std::string>("output_topic", "pointlio/registered_scan");
    world_frame_ = declare_parameter<std::string>("world_frame", "camera_init");
    body_frame_ = declare_parameter<std::string>("body_frame", "body");
    sensor_frame_ = declare_parameter<std::string>("sensor_frame", "imu_link");
    lidar_frame_ = declare_parameter<std::string>("lidar_frame", "left_mid360");
    max_age_ = declare_parameter<double>("max_scan_age", 0.5);
    config_.horizontal_samples = declare_parameter<int>("lidar_horizontal_samples", 360);
    config_.vertical_samples = declare_parameter<int>("lidar_vertical_samples", 288);
    config_.horizontal_min_angle = declare_parameter<double>("lidar_horizontal_min_angle", 0.0);
    config_.horizontal_max_angle = declare_parameter<double>("lidar_horizontal_max_angle", 6.283185307179586);
    config_.vertical_min_angle = declare_parameter<double>("lidar_vertical_min_angle", -0.1260127724939906);
    config_.vertical_max_angle = declare_parameter<double>("lidar_vertical_max_angle", 0.9637708129512688);
    config_.miss_ray_length = declare_parameter<double>("no_return_ray_length", 10.5);
    config_.rog_raycast_max_range = declare_parameter<double>("rog_raycast_max_range", 10.0);
    config_.rog_map_resolution = declare_parameter<double>("rog_map_resolution", 0.05);
    if (!validMissRayConfig(config_) || !std::isfinite(max_age_) || max_age_ <= 0.) {
      throw std::invalid_argument("invalid scan geometry or max_scan_age");
    }
    buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    listener_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);
    publisher_ = create_publisher<Cloud>(output_topic, rclcpp::SensorDataQoS());
    clouds_.subscribe(this, cloud_topic, rmw_qos_profile_sensor_data);
    poses_.subscribe(this, odom_topic, rmw_qos_profile_sensor_data);
    sync_ = std::make_shared<Synchronizer>(Policy(10,
      rclcpp::Duration::from_nanoseconds(kPointlioStampRoundingNanoseconds)), clouds_, poses_);
    sync_->registerCallback(std::bind(&PointlioScanRegistrar::matched, this,
      std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(get_logger(), "Registering organized lidar with exact-stamp Point-LIO poses; "
      "frames %s/%s (TF alias %s), output=%s. No truth input or TF output.",
      world_frame_.c_str(), body_frame_.c_str(), sensor_frame_.c_str(), output_topic.c_str());
  }
private:
  void matched(const Cloud::ConstSharedPtr & scan, const Odom::ConstSharedPtr & odom)
  {
    const rclcpp::Time stamp(scan->header.stamp);
    const double age = (now() - stamp).seconds();
    if (stamp.nanoseconds() <= last_stamp_ || age < -0.05 || age > max_age_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Discarding stale/out-of-order paired scan: age=%.3f", age);
      return;
    }
    try {
      // Point-LIO's 'body' is the IMU, not base_link. Resolve only IMU->lidar;
      // applying the base mounting angle here again would rotate the scan twice.
      const auto extrinsic = buffer_->lookupTransform(sensor_frame_, lidar_frame_, stamp);
      auto cloud = registerPointlioScan(*scan, *odom, extrinsic.transform,
        config_, world_frame_, body_frame_, lidar_frame_);
      last_stamp_ = stamp.nanoseconds();
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
        "Registered dense scan: stamp=%.3f age=%.3f points=%u",
        stamp.seconds(), age, cloud.width * cloud.height);
      publisher_->publish(std::move(cloud));
    } catch (const std::exception & error) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Cannot register paired scan: %s", error.what());
    }
  }
  OrganizedLidarMissRayConfig config_;
  std::string world_frame_, body_frame_, sensor_frame_, lidar_frame_;
  double max_age_;
  int64_t last_stamp_{0};
  message_filters::Subscriber<Cloud> clouds_;
  message_filters::Subscriber<Odom> poses_;
  std::shared_ptr<Synchronizer> sync_;
  std::shared_ptr<tf2_ros::Buffer> buffer_;
  std::shared_ptr<tf2_ros::TransformListener> listener_;
  rclcpp::Publisher<Cloud>::SharedPtr publisher_;
};
}  // namespace rm_27_stimulation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rm_27_stimulation::PointlioScanRegistrar>());
  rclcpp::shutdown();
  return 0;
}
