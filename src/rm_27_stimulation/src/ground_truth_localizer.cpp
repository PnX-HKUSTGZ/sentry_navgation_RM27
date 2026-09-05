#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl_ros/transforms.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace rm_27_stimulation
{

class GroundTruthLocalizer : public rclcpp::Node
{
public:
  GroundTruthLocalizer()
  : Node("rm27_ground_truth_localizer")
  {
    const auto ground_truth_topic =
      this->declare_parameter<std::string>("ground_truth_odom_topic", "ground_truth/odometry");
    const auto lidar_cloud_topic =
      this->declare_parameter<std::string>("lidar_cloud_topic", "livox/lidar");
    const auto odometry_topic =
      this->declare_parameter<std::string>("odometry_topic", "odometry");
    const auto lidar_odometry_topic =
      this->declare_parameter<std::string>("lidar_odometry_topic", "lidar_odometry");
    const auto registered_scan_topic =
      this->declare_parameter<std::string>("registered_scan_topic", "registered_scan");

    map_frame_ = this->declare_parameter<std::string>("map_frame", "map");
    odom_frame_ = this->declare_parameter<std::string>("odom_frame", "odom");
    robot_base_frame_ =
      this->declare_parameter<std::string>("robot_base_frame", "base_footprint");
    lidar_frame_ = this->declare_parameter<std::string>("lidar_frame", "left_mid360");

    const auto lidar_xyz = this->declare_parameter<std::vector<double>>(
      "base_to_lidar_xyz", {0.0, 0.18, 0.14});
    const auto lidar_rpy = this->declare_parameter<std::vector<double>>(
      "base_to_lidar_rpy", {0.0, 0.0, 0.0});
    if (lidar_xyz.size() != 3 || lidar_rpy.size() != 3) {
      throw std::runtime_error("base_to_lidar_xyz and base_to_lidar_rpy must contain 3 values");
    }

    tf2::Quaternion base_to_lidar_rotation;
    base_to_lidar_rotation.setRPY(lidar_rpy[0], lidar_rpy[1], lidar_rpy[2]);
    base_to_lidar_.setOrigin(tf2::Vector3(lidar_xyz[0], lidar_xyz[1], lidar_xyz[2]));
    base_to_lidar_.setRotation(base_to_lidar_rotation);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    odometry_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(odometry_topic, 10);
    lidar_odometry_pub_ =
      this->create_publisher<nav_msgs::msg::Odometry>(lidar_odometry_topic, 10);
    registered_scan_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      registered_scan_topic, rclcpp::QoS(5));

    ground_truth_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      ground_truth_topic, rclcpp::QoS(10),
      std::bind(&GroundTruthLocalizer::groundTruthCallback, this, std::placeholders::_1));
    lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      lidar_cloud_topic, rclcpp::SensorDataQoS(),
      std::bind(&GroundTruthLocalizer::lidarCallback, this, std::placeholders::_1));

    RCLCPP_INFO(
      this->get_logger(), "Using Gazebo ground truth from [%s] for simulation localization",
      ground_truth_topic.c_str());
  }

private:
  static void setPose(const tf2::Transform & transform, geometry_msgs::msg::Pose & pose)
  {
    pose.position.x = transform.getOrigin().x();
    pose.position.y = transform.getOrigin().y();
    pose.position.z = transform.getOrigin().z();
    pose.orientation = tf2::toMsg(transform.getRotation());
  }

  void groundTruthCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
  {
    tf2::Transform world_to_base;
    tf2::fromMsg(msg->pose.pose, world_to_base);

    tf2::Transform odom_to_base;
    tf2::Transform odom_to_lidar;
    tf2::Transform map_to_odom;
    bool initialized_now = false;
    {
      std::lock_guard<std::mutex> lock(transform_mutex_);
      if (!initialized_) {
        initial_world_to_base_ = world_to_base;
        initialized_ = true;
        initialized_now = true;
      }

      map_to_odom = initial_world_to_base_;
      odom_to_base = initial_world_to_base_.inverse() * world_to_base;
      odom_to_lidar = odom_to_base * base_to_lidar_;
      latest_odom_to_lidar_ = odom_to_lidar;
    }

    if (initialized_now) {
      RCLCPP_INFO(
        this->get_logger(),
        "Initialized map->odom from Gazebo pose: x=%.3f y=%.3f z=%.3f",
        map_to_odom.getOrigin().x(), map_to_odom.getOrigin().y(),
        map_to_odom.getOrigin().z());
    }

    nav_msgs::msg::Odometry odometry = *msg;
    odometry.header.frame_id = odom_frame_;
    odometry.child_frame_id = robot_base_frame_;
    setPose(odom_to_base, odometry.pose.pose);
    odometry_pub_->publish(odometry);

    nav_msgs::msg::Odometry lidar_odometry = odometry;
    lidar_odometry.child_frame_id = lidar_frame_;
    setPose(odom_to_lidar, lidar_odometry.pose.pose);
    lidar_odometry_pub_->publish(lidar_odometry);

    // Keep both transforms on /tf so late-starting Nav2 buffers get a connected tree.
    geometry_msgs::msg::TransformStamped map_transform;
    map_transform.header.stamp = msg->header.stamp;
    map_transform.header.frame_id = map_frame_;
    map_transform.child_frame_id = odom_frame_;
    map_transform.transform = tf2::toMsg(map_to_odom);

    geometry_msgs::msg::TransformStamped base_transform;
    base_transform.header.stamp = msg->header.stamp;
    base_transform.header.frame_id = odom_frame_;
    base_transform.child_frame_id = robot_base_frame_;
    base_transform.transform = tf2::toMsg(odom_to_base);
    tf_broadcaster_->sendTransform(
      std::vector<geometry_msgs::msg::TransformStamped>{map_transform, base_transform});
  }

  void lidarCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
  {
    tf2::Transform odom_to_lidar;
    {
      std::lock_guard<std::mutex> lock(transform_mutex_);
      if (!initialized_) {
        return;
      }
      odom_to_lidar = latest_odom_to_lidar_;
    }

    sensor_msgs::msg::PointCloud2 registered_scan;
    pcl_ros::transformPointCloud(odom_frame_, odom_to_lidar, *msg, registered_scan);
    registered_scan.header.stamp = msg->header.stamp;
    registered_scan.header.frame_id = odom_frame_;
    registered_scan_pub_->publish(registered_scan);
  }

  std::string map_frame_;
  std::string odom_frame_;
  std::string robot_base_frame_;
  std::string lidar_frame_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr lidar_odometry_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr registered_scan_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ground_truth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;

  std::mutex transform_mutex_;
  tf2::Transform initial_world_to_base_;
  tf2::Transform latest_odom_to_lidar_;
  tf2::Transform base_to_lidar_;
  bool initialized_{false};
};

}  // namespace rm_27_stimulation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rm_27_stimulation::GroundTruthLocalizer>());
  rclcpp::shutdown();
  return 0;
}
