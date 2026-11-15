// Copyright 2025 Lihan Chen
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef FAKE_VEL_TRANSFORM__FAKE_VEL_TRANSFORM_HPP_
#define FAKE_VEL_TRANSFORM__FAKE_VEL_TRANSFORM_HPP_

#include <chrono>
#include <memory>
#include <mutex>
#include <string>

#include "example_interfaces/msg/float32.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rm27_localization_interfaces/msg/localization_status.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace fake_vel_transform
{
/// 底盘速度坐标转换与输入新鲜度、地形及定位许可的最终执行门。
class FakeVelTransform : public rclcpp::Node
{
public:
  /// 声明参数并建立速度、里程计和定位状态订阅。
  explicit FakeVelTransform(const rclcpp::NodeOptions & options);

private:
  /// 后端状态必须新鲜且可信，恢复后要求新的规划路径。
  bool localizationReady() const;
  /// 接收可信快照，失效立即停止并作废恢复前路径许可。
  void localizationCallback(const rm27_localization_interfaces::msg::LocalizationStatus & msg);
  bool localization_guard_enabled_{false}, localization_trusted_{false},
    localization_replanned_{false};
  double localization_timeout_{.5};
  std::uint64_t localization_generation_{0}, localization_epoch_{0};
  std::string localization_instance_;
  bool localization_continuous_{false};  // 同代短暂 TF 等待不冒充全局恢复。
  std::chrono::steady_clock::time_point localization_received_, localization_recovered_;
  rclcpp::Time localization_recovered_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Subscription<rm27_localization_interfaces::msg::LocalizationStatus>::SharedPtr
    localization_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr recovery_plan_sub_;
  /// 旧控制链使用同刻里程计与路径转换速度。
  void syncCallback(
    const nav_msgs::msg::Odometry::ConstSharedPtr & odom,
    const nav_msgs::msg::Path::ConstSharedPtr & local_plan);
  /// 验收里程计数值和时间后更新底盘朝向。
  void odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr & msg);
  /// 记录地形数据到达时间以维护执行许可。
  void terrainMapCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg);
  /// 缓存旧控制链路径时间，配合同步器使用。
  void localPlanCallback(const nav_msgs::msg::Path::ConstSharedPtr & msg);
  /// 验收速度后经统一许可门向底盘发布。
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  /// 验收可选自旋指令，不绕过统一速度许可。
  void cmdSpinCallback(example_interfaces::msg::Float32::SharedPtr msg);
  /// 定时检查失效并发布底盘辅助坐标变换。
  void publishTransform();
  /// 检查位姿、速度、四元数和里程计时间。
  bool validOdometry(
    const nav_msgs::msg::Odometry & odometry, const rclcpp::Time & now, double & yaw,
    const char *& reason) const;
  /// 判断最近里程计是否仍在允许时间窗口内。
  bool odometryFresh(const rclcpp::Time & now) const;
  /// 地形门开启时要求最近观测未超时。
  bool terrainMapFresh(const rclcpp::Time & now) const;
  /// 统一定位和地形许可，失效时输出零速度。
  bool publishGuardedVelocity(const geometry_msgs::msg::Twist & command, const rclcpp::Time & now);
  /// 发布零速度停止底盘运动。
  void publishStop();
  /// 将导航平面速度旋转到当前底盘坐标并叠加允许的自旋。
  geometry_msgs::msg::Twist transformVelocity(
    const geometry_msgs::msg::Twist::SharedPtr & twist, double yaw_diff);

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<example_interfaces::msg::Float32>::SharedPtr cmd_spin_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr terrain_map_sub_;

  message_filters::Subscriber<nav_msgs::msg::Odometry> odom_sub_filter_;
  message_filters::Subscriber<nav_msgs::msg::Path> local_plan_sub_filter_;
  using SyncPolicy =
    message_filters::sync_policies::ApproximateTime<nav_msgs::msg::Odometry, nav_msgs::msg::Path>;
  std::unique_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_chassis_pub_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::TimerBase::SharedPtr timer_;

  std::string robot_base_frame_;
  std::string fake_robot_base_frame_;
  std::string odom_topic_;
  std::string local_plan_topic_;
  std::string cmd_spin_topic_;
  std::string input_cmd_vel_topic_;
  std::string output_cmd_vel_topic_;
  std::string terrain_map_topic_;
  bool enable_cmd_spin_{true};
  bool terrain_guard_enabled_{false};
  bool use_latest_odom_for_cmd_;
  double odom_timeout_;
  double odom_future_tolerance_;
  double terrain_timeout_{0.5};
  float spin_speed_;

  std::mutex cmd_vel_mutex_;
  geometry_msgs::msg::Twist::SharedPtr latest_cmd_vel_;
  double current_robot_base_angle_;
  rclcpp::Time latest_odom_stamp_;
  rclcpp::Time latest_terrain_receive_time_;
  bool has_latest_odom_stamp_{false};
  bool has_latest_terrain_{false};
  bool motion_command_active_{false};
  rclcpp::Time last_controller_activate_time_;
};

}  // namespace fake_vel_transform

#endif  // FAKE_VEL_TRANSFORM__FAKE_VEL_TRANSFORM_HPP_
