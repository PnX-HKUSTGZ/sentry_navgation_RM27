/**
 * This file is part of ROG-Map
 *
 * Copyright 2024 Yunfan REN, MaRS Lab, University of Hong Kong, <mars.hku.hk>
 * Developed by Yunfan REN <renyf at connect dot hku dot hk>
 * for more information see <https://github.com/hku-mars/ROG-Map>.
 * If you use this code, please cite the respective publications as
 * listed on the above website.
 *
 * ROG-Map is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ROG-Map is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with ROG-Map. If not, see <http://www.gnu.org/licenses/>.
 */

#include <cstddef>
#include <cstdint>
#include <vector>

#ifndef ROG_MAP_ROS_HPP
#define ROG_MAP_ROS_HPP

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/create_publisher.hpp>
#include <rclcpp/create_subscription.hpp>
#include <rclcpp/create_timer.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include <rog_map/rog_map.h>
#include <rog_map/rog_map_visualizer.hpp>
#include <rog_map_ros/cloud_registered_crop_filter.hpp>
#include <rog_map_ros/latest_value_mailbox.hpp>
#include <rog_map_ros/timestamped_pose_matcher.hpp>
#include <super_utils/color_msg_utils.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <utility>

namespace rog_map {
using namespace super_utils;

class ROGMapROS : public ROGMap {
  rclcpp::Node::SharedPtr nh_;
  rclcpp_lifecycle::LifecycleNode::SharedPtr lifecycle_nh_;
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_;
  rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics_;
  rclcpp::node_interfaces::NodeTimersInterface::SharedPtr node_timers_;
  rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock_;
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_;
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_parameters_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  // Cloud self-filtering performs an exact-time lookup from its sensor
  // callback.  Keep that lookup on a dedicated listener thread so waiting for
  // a slightly reordered /tf sample cannot starve the callback that inserts
  // the sample into the shared Nav2 buffer.
  std::shared_ptr<tf2_ros::Buffer> cloud_filter_tf_;
  std::unique_ptr<tf2_ros::TransformListener> cloud_filter_tf_listener_;
  std::unique_ptr<ROGMapVisualizer> visualizer_driver_;
  std::unique_ptr<CloudRegisteredCropFilter> cloud_filter_;
  std::mutex map_data_mutex_;
  std::mutex odom_callback_execution_mutex_;
  std::mutex cloud_callback_execution_mutex_;
  std::mutex update_callback_execution_mutex_;
  std::mutex viz_callback_execution_mutex_;
  std::atomic<bool> map_update_waiting_{false};
  std::atomic<bool> callbacks_stopping_{false};

  const double getSystemWalltimeNow() override { return now().seconds(); }

  void getSystemWalltimeNow(rclcpp::Time &_in) { _in = now(); };

  rclcpp::Time now() const { return node_clock_->get_clock()->now(); }

  bool getPriorMapTransform(PriorMapTransform2D &transform) override {
    if (cfg_.prior_map_frame == cfg_.frame_id) {
      transform = PriorMapTransform2D{};
      return true;
    }
    if (!tf_) {
      RCLCPP_WARN_THROTTLE(node_logging_->get_logger(),
                           *node_clock_->get_clock(), 2000,
                           "[ROGMap] prior map TF is unavailable because the "
                           "shared TF buffer is null");
      return false;
    }
    try {
      // lookupTransform(target, source, ...) returns T_target_source.
      const auto tf_msg = tf_->lookupTransform(
          cfg_.prior_map_frame, cfg_.frame_id, tf2::TimePointZero);
      const double tf_stamp =
          static_cast<double>(tf_msg.header.stamp.sec) +
          static_cast<double>(tf_msg.header.stamp.nanosec) * 1.0e-9;
      if (tf_stamp > 0.0) {
        const double age = now().seconds() - tf_stamp;
        if (!std::isfinite(age) || age > cfg_.prior_map_transform_timeout ||
            age < -cfg_.prior_map_transform_future_tolerance) {
          RCLCPP_WARN_THROTTLE(
              node_logging_->get_logger(), *node_clock_->get_clock(), 2000,
              "[ROGMap] prior map TF from '%s' to '%s' is stale/future "
              "(age=%.3f s)",
              cfg_.frame_id.c_str(), cfg_.prior_map_frame.c_str(), age);
          return false;
        }
      }
      transform.tx = tf_msg.transform.translation.x;
      transform.ty = tf_msg.transform.translation.y;
      transform.tz = tf_msg.transform.translation.z;
      const auto &q = tf_msg.transform.rotation;
      const double norm =
          std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
      if (!std::isfinite(norm) || norm <= 1.0e-12) {
        RCLCPP_WARN_THROTTLE(
            node_logging_->get_logger(), *node_clock_->get_clock(), 2000,
            "[ROGMap] prior map TF from '%s' to '%s' has an invalid quaternion",
            cfg_.frame_id.c_str(), cfg_.prior_map_frame.c_str());
        return false;
      }
      const double qx = q.x / norm;
      const double qy = q.y / norm;
      const double qz = q.z / norm;
      const double qw = q.w / norm;
      transform.roll = std::atan2(2.0 * (qw * qx + qy * qz),
                                  1.0 - 2.0 * (qx * qx + qy * qy));
      transform.pitch =
          std::asin(std::clamp(2.0 * (qw * qy - qz * qx), -1.0, 1.0));
      transform.yaw = std::atan2(2.0 * (qw * qz + qx * qy),
                                 1.0 - 2.0 * (qy * qy + qz * qz));
      if (std::isfinite(transform.tx) && std::isfinite(transform.ty) &&
          std::isfinite(transform.tz) && std::isfinite(transform.roll) &&
          std::isfinite(transform.pitch) && std::isfinite(transform.yaw) &&
          std::abs(transform.roll) <= 1.0e-5 &&
          std::abs(transform.pitch) <= 1.0e-5) {
        return true;
      }
      RCLCPP_WARN_THROTTLE(
          node_logging_->get_logger(), *node_clock_->get_clock(), 2000,
          "[ROGMap] prior map TF from '%s' to '%s' is non-finite or "
          "non-horizontal "
          "(roll=%.6f pitch=%.6f); ground support is fail-closed",
          cfg_.frame_id.c_str(), cfg_.prior_map_frame.c_str(), transform.roll,
          transform.pitch);
      return false;
    } catch (const tf2::TransformException &error) {
      RCLCPP_WARN_THROTTLE(
          node_logging_->get_logger(), *node_clock_->get_clock(), 2000,
          "[ROGMap] cannot transform projection from '%s' to prior map frame "
          "'%s': %s",
          cfg_.frame_id.c_str(), cfg_.prior_map_frame.c_str(), error.what());
      return false;
    }
  }

  template <typename NodeT> void bindNode(NodeT node) {
    node_base_ = node->get_node_base_interface();
    node_topics_ = node->get_node_topics_interface();
    node_timers_ = node->get_node_timers_interface();
    node_clock_ = node->get_node_clock_interface();
    node_logging_ = node->get_node_logging_interface();
    node_parameters_ = node->get_node_parameters_interface();
  }

  rclcpp::CallbackGroup::SharedPtr
  createCallbackGroup(rclcpp::CallbackGroupType type) {
    return node_base_->create_callback_group(type);
  }

  template <typename MsgT>
  typename rclcpp::Publisher<MsgT>::SharedPtr
  createPublisher(const std::string &topic, const rclcpp::QoS &qos) {
    return rclcpp::create_publisher<MsgT>(node_parameters_, node_topics_, topic,
                                          qos);
  }

  template <typename MsgT, typename CallbackT>
  typename rclcpp::Subscription<MsgT>::SharedPtr
  createSubscription(const std::string &topic, const rclcpp::QoS &qos,
                     CallbackT &&callback,
                     const rclcpp::SubscriptionOptions &options =
                         rclcpp::SubscriptionOptions()) {
    return rclcpp::create_subscription<MsgT>(
        node_parameters_, node_topics_, topic, qos,
        std::forward<CallbackT>(callback), options);
  }

  template <typename DurationRepT, typename DurationT, typename CallbackT>
  rclcpp::TimerBase::SharedPtr
  createTimer(std::chrono::duration<DurationRepT, DurationT> period,
              CallbackT &&callback,
              rclcpp::CallbackGroup::SharedPtr group = nullptr) {
    return rclcpp::create_timer(node_clock_->get_clock(), period,
                                std::forward<CallbackT>(callback), group,
                                node_base_.get(), node_timers_.get());
  }

  ROGMapVisualizer::Publishers vm_;

  static uint8_t reasonVisualizationValue(ProjectionClassReason reason) {
    // Spread the reason enum over RViz's 0..100 map palette. The exact enum and
    // its text name are reported by the trajectory rejection log.
    return static_cast<uint8_t>(std::min(100, static_cast<int>(reason) * 7));
  }

  struct PendingCloudFrame {
    PointCloud pc;
    Pose pose;
    double odom_age_ms{0.0};
    double sensor_stamp{0.0};
  };

  struct OdomPoseSample {
    Pose pose;
    std::string parent_frame;
    std::string child_frame;
  };

  using CloudPoseMatcher =
      detail::TimestampedPoseMatcher<OdomPoseSample,
                                     sensor_msgs::msg::PointCloud2::UniquePtr>;

  struct ROSCallback {
    rclcpp::CallbackGroup::SharedPtr odom_me_cbk_group, cloud_me_cbk_group,
        update_cbk_group;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub;
    detail::LatestValueMailbox<PendingCloudFrame> pending_frames;
    std::unique_ptr<CloudPoseMatcher> cloud_pose_matcher;
    double last_accepted_cloud_stamp{0.0};
    rclcpp::TimerBase::SharedPtr update_timer;
    mutex updete_lock;
  } rc_;

  bool stampIsAcceptable(double stamp, double timeout,
                         const char *input_name) const {
    const double now_s = now().seconds();
    const double age = now_s - stamp;
    if (!std::isfinite(stamp) || stamp <= 0.0 || !std::isfinite(now_s) ||
        now_s <= 0.0 || !std::isfinite(age) || age > timeout ||
        age < -cfg_.sensor_future_tolerance) {
      RCLCPP_WARN_THROTTLE(
          node_logging_->get_logger(), *node_clock_->get_clock(), 1000,
          "[ROGMap] rejecting %s timestamp: stamp=%.6f age=%.3f timeout=%.3f "
          "future_tolerance=%.3f",
          input_name, stamp, age, timeout, cfg_.sensor_future_tolerance);
      return false;
    }
    return true;
  }

  bool frameIsExpected(const std::string &frame, const char *input_name) const {
    if (!frame.empty() && frame == cfg_.frame_id) {
      return true;
    }
    RCLCPP_WARN_THROTTLE(node_logging_->get_logger(), *node_clock_->get_clock(),
                         1000,
                         "[ROGMap] rejecting %s frame '%s'; expected '%s'",
                         input_name, frame.c_str(), cfg_.frame_id.c_str());
    return false;
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom_msg) {
    std::lock_guard<std::mutex> execution_lock(odom_callback_execution_mutex_);
    if (callbacks_stopping_.load(std::memory_order_acquire)) {
      return;
    }
    const double receive_time = now().seconds();
    const double stamp = rclcpp::Time(odom_msg->header.stamp).seconds();
    if (performance_monitor_) {
      performance_monitor_->recordOdom(receive_time);
    }
    if (!frameIsExpected(odom_msg->header.frame_id, "odometry") ||
        !stampIsAcceptable(stamp, cfg_.odom_timeout, "odometry")) {
      return;
    }
    const auto &position = odom_msg->pose.pose.position;
    const auto &orientation = odom_msg->pose.pose.orientation;
    if (!std::isfinite(position.x) || !std::isfinite(position.y) ||
        !std::isfinite(position.z) || !std::isfinite(orientation.w) ||
        !std::isfinite(orientation.x) || !std::isfinite(orientation.y) ||
        !std::isfinite(orientation.z)) {
      RCLCPP_WARN_THROTTLE(
          node_logging_->get_logger(), *node_clock_->get_clock(), 1000,
          "[ROGMap] rejecting odometry with a non-finite pose");
      return;
    }
    Quatf rotation(orientation.w, orientation.x, orientation.y, orientation.z);
    if (!std::isfinite(rotation.norm()) || rotation.norm() < 1.0e-6F) {
      RCLCPP_WARN_THROTTLE(
          node_logging_->get_logger(), *node_clock_->get_clock(), 1000,
          "[ROGMap] rejecting odometry with an invalid quaternion");
      return;
    }
    rotation.normalize();
    const Pose pose =
        std::make_pair(Vec3f(position.x, position.y, position.z), rotation);
    bool accepted = false;
    {
      std::lock_guard<mutex> lock(rc_.updete_lock);
      if (rc_.cloud_pose_matcher) {
        accepted = rc_.cloud_pose_matcher->addPose(
            stamp, receive_time,
            OdomPoseSample{pose, odom_msg->header.frame_id,
                           odom_msg->child_frame_id});
      }
    }
    if (!accepted) {
      RCLCPP_WARN_THROTTLE(
          node_logging_->get_logger(), *node_clock_->get_clock(), 1000,
          "[ROGMap] rejecting invalid or out-of-order odometry: stamp=%.6f",
          stamp);
      return;
    }
    updateRobotState(pose);
  }

  void cloudCallback(sensor_msgs::msg::PointCloud2::UniquePtr cloud_msg) {
    std::lock_guard<std::mutex> execution_lock(cloud_callback_execution_mutex_);
    if (callbacks_stopping_.load(std::memory_order_acquire)) {
      return;
    }
    const double cbk_t = now().seconds();
    const double msg_stamp = rclcpp::Time(cloud_msg->header.stamp).seconds();
    const double queue_delay_ms =
        msg_stamp > 0.0 ? std::max(0.0, cbk_t - msg_stamp) * 1000.0 : 0.0;
    const double msg_points = static_cast<double>(cloud_msg->width) *
                              static_cast<double>(cloud_msg->height);
    if (performance_monitor_) {
      performance_monitor_->recordCloudCallback(cbk_t, msg_points,
                                                queue_delay_ms, 0.0);
    }
    if (msg_points <= 0.0) {
      if (performance_monitor_) {
        performance_monitor_->recordCloudDropEmpty();
      }
      return;
    }
    if (!frameIsExpected(cloud_msg->header.frame_id, "point cloud") ||
        !stampIsAcceptable(msg_stamp, cfg_.cloud_timeout, "point cloud")) {
      return;
    }
    CloudPoseMatcher::CloudInsertResult insert_result;
    {
      std::lock_guard<mutex> lock(rc_.updete_lock);
      if (!rc_.cloud_pose_matcher) {
        return;
      }
      insert_result = rc_.cloud_pose_matcher->addCloud(msg_stamp, cbk_t,
                                                       std::move(cloud_msg));
    }
    if (insert_result.status != CloudPoseMatcher::CloudInsertStatus::STORED) {
      RCLCPP_WARN_THROTTLE(node_logging_->get_logger(),
                           *node_clock_->get_clock(), 1000,
                           "[ROGMap] rejecting duplicate, invalid, or "
                           "out-of-order point cloud: stamp=%.6f",
                           msg_stamp);
      return;
    }
    if (insert_result.evicted_count > 0U) {
      if (performance_monitor_) {
        for (std::size_t i = 0U; i < insert_result.evicted_count; ++i) {
          performance_monitor_->recordCloudDropOdomTimeout();
        }
      }
      RCLCPP_WARN_THROTTLE(
          node_logging_->get_logger(), *node_clock_->get_clock(), 1000,
          "[ROGMap] pending cloud queue full; evicted %zu oldest frame(s)",
          insert_result.evicted_count);
    }
  }

  void processMatchedCloud(CloudPoseMatcher::Match match) {
    if (!match.payload ||
        !stampIsAcceptable(match.cloud_stamp, cfg_.cloud_timeout,
                           "pending point cloud")) {
      if (performance_monitor_) {
        performance_monitor_->recordCloudDropOdomTimeout();
      }
      return;
    }
    PointCloud temp_pc;
    const auto convert_start = std::chrono::steady_clock::now();
    pcl::fromROSMsg(*match.payload, temp_pc);
    const double convert_time_ms =
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - convert_start)
            .count();
    if (performance_monitor_) {
      performance_monitor_->recordCloudConvertTime(convert_time_ms);
    }
    if (temp_pc.empty()) {
      if (performance_monitor_) {
        performance_monitor_->recordCloudDropEmpty();
      }
      return;
    }
    if (cloud_filter_ && !cloud_filter_->filter(
                             temp_pc, match.payload->header, match.pose.pose,
                             match.pose.parent_frame, match.pose.child_frame)) {
      return;
    }
    if (temp_pc.empty()) {
      if (performance_monitor_) {
        performance_monitor_->recordCloudDropEmpty();
      }
      return;
    }
    {
      std::lock_guard<mutex> lock(rc_.updete_lock);
      if (match.cloud_stamp <= rc_.last_accepted_cloud_stamp + 1.0e-9) {
        return;
      }
      rc_.last_accepted_cloud_stamp = match.cloud_stamp;
      rc_.pending_frames.store(
          PendingCloudFrame{std::move(temp_pc), match.pose.pose,
                            match.stamp_delta * 1000.0, match.cloud_stamp});
      {
        std::lock_guard<std::mutex> map_lock(map_data_mutex_);
        map_empty_ = false;
      }
    }
  }

  void drainPendingClouds() {
    while (true) {
      CloudPoseMatcher::TakeResult result;
      {
        std::lock_guard<mutex> lock(rc_.updete_lock);
        if (!rc_.cloud_pose_matcher) {
          return;
        }
        result = rc_.cloud_pose_matcher->takeNext(now().seconds());
      }

      if (result.status == CloudPoseMatcher::TakeStatus::NONE) {
        return;
      }
      if (result.status == CloudPoseMatcher::TakeStatus::MATCHED &&
          result.match) {
        processMatchedCloud(std::move(*result.match));
        continue;
      }

      if (performance_monitor_) {
        if (result.status ==
            CloudPoseMatcher::TakeStatus::DROPPED_NO_ODOMETRY) {
          performance_monitor_->recordCloudDropNoOdom();
        } else {
          performance_monitor_->recordCloudDropOdomTimeout();
        }
      }
      const char *reason = "SYNC_TOLERANCE";
      if (result.status == CloudPoseMatcher::TakeStatus::DROPPED_NO_ODOMETRY) {
        reason = "NO_ODOMETRY";
      } else if (result.status ==
                 CloudPoseMatcher::TakeStatus::DROPPED_ODOMETRY_TIMEOUT) {
        reason = "ODOMETRY_TIMEOUT";
      }
      RCLCPP_WARN_THROTTLE(
          node_logging_->get_logger(), *node_clock_->get_clock(), 1000,
          "[ROGMap] dropping pending point cloud without acceptable "
          "timestamped odometry: "
          "cloud=%.6f closest_odom=%.6f delta=%.3f tolerance=%.3f reason=%s",
          result.cloud_stamp, result.closest_pose_stamp,
          result.closest_stamp_delta, cfg_.cloud_odom_sync_tolerance, reason);
    }
  }

  void updateCallback() {
    std::lock_guard<std::mutex> execution_lock(
        update_callback_execution_mutex_);
    if (callbacks_stopping_.load(std::memory_order_acquire)) {
      return;
    }
    drainPendingClouds();

    auto pending_frame = rc_.pending_frames.takeLatest();
    if (!pending_frame) {
      bool map_empty = false;
      {
        std::lock_guard<std::mutex> map_lock(map_data_mutex_);
        map_empty = map_empty_;
      }
      if (!map_empty) {
        return;
      }
      static double last_print_t = now().seconds();
      double cur_t = now().seconds();
      if (cfg_.ros_callback_en && (cur_t - last_print_t > 1.0)) {
        std::cout
            << YELLOW
            << " -- [ROG WARN] No point cloud input, check the topic name."
            << RESET << std::endl;
        last_print_t = cur_t;
      }
      return;
    }

    if (pending_frame->pending_count > 1U) {
      std::cout << YELLOW
                << " -- [ROG WARN] Unfinished frame cnt > 1, the map may not "
                   "work in real-time"
                << RESET << std::endl;
    }

    if (performance_monitor_) {
      performance_monitor_->recordValidCloud(pending_frame->value.odom_age_ms);
    }
    map_update_waiting_.store(true, std::memory_order_release);
    std::unique_lock<std::mutex> map_lock(map_data_mutex_);
    map_update_waiting_.store(false, std::memory_order_release);
    updateMapInternal(pending_frame->value.pc, pending_frame->value.pose,
                      pending_frame->value.sensor_stamp);
  }

  void vizCallback() {
    std::lock_guard<std::mutex> execution_lock(viz_callback_execution_mutex_);
    if (callbacks_stopping_.load(std::memory_order_acquire)) {
      return;
    }
    if (!cfg_.visualization_en) {
      return;
    }
    if (!hasVisualizationSubscriber()) {
      return;
    }

    if (map_update_waiting_.load(std::memory_order_acquire)) {
      return;
    }
    std::unique_lock<std::mutex> map_lock(map_data_mutex_, std::try_to_lock);
    if (!map_lock.owns_lock() ||
        map_update_waiting_.load(std::memory_order_acquire)) {
      return;
    }

    std::vector<std::function<void()>> deferred_publications;
    auto defer_message = [&deferred_publications](auto publisher,
                                                  auto message) {
      deferred_publications.emplace_back(
          [publisher = std::move(publisher),
           message = std::move(message)]() mutable {
            publisher->publish(std::move(message));
          });
    };
    auto defer_xyz_cloud = [this, &deferred_publications](auto publisher,
                                                          vec_E<Vec3f> points) {
      deferred_publications.emplace_back(
          [this, publisher = std::move(publisher),
           points = std::move(points)]() mutable {
            sensor_msgs::msg::PointCloud2 message;
            vecEVec3fToPC2(points, message);
            publisher->publish(std::move(message));
          });
    };
    const auto visualization_start = std::chrono::steady_clock::now();
    if (map_empty_) {
      return;
    }

    const RobotState robot_state = getRobotState();
    Vec3f box_max = robot_state.p + cfg_.visualization_range / 2;
    Vec3f box_min = robot_state.p - cfg_.visualization_range / 2;

    boundBoxByLocalMap(box_min, box_max);
    if ((box_max - box_min).minCoeff() <= 0) {
      cout << YELLOW << " -- [ROGMap] Visualization range is too small."
           << RESET << endl;
      return;
    }

    if (vm_.unknown_pub && vm_.unknown_pub->get_subscription_count() >= 1) {
      vec_E<Vec3f> unknown_map;
      boxSearch(box_min, box_max, UNKNOWN, unknown_map);
      defer_xyz_cloud(vm_.unknown_pub, std::move(unknown_map));
    }
    if (cfg_.unk_inflation_en && vm_.unknown_inf_pub &&
        vm_.unknown_inf_pub->get_subscription_count() >= 1) {
      vec_E<Vec3f> inf_unknown_map;
      boxSearchInflate(box_min, box_max, UNKNOWN, inf_unknown_map);
      defer_xyz_cloud(vm_.unknown_inf_pub, std::move(inf_unknown_map));
    }

    if (layer_ && !layer_->empty()) {
      if (vm_.layer_value_pub &&
          vm_.layer_value_pub->get_subscription_count() >= 1) {
        nav_msgs::msg::OccupancyGrid grid;
        fillLayerMaskGrid(fused_projection_mask_, grid);
        defer_message(vm_.layer_value_pub, std::move(grid));
      }
      if (vm_.layer_value_dynamic_pub &&
          vm_.layer_value_dynamic_pub->get_subscription_count() >= 1) {
        nav_msgs::msg::OccupancyGrid grid;
        fillLayerMaskGrid(layer_->mask(), grid);
        defer_message(vm_.layer_value_dynamic_pub, std::move(grid));
      }
      if (vm_.layer_value_static_pub &&
          vm_.layer_value_static_pub->get_subscription_count() >= 1) {
        nav_msgs::msg::OccupancyGrid grid;
        fillLayerMaskGrid(prior_projection_mask_, grid);
        defer_message(vm_.layer_value_static_pub, std::move(grid));
      }
      if (vm_.layer_type_pub &&
          vm_.layer_type_pub->get_subscription_count() >= 1) {
        std::vector<uint8_t> types(layer_->cells().size(), 0U);
        for (size_t i = 0; i < layer_->cells().size(); ++i) {
          switch (layer_->cells()[i].type) {
          case CellType::UNKNOWN:
            types[i] = 0U;
            break;
          case CellType::FREE:
            types[i] = 33U;
            break;
          case CellType::PASSABLE:
            types[i] = 66U;
            break;
          case CellType::OCCUPIED:
            types[i] = 100U;
            break;
          }
        }
        nav_msgs::msg::OccupancyGrid grid;
        fillLayerGrid(types, grid);
        defer_message(vm_.layer_type_pub, std::move(grid));
      }
      if (vm_.layer_confidence_pub &&
          vm_.layer_confidence_pub->get_subscription_count() >= 1) {
        std::vector<uint8_t> confidence(layer_->cells().size(), 0U);
        for (size_t i = 0; i < layer_->cells().size(); ++i) {
          confidence[i] = static_cast<uint8_t>(
              std::clamp(static_cast<int>(std::round(
                             layer_->cells()[i].confidence * 100.0f)),
                         0, 100));
        }
        nav_msgs::msg::OccupancyGrid grid;
        fillLayerGrid(confidence, grid);
        defer_message(vm_.layer_confidence_pub, std::move(grid));
      }
      if (vm_.layer_headroom_ratio_pub &&
          vm_.layer_headroom_ratio_pub->get_subscription_count() >= 1) {
        std::vector<uint8_t> ratio(layer_->cells().size(), 0U);
        for (size_t i = 0; i < layer_->cells().size(); ++i) {
          ratio[i] = static_cast<uint8_t>(
              std::clamp(static_cast<int>(std::round(
                             layer_->cells()[i].headroom_known_ratio * 100.0F)),
                         0, 100));
        }
        nav_msgs::msg::OccupancyGrid grid;
        fillLayerGrid(ratio, grid);
        defer_message(vm_.layer_headroom_ratio_pub, std::move(grid));
      }
      if (vm_.layer_height_delta_pub &&
          vm_.layer_height_delta_pub->get_subscription_count() >= 1) {
        sensor_msgs::msg::PointCloud2 height_delta_cloud;
        fillLayerHeightDeltaCloud(height_delta_cloud);
        defer_message(vm_.layer_height_delta_pub,
                      std::move(height_delta_cloud));
      }
      if (vm_.layer_headroom_pub &&
          vm_.layer_headroom_pub->get_subscription_count() >= 1) {
        sensor_msgs::msg::PointCloud2 headroom_cloud;
        fillLayerHeadroomCloud(headroom_cloud);
        defer_message(vm_.layer_headroom_pub, std::move(headroom_cloud));
      }
      if (vm_.layer_clearance_status_pub &&
          vm_.layer_clearance_status_pub->get_subscription_count() >= 1) {
        std::vector<uint8_t> status(layer_->cells().size(), 0U);
        for (size_t i = 0; i < layer_->cells().size(); ++i) {
          const auto &cell = layer_->cells()[i];
          if (cell.ground_candidate == 0U) {
            // 25 distinguishes verified underpass/overhead clearance from a
            // column that has neither a ground candidate nor valid clearance.
            if ((cell.raw_reason ==
                     ProjectionClassReason::ROBOT_FOOTPRINT_CLEAR ||
                 cell.raw_reason ==
                     ProjectionClassReason::SURVEYED_NEAR_FIELD_CLEAR) &&
                cell.ground_support_verified != 0U) {
              status[i] = 5U;
            } else if (cell.empty_support_verified != 0U) {
              status[i] = 4U;
            } else {
              status[i] = cell.clearance_verified != 0U ? 25U : 0U;
            }
          } else if (cell.ground_verified == 0U) {
            status[i] = 100U;
          } else if (cell.clearance_verified == 0U) {
            status[i] = 50U;
          } else if (cell.ground_bridge_verified != 0U) {
            // A bounded bridge has lower confidence than directly connected
            // ground, but still requires observed body-volume clearance.
            status[i] = 2U;
          } else if (cell.ground_support_verified != 0U) {
            status[i] = 3U;
          } else {
            status[i] = 1U;
          }
        }
        nav_msgs::msg::OccupancyGrid grid;
        fillLayerGrid(status, grid);
        defer_message(vm_.layer_clearance_status_pub, std::move(grid));
      }
      if (vm_.layer_reason_pub &&
          vm_.layer_reason_pub->get_subscription_count() >= 1) {
        std::vector<uint8_t> reasons(layer_->cells().size(), 0U);
        for (size_t i = 0; i < layer_->cells().size(); ++i) {
          reasons[i] = reasonVisualizationValue(layer_->cells()[i].raw_reason);
        }
        nav_msgs::msg::OccupancyGrid grid;
        fillLayerGrid(reasons, grid);
        defer_message(vm_.layer_reason_pub, std::move(grid));
      }
    }

    if (field_ && field_->isValid() && vm_.field_pub &&
        vm_.field_pub->get_subscription_count() >= 1) {
      sensor_msgs::msg::PointCloud2 field_cloud;
      fillFieldCloud(field_cloud);
      defer_message(vm_.field_pub, std::move(field_cloud));
    }
    if (vm_.decay_cells_pub &&
        vm_.decay_cells_pub->get_subscription_count() >= 1) {
      sensor_msgs::msg::PointCloud2 decay_cloud;
      fillDecayCellsCloud(decay_cloud);
      defer_message(vm_.decay_cells_pub, std::move(decay_cloud));
    }

    if (cfg_.frontier_extraction_en && vm_.frontier_pub &&
        vm_.frontier_pub->get_subscription_count() >= 1) {
      vec_E<Vec3f> frontier_map;
      boxSearch(box_min, box_max, FRONTIER, frontier_map);
      defer_xyz_cloud(vm_.frontier_pub, std::move(frontier_map));
    }

    vec_E<Vec3f> occ_map;
    bool original_occ_available = false;

    if (vm_.occ_pub && vm_.occ_pub->get_subscription_count() >= 1) {
      boxSearch(box_min, box_max, OCCUPIED, occ_map);
      original_occ_available = true;
    }

    if (vm_.raw_occ_pub && vm_.raw_occ_pub->get_subscription_count() >= 1) {
      vec_E<Vec3f> raw_occ_map;
      rawOccupiedBoxSearch(box_min, box_max, raw_occ_map);

      static auto last_raw_occ_log = std::chrono::steady_clock::time_point{};
      const auto log_now = std::chrono::steady_clock::now();
      if (last_raw_occ_log.time_since_epoch().count() == 0 ||
          std::chrono::duration<double>(log_now - last_raw_occ_log).count() >=
              1.0) {
        if (!original_occ_available) {
          boxSearch(box_min, box_max, OCCUPIED, occ_map);
        }
        std::cout << "[ROGMapViz] raw_occupied_points=" << raw_occ_map.size()
                  << ", original_occupied_points=" << occ_map.size()
                  << std::endl;
        last_raw_occ_log = log_now;
      }
      defer_xyz_cloud(vm_.raw_occ_pub, std::move(raw_occ_map));
    }

    if (vm_.occ_inf_pub && vm_.occ_inf_pub->get_subscription_count() >= 1) {
      vec_E<Vec3f> inf_occ_map;
      boxSearchInflate(box_min, box_max, OCCUPIED, inf_occ_map);
      defer_xyz_cloud(vm_.occ_inf_pub, std::move(inf_occ_map));
    }
    if (original_occ_available) {
      defer_xyz_cloud(vm_.occ_pub, std::move(occ_map));
    }

    /* visualize ESDF Map*/
    if (cfg_.esdf_en) {
      if (vm_.esdf_pub && vm_.esdf_pub->get_subscription_count() >= 1) {
        PointCloud pc;
        esdf_map_->getPositiveESDFPointCloud(box_min, box_max,
                                             robot_state.p.z() - 0.5, pc);
        auto publisher = vm_.esdf_pub;
        deferred_publications.emplace_back([this,
                                            publisher = std::move(publisher),
                                            pc = std::move(pc)]() mutable {
          sensor_msgs::msg::PointCloud2 message;
          pcl::toROSMsg(pc, message);
          message.header.frame_id = cfg_.visualization_frame_id;
          message.header.stamp = now();
          publisher->publish(std::move(message));
        });
      }

      // if (vm_.esdf_neg_pub->get_subscription_count() >= 1) {
      //     PointCloud pc;
      //     esdf_map_->getNegativeESDFPointCloud(box_min, box_max,
      //     robot_state.p.z() - 0.5, pc); pcl::toROSMsg(pc, cloud_msg);
      //     cloud_msg.header.frame_id = "world";
      //     cloud_msg.header.stamp = now();
      //     vm_.esdf_neg_pub->publish(cloud_msg);
      // }

#ifdef ESDF_MAP_DEBUG
      sensor_msgs::msg::PointCloud2 cloud_msg;
      esdf_map_->getESDFOccPC2(box_min, box_max, cloud_msg);
      cloud_msg.header.stamp = now();
      defer_message(vm_.esdf_occ_pub, std::move(cloud_msg));
#endif
    }

    if (vm_.mkr_arr_pub && vm_.mkr_arr_pub->get_subscription_count() >= 1) {
      visualization_msgs::msg::MarkerArray mkr_arr;
      visualizeBoundingBox(mkr_arr, now().seconds(), box_min, box_max,
                           "Visualization Range", Color::Purple());
      visualizeText(mkr_arr, now().seconds(), "Visualization Range Text",
                    "Visualization Range", box_max + Vec3f(0, 0, 0.5),
                    Color::Purple(), 0.6, 0);

      Vec3f local_map_max(999, 999, 999), local_map_min(-999, -999, -999);
      boundBoxByLocalMap(local_map_min, local_map_max);
      visualizeBoundingBox(mkr_arr, now().seconds(), local_map_min,
                           local_map_max, "Local Map Range", Color::Orange());
      visualizeText(mkr_arr, now().seconds(), "Local Map Range Text",
                    "Local Map Range", local_map_max + Vec3f(0, 0, 1.0),
                    Color::Orange(), 0.6, 0);

      visualizeBoundingBox(
          mkr_arr, now().seconds(), raycast_data_.cache_box_min,
          raycast_data_.cache_box_max, "Updating Range", Color::Green());
      visualizeText(mkr_arr, now().seconds(), "Updating Range Text",
                    "Updating Range",
                    raycast_data_.cache_box_max + Vec3f(0, 0, 0.5),
                    Color::Green(), 0.6, 0);

      visualizePoint(mkr_arr, now().seconds(), local_map_origin_d_,
                     Color::Red(), "Local Map Origin", 0.2, 0);

      if (cfg_.esdf_en) {
        Vec3f esdf_box_max, esdf_box_min;
        esdf_map_->getUpdatedBbox(esdf_box_min, esdf_box_max);
        visualizeText(mkr_arr, now().seconds(), "ESDF Map Text", "ESDF Map",
                      esdf_box_max + Vec3f(0, 0, 1.0), Color::Blue(), 0.6, 0);
        visualizeBoundingBox(mkr_arr, now().seconds(), esdf_box_min,
                             esdf_box_max, "ESDF Updating Range",
                             Color::Blue());
      }

      for (auto &marker : mkr_arr.markers) {
        marker.header.frame_id = cfg_.visualization_frame_id;
      }
      defer_message(vm_.mkr_arr_pub, std::move(mkr_arr));
    }

    map_lock.unlock();
    const double map_lock_time_ms =
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - visualization_start)
            .count();
    if (map_lock_time_ms > 250.0) {
      RCLCPP_WARN_THROTTLE(
          node_logging_->get_logger(), *node_clock_->get_clock(), 2000,
          "[ROGMap] visualization held the map lock for %.1f ms",
          map_lock_time_ms);
    }
    for (auto &publish : deferred_publications) {
      publish();
    }
  }

  bool hasVisualizationSubscriber() {
    return (vm_.unknown_pub &&
            vm_.unknown_pub->get_subscription_count() >= 1) ||
           (vm_.unknown_inf_pub &&
            vm_.unknown_inf_pub->get_subscription_count() >= 1) ||
           (vm_.layer_value_pub &&
            vm_.layer_value_pub->get_subscription_count() >= 1) ||
           (vm_.layer_value_dynamic_pub &&
            vm_.layer_value_dynamic_pub->get_subscription_count() >= 1) ||
           (vm_.layer_value_static_pub &&
            vm_.layer_value_static_pub->get_subscription_count() >= 1) ||
           (vm_.layer_type_pub &&
            vm_.layer_type_pub->get_subscription_count() >= 1) ||
           (vm_.layer_confidence_pub &&
            vm_.layer_confidence_pub->get_subscription_count() >= 1) ||
           (vm_.layer_headroom_ratio_pub &&
            vm_.layer_headroom_ratio_pub->get_subscription_count() >= 1) ||
           (vm_.layer_height_delta_pub &&
            vm_.layer_height_delta_pub->get_subscription_count() >= 1) ||
           (vm_.layer_headroom_pub &&
            vm_.layer_headroom_pub->get_subscription_count() >= 1) ||
           (vm_.layer_clearance_status_pub &&
            vm_.layer_clearance_status_pub->get_subscription_count() >= 1) ||
           (vm_.layer_reason_pub &&
            vm_.layer_reason_pub->get_subscription_count() >= 1) ||
           (vm_.field_pub && vm_.field_pub->get_subscription_count() >= 1) ||
           (vm_.decay_cells_pub &&
            vm_.decay_cells_pub->get_subscription_count() >= 1) ||
           (vm_.frontier_pub &&
            vm_.frontier_pub->get_subscription_count() >= 1) ||
           (vm_.occ_pub && vm_.occ_pub->get_subscription_count() >= 1) ||
           (vm_.raw_occ_pub &&
            vm_.raw_occ_pub->get_subscription_count() >= 1) ||
           (vm_.occ_inf_pub &&
            vm_.occ_inf_pub->get_subscription_count() >= 1) ||
           (vm_.esdf_pub && vm_.esdf_pub->get_subscription_count() >= 1) ||
           (vm_.mkr_arr_pub && vm_.mkr_arr_pub->get_subscription_count() >= 1);
  }

  void vecEVec3fToPC2(const vec_E<Vec3f> &points,
                      sensor_msgs::msg::PointCloud2 &cloud) {
    // 设置header信息
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    pcl_cloud.resize(points.size());
    for (long unsigned int i = 0; i < points.size(); i++) {
      pcl_cloud[i].x = static_cast<float>(points[i][0]);
      pcl_cloud[i].y = static_cast<float>(points[i][1]);
      pcl_cloud[i].z = static_cast<float>(points[i][2]);
    }
    pcl::toROSMsg(pcl_cloud, cloud);
    cloud.header.stamp = now();
    cloud.header.frame_id = cfg_.visualization_frame_id;
  }

  void fillLayerGrid(const std::vector<uint8_t> &data,
                     nav_msgs::msg::OccupancyGrid &grid) {
    grid.header.stamp = now();
    grid.header.frame_id = cfg_.visualization_frame_id;
    grid.info.resolution = static_cast<float>(layer_->resolution());
    grid.info.width = static_cast<uint32_t>(layer_->width());
    grid.info.height = static_cast<uint32_t>(layer_->height());
    grid.info.origin.position.x = layer_->origin().x();
    grid.info.origin.position.y = layer_->origin().y();
    grid.info.origin.orientation.w = 1.0;
    grid.data.resize(data.size());
    for (size_t i = 0; i < data.size(); ++i) {
      grid.data[i] = static_cast<int8_t>(std::min<int>(100, data[i]));
    }
  }

  void fillLayerMaskGrid(const std::vector<uint8_t> &mask,
                         nav_msgs::msg::OccupancyGrid &grid) {
    std::vector<uint8_t> occupancy(mask.size(), 0U);
    for (size_t i = 0; i < mask.size(); ++i) {
      occupancy[i] = mask[i] == 0U ? 100U : 0U;
    }
    fillLayerGrid(occupancy, grid);
  }

  void fillLayerHeightDeltaCloud(sensor_msgs::msg::PointCloud2 &cloud) {
    pcl::PointCloud<pcl::PointXYZI> pcl_cloud;
    const auto &cells = layer_->cells();
    pcl_cloud.reserve(cells.size());
    for (int y = 0; y < layer_->height(); ++y) {
      for (int x = 0; x < layer_->width(); ++x) {
        const size_t idx =
            static_cast<size_t>(y) * static_cast<size_t>(layer_->width()) +
            static_cast<size_t>(x);
        if (idx >= cells.size()) {
          continue;
        }
        const auto &cell = cells[idx];
        if (cell.type == CellType::UNKNOWN) {
          continue;
        }
        pcl::PointXYZI p;
        p.x = static_cast<float>(layer_->origin().x() +
                                 (static_cast<double>(x) + 0.5) *
                                     layer_->resolution());
        p.y = static_cast<float>(layer_->origin().y() +
                                 (static_cast<double>(y) + 0.5) *
                                     layer_->resolution());
        p.z = cell.occupied_z_max_abs;
        p.intensity = cell.height_delta;
        pcl_cloud.push_back(p);
      }
    }
    pcl::toROSMsg(pcl_cloud, cloud);
    cloud.header.stamp = now();
    cloud.header.frame_id = cfg_.visualization_frame_id;
  }

  void fillLayerHeadroomCloud(sensor_msgs::msg::PointCloud2 &cloud) {
    pcl::PointCloud<pcl::PointXYZI> pcl_cloud;
    const auto &cells = layer_->cells();
    pcl_cloud.reserve(cells.size());
    for (int y = 0; y < layer_->height(); ++y) {
      for (int x = 0; x < layer_->width(); ++x) {
        const size_t idx =
            static_cast<size_t>(y) * static_cast<size_t>(layer_->width()) +
            static_cast<size_t>(x);
        if (idx >= cells.size() || cells[idx].ground_candidate == 0U ||
            !std::isfinite(cells[idx].ground_z_abs)) {
          continue;
        }
        pcl::PointXYZI p;
        p.x = static_cast<float>(layer_->origin().x() +
                                 (static_cast<double>(x) + 0.5) *
                                     layer_->resolution());
        p.y = static_cast<float>(layer_->origin().y() +
                                 (static_cast<double>(y) + 0.5) *
                                     layer_->resolution());
        p.z = cells[idx].ground_z_abs;
        // -1 means that no occupied ceiling was observed in this column.
        p.intensity =
            std::isfinite(cells[idx].headroom) ? cells[idx].headroom : -1.0F;
        pcl_cloud.push_back(p);
      }
    }
    pcl::toROSMsg(pcl_cloud, cloud);
    cloud.header.stamp = now();
    cloud.header.frame_id = cfg_.visualization_frame_id;
  }

  void fillFieldCloud(sensor_msgs::msg::PointCloud2 &cloud) {
    pcl::PointCloud<pcl::PointXYZI> pcl_cloud;
    const auto distances = field_->distances();
    const int width = field_->width();
    const int height = field_->height();
    const double resolution = field_->resolution();
    const Eigen::Vector2d origin = field_->origin();
    pcl_cloud.reserve(distances.size());
    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        const size_t idx = static_cast<size_t>(y) * static_cast<size_t>(width) +
                           static_cast<size_t>(x);
        if (idx >= distances.size()) {
          continue;
        }
        const double d = distances[idx];
        if (!std::isfinite(d)) {
          continue;
        }
        pcl::PointXYZI p;
        p.x = static_cast<float>(origin.x() +
                                 (static_cast<double>(x) + 0.5) * resolution);
        p.y = static_cast<float>(origin.y() +
                                 (static_cast<double>(y) + 0.5) * resolution);
        p.z = 0.0f;
        p.intensity = static_cast<float>(d);
        pcl_cloud.push_back(p);
      }
    }
    pcl::toROSMsg(pcl_cloud, cloud);
    cloud.header.stamp = now();
    cloud.header.frame_id = cfg_.visualization_frame_id;
  }

  void fillDecayCellsCloud(sensor_msgs::msg::PointCloud2 &cloud) {
    pcl::PointCloud<pcl::PointXYZI> pcl_cloud;
    pcl_cloud.reserve(active_ids_.size());
    const double now_s = now().seconds();
    for (const int hash_id : active_ids_) {
      if (hash_id < 0 ||
          hash_id >= static_cast<int>(occupancy_buffer_.size()) ||
          !active_flags_[hash_id] || !isOccupied(occupancy_buffer_[hash_id])) {
        continue;
      }
      Vec3f pos;
      hashIdToPos(hash_id, pos);
      pcl::PointXYZI p;
      p.x = static_cast<float>(pos.x());
      p.y = static_cast<float>(pos.y());
      p.z = static_cast<float>(pos.z());
      p.intensity = static_cast<float>(
          std::max(0.0, now_s - static_cast<double>(last_hit_time_[hash_id])));
      pcl_cloud.push_back(p);
    }
    pcl::toROSMsg(pcl_cloud, cloud);
    cloud.header.stamp = now();
    cloud.header.frame_id = cfg_.visualization_frame_id;
  }

  void initializeRos() {
    // TODO: The current implementation uses a lenient QoS configuration for
    // message transmission.
    const rclcpp::QoS qos(
        rclcpp::QoS(1).best_effort().keep_last(1).durability_volatile());

    if (cfg_.prior_map_enable) {
      prior_map_ =
          loadPriorMap(cfg_.prior_map_yaml_path, cfg_.prior_map_pgm_path);
      const auto occupied_count = std::count(prior_map_.occupied.begin(),
                                             prior_map_.occupied.end(), 1U);
      const auto known_free_count = std::count(prior_map_.known_free.begin(),
                                               prior_map_.known_free.end(), 1U);
      const auto unknown_count = prior_map_.occupied.size() -
                                 static_cast<size_t>(occupied_count) -
                                 static_cast<size_t>(known_free_count);
      RCLCPP_INFO(
          node_logging_->get_logger(),
          "[ROGMap] loaded prior map %dx%d at %.3f m/cell in frame '%s': "
          "occupied=%zu known_free=%zu unknown=%zu",
          prior_map_.width, prior_map_.height, prior_map_.resolution,
          cfg_.prior_map_frame.c_str(), static_cast<size_t>(occupied_count),
          static_cast<size_t>(known_free_count), unknown_count);
      if (prior_map_.ground_elevation_loaded) {
        const auto grid_support_count = std::count_if(
            prior_map_.ground_elevation_grid.begin(),
            prior_map_.ground_elevation_grid.end(), [this](uint16_t value) {
              return value != prior_map_.ground_elevation_grid_no_data;
            });
        RCLCPP_INFO(node_logging_->get_logger(),
                    "[ROGMap] loaded ground-elevation support: default=%.3f m, "
                    "grid=%s (%zu supported cells), patches=%zu",
                    prior_map_.ground_elevation_default_height,
                    prior_map_.ground_elevation_grid_loaded ? "enabled"
                                                            : "disabled",
                    static_cast<size_t>(grid_support_count),
                    prior_map_.ground_elevation_patches.size());
      } else if (cfg_.require_ground_support) {
        RCLCPP_ERROR(
            node_logging_->get_logger(),
            "[ROGMap] projection.prior_map.require_ground_support is true but "
            "the prior YAML has no ground_elevation block; all ground support "
            "authorization remains fail-closed");
      }
    }
    if (cfg_.cloud_filter_en) {
      cloud_filter_tf_ =
          std::make_shared<tf2_ros::Buffer>(node_clock_->get_clock());
      cloud_filter_tf_listener_ =
          std::make_unique<tf2_ros::TransformListener>(*cloud_filter_tf_, true);
      CloudRegisteredCropFilter::MarkerPublisher::SharedPtr marker_publisher;
      if (cfg_.cloud_filter_publish_visualization) {
        marker_publisher =
            createPublisher<visualization_msgs::msg::MarkerArray>(
                cfg_.cloud_filter_visualization_topic,
                rclcpp::QoS(rclcpp::KeepLast(1)).transient_local());
      }
      cloud_filter_ = std::make_unique<CloudRegisteredCropFilter>(
          cfg_, cloud_filter_tf_, node_clock_->get_clock(),
          node_logging_->get_logger(), marker_publisher);
    }
    init();
    /// Initialize visualization module
    if (cfg_.visualization_en) {
      visualizer_driver_ = std::make_unique<ROGMapVisualizer>();
      visualizer_driver_->configure(node_base_, node_parameters_, node_topics_,
                                    node_timers_, node_clock_, cfg_,
                                    std::bind(&ROGMapROS::vizCallback, this));
      const auto &pubs = visualizer_driver_->publishers();
      vm_.occ_pub = pubs.occ_pub;
      vm_.raw_occ_pub = pubs.raw_occ_pub;
      vm_.unknown_pub = pubs.unknown_pub;
      vm_.esdf_neg_pub = pubs.esdf_neg_pub;
      vm_.esdf_occ_pub = pubs.esdf_occ_pub;
      vm_.occ_inf_pub = pubs.occ_inf_pub;
      vm_.unknown_inf_pub = pubs.unknown_inf_pub;
      vm_.frontier_pub = pubs.frontier_pub;
      vm_.esdf_pub = pubs.esdf_pub;
      vm_.layer_height_delta_pub = pubs.layer_height_delta_pub;
      vm_.layer_headroom_pub = pubs.layer_headroom_pub;
      vm_.field_pub = pubs.field_pub;
      vm_.decay_cells_pub = pubs.decay_cells_pub;
      vm_.layer_value_pub = pubs.layer_value_pub;
      vm_.layer_value_dynamic_pub = pubs.layer_value_dynamic_pub;
      vm_.layer_value_static_pub = pubs.layer_value_static_pub;
      vm_.layer_type_pub = pubs.layer_type_pub;
      vm_.layer_confidence_pub = pubs.layer_confidence_pub;
      vm_.layer_headroom_ratio_pub = pubs.layer_headroom_ratio_pub;
      vm_.layer_clearance_status_pub = pubs.layer_clearance_status_pub;
      vm_.mkr_arr_pub = pubs.mkr_arr_pub;
    }

    if (cfg_.ros_callback_en) {
      rc_.cloud_pose_matcher =
          std::make_unique<CloudPoseMatcher>(CloudPoseMatcher::Config{
              cfg_.cloud_odom_sync_tolerance,
              std::min(cfg_.cloud_timeout, cfg_.odom_timeout),
              cfg_.odom_timeout, 32U, 4U});
      rc_.odom_me_cbk_group =
          createCallbackGroup(rclcpp::CallbackGroupType::MutuallyExclusive);
      rc_.cloud_me_cbk_group =
          createCallbackGroup(rclcpp::CallbackGroupType::MutuallyExclusive);
      rclcpp::SubscriptionOptions so;
      so.callback_group = rc_.odom_me_cbk_group;
      rc_.odom_sub = createSubscription<nav_msgs::msg::Odometry>(
          cfg_.odom_topic, qos,
          std::bind(&ROGMapROS::odomCallback, this, std::placeholders::_1), so);
      so.callback_group = rc_.cloud_me_cbk_group;
      so.use_intra_process_comm = rclcpp::IntraProcessSetting::Enable;
      rc_.cloud_sub = createSubscription<sensor_msgs::msg::PointCloud2>(
          cfg_.cloud_topic, rclcpp::SensorDataQoS().keep_last(1),
          [this](sensor_msgs::msg::PointCloud2::UniquePtr msg) {
            this->cloudCallback(std::move(msg));
          },
          so);
      rc_.update_cbk_group =
          createCallbackGroup(rclcpp::CallbackGroupType::MutuallyExclusive);
      rc_.update_timer = createTimer(
          std::chrono::milliseconds(cfg_.update_period_ms),
          std::bind(&ROGMapROS::updateCallback, this), rc_.update_cbk_group);
    }
  }

public:
  typedef shared_ptr<ROGMapROS> Ptr;

  ROGMapROS(const rclcpp_lifecycle::LifecycleNode::SharedPtr nh,
            const rog_map::Config &cfg,
            const std::shared_ptr<tf2_ros::Buffer> &tf = nullptr)
      : lifecycle_nh_(nh), tf_(tf) {
    bindNode(lifecycle_nh_);
    cfg_ = cfg;
    initializeRos();
  }

  ROGMapROS(const rclcpp::Node::SharedPtr nh, const rog_map::Config &cfg,
            const std::shared_ptr<tf2_ros::Buffer> &tf = nullptr)
      : nh_(nh), tf_(tf) {
    bindNode(nh_);
    cfg_ = cfg;
    initializeRos();
  }

  ~ROGMapROS() override { shutdownRosCallbacks(); }

  void shutdownRosCallbacks() {
    if (callbacks_stopping_.exchange(true, std::memory_order_acq_rel)) {
      return;
    }
    if (rc_.update_timer) {
      rc_.update_timer->cancel();
    }
    if (visualizer_driver_) {
      visualizer_driver_->reset();
    }
    rc_.odom_sub.reset();
    rc_.cloud_sub.reset();

    // MultiThreadedExecutor may already be running one callback while the
    // planner lifecycle cleanup removes the plugin. Wait for all ROG callbacks
    // before releasing the field and projection storage they access.
    {
      std::scoped_lock execution_locks(
          odom_callback_execution_mutex_, cloud_callback_execution_mutex_,
          update_callback_execution_mutex_, viz_callback_execution_mutex_);
    }
    {
      std::lock_guard<mutex> lock(rc_.updete_lock);
      rc_.cloud_pose_matcher.reset();
      rc_.pending_frames.takeLatest();
    }
    rc_.update_timer.reset();
    rc_.odom_me_cbk_group.reset();
    rc_.cloud_me_cbk_group.reset();
    rc_.update_cbk_group.reset();
    visualizer_driver_.reset();
    vm_ = ROGMapVisualizer::Publishers{};
    cloud_filter_.reset();
    cloud_filter_tf_listener_.reset();
    cloud_filter_tf_.reset();
  }

  std::shared_ptr<rog_map::MapQueryInterface> queryInterface() const {
    return getQueryInterface();
  }

private:
  static void visualizeBoundingBox(visualization_msgs::msg::MarkerArray &mkrarr,
                                   const double &stamp, const Vec3f &box_min,
                                   const Vec3f &box_max, const string &ns,
                                   const Color &color,
                                   const double &size_x = 0.1,
                                   const double &alpha = 1.0,
                                   const bool &print_ns = true) {
    Vec3f size = (box_max - box_min) / 2;
    Vec3f vis_pos_world = (box_min + box_max) / 2;
    double width = size.x();
    double length = size.y();
    double hight = size.z();

    // Publish Bounding box
    int id = 0;
    visualization_msgs::msg::Marker line_strip;
    line_strip.header.stamp = rclcpp::Time(stamp);
    line_strip.header.frame_id = "world";
    line_strip.action = visualization_msgs::msg::Marker::ADD;
    line_strip.ns = ns;
    line_strip.pose.orientation.w = 1.0;
    line_strip.id = id++; // unique id, useful when multiple markers exist.
    line_strip.type =
        visualization_msgs::msg::Marker::LINE_STRIP; // marker type
    line_strip.scale.x = size_x;

    line_strip.color = color;
    line_strip.color.a = alpha; // 不透明度，设0则全透明
    geometry_msgs::msg::Point p[8];

    // vis_pos_world是目标物的坐标
    p[0].x = vis_pos_world(0) - width;
    p[0].y = vis_pos_world(1) + length;
    p[0].z = vis_pos_world(2) + hight;
    p[1].x = vis_pos_world(0) - width;
    p[1].y = vis_pos_world(1) - length;
    p[1].z = vis_pos_world(2) + hight;
    p[2].x = vis_pos_world(0) - width;
    p[2].y = vis_pos_world(1) - length;
    p[2].z = vis_pos_world(2) - hight;
    p[3].x = vis_pos_world(0) - width;
    p[3].y = vis_pos_world(1) + length;
    p[3].z = vis_pos_world(2) - hight;
    p[4].x = vis_pos_world(0) + width;
    p[4].y = vis_pos_world(1) + length;
    p[4].z = vis_pos_world(2) - hight;
    p[5].x = vis_pos_world(0) + width;
    p[5].y = vis_pos_world(1) - length;
    p[5].z = vis_pos_world(2) - hight;
    p[6].x = vis_pos_world(0) + width;
    p[6].y = vis_pos_world(1) - length;
    p[6].z = vis_pos_world(2) + hight;
    p[7].x = vis_pos_world(0) + width;
    p[7].y = vis_pos_world(1) + length;
    p[7].z = vis_pos_world(2) + hight;
    // LINE_STRIP类型仅仅将line_strip.points中相邻的两个点相连，如0和1，1和2，2和3
    for (int i = 0; i < 8; i++) {
      line_strip.points.push_back(p[i]);
    }
    // 为了保证矩形框的八条边都存在：
    line_strip.points.push_back(p[0]);
    line_strip.points.push_back(p[3]);
    line_strip.points.push_back(p[2]);
    line_strip.points.push_back(p[5]);
    line_strip.points.push_back(p[6]);
    line_strip.points.push_back(p[1]);
    line_strip.points.push_back(p[0]);
    line_strip.points.push_back(p[7]);
    line_strip.points.push_back(p[4]);
    mkrarr.markers.push_back(line_strip);
  }

  static void visualizeText(visualization_msgs::msg::MarkerArray &mkr_arr,
                            const double &stamp, const std::string &ns,
                            const std::string &text, const Vec3f &position,
                            const Color &c = Color::White(),
                            const double &size = 0.6, const int &id = -1) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = rclcpp::Time(stamp);
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.ns = ns.c_str();
    if (id >= 0) {
      marker.id = id;
    } else {
      static int id = 0;
      marker.id = id++;
    }
    marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker.scale.z = size;
    marker.color = c;
    marker.text = text;
    marker.pose.position.x = position.x();
    marker.pose.position.y = position.y();
    marker.pose.position.z = position.z();
    marker.pose.orientation.w = 1.0;
    mkr_arr.markers.push_back(marker);
  };

  static void visualizePoint(visualization_msgs::msg::MarkerArray &mkr_arr,
                             const double &stamp, const Vec3f &pt,
                             Color color = Color::Pink(), std::string ns = "pt",
                             double size = 0.1, int id = -1,
                             const bool &print_ns = true) {
    visualization_msgs::msg::Marker marker_ball;
    static int cnt = 0;
    Vec3f cur_pos = pt;
    if (isnan(pt.x()) || isnan(pt.y()) || isnan(pt.z())) {
      return;
    }
    marker_ball.header.frame_id = "world";
    marker_ball.header.stamp = rclcpp::Time(stamp);
    marker_ball.ns = ns.c_str();
    marker_ball.id = id >= 0 ? id : cnt++;
    marker_ball.action = visualization_msgs::msg::Marker::ADD;
    marker_ball.pose.orientation.w = 1.0;
    marker_ball.type = visualization_msgs::msg::Marker::SPHERE;
    marker_ball.scale.x = size;
    marker_ball.scale.y = size;
    marker_ball.scale.z = size;
    marker_ball.color = color;

    geometry_msgs::msg::Point p;
    p.x = cur_pos.x();
    p.y = cur_pos.y();
    p.z = cur_pos.z();

    marker_ball.pose.position = p;
    mkr_arr.markers.push_back(marker_ball);

    // add test
    if (print_ns) {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "world";
      marker.header.stamp = rclcpp::Time(stamp);
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.orientation.w = 1.0;
      marker.ns = ns + "_text";
      if (id >= 0) {
        marker.id = id;
      } else {
        static int id = 0;
        marker.id = id++;
      }
      marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      marker.scale.z = 0.6;
      marker.color = color;
      marker.text = ns;
      marker.pose.position.x = cur_pos.x();
      marker.pose.position.y = cur_pos.y();
      marker.pose.position.z = cur_pos.z() + 0.5;
      marker.pose.orientation.w = 1.0;
      mkr_arr.markers.push_back(marker);
    }
  }
};
} // namespace rog_map
#endif // ROG_MAP_ROS_HPP
