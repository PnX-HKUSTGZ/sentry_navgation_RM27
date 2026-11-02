// Copyright 2026 RM27 contributors. SPDX-License-Identifier: Apache-2.0
// Independent sensor-only mapping backend; links, never rebuilds, the frozen
// relocalization frontend. No prior map, initial pose, GT or TF subscription.
#include "mapping_output.hpp"
#include "mapping_scan_worker.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/string.hpp>
#include <nlohmann/json.hpp>
#include <atomic>

namespace raw_gicp_mapping {
using namespace global_relocalization;
class Node : public rclcpp::Node {
  using Message=sensor_msgs::msg::PointCloud2::ConstSharedPtr;
  using Worker=MappingScanWorker<Message,RawFrontendResult>;
public:
  Node():rclcpp::Node("raw_mapping") {
    RawFrontendParams p;
    p.range_filter=true;p.deterministic=true;p.imu_rotation_prior=true;
    p.numerical_recovery=false;
#define READ(name) p.name=declare_parameter("frontend." #name,p.name)
    READ(stationary_duration);READ(max_base_linear_speed);READ(max_base_translation_step);
    READ(max_imu_gap);READ(max_scan_gap);READ(max_fitness);READ(min_inlier_ratio);
    READ(max_iterations);READ(retry_iterations);READ(leaf_size);READ(max_correspondence);
    READ(diagnostic_directory);
#undef READ
    p.submap_scans=declare_parameter<int>("frontend.submap_scans",20);
    const auto rigid=[&](const std::string & name) {
      std::vector<double> identity(16,0.);for(size_t k=0;k<4;++k)identity[k*4+k]=1.;
      const auto values=declare_parameter<std::vector<double>>(name,identity);
      if(values.size()!=16)throw std::invalid_argument(name+" must be a row-major 4x4 rigid transform");
      Eigen::Isometry3d pose;for(size_t r=0;r<4;++r)for(size_t c=0;c<4;++c)pose.matrix()(r,c)=values[r*4+c];
      return pose;
    };
    instantaneous_=declare_parameter<bool>("preprocess.instantaneous_scan",declare_parameter<bool>("instantaneous_scan",true));
    if(!instantaneous_)throw std::invalid_argument("this backend supports instantaneous scans only");
    if(declare_parameter<bool>("prior_pcd.enable",false))throw std::invalid_argument("raw mapping cannot load a prior PCD");
    if(!declare_parameter<bool>("mapping.imu_en",true))throw std::invalid_argument("raw mapping requires IMU input");
    shutdown_on_fault_=declare_parameter<bool>("shutdown_on_fault",true);
    frontend_=std::make_unique<StationaryRawFrontend>(p,rigid("sensor_from_imu"),rigid("base_from_sensor"));
    odom_=create_publisher<nav_msgs::msg::Odometry>("/aft_mapped_to_init",20);
    cloud_=create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered",rclcpp::QoS(10));
    status_=create_publisher<std_msgs::msg::String>("/raw_mapping/status",rclcpp::QoS(10).transient_local());
    const double timeout=declare_parameter<double>("source_timeout_sec",5.);
    worker_=std::make_unique<Worker>([this](const Worker::Item & item){
      Cloud points;pcl::fromROSMsg(*item.payload,points);
      return frontend_->processScan(points,item.stamp,instantaneous_);
    },[this]{return now().seconds();},timeout,p.max_scan_gap);
    worker_->begin();
    imu_group_=create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    scan_group_=create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    result_group_=create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions imu_options;imu_options.callback_group=imu_group_;
    rclcpp::SubscriptionOptions scan_options;scan_options.callback_group=scan_group_;
    auto imu_qos=rclcpp::SensorDataQoS();imu_qos.keep_last(2000);
    const auto imu_topic=declare_parameter<std::string>("common.imu_topic",declare_parameter<std::string>("imu_topic","/livox/imu"));
    const auto cloud_topic=declare_parameter<std::string>("common.lid_topic",declare_parameter<std::string>("cloud_topic","/livox/lidar"));
    imu_=create_subscription<sensor_msgs::msg::Imu>(imu_topic,imu_qos,
      [this](sensor_msgs::msg::Imu::ConstSharedPtr msg){
        if(faulted_)return;
        ++imu_received_;
        RawImuSample sample;sample.stamp=rclcpp::Time(msg->header.stamp).seconds();
        sample.angular_velocity<<msg->angular_velocity.x,msg->angular_velocity.y,msg->angular_velocity.z;
        sample.linear_acceleration<<msg->linear_acceleration.x,msg->linear_acceleration.y,msg->linear_acceleration.z;
        const double previous=last_imu_stamp_.exchange(sample.stamp);
        if(previous>=0.)max_imu_gap_.store(std::max(max_imu_gap_.load(),sample.stamp-previous));
        std::string reason;if(!frontend_->addImu(sample,&reason)){fault(reason);return;}
        worker_->watermark(sample.stamp);
      },imu_options);
    auto scan_qos=rclcpp::SensorDataQoS();scan_qos.keep_last(100);
    scan_=create_subscription<sensor_msgs::msg::PointCloud2>(cloud_topic,scan_qos,
      [this](Message msg){
        if(faulted_)return;
        const double stamp=rclcpp::Time(msg->header.stamp).seconds();
        if(!worker_->enqueue(msg,msg->data.size(),stamp,now().seconds()))fault(worker_->stats().error);
      },scan_options);
    timer_=create_wall_timer(std::chrono::milliseconds(5),[this]{drain();},result_group_);
    RCLCPP_INFO(get_logger(),"Sensor-only GICP mapping ready; camera_init/body output; no GT/prior/TF input");
  }
  ~Node() override {if(worker_)worker_->close();}
  bool faulted() const {return faulted_.load();}
private:
  void fault(const std::string & reason) {
    if(faulted_.exchange(true))return;
    const auto queue=worker_->stats();worker_->cancel();
    nlohmann::json data={{"fault",reason},{"ground_truth_used",false},{"accepted_scans",accepted_.load()},
      {"last_imu_stamp",last_imu_stamp_.load()},{"max_received_imu_gap",max_imu_gap_.load()},
      {"received_scan_count",queue.received_count},{"received_imu_count",imu_received_.load()},
      {"max_received_scan_gap",queue.max_stamp_gap},{"last_received_scan_stamp",queue.latest_stamp}};
    std_msgs::msg::String message;message.data=data.dump();status_->publish(message);
    RCLCPP_ERROR(get_logger(),"Raw mapping stopped: %s",reason.c_str());
    if(shutdown_on_fault_)rclcpp::shutdown();
  }
  void drain() {
    if(faulted_)return;
    const auto queue=worker_->stats();if(!queue.error.empty()){fault(queue.error);return;}
    while(auto done=worker_->takeResult()) {
      if(faulted_)return;
      const auto & result=done->result;
      nlohmann::json data={{"stamp",done->item.stamp},{"accepted",result.accepted},{"reason",result.reason},
        {"failure_class",result.failure_class},{"fitness",result.fitness},{"inlier_ratio",result.inlier_ratio},
        {"translation_step",result.translation_step},{"linear_speed",result.linear_speed},{"angular_speed",result.angular_speed},
        {"rotation_prior_error",result.rotation_prior_error},{"information_eigen_ratio",result.information_eigen_ratio},
        {"processing_sec",result.processing_sec},{"over_budget",result.over_budget},{"queue_wait_sec",done->queue_wait_sec},{"source_age_sec",done->source_age_sec},
        {"last_imu_stamp",last_imu_stamp_.load()},{"max_received_imu_gap",max_imu_gap_.load()},
        {"received_scan_count",queue.received_count},{"received_imu_count",imu_received_.load()},
        {"max_received_scan_gap",queue.max_stamp_gap},{"queue_high_watermark",queue.high_watermark},{"ground_truth_used",false},{"covariance_available",false}};
      std_msgs::msg::String status;status.data=data.dump();status_->publish(status);
      if(!result.accepted) {
        if(result.failure_class=="waiting")continue;
        fault(result.reason);return;
      }
      const builtin_interfaces::msg::Time stamp=rclcpp::Time(static_cast<int64_t>(std::llround(done->item.stamp*1e9)));
      auto output=mappingOutput(result,stamp);
      odom_->publish(output->odometry);cloud_->publish(output->registered_cloud);++accepted_;
    }
  }
  std::unique_ptr<StationaryRawFrontend> frontend_;std::unique_ptr<Worker> worker_;
  std::atomic_bool faulted_{false};std::atomic<size_t> accepted_{0},imu_received_{0};bool instantaneous_{true},shutdown_on_fault_{true};
  std::atomic<double> last_imu_stamp_{-1.},max_imu_gap_{0.};
  rclcpp::CallbackGroup::SharedPtr imu_group_,scan_group_,result_group_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_;rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr scan_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_;rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;rclcpp::TimerBase::SharedPtr timer_;
};
}
int main(int argc,char ** argv) {
  rclcpp::init(argc,argv);
  try {
    auto node=std::make_shared<raw_gicp_mapping::Node>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),3);executor.add_node(node);executor.spin();
    if(node->faulted())return 1;
  } catch(const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("raw_mapping"),"%s",error.what());
    if(rclcpp::ok())rclcpp::shutdown();return 2;
  }
  if(rclcpp::ok())rclcpp::shutdown();return 0;
}
