// Copyright 2026 RM27 contributors. SPDX-License-Identifier: Apache-2.0
#include "global_relocalization/core.hpp"
#include "global_relocalization/stationary_raw_frontend.hpp"
#include "global_relocalization/raw_scan_worker.hpp"
#include <global_relocalization_interfaces/action/relocalize.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/string.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <nlohmann/json.hpp>
#include <atomic>
#include <chrono>
#include <cmath>
#include <future>
#include <fstream>
#include <mutex>
#include <deque>

namespace global_relocalization {
class Node : public rclcpp::Node {
  using Action=global_relocalization_interfaces::action::Relocalize;
  using Handle=rclcpp_action::ServerGoalHandle<Action>;
  using Clock=std::chrono::steady_clock;
public:
  Node():rclcpp::Node("global_relocalization") {
    auto str=[this](const std::string & n,const std::string & v){return declare_parameter<std::string>(n,v);};
    auto number=[this](const std::string & n,double v){return declare_parameter<double>(n,v);};
    input_mode_=str("input_mode","registered");
    if(input_mode_!="registered"&&input_mode_!="raw_imu")throw std::invalid_argument("input_mode must be registered or raw_imu");
    raw_scan_model_=str("raw_scan_model","instantaneous");
    if(raw_scan_model_!="instantaneous")throw std::invalid_argument("timed raw scans require explicit measured per-point timing; unsupported in this profile");
    raw_params_.range_filter=declare_parameter<bool>("raw_range_filter",false);
    raw_params_.deterministic=declare_parameter<bool>("raw_deterministic",false);
    raw_params_.submap_scans=declare_parameter<int>("raw_submap_scans",1);
    raw_params_.imu_rotation_prior=declare_parameter<bool>("raw_imu_rotation_prior",false);
    raw_params_.numerical_recovery=declare_parameter<bool>("raw_numerical_recovery",false);
    raw_params_.min_range=number("raw_min_range",0.5);raw_params_.max_range=number("raw_max_range",30.0);
    raw_params_.gyro_noise_density=number("raw_gyro_noise_density",0.005);raw_params_.rotation_sigma_floor=number("raw_rotation_sigma_floor",0.005);
    raw_diagnostic_directory_=str("raw_diagnostic_directory","");
    raw_calibration_sec_=number("raw_calibration_sec",2.0);
    raw_params_.leaf_size=number("raw_local_leaf_size",.12);
    raw_params_.max_correspondence=number("raw_local_max_correspondence",.5);
    raw_params_.max_iterations=declare_parameter<int>("raw_local_max_iterations",15);
    raw_params_.retry_iterations=declare_parameter<int>("raw_local_retry_iterations",15);
    raw_params_.max_fitness=number("raw_local_max_fitness",.12);
    raw_params_.min_inlier_ratio=number("raw_local_min_inlier_ratio",.65);
    raw_params_.diagnostic_budget_sec=number("raw_local_budget_ms",100.)/1000.;
    raw_params_.stationary_duration=raw_calibration_sec_;
    raw_confirmation_frames_=declare_parameter<int>("raw_confirmation_frames",3);
    if(raw_params_.submap_scans<1||raw_params_.submap_scans>5||raw_params_.min_range<0||raw_params_.max_range<=raw_params_.min_range||raw_params_.gyro_noise_density<=0||raw_params_.rotation_sigma_floor<=0||raw_params_.retry_iterations<0||raw_params_.retry_iterations>15||raw_confirmation_frames_<3||raw_confirmation_frames_>10||raw_calibration_sec_<=0||raw_params_.leaf_size<=0||raw_params_.max_correspondence<=0||raw_params_.max_iterations<1||raw_params_.max_fitness<=0||raw_params_.min_inlier_ratio<=0||raw_params_.min_inlier_ratio>1||raw_params_.diagnostic_budget_sec<=0)
      throw std::invalid_argument("invalid raw calibration/registration/confirmation parameters");
    map_frame_=str("map_frame","map");odom_frame_=str("odom_frame","odom");
    base_frame_=str("base_frame","base_footprint");lidar_frame_=str("lidar_frame","left_mid360");
    settle_sec_=number("settle_sec",2.0);spin_speed_=number("spin_speed",0.5);
    spin_angle_=number("spin_angle",6.283185307179586);timeout_sec_=number("timeout_sec",60.0);
    data_timeout_=number("data_timeout_sec",0.7);
    accumulation_leaf_=number("accumulation_leaf_size",0.05);
    max_translation_=number("max_spin_translation",0.30);
    Params p;
    p.deterministic_preprocessing=raw_params_.deterministic;
    p.diagnostic_output_dir=str("diagnostic_output_dir","");
    p.search_side=str("search_side","all");p.side_boundary_x=number("side_boundary_x",0.0);search_prior_=p;
    p.sc_rings=declare_parameter<int>("sc_rings",20);p.sc_sectors=declare_parameter<int>("sc_sectors",60);
    p.top_k=declare_parameter<int>("top_k",5);
    p.ring_key_candidates=declare_parameter<int>("ring_key_candidates",input_mode_=="raw_imu"?5:0);
    p.retrieval_all_views=declare_parameter<bool>("retrieval_all_views",input_mode_=="raw_imu");
    p.db_grid_resolution=number("db_grid_resolution",2.0);p.db_base_z=number("db_base_z",0.0);
    p.db_max_radius=number("db_max_radius",25.0);p.db_min_points=declare_parameter<int>("db_min_points",80);
    p.db_min_elevation_deg=number("db_min_elevation_deg",-40.0);
    p.db_max_elevation_deg=number("db_max_elevation_deg",85.0);
    p.gicp_leaf_size=number("gicp_leaf_size",0.15);p.gicp_max_correspondence=number("gicp_max_correspondence",1.0);
    p.gicp_max_iterations=declare_parameter<int>("gicp_max_iterations",40);
    p.candidate_workers=declare_parameter<int>("candidate_workers",1);
    p.max_fitness=number("max_fitness",0.20);p.min_inlier_ratio=number("min_inlier_ratio",0.65);
    p.ambiguity_margin=number("ambiguity_margin",0.15);
    p.confirmation_max_translation=number("confirmation_max_translation",0.15);
    p.confirmation_max_rotation=number("confirmation_max_rotation",0.0872664626);
    if(settle_sec_<=0||std::abs(spin_speed_)<0.01||std::abs(spin_speed_)>1.5||spin_angle_<=0||timeout_sec_<=0||data_timeout_<=0)
      throw std::invalid_argument("Invalid experiment motion/timing parameters");
    const auto map_path=str("map_path","");const auto manifest_path=str("manifest_path","");
    core_=std::make_unique<Core>(p);core_->loadMap(map_path);
    if(!manifest_path.empty()) {
      std::ifstream f(manifest_path);nlohmann::json manifest;f>>manifest;
      if(manifest.contains("frame_id")&&manifest["frame_id"]!=map_frame_)
        throw std::runtime_error("Map manifest frame_id disagrees with configured map_frame");
      if(manifest.contains("map_frame")&&manifest["map_frame"]!=map_frame_)
        throw std::runtime_error("Map manifest frame disagrees with configured map_frame");
      if(manifest.contains("map_sha256")&&manifest["map_sha256"]!=core_->mapHash())
        throw std::runtime_error("Map manifest SHA256 disagrees with PCD");
    }
    tf_buffer_=std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_=std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    cmd_pub_=create_publisher<geometry_msgs::msg::Twist>(str("cmd_vel_topic","/cmd_vel"),10);
    pose_pub_=create_publisher<geometry_msgs::msg::PoseStamped>("/global_relocalization/pose",10);
    status_pub_=create_publisher<std_msgs::msg::String>("/global_relocalization/status",10);
    const auto registered_topic=str("cloud_topic","/registered_scan"), odometry_topic=str("odom_topic","/odometry");
    const auto raw_topic=str("raw_cloud_topic","/livox/lidar"), imu_topic=str("imu_topic","/livox/imu");
    if(input_mode_=="registered") {
      cloud_sub_=create_subscription<sensor_msgs::msg::PointCloud2>(registered_topic,rclcpp::SensorDataQoS(),[this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg){cloud(msg);});
      odom_sub_=create_subscription<nav_msgs::msg::Odometry>(odometry_topic,rclcpp::SensorDataQoS(),[this](nav_msgs::msg::Odometry::ConstSharedPtr msg){odom(msg);});
    }else {
      cloud_group_=create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
      rclcpp::SubscriptionOptions cloud_options;cloud_options.callback_group=cloud_group_;
      raw_pipeline_=std::make_unique<RawPipeline>([this](const RawPipeline::Item & item){return processRawItem(item);},[this]{return now().seconds();},data_timeout_);
      cloud_sub_=create_subscription<sensor_msgs::msg::PointCloud2>(raw_topic,rclcpp::QoS(rclcpp::KeepLast(20)).reliable(),[this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg){rawCloud(msg);},cloud_options);
      imu_group_=create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
      rclcpp::SubscriptionOptions imu_options;imu_options.callback_group=imu_group_;
      imu_sub_=create_subscription<sensor_msgs::msg::Imu>(imu_topic,rclcpp::QoS(rclcpp::KeepLast(200)).reliable(),[this](sensor_msgs::msg::Imu::ConstSharedPtr msg){rawImu(msg);},imu_options);
    }
    server_=rclcpp_action::create_server<Action>(this,"/global_relocalization/run",
      [this](const rclcpp_action::GoalUUID &,std::shared_ptr<const Action::Goal>){
        std::lock_guard<std::mutex> lock(mutex_);
        return (handle_||shutting_down_.load())?rclcpp_action::GoalResponse::REJECT:rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [this](std::shared_ptr<Handle>){cancel_.store(true);return rclcpp_action::CancelResponse::ACCEPT;},
      [this](std::shared_ptr<Handle> h){
        std::lock_guard<std::mutex> lock(mutex_);handle_=h;cancel_.store(false);start_=Clock::now();
        stage_start_=start_;stable_start_=start_;sim_start_=now();accumulated_.clear();yaw_=0;have_yaw_=false;
        have_reference_=false;failure_.clear();input_error_.clear();run_id_=std::to_string(++run_count_);
        if(input_mode_=="raw_imu"){
          raw_capture_mode_.store(0);raw_pipeline_->cancel();raw_pipeline_->waitIdle();
          {std::lock_guard<std::mutex> imu_lock(raw_imu_mutex_);raw_ingress_=RawImuIngress{};}
          raw_ingress_fault_.store(false);raw_capture_=std::make_shared<RawCapture>();raw_capture_mode_.store(1);
          raw_generation_=raw_pipeline_->begin();
          {std::lock_guard<std::mutex> imu_lock(raw_imu_mutex_);raw_ingress_.active=true;}
          raw_frontend_.reset();raw_recovery_stage_.clear();raw_recovery_stop_tick_=false;confirmations_.clear();have_cloud_=have_odom_=false;raw_status_=rawInitialStatus();latest_imu_stamp_=-1;raw_yaw_at_spin_=raw_latest_yaw_=0;raw_bias_.setZero();
        }
        result_=SearchResult{};stage_ros_times_=nlohmann::json::object();stage_times_=nlohmann::json::object();stage_="IDLE";changeStage("SETTLING");
      });
    timer_=create_wall_timer(std::chrono::milliseconds(50),[this]{tick();});
    shutdown_callback_=get_node_base_interface()->get_context()->add_pre_shutdown_callback([this] {
      // A stop published after rmw_shutdown is too late to reach the bridge.
      // Serialize with tick while DDS is still alive, then wait briefly for
      // reliable subscribers to acknowledge the zero command.
      shutting_down_.store(true);cancel_.store(true);if(raw_pipeline_)raw_pipeline_->cancel();
      std::lock_guard<std::mutex> lock(mutex_);
      try {
        cmd_pub_->publish(geometry_msgs::msg::Twist{});
        cmd_pub_->wait_for_all_acked(std::chrono::milliseconds(100));
      }catch(const std::exception & e){std::cerr<<"global_relocalization pre-shutdown stop: "<<e.what()<<std::endl;}
    });
    RCLCPP_INFO(get_logger(),"Independent experiment ready: map SHA256=%s; no TF/initialpose publication",core_->mapHash().c_str());
  }
  ~Node()override {
    get_node_base_interface()->get_context()->remove_pre_shutdown_callback(shutdown_callback_);
    cancel_.store(true);
    try{cmd_pub_->publish(geometry_msgs::msg::Twist{});}
    catch(const std::exception & e){std::cerr<<"global_relocalization shutdown stop publication: "<<e.what()<<std::endl;}
    if(worker_.valid())worker_.wait();
    if(raw_pipeline_)raw_pipeline_->close();
    // ServerGoalHandle's destructor auto-cancels active goals and invokes its
    // weak Server terminal callback. Release Server first so shutdown cannot
    // dispatch a result into an action server that has already removed goals.
    server_.reset();handle_.reset();
  }
private:
  double elapsed()const{return std::chrono::duration<double>(Clock::now()-start_).count();}
  double stageElapsed()const{return std::chrono::duration<double>(Clock::now()-stage_start_).count();}
  static double yaw(const Eigen::Isometry3d & t){return std::atan2(t.linear()(1,0),t.linear()(0,0));}
  static double wrap(double a){return std::atan2(std::sin(a),std::cos(a));}
  Eigen::Isometry3d transform(const std::string & target,const std::string & source,const builtin_interfaces::msg::Time & stamp) {
    return tf2::transformToEigen(tf_buffer_->lookupTransform(target,source,rclcpp::Time(stamp),rclcpp::Duration::from_seconds(0.0)).transform);
  }
  void cloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    if(msg->header.frame_id!=odom_frame_){input_error_="registered scan frame must equal odom_frame";return;}
    pending_clouds_.push_back({msg,Clock::now()});
    // Scan and TF travel through independent subscriptions. Keep their original
    // stamps and wait for exact-time TF rather than dropping every arriving scan
    // or applying a transform from a different time.
    while(pending_clouds_.size()>8)pending_clouds_.pop_front();
  }
  void processClouds() {
    while(!pending_clouds_.empty()) {
      const auto pending=pending_clouds_.front();
      if(std::chrono::duration<double>(Clock::now()-pending.received).count()>data_timeout_) {
        pending_clouds_.pop_front();input_error_="registered scan expired while waiting for its timestamped TF";continue;
      }
      try {
        transform(odom_frame_,base_frame_,pending.message->header.stamp);
      }catch(const std::exception & e){input_error_=e.what();break;}
      pending_clouds_.pop_front();consumeCloud(pending.message,pending.received);
    }
  }
  void consumeCloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg,Clock::time_point received) {
    try {
      const auto current=transform(odom_frame_,base_frame_,msg->header.stamp);
      Cloud points;pcl::fromROSMsg(*msg,points);Cloud finite;
      for(const auto & p:points)if(std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.z))finite.push_back(p);
      if(finite.size()<30){input_error_="scan has fewer than 30 finite points";return;}
      latest_cloud_=std::move(finite);cloud_stamp_=msg->header.stamp;latest_base_=current;
      cloud_received_=received;have_cloud_=true;input_error_.clear();
      if(handle_&&have_reference_&&(stage_=="SPINNING"||stage_=="SETTLING")) {
        Cloud local;pcl::transformPointCloud(latest_cloud_,local,odom_reference_.inverse().matrix().cast<float>());
        accumulated_+=local;
        // Bound accumulation memory and density without changing its reference frame.
        if(accumulated_.size()>250000) {
          pcl::VoxelGrid<pcl::PointXYZ> voxel;auto input=std::make_shared<Cloud>(accumulated_);
          voxel.setInputCloud(input);voxel.setLeafSize(accumulation_leaf_,accumulation_leaf_,accumulation_leaf_);voxel.filter(accumulated_);
        }
      }
    }catch(const std::exception & e){input_error_=e.what();}
  }
  struct RawJob {sensor_msgs::msg::PointCloud2::ConstSharedPtr message;};
  struct RawProcessed {RawFrontendResult frontend;Cloud local_cloud;builtin_interfaces::msg::Time stamp;};
  using RawPipeline=RawScanWorker<RawJob,RawProcessed>;
  struct RawCapture {Cloud accumulated;Eigen::Isometry3d origin{Eigen::Isometry3d::Identity()};bool have_origin{false};};
  struct RawImuIngress {
    bool active{false};std::shared_ptr<StationaryRawFrontend> frontend;
    std::string frame,error,tf_error;
    Eigen::Isometry3d base_sensor{Eigen::Isometry3d::Identity()};
    Eigen::Vector3d gyro{Eigen::Vector3d::Zero()},bias{Eigen::Vector3d::Zero()};
    builtin_interfaces::msg::Time stamp{};
    Clock::time_point received;
    double stamp_sec{-1}, max_stamp_gap{0},max_callback_gap{0},latest_ros_delay{0};
    uint64_t received_count{0},accepted_count{0},rejected_count{0};
  };
  static nlohmann::json rawInitialStatus(){return {{"gravity_initialized",false},{"accepted",false},{"queue_dropped",0},{"lidar_received",0},{"lidar_used",0},{"lidar_rejected",0},{"lidar_max_received_stamp_gap_sec",0.0},{"lidar_max_callback_wall_gap_sec",0.0}};}
  void rawCloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
    // Independent ingress group: no action-state or registration mutex.
    if(cancel_.load()||shutting_down_.load())return;
    if(msg->header.frame_id!=lidar_frame_) {
      if(raw_pipeline_->stats().generation==raw_generation_.load()){raw_ingress_fault_.store(true);cmd_pub_->publish(geometry_msgs::msg::Twist{});}
      return;
    }
    const auto received=Clock::now();const auto receipt_ros=now().seconds();
    if(!raw_pipeline_->enqueue({msg},msg->data.size(),rclcpp::Time(msg->header.stamp).seconds(),receipt_ros,received))
      cmd_pub_->publish(geometry_msgs::msg::Twist{});
  }
  void rawImu(sensor_msgs::msg::Imu::ConstSharedPtr msg) {
    // This callback never takes mutex_: cloud processing and action state cannot
    // block the 100 Hz buffer producer. This group serializes IMU arrival order.
    std::lock_guard<std::mutex> lock(raw_imu_mutex_);
    auto & ingress=raw_ingress_;
    if(!ingress.active||cancel_.load()||shutting_down_.load())return;
    const double stamp=rclcpp::Time(msg->header.stamp).seconds();const auto received=Clock::now();
    ++ingress.received_count;
    if(ingress.stamp_sec>=0){ingress.max_stamp_gap=std::max(ingress.max_stamp_gap,stamp-ingress.stamp_sec);ingress.max_callback_gap=std::max(ingress.max_callback_gap,std::chrono::duration<double>(received-ingress.received).count());}
    ingress.latest_ros_delay=(now()-rclcpp::Time(msg->header.stamp)).seconds();
    if(!ingress.frontend) {
      try {
        ingress.base_sensor=transform(base_frame_,lidar_frame_,builtin_interfaces::msg::Time{});
        const auto sensor_imu=transform(lidar_frame_,msg->header.frame_id,builtin_interfaces::msg::Time{});
        auto frontend_params=raw_params_;if(!raw_diagnostic_directory_.empty())frontend_params.diagnostic_directory=raw_diagnostic_directory_+"/action_"+run_id_+"_"+std::to_string(std::chrono::system_clock::now().time_since_epoch().count());
        ingress.frontend=std::make_shared<StationaryRawFrontend>(frontend_params,sensor_imu,ingress.base_sensor);
        ingress.frame=msg->header.frame_id;ingress.tf_error.clear();
      }catch(const std::exception&e){ingress.tf_error=e.what();return;}
    }
    if(msg->header.frame_id!=ingress.frame){ingress.error="IMU frame changed during action";++ingress.rejected_count;return;}
    RawImuSample sample;sample.stamp=stamp;
    sample.angular_velocity<<msg->angular_velocity.x,msg->angular_velocity.y,msg->angular_velocity.z;
    sample.linear_acceleration<<msg->linear_acceleration.x,msg->linear_acceleration.y,msg->linear_acceleration.z;
    std::string reason;
    if(!ingress.frontend->addImu(sample,&reason)){ingress.error=reason;++ingress.rejected_count;return;}
    ++ingress.accepted_count;ingress.stamp_sec=stamp;ingress.stamp=msg->header.stamp;
    raw_pipeline_->watermark(stamp);
    ingress.received=received;ingress.gyro=sample.angular_velocity;
  }
  void snapshotRawImu() {
    std::lock_guard<std::mutex> lock(raw_imu_mutex_);const auto & ingress=raw_ingress_;
    raw_status_.update({{"imu_received",ingress.received_count},{"imu_accepted",ingress.accepted_count},{"imu_rejected",ingress.rejected_count},
      {"imu_max_received_stamp_gap_sec",ingress.max_stamp_gap},{"imu_max_callback_wall_gap_sec",ingress.max_callback_gap},
      {"imu_latest_callback_ros_delay_sec",ingress.received_count?nlohmann::json(ingress.latest_ros_delay):nlohmann::json(nullptr)},
      {"imu_qos_depth",200},{"cloud_qos_depth",20},{"raw_qos_reliability","reliable"}});
    if(!ingress.error.empty()){failure_=ingress.error;cancel_.store(true);}
    if(!ingress.tf_error.empty())input_error_=ingress.tf_error;
    if(ingress.frontend){raw_frontend_=ingress.frontend;base_lidar_=ingress.base_sensor;}
    latest_imu_stamp_=ingress.stamp_sec;
    if(ingress.accepted_count){angular_speed_=(ingress.gyro-ingress.bias).norm();odom_stamp_=ingress.stamp;odom_received_=ingress.received;}
  }
  RawProcessed processRawItem(const RawPipeline::Item & item) {
    RawProcessed processed;processed.stamp=item.payload.message->header.stamp;std::shared_ptr<StationaryRawFrontend> frontend;
    {std::lock_guard<std::mutex> lock(raw_imu_mutex_);frontend=raw_ingress_.frontend;}
    if(!frontend)throw std::runtime_error("raw worker has no initialized IMU ingress");
    Cloud scan;pcl::fromROSMsg(*item.payload.message,scan);processed.frontend=frontend->processScan(scan,item.stamp,true);
    if(processed.frontend.accepted) {
      const Cloud & finite=processed.frontend.filtered_sensor_cloud;
      Cloud reference_cloud;pcl::transformPointCloud(finite,reference_cloud,processed.frontend.reference_from_sensor.matrix().cast<float>());
      auto & capture=*raw_capture_;
      if(!capture.have_origin){capture.origin.setIdentity();capture.origin.translation()=processed.frontend.reference_from_base.translation();capture.have_origin=true;}
      pcl::transformPointCloud(reference_cloud,processed.local_cloud,capture.origin.inverse().matrix().cast<float>());
      if(raw_capture_mode_.load()==1) {
        capture.accumulated+=processed.local_cloud;
        if(capture.accumulated.size()>250000){pcl::VoxelGrid<pcl::PointXYZ> voxel;voxel.setInputCloud(capture.accumulated.makeShared());voxel.setLeafSize(accumulation_leaf_,accumulation_leaf_,accumulation_leaf_);Cloud filtered;voxel.filter(filtered);capture.accumulated=std::move(filtered);}
      }
    }
    return processed;
  }
  void consumeRawResults() {
    for(size_t count=0;count<RawPipeline::capacity;++count) {
      auto completed=raw_pipeline_->takeResult();if(!completed)break;auto & done=*completed;
      if(done.item.generation!=raw_generation_.load()||!handle_||cancel_.load())continue;
      auto & processed=done.result;const auto & update=processed.frontend;
      raw_status_["max_worker_wall_sec"]=std::max(raw_status_.value("max_worker_wall_sec",0.0),done.worker_wall_sec);
      raw_status_["max_scan_queue_wait_sec"]=std::max(raw_status_.value("max_scan_queue_wait_sec",0.0),done.queue_wait_sec);
      raw_status_.update({{"worker_wall_sec",done.worker_wall_sec},{"scan_queue_wait_sec",done.queue_wait_sec},
        {"processed_source_stamp_sec",done.item.stamp},{"processed_source_age_sec",done.source_age_sec},
        {"failure_class",update.failure_class},{"recovery_pending",update.recovery_pending},{"recovery_attempts",update.recovery_attempts},{"recovered_scan_count",update.recovered_scan_count},{"input_points",update.input_points},{"finite_points",update.finite_points},{"range_points",update.range_points},{"downsampled_points",update.points},{"submap_size",update.submap_size},{"rotation_prior_sigma",update.rotation_prior_sigma},{"rotation_prior_weight",update.rotation_prior_weight},
        {"accepted",update.accepted},{"reason",update.reason},{"gravity_initialized",update.gravity_initialized},{"imu_covered",update.imu_covered},
        {"fitness",update.fitness},{"inlier_ratio",update.inlier_ratio},{"processing_sec",update.processing_sec},{"over_budget",update.over_budget},
        {"geometry_eigen_ratio",update.geometry_eigen_ratio},{"information_eigen_ratio",update.information_eigen_ratio},{"rotation_prior_error",update.rotation_prior_error},
        {"translation_step",update.translation_step},{"linear_speed",update.linear_speed},{"cumulative_yaw",update.cumulative_yaw},
        {"gyro_bias",{update.gyro_bias.x(),update.gyro_bias.y(),update.gyro_bias.z()}},
        {"converged",update.converged},{"transform_finite",update.transform_finite},{"iterations",update.iterations},
        {"first_attempt_iterations",update.first_attempt_iterations},{"retry_count",update.retry_count},{"rejected_gates",update.rejected_gates}});
      raw_status_["registration_attempts"]=nlohmann::json::array();
      for(const auto & a:update.registration_attempts)raw_status_["registration_attempts"].push_back({{"attempt",a.attempt},{"iterations",a.iterations},{"converged",a.converged},{"transform_finite",a.transform_finite},{"fitness",a.fitness},{"inlier_ratio",a.inlier_ratio},{"information_eigen_ratio",a.information_eigen_ratio},{"rotation_prior_error",a.rotation_prior_error},{"translation_step",a.translation_step},{"linear_speed",a.linear_speed},{"rejected_gates",a.rejected_gates}});
      if(update.accepted) {
        if(stage_=="RECOVERING"){changeStage(raw_recovery_stage_);raw_recovery_stage_.clear();}
        raw_status_["lidar_used"]=raw_status_.value("lidar_used",0)+1;
        {std::lock_guard<std::mutex> imu_lock(raw_imu_mutex_);raw_ingress_.bias=update.gyro_bias;}
        raw_bias_=update.gyro_bias;latest_base_=update.reference_from_base;
        if(!have_reference_){odom_reference_.setIdentity();odom_reference_.translation()=latest_base_.translation();raw_reference_sensor_offset_=-latest_base_.translation();have_reference_=true;}
        cloud_stamp_=processed.stamp;cloud_received_=done.item.received;
        have_cloud_=have_odom_=true;linear_speed_=update.linear_speed;input_error_.clear();
        if(stage_=="SPINNING")yaw_=(update.cumulative_yaw-raw_yaw_at_spin_)*(spin_speed_>0?1:-1);
        raw_latest_yaw_=update.cumulative_yaw;
        if(stage_=="STOPPING") {
          if(linear_speed_>.04||update.angular_speed>.06)confirmations_.clear();
          else if(std::chrono::duration<double>(Clock::now()-stable_start_).count()>=settle_sec_&&done.item.stamp>=raw_stable_ros_stamp_+settle_sec_){confirmations_.push_back(std::move(processed.local_cloud));while(confirmations_.size()>size_t(raw_confirmation_frames_))confirmations_.erase(confirmations_.begin());}
        }
      }else {
        input_error_=update.reason;
        if(update.failure_class=="numerical_recoverable") {
          cmd_pub_->publish(geometry_msgs::msg::Twist{});
          if(raw_frontend_)raw_frontend_->authorizeRecoveryAfterStop(now().seconds());
          stable_start_=Clock::now();raw_stable_ros_stamp_=now().seconds();confirmations_.clear();
          raw_recovery_stage_=stage_;raw_status_["recovery_reason"]=update.reason;changeStage("RECOVERING");
          // Stop remains visible for this tick. Do not consume a queued recovery result here.
          raw_recovery_stop_tick_=true;break;
        }
        const bool waiting=update.failure_class=="waiting";
        if(!waiting){raw_status_["lidar_rejected"]=raw_status_.value("lidar_rejected",0)+1;failure_=update.reason;cancel_.store(true);raw_pipeline_->cancel();break;}
      }
    }
  }
  void snapshotRawScans() {
    const auto s=raw_pipeline_->stats();
    raw_status_.update({{"generation",s.generation},{"scan_queue_capacity",RawPipeline::capacity},{"max_scan_message_bytes",RawPipeline::max_message_bytes},
      {"scan_queue_depth",s.queue_depth},{"scan_result_depth",s.result_depth},{"scan_outstanding_count",s.outstanding},{"scan_queue_bytes",s.bytes},
      {"scan_queue_high_watermark",s.high_watermark},{"scan_queue_max_bytes",s.max_bytes},{"queue_overflow_count",s.overflow_count},
      {"oversized_scan_count",s.oversized_count},{"stale_scan_count",s.stale_count},{"capture_ignored_count",s.ignored_count},{"canceled_scan_count",s.canceled_count},
      {"lidar_received",s.received_count},{"lidar_max_received_stamp_gap_sec",s.max_stamp_gap},{"lidar_max_callback_wall_gap_sec",s.max_callback_gap},
      {"lidar_latest_source_stamp_sec",s.latest_stamp},{"lidar_latest_receipt_wall_sec",s.receipt_wall_sec},{"lidar_latest_receipt_ros_sec",s.receipt_ros_sec},
      {"lidar_latest_receipt_ros_delay_sec",s.receipt_delay_sec},{"lidar_max_receipt_ros_delay_sec",s.max_receipt_delay_sec},{"worker_error",s.error}});
    if(s.latest_stamp>=0){raw_input_stamp_=rclcpp::Time(static_cast<int64_t>(std::llround(s.latest_stamp*1e9)),RCL_ROS_TIME);raw_input_received_=Clock::time_point(std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(s.receipt_wall_sec)));}
    if(!s.error.empty()){failure_=s.error;cancel_.store(true);}
    if(raw_ingress_fault_.exchange(false)){failure_="raw scan frame must equal configured lidar_frame";cancel_.store(true);}
  }
  void processRaw() {snapshotRawImu();snapshotRawScans();consumeRawResults();}
  void odom(nav_msgs::msg::Odometry::ConstSharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    if(msg->header.frame_id!=odom_frame_||msg->child_frame_id!=base_frame_) {
      input_error_="odometry must describe odom -> base_footprint";return;
    }
    const auto & q=msg->pose.pose.orientation;
    Eigen::Quaterniond rotation(q.w,q.x,q.y,q.z);
    const auto & p=msg->pose.pose.position;
    if(!rotation.coeffs().allFinite()||rotation.norm()<1e-6||!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z))return;
    const double now_yaw=yaw(Eigen::Translation3d(p.x,p.y,p.z)*rotation.normalized());
    if(handle_&&stage_=="SPINNING"&&have_yaw_) {
      const double step=wrap(now_yaw-last_yaw_);
      if(std::abs(step)>0.4){failure_="local odometry yaw discontinuity";cancel_.store(true);}
      else yaw_+=step*(spin_speed_>0?1:-1);
    }
    last_yaw_=now_yaw;have_yaw_=true;odom_stamp_=msg->header.stamp;
    odom_received_=Clock::now();have_odom_=true;
    const auto & v=msg->twist.twist;
    if(!std::isfinite(v.linear.x)||!std::isfinite(v.linear.y)||!std::isfinite(v.linear.z)||
      !std::isfinite(v.angular.x)||!std::isfinite(v.angular.y)||!std::isfinite(v.angular.z)) {
      have_odom_=false;input_error_="non-finite odometry twist";return;
    }
    linear_speed_=std::sqrt(v.linear.x*v.linear.x+v.linear.y*v.linear.y+v.linear.z*v.linear.z);
    angular_speed_=std::sqrt(v.angular.x*v.angular.x+v.angular.y*v.angular.y+v.angular.z*v.angular.z);
  }
  bool fresh() {
    input_health_={{"have_cloud",have_cloud_},{"have_odom",have_odom_}};
    if(!have_cloud_||!have_odom_)return false;
    const auto now=Clock::now();
    const auto sensor_received=input_mode_=="raw_imu"&&stage_=="SEARCHING"?raw_input_received_:cloud_received_;
    const auto sensor_stamp=input_mode_=="raw_imu"&&stage_=="SEARCHING"?raw_input_stamp_:cloud_stamp_;
    const double cr=std::chrono::duration<double>(now-sensor_received).count();
    const double od=std::chrono::duration<double>(now-odom_received_).count();
    const double ca=(this->now()-rclcpp::Time(sensor_stamp)).seconds();
    const double oa=(this->now()-rclcpp::Time(odom_stamp_)).seconds();
    input_health_.update({{"cloud_wall_age_sec",cr},{"odom_wall_age_sec",od},
      {"cloud_ros_age_sec",ca},{"odom_ros_age_sec",oa},{"timeout_sec",data_timeout_}});
    return cr<data_timeout_&&od<data_timeout_&&ca>=-0.1&&ca<data_timeout_&&oa>=-0.1&&oa<data_timeout_;
  }
  void changeStage(const std::string & stage) {
    if(stage_!="IDLE"){stage_times_[stage_]=stage_times_.value(stage_,0.0)+stageElapsed();stage_ros_times_[stage_]=stage_ros_times_.value(stage_,0.0)+(now()-stage_ros_start_).seconds();}
    stage_ros_start_=now();stage_=stage;stage_start_=Clock::now();publishStatus("");
  }
  void publishStatus(const std::string & reason) {
    std_msgs::msg::String msg;
    nlohmann::json data={{"status_ros_ns",now().nanoseconds()},{"status_steady_sec",std::chrono::duration<double>(Clock::now().time_since_epoch()).count()},{"run_id",run_id_},{"stage",stage_},{"reason",reason},{"elapsed_sec",handle_?elapsed():last_elapsed_},
      {"estimate_frame",base_frame_},{"map_frame",map_frame_},{"map_sha256",core_->mapHash()},
      {"search_prior",{{"side",search_prior_.search_side},{"boundary_x",search_prior_.side_boundary_x}}},
      {"elapsed_sim_sec",handle_?(now()-sim_start_).seconds():last_sim_elapsed_},
      {"stage_elapsed_sec",stageElapsed()},
      {"input_health",input_health_},
      {"stage_wall_sec",stage_times_},{"stage_ros_sec",stage_ros_times_},{"input_mode",input_mode_},
      {"accumulated_yaw",yaw_},{"metrics",{{"fitness",result_.candidate.fitness},{"inlier_ratio",result_.candidate.inlier_ratio},
      {"retrieval_sec",result_.retrieval_sec},{"gicp_sec",result_.gicp_sec},{"confirmation_sec",result_.confirmation_sec},
      {"database_size",result_.database_size},{"candidates_tested",result_.candidates_tested},
      {"candidate_workers",result_.candidate_workers},{"retrieval_scored_views",result_.retrieval_scored_views},{"retrieval_candidate_count",result_.retrieval_candidate_count},{"retrieval_ring_added",result_.retrieval_ring_added}}}};
    if(input_mode_=="raw_imu")data["raw_frontend"]=raw_status_;
    data["metrics"]["hypotheses"]=nlohmann::json::array();
    for(const auto & h:result_.hypotheses)data["metrics"]["hypotheses"].push_back(
      {{"fitness",h.fitness},{"inlier_ratio",h.inlier_ratio},{"converged",h.converged},
      {"source_points",h.source_points},{"coarse_source_points",h.coarse_source_points},{"coarse_iterations",h.coarse_iterations},{"fine_iterations",h.fine_iterations},
      {"coarse_sec",h.coarse_sec},{"fine_sec",h.fine_sec},{"quality_sec",h.quality_sec},{"total_sec",h.total_sec},{"canceled",h.canceled},
      {"within_search_side",h.within_search_side},
      {"descriptor_distance",h.descriptor_distance},{"ring_key_rank",h.ring_key_rank},{"scan_context_rank",h.scan_context_rank},
      {"from_scan_context",h.from_scan_context},{"from_ring_key",h.from_ring_key},{"seed_xyz",{h.seed_translation.x(),h.seed_translation.y(),h.seed_translation.z()}},{"seed_yaw",h.seed_yaw},{"xyz",{h.map_from_reference.translation().x(),h.map_from_reference.translation().y(),h.map_from_reference.translation().z()}},
      {"yaw",yaw(h.map_from_reference)}});
    data["metrics"]["confirmations"]=nlohmann::json::array();
    for(const auto & h:result_.confirmations)data["metrics"]["confirmations"].push_back({{"fitness",h.fitness},{"inlier_ratio",h.inlier_ratio},{"converged",h.converged},
      {"source_points",h.source_points},{"fine_iterations",h.fine_iterations},{"fine_sec",h.fine_sec},{"quality_sec",h.quality_sec},{"total_sec",h.total_sec},{"canceled",h.canceled}});
    msg.data=data.dump();status_pub_->publish(msg);
  }
  void finish(bool success,const std::string & reason,
    const geometry_msgs::msg::PoseStamped & pose=geometry_msgs::msg::PoseStamped()) {
    cmd_pub_->publish(geometry_msgs::msg::Twist{});
    if(!rclcpp::ok(get_node_base_interface()->get_context())) {
      cancel_.store(true);return;
    }
    auto message=std::make_shared<Action::Result>();message->success=success;message->reason=reason;message->pose=pose;
    message->fitness=result_.candidate.fitness;message->inlier_ratio=result_.candidate.inlier_ratio;message->elapsed_sec=elapsed();last_elapsed_=message->elapsed_sec;last_sim_elapsed_=(now()-sim_start_).seconds();
    stage_times_[stage_]=stage_times_.value(stage_,0.0)+stageElapsed();stage_ros_times_[stage_]=stage_ros_times_.value(stage_,0.0)+(now()-stage_ros_start_).seconds();stage_=success?"SUCCEEDED":(handle_->is_canceling()?"CANCELED":"FAILED");publishStatus(reason);
    try {
      if(success){handle_->succeed(message);pose_pub_->publish(pose);}
      else if(handle_->is_canceling())handle_->canceled(message);else handle_->abort(message);
    }catch(const std::runtime_error & e) {
      // The context can shut down between the check above and terminal result
      // dispatch. ActionServer may already have removed its active goals then.
      // A normal-runtime action error still propagates and remains visible.
      if(rclcpp::ok(get_node_base_interface()->get_context()))throw;
      std::cerr<<"global_relocalization action stopped during shutdown: "<<e.what()<<std::endl;
    }
    {std::lock_guard<std::mutex> imu_lock(raw_imu_mutex_);raw_ingress_.active=false;}
    if(raw_pipeline_)raw_pipeline_->cancel();raw_capture_mode_.store(0);
    handle_.reset();accumulated_.clear();have_reference_=false;
  }
  void tick() {
    if(shutting_down_.load()||!rclcpp::ok(get_node_base_interface()->get_context())){cancel_.store(true);return;}
    std::lock_guard<std::mutex> lock(mutex_);geometry_msgs::msg::Twist cmd;
    if(shutting_down_.load())return;
    if(input_mode_=="raw_imu")processRaw();else processClouds();
    if(!handle_){cmd_pub_->publish(cmd);return;}
    if(elapsed()>timeout_sec_){failure_="wall-clock experiment timeout";
      if(!input_error_.empty())failure_+=": "+input_error_;
      cancel_.store(true);}
    if(cancel_.load()) {
      if(raw_pipeline_&&raw_capture_mode_.exchange(0)!=0)raw_pipeline_->cancel();
      cmd_pub_->publish(cmd);
      if(worker_.valid()&&worker_.wait_for(std::chrono::seconds(0))!=std::future_status::ready)return;
      if(worker_.valid())try{result_=worker_.get();}catch(...){}
      finish(false,failure_.empty()?"requested cancellation":failure_);return;
    }
    if(!fresh()) {
      cmd_pub_->publish(cmd);
      if(stage_=="SETTLING"){stable_start_=Clock::now();publishStatus(input_error_.empty()?"waiting for fresh scan/odometry/TF":input_error_);return;}
      failure_="scan/odometry/TF became stale";
      if(!input_error_.empty())failure_+=": "+input_error_;
      failure_+=" "+input_health_.dump();
      cancel_.store(true);return;
    }
    if(raw_recovery_stop_tick_){raw_recovery_stop_tick_=false;cmd_pub_->publish(cmd);publishStatus("numerical recovery stop published");return;}
    if(stage_=="RECOVERING"){cmd_pub_->publish(cmd);publishStatus("waiting for one post-stop scan against last trusted submap");return;}
    if(stage_=="SETTLING") {
      if(!have_reference_) {
        try {base_lidar_=transform(base_frame_,lidar_frame_,cloud_stamp_);odom_reference_=latest_base_;have_reference_=true;}
        catch(const std::exception & e){stable_start_=Clock::now();publishStatus(e.what());cmd_pub_->publish(cmd);return;}
      }
      if(linear_speed_>0.04||angular_speed_>0.06)stable_start_=Clock::now();
      if(std::chrono::duration<double>(Clock::now()-stable_start_).count()>=settle_sec_){yaw_=0;raw_yaw_at_spin_=raw_latest_yaw_;changeStage("SPINNING");}
    }else if(stage_=="SPINNING") {
      if((latest_base_.translation()-odom_reference_.translation()).norm()>max_translation_) {
        failure_="local odometry shows excessive translation during rotation";cancel_.store(true);
      }else if(yaw_>=spin_angle_){stable_start_=Clock::now();raw_stable_ros_stamp_=now().seconds();raw_capture_mode_.store(2);confirmations_.clear();changeStage("STOPPING");}
      else cmd.angular.z=spin_speed_;
    }else if(stage_=="STOPPING") {
      if(linear_speed_>0.04||angular_speed_>0.06){stable_start_=Clock::now();raw_stable_ros_stamp_=now().seconds();}
      if(std::chrono::duration<double>(Clock::now()-stable_start_).count()>=settle_sec_) {
        if(input_mode_=="raw_imu") {
          if(confirmations_.size()<size_t(raw_confirmation_frames_)){cmd_pub_->publish(cmd);return;}
          raw_pipeline_->freeze();consumeRawResults();snapshotRawScans();
          if(cancel_.load()){cmd_pub_->publish(cmd);return;}
          accumulated_=std::move(raw_capture_->accumulated);raw_capture_mode_.store(3);
        }
        if(accumulated_.size()<100){finish(false,"insufficient accumulated points");return;}
        Cloud independent;if(input_mode_=="registered")pcl::transformPointCloud(latest_cloud_,independent,odom_reference_.inverse().matrix().cast<float>());
        final_odom_=latest_base_;final_stamp_=cloud_stamp_;
        auto query=std::move(accumulated_);Eigen::Vector3d offset=input_mode_=="raw_imu"?raw_reference_sensor_offset_:base_lidar_.translation();auto confirmations=std::move(confirmations_);if(input_mode_=="registered")confirmations={independent};changeStage("SEARCHING");
        worker_=std::async(std::launch::async,[this,query=std::move(query),confirmations=std::move(confirmations),offset]{
          return core_->search(query,confirmations,offset,[this]{return cancel_.load();});
        });
      }
    }else if(stage_=="SEARCHING") {
      if(worker_.valid()&&worker_.wait_for(std::chrono::seconds(0))==std::future_status::ready) {
        try{result_=worker_.get();}catch(const std::exception & e){finish(false,e.what());return;}
        if(!result_.success){finish(false,result_.reason);return;}
        const auto pose=composeCurrentBasePose(result_.candidate.map_from_reference,odom_reference_,final_odom_);
        if(!poseInSearchSide(pose,search_prior_)){finish(false,"current base pose crossed configured search-side boundary");return;}
        geometry_msgs::msg::PoseStamped message;message.header.frame_id=map_frame_;message.header.stamp=final_stamp_;
        message.pose.position.x=pose.translation().x();message.pose.position.y=pose.translation().y();message.pose.position.z=pose.translation().z();
        const Eigen::Quaterniond q(pose.rotation());message.pose.orientation.x=q.x();message.pose.orientation.y=q.y();message.pose.orientation.z=q.z();message.pose.orientation.w=q.w();
        finish(true,result_.reason,message);return;
      }
    }
    cmd_pub_->publish(cmd);
    auto feedback=std::make_shared<Action::Feedback>();feedback->stage=stage_;
    feedback->progress=stage_=="SPINNING"?float(std::clamp(yaw_/spin_angle_,0.,1.)*.6+.1):stage_=="SETTLING"?0.:.8;
    feedback->accumulated_yaw=yaw_;handle_->publish_feedback(feedback);
  }
  Eigen::Vector3d raw_bias_{Eigen::Vector3d::Zero()},raw_reference_sensor_offset_{Eigen::Vector3d::Zero()};
  builtin_interfaces::msg::Time raw_input_stamp_;
  Clock::time_point raw_input_received_;
  double latest_imu_stamp_{-1},raw_yaw_at_spin_{0},raw_latest_yaw_{0},raw_stable_ros_stamp_{0};
  std::atomic<uint64_t> raw_generation_{0};std::atomic<int> raw_capture_mode_{0};std::atomic_bool raw_ingress_fault_{false};
  std::string imu_frame_;
  std::string input_mode_,raw_scan_model_,raw_diagnostic_directory_,raw_recovery_stage_;bool raw_recovery_stop_tick_{false};double raw_calibration_sec_;int raw_confirmation_frames_;
  RawFrontendParams raw_params_;std::shared_ptr<StationaryRawFrontend> raw_frontend_;
  std::unique_ptr<RawPipeline> raw_pipeline_;std::shared_ptr<RawCapture> raw_capture_;
  std::mutex raw_imu_mutex_;RawImuIngress raw_ingress_;
  rclcpp::CallbackGroup::SharedPtr imu_group_,cloud_group_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  std::vector<Cloud> confirmations_;
  nlohmann::json raw_status_;nlohmann::json stage_ros_times_=nlohmann::json::object();
  rclcpp::Time stage_ros_start_{0,0,RCL_ROS_TIME};
  std::mutex mutex_;std::atomic_bool cancel_{false},shutting_down_{false};
  rclcpp::PreShutdownCallbackHandle shutdown_callback_;
  std::unique_ptr<Core> core_;std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  Params search_prior_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp_action::Server<Action>::SharedPtr server_;std::shared_ptr<Handle> handle_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::future<SearchResult> worker_;SearchResult result_;
  nlohmann::json stage_times_=nlohmann::json::object();
  nlohmann::json input_health_=nlohmann::json::object();
  std::string stage_{"IDLE"},failure_,input_error_,run_id_,map_frame_,odom_frame_,base_frame_,lidar_frame_;
  uint64_t run_count_{0};
  Clock::time_point start_,stage_start_,stable_start_,cloud_received_,odom_received_;
  double settle_sec_,spin_speed_,spin_angle_,timeout_sec_,data_timeout_,max_translation_,accumulation_leaf_;
  double yaw_{0},last_yaw_{0},linear_speed_{0},angular_speed_{0},last_elapsed_{0},last_sim_elapsed_{0};
  rclcpp::Time sim_start_{0,0,RCL_ROS_TIME};
  bool have_cloud_{false},have_odom_{false},have_yaw_{false},have_reference_{false};
  Cloud latest_cloud_,accumulated_;
  struct PendingCloud {sensor_msgs::msg::PointCloud2::ConstSharedPtr message;Clock::time_point received;};
  std::deque<PendingCloud> pending_clouds_;
  builtin_interfaces::msg::Time cloud_stamp_,odom_stamp_,final_stamp_;
  Eigen::Isometry3d latest_base_{Eigen::Isometry3d::Identity()},odom_reference_{Eigen::Isometry3d::Identity()},final_odom_{Eigen::Isometry3d::Identity()},base_lidar_{Eigen::Isometry3d::Identity()};
};
}
int main(int argc,char ** argv) {
  rclcpp::init(argc,argv);
  try{auto node=std::make_shared<global_relocalization::Node>();rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions{},3);executor.add_node(node);executor.spin();executor.remove_node(node);}
  catch(const std::exception & e){std::cerr<<"global_relocalization: "<<e.what()<<std::endl;rclcpp::shutdown();return 1;}
  rclcpp::shutdown();return 0;
}
