// Copyright 2026 RM27 contributors. SPDX-License-Identifier: Apache-2.0
#pragma once
#include "global_relocalization/core.hpp"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <nlohmann/json.hpp>
#include <deque>
#include <mutex>
#include <sstream>
#include <map>

namespace global_relocalization {
// Diagnostic snapshots only. Solver callbacks enqueue bounded data and never publish.
class Visualization {
  using Steady=std::chrono::steady_clock;
  using Marker=visualization_msgs::msg::Marker;
  rclcpp::Node & node_;
  std::string map_frame_,reference_frame_{"relocalization_reference"},run_id_,stage_{"IDLE"};
  Params params_;
  double period_; size_t cap_;
  std::mutex mutex_,publication_mutex_;
  struct Pending {SearchEvent event; Cloud scan; uint64_t sequence;};
  std::deque<Pending> pending_;
  uint64_t generation_{0},sequence_{0},dropped_{0};
  size_t map_subscribers_{0},marker_subscribers_{0};
  Cloud query_,map_;
  bool query_dirty_{false},map_dirty_{true},clear_dirty_{false},stage_dirty_{true},closed_{false};
  Steady::time_point last_query_{},last_aligned_{};
  Eigen::Isometry3d alignment_{Eigen::Isometry3d::Identity()};
  bool have_alignment_{false};
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr query_pub_,independent_pub_,map_pub_,aligned_pub_,aligned_independent_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr events_pub_;
  std::map<int,Marker> displayed_;
  Marker marker(int id,int type) const {
    Marker m;m.header.frame_id=map_frame_;m.header.stamp=node_.now();m.ns="observer";m.id=id;m.type=type;m.action=Marker::ADD;m.pose.orientation.w=1.;m.color.a=1.;return m;
  }
  sensor_msgs::msg::PointCloud2 cloud(const Cloud & data,const std::string & frame) const {
    sensor_msgs::msg::PointCloud2 msg;pcl::toROSMsg(data,msg);msg.header.frame_id=frame;msg.header.stamp=node_.now();return msg;
  }
  void arrow(const Candidate & c,int id,const std::string & label,float r,float g,float b) {
    if(!c.map_from_reference.matrix().allFinite())return;
    auto m=marker(id,Marker::ARROW);m.pose.position.x=c.map_from_reference.translation().x();m.pose.position.y=c.map_from_reference.translation().y();m.pose.position.z=c.map_from_reference.translation().z();
    Eigen::Quaterniond q(c.map_from_reference.rotation());m.pose.orientation.x=q.x();m.pose.orientation.y=q.y();m.pose.orientation.z=q.z();m.pose.orientation.w=q.w();m.scale.x=.8;m.scale.y=.12;m.scale.z=.12;m.color.r=r;m.color.g=g;m.color.b=b;displayed_[id]=m;
    auto text=marker(id+1000,Marker::TEXT_VIEW_FACING);text.pose.position=m.pose.position;text.pose.position.z+=.5;text.scale.z=.25;text.color.r=text.color.g=text.color.b=1.;text.text=label;displayed_[text.id]=text;
  }
public:
  Visualization(rclcpp::Node & node,const std::string & map_path,const std::string & frame,const Params & params)
    :node_(node),map_frame_(frame),params_(params) {
    const auto hz=node.declare_parameter<double>("visualization_cloud_hz",1.);
    const auto limit=node.declare_parameter<int>("visualization_max_points",3000);
    if(!std::isfinite(hz)||hz<=0||hz>5||limit<100||limit>20000)throw std::invalid_argument("visualization_cloud_hz must be (0,5], visualization_max_points [100,20000]");
    period_=1./hz;cap_=size_t(limit);
    Cloud input;if(pcl::io::loadPCDFile(map_path,input)<0||input.empty())throw std::runtime_error("Cannot load observer prior PCD: "+map_path);
    map_=sample(input);if(map_.empty())throw std::runtime_error("Observer prior PCD has no finite sampled points");
    const auto qos=rclcpp::QoS(1).best_effort().transient_local();
    query_pub_=node.create_publisher<sensor_msgs::msg::PointCloud2>("/global_relocalization/visual/query",qos);
    independent_pub_=node.create_publisher<sensor_msgs::msg::PointCloud2>("/global_relocalization/visual/independent",qos);
    aligned_pub_=node.create_publisher<sensor_msgs::msg::PointCloud2>("/global_relocalization/visual/aligned_query",qos);
    aligned_independent_pub_=node.create_publisher<sensor_msgs::msg::PointCloud2>("/global_relocalization/visual/aligned_independent",qos);
    map_pub_=node.create_publisher<sensor_msgs::msg::PointCloud2>("/global_relocalization/visual/map",qos);
    markers_pub_=node.create_publisher<visualization_msgs::msg::MarkerArray>("/global_relocalization/visual/markers",qos);
    events_pub_=node.create_publisher<std_msgs::msg::String>("/global_relocalization/visual/events",rclcpp::QoS(32).best_effort());
  }
  Cloud sample(const Cloud & input) const {
    Cloud out;const size_t stride=std::max(size_t(1),(input.size()+cap_-1)/cap_);out.reserve(std::min(cap_,input.size()));
    for(size_t i=0;i<input.size();i+=stride)if(pcl::isFinite(input[i]))out.push_back(input[i]);return out;
  }
  uint64_t reset(const std::string & id) {
    // Serialize with outgoing snapshots so a new generation cannot inherit late publications.
    std::lock_guard<std::mutex> output(publication_mutex_);std::lock_guard<std::mutex> lock(mutex_);
    ++generation_;run_id_=id;pending_.clear();query_.clear();query_dirty_=clear_dirty_=stage_dirty_=true;have_alignment_=false;closed_=false;sequence_=dropped_=0;last_query_={};last_aligned_={};displayed_.clear();return generation_;
  }
  void query(const Cloud & input,uint64_t generation,bool force=false) {
    std::unique_lock<std::mutex> lock(mutex_,std::try_to_lock);if(!lock.owns_lock()||generation!=generation_||closed_)return;
    const auto now=Steady::now();if(!force&&std::chrono::duration<double>(now-last_query_).count()<period_)return;
    last_query_=now;query_=sample(input);query_dirty_=true;
  }
  void observe(const SearchEvent & event,uint64_t generation) {
    std::lock_guard<std::mutex> lock(mutex_);if(generation!=generation_||closed_)return;
    if(pending_.size()>=256){pending_.pop_front();++dropped_;}
    Pending item{event,{},++sequence_};if(event.scan)item.scan=sample(*event.scan);item.event.scan=nullptr;pending_.push_back(std::move(item));
  }
  void stage(const std::string & name) {std::lock_guard<std::mutex> lock(mutex_);stage_=name;stage_dirty_=true;}
  void finish(const std::string & name,const geometry_msgs::msg::PoseStamped * accepted_pose=nullptr) {
    std::lock_guard<std::mutex> lock(mutex_);closed_=true;stage_=name;stage_dirty_=true;
    if(accepted_pose) {Candidate c;const auto & p=accepted_pose->pose;c.map_from_reference.translation()<<p.position.x,p.position.y,p.position.z;c.map_from_reference.linear()=Eigen::Quaterniond(p.orientation.w,p.orientation.x,p.orientation.y,p.orientation.z).toRotationMatrix();arrow(c,900,"CURRENT RUN ACCEPTED",0.F,1.F,0.F);}
  }
  void drain() {
    std::lock_guard<std::mutex> output(publication_mutex_);
    using CloudPub=rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr;
    std::vector<std::pair<CloudPub,sensor_msgs::msg::PointCloud2>> clouds;
    std::vector<std_msgs::msg::String> events;visualization_msgs::msg::MarkerArray markers;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const auto map_subscribers=map_pub_->get_subscription_count();
      if(map_subscribers>0&&map_subscribers!=map_subscribers_)map_dirty_=true;
      map_subscribers_=map_subscribers;
      const auto marker_subscribers=markers_pub_->get_subscription_count();
      if(marker_subscribers>0&&marker_subscribers!=marker_subscribers_)stage_dirty_=true;
      marker_subscribers_=marker_subscribers;
      const bool changed=clear_dirty_||stage_dirty_||!pending_.empty();
      if(clear_dirty_) {
        auto clear=marker(0,0);clear.action=Marker::DELETEALL;markers.markers.push_back(clear);
        for(const auto & pub:{query_pub_,independent_pub_})clouds.emplace_back(pub,cloud({},reference_frame_));
        for(const auto & pub:{aligned_pub_,aligned_independent_pub_})clouds.emplace_back(pub,cloud({},map_frame_));clear_dirty_=false;
      }
      if(map_dirty_&&map_subscribers>0){clouds.emplace_back(map_pub_,cloud(map_,map_frame_));map_dirty_=false;}
      if(query_dirty_){clouds.emplace_back(query_pub_,cloud(query_,reference_frame_));query_dirty_=false;}
      for(auto & item:pending_) {
        const auto & e=item.event;const auto & c=e.candidate;
        const bool fine=e.stage=="GICP_FINE_QUALITY_COMPLETE"||e.stage=="CONFIRMATION_COMPLETE";
        const bool coarse=e.stage=="GICP_COARSE_SEED_COMPLETE";
        const bool finite_transform=c.map_from_reference.matrix().allFinite();
        nlohmann::json gates={{"converged",c.converged},{"inliers",c.inliers>=20},{"finite_transform",finite_transform},{"finite_fitness",std::isfinite(c.fitness)},{"fitness",c.fitness<=params_.max_fitness},{"inlier_ratio",c.inlier_ratio>=params_.min_inlier_ratio},{"search_side",c.within_search_side},{"not_canceled",!c.canceled}};
        const bool quality=c.converged&&c.inliers>=20&&finite_transform&&std::isfinite(c.fitness)&&c.fitness<=params_.max_fitness&&c.inlier_ratio>=params_.min_inlier_ratio&&c.within_search_side&&!c.canceled;
        const bool confirmation=e.stage=="CONFIRMATION_COMPLETE";
        const bool consistent=confirmation&&std::isfinite(e.best_translation_delta)&&std::isfinite(e.best_rotation_delta)&&std::isfinite(e.previous_translation_delta)&&std::isfinite(e.previous_rotation_delta)&&e.best_translation_delta<=params_.confirmation_max_translation&&e.best_rotation_delta<=params_.confirmation_max_rotation&&e.previous_translation_delta<=params_.confirmation_max_translation&&e.previous_rotation_delta<=params_.confirmation_max_rotation;
        nlohmann::json data={{"search_id",run_id_},{"sequence",item.sequence},{"dropped_events",dropped_},{"event",e.stage},{"index",e.index},{"total",e.total},{"scan_context_rank",c.scan_context_rank},{"ring_key_rank",c.ring_key_rank},{"fitness",c.fitness},{"inliers",c.inliers},{"inlier_ratio",c.inlier_ratio},{"converged",c.converged},{"within_search_side",c.within_search_side},{"canceled",c.canceled},{"coarse_iterations",c.coarse_iterations},{"fine_iterations",c.fine_iterations},{"max_fitness",params_.max_fitness},{"min_inliers",20},{"min_inlier_ratio",params_.min_inlier_ratio},{"confirmation_max_translation",params_.confirmation_max_translation},{"confirmation_max_rotation",params_.confirmation_max_rotation},{"ambiguity_margin",params_.ambiguity_margin}};
        data["quality_gates"]=fine?gates:nlohmann::json(nullptr);data["quality_gates_passed"]=fine?nlohmann::json(quality):nlohmann::json(nullptr);
        data["confirmation_consistency_passed"]=confirmation?nlohmann::json(consistent):nlohmann::json(nullptr);
        data["coarse_seed_state"]=coarse?nlohmann::json(c.coarse_iterations==0?"skipped":finite_transform?"usable":"nonfinite"):nlohmann::json(nullptr);
        data["coarse_seed_converged"]=coarse?nlohmann::json(c.converged):nlohmann::json(nullptr);
        if(finite_transform){Eigen::Quaterniond q(c.map_from_reference.rotation());data["xyz"]={c.map_from_reference.translation().x(),c.map_from_reference.translation().y(),c.map_from_reference.translation().z()};data["quaternion_xyzw"]={q.x(),q.y(),q.z(),q.w()};}
        if(confirmation){data["best_translation_delta"]=e.best_translation_delta;data["best_rotation_delta"]=e.best_rotation_delta;data["previous_translation_delta_max"]=e.previous_translation_delta;data["previous_rotation_delta_max"]=e.previous_rotation_delta;}
        std_msgs::msg::String message;message.data=data.dump();events.push_back(std::move(message));
        std::ostringstream label;label<<e.stage<<" index="<<e.index<<" SC#"<<c.scan_context_rank<<" ring#"<<c.ring_key_rank;
        if(fine)label<<" quality="<<quality<<" RMSE="<<c.fitness<<"<="<<params_.max_fitness<<" inliers="<<c.inliers<<">=20 ratio="<<c.inlier_ratio<<">="<<params_.min_inlier_ratio<<" converged="<<c.converged<<" finite="<<finite_transform<<" side="<<c.within_search_side<<" canceled="<<c.canceled;
        if(coarse)label<<" seed="<<data["coarse_seed_state"]<<" (no quality gate)";
        if(confirmation)label<<" consistent="<<consistent<<" delta(best)="<<e.best_translation_delta<<"m/"<<e.best_rotation_delta<<"rad delta(previous,max)="<<e.previous_translation_delta<<"m/"<<e.previous_rotation_delta<<"rad";
        auto status=marker(0,Marker::TEXT_VIEW_FACING);status.scale.z=.35;status.pose.position.z=3.;status.color.r=status.color.g=status.color.b=1.;status.text=run_id_+" seq="+std::to_string(item.sequence)+"\n"+label.str()+"\nStage boundaries only; candidate quality is not action acceptance";displayed_[0]=status;
        if(e.stage=="RETRIEVAL_COMPLETE")for(size_t i=0;i<e.seeds.size();++i)arrow(e.seeds[i],int(i)+10,"SC#"+std::to_string(e.seeds[i].scan_context_rank)+" ring#"+std::to_string(e.seeds[i].ring_key_rank)+" seed",1.F,.7F,0.F);
        else {
          const bool pass=fine&&quality&&(!confirmation||consistent);
          arrow(c,e.stage.find("CONFIRMATION")==0?500+int(e.index):10+int(e.index),label.str(),fine?(pass?0.F:1.F):1.F,fine?(pass?1.F:0.F):.7F,0.F);
          if(finite_transform){alignment_=c.map_from_reference;have_alignment_=true;}
        }
        if(!item.scan.empty()) {
          clouds.emplace_back(independent_pub_,cloud(item.scan,reference_frame_));
          if(finite_transform){Cloud aligned;pcl::transformPointCloud(item.scan,aligned,c.map_from_reference.matrix().cast<float>());clouds.emplace_back(aligned_independent_pub_,cloud(aligned,map_frame_));}
        }
      }
      pending_.clear();
      if(changed){auto status=marker(1,Marker::TEXT_VIEW_FACING);status.pose.position.z=4.;status.scale.z=.45;status.color.r=status.color.g=status.color.b=1.;status.text=run_id_+" "+stage_;displayed_[1]=status;for(const auto & pair:displayed_)markers.markers.push_back(pair.second);stage_dirty_=marker_subscribers==0;}
      if(have_alignment_&&std::chrono::duration<double>(Steady::now()-last_aligned_).count()>=period_) {Cloud aligned;pcl::transformPointCloud(query_,aligned,alignment_.matrix().cast<float>());clouds.emplace_back(aligned_pub_,cloud(aligned,map_frame_));last_aligned_=Steady::now();}
    }
    // No node/control or observer queue mutex is held during middleware publication.
    for(const auto & pair:clouds)pair.first->publish(pair.second);
    for(const auto & event:events)events_pub_->publish(event);
    if(!markers.markers.empty())markers_pub_->publish(markers);
  }
};
}
