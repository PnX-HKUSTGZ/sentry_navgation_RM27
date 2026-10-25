// Copyright 2026 RM27 contributors. SPDX-License-Identifier: Apache-2.0
#include "global_relocalization/stationary_raw_frontend.hpp"
#include <pcl/io/pcd_io.h>
#include <nlohmann/json.hpp>
#include <filesystem>
#include <fstream>
#include <iostream>
using namespace global_relocalization;
int main(int argc,char**argv){try{
  if(argc<2||argc>3)throw std::runtime_error("usage: global_relocalization_raw_replay DIRECTORY [REPETITIONS=20]");
  int repetitions=argc>2?std::stoi(argv[2]):20;if(repetitions<1||repetitions>1000)throw std::runtime_error("repetitions must be 1..1000");
  std::vector<std::filesystem::path>files;for(auto&e:std::filesystem::directory_iterator(argv[1]))if(e.path().extension()==".json")files.push_back(e.path());std::sort(files.begin(),files.end());if(files.empty())throw std::runtime_error("no diagnostic scans");
  auto read=[](const auto&p){std::ifstream f(p);nlohmann::json j;f>>j;return j;};auto first=read(files.front());auto config=first.at("params");
  RawFrontendParams p;
#define PARAM(name) if(config.contains(#name))config.at(#name).get_to(p.name)
  PARAM(retry_iterations);PARAM(stationary_duration);PARAM(stationary_min_samples);PARAM(stationary_max_gyro);PARAM(stationary_gyro_std);PARAM(gravity_magnitude);PARAM(gravity_tolerance);PARAM(stationary_accel_std);PARAM(max_imu_gap);PARAM(max_scan_gap);PARAM(imu_history_sec);PARAM(leaf_size);PARAM(max_correspondence);PARAM(max_iterations);PARAM(max_fitness);PARAM(min_inlier_ratio);PARAM(min_points);PARAM(max_rotation_prior_error);PARAM(max_base_translation_step);PARAM(max_base_linear_speed);PARAM(min_geometry_eigen_ratio);PARAM(min_information_eigen_ratio);PARAM(diagnostic_budget_sec);PARAM(range_filter);PARAM(min_range);PARAM(max_range);PARAM(deterministic);PARAM(submap_scans);PARAM(imu_rotation_prior);PARAM(gyro_noise_density);PARAM(rotation_sigma_floor);PARAM(numerical_recovery);
#undef PARAM
  p.record_iterations=true;
  auto matrix=[](const nlohmann::json&v){Eigen::Isometry3d t=Eigen::Isometry3d::Identity();for(int r=0;r<4;++r)for(int c=0;c<4;++c)t.matrix()(r,c)=v.at(r*4+c).get<double>();return t;};
  nlohmann::json report={{"schema",1},{"numerical_only",true},{"replays_receipt_latency",false},{"repetitions",repetitions},{"scan_count",files.size()},{"params",config},{"runs",nlohmann::json::array()}};std::vector<double>baseline;std::vector<bool>baseline_accept;bool identical=true;
  for(int run=0;run<repetitions;++run){StationaryRawFrontend frontend(p,matrix(first.at("sensor_from_imu")),matrix(first.at("base_from_sensor")));double last_imu=-1;size_t accepted=0,rejected=0;std::vector<double>poses;std::vector<bool>accept;auto rows=nlohmann::json::array();nlohmann::json first_failure=nullptr;double last_accepted=-1,last_yaw=0;
    for(auto&file:files){auto j=read(file);for(auto&row:j.at("imu")){double t=row.at(0);if(t<=last_imu)continue;std::string reason;if(!frontend.addImu({t,Eigen::Vector3d(row.at(1).get<double>(),row.at(2).get<double>(),row.at(3).get<double>()),Eigen::Vector3d(row.at(4).get<double>(),row.at(5).get<double>(),row.at(6).get<double>())},&reason))throw std::runtime_error(reason);last_imu=t;}
      if(j.value("recovery_after",-1.)>=0)frontend.authorizeRecoveryAfterStop(j.at("recovery_after"));
      Cloud cloud;auto path=file;path.replace_extension(".pcd");if(pcl::io::loadPCDFile(path.string(),cloud)<0)throw std::runtime_error("cannot load scan");auto out=frontend.processScan(cloud,j.at("stamp"),j.at("instantaneous"));if(out.accepted){++accepted;last_accepted=j.at("stamp");last_yaw=out.cumulative_yaw;}else {++rejected;if(out.failure_class=="hard_fault"&&first_failure.is_null())first_failure={{"stamp",j.at("stamp")},{"reason",out.reason}};}
      // Recovery authorization comes only from the captured post-stop event.
      for(int r=0;r<4;++r)for(int c=0;c<4;++c)poses.push_back(out.reference_from_sensor.matrix()(r,c));accept.push_back(out.accepted);std::vector<double> row_pose;for(int r=0;r<4;++r)for(int c=0;c<4;++c)row_pose.push_back(out.reference_from_sensor.matrix()(r,c));rows.push_back({{"stamp",j.at("stamp")},{"pose",row_pose},{"cumulative_yaw",out.cumulative_yaw},{"accepted",out.accepted},{"reason",out.reason},{"failure_class",out.failure_class},{"fitness",out.fitness},{"inlier_ratio",out.inlier_ratio},{"information_eigen_ratio",out.information_eigen_ratio},{"converged",out.converged},{"rejected_gates",out.rejected_gates},{"translation_step",out.translation_step},{"rotation_prior_error",out.rotation_prior_error},{"iterations",out.iterations}});
      if(!out.accepted&&out.failure_class!="waiting") {rows.back()["iteration_trace"]=nlohmann::json::array();for(const auto&r:out.iteration_trace){std::vector<double>d(r.increment.data(),r.increment.data()+6);rows.back()["iteration_trace"].push_back({{"attempt",r.attempt},{"iteration",r.iteration},{"inner",r.inner},{"objective",r.objective},{"candidate_objective",r.candidate_objective},{"lambda",r.lambda},{"increment",d},{"accepted",r.accepted}});}}
    }
    if(run==0){baseline=poses;baseline_accept=accept;}else if(poses!=baseline||accept!=baseline_accept)identical=false;report["runs"].push_back({{"run",run+1},{"first_hard_rejection",first_failure},{"last_accepted_stamp",last_accepted},{"last_accepted_cumulative_yaw",last_yaw},{"accepted",accepted},{"rejected",rejected},{"scans",rows}});
  }
  report["bitwise_identical"]=identical;std::cout<<report.dump(2)<<std::endl;return identical?0:2;
}catch(const std::exception&e){std::cerr<<e.what()<<std::endl;return 1;}}
