// Copyright 2026 RM27 contributors. SPDX-License-Identifier: Apache-2.0
#include "global_relocalization/stationary_raw_frontend.hpp"
#include "global_relocalization/raw_preprocessing.hpp"
#include "global_relocalization/raw_registration.hpp"
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <nlohmann/json.hpp>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <pcl/kdtree/kdtree_flann.h>
#include <small_gicp/pcl/pcl_registration.hpp>
#include <small_gicp/pcl/pcl_point.hpp>
#include <small_gicp/util/downsampling_omp.hpp>
#include <Eigen/Eigenvalues>
#include <chrono>
#include <deque>
#include <mutex>
#include <atomic>

namespace global_relocalization {
namespace { bool validRigid(const Eigen::Isometry3d&t){return t.matrix().allFinite()&&(t.rotation().transpose()*t.rotation()).isApprox(Eigen::Matrix3d::Identity(),1e-6)&&std::abs(t.rotation().determinant()-1.)<1e-6&&t.matrix().row(3).isApprox(Eigen::RowVector4d(0,0,0,1),1e-12);} }

struct StationaryRawFrontend::Impl {
  RawFrontendParams p; Eigen::Isometry3d sensor_imu, base_sensor;
  std::mutex imu_mutex; std::deque<RawImuSample> imu; std::string imu_error;
  double gyro_sample_variance{0},gyro_bias_variance{0},calibration_sample_dt{0};
  double last_input{-1}, previous_stamp{-1}, calibration_stamp{-1}, cumulative_yaw{0};
  bool initialized{false}; Eigen::Vector3d bias{Eigen::Vector3d::Zero()};
  std::atomic<double> recovery_after{-1};
  bool hard_fault{false},recovery_pending{false};size_t recovery_attempts{0},recovered_count{0},sequence{0};
  std::deque<std::pair<Cloud::Ptr,Eigen::Isometry3d>> submap;
  Eigen::Isometry3d pose{Eigen::Isometry3d::Identity()}; Cloud::Ptr previous;
  Impl(const RawFrontendParams & params,const Eigen::Isometry3d & si,const Eigen::Isometry3d & bs):p(params),sensor_imu(si),base_sensor(bs){}
  static Eigen::Vector3d gyroAt(const std::deque<RawImuSample>& samples,double stamp) {
    for(size_t i=0;i<samples.size();++i) {
      if(samples[i].stamp==stamp)return samples[i].angular_velocity;
      if(samples[i].stamp>stamp&&i){const auto&a=samples[i-1];const auto&b=samples[i];double u=(stamp-a.stamp)/(b.stamp-a.stamp);return (1-u)*a.angular_velocity+u*b.angular_velocity;}
    }
    return samples.back().angular_velocity;
  }
};
StationaryRawFrontend::StationaryRawFrontend(const RawFrontendParams & p,const Eigen::Isometry3d & si,const Eigen::Isometry3d & bs):impl_(std::make_unique<Impl>(p,si,bs)) {
  if(!std::isfinite(p.stationary_translation_sigma_floor)||p.stationary_translation_sigma_floor<=0)throw std::invalid_argument("invalid stationary translation prior uncertainty");
  if(p.stationary_duration<=0||p.stationary_min_samples<2||p.leaf_size<=0||p.max_imu_gap<=0||p.max_scan_gap<=0||p.max_iterations<1||p.retry_iterations<0||p.retry_iterations>15||p.submap_scans<1||p.submap_scans>30||p.min_range<0||p.max_range<=p.min_range||p.rotation_sigma_floor<=0||p.gyro_noise_density<=0||!validRigid(si)||!validRigid(bs))throw std::invalid_argument("invalid raw frontend parameters/extrinsics");
}
StationaryRawFrontend::~StationaryRawFrontend()=default;
void StationaryRawFrontend::reset(){auto & i=*impl_;std::lock_guard<std::mutex> lock(i.imu_mutex);i.imu.clear();i.imu_error.clear();i.last_input=i.previous_stamp=i.calibration_stamp=-1;i.initialized=false;i.bias.setZero();i.pose.setIdentity();i.previous.reset();i.cumulative_yaw=0;i.hard_fault=false;i.recovery_pending=false;i.recovery_attempts=i.recovered_count=i.sequence=0;i.submap.clear();i.recovery_after.store(-1);i.gyro_sample_variance=i.gyro_bias_variance=i.calibration_sample_dt=0;}
void StationaryRawFrontend::authorizeRecoveryAfterStop(double stamp){impl_->recovery_after.store(stamp);}
bool StationaryRawFrontend::addImu(const RawImuSample & s,std::string * reason) {
  auto & i=*impl_;std::lock_guard<std::mutex> lock(i.imu_mutex);
  std::string error;
  if(!std::isfinite(s.stamp)||!s.angular_velocity.allFinite()||!s.linear_acceleration.allFinite())error="non-finite IMU sample";
  else if(i.last_input>=0&&s.stamp<=i.last_input)error="IMU timestamp not strictly increasing";
  else if(i.last_input>=0&&s.stamp-i.last_input>i.p.max_imu_gap)error="IMU coverage gap";
  if(!error.empty()){i.imu_error=error;if(reason)*reason=error;return false;}
  i.last_input=s.stamp;i.imu.push_back(s);
  while(i.imu.size()>2&&s.stamp-i.imu.front().stamp>i.p.imu_history_sec)i.imu.pop_front();
  return true;
}
RawFrontendResult StationaryRawFrontend::processScan(const Cloud & input,double stamp,bool instantaneous) {
  auto & i=*impl_; RawFrontendResult out;auto begin=std::chrono::steady_clock::now();
  std::deque<RawImuSample> samples;std::string imu_error;
  {std::lock_guard<std::mutex> lock(i.imu_mutex);samples=i.imu;imu_error=i.imu_error;}
  Eigen::Isometry3d diagnostic_guess=Eigen::Isometry3d::Identity();Cloud diagnostic_target;
  auto finish=[&](const std::string & reason){
    out.reason=reason;out.gravity_initialized=i.initialized;out.gyro_bias=i.bias;out.reference_from_sensor=i.pose;out.reference_from_base=i.pose*i.base_sensor.inverse();out.cumulative_yaw=i.cumulative_yaw;
    if(out.accepted)out.failure_class="none";
    else if(reason.rfind("waiting",0)==0||(!i.initialized&&reason.rfind("IMU calibration",0)==0))out.failure_class="waiting";
    else if(out.failure_class!="numerical_recoverable"){i.hard_fault=true;i.recovery_pending=false;}
    out.recovery_pending=i.recovery_pending;out.recovery_attempts=i.recovery_attempts;out.recovered_scan_count=i.recovered_count;out.submap_size=i.submap.size();
    out.processing_sec=std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count();out.over_budget=out.processing_sec>i.p.diagnostic_budget_sec;
    if(!i.p.diagnostic_directory.empty()) {
      namespace fs=std::filesystem;fs::create_directories(i.p.diagnostic_directory);std::ostringstream name;name<<std::setw(6)<<std::setfill('0')<<i.sequence++;const auto path=fs::path(i.p.diagnostic_directory)/name.str();
      if(fs::exists(path.string()+".json")||fs::exists(path.string()+".pcd"))throw std::runtime_error("refusing to overwrite raw diagnostic scan");
      if(pcl::io::savePCDFileBinary(path.string()+".pcd",input)<0)throw std::runtime_error("raw diagnostic input write failed");if(!diagnostic_target.empty()&&pcl::io::savePCDFileBinary(path.string()+"_target.pcd",diagnostic_target)<0)throw std::runtime_error("raw diagnostic target write failed");
      auto matrix=[](const Eigen::Isometry3d&t){std::vector<double> v;for(int r=0;r<4;++r)for(int c=0;c<4;++c)v.push_back(t.matrix()(r,c));return v;};
      nlohmann::json j={{"schema",1},{"recovery_after",i.recovery_after.load()},{"stamp",stamp},{"instantaneous",instantaneous},{"sensor_from_imu",matrix(i.sensor_imu)},{"base_from_sensor",matrix(i.base_sensor)},{"initial_guess",matrix(diagnostic_guess)},{"output_pose",matrix(i.pose)},{"accepted",out.accepted},{"reason",reason},{"failure_class",out.failure_class},{"input_points",out.input_points},{"finite_points",out.finite_points},{"range_points",out.range_points},{"points",out.points},{"rotation_prior_sigma",out.rotation_prior_sigma},{"rotation_prior_weight",out.rotation_prior_weight},{"calibration_gyro_sample_variance",i.gyro_sample_variance},{"calibration_gyro_bias_variance",i.gyro_bias_variance},{"gyro_bias",{i.bias.x(),i.bias.y(),i.bias.z()}}};
      j["params"]={{"stationary_duration",i.p.stationary_duration},{"stationary_min_samples",i.p.stationary_min_samples},{"leaf_size",i.p.leaf_size},{"max_correspondence",i.p.max_correspondence},{"max_iterations",i.p.max_iterations},{"max_fitness",i.p.max_fitness},{"min_inlier_ratio",i.p.min_inlier_ratio},{"range_filter",i.p.range_filter},{"min_range",i.p.min_range},{"max_range",i.p.max_range},{"deterministic",i.p.deterministic},{"submap_scans",i.p.submap_scans},{"imu_rotation_prior",i.p.imu_rotation_prior},{"gyro_noise_density",i.p.gyro_noise_density},{"rotation_sigma_floor",i.p.rotation_sigma_floor},{"numerical_recovery",i.p.numerical_recovery}};
      j["params"]["stationary_translation_prior"]=i.p.stationary_translation_prior;
      j["params"]["stationary_translation_sigma_floor"]=i.p.stationary_translation_sigma_floor;
      j["translation_prior_sigma"]=out.translation_prior_sigma;j["translation_prior_weight"]=out.translation_prior_weight;
#define SAVE_PARAM(name) j["params"][#name]=i.p.name
      SAVE_PARAM(retry_iterations);SAVE_PARAM(stationary_max_gyro);SAVE_PARAM(stationary_gyro_std);SAVE_PARAM(gravity_magnitude);SAVE_PARAM(gravity_tolerance);SAVE_PARAM(stationary_accel_std);SAVE_PARAM(max_imu_gap);SAVE_PARAM(max_scan_gap);SAVE_PARAM(imu_history_sec);SAVE_PARAM(min_points);SAVE_PARAM(max_rotation_prior_error);SAVE_PARAM(max_base_translation_step);SAVE_PARAM(max_base_linear_speed);SAVE_PARAM(min_geometry_eigen_ratio);SAVE_PARAM(min_information_eigen_ratio);SAVE_PARAM(diagnostic_budget_sec);
#undef SAVE_PARAM
      j["imu"]=nlohmann::json::array();for(const auto&v:samples)j["imu"].push_back({v.stamp,v.angular_velocity.x(),v.angular_velocity.y(),v.angular_velocity.z(),v.linear_acceleration.x(),v.linear_acceleration.y(),v.linear_acceleration.z()});
      j["iterations"]=nlohmann::json::array();for(const auto&r:out.iteration_trace){std::vector<double>d(r.increment.data(),r.increment.data()+6);j["iterations"].push_back({{"attempt",r.attempt},{"iteration",r.iteration},{"inner",r.inner},{"objective",r.objective},{"candidate_objective",r.candidate_objective},{"lambda",r.lambda},{"increment",d},{"accepted",r.accepted}});}std::ofstream file(path.string()+".json");file<<j.dump(2);file.close();if(!file)throw std::runtime_error("raw diagnostic metadata write failed");
    }
    return out;};
  if(i.hard_fault)return finish("frontend latched hard fault; reset required");
  if(i.recovery_pending&&(i.recovery_after.load()<0||stamp<=i.recovery_after.load()))return finish("waiting for next scan acquired after recovery stop");
  if(!instantaneous)return finish("timed raw scans require measured per-point timing");
  if(!std::isfinite(stamp)||(i.previous_stamp>=0&&stamp<=i.previous_stamp))return finish("scan timestamp not strictly increasing");
  if(i.previous_stamp>=0&&stamp-i.previous_stamp>i.p.max_scan_gap)return finish("raw scan coverage gap");
  if(!imu_error.empty())return finish(imu_error);
  if(samples.empty()||samples.back().stamp<stamp||samples.front().stamp>stamp)return finish("waiting for exact scan-stamp IMU coverage");
  if(!i.initialized) {
    std::vector<RawImuSample> window;for(const auto&s:samples)if(s.stamp<=stamp&&s.stamp>=stamp-i.p.stationary_duration-i.p.max_imu_gap)window.push_back(s);
    if(window.size()<i.p.stationary_min_samples||window.back().stamp-window.front().stamp<i.p.stationary_duration)return finish("waiting for stationary IMU calibration");
    Eigen::Vector3d mean_w=Eigen::Vector3d::Zero(),mean_a=Eigen::Vector3d::Zero();for(const auto&s:window){mean_w+=s.angular_velocity;mean_a+=s.linear_acceleration;}mean_w/=window.size();mean_a/=window.size();
    double variance_w=0,variance_a=0,max_w=0;for(const auto&s:window){variance_w+=(s.angular_velocity-mean_w).squaredNorm();variance_a+=(s.linear_acceleration-mean_a).squaredNorm();max_w=std::max(max_w,s.angular_velocity.norm());}
    if(max_w>i.p.stationary_max_gyro||std::sqrt(variance_w/window.size())>i.p.stationary_gyro_std||std::sqrt(variance_a/window.size())>i.p.stationary_accel_std||std::abs(mean_a.norm()-i.p.gravity_magnitude)>i.p.gravity_tolerance)return finish("IMU calibration requires stationary gyro and gravity");
    i.gyro_sample_variance=variance_w/(3.*window.size());i.gyro_bias_variance=i.gyro_sample_variance/window.size();i.calibration_sample_dt=(window.back().stamp-window.front().stamp)/(window.size()-1);
    i.bias=mean_w;i.pose.linear()=Eigen::Quaterniond::FromTwoVectors(i.sensor_imu.rotation()*mean_a,Eigen::Vector3d::UnitZ()).toRotationMatrix();i.calibration_stamp=stamp;i.initialized=true;
  }
  const double start=i.previous_stamp>=0?i.previous_stamp:stamp;
  if(samples.front().stamp>start)return finish("IMU history no longer covers previous scan");
  out.imu_covered=true;out.angular_speed=(Impl::gyroAt(samples,stamp)-i.bias).norm();
  Cloud finite;out.input_points=input.size();for(const auto&p:input)if(std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.z)){++out.finite_points;double range=Eigen::Vector3d(p.x,p.y,p.z).norm();if(!i.p.range_filter||(range>=i.p.min_range&&range<=i.p.max_range))finite.push_back(p);}out.range_points=finite.size();out.filtered_sensor_cloud=finite;
  auto cloud=std::make_shared<Cloud>();pcl::VoxelGrid<pcl::PointXYZ> voxel;voxel.setInputCloud(finite.makeShared());voxel.setLeafSize(i.p.leaf_size,i.p.leaf_size,i.p.leaf_size);voxel.filter(*cloud);if(i.p.deterministic)cloud=deterministicVoxel(finite,i.p.leaf_size);out.points=cloud->size();
  if(cloud->size()<i.p.min_points)return finish("raw scan has insufficient finite geometry");
  Eigen::Vector3d mean=Eigen::Vector3d::Zero();for(const auto&p:*cloud)mean+=Eigen::Vector3d(p.x,p.y,p.z);mean/=cloud->size();Eigen::Matrix3d cov=Eigen::Matrix3d::Zero();for(const auto&p:*cloud){Eigen::Vector3d d=Eigen::Vector3d(p.x,p.y,p.z)-mean;cov+=d*d.transpose();}
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> geometry(cov);out.geometry_eigen_ratio=geometry.eigenvalues().minCoeff()/std::max(1e-12,geometry.eigenvalues().maxCoeff());
  if(out.geometry_eigen_ratio<i.p.min_geometry_eigen_ratio)return finish("raw scan geometry is degenerate");
  if(!i.previous){i.previous=cloud;i.submap.emplace_back(cloud,i.pose);i.previous_stamp=stamp;out.accepted=true;return finish("gravity initialized at first sensor origin");}
  // Integrate measured sample intervals, with interpolated exact endpoints.
  Eigen::Matrix3d delta=Eigen::Matrix3d::Identity();double t=start;Eigen::Vector3d prev_w=i.sensor_imu.rotation()*(Impl::gyroAt(samples,start)-i.bias);
  auto integrate=[&](double next,const Eigen::Vector3d&w){Eigen::Vector3d angle=.5*(prev_w+w)*(next-t);if(angle.norm()>1e-12)delta=delta*Eigen::AngleAxisd(angle.norm(),angle.normalized()).toRotationMatrix();t=next;prev_w=w;};
  for(const auto&s:samples)if(s.stamp>start&&s.stamp<stamp)integrate(s.stamp,i.sensor_imu.rotation()*(s.angular_velocity-i.bias));
  integrate(stamp,i.sensor_imu.rotation()*(Impl::gyroAt(samples,stamp)-i.bias));
  // Base rotation lever arm predicts sensor displacement; registration still estimates all six axes.
  const Eigen::Isometry3d sensor_base=i.base_sensor.inverse();Eigen::Isometry3d guess=Eigen::Isometry3d::Identity();guess.linear()=delta;guess.translation()=sensor_base.translation()-delta*sensor_base.translation();
  diagnostic_guess=guess;
  auto target_cloud=std::make_shared<Cloud>();
  for(const auto & entry:i.submap){Cloud transformed;pcl::transformPointCloud(*entry.first,transformed,(i.pose.inverse()*entry.second).matrix().cast<float>());*target_cloud+=transformed;}
  if(i.p.deterministic)target_cloud=deterministicVoxel(*target_cloud,i.p.leaf_size);diagnostic_target=*target_cloud;
  const int threads=i.p.deterministic?1:2;
  auto target=small_gicp::voxelgrid_sampling_omp<Cloud,pcl::PointCloud<pcl::PointCovariance>>(*target_cloud,i.p.leaf_size,threads);
  auto source=small_gicp::voxelgrid_sampling_omp<Cloud,pcl::PointCloud<pcl::PointCovariance>>(*cloud,i.p.leaf_size,threads);small_gicp::estimate_covariances_omp(*target,20,threads);small_gicp::estimate_covariances_omp(*source,20,threads);
  small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>> tree(target,small_gicp::KdTreeBuilderOMP(threads));small_gicp::Registration<small_gicp::GICPFactor,small_gicp::ParallelReductionOMP,small_gicp::NullFactor,small_gicp::DistanceRejector,RawOptimizer> reg;reg.reduction.num_threads=threads;reg.optimizer.trace=(i.p.record_iterations||!i.p.diagnostic_directory.empty())?&out.iteration_trace:nullptr;reg.optimizer.prior.enabled=i.p.imu_rotation_prior;reg.optimizer.prior.rotation=delta;reg.optimizer.prior.inverse_variance=1./std::max(i.p.rotation_sigma_floor*i.p.rotation_sigma_floor,std::max(i.p.gyro_noise_density*i.p.gyro_noise_density,i.gyro_sample_variance*i.calibration_sample_dt)*(stamp-start)+i.gyro_bias_variance*(stamp-start)*(stamp-start));reg.rejector.max_dist_sq=i.p.max_correspondence*i.p.max_correspondence;const int first_budget=std::min(30,i.p.max_iterations);reg.optimizer.max_iterations=first_budget;
  out.rotation_prior_sigma=std::sqrt(1./reg.optimizer.prior.inverse_variance);out.rotation_prior_weight=i.p.imu_rotation_prior?reg.optimizer.prior.inverse_variance:0.;
  reg.optimizer.base_translation_prior.enabled=i.p.stationary_translation_prior;
  reg.optimizer.base_translation_prior.sensor_base_translation=sensor_base.translation();
  out.translation_prior_sigma=std::max(i.p.stationary_translation_sigma_floor,i.p.max_base_linear_speed*(stamp-start)/3.);
  reg.optimizer.base_translation_prior.inverse_variance=1./(out.translation_prior_sigma*out.translation_prior_sigma);
  out.translation_prior_weight=i.p.stationary_translation_prior?reg.optimizer.base_translation_prior.inverse_variance:0.;
  pcl::KdTreeFLANN<pcl::PointXYZ> quality;quality.setInputCloud(target_cloud);
  auto evaluate=[&](const auto & fit,int attempt_number) {
    RawRegistrationAttempt a;a.attempt=attempt_number;
    // The pinned small_gicp reports the zero-based final outer iteration.
    a.iterations=static_cast<int>(fit.iterations)+1;a.converged=fit.converged;
    a.transform_finite=validRigid(fit.T_target_source)&&fit.H.allFinite();
    if(a.transform_finite) {
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double,6,6>> info(fit.H);
      a.information_eigen_ratio=info.eigenvalues().minCoeff()/std::max(1e-12,info.eigenvalues().maxCoeff());
      a.rotation_prior_error=Eigen::AngleAxisd(delta.transpose()*fit.T_target_source.rotation()).angle();
      const auto new_pose=i.pose*fit.T_target_source;const auto old_base=i.pose*sensor_base,new_base=new_pose*sensor_base;
      a.translation_step=(new_base.translation()-old_base.translation()).norm();a.linear_speed=a.translation_step/(stamp-start);
      std::vector<int> idx(1);std::vector<float> d(1);double sum=0;size_t inliers=0;
      for(const auto & point:*cloud){const Eigen::Vector3d v=fit.T_target_source*Eigen::Vector3d(point.x,point.y,point.z);
        if(quality.nearestKSearch(pcl::PointXYZ(v.x(),v.y(),v.z()),1,idx,d)>0&&d[0]<=reg.rejector.max_dist_sq){sum+=d[0];++inliers;}}
      out.inliers=inliers;a.fitness=inliers?std::sqrt(sum/inliers):1e9;a.inlier_ratio=double(inliers)/cloud->size();
    }
    if(!a.converged)a.rejected_gates.push_back("convergence");
    if(!a.transform_finite)a.rejected_gates.push_back("finite");
    if(!std::isfinite(a.fitness)||a.fitness>i.p.max_fitness)a.rejected_gates.push_back("fitness");
    if(!std::isfinite(a.inlier_ratio)||a.inlier_ratio<i.p.min_inlier_ratio)a.rejected_gates.push_back("inlier_ratio");
    if(!std::isfinite(a.information_eigen_ratio)||a.information_eigen_ratio<i.p.min_information_eigen_ratio)a.rejected_gates.push_back("information_geometry");
    if(!std::isfinite(a.rotation_prior_error)||a.rotation_prior_error>i.p.max_rotation_prior_error)a.rejected_gates.push_back("rotation_prior");
    if(!std::isfinite(a.translation_step)||a.translation_step>i.p.max_base_translation_step)a.rejected_gates.push_back("translation_step");
    if(!std::isfinite(a.linear_speed)||a.linear_speed>i.p.max_base_linear_speed)a.rejected_gates.push_back("linear_speed");
    return a;
  };
  auto fit=reg.align(*target,*source,tree,guess);auto attempt=evaluate(fit,1);
  out.first_attempt_iterations=attempt.iterations;out.iterations=attempt.iterations;out.registration_attempts.push_back(attempt);
  const int extra=std::min(i.p.retry_iterations,std::max(0,30-first_budget));
  if(extra>0&&rawRegistrationRetryEligible(attempt,i.p)) {
    reg.optimizer.max_iterations=extra;reg.optimizer.attempt=2;fit=reg.align(*target,*source,tree,fit.T_target_source);
    attempt=evaluate(fit,2);out.retry_count=1;out.iterations+=attempt.iterations;out.registration_attempts.push_back(attempt);
  }
  out.converged=attempt.converged;out.transform_finite=attempt.transform_finite;out.fitness=attempt.fitness;out.inlier_ratio=attempt.inlier_ratio;
  out.information_eigen_ratio=attempt.information_eigen_ratio;out.rotation_prior_error=attempt.rotation_prior_error;
  out.translation_step=attempt.translation_step;out.linear_speed=attempt.linear_speed;out.rejected_gates=attempt.rejected_gates;
  if(!attempt.rejected_gates.empty()) {
    if(i.p.numerical_recovery&&!i.recovery_pending&&i.recovery_attempts==0&&attempt.rejected_gates==std::vector<std::string>{"convergence"}){i.recovery_pending=true;++i.recovery_attempts;out.failure_class="numerical_recoverable";return finish("numerical convergence failure; one following scan may recover");}
    i.recovery_pending=false;
    if(!out.converged||!out.transform_finite||out.fitness>i.p.max_fitness||out.inlier_ratio<i.p.min_inlier_ratio||out.information_eigen_ratio<i.p.min_information_eigen_ratio)
      return finish("local GICP failed convergence, geometry or quality gates");
    if(out.rotation_prior_error>i.p.max_rotation_prior_error)return finish("local GICP rotation disagrees with integrated IMU");
    return finish("local GICP base translation exceeds stationary-motion gate");
  }
  const auto new_pose=i.pose*fit.T_target_source;
  const Eigen::Matrix3d before=i.pose.rotation(),after=before*delta;double yaw_step=std::atan2(after(1,0),after(0,0))-std::atan2(before(1,0),before(0,0));i.cumulative_yaw+=std::atan2(std::sin(yaw_step),std::cos(yaw_step));
  i.pose=new_pose;i.previous=cloud;i.submap.emplace_back(cloud,i.pose);while(i.submap.size()>i.p.submap_scans)i.submap.pop_front();i.previous_stamp=stamp;if(i.recovery_pending){++i.recovered_count;i.recovery_pending=false;}out.accepted=true;return finish("full SE(3) local scan registration accepted");
}
}
