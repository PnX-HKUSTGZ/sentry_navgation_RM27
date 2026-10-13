// Copyright 2026 RM27 contributors. SPDX-License-Identifier: Apache-2.0
#include "global_relocalization/stationary_raw_frontend.hpp"
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <small_gicp/pcl/pcl_registration.hpp>
#include <small_gicp/pcl/pcl_point.hpp>
#include <small_gicp/util/downsampling_omp.hpp>
#include <Eigen/Eigenvalues>
#include <chrono>
#include <deque>
#include <mutex>

namespace global_relocalization {
struct StationaryRawFrontend::Impl {
  RawFrontendParams p; Eigen::Isometry3d sensor_imu, base_sensor;
  std::mutex imu_mutex; std::deque<RawImuSample> imu; std::string imu_error;
  double last_input{-1}, previous_stamp{-1}, calibration_stamp{-1}, cumulative_yaw{0};
  bool initialized{false}; Eigen::Vector3d bias{Eigen::Vector3d::Zero()};
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
  if(p.stationary_duration<=0||p.stationary_min_samples<2||p.leaf_size<=0||p.max_imu_gap<=0||p.max_scan_gap<=0||p.max_iterations<1||!si.matrix().allFinite()||!bs.matrix().allFinite())throw std::invalid_argument("invalid raw frontend parameters/extrinsics");
}
StationaryRawFrontend::~StationaryRawFrontend()=default;
void StationaryRawFrontend::reset(){auto & i=*impl_;std::lock_guard<std::mutex> lock(i.imu_mutex);i.imu.clear();i.imu_error.clear();i.last_input=i.previous_stamp=i.calibration_stamp=-1;i.initialized=false;i.bias.setZero();i.pose.setIdentity();i.previous.reset();i.cumulative_yaw=0;}
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
  auto finish=[&](const std::string & reason){out.reason=reason;out.gravity_initialized=i.initialized;out.gyro_bias=i.bias;out.reference_from_sensor=i.pose;out.reference_from_base=i.pose*i.base_sensor.inverse();out.cumulative_yaw=i.cumulative_yaw;out.processing_sec=std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count();out.over_budget=out.processing_sec>i.p.diagnostic_budget_sec;return out;};
  if(!instantaneous)return finish("timed raw scans require measured per-point timing");
  if(!std::isfinite(stamp)||(i.previous_stamp>=0&&stamp<=i.previous_stamp))return finish("scan timestamp not strictly increasing");
  if(i.previous_stamp>=0&&stamp-i.previous_stamp>i.p.max_scan_gap)return finish("raw scan coverage gap");
  std::deque<RawImuSample> samples;std::string imu_error;
  {std::lock_guard<std::mutex> lock(i.imu_mutex);samples=i.imu;imu_error=i.imu_error;}
  if(!imu_error.empty())return finish(imu_error);
  if(samples.empty()||samples.back().stamp<stamp||samples.front().stamp>stamp)return finish("waiting for exact scan-stamp IMU coverage");
  if(!i.initialized) {
    std::vector<RawImuSample> window;for(const auto&s:samples)if(s.stamp<=stamp&&s.stamp>=stamp-i.p.stationary_duration-i.p.max_imu_gap)window.push_back(s);
    if(window.size()<i.p.stationary_min_samples||window.back().stamp-window.front().stamp<i.p.stationary_duration)return finish("waiting for stationary IMU calibration");
    Eigen::Vector3d mean_w=Eigen::Vector3d::Zero(),mean_a=Eigen::Vector3d::Zero();for(const auto&s:window){mean_w+=s.angular_velocity;mean_a+=s.linear_acceleration;}mean_w/=window.size();mean_a/=window.size();
    double variance_w=0,variance_a=0,max_w=0;for(const auto&s:window){variance_w+=(s.angular_velocity-mean_w).squaredNorm();variance_a+=(s.linear_acceleration-mean_a).squaredNorm();max_w=std::max(max_w,s.angular_velocity.norm());}
    if(max_w>i.p.stationary_max_gyro||std::sqrt(variance_w/window.size())>i.p.stationary_gyro_std||std::sqrt(variance_a/window.size())>i.p.stationary_accel_std||std::abs(mean_a.norm()-i.p.gravity_magnitude)>i.p.gravity_tolerance)return finish("IMU calibration requires stationary gyro and gravity");
    i.bias=mean_w;i.pose.linear()=Eigen::Quaterniond::FromTwoVectors(i.sensor_imu.rotation()*mean_a,Eigen::Vector3d::UnitZ()).toRotationMatrix();i.calibration_stamp=stamp;i.initialized=true;
  }
  const double start=i.previous_stamp>=0?i.previous_stamp:stamp;
  if(samples.front().stamp>start)return finish("IMU history no longer covers previous scan");
  out.imu_covered=true;out.angular_speed=(Impl::gyroAt(samples,stamp)-i.bias).norm();
  Cloud finite;for(const auto&p:input)if(std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.z))finite.push_back(p);
  auto cloud=std::make_shared<Cloud>();pcl::VoxelGrid<pcl::PointXYZ> voxel;voxel.setInputCloud(finite.makeShared());voxel.setLeafSize(i.p.leaf_size,i.p.leaf_size,i.p.leaf_size);voxel.filter(*cloud);out.points=cloud->size();
  if(cloud->size()<i.p.min_points)return finish("raw scan has insufficient finite geometry");
  Eigen::Vector3d mean=Eigen::Vector3d::Zero();for(const auto&p:*cloud)mean+=Eigen::Vector3d(p.x,p.y,p.z);mean/=cloud->size();Eigen::Matrix3d cov=Eigen::Matrix3d::Zero();for(const auto&p:*cloud){Eigen::Vector3d d=Eigen::Vector3d(p.x,p.y,p.z)-mean;cov+=d*d.transpose();}
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> geometry(cov);out.geometry_eigen_ratio=geometry.eigenvalues().minCoeff()/std::max(1e-12,geometry.eigenvalues().maxCoeff());
  if(out.geometry_eigen_ratio<i.p.min_geometry_eigen_ratio)return finish("raw scan geometry is degenerate");
  if(!i.previous){i.previous=cloud;i.previous_stamp=stamp;out.accepted=true;return finish("gravity initialized at first sensor origin");}
  // Integrate measured sample intervals, with interpolated exact endpoints.
  Eigen::Matrix3d delta=Eigen::Matrix3d::Identity();double t=start;Eigen::Vector3d prev_w=i.sensor_imu.rotation()*(Impl::gyroAt(samples,start)-i.bias);
  auto integrate=[&](double next,const Eigen::Vector3d&w){Eigen::Vector3d angle=.5*(prev_w+w)*(next-t);if(angle.norm()>1e-12)delta=delta*Eigen::AngleAxisd(angle.norm(),angle.normalized()).toRotationMatrix();t=next;prev_w=w;};
  for(const auto&s:samples)if(s.stamp>start&&s.stamp<stamp)integrate(s.stamp,i.sensor_imu.rotation()*(s.angular_velocity-i.bias));
  integrate(stamp,i.sensor_imu.rotation()*(Impl::gyroAt(samples,stamp)-i.bias));
  // Base rotation lever arm predicts sensor displacement; registration still estimates all six axes.
  const Eigen::Isometry3d sensor_base=i.base_sensor.inverse();Eigen::Isometry3d guess=Eigen::Isometry3d::Identity();guess.linear()=delta;guess.translation()=sensor_base.translation()-delta*sensor_base.translation();
  auto target=small_gicp::voxelgrid_sampling_omp<Cloud,pcl::PointCloud<pcl::PointCovariance>>(*i.previous,i.p.leaf_size,2);
  auto source=small_gicp::voxelgrid_sampling_omp<Cloud,pcl::PointCloud<pcl::PointCovariance>>(*cloud,i.p.leaf_size,2);small_gicp::estimate_covariances_omp(*target,20,2);small_gicp::estimate_covariances_omp(*source,20,2);
  small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>> tree(target,small_gicp::KdTreeBuilderOMP(2));small_gicp::Registration<small_gicp::GICPFactor,small_gicp::ParallelReductionOMP> reg;reg.reduction.num_threads=2;reg.rejector.max_dist_sq=i.p.max_correspondence*i.p.max_correspondence;reg.optimizer.max_iterations=i.p.max_iterations;
  const auto fit=reg.align(*target,*source,tree,guess);out.converged=fit.converged;
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double,6,6>> info(fit.H);out.information_eigen_ratio=info.eigenvalues().minCoeff()/std::max(1e-12,info.eigenvalues().maxCoeff());
  out.rotation_prior_error=Eigen::AngleAxisd(delta.transpose()*fit.T_target_source.rotation()).angle();
  const auto new_pose=i.pose*fit.T_target_source;const auto old_base=i.pose*sensor_base,new_base=new_pose*sensor_base;out.translation_step=(new_base.translation()-old_base.translation()).norm();out.linear_speed=out.translation_step/(stamp-start);
  pcl::KdTreeFLANN<pcl::PointXYZ> quality;quality.setInputCloud(i.previous);std::vector<int> idx(1);std::vector<float> d(1);double sum=0;
  for(const auto&p:*cloud){Eigen::Vector3d v=fit.T_target_source*Eigen::Vector3d(p.x,p.y,p.z);if(quality.nearestKSearch(pcl::PointXYZ(v.x(),v.y(),v.z()),1,idx,d)>0&&d[0]<=reg.rejector.max_dist_sq){sum+=d[0];++out.inliers;}}
  out.fitness=out.inliers?std::sqrt(sum/out.inliers):1e9;out.inlier_ratio=double(out.inliers)/cloud->size();
  if(!fit.converged||!fit.T_target_source.matrix().allFinite()||out.fitness>i.p.max_fitness||out.inlier_ratio<i.p.min_inlier_ratio||out.information_eigen_ratio<i.p.min_information_eigen_ratio)return finish("local GICP failed convergence, geometry or quality gates");
  if(out.rotation_prior_error>i.p.max_rotation_prior_error)return finish("local GICP rotation disagrees with integrated IMU");
  if(out.translation_step>i.p.max_base_translation_step||out.linear_speed>i.p.max_base_linear_speed)return finish("local GICP base translation exceeds stationary-motion gate");
  const Eigen::Matrix3d before=i.pose.rotation(),after=before*delta;double yaw_step=std::atan2(after(1,0),after(0,0))-std::atan2(before(1,0),before(0,0));i.cumulative_yaw+=std::atan2(std::sin(yaw_step),std::cos(yaw_step));
  i.pose=new_pose;i.previous=cloud;i.previous_stamp=stamp;out.accepted=true;return finish("full SE(3) local scan registration accepted");
}
}
