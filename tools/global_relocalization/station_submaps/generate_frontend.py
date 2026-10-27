#!/usr/bin/env python3
"""Generate an isolated class from the exact frozen frontend; never edits src/."""
from pathlib import Path
import argparse,hashlib
p=argparse.ArgumentParser();p.add_argument('root',type=Path);p.add_argument('output',type=Path);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True);root=a.root;source=root/'src/global_relocalization/src/stationary_raw_frontend.cpp';original=source.read_text();old='StationaryRawFrontend';new='ExperimentalCalibratedFrontend';header=(root/'src/global_relocalization/include/global_relocalization/stationary_raw_frontend.hpp').read_text();body=header[header.index('class '+old+' {'):header.index('}  // namespace global_relocalization')].replace(old,new)
extra='''
  TrustedCalibration exportCalibration(const std::string & source_sha256) const;
  void restoreLocalMap(const TrustedCalibration &, const Eigen::Matrix3d & reference_from_sensor_at_first_scan, double first_scan_stamp);
''';body=body.replace('private:\n  struct Impl;',extra+'private:\n  struct Impl;')
prefix='''#pragma once
#include "global_relocalization/stationary_raw_frontend.hpp"
#include <string>
namespace global_relocalization {
class ExperimentalCalibratedFrontend;
class TrustedCalibration {
  friend class ExperimentalCalibratedFrontend;
  TrustedCalibration()=default;
  std::string parameter_signature_,source_sha256_;
  Eigen::Isometry3d sensor_imu_,base_sensor_;
  Eigen::Vector3d bias_;
  Eigen::Matrix3d gravity_alignment_;
  double stamp_,sample_variance_,bias_variance_,sample_dt_;
public:
  TrustedCalibration(const TrustedCalibration &)=default;
  double stamp() const {return stamp_;}
  const Eigen::Vector3d & bias() const {return bias_;}
  const Eigen::Matrix3d & gravityAlignment() const {return gravity_alignment_;}
  double sampleVariance() const {return sample_variance_;}
  double biasVariance() const {return bias_variance_;}
  const std::string & sourceSHA256() const {return source_sha256_;}
};
''';(a.output/'experimental_frontend.hpp').write_text(prefix+body+'}\n')
fields=['range_filter','deterministic','imu_rotation_prior','numerical_recovery','stationary_translation_prior','stationary_translation_sigma_floor','submap_scans','min_range','max_range','gyro_noise_density','rotation_sigma_floor','stationary_duration','stationary_min_samples','stationary_max_gyro','stationary_gyro_std','gravity_magnitude','gravity_tolerance','stationary_accel_std','max_imu_gap','max_scan_gap','imu_history_sec','leaf_size','max_correspondence','max_iterations','retry_iterations','max_fitness','min_inlier_ratio','min_points','max_rotation_prior_error','max_base_translation_step','max_base_linear_speed','min_geometry_eigen_ratio','min_information_eigen_ratio','diagnostic_budget_sec'];signature='namespace { std::string parameterSignature(const RawFrontendParams & p) {return nlohmann::json{'+','.join('{"'+f+'",p.'+f+'}'for f in fields)+'}.dump();} }\n'
methods='''
TrustedCalibration ExperimentalCalibratedFrontend::exportCalibration(const std::string & source_sha256) const {
 auto & i=*impl_;std::lock_guard<std::mutex> lock(i.imu_mutex);
 if(!i.initialized||!i.previous||i.hard_fault||i.recovery_pending||i.previous_stamp!=i.calibration_stamp)
  throw std::runtime_error("trusted snapshot requires immediately accepted first calibration scan");
 if(source_sha256.size()!=64||source_sha256.find_first_not_of("0123456789abcdef")!=std::string::npos)
  throw std::invalid_argument("calibration source must have an explicit SHA256");
 TrustedCalibration s;s.parameter_signature_=parameterSignature(i.p);s.source_sha256_=source_sha256;
 s.sensor_imu_=i.sensor_imu;s.base_sensor_=i.base_sensor;s.bias_=i.bias;s.gravity_alignment_=i.pose.rotation();
 s.stamp_=i.calibration_stamp;s.sample_variance_=i.gyro_sample_variance;s.bias_variance_=i.gyro_bias_variance;s.sample_dt_=i.calibration_sample_dt;return s;
}
void ExperimentalCalibratedFrontend::restoreLocalMap(const TrustedCalibration & s,const Eigen::Matrix3d & rotation,double first_stamp) {
 auto & i=*impl_;
 if(s.parameter_signature_!=parameterSignature(i.p)||!s.sensor_imu_.matrix().isApprox(i.sensor_imu.matrix(),1e-12)||!s.base_sensor_.matrix().isApprox(i.base_sensor.matrix(),1e-12))
  throw std::invalid_argument("calibration parameters or fixed extrinsics differ");
 Eigen::Isometry3d initial=Eigen::Isometry3d::Identity();initial.linear()=rotation;
 if(!validRigid(initial)||!std::isfinite(first_stamp)||first_stamp<s.stamp_||!s.bias_.allFinite())throw std::invalid_argument("invalid restored gauge/stamp");
 reset();std::lock_guard<std::mutex> lock(i.imu_mutex);
 i.initialized=true;i.bias=s.bias_;i.gyro_sample_variance=s.sample_variance_;i.gyro_bias_variance=s.bias_variance_;i.calibration_sample_dt=s.sample_dt_;i.calibration_stamp=s.stamp_;i.pose=initial;
}
'''
result=original.replace('"global_relocalization/stationary_raw_frontend.hpp"','"experimental_frontend.hpp"').replace(old,new);at=result.rfind('\n}');result=result[:at]+'\n'+signature+methods+result[at:];(a.output/'experimental_frontend.cpp').write_text('// Generated isolated experimental class; original SHA256 '+hashlib.sha256(source.read_bytes()).hexdigest()+'\n'+result)
