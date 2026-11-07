// Copyright 2026 RM27 contributors. SPDX-License-Identifier: Apache-2.0
#pragma once
#include <algorithm>
#include <cmath>
#include <small_gicp/registration/optimizer.hpp>
#include <small_gicp/util/lie.hpp>

namespace global_relocalization {
// The query reference is gravity aligned, while map_up is explicitly declared
// in map coordinates. This constrains two rotation axes, never yaw or XYZ.
struct GravityDirectionPrior {
  bool enabled{false};double inverse_variance{0.};
  Eigen::Vector3d map_up{Eigen::Vector3d::UnitZ()};
  Eigen::Vector3d residual(const Eigen::Isometry3d & t) const {return t.rotation()*Eigen::Vector3d::UnitZ()-map_up;}
  Eigen::Matrix<double,3,6> jacobian(const Eigen::Isometry3d & t) const {
    Eigen::Matrix<double,3,6> j=Eigen::Matrix<double,3,6>::Zero();
    j.leftCols<3>()=-t.rotation()*small_gicp::skew(Eigen::Vector3d::UnitZ());return j;
  }
  double cost(const Eigen::Isometry3d & t) const {return enabled?.5*inverse_variance*residual(t).squaredNorm():0.;}
  void linearize(const Eigen::Isometry3d & t,Eigen::Matrix<double,6,6> & h,Eigen::Matrix<double,6,1> & b,double & e) const {
    if(!enabled)return;const auto r=residual(t);const auto j=jacobian(t);
    h+=inverse_variance*j.transpose()*j;b+=inverse_variance*j.transpose()*r;e+=cost(t);
  }
};
// Preserve the pinned optimizer exactly when disabled (including all global
// preparation fits). The active branch evaluates candidate prior cost in the
// candidate objective; the pinned GeneralFactor hook mutates the old cost.
struct GravityConstrainedOptimizer : small_gicp::LevenbergMarquardtOptimizer {
  GravityDirectionPrior gravity;
  template<class T,class S,class Tree,class Reject,class Criteria,class Reduction,class Factor,class General>
  small_gicp::RegistrationResult optimize(const T & target,const S & source,const Tree & tree,const Reject & reject,const Criteria & criteria,Reduction & reduction,const Eigen::Isometry3d & initial,std::vector<Factor> & factors,General & general) const {
    if(!gravity.enabled)return small_gicp::LevenbergMarquardtOptimizer::optimize(target,source,tree,reject,criteria,reduction,initial,factors,general);
    small_gicp::RegistrationResult result(initial);double lambda=init_lambda;
    for(int iteration=0;iteration<max_iterations&&!result.converged;++iteration) {
      auto [h,b,e]=reduction.linearize(target,source,tree,reject,result.T_target_source,factors);
      result.H=h;result.b=b;general.update_linearized_system(target,source,tree,result.T_target_source,&h,&b,&e);
      gravity.linearize(result.T_target_source,h,b,e);bool success=false;
      for(int inner=0;inner<max_inner_iterations;++inner) {
        const Eigen::Matrix<double,6,1> d=(h+lambda*Eigen::Matrix<double,6,6>::Identity()).ldlt().solve(-b);
        const auto candidate=result.T_target_source*small_gicp::se3_exp(d);
        double candidate_e=reduction.error(target,source,candidate,factors)+gravity.cost(candidate);
        general.update_error(target,source,candidate,&candidate_e);
        if(d.allFinite()&&candidate.matrix().allFinite()&&std::isfinite(candidate_e)&&candidate_e<=e) {
          result.converged=criteria.converged(d);result.T_target_source=candidate;lambda/=lambda_factor;e=candidate_e;success=true;break;
        }
        lambda*=lambda_factor;
      }
      result.iterations=iteration;result.error=e;if(!success)break;
    }
    result.num_inliers=std::count_if(factors.begin(),factors.end(),[](const auto & factor){return factor.inlier();});return result;
  }
};
}
