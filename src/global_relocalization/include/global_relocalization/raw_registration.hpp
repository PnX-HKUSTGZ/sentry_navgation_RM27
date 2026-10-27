// Copyright 2026 RM27 contributors. SPDX-License-Identifier: Apache-2.0
#pragma once
#include "global_relocalization/stationary_raw_frontend.hpp"
#include <small_gicp/util/lie.hpp>
#include <small_gicp/registration/registration_result.hpp>
namespace global_relocalization {
struct RawRotationPrior {
  bool enabled{false}; double inverse_variance{0};
  Eigen::Matrix3d rotation{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d residual(const Eigen::Isometry3d & t) const {
    Eigen::AngleAxisd a(rotation.transpose()*t.rotation());return a.angle()*a.axis();
  }
  Eigen::Matrix3d jacobian(const Eigen::Isometry3d & t) const {
    // Right SO(3) increments, matching small_gicp's SE(3) convention.
    Eigen::Matrix3d j;constexpr double eps=1e-6;
    for(int k=0;k<3;++k){auto a=t,b=t;a.linear()=t.rotation()*Eigen::AngleAxisd(eps,Eigen::Vector3d::Unit(k)).toRotationMatrix();b.linear()=t.rotation()*Eigen::AngleAxisd(-eps,Eigen::Vector3d::Unit(k)).toRotationMatrix();j.col(k)=(residual(a)-residual(b))/(2*eps);}return j;
  }
  double cost(const Eigen::Isometry3d&t)const{return enabled?0.5*inverse_variance*residual(t).squaredNorm():0.;}
  void linearize(const Eigen::Isometry3d&t,Eigen::Matrix<double,6,6>&h,Eigen::Matrix<double,6,1>&b,double&e)const {
    if(!enabled)return;const auto r=residual(t);const auto j=jacobian(t);h.topLeftCorner<3,3>()+=inverse_variance*j.transpose()*j;b.head<3>()+=inverse_variance*j.transpose()*r;e+=cost(t);
  }
};
struct RawBaseTranslationPrior {
  bool enabled{false};double inverse_variance{0};
  Eigen::Vector3d sensor_base_translation{Eigen::Vector3d::Zero()};
  Eigen::Vector3d residual(const Eigen::Isometry3d & t) const {
    return t.translation()+t.rotation()*sensor_base_translation-sensor_base_translation;
  }
  Eigen::Matrix<double,3,6> jacobian(const Eigen::Isometry3d & t) const {
    Eigen::Matrix<double,3,6> j;
    j.leftCols<3>()=-t.rotation()*small_gicp::skew(sensor_base_translation);
    j.rightCols<3>()=t.rotation();return j;
  }
  double cost(const Eigen::Isometry3d & t) const {return enabled?.5*inverse_variance*residual(t).squaredNorm():0.;}
  void linearize(const Eigen::Isometry3d & t,Eigen::Matrix<double,6,6> & h,Eigen::Matrix<double,6,1> & b,double & e) const {
    if(!enabled)return;const auto r=residual(t);const auto j=jacobian(t);
    h+=inverse_variance*j.transpose()*j;b+=inverse_variance*j.transpose()*r;e+=cost(t);
  }
};
// Adapted from small_gicp's MIT LM optimizer (Kenji Koide, 2024).
// The candidate prior belongs in new_e; never mutate the old objective during
// trial evaluation. Keep result.H LiDAR-only for independent observability gates.
struct RawOptimizer {
  int max_iterations{15},max_inner_iterations{10};
  RawRotationPrior prior;RawBaseTranslationPrior base_translation_prior;std::vector<RawIteration>*trace{nullptr};int attempt{1};
  template<class T,class S,class Tree,class Reject,class Criteria,class Reduction,class Factor,class General>
  small_gicp::RegistrationResult optimize(const T&target,const S&source,const Tree&tree,const Reject&reject,const Criteria&criteria,Reduction&reduction,const Eigen::Isometry3d&initial,std::vector<Factor>&factors,General&)const {
    small_gicp::RegistrationResult result(initial);double lambda=1e-3;
    for(int iteration=0;iteration<max_iterations&&!result.converged;++iteration){
      auto [h,b,e]=reduction.linearize(target,source,tree,reject,result.T_target_source,factors);
      result.H=h;result.b=b;prior.linearize(result.T_target_source,h,b,e);base_translation_prior.linearize(result.T_target_source,h,b,e);bool success=false;
      for(int inner=0;inner<max_inner_iterations;++inner){
        Eigen::Matrix<double,6,1> d=(h+lambda*Eigen::Matrix<double,6,6>::Identity()).ldlt().solve(-b);
        const Eigen::Isometry3d candidate=result.T_target_source*small_gicp::se3_exp(d);
        const double candidate_e=reduction.error(target,source,candidate,factors)+prior.cost(candidate)+base_translation_prior.cost(candidate);
        bool accept=d.allFinite()&&candidate.matrix().allFinite()&&std::isfinite(candidate_e)&&candidate_e<=e;
        if(trace){RawIteration row;row.attempt=attempt;row.iteration=iteration;row.inner=inner;row.objective=e;row.candidate_objective=candidate_e;row.lambda=lambda;row.increment=d;row.rotation_increment=d.head<3>().norm();row.translation_increment=d.tail<3>().norm();row.accepted=accept;trace->push_back(row);}
        if(accept){result.converged=criteria.converged(d);result.T_target_source=candidate;lambda/=10.;success=true;e=candidate_e;break;}lambda*=10.;
      }
      result.iterations=iteration;result.error=e;if(!success)break;
    }
    result.num_inliers=std::count_if(factors.begin(),factors.end(),[](const auto&f){return f.inlier();});return result;
  }
};
}
