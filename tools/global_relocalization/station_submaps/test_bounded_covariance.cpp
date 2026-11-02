// Physical/numerical contracts for the isolated covariance observation model.
#include "bounded_covariance.hpp"
#include "global_relocalization/raw_registration.hpp"
#include <small_gicp/points/point_cloud.hpp>
#include <small_gicp/ann/kdtree_omp.hpp>
#include <small_gicp/factors/gicp_factor.hpp>
#include <small_gicp/registration/reduction_omp.hpp>
#include <small_gicp/registration/rejector.hpp>
#include <small_gicp/registration/termination_criteria.hpp>
#include <iostream>
using Cloud=small_gicp::PointCloud;
Cloud points(std::initializer_list<Eigen::Vector3d> p){return Cloud(std::vector<Eigen::Vector3d>(p));}
struct BoundaryTree {
  size_t knn_search(const Eigen::Vector4d &,size_t,size_t*ids,double*dd) const {
    ids[0]=7;dd[0]=.375*.375;
    ids[1]=8;dd[1]=std::nextafter(.375*.375,std::numeric_limits<double>::infinity());
    ids[2]=9;dd[2]=0.;return 3;
  }
};
int main(){
  BoundaryTree fake;station_diagnostic::RadiusBoundedKnn<BoundaryTree> bound(fake,.375);
  size_t ids[20];double dd[20];auto n=bound.knn_search(Eigen::Vector4d::Zero(),20,ids,dd);
  if(n!=2||ids[0]!=7||ids[1]!=9||dd[0]!=.375*.375)return 1;
  bool bad=false;try{station_diagnostic::RadiusBoundedKnn<BoundaryTree> invalid(fake,0.);}catch(const std::invalid_argument &){bad=true;}if(!bad)return 2;
  auto sparse=points({{0,0,0},{.1,0,0},{0,.1,0},{2,0,0},{2,.1,0},{2,0,.1}});auto old_sparse=sparse;
  small_gicp::estimate_covariances_omp(old_sparse,20,1);
  station_diagnostic::estimate_bounded_covariances(sparse,.36,20,1);
  Eigen::Matrix4d identity=Eigen::Matrix4d::Identity();identity(3,3)=0.;
  for(size_t i=0;i<sparse.size();++i)if(sparse.cov(i)!=identity||sparse.point(i)!=old_sparse.point(i))return 3;
  // Inverse precision is never increased by stock invalid-I fallback relative
  // to any stock planar covariance (eigenvalues .001,1,1).
  for(size_t i=0;i<sparse.size();++i){Eigen::Matrix3d difference=old_sparse.cov(i).topLeftCorner<3,3>().inverse()-Eigen::Matrix3d::Identity();if(Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d>(difference).eigenvalues().minCoeff() < -1e-9)return 4;}
  std::vector<Eigen::Vector3d> dense_points;
  for(int x=-2;x<=2;++x)for(int y=-2;y<=2;++y)for(int z=-2;z<=2;++z)dense_points.emplace_back(.025*x,.025*y,.025*z);
  auto dense=std::make_shared<Cloud>(dense_points);auto stock=*dense;
  small_gicp::UnsafeKdTree<Cloud> registration_tree(*dense);size_t before_id;double before_distance;
  const Eigen::Vector4d distant(.8,.3,.2,1.);registration_tree.knn_search(distant,1,&before_id,&before_distance);
  small_gicp::estimate_covariances_omp(stock,20,1);
  station_diagnostic::estimate_bounded_covariances(*dense,.36,20,1);
  for(size_t i=0;i<dense->size();++i)if(dense->cov(i)!=stock.cov(i)||dense->point(i)!=stock.point(i))return 5;
  size_t after_id;double after_distance;registration_tree.knn_search(distant,1,&after_id,&after_distance);
  if(before_id!=after_id||before_distance!=after_distance||after_distance<=.36*.36)return 6;
  small_gicp::KdTree<Cloud> target_tree(dense,small_gicp::KdTreeBuilderOMP(1));
  small_gicp::ParallelReductionOMP reduction;reduction.num_threads=1;
  small_gicp::DistanceRejector reject;reject.max_dist_sq=.3*.3;
  Eigen::Isometry3d initial=Eigen::Isometry3d::Identity();initial.translation()<<.005,.004,.003;
  std::vector<small_gicp::GICPFactor> independent(dense->size());
  auto [expected_h,expected_b,expected_e]=reduction.linearize(*dense,*dense,target_tree,reject,initial,independent);
  global_relocalization::RawOptimizer optimizer;optimizer.max_iterations=1;
  optimizer.prior.enabled=true;optimizer.prior.inverse_variance=40000.;
  optimizer.base_translation_prior.enabled=true;optimizer.base_translation_prior.inverse_variance=7000.;
  std::vector<small_gicp::GICPFactor> factors(dense->size());int ignored=0;
  auto fit=optimizer.optimize(*dense,*dense,target_tree,reject,small_gicp::TerminationCriteria(),reduction,initial,factors,ignored);
  if((fit.H-expected_h).cwiseAbs().maxCoeff()>1e-12||
      (fit.b-expected_b).cwiseAbs().maxCoeff()>1e-12)return 7;
  std::cout<<"bounded covariance contracts passed: inclusive radius, stock low-precision fallback, dense exact equality, unchanged registration NN, LiDAR-only H/b\n";
  return 0;
}
