// Experimental observation model only. Production covariance remains unchanged.
#pragma once
#include <cmath>
#include <stdexcept>
#include <small_gicp/util/normal_estimation_omp.hpp>

namespace station_diagnostic {
// Filter the stock sorted KNN result without changing its order/arithmetic.
// This adaptor is private to covariance estimation, never registration NN.
template<class Tree> class RadiusBoundedKnn {
  const Tree & tree_;
  const double radius_sq_;
public:
  RadiusBoundedKnn(const Tree & tree,double radius):tree_(tree),radius_sq_(radius*radius) {
    if(!std::isfinite(radius)||radius<=0)throw std::invalid_argument("positive finite covariance radius required");
  }
  size_t knn_search(const Eigen::Vector4d & query,size_t k,size_t * indices,double * sq_distances) const {
    const size_t n=tree_.knn_search(query,k,indices,sq_distances);
    size_t count=0;
    for(size_t i=0;i<n;++i)if(sq_distances[i]<=radius_sq_) {
      indices[count]=indices[i];sq_distances[count]=sq_distances[i];++count;
    }
    return count;
  }
};

template<class PointCloud>
void estimate_bounded_covariances(PointCloud & cloud,double radius,int neighbors=20,int threads=1) {
  small_gicp::UnsafeKdTree<PointCloud> original_tree(cloud);
  RadiusBoundedKnn<decltype(original_tree)> bounded(original_tree,radius);
  // Stock algorithm: <5 -> I(3D), zero homogeneous padding; >=5 -> unchanged
  // PCA and (1e-3,1,1) regularization. No point or correspondence is removed.
  small_gicp::estimate_covariances_omp(cloud,bounded,neighbors,threads);
}
}  // namespace station_diagnostic
