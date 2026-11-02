// Exercise the actual patched ESEKF branches against the innovation-form reference.
#include "IKFoM/IKFoM_toolkit/esekfom/esekfom.hpp"
#include <iostream>
#include <stdexcept>
using V24 = MTK::vect<24, double>;
using V30 = MTK::vect<30, double>;
MTK_BUILD_MANIFOLD(State24, ((V24, value)));
MTK_BUILD_MANIFOLD(State30, ((V30, value)));
Eigen::MatrixXd jacobian;
Eigen::VectorXd residual;
double variance;
template<class State>
void observation(State &, Eigen::Matrix3d, Eigen::Matrix3d,
                 esekfom::dyn_share_modified<double> & data) {
  data.valid = true;
  data.h_x = jacobian;
  data.z = residual;
  data.M_Noise = variance;
}
template<class State>
void check(int rows, double noise) {
  constexpr int n = State::DOF;
  using Filter = esekfom::esekf<State, n>;
  typename Filter::cov prior;
  Eigen::MatrixXd a = Eigen::MatrixXd::Random(n,n);
  prior = a*a.transpose() + .1*Eigen::MatrixXd::Identity(n,n);
  jacobian = Eigen::MatrixXd::Random(rows,12);
  jacobian.rightCols(6).setZero();  // Include rank-deficient geometry.
  residual = Eigen::VectorXd::Random(rows)*.02;
  variance = noise;
  Eigen::MatrixXd h = Eigen::MatrixXd::Zero(rows,n);
  h.leftCols(12) = jacobian;
  Eigen::MatrixXd innovation = h*prior*h.transpose() + noise*Eigen::MatrixXd::Identity(rows,rows);
  Eigen::MatrixXd gain = innovation.ldlt().solve(h*prior).transpose();
  Eigen::VectorXd expected = gain*residual;
  Eigen::MatrixXd expected_cov = prior - gain*h*prior;
  Filter filter(State{}, prior);
  filter.init_dyn_share_modified_2h(nullptr,nullptr,observation<State>);
  if (!filter.update_iterated_dyn_share_modified()) throw std::runtime_error("update rejected");
  double state_error = (filter.get_x().value-expected).norm();
  double covariance_error = (filter.get_P()-expected_cov).norm();
  if (state_error>1e-8 || covariance_error>1e-7) {
    std::cerr << "n=" << n << " rows=" << rows << " R=" << noise
              << " state_error=" << state_error << " covariance_error=" << covariance_error << '\n';
    throw std::runtime_error("ESEKF does not match innovation reference");
  }
}
int main() {
  for (double noise : {.001,.1,2.}) for (int rows : {1,23,24,29,30,65}) {
    check<State24>(rows,noise);
    check<State30>(rows,noise);
  }
  std::cout << "36 ESEKF variance / dimension / degeneracy cases passed\n";
}
