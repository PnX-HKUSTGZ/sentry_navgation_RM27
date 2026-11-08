#include "rm27_backend/measurement.hpp"
#include <Eigen/Eigenvalues>
#include <cmath>
#include <limits>

namespace rm27_backend {
namespace {
/// 数值零空间判定使用的相对舍入容差。
constexpr double kRoundoff = 1e-12;
} // 匿名命名空间
/// 按平移/旋转块尺度平衡 Hessian，特征分解后保留数值可观且不弱于条件数门限的方向。
/// 门限只决定是否给出该方向的信息，不抬升任何弱方向的信息量。
HessianAnalysis analyzeHessian(const Matrix6 &hessian, double max_condition) {
  HessianAnalysis out;
  if (!hessian.allFinite()) {
    out.reason = "pose Hessian contains a non-finite value";
    return out;
  }
  const Matrix6 h = .5 * (hessian + hessian.transpose());
  const double magnitude = h.cwiseAbs().maxCoeff();
  if (magnitude <= 0) {
    out.reason = "pose Hessian is zero";
    return out;
  }
  Matrix6 inverse = Matrix6::Zero();
  out.diagonal_sqrt.setZero();
  for (int offset : {0, 3}) {
    const double scale = std::max(h.block<3, 3>(offset, offset).norm(),
                                  magnitude * kRoundoff);
    out.diagonal_sqrt.block<3, 3>(offset, offset) =
        std::sqrt(scale) * Eigen::Matrix3d::Identity();
    inverse.block<3, 3>(offset, offset) =
        Eigen::Matrix3d::Identity() / std::sqrt(scale);
  }
  const Matrix6 balanced = inverse * h * inverse;
  Eigen::SelfAdjointEigenSolver<Matrix6> solver(balanced);
  if (solver.info() != Eigen::Success || !solver.eigenvalues().allFinite()) {
    out.reason = "pose Hessian eigendecomposition failed";
    return out;
  }
  out.eigenvalues = solver.eigenvalues();
  out.eigenvectors = solver.eigenvectors();
  const double largest = out.eigenvalues.cwiseAbs().maxCoeff();
  const double tolerance = kRoundoff * largest;
  if (out.eigenvalues.minCoeff() < -8. * tolerance) {
    out.reason = "pose Hessian is materially indefinite";
    return out;
  }
  const double weak = max_condition > 1. ? largest / max_condition : tolerance;
  double smallest_kept = largest;
  for (int i = 0; i < 6; ++i) {
    if (out.eigenvalues(i) > std::max(tolerance, weak)) {
      ++out.rank;
      smallest_kept = std::min(smallest_kept, out.eigenvalues(i));
    } else {
      out.eigenvalues(i) = 0;
    }
  }
  if (!out.rank) {
    out.reason = "pose Hessian has no observable direction";
    return out;
  }
  out.condition = largest / smallest_kept;
  out.valid = true;
  return out;
}
/// 按 z-localization 的形式 L_row = sqrt(s·λ_i)·v_iᵀ·D 生成平方根信息，
/// 其中 s 为单位残差的信息尺度，λ_i/v_i 为平衡坐标下保留的特征对。
Eigen::MatrixXd sqrtInformationFromHessian(const HessianAnalysis &analysis,
                                           double information_per_unit) {
  if (!analysis.valid || analysis.rank <= 0 ||
      !std::isfinite(information_per_unit) || information_per_unit <= 0)
    return {};
  Eigen::MatrixXd l(analysis.rank, 6);
  const int first = 6 - analysis.rank;
  for (int row = 0; row < analysis.rank; ++row) {
    const int index = first + row;
    l.row(row) = std::sqrt(information_per_unit * analysis.eigenvalues(index)) *
                 analysis.eigenvectors.col(index).transpose() *
                 analysis.diagonal_sqrt;
  }
  return l.allFinite() ? l : Eigen::MatrixXd{};
}
/// 白化后测量协方差为 I；叠加共模协方差 C 后为 I + L C Lᵀ，再用其 Cholesky 因子重新白化。
std::optional<Eigen::MatrixXd>
addMeasurementCovariance(const Eigen::MatrixXd &l, const Covariance &c) {
  if (l.cols() != 6 || l.rows() < 1 || !l.allFinite() || !c.allFinite())
    return std::nullopt;
  const Covariance symmetric = .5 * (c + c.transpose());
  Eigen::SelfAdjointEigenSolver<Covariance> check(symmetric);
  if (check.info() != Eigen::Success || check.eigenvalues().minCoeff() < -1e-12)
    return std::nullopt;
  Eigen::MatrixXd s = Eigen::MatrixXd::Identity(l.rows(), l.rows()) +
                      l * symmetric * l.transpose();
  s = .5 * (s + s.transpose()).eval();
  Eigen::LLT<Eigen::MatrixXd> llt(s);
  if (llt.info() != Eigen::Success)
    return std::nullopt;
  Eigen::MatrixXd out = llt.matrixL().solve(l);
  if (!out.allFinite())
    return std::nullopt;
  return out;
}
/// 满秩时直接求逆信息矩阵，并对称化消除舍入误差。
std::optional<Covariance>
covarianceFromSqrtInformation(const Eigen::MatrixXd &l) {
  if (l.rows() != 6 || l.cols() != 6 || !l.allFinite())
    return std::nullopt;
  const Covariance information = l.transpose() * l;
  Eigen::LLT<Covariance> llt(information);
  if (llt.info() != Eigen::Success)
    return std::nullopt;
  Covariance p = llt.solve(Covariance::Identity());
  p = .5 * (p + p.transpose()).eval();
  if (!positiveCovariance(p))
    return std::nullopt;
  return p;
}
/// [x,y,z,roll,pitch,yaw] 按 Rz·Ry·Rx 组成旋转；长度不足 6 时返回单位变换。
Pose poseFromVector(const std::vector<double> &v) {
  Pose p = Pose::Identity();
  if (v.size() < 6)
    return p;
  p.translation() << v[0], v[1], v[2];
  p.linear() = (Eigen::AngleAxisd(v[5], Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxisd(v[4], Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(v[3], Eigen::Vector3d::UnitX()))
                   .toRotationMatrix();
  return p;
}
} // 命名空间 rm27_backend
