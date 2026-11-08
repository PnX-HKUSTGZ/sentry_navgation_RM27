#pragma once
#include "rm27_backend/types.hpp"
#include <optional>
#include <vector>

namespace rm27_backend {
using Matrix6 = Eigen::Matrix<double, 6, 6>;
using Vector6 = Eigen::Matrix<double, 6, 1>;
/// 配准位姿 Hessian 的可观性分析；所有矩阵均为局部右扰动 [平移, 旋转]。
/// 平移/旋转两个块各自按同一尺度平衡后再做特征分解，避免逐轴归一化把弱轴放大。
struct HessianAnalysis {
  bool valid = false;
  int rank = 0;                         // 保留的可观方向数
  Vector6 eigenvalues = Vector6::Zero(); // 平衡坐标下的特征值，升序，未保留方向置零
  Matrix6 eigenvectors = Matrix6::Identity();
  Matrix6 diagonal_sqrt = Matrix6::Identity(); // H = D·B·D 中的块尺度 D
  double condition = 0;                        // 保留方向内的最大/最小特征值比
  std::string reason;
};
/// 分析 6×6 位姿 Hessian：剔除数值零空间以及弱于 λmax/max_condition 的退化方向。
/// max_condition≤1 时只剔除数值零空间，与 z-localization 的数值秩判定一致。
HessianAnalysis analyzeHessian(const Matrix6 &hessian, double max_condition);
/// 由 Hessian 分析和单位残差信息尺度构造 rank×6 平方根信息，行顺序按特征值升序。
/// information_per_unit = information_scale / residual_variance；返回空矩阵表示不可用。
Eigen::MatrixXd sqrtInformationFromHessian(const HessianAnalysis &analysis,
                                           double information_per_unit);
/// 在测量切空间叠加共模协方差：L' = chol(I + L C Lᵀ)⁻¹ L，适用于部分秩信息。
/// 用于表达地图误差、平面拟合误差和点间相关性造成的不可平均误差下限。
std::optional<Eigen::MatrixXd>
addMeasurementCovariance(const Eigen::MatrixXd &sqrt_information,
                         const Covariance &covariance);
/// 仅对满秩平方根信息求协方差 (LᵀL)⁻¹；秩不足或数值异常返回空。
std::optional<Covariance>
covarianceFromSqrtInformation(const Eigen::MatrixXd &sqrt_information);
/// 计算 4×4 位姿的 ZYX 欧拉向量 [x,y,z,roll,pitch,yaw] 对应变换，约定与定位管理器一致。
Pose poseFromVector(const std::vector<double> &pose);
} // 命名空间 rm27_backend
