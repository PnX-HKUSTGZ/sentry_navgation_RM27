#pragma once
#include <Eigen/Cholesky>
#include <Eigen/Geometry>
#include <cstdint>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <stdexcept>
#include <string>

namespace rm27_backend {
using Pose = Eigen::Isometry3d;
using Covariance = Eigen::Matrix<double, 6, 6>;
using Cloud = pcl::PointCloud<pcl::PointXYZI>;
using Key = std::uint64_t;
/// 前端进程与重置代次的身份；同一进程不同 epoch 的位姿不能直接组成里程计边。
struct Identity {
  std::string instance;
  std::uint64_t epoch = 0;
  /// 判断两帧是否来自同一前端进程和重置代次。
  bool operator==(const Identity &b) const {
    return instance == b.instance && epoch == b.epoch;
  }
  /// 判断前端身份是否发生变化。
  bool operator!=(const Identity &b) const { return !(*this == b); }
};
/// 适配后的单帧元数据：odom 表示局部世界系下的 IMU 位姿，速度在 IMU 系表达。
/// local_covariance 使用局部右扰动 [平移, 旋转]；无效时由 covariance_valid 标记。
struct Frame {
  Identity identity;
  std::uint64_t sequence = 0;
  std::int64_t stamp_ns = 0;
  Pose odom = Pose::Identity();
  Eigen::Vector3d body_velocity = Eigen::Vector3d::Zero();
  Covariance local_covariance = Covariance::Zero();
  bool covariance_valid = false;
  bool degraded = false;
};
/// 后端关键帧元数据；frame 保留原始观测，estimate 保存图估计，id 不因分段复用。
struct Keyframe {
  Key id = 0;
  std::uint64_t segment = 0;
  Frame frame;
  Pose estimate = Pose::Identity();
};
// Covariances use local RIGHT perturbations [translation, rotation].
// Z_from_to maps points from the destination sensor into the source sensor.
/// 同一轨迹段内的相对位姿测量；relative 将 to 帧坐标转换到 from 帧。
/// 协方差使用局部右扰动 [平移, 旋转]，时间、净位移和净转角用于过程噪声建模。
struct OdometryMeasurement {
  Key from = 0, to = 0;
  Pose relative = Pose::Identity();
  Covariance covariance = Covariance::Identity();
  double seconds = 0, distance = 0, angle = 0;
};
// Interfaces for independently verified external observations. No ROS producer
// is connected yet. Empty/stale/duplicate identities are rejected by PoseGraph.
/// 预留的先验地图位姿观测，提交前应完成外部几何验收并统一到图坐标系。
/// sqrt_information 为 rank×6 矩阵，列顺序是局部右扰动 [平移, 旋转]。
struct AbsolutePoseMeasurement {
  std::string measurement_id;
  Key key = 0;
  std::uint64_t segment = 0;
  Pose pose = Pose::Identity();
  Eigen::MatrixXd
      sqrt_information; // rank x 6, local right [translation, rotation]
};
/// 预留的历史关键帧回环测量；relative 为 from←to，两个端点都必须仍在活动图中。
struct LoopMeasurement {
  std::string measurement_id;
  Key from = 0, to = 0;
  std::uint64_t segment = 0;
  Pose relative = Pose::Identity();
  Eigen::MatrixXd sqrt_information;
};
/// 检查齐次矩阵数值有限、末行合法，且旋转满足 SO(3) 的正交性和行列式要求。
inline bool validPose(const Pose &p) {
  return p.matrix().allFinite() &&
         p.matrix().row(3).isApprox(Eigen::RowVector4d(0, 0, 0, 1), 1e-9) &&
         (p.linear().transpose() * p.linear())
             .isApprox(Eigen::Matrix3d::Identity(), 1e-6) &&
         std::abs(p.linear().determinant() - 1.) < 1e-6;
}
/// 返回相对位姿的最小旋转角，单位 rad。
inline double angle(const Pose &p) {
  return Eigen::AngleAxisd(p.linear()).angle();
}
/// 检查协方差有限、近似对称且严格正定，拒绝奇异或不定矩阵。
inline bool positiveCovariance(const Covariance &p) {
  return p.allFinite() && p.isApprox(p.transpose(), 1e-8) &&
         Eigen::LLT<Covariance>(p).info() == Eigen::Success;
}
} // namespace rm27_backend
