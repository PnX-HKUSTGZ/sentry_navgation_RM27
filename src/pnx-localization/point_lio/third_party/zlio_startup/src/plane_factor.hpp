/**
 * @file plane_factor.hpp
 * @brief 消去多帧点面约束中的共享自由平面参数。
 */
#pragma once

#include <Eigen/Cholesky>

#include "moments.hpp"
#include "zlio/state.hpp"

namespace zlio {

/// 保存消去自由平面后的目标信息与梯度噪声协方差。
struct PlaneCalibration {
    Eigen::MatrixXd information;
    Eigen::MatrixXd score_covariance;
    bool valid = false;
};

/// 累计多帧点面约束并消去三维自由平面参数。
class FreePlaneFactor {
    Eigen::MatrixXd aa_, noise_aa_, ax_, noise_ax_;
    Mat3 xx_ = Mat3::Zero(), noise_xx_ = Mat3::Zero();

public:
    /// 按窗口帧数分配位姿信息及共享平面工作区。
    explicit FreePlaneFactor(int frames)
        : aa_(Eigen::MatrixXd::Zero(6 * frames, 6 * frames)),
          noise_aa_(aa_),
          ax_(Eigen::MatrixXd::Zero(6 * frames, 3)),
          noise_ax_(ax_) {}

    /// 以机体系右旋转和世界系平移扰动累计一帧的几何信息与噪声信息。
    void addFrame(int frame, const plane::MomentStatistics& moments, const Vec3& origin,
                  const Mat3& rotation, const Vec3& position, const Vec3& center,
                  const Mat3& plane_axes) {
        const Vec3 normal = plane_axes.col(0);
        const Vec3 body_normal = rotation.transpose() * normal;
        Eigen::Matrix<double, 6, 4> target = Eigen::Matrix<double, 6, 4>::Zero();
        target.topLeftCorner<3, 3>() = -skew(body_normal);
        target.topRightCorner<3, 1>() = origin.cross(body_normal);
        target.bottomRightCorner<3, 1>() = normal;
        Eigen::Matrix<double, 3, 4> plane = Eigen::Matrix<double, 3, 4>::Zero();
        plane.topLeftCorner<2, 3>() = plane_axes.rightCols<2>().transpose() * rotation;
        plane.topRightCorner<2, 1>() =
            plane_axes.rightCols<2>().transpose() * (rotation * origin + position - center);
        plane(2, 3) = -1.;
        const Eigen::Matrix4d geometry = moments.homogeneousMoments();
        const Eigen::Matrix4d noise = moments.normalNoiseMoments(body_normal);
        const int offset = 6 * frame;
        aa_.block<6, 6>(offset, offset).noalias() += target * geometry * target.transpose();
        noise_aa_.block<6, 6>(offset, offset).noalias() += target * noise * target.transpose();
        ax_.block<6, 3>(offset, 0).noalias() += target * geometry * plane.transpose();
        noise_ax_.block<6, 3>(offset, 0).noalias() += target * noise * plane.transpose();
        xx_.noalias() += plane * geometry * plane.transpose();
        noise_xx_.noalias() += plane * noise * plane.transpose();
    }

    /// 消去平面参数，返回约束信息及其夹心协方差中间量。
    PlaneCalibration eliminate() const {
        const Eigen::LDLT<Mat3> factor(xx_);
        if (factor.info() != Eigen::Success || !factor.isPositive())
            return {};
        const Eigen::MatrixXd K = factor.solve(ax_.transpose());
        const Eigen::MatrixXd mixed = noise_ax_ * K;
        return {aa_ - ax_ * K,
                noise_aa_ - mixed - mixed.transpose() + K.transpose() * noise_xx_ * K, true};
    }
};
}  // namespace zlio
