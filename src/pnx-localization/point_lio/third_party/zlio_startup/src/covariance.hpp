/**
 * @file covariance.hpp
 * @brief 计算射线测量噪声及位姿不确定性引起的点协方差。
 */
#pragma once
#include <Eigen/Cholesky>
#include <array>
#include <cmath>

#include "zlio/state.hpp"
namespace zlio {
/// 在当前机体系计算位姿协方差对点位置的二次型传播。
class PointCovariance {
    std::array<std::array<double, 10>, 6> coefficients_{};
    static constexpr int kSymmetricPairs[6][2] = {{0, 0}, {0, 1}, {0, 2}, {1, 1}, {1, 2}, {2, 2}};

public:
    /// 将导航位姿协方差转换为机体系二次多项式系数。
    PointCovariance(const State& state, const Mat15& covariance) {
        const Mat3 inverse = state.pose.rotation.toRotationMatrix().transpose();
        Mat6 frame = Mat6::Zero();
        frame.topLeftCorner<3, 3>() = inverse;
        frame.topRightCorner<3, 3>() = -inverse * skew(state.pose.position);
        frame.bottomRightCorner<3, 3>() = inverse;
        const Mat6 P = covariance.topLeftCorner<6, 6>();
        const Eigen::LLT<Mat6> factor(P);
        Mat6 local;
        if (factor.info() == Eigen::Success) {
            const Mat6 root = frame * factor.matrixL().toDenseMatrix();
            local = root * root.transpose();
        } else
            local = frame * P * frame.transpose();
        // 展开常数、一次与二次坐标项。
        const Mat3 tt = local.topLeftCorner<3, 3>(), tr = local.topRightCorner<3, 3>(),
                   rr = local.bottomRightCorner<3, 3>();
        std::array<Mat3, 10> polynomial;
        polynomial[0] = tt;
        std::array<Mat3, 3> axes{skew(Vec3::UnitX()), skew(Vec3::UnitY()), skew(Vec3::UnitZ())};
        for (int i = 0; i < 3; ++i)
            polynomial[1 + i] = tr * axes[i] - axes[i] * tr.transpose();
        for (int k = 0; k < 6; ++k) {
            const int i = kSymmetricPairs[k][0], j = kSymmetricPairs[k][1];
            polynomial[4 + k] = -axes[i] * rr * axes[j];
            if (i != j)
                polynomial[4 + k].noalias() -= axes[j] * rr * axes[i];
        }
        for (int k = 0; k < 6; ++k)
            for (int j = 0; j < 10; ++j)
                coefficients_[k][j] =
                    .5 * (polynomial[j](kSymmetricPairs[k][0], kSymmetricPairs[k][1]) +
                          polynomial[j](kSymmetricPairs[k][1], kSymmetricPairs[k][0]));
    }
    /// 计算指定机体系点的位姿传播协方差。
    Mat3 operator()(const Vec3& p) const {
        std::array<double, 10> z{1,
                                 p.x(),
                                 p.y(),
                                 p.z(),
                                 p.x() * p.x(),
                                 p.x() * p.y(),
                                 p.x() * p.z(),
                                 p.y() * p.y(),
                                 p.y() * p.z(),
                                 p.z() * p.z()};
        Mat3 out;
        for (int k = 0; k < 6; ++k) {
            double v = 0;
            for (int j = 0; j < 10; ++j)
                v += coefficients_[k][j] * z[j];
            out(kSymmetricPairs[k][0], kSymmetricPairs[k][1]) =
                out(kSymmetricPairs[k][1], kSymmetricPairs[k][0]) = v;
        }
        return out;
    }
};

/// 按原始雷达射线计算径向及切向测量协方差。
class RayMeasurementCovariance {
    double range_variance_, angular_sine_;

public:
    /// 缓存测距方差和角噪声正弦值，角度单位为度。
    RayMeasurementCovariance(double range_sigma, double angle_sigma_degrees)
        : range_variance_(range_sigma * range_sigma),
          angular_sine_(std::sin(angle_sigma_degrees * std::acos(-1.) / 180.)) {}
    /// 根据原始射线长度及方向构造三维协方差。
    Mat3 operator()(const Vec3& point) const {
        const double range_squared = point.squaredNorm();
        if (range_squared < 1e-12)
            return Mat3::Identity() * range_variance_;
        const Vec3 direction = point / std::sqrt(range_squared);
        const Mat3 radial = direction * direction.transpose();
        return range_variance_ * radial +
               range_squared * angular_sine_ * angular_sine_ * (Mat3::Identity() - radial);
    }
};
}  // namespace zlio
