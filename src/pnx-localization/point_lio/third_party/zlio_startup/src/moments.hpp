/**
 * @file moments.hpp
 * @brief 累计点矩与噪声矩，计算平面参数及其协方差。
 */
#pragma once
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>

#include "zlio/plane.hpp"

namespace zlio::plane {
using Vec9 = Eigen::Matrix<double, 9, 1>;
using Mat9 = Eigen::Matrix<double, 9, 9>;
using Jacobian = Eigen::Matrix<double, 6, 9>;
inline constexpr std::array<std::array<int, 2>, 6> kSymmetricPairs{
    {{0, 0}, {0, 1}, {0, 2}, {1, 1}, {1, 2}, {2, 2}}};

/// 从法向散布中扣除已解释噪声，估计表面剩余方差。
inline double residualScatter(std::size_t count, double normal_variance,
                              double known_residual_trace) {
    if (count <= 3 || !std::isfinite(known_residual_trace))
        return std::numeric_limits<double>::quiet_NaN();
    const double explained = known_residual_trace / (count - 3.);
    return std::max(0., std::max(0., normal_variance) * count / (count - 3.) - explained);
}

/// 保存拟合平面的几何参数和三维参数协方差。
struct Fit {
    Vec3 center = Vec3::Zero(), normal = Vec3::Zero();
    /// 两个法向切向坐标及法向中心偏移的对称协方差。
    std::array<double, 6> noise{};
    double radius = 0., surface_variance = 0.;
    bool valid = false;
    /// 根据单位法向构造正交切向基。
    Eigen::Matrix<double, 3, 2> tangentFrame() const {
        Eigen::Matrix<double, 3, 2> B;
        B.col(0) = normal.unitOrthogonal();
        B.col(1) = normal.cross(B.col(0));
        return B;
    }
    /// 将平面参数协方差传播为查询点的点面残差方差。
    double variance(const Vec3& point, Vec3& jacobian) const {
        const auto B = tangentFrame();
        const Vec3 delta = point - center;
        const Vec3 H(delta.dot(B.col(0)), delta.dot(B.col(1)), -1.);
        jacobian = H;
        double variance = 0.;
        for (std::size_t k = 0; k < kSymmetricPairs.size(); ++k) {
            const auto ij = kSymmetricPairs[k];
            variance += (ij[0] == ij[1] ? 1. : 2.) * H[ij[0]] * H[ij[1]] * noise[k];
        }
        const double scale =
            H.squaredNorm() * (std::abs(noise[0]) + std::abs(noise[3]) + std::abs(noise[5]));
        if (variance < -128 * std::numeric_limits<double>::epsilon() * std::max(scale, 1e-30))
            return std::numeric_limits<double>::quiet_NaN();
        return std::max(0., variance);
    }
};

/// 累计九维点矩和协方差加权空间矩。
class MomentStatistics {
public:
    /// 相对局部原点累加支持点的几何量和噪声量。
    template <class Point>
    void add(const Point& value, const Vec3& origin) {
        const Vec3 x = value.point - origin;
        const Mat3 marginal = value.marginalCovariance();
        const Mat3 covariance = .5 * (marginal + marginal.transpose()).eval();
        Vec9 z;
        z.head<3>() = x;
        for (std::size_t k = 0; k < kSymmetricPairs.size(); ++k) {
            const auto ij = kSymmetricPairs[k];
            z[3 + k] = x[ij[0]] * x[ij[1]];
        }
        sum_ += z;
        // 累加六个协方差分量与十个空间单项式的乘积。
        for (std::size_t k = 0; k < kSymmetricPairs.size(); ++k) {
            const auto ij = kSymmetricPairs[k];
            const double scale = covariance(ij[0], ij[1]);
            noise_[k][0] += scale;
            for (int j = 0; j < 9; ++j)
                noise_[k][j + 1] += scale * z[j];
        }
        ++count_;
    }
    /// 返回局部齐次坐标 [point-origin;1] 的二阶矩。
    Eigen::Matrix4d homogeneousMoments() const {
        Eigen::Matrix4d result = Eigen::Matrix4d::Zero();
        for (std::size_t k = 0; k < kSymmetricPairs.size(); ++k) {
            const auto ij = kSymmetricPairs[k];
            result(ij[0], ij[1]) = result(ij[1], ij[0]) = sum_[3 + k];
        }
        result.topRightCorner<3, 1>() = sum_.head<3>();
        result.bottomLeftCorner<1, 3>() = sum_.head<3>().transpose();
        result(3, 3) = count_;
        return result;
    }
    /// 返回沿指定方向的噪声方差加权齐次二阶矩。
    Eigen::Matrix4d normalNoiseMoments(const Vec3& direction) const {
        const auto projected = normalNoiseMonomials(direction);
        Eigen::Matrix4d result = Eigen::Matrix4d::Zero();
        for (std::size_t k = 0; k < kSymmetricPairs.size(); ++k) {
            const auto ij = kSymmetricPairs[k];
            result(ij[0], ij[1]) = result(ij[1], ij[0]) = projected[k + 4];
        }
        const Vec3 first(projected[1], projected[2], projected[3]);
        result.topRightCorner<3, 1>() = first;
        result.bottomLeftCorner<1, 3>() = first.transpose();
        result(3, 3) = projected[0];
        return result;
    }
    /// 根据一阶及二阶点矩计算几何散布矩阵。
    Mat3 scatter() const {
        const Vec3 mean = sum_.head<3>() / static_cast<double>(count_);
        Mat3 scatter;
        for (std::size_t k = 0; k < kSymmetricPairs.size(); ++k) {
            const auto ij = kSymmetricPairs[k];
            scatter(ij[0], ij[1]) = scatter(ij[1], ij[0]) =
                sum_[3 + k] / static_cast<double>(count_) - mean[ij[0]] * mean[ij[1]];
        }
        return scatter;
    }
    /// 根据加权空间矩恢复九维点矩协方差。
    Mat9 momentCovariance() const {
        Mat9 matrix = Mat9::Zero();
        for (int i = 0; i < 9; ++i)
            for (int j = i; j < 9; ++j) {
                for (int a = 0; a < (i < 3 ? 1 : 2); ++a)
                    for (int b = 0; b < (j < 3 ? 1 : 2); ++b) {
                        const int ai = i < 3 ? i : kSymmetricPairs[i - 3][a];
                        const int ap = i < 3 ? -1 : kSymmetricPairs[i - 3][1 - a];
                        const int bi = j < 3 ? j : kSymmetricPairs[j - 3][b];
                        const int bp = j < 3 ? -1 : kSymmetricPairs[j - 3][1 - b];
                        matrix(i, j) += noise_[pairIndex(ai, bi)][monomialIndex(ap, bp)];
                    }
                matrix(j, i) = matrix(i, j);
            }
        return matrix;
    }
    /// 计算扣除三维平面拟合自由度后的已知残差噪声迹。
    double knownResidualTrace(const Vec3& normal, const Mat3& eigenvectors,
                              const Vec3& eigenvalues) const {
        const auto projected = normalNoiseMonomials(normal);
        const Vec3 mean = sum_.head<3>() / count_;
        const Vec3 first(projected[1], projected[2], projected[3]);
        Mat3 second;
        for (std::size_t k = 0; k < kSymmetricPairs.size(); ++k) {
            const auto ij = kSymmetricPairs[k];
            second(ij[0], ij[1]) = second(ij[1], ij[0]) = projected[k + 4];
        }
        const Mat3 centered = second - mean * first.transpose() - first * mean.transpose() +
                              projected[0] * mean * mean.transpose();
        double removed = projected[0] / count_;
        for (int k = 1; k < 3; ++k) {
            if (!(eigenvalues[k] > 0))
                return std::numeric_limits<double>::quiet_NaN();
            removed +=
                eigenvectors.col(k).dot(centered * eigenvectors.col(k)) / (count_ * eigenvalues[k]);
        }
        const double trace = projected[0] - removed;
        const double tolerance = 1e-10 * std::max(projected[0], 1e-30);
        if (trace < -tolerance || trace > projected[0] + tolerance)
            return std::numeric_limits<double>::quiet_NaN();
        return std::max(0., trace);
    }
    /// 拟合局部平面并传播几何噪声和表面散布。
    Fit fit(const Vec3& origin, double threshold = .01,
            const Mat3* independent_sensor_sum = nullptr) const {
        Fit out;
        if (count_ < 6u)
            return out;
        const Vec3 mean = sum_.head<3>() / static_cast<double>(count_);
        const Mat3 A = scatter();
        Eigen::SelfAdjointEigenSolver<Mat3> eigen(A);
        if (eigen.info() != Eigen::Success || !eigen.eigenvalues().allFinite())
            return out;
        const Vec3& values = eigen.eigenvalues();
        const Mat3& U = eigen.eigenvectors();
        out.center = origin + mean;
        out.normal = U.col(0);
        out.radius = std::sqrt(std::max(0., values[2]));
        // 平面需有二维切向支撑；重复点或共线支持不参与法向及协方差计算。
        if (!(values[0] < threshold && values[1] > kMinPlaneTangentialVariance))
            return out;
        // 由散布特征向量的导数构造平面参数 Jacobian。
        Jacobian J = Jacobian::Zero();
        for (int axis = 1; axis < 3; ++axis) {
            const double gap = values[0] - values[axis];
            if (std::abs(gap) <=
                64 * std::numeric_limits<double>::epsilon() * std::max(A.norm(), 1e-30))
                return out;
            const Vec3 u = U.col(axis), scaled = u / (static_cast<double>(count_) * gap);
            J.topLeftCorner<3, 3>().noalias() -=
                scaled * (mean.dot(out.normal) * u + mean.dot(u) * out.normal).transpose();
            for (std::size_t k = 0; k < kSymmetricPairs.size(); ++k) {
                const auto ij = kSymmetricPairs[k];
                const double coefficient = u[ij[0]] * out.normal[ij[1]] +
                                           (ij[0] == ij[1] ? 0. : u[ij[1]] * out.normal[ij[0]]);
                J.block<3, 1>(0, 3 + k).noalias() += scaled * coefficient;
            }
        }
        J.block<3, 3>(3, 0) = Mat3::Identity() / static_cast<double>(count_);
        // 将法向及中心协方差投影到三维平面参数。
        const auto B = out.tangentFrame();
        Eigen::Matrix<double, 3, 6> projection = Eigen::Matrix<double, 3, 6>::Zero();
        projection.topLeftCorner<2, 3>() = B.transpose();
        projection.block<1, 3>(2, 3) = out.normal.transpose();
        const Mat9 S = momentCovariance();
        const Mat6 raw = J * S * J.transpose();
        Mat6 C = .5 * (raw + raw.transpose()).eval();
        {
            // 扣除测量噪声，估计剩余表面散布。
            if (independent_sensor_sum) {
                out.surface_variance =
                    std::max(0., count_ * std::max(0., values[0]) / (count_ - 3.) -
                                     out.normal.dot(*independent_sensor_sum * out.normal) / count_);
            } else {
                const double trace = knownResidualTrace(out.normal, U, values);
                if (!std::isfinite(trace))
                    return out;
                out.surface_variance = residualScatter(count_, values[0], trace);
            }
            const double variance_per_support = out.surface_variance / count_;
            for (int k = 1; k < 3; ++k) {
                const double gap = values[0] - values[k];
                C.topLeftCorner<3, 3>().noalias() += variance_per_support * values[k] /
                                                     (gap * gap) *
                                                     (U.col(k) * U.col(k).transpose());
            }
            C.bottomRightCorner<3, 3>().noalias() +=
                variance_per_support * (out.normal * out.normal.transpose());
        }
        if (!C.allFinite())
            return out;
        const Mat3 compact = projection * C * projection.transpose();
        for (std::size_t k = 0; k < kSymmetricPairs.size(); ++k)
            out.noise[k] = .5 * (compact(kSymmetricPairs[k][0], kSymmetricPairs[k][1]) +
                                 compact(kSymmetricPairs[k][1], kSymmetricPairs[k][0]));
        out.valid = true;
        return out;
    }

private:
    /// 将各协方差分量投影为指定法向上的加权单项式。
    std::array<double, 10> normalNoiseMonomials(const Vec3& normal) const {
        std::array<double, 10> projected{};
        for (std::size_t k = 0; k < kSymmetricPairs.size(); ++k) {
            const auto ij = kSymmetricPairs[k];
            const double scale = (ij[0] == ij[1] ? 1. : 2.) * normal[ij[0]] * normal[ij[1]];
            for (int j = 0; j < 10; ++j)
                projected[j] += scale * noise_[k][j];
        }
        return projected;
    }
    /// 返回对称三维矩阵元素的紧凑索引。
    static int pairIndex(int a, int b) {
        constexpr int kPairIndex[3][3] = {{0, 1, 2}, {1, 3, 4}, {2, 4, 5}};
        return kPairIndex[a][b];
    }
    /// 返回常数、一阶量或二阶乘积的单项式索引。
    static int monomialIndex(int a, int b) {
        if (a < 0)
            return b < 0 ? 0 : b + 1;
        if (b < 0)
            return a + 1;
        return 4 + pairIndex(a, b);
    }
    Vec9 sum_ = Vec9::Zero();
    std::array<std::array<double, 10>, 6> noise_{};
    std::uint32_t count_ = 0;
};
}  // namespace zlio::plane
