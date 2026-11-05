/**
 * @file plane.cpp
 * @brief 实现平面谱判据与世界系地图点的协方差传播。
 */
#include "zlio/plane.hpp"

#include "zlio/state.hpp"
namespace zlio {
/// 按升序散布特征值同时检查绝对厚度和相对平面形状。
bool planarSpectrum(const Vec3& values, double threshold, double eigenvalue_ratio,
                    double middle_ratio) {
    return values.allFinite() && values[0] < threshold && values[1] > kMinPlaneTangentialVariance &&
           values[0] < eigenvalue_ratio * values[2] && values[0] <= middle_ratio * values[1];
}

/// 将共同位姿误差传播至世界系支持点并叠加独立测量噪声。
Mat3 MapPoint::marginalCovariance() const {
    if (!shared_pose_covariance)
        return independent_covariance;
    const auto J = pointPoseJacobian(point);
    return J * (*shared_pose_covariance) * J.transpose() + independent_covariance;
}

/// 将观测变换到世界系，分别保存独立测量项和共同位姿项。
std::vector<MapPoint> transformForMap(const std::vector<Observation>& points,
                                      const Estimate& estimate) {
    std::vector<MapPoint> result;
    result.reserve(points.size());
    const Mat3 rotation = estimate.state.pose.rotation.toRotationMatrix();
    const MapPoseCovariance shared(estimate.covariance.topLeftCorner<6, 6>());
    for (const auto& point : points) {
        const Vec3 world = estimate.state.pose.apply(point.point);
        const Mat3 independent = rotation * point.covariance * rotation.transpose();
        result.push_back({world, independent, shared});
    }
    return result;
}

}  // namespace zlio
