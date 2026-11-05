/**
 * @file preintegration.cpp
 * @brief 计算帧间惯性预积分及首帧加速度偏置约束。
 */
#include <Eigen/Dense>
#include <algorithm>
#include <stdexcept>

#include "bootstrap.hpp"
#include "zlio/config.hpp"
#include "zlio/inertial.hpp"
#include "zlio/state.hpp"

namespace zlio {
namespace {
inline constexpr int kStateDimension = 15;
inline const Eigen::Matrix3d kIdentity3 = Eigen::Matrix3d::Identity();
/// 计算旋转指数映射的右 Jacobian。
inline Mat3 rightJacobian(Vec3 rotation_vector) {
    const double angle = rotation_vector.norm();
    if (angle < 1e-9)
        return kIdentity3;
    rotation_vector /= angle;
    const double sinc_angle = std::sin(angle) / angle;
    return sinc_angle * kIdentity3 +
           (1 - sinc_angle) * rotation_vector * rotation_vector.transpose() -
           (1 - std::cos(angle)) / angle * skew(rotation_vector);
}

/// 计算旋转对数映射的逆右 Jacobian。
inline Mat3 inverseRightJacobian(const Mat3& rotation) {
    const Eigen::AngleAxisd angle_axis(rotation);
    const Vec3& axis = angle_axis.axis();
    const double angle = angle_axis.angle();
    if (angle < 1e-9)
        return kIdentity3;
    const double half_angle_cotangent = angle / 2 / std::tan(angle / 2);
    return half_angle_cotangent * kIdentity3 +
           (1 - half_angle_cotangent) * axis * axis.transpose() + angle / 2 * skew(axis);
}

class ImuPreintegration {
public:
    /// 恢复最近一次试探前的两种偏置修正。
    void rollbackBiasIncrement() {
        gyro_increment_ = previous_gyro_increment_;
        accel_increment_ = previous_accel_increment_;
    }
    /// 以指定偏置及噪声建立尚未积分的帧间因子。
    ImuPreintegration(const Eigen::Vector3d& initial_gyro_bias,
                      const Eigen::Vector3d& initial_accel_bias, ImuNoise noise);

    /// 按相邻 IMU 中值积分并建立冻结信息矩阵。
    void integrate(const std::vector<zlio::Imu>& imus);

    /// 计算代价并返回两帧及重力的线性化块。
    double linearize(const WindowState& first_state, const WindowState& second_state,
                     Eigen::MatrixXd& factor_information, Eigen::VectorXd& factor_gradient);
    /// 使用当前残差和冻结权重计算试探代价。
    double evaluateCost(const WindowState& first_state, const WindowState& second_state);

    /// 保存旧修正并累积当前试探的偏置增量。
    void applyBiasIncrement(const Eigen::Matrix<double, kStateDimension, 1>& increment);

private:
    struct ResidualTerms {
        Mat3 rotation;
        Vec3 expected_position, expected_velocity;
        Vec15 value;
    };
    using FactorJacobian = Eigen::Matrix<double, kStateDimension, 2 * kStateDimension + 3>;
    /// 形成两帧状态在当前偏置修正下的残差项。
    ResidualTerms residualTerms(const WindowState& first, const WindowState& second) const;
    /// 按残差项构造两帧和公共重力的导数。
    FactorJacobian factorJacobian(const WindowState& first, const WindowState& second,
                                  const ResidualTerms& terms) const;
    Eigen::Matrix3d rotation_delta_;
    Eigen::Vector3d position_delta_, velocity_delta_;
    Eigen::Vector3d gyro_bias_, accel_bias_;

    Eigen::Matrix3d rotation_gyro_jacobian_;
    Eigen::Matrix3d position_gyro_jacobian_, position_accel_jacobian_;
    Eigen::Matrix3d velocity_gyro_jacobian_, velocity_accel_jacobian_;

    double duration_;
    Mat6 measurement_noise_, bias_noise_;
    Mat15 information_ = Mat15::Zero();
    bool information_ready_ = false;
    /// 检查并建立当前预积分的信息矩阵缓存。
    void prepareInformation();
    /// 按单个有效 IMU 间隔推进预积分量。
    void integrateSample(const Vec3& angular_velocity, const Vec3& acceleration, double interval,
                         double source_interval);

    Eigen::Vector3d gyro_increment_, accel_increment_;
    Eigen::Vector3d previous_gyro_increment_, previous_accel_increment_;

    Eigen::Matrix<double, kStateDimension, kStateDimension> covariance_;
};
/// 首帧 IMU 系加速度偏置的对角高斯先验。
class AccelerationBiasPrior {
public:
    /// 检查并保存首帧 IMU 系加速度偏置的对角高斯先验。
    AccelerationBiasPrior(const Vec3& mean, const Vec3& deviations) : mean_(mean) {
        if (!mean.allFinite() || !deviations.allFinite() || (deviations.array() <= 0).any())
            throw std::invalid_argument("Invalid accelerometer bias prior mean or deviation");
        information_ = deviations.array().square().inverse();
        if (!information_.allFinite() || (information_.array() <= 0).any())
            throw std::invalid_argument("Invalid accelerometer bias prior information");
    }
    /// 返回当前首帧偏置相对先验中心的加权残差平方。
    double squaredError(const WindowState& state) const {
        const Vec3 residual = state.accel_bias - mean_;
        return residual.dot(information_.cwiseProduct(residual));
    }
    /// 把先验半平方代价的梯度和 Hessian 加入首帧偏置块。
    void accumulate(const WindowState& state, Eigen::MatrixXd& hessian,
                    Eigen::VectorXd& gradient) const {
        hessian.block<3, 3>(12, 12).diagonal() += information_;
        gradient.segment<3>(12) += information_.cwiseProduct(state.accel_bias - mean_);
    }

private:
    Vec3 mean_, information_;
};

/// 初始化预积分均值、偏置导数及测量和随机游走噪声。
ImuPreintegration::ImuPreintegration(const Eigen::Vector3d& initial_gyro_bias,
                                     const Eigen::Vector3d& initial_accel_bias, ImuNoise noise) {
    measurement_noise_.setZero();
    measurement_noise_.topLeftCorner<3, 3>().diagonal().setConstant(noise.gyro_sample_variance);
    measurement_noise_.bottomRightCorner<3, 3>().diagonal().setConstant(
        noise.accel_sample_variance);
    bias_noise_.setZero();
    bias_noise_.topLeftCorner<3, 3>().diagonal().setConstant(noise.gyro_bias_psd);
    bias_noise_.bottomRightCorner<3, 3>().diagonal().setConstant(noise.accel_bias_psd);
    gyro_bias_ = initial_gyro_bias;
    accel_bias_ = initial_accel_bias;
    rotation_delta_.setIdentity();
    position_delta_.setZero();
    velocity_delta_.setZero();

    rotation_gyro_jacobian_.setZero();
    position_gyro_jacobian_.setZero();
    position_accel_jacobian_.setZero();
    velocity_gyro_jacobian_.setZero();
    velocity_accel_jacobian_.setZero();

    duration_ = 0;

    gyro_increment_.setZero();
    accel_increment_.setZero();
    previous_gyro_increment_.setZero();
    previous_accel_increment_.setZero();

    covariance_.setZero();
}

/// 积分完整帧间 IMU 包并检查协方差可逆性。
void ImuPreintegration::integrate(const std::vector<zlio::Imu>& imus) {
    for (std::size_t i = 1; i < imus.size(); ++i) {
        Eigen::Vector3d angular_velocity =
            0.5 * (imus[i - 1].angular_velocity + imus[i].angular_velocity) - gyro_bias_;
        Eigen::Vector3d acceleration =
            0.5 * (imus[i - 1].acceleration + imus[i].acceleration) - accel_bias_;
        integrateSample(angular_velocity, acceleration,
                        zlio::seconds(imus[i].time - imus[i - 1].time),
                        zlio::seconds(originalImuInterval(imus[i - 1], imus[i])));
    }
    prepareInformation();
}

/// 按单个正时间段推进预积分均值、偏置 Jacobian 与协方差。
void ImuPreintegration::integrateSample(const Vec3& angular_velocity, const Vec3& acceleration,
                                        double interval, double source_interval) {
    if (!std::isfinite(interval) || interval <= 0 || !angular_velocity.allFinite() ||
        !acceleration.allFinite())
        throw std::invalid_argument("Invalid IMU preintegration interval or measurement");
    information_ready_ = false;
    duration_ += interval;
    Eigen::Matrix3d rotation_increment = rotationIncrement(angular_velocity, interval);
    Eigen::Matrix3d rotation_jacobian(rightJacobian(angular_velocity * interval));

    Eigen::Matrix3d velocity_acceleration_gain = interval * rotation_delta_;
    Eigen::Matrix3d position_acceleration_gain = 0.5 * interval * interval * rotation_delta_;

    Eigen::Matrix3d acceleration_cross;
    acceleration_cross = skew(acceleration);

    // 传播偏置对位置、速度和旋转的导数。
    position_accel_jacobian_ =
        position_accel_jacobian_ + velocity_accel_jacobian_ * interval - position_acceleration_gain;
    position_gyro_jacobian_ =
        position_gyro_jacobian_ + velocity_gyro_jacobian_ * interval -
        position_acceleration_gain * acceleration_cross * rotation_gyro_jacobian_;
    velocity_accel_jacobian_ = velocity_accel_jacobian_ - velocity_acceleration_gain;
    velocity_gyro_jacobian_ = velocity_gyro_jacobian_ - velocity_acceleration_gain *
                                                            acceleration_cross *
                                                            rotation_gyro_jacobian_;
    rotation_gyro_jacobian_ =
        rotation_increment.transpose() * rotation_gyro_jacobian_ - rotation_jacobian * interval;

    // 传播运动协方差并累计测量及随机游走噪声。
    Eigen::Matrix<double, 9, 9> transition;
    Eigen::Matrix<double, 9, 6> noise_gain;
    transition.setIdentity();
    noise_gain.setZero();

    transition.block<3, 3>(0, 0) = rotation_increment.transpose();
    transition.block<3, 3>(3, 0) = -position_acceleration_gain * acceleration_cross;
    transition.block<3, 3>(3, 6) = kIdentity3 * interval;
    transition.block<3, 3>(6, 0) = -velocity_acceleration_gain * acceleration_cross;

    noise_gain.block<3, 3>(0, 0) = rotation_jacobian * interval;
    noise_gain.block<3, 3>(3, 3) = position_acceleration_gain;
    noise_gain.block<3, 3>(6, 3) = velocity_acceleration_gain;

    covariance_.block<9, 9>(0, 0) =
        transition * covariance_.block<9, 9>(0, 0) * transition.transpose() +
        (source_interval / interval) * noise_gain * measurement_noise_ * noise_gain.transpose();
    covariance_.block<6, 6>(9, 9) += bias_noise_ * interval;

    // 更新积分后的相对位置、速度和旋转。
    position_delta_ += velocity_delta_ * interval + position_acceleration_gain * acceleration;
    velocity_delta_ += velocity_acceleration_gain * acceleration;
    rotation_delta_ = rotation_delta_ * rotation_increment;
}

/// 检查数值正定性并缓存当前积分区间的信息矩阵。
void ImuPreintegration::prepareInformation() {
    if (information_ready_)
        return;
    // 每个积分区间只计算一次信息矩阵，偏置试探沿用冻结权重。
    information_ = inversePositiveDefinite(covariance_);
    information_ready_ = true;
}

/// 计算偏置修正后的旋转、位置、速度及偏置连续性残差。
ImuPreintegration::ResidualTerms ImuPreintegration::residualTerms(
    const WindowState& first_state, const WindowState& second_state) const {
    ResidualTerms terms;
    terms.value.setZero();
    Eigen::Matrix3d corrected_rotation =
        rotation_delta_ * windowRotation(rotation_gyro_jacobian_ * gyro_increment_);
    Eigen::Vector3d corrected_position = position_delta_ +
                                         position_gyro_jacobian_ * gyro_increment_ +
                                         position_accel_jacobian_ * accel_increment_;
    Eigen::Vector3d corrected_velocity = velocity_delta_ +
                                         velocity_gyro_jacobian_ * gyro_increment_ +
                                         velocity_accel_jacobian_ * accel_increment_;

    terms.rotation =
        corrected_rotation.transpose() * first_state.rotation.transpose() * second_state.rotation;
    terms.expected_velocity =
        first_state.rotation.transpose() *
        (second_state.velocity - first_state.velocity - duration_ * first_state.gravity);
    Eigen::Vector3d velocity_residual = terms.expected_velocity - corrected_velocity;
    terms.expected_position =
        first_state.rotation.transpose() *
        (second_state.position - first_state.position - first_state.velocity * duration_ -
         0.5 * duration_ * duration_ * first_state.gravity);
    Eigen::Vector3d position_residual = terms.expected_position - corrected_position;

    Eigen::Vector3d gyro_bias_residual = second_state.gyro_bias - first_state.gyro_bias;
    Eigen::Vector3d accel_bias_residual = second_state.accel_bias - first_state.accel_bias;

    terms.value.block<3, 1>(0, 0) = logRotation(Eigen::Quaterniond(terms.rotation));
    terms.value.block<3, 1>(3, 0) = position_residual;
    terms.value.block<3, 1>(6, 0) = velocity_residual;
    terms.value.block<3, 1>(9, 0) = gyro_bias_residual;
    terms.value.block<3, 1>(12, 0) = accel_bias_residual;

    return terms;
}

/// 组装两帧右扰动和公共重力对十五维残差的 Jacobian。
ImuPreintegration::FactorJacobian ImuPreintegration::factorJacobian(
    const WindowState& first_state, const WindowState& second_state,
    const ResidualTerms& terms) const {
    Mat15 first_jacobian = Mat15::Zero(), second_jacobian = Mat15::Zero();
    Eigen::Matrix<double, kStateDimension, 3> gravity_jacobian =
        Eigen::Matrix<double, kStateDimension, 3>::Zero();
    Eigen::Matrix3d inverse_rotation_jacobian = inverseRightJacobian(terms.rotation);
    first_jacobian.block<3, 3>(0, 0) =
        -inverse_rotation_jacobian * second_state.rotation.transpose() * first_state.rotation;
    second_jacobian.block<3, 3>(0, 0) = inverse_rotation_jacobian;
    first_jacobian.block<3, 3>(0, 9) = -inverse_rotation_jacobian * terms.rotation.transpose() *
                                       rightJacobian(rotation_gyro_jacobian_ * gyro_increment_) *
                                       rotation_gyro_jacobian_;

    // 组装平移和速度残差对两帧状态的导数。
    first_jacobian.block<3, 3>(3, 0) = skew(terms.expected_position);
    first_jacobian.block<3, 3>(3, 3) = -first_state.rotation.transpose();
    first_jacobian.block<3, 3>(3, 6) = -first_state.rotation.transpose() * duration_;
    first_jacobian.block<3, 3>(3, 9) = -position_gyro_jacobian_;
    first_jacobian.block<3, 3>(3, 12) = -position_accel_jacobian_;
    second_jacobian.block<3, 3>(3, 3) = first_state.rotation.transpose();

    first_jacobian.block<3, 3>(6, 0) = skew(terms.expected_velocity);
    first_jacobian.block<3, 3>(6, 6) = -first_state.rotation.transpose();
    first_jacobian.block<3, 3>(6, 9) = -velocity_gyro_jacobian_;
    first_jacobian.block<3, 3>(6, 12) = -velocity_accel_jacobian_;
    second_jacobian.block<3, 3>(6, 6) = first_state.rotation.transpose();

    first_jacobian.block<3, 3>(9, 9) = -kIdentity3;
    first_jacobian.block<3, 3>(12, 12) = -kIdentity3;
    second_jacobian.block<3, 3>(9, 9) = kIdentity3;
    second_jacobian.block<3, 3>(12, 12) = kIdentity3;

    gravity_jacobian.block<3, 3>(3, 0) =
        first_state.rotation.transpose() * (-0.5 * duration_ * duration_);
    gravity_jacobian.block<3, 3>(6, 0) = first_state.rotation.transpose() * (-duration_);

    Eigen::Matrix<double, kStateDimension, 2 * kStateDimension + 3> jacobian;
    jacobian.block<kStateDimension, kStateDimension>(0, 0) = first_jacobian;
    jacobian.block<kStateDimension, kStateDimension>(0, kStateDimension) = second_jacobian;
    jacobian.block<kStateDimension, 3>(0, 2 * kStateDimension) = gravity_jacobian;

    return jacobian;
}

/// 使用冻结权重组装因子的法方程并返回代价。
double ImuPreintegration::linearize(const WindowState& first_state, const WindowState& second_state,
                                    Eigen::MatrixXd& information, Eigen::VectorXd& gradient) {
    prepareInformation();
    const auto terms = residualTerms(first_state, second_state);
    const auto jacobian = factorJacobian(first_state, second_state, terms);
    information = jacobian.transpose() * information_ * jacobian;
    gradient = jacobian.transpose() * information_ * terms.value;
    return terms.value.dot(information_ * terms.value);
}

/// 计算当前试探状态的加权残差代价。
double ImuPreintegration::evaluateCost(const WindowState& first_state,
                                       const WindowState& second_state) {
    prepareInformation();
    const auto terms = residualTerms(first_state, second_state);
    return terms.value.dot(information_ * terms.value);
}

/// 保存回滚点并加入本次试探偏置变化。
void ImuPreintegration::applyBiasIncrement(
    const Eigen::Matrix<double, kStateDimension, 1>& increment) {
    previous_gyro_increment_ = gyro_increment_;
    previous_accel_increment_ = accel_increment_;

    gyro_increment_ += increment.block<3, 1>(9, 0);
    accel_increment_ += increment.block<3, 1>(12, 0);
}
}  // namespace

struct StartupInertialModel::Impl {
    StartupOptimizationConfig config;
    AccelerationBiasPrior prior;
    std::vector<ImuPreintegration> factors;
    /// 保存窗口惯性配置及首帧偏置先验。
    Impl(const StartupOptimizationConfig& value, const Vec3& bias, const Vec3& deviations)
        : config(value), prior(bias, deviations) {}
};

/// 建立仅由本窗口模型拥有的预积分与先验实现。
StartupInertialModel::StartupInertialModel(const StartupOptimizationConfig& config,
                                           const Vec3& bias, const Vec3& deviations)
    : impl_(std::make_unique<Impl>(config, bias, deviations)) {
}
/// 释放窗口全部预积分和先验实现。
StartupInertialModel::~StartupInertialModel() = default;

/// 按新外层状态偏置重建相邻帧预积分。
void StartupInertialModel::rebuild(const std::vector<WindowState>& states,
                                   const std::vector<WindowFrame>& frames) {
    impl_->factors.clear();
    impl_->factors.reserve(states.size() - 1);
    for (std::size_t i = 1; i < states.size(); ++i) {
        impl_->factors.emplace_back(states[i - 1].gyro_bias, states[i - 1].accel_bias,
                                    impl_->config.noise);
        impl_->factors.back().integrate(frames[i].imus);
    }
}

/// 清零并组装完整惯性系统，最后统一缩放权重和半平方代价。
double StartupInertialModel::linearize(const std::vector<WindowState>& states,
                                       Eigen::MatrixXd& hessian, Eigen::VectorXd& gradient) {
    hessian.setZero();
    gradient.setZero();
    const int frame_count = states.size();
    const int system_dimension = kStateDimension * frame_count + 3;
    auto& inertial = impl_->factors;
    double residual = 0;
    Eigen::MatrixXd factor_information(2 * kStateDimension + 3, 2 * kStateDimension + 3);
    Eigen::VectorXd factor_gradient(2 * kStateDimension + 3);

    for (int i = 0; i < frame_count - 1; i++) {
        residual +=
            inertial[i].linearize(states[i], states[i + 1], factor_information, factor_gradient);
        hessian.block<kStateDimension * 2, kStateDimension * 2>(i * kStateDimension,
                                                                i * kStateDimension) +=
            factor_information.block<2 * kStateDimension, 2 * kStateDimension>(0, 0);
        hessian.block<kStateDimension * 2, 3>(i * kStateDimension, system_dimension - 3) +=
            factor_information.block<2 * kStateDimension, 3>(0, 2 * kStateDimension);
        hessian.block<3, kStateDimension * 2>(system_dimension - 3, i * kStateDimension) +=
            factor_information.block<3, 2 * kStateDimension>(2 * kStateDimension, 0);
        hessian.block<3, 3>(system_dimension - 3, system_dimension - 3) +=
            factor_information.block<3, 3>(2 * kStateDimension, 2 * kStateDimension);

        gradient.block<kStateDimension * 2, 1>(i * kStateDimension, 0) +=
            factor_gradient.head(2 * kStateDimension);
        gradient.tail(3) += factor_gradient.tail(3);
    }

    // 加入首帧偏置先验并统一缩放惯性权重。
    impl_->prior.accumulate(states.front(), hessian, gradient);
    residual += impl_->prior.squaredError(states.front());
    hessian *= impl_->config.imu_weight;
    gradient *= impl_->config.imu_weight;
    residual *= (impl_->config.imu_weight * 0.5);

    return residual;
}

/// 求当前预积分及唯一首帧先验的加权半平方代价。
double StartupInertialModel::evaluateCost(const std::vector<WindowState>& states) {
    double cost = 0;
    for (std::size_t i = 0; i < impl_->factors.size(); ++i)
        cost += impl_->factors[i].evaluateCost(states[i], states[i + 1]);
    cost += impl_->prior.squaredError(states.front());
    return cost * (impl_->config.imu_weight * .5);
}

/// 按窗口索引为每个起始帧预积分加入试探偏置增量。
void StartupInertialModel::applyBiasIncrement(const Eigen::VectorXd& increment) {
    for (std::size_t i = 0; i < impl_->factors.size(); ++i)
        impl_->factors[i].applyBiasIncrement(
            increment.segment<kStateDimension>(kStateDimension * i));
}

/// 恢复全部帧间因子在最近一次试探前的偏置修正。
void StartupInertialModel::rollbackBiasIncrement() {
    for (auto& factor : impl_->factors)
        factor.rollbackBiasIncrement();
}

}  // namespace zlio
