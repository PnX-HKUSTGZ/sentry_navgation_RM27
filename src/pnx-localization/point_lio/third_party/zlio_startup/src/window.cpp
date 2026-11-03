/**
 * @file window.cpp
 * @brief 协调启动窗口模型、并行分区与固定规范下的阻尼求解和试探回滚。
 */
#include "zlio/window.hpp"

#include <Eigen/Dense>
#include <limits>
#include <optional>
#include <stdexcept>

#include "bootstrap.hpp"
#include "execution.hpp"
#include "solver.hpp"
#include "zlio/config.hpp"
#include "zlio/state.hpp"

namespace zlio {
namespace {
inline constexpr int kStateDimension = 15;
inline constexpr int kPoseDimension = 6;
struct OptimizationCosts {
    double initial = 0, final = 0;
    OptimizationStatus status = OptimizationStatus::NoProgress;
};

class PlanePartitions {
public:
    struct Result {
        Eigen::MatrixXd hessian;
        Eigen::VectorXd gradient;
        double cost = 0;
    };
    /// 建立本次启动尝试的分区结果存储及执行线程。
    explicit PlanePartitions(int count)
        : results_(checkedCount(count)), executor_(results_.size()) {}

    /// 并行求值几何分区，并在主分区计算惯性项。
    template <class InertialWork>
    double evaluate(StartupPlaneModel& planes, const std::vector<WindowState>& states,
                    bool derivatives, InertialWork&& inertial_work) {
        active_ = planes.size() < results_.size() ? 1 : results_.size();
        if (planes.size() > static_cast<std::size_t>(std::numeric_limits<int>::max()))
            throw std::length_error("initialization plane count exceeds the index range");
        if (derivatives)
            for (auto& result : results_) {
                result.hessian.resize(6 * states.size(), 6 * states.size());
                result.gradient.resize(6 * states.size());
            }
        double residual = 0;
        executor_.run(active_, [&](std::size_t index) {
            if (index == 0)
                residual = inertial_work();
            auto& result = results_[index];
            const int begin = partitionBoundary(planes.size(), active_, index);
            const int end = partitionBoundary(planes.size(), active_, index + 1);
            if (derivatives)
                planes.linearize(states, begin, end, result.hessian, result.gradient, result.cost);
            else
                planes.evaluateCost(states, begin, end, result.cost);
        });
        return residual;
    }
    /// 返回本次实际使用的几何分区数量。
    std::size_t active() const { return active_; }
    /// 访问已完成分区的结果供主线程按序归并。
    Result& operator[](std::size_t index) { return results_[index]; }

private:
    /// 检查包含主线程在内的分区数量为正。
    static std::size_t checkedCount(int count) {
        if (count <= 0)
            throw std::invalid_argument("initialization partition count must be positive");
        return count;
    }
    std::vector<Result> results_;
    BatchExecutor executor_;
    std::size_t active_ = 0;
};

class StartupOptimizer {
public:
    /// 固定求解配置并建立跨外层复用的几何工作区。
    explicit StartupOptimizer(const StartupOptimizationConfig& config)
        : workers_(config.thread_count) {}
    /// 在固定首帧位姿和重力模长下执行有限次阻尼试探。
    OptimizationCosts optimize(std::vector<WindowState>& states, StartupPlaneModel& planes,
                               StartupInertialModel& inertial, int maximum_iterations);

private:
    int frame_count_ = 0;
    PlanePartitions workers_;
    /// 把紧凑位姿块嵌入十五维窗口法方程。
    void addPoseBlocks(Eigen::MatrixXd& hessian, Eigen::VectorXd& gradient,
                       Eigen::MatrixXd& pose_information, Eigen::VectorXd& pose_gradient);
    /// 统一组装当前状态下的惯性和几何系统。
    double linearize(std::vector<WindowState>& states, StartupPlaneModel& planes,
                     StartupInertialModel& inertial, Eigen::MatrixXd& hessian,
                     Eigen::VectorXd& gradient);
    /// 计算一次试探总代价并更新平面缓存。
    double evaluateCost(std::vector<WindowState>& states, StartupPlaneModel& planes,
                        StartupInertialModel& inertial);
};

/// 将几何位姿 Hessian 和梯度累加到导航系统。
void StartupOptimizer::addPoseBlocks(Eigen::MatrixXd& hessian, Eigen::VectorXd& gradient,
                                     Eigen::MatrixXd& pose_information,
                                     Eigen::VectorXd& pose_gradient) {
    for (int i = 0; i < frame_count_; i++) {
        gradient.block<kPoseDimension, 1>(i * kStateDimension, 0) +=
            pose_gradient.block<kPoseDimension, 1>(i * kPoseDimension, 0);
        for (int j = 0; j < frame_count_; j++)
            hessian.block<kPoseDimension, kPoseDimension>(i * kStateDimension,
                                                          j * kStateDimension) +=
                pose_information.block<kPoseDimension, kPoseDimension>(i * kPoseDimension,
                                                                       j * kPoseDimension);
    }
}

/// 并行求值几何分区并按固定顺序归并惯性和几何导数。
double StartupOptimizer::linearize(std::vector<WindowState>& states, StartupPlaneModel& planes,
                                   StartupInertialModel& inertial, Eigen::MatrixXd& hessian,
                                   Eigen::VectorXd& gradient) {
    double residual = workers_.evaluate(
        planes, states, true, [&] { return inertial.linearize(states, hessian, gradient); });
    for (std::size_t i = 0; i < workers_.active(); ++i) {
        auto& result = workers_[i];
        addPoseBlocks(hessian, gradient, result.hessian, result.gradient);
        residual += result.cost;
    }

    return residual;
}

/// 并行计算几何代价并与惯性代价合并。
double StartupOptimizer::evaluateCost(std::vector<WindowState>& states, StartupPlaneModel& planes,
                                      StartupInertialModel& inertial) {
    const double inertial_cost =
        workers_.evaluate(planes, states, false, [&] { return inertial.evaluateCost(states); });
    double plane_cost = 0;
    for (std::size_t i = 0; i < workers_.active(); ++i)
        plane_cost += workers_[i].cost;
    return inertial_cost + plane_cost;
}

struct ConstrainedStep {
    Eigen::VectorXd increment;
    double predicted_reduction = 0;
    bool stationary = false;
    bool valid = false;
};

/// 投影固定规范、检查驻点并求解带对角阻尼的切空间增量。
ConstrainedStep constrainedIncrement(const std::vector<WindowState>& states,
                                     const Eigen::MatrixXd& hessian,
                                     const Eigen::VectorXd& gradient, double damping, double cost) {
    if (!hessian.allFinite() || !gradient.allFinite() || !std::isfinite(cost))
        throw NumericalError("initialization optimization system contains non-finite values");
    const auto gravity_basis = gravityTangent(states.front().gravity);
    const Eigen::Index gravity_offset = hessian.rows() - 3;
    const Eigen::Index free_dimension = gravity_offset - 6;
    Eigen::MatrixXd reduced_hessian = projectStartupMatrix(hessian, gravity_basis);
    Eigen::VectorXd reduced_gradient(free_dimension + 2);
    reduced_gradient.head(free_dimension) = gradient.segment(6, free_dimension);
    reduced_gradient.tail<2>().noalias() = gravity_basis.transpose() * gradient.tail<3>();
    if (!reduced_hessian.allFinite() || !reduced_gradient.allFinite())
        throw NumericalError("initialization optimization system contains non-finite values");
    ConstrainedStep result;
    // 按对角曲率缩放切空间梯度并检查驻点。
    const double gradient_norm =
        (reduced_gradient.array().abs() / (1 + reduced_hessian.diagonal().array().abs().sqrt()))
            .maxCoeff();
    if (gradient_norm <= 1e-6 * std::max(1., std::sqrt(std::abs(cost)))) {
        result.increment = Eigen::VectorXd::Zero(hessian.rows());
        result.stationary = true;
        return result;
    }
    const auto trial = dampedStartupStep(reduced_hessian, reduced_gradient, damping);
    if (!trial.valid)
        return result;
    const auto& step = trial.increment;
    result.increment = Eigen::VectorXd::Zero(hessian.rows());
    result.increment.segment(6, free_dimension) = step.head(free_dimension);
    result.increment.tail<3>().noalias() = gravity_basis * step.tail<2>();
    if (!result.increment.allFinite())
        throw NumericalError("initialization optimization increment contains non-finite values");

    result.predicted_reduction = trial.predicted_reduction;
    result.valid = true;
    return result;
}
/// 注入全部试探导航增量并将公共重力投回固定模长球面。
void applyIncrement(const std::vector<WindowState>& states, const Eigen::VectorXd& increment,
                    double gravity_norm, std::vector<WindowState>& trial_states) {
    trial_states[0].gravity = gravity_norm * (states[0].gravity + increment.tail(3)).normalized();
    for (int j = 0; j < static_cast<int>(states.size()); j++) {
        applyWindowIncrement(states[j], increment.segment<kStateDimension>(kStateDimension * j),
                             trial_states[j]);
        trial_states[j].gravity = trial_states[0].gravity;
    }
}

struct DampingPolicy {
    double value = .01, rejection_multiplier = 2;
    /// 依据接受步的增益比收缩阻尼并重置拒绝倍率。
    void accept(double gain_ratio) {
        const double minimum_factor = 1.0 / 3;
        rejection_multiplier = 2;
        const double factor = 1 - std::pow(2 * gain_ratio - 1, 3);
        value *= (factor < minimum_factor ? minimum_factor : factor);
    }
    /// 增加阻尼及下一次试探的拒绝倍率。
    void reject() {
        value = value * rejection_multiplier;
        rejection_multiplier = 2 * rejection_multiplier;
    }
};

/// 执行接受与回滚流程，退出前确保几何缓存对应已接受状态。
OptimizationCosts StartupOptimizer::optimize(std::vector<WindowState>& states,
                                             StartupPlaneModel& planes,
                                             StartupInertialModel& inertial,
                                             int maximum_iterations) {
    OptimizationCosts costs;
    frame_count_ = planes.frameCount();
    const int system_dimension = frame_count_ * kStateDimension + 3;
    DampingPolicy damping;
    Eigen::MatrixXd hessian(system_dimension, system_dimension);
    Eigen::VectorXd gradient(system_dimension), increment(system_dimension);

    const double gravity_norm = states.front().gravity.norm();
    double reference_cost = 0, trial_cost;
    bool relinearize = true;
    std::optional<double> current_cost;
    std::vector<WindowState> trial_states = states;

    constexpr int kMaximumRejectedTrials = 8;
    int rejected_trials = 0;
    // 在线性化不变的局部模型内调整阻尼并尝试下降步。
    for (int accepted_steps = 0; accepted_steps < maximum_iterations;) {
        if (relinearize) {
            reference_cost = linearize(states, planes, inertial, hessian, gradient);
            current_cost = reference_cost;
        }

        if (accepted_steps == 0)
            costs.initial = reference_cost;

        const auto step =
            constrainedIncrement(states, hessian, gradient, damping.value, reference_cost);
        if (step.stationary) {
            costs.status = OptimizationStatus::Converged;
            break;
        }
        if (!step.valid) {
            damping.reject();
            relinearize = false;
            if (++rejected_trials >= kMaximumRejectedTrials)
                break;
            continue;
        }
        increment = step.increment;
        applyIncrement(states, increment, gravity_norm, trial_states);

        inertial.applyBiasIncrement(increment);

        trial_cost = evaluateCost(trial_states, planes, inertial);
        current_cost.reset();  // 平面缓存现在属于试探状态。
        const double actual_reduction = reference_cost - trial_cost;

        if (actual_reduction > 0 && std::isfinite(trial_cost) && step.predicted_reduction > 0 &&
            std::isfinite(step.predicted_reduction)) {
            ++accepted_steps;
            rejected_trials = 0;
            states = trial_states;
            current_cost = trial_cost;
            costs.status = OptimizationStatus::Progress;
            damping.accept(actual_reduction / step.predicted_reduction);
            relinearize = true;
            if (actual_reduction <= 1e-6 * std::max(std::abs(reference_cost), 1e-12)) {
                costs.status = OptimizationStatus::Converged;
                break;
            }
        } else {
            damping.reject();
            relinearize = false;

            inertial.rollbackBiasIncrement();
            if (++rejected_trials >= kMaximumRejectedTrials)
                break;
        }
    }
    // 恢复最终接受状态对应的几何代价和缓存。
    costs.final = current_cost ? *current_cost : evaluateCost(states, planes, inertial);
    return costs;
}
}  // namespace

/// 按启动右扰动的小角度约定将旋转向量映射为旋转矩阵。
Mat3 windowRotation(const Eigen::Vector3d& rotation_vector) {
    const Mat3 identity = Mat3::Identity();
    double angle = rotation_vector.norm();
    if (angle >= 1e-11) {
        Eigen::Vector3d axis = rotation_vector / angle;
        Eigen::Matrix3d cross_matrix;
        cross_matrix = skew(axis);
        return identity + std::sin(angle) * cross_matrix +
               (1.0 - std::cos(angle)) * cross_matrix * cross_matrix;
    }

    return identity;
}

/// 注入启动右扰动导航增量，保持输出重力及协方差不变。
void applyWindowIncrement(const WindowState& reference, const Vec15& increment,
                          WindowState& result) {
    result.rotation = reference.rotation * windowRotation(increment.segment<3>(0));
    result.position = reference.position + increment.segment<3>(3);
    result.velocity = reference.velocity + increment.segment<3>(6);
    result.gyro_bias = reference.gyro_bias + increment.segment<3>(9);
    result.accel_bias = reference.accel_bias + increment.segment<3>(12);
}

/// 构造单位姿态与零值窗口状态。
WindowState::WindowState() {
    setZero();
}

/// 将十五维右扰动增量原位注入窗口状态。
WindowState& WindowState::operator+=(const Eigen::Matrix<double, kStateDimension, 1>& increment) {
    applyWindowIncrement(*this, increment, *this);
    return *this;
}

/// 计算相对参考状态的右旋转及线性导航增量。
Vec15 WindowState::operator-(const WindowState& reference) const {
    Vec15 difference;
    difference.segment<3>(0) =
        logRotation(Eigen::Quaterniond(reference.rotation.transpose() * rotation));
    difference.segment<3>(3) = position - reference.position;
    difference.segment<3>(6) = velocity - reference.velocity;
    difference.segment<3>(9) = gyro_bias - reference.gyro_bias;
    difference.segment<3>(12) = accel_bias - reference.accel_bias;
    return difference;
}

/// 恢复单位姿态并清零导航量、重力及协方差。
void WindowState::setZero() {
    rotation.setIdentity();
    position.setZero();
    velocity.setZero();
    gyro_bias.setZero();
    accel_bias.setZero();
    gravity.setZero();
    covariance.setZero();
}

struct WindowOptimizer::Impl {
    StartupOptimizationConfig config;
    StartupInertialModel inertial;
    StartupPlaneModel planes;
    StartupOptimizer solver;
    /// 建立窗口约束及其阻尼求解器。
    Impl(const StartupOptimizationConfig& value, const Vec3& mean, const Vec3& deviations)
        : config(value), inertial(config, mean, deviations), planes(config), solver(config) {}
};

/// 固定本次初始化尝试的配置及偏置先验。
WindowOptimizer::WindowOptimizer(const StartupOptimizationConfig& config, const Vec3& accel_bias,
                                 const Vec3& accel_bias_std)
    : impl_(std::make_unique<Impl>(config, accel_bias, accel_bias_std)) {
}
/// 先回收求解线程，再释放窗口模型。
WindowOptimizer::~WindowOptimizer() = default;

/// 检查几何支撑并重建预积分，执行一次外层窗口优化。
OptimizationStep WindowOptimizer::refine(std::vector<WindowState>& states,
                                         const std::vector<WindowFrame>& frames,
                                         const std::vector<std::vector<Vec3>>& local_points,
                                         bool refined) {
    if (states.size() < 2 || states.size() != frames.size() || states.size() != local_points.size())
        throw std::invalid_argument("window state and observation counts do not match");
    impl_->planes.rebuild(local_points, states, refined);
    OptimizationStep result;
    result.planes = impl_->planes.size();
    if (result.planes < impl_->config.minimum_planes)
        return result;
    if (!impl_->planes.connected()) {
        result.status = OptimizationStatus::DisconnectedWindow;
        return result;
    }
    impl_->inertial.rebuild(states, frames);
    const auto costs = impl_->solver.optimize(states, impl_->planes, impl_->inertial,
                                              impl_->config.maximum_iterations);
    result.initial_cost = costs.initial;
    result.final_cost = costs.final;
    result.status = costs.status;
    return result;
}

/// 返回当前已接受几何缓存的方向支撑。
Mat3 WindowOptimizer::normalSupport() const {
    return impl_->planes.normalSupport();
}

}  // namespace zlio
