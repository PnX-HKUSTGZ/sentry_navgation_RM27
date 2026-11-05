/**
 * @file initialization.cpp
 * @brief 管理启动窗口收集、粗配准、重去畸变、优化接受及状态地图交接。
 */
#include "zlio/initialization.hpp"

#include <Eigen/Eigenvalues>
#include <algorithm>
#include <iterator>
#include <limits>
#include <unordered_map>

#include "bootstrap.hpp"
#include "covariance.hpp"
#include "registration.hpp"
#include "zlio/inertial.hpp"
#include "zlio/state.hpp"
#include "zlio/voxel.hpp"
#include "zlio/window.hpp"

namespace zlio {
namespace {
/// 返回初始化状态对应的诊断说明。
const char* initializationReason(InitializationState state) {
    switch (state) {
        case InitializationState::Collecting:
            return "collecting initialization data";
        case InitializationState::NotConverged:
            return "initialization window has not converged";
        case InitializationState::InsufficientPlanes:
            return "insufficient valid cross-frame planes";
        case InitializationState::DisconnectedWindow:
            return "initialization cross-frame geometry is disconnected";
        case InitializationState::NoProgress:
            return "initialization optimization made no effective progress";
        case InitializationState::NonfiniteState:
            return "initialization state contains non-finite values";
        case InitializationState::WeakGeometry:
            return "insufficient support in the weakest initialization geometry direction";
        case InitializationState::Ready:
            return "initialization complete";
        case InitializationState::Failed:
            return "initialization failed";
        case InitializationState::WeakInformation:
            return "insufficient joint initialization information";
    }
    throw std::logic_error("unknown initialization status");
}

/// 每格选取最接近点集中心的原始回波并按获取时间排序。
Scan sampleOriginal(const Scan& scan, double size) {
    struct Cell {
        Vec3 sum = Vec3::Zero();
        std::size_t count = 0, best = 0;
        double distance = std::numeric_limits<double>::infinity();
    };
    std::unordered_map<VoxelKey, std::size_t, VoxelHash> cells;
    std::vector<Cell> statistics;
    std::vector<std::size_t> cell_indices;
    cell_indices.reserve(scan.size());
    for (std::size_t i = 0; i < scan.size(); ++i) {
        auto [entry, added] =
            cells.try_emplace(voxelKey(scan[i].position, size), statistics.size());
        if (added) {
            statistics.emplace_back();
            statistics.back().best = i;
        }
        auto& cell = statistics[entry->second];
        cell.sum += scan[i].position;
        ++cell.count;
        cell_indices.push_back(entry->second);
    }
    for (auto& cell : statistics)
        cell.sum /= cell.count;
    // 选择距离格内点集质心最近的原始回波。
    for (std::size_t i = 0; i < scan.size(); ++i) {
        auto& cell = statistics[cell_indices[i]];
        const double distance = (scan[i].position - cell.sum).squaredNorm();
        if (distance < cell.distance) {
            cell.distance = distance;
            cell.best = i;
        }
    }
    Scan result;
    result.reserve(cells.size());
    for (const auto& entry : cells) {
        result.push_back(scan[statistics[entry.second].best]);
    }
    std::sort(result.begin(), result.end(),
              [](const auto& a, const auto& b) { return a.time < b.time; });
    return result;
}

/// 以完整同步 IMU 包推进粗状态及其右扰动协方差。
void propagate(WindowState& state, const std::vector<Imu>& imus, const RegistrationConfig& config) {
    for (std::size_t i = 1; i < imus.size(); ++i) {
        const double dt = seconds(imus[i].time - imus[i - 1].time);
        const double source_dt = seconds(originalImuInterval(imus[i - 1], imus[i]));
        const Vec3 omega =
            .5 * (imus[i - 1].angular_velocity + imus[i].angular_velocity) - state.gyro_bias;
        const Vec3 acc = .5 * (imus[i - 1].acceleration + imus[i].acceleration) - state.accel_bias;
        Mat15 F = Mat15::Identity(), Q = Mat15::Zero();
        F.block<3, 3>(0, 0) = rotationIncrement(omega, -dt);
        F.block<3, 3>(0, 9) = -Mat3::Identity() * dt;
        F.block<3, 3>(3, 6) = Mat3::Identity() * dt;
        F.block<3, 3>(6, 0) = -state.rotation * skew(acc) * dt;
        F.block<3, 3>(6, 12) = -state.rotation * dt;
        Q.block<3, 3>(0, 0).diagonal().setConstant(config.gyro_variance * dt * source_dt);
        Q.block<3, 3>(6, 6).diagonal().setConstant(config.accel_variance * dt * source_dt);
        Q.block<6, 6>(9, 9).diagonal().setConstant(config.bias_variance * dt * source_dt);
        state.covariance = F * state.covariance * F.transpose() + Q;
        const Vec3 world_acc = state.rotation * acc + state.gravity;
        state.position += state.velocity * dt + .5 * world_acc * dt * dt;
        state.velocity += world_acc * dt;
        state.rotation = state.rotation * rotationIncrement(omega, dt);
    }
}

/// 从扫描末状态反向建立名义轨迹，并向消费者提供参考 IMU 系点及噪声旋转。
template <typename Consumer>
void deskew(const Scan& scan, const std::vector<Imu>& imus, const WindowState& end,
            const WindowState& previous, const Pose& extrinsic, Consumer consume) {
    struct Segment {
        Time time;
        Mat3 rotation;
        Vec3 position, velocity, omega, acceleration;
    };
    // 从扫描末时刻反向积分各段名义轨迹。
    std::vector<Segment> segments;
    WindowState x = end;
    for (std::size_t i = imus.size() - 1; i > 0; --i) {
        const double dt = seconds(imus[i - 1].time - imus[i].time);
        const Vec3 omega =
            .5 * (imus[i - 1].angular_velocity + imus[i].angular_velocity) - previous.gyro_bias;
        const Vec3 acc =
            .5 * (imus[i - 1].acceleration + imus[i].acceleration) - previous.accel_bias;
        const Vec3 world_acc = x.rotation * acc + x.gravity;
        x.position += x.velocity * dt + .5 * world_acc * dt * dt;
        x.velocity += world_acc * dt;
        x.rotation = x.rotation * rotationIncrement(omega, dt);
        segments.push_back(
            {imus[i - 1].time, x.rotation, x.position, x.velocity, omega, world_acc});
    }
    std::reverse(segments.begin(), segments.end());
    // 插值每个回波的位姿并转换到参考 IMU 系。
    std::size_t segment = 0;
    for (const auto& point : scan) {
        if (point.time < imus.front().time || point.time > imus.back().time)
            throw std::logic_error(
                "initialization return timestamp is outside the synchronized IMU interval");
        while (segment + 1 < segments.size() && segments[segment + 1].time <= point.time)
            ++segment;
        const auto& s = segments[segment];
        const double dt = seconds(point.time - s.time);
        const Vec3 world =
            s.rotation * rotationIncrement(s.omega, dt) * extrinsic.apply(point.position) +
            s.position + s.velocity * dt + .5 * s.acceleration * dt * dt;
        const Mat3 rotation = end.rotation.transpose() * s.rotation *
                              rotationIncrement(s.omega, dt) *
                              extrinsic.rotation.toRotationMatrix();
        consume(point, end.rotation.transpose() * (world - end.position), rotation);
    }
}

/// 将带真实获取时间的原始回波去畸变到末帧 IMU 系。
std::vector<Vec3> deskewPoints(const Scan& scan, const std::vector<Imu>& imus,
                               const WindowState& end, const WindowState& previous,
                               const Pose& extrinsic) {
    std::vector<Vec3> points;
    points.reserve(scan.size());
    deskew(scan, imus, end, previous, extrinsic,
           [&](const Point&, const Vec3& point, const Mat3&) { points.push_back(point); });
    return points;
}

/// 统一旋转窗口世界坐标，使重力朝负 z 轴并以首帧为原点。
void alignGravity(std::vector<WindowState>& states) {
    const Mat3 rotation = Eigen::Quaterniond::FromTwoVectors(states.front().gravity, -Vec3::UnitZ())
                              .toRotationMatrix();
    const Vec3 origin = states.front().position;
    for (auto& state : states) {
        state.position = rotation * (state.position - origin);
        state.rotation = rotation * state.rotation;
        state.velocity = rotation * state.velocity;
        state.gravity = rotation * state.gravity;
    }
}

/// 把启动窗口状态转换为指定真实时刻的运行导航状态。
State navigationState(const WindowState& state, Time time) {
    return {time,
            {Eigen::Quaterniond(state.rotation).normalized(), state.position},
            state.velocity,
            state.gyro_bias,
            state.accel_bias};
}
}  // namespace
/// 按配置直接构造导航误差坐标中的对角先验。
Mat15 initialCovariance(const InitialUncertainty& uncertainty) {
    Mat15 covariance = Mat15::Zero();
    for (int block = 0; block < 5; ++block)
        covariance.block<3, 3>(3 * block, 3 * block).diagonal() = uncertainty.variance[block];

    return covariance;
}
struct Initializer::Impl {
    /// 保存初始化配置并建立可清窗重试的粗启动状态。
    explicit Impl(const Config& value)
        : config_(value),
          noise_(value.map.range_sigma, value.map.angle_sigma_degrees),
          registration_(value.initialization.registration) {
        resetCoarse();
    }
    /// 返回最近一次启动决策的状态说明。
    const InitializationStatus& status() const { return status_; }
    /// 消费完整同步扫描包，窗口满足要求时尝试初始化。
    std::optional<InitializationResult> add(const Scan& scan, Time end,
                                            const std::vector<Imu>& imus);

private:
    Config config_;
    RayMeasurementCovariance noise_;
    WindowState coarse_;
    Registration registration_;
    std::vector<WindowFrame> frames_;
    std::vector<WindowState> states_;
    std::size_t mean_count_ = 0;
    Vec3 mean_acc_ = Vec3::Zero();
    InitializationStatus status_;
    /// 更新初始化状态及其说明。
    void setStatus(InitializationState state, const char* detail = nullptr) {
        std::string reason = initializationReason(state);
        if (detail && *detail)
            reason += ": " + std::string(detail);
        status_ = {std::move(reason), state};
    }
    /// 按配置恢复粗状态、偏置初值、重力模长及粗协方差。
    void resetCoarse() {
        coarse_ = WindowState();
        coarse_.covariance =
            Mat15::Identity() * config_.initialization.registration.initial_state_variance;
        coarse_.covariance.bottomRightCorner<6, 6>() =
            Eigen::Matrix<double, 6, 6>::Identity() *
            config_.initialization.registration.initial_bias_variance;
        coarse_.accel_bias = config_.initial_accel_bias;
        coarse_.gravity = Vec3(0, 0, -config_.gravity);
    }
    /// 释放当前尝试的观测、状态与粗地图索引。
    void clearWindow() {
        frames_.clear();
        states_.clear();
        registration_.clear();
    }
    /// 清除失败尝试并用当前同步包重新积累启动种子。
    void restart(const std::vector<Imu>& imus) {
        clearWindow();
        resetCoarse();
        mean_acc_.setZero();
        mean_count_ = 0;

        seed(imus);
    }
    /// 由去除偏置初值后的平均比力猜测固定模长重力方向。
    void seed(const std::vector<Imu>& imus) {
        for (std::size_t i = 1; i < imus.size(); ++i) {
            ++mean_count_;
            mean_acc_ += (imus[i].acceleration - mean_acc_) / mean_count_;
        }
        const Vec3 specific_force = mean_acc_ - config_.initial_accel_bias;
        if (specific_force.norm() > 1e-9)
            coarse_.gravity = -config_.gravity * specific_force.normalized();
    }
    /// 使用最新窗口状态重新去畸变全部启动帧。
    std::vector<std::vector<Vec3>> redeskew() const {
        std::vector<std::vector<Vec3>> clouds;
        for (std::size_t i = 0; i < frames_.size(); ++i)
            clouds.push_back(deskewPoints(frames_[i].raw, frames_[i].imus, states_[i],
                                          states_[i ? i - 1 : 0], config_.body_from_lidar));
        return clouds;
    }

    /// 以有效平面数提供数值下限，计算本轮相对代价变化。
    double relativeChange(const OptimizationStep& step) const {
        const double floor = 1e-9 * step.planes;
        const double change = std::abs(step.initial_cost - step.final_cost) /
                              std::max(std::abs(step.initial_cost), floor);
        if (!std::isfinite(change))
            throw NumericalError("initialization optimization cost change is non-finite");
        return change;
    }
    /// 检查当前有效平面法向支撑的最弱方向。
    bool geometrySupported(const WindowOptimizer& optimizer) const {
        const Mat3 support = optimizer.normalSupport();
        const double minimum_support =
            Eigen::SelfAdjointEigenSolver<Mat3>(support).eigenvalues()[0];
        return minimum_support >= config_.initialization.minimum_directional_support;
    }
    /// 保存本次交替细化的阶段及几何接受条件。
    struct RefinementProgress {
        bool aligned = false;
        bool refined = false;
        bool geometry_supported = false;
        bool enough_planes = false;
    };
    /// 检查末窗有限性与阶段结果，并按优先级保存拒绝原因。
    bool acceptRefinement(const RefinementProgress& progress) {
        const bool finite = std::all_of(states_.begin(), states_.end(), [](const WindowState& x) {
            return x.rotation.allFinite() && x.position.allFinite() && x.velocity.allFinite() &&
                   x.gyro_bias.allFinite() && x.accel_bias.allFinite() && x.gravity.allFinite();
        });
        if (!finite)
            setStatus(InitializationState::NonfiniteState);
        else if (progress.aligned && !progress.geometry_supported &&
                 status_.state == InitializationState::NotConverged)
            setStatus(InitializationState::WeakGeometry);
        return finite && progress.refined && progress.geometry_supported && progress.enough_planes;
    }
    /// 交替重去畸变与粗细阶段优化，并执行有限性和几何接受检查。
    bool refine() {
        WindowOptimizer optimizer(config_.initialization.optimization, config_.initial_accel_bias,
                                  config_.initial_accel_bias_std);
        RefinementProgress progress;
        for (int iteration = 0; iteration < config_.initialization.maximum_refinements;
             ++iteration) {
            const auto step = optimizer.refine(states_, frames_, redeskew(), progress.aligned);
            progress.enough_planes =
                step.planes >= config_.initialization.optimization.minimum_planes;
            if (!progress.enough_planes) {
                setStatus(InitializationState::InsufficientPlanes);
                break;
            }
            if (step.status == OptimizationStatus::DisconnectedWindow) {
                setStatus(InitializationState::DisconnectedWindow);
                break;
            }
            if (step.status == OptimizationStatus::NoProgress) {
                setStatus(InitializationState::NoProgress);
                break;
            }
            const double change = relativeChange(step);
            const double tolerance =
                config_.initialization.relative_cost_tolerance[progress.aligned ? 1 : 0];
            if (iteration < 2 || change >= tolerance)
                continue;
            progress.geometry_supported = geometrySupported(optimizer);
            if (progress.aligned) {
                progress.refined = true;
                break;
            }
            alignGravity(states_);
            progress.aligned = true;
        }
        return acceptRefinement(progress);
    }
    /// 生成单帧去畸变观测及其测量协方差。
    std::vector<Observation> mapObservations(std::size_t index) const {
        const auto& frame = frames_[index];
        std::vector<Observation> observations;
        observations.reserve(frame.raw.size());
        deskew(frame.raw, frame.imus, states_[index], states_[index ? index - 1 : 0],
               config_.body_from_lidar,
               [&](const Point& raw, const Vec3& point, const Mat3& rotation) {
                   // 将雷达射线测量协方差旋转到参考 IMU 系。
                   const Mat3 covariance = noise_(raw.position);
                   observations.push_back(
                       {point, rotation * covariance * rotation.transpose(), raw.time, raw.id});
               });
        return observations;
    }
    /// 根据指定导航协方差生成单帧初始地图点。
    void appendMapFrame(InitializationResult& result, std::size_t index,
                        const Mat15* covariance = nullptr) const {
        const auto observations = mapObservations(index);
        Estimate estimate;
        estimate.state = navigationState(states_[index], frames_[index].end);
        estimate.covariance =
            covariance ? *covariance : initialCovariance(config_.initial_uncertainty);
        auto points = transformForMap(observations, estimate);
        result.map.insert(result.map.end(), std::make_move_iterator(points.begin()),
                          std::make_move_iterator(points.end()));
    }
    /// 统一世界重力坐标并生成末状态、初始协方差和地图。
    std::optional<InitializationResult> makeResult() {
        StartupCovariance inherited;
        {
            std::vector<std::vector<Observation>> clouds;
            for (std::size_t i = 0; i < frames_.size(); ++i)
                clouds.push_back(mapObservations(i));
            try {
                inherited = recoverStartupCovariance(
                    states_, frames_, clouds, config_.initialization.optimization,
                    config_.initial_accel_bias, config_.initial_accel_bias_std);
            } catch (const NumericalError& error) {
                inherited.reason = error.what();
            }
        }
        if (!inherited.information_supported) {
            setStatus(inherited.failure == StartupCovarianceFailure::WeakInformation
                          ? InitializationState::WeakInformation
                          : InitializationState::Failed,
                      inherited.reason.c_str());
            return std::nullopt;
        }
        if (!inherited.valid && !config_.initial_uncertainty.allow_configured_fallback)
            throw NumericalError(
                "initialization covariance recovery failed with configured prior fallback "
                "disabled: " +
                inherited.reason);
        // 将全部窗口状态转换到重力对齐的世界坐标系。
        alignGravity(states_);
        InitializationResult result;
        std::size_t total_points = 0;
        for (const auto& frame : frames_)
            total_points += frame.raw.size();
        result.map.reserve(total_points);
        result.gravity = Vec3(0, 0, -config_.gravity);
        result.estimate.state = navigationState(states_.back(), frames_.back().end);
        result.estimate.covariance = inherited.valid
                                         ? inherited.frames.back()
                                         : initialCovariance(config_.initial_uncertainty);
        for (std::size_t i = 0; i < frames_.size(); ++i)
            appendMapFrame(result, i, inherited.valid ? &inherited.frames[i] : nullptr);
        setStatus(InitializationState::Ready, inherited.valid
                                                  ? nullptr
                                                  : ("joint information accepted; covariance uses "
                                                     "the configured numerical fallback: " +
                                                     inherited.reason)
                                                        .c_str());
        return result;
    }
    /// 执行窗口细化并构造通过验收的交接结果。
    std::optional<InitializationResult> optimize() {
        setStatus(InitializationState::NotConverged);
        if (refine())
            return makeResult();
        return std::nullopt;
    }
    /// 推进粗状态、选择原始代表点、去畸变和配准后追加一帧。
    void append(const Scan& scan, Time end, const std::vector<Imu>& imus) {
        propagate(coarse_, imus, config_.initialization.registration);
        auto raw = sampleOriginal(scan, config_.initialization.sample_size);
        if (raw.size() < config_.initialization.minimum_sampled_points)
            raw = sampleOriginal(scan, config_.initialization.fallback_sample_size);
        auto points = deskewPoints(raw, imus, coarse_, coarse_, config_.body_from_lidar);
        registration_.update(coarse_, std::move(points), config_.body_from_lidar);
        frames_.push_back({std::move(raw), end, imus});
        states_.push_back(coarse_);
    }
};
/// 完成种子收集、窗口追加及成功交接或失败重试。
std::optional<InitializationResult> Initializer::Impl::add(const Scan& scan, Time end,
                                                           const std::vector<Imu>& imus) {
    if (scan.empty() || imus.size() < 2 || imus.back().time != end)
        return std::nullopt;
    if (mean_count_ < config_.initialization.seed_imu_samples) {
        seed(imus);
        return std::nullopt;
    }
    try {
        append(scan, end, imus);
        if (frames_.size() < config_.initialization.window_frames)
            return std::nullopt;
        auto result = optimize();
        if (!result)
            restart(imus);
        else
            clearWindow();
        return result;
    } catch (const NumericalError& error) {
        setStatus(InitializationState::Failed, error.what());
        restart(imus);
        return std::nullopt;
    }
}

/// 规范化配置后创建初始化会话实现。
Initializer::Initializer(const Config& config) {
    impl_ = std::make_unique<Impl>(normalizeConfig(config));
}
/// 释放尚未交接的启动会话及其全部资源。
Initializer::~Initializer() = default;
/// 读取最近一次启动决策状态。
const InitializationStatus& Initializer::status() const {
    return impl_->status();
}
/// 向初始化会话转交完整同步扫描与 IMU 区间。
std::optional<InitializationResult> Initializer::add(const Scan& scan, Time end,
                                                     const std::vector<Imu>& imus) {
    return impl_->add(scan, end, imus);
}

}  // namespace zlio
