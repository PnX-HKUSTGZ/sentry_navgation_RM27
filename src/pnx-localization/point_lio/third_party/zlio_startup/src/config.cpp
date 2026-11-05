/**
 * @file config.cpp
 * @brief 读取并校验激光惯性里程计的算法参数。
 */
#include "zlio/config.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <exception>
#include <initializer_list>
#include <limits>
#include <stdexcept>

namespace zlio {
namespace {
/// 将配置位置转换为从一开始计数的行列说明。
std::string configLocation(const YAML::Mark& mark) {
    if (mark.is_null())
        return {};
    return " (line " + std::to_string(mark.line + 1) + ", column " +
           std::to_string(mark.column + 1) + ")";
}

/// 转换配置值，保留字段名称、位置及底层异常。
template <class T>
T configValue(const YAML::Node& node, const char* name) {
    try {
        return node.as<T>();
    } catch (const YAML::BadConversion& error) {
        std::throw_with_nested(
            std::invalid_argument(std::string("invalid configuration type or value: ") + name +
                                  configLocation(error.mark)));
    }
}

/// 校验映射节点中的配置键。
void checkKeys(const YAML::Node& node, std::initializer_list<const char*> allowed) {
    if (!node)
        return;
    if (!node.IsMap())
        throw std::invalid_argument("configuration node must be a mapping");
    for (const auto& entry : node) {
        const auto key = entry.first.as<std::string>();
        if (std::none_of(allowed.begin(), allowed.end(),
                         [&](const char* item) { return key == item; }))
            throw std::invalid_argument("unknown configuration key: " + key);
    }
}

/// 读取存在的标量配置并保留缺省值。
template <typename T>
void read(const YAML::Node& node, const char* key, T& value) {
    if (node && node[key])
        value = configValue<T>(node[key], key);
}

/// 读取固定长度的数值配置数组。
template <std::size_t N>
void read(const YAML::Node& node, const char* key, std::array<double, N>& value) {
    if (!node || !node[key])
        return;
    const auto values = configValue<std::vector<double>>(node[key], key);
    if (values.size() != N)
        throw std::invalid_argument(std::string(key) + " has an invalid array length");
    std::copy(values.begin(), values.end(), value.begin());
}

/// 读取惯性采样方差及两类偏置随机游走强度。
void readImuNoise(const YAML::Node& node, ImuNoise& noise) {
    read(node, "gyro_variance", noise.gyro_sample_variance);
    read(node, "accel_variance", noise.accel_sample_variance);
    read(node, "gyro_bias_psd", noise.gyro_bias_psd);
    read(node, "accel_bias_psd", noise.accel_bias_psd);
}

/// 校验数值为有限正数。
void positive(double value, const char* name) {
    if (!std::isfinite(value) || value <= 0)
        throw std::invalid_argument(std::string(name) + " must be finite and positive");
}

/// 按配置路径校验惯性测量和偏置噪声。
void validateImuNoise(const ImuNoise& noise, const std::string& path) {
    positive(noise.gyro_sample_variance, (path + ".gyro_variance").c_str());
    positive(noise.accel_sample_variance, (path + ".accel_variance").c_str());
    positive(noise.gyro_bias_psd, (path + ".gyro_bias_psd").c_str());
    positive(noise.accel_bias_psd, (path + ".accel_bias_psd").c_str());
}

/// 校验必填的配置节点。
void require(const YAML::Node& node, const char* key, const char* path) {
    if (!node || !node.IsMap() || !node[key] || node[key].IsNull())
        throw std::invalid_argument(std::string("missing required configuration: ") + path);
}

/// 读取重力、测距范围和传感器外参。
void readSensor(const YAML::Node& node, Config& config) {
    require(node, "gravity", "sensor.gravity");
    require(node, "body_from_lidar", "sensor.body_from_lidar");
    checkKeys(node, {"gravity", "minimum_range", "maximum_range", "body_from_lidar"});
    read(node, "gravity", config.gravity);
    read(node, "minimum_range", config.minimum_range);
    read(node, "maximum_range", config.maximum_range);
    const auto extrinsic = node["body_from_lidar"];
    require(extrinsic, "translation", "sensor.body_from_lidar.translation");
    require(extrinsic, "rotation_xyzw", "sensor.body_from_lidar.rotation_xyzw");
    checkKeys(extrinsic, {"translation", "rotation_xyzw"});
    std::array<double, 3> position{0, 0, 0};
    std::array<double, 4> rotation{0, 0, 0, 1};
    read(extrinsic, "translation", position);
    read(extrinsic, "rotation_xyzw", rotation);
    config.body_from_lidar.position = Eigen::Map<Vec3>(position.data());
    config.body_from_lidar.rotation =
        Eigen::Quaterniond(rotation[3], rotation[0], rotation[1], rotation[2]);
}

/// 读取五组导航误差方差，每组应用于对应的三个轴。
void readUncertainty(const YAML::Node& node, InitialUncertainty& prior) {
    if (!node)
        return;
    checkKeys(node, {"state_variance", "allow_configured_fallback"});
    read(node, "allow_configured_fallback", prior.allow_configured_fallback);
    if (node["state_variance"]) {
        std::array<double, 5> values;
        read(node, "state_variance", values);
        for (int i = 0; i < 5; ++i)
            prior.variance[i].setConstant(values[i]);
    }
}

/// 读取启动粗配准参数。
void readRegistration(const YAML::Node& node, RegistrationConfig& config) {
    checkKeys(node, {"voxel_size", "neighbors", "minimum_map_points", "maximum_iterations",
                     "plane_residual_tolerance", "prior_covariance_scale", "rotation_tolerance",
                     "position_tolerance", "gyro_variance", "accel_variance", "bias_variance",
                     "initial_state_variance", "initial_bias_variance"});
    read(node, "voxel_size", config.voxel_size);
    read(node, "neighbors", config.neighbors);
    read(node, "minimum_map_points", config.minimum_map_points);
    read(node, "maximum_iterations", config.maximum_iterations);
    read(node, "plane_residual_tolerance", config.plane_residual_tolerance);
    read(node, "prior_covariance_scale", config.prior_covariance_scale);
    read(node, "rotation_tolerance", config.rotation_tolerance);
    read(node, "position_tolerance", config.position_tolerance);
    read(node, "gyro_variance", config.gyro_variance);
    read(node, "accel_variance", config.accel_variance);
    read(node, "bias_variance", config.bias_variance);
    read(node, "initial_state_variance", config.initial_state_variance);
    read(node, "initial_bias_variance", config.initial_bias_variance);
}

/// 读取启动窗口优化参数。
void readStartupOptimization(const YAML::Node& node, StartupOptimizationConfig& config) {
    checkKeys(node, {"voxel_size", "maximum_depth", "minimum_points", "minimum_planes",
                     "plane_variance", "plane_eigenvalue_ratio", "plane_middle_ratio", "imu_weight",
                     "thread_count", "maximum_iterations", "gyro_variance", "accel_variance",
                     "gyro_bias_psd", "accel_bias_psd"});
    read(node, "voxel_size", config.voxel_size);
    read(node, "maximum_depth", config.maximum_depth);
    read(node, "minimum_points", config.minimum_points);
    read(node, "minimum_planes", config.minimum_planes);
    read(node, "plane_variance", config.plane_variance);
    read(node, "plane_eigenvalue_ratio", config.plane_eigenvalue_ratio);
    read(node, "plane_middle_ratio", config.plane_middle_ratio);
    read(node, "imu_weight", config.imu_weight);
    read(node, "thread_count", config.thread_count);
    read(node, "maximum_iterations", config.maximum_iterations);
    readImuNoise(node, config.noise);
}

/// 读取启动窗口、偏置和求解设置。
void readInitialization(const YAML::Node& node, Config& config) {
    checkKeys(node, {"accel_bias", "accel_bias_std", "uncertainty", "registration", "optimization",
                     "window_frames", "seed_imu_samples", "maximum_refinements", "sample_size",
                     "fallback_sample_size", "minimum_sampled_points",
                     "minimum_directional_support", "relative_cost_tolerance"});
    read(node, "window_frames", config.initialization.window_frames);
    read(node, "seed_imu_samples", config.initialization.seed_imu_samples);
    read(node, "maximum_refinements", config.initialization.maximum_refinements);
    read(node, "sample_size", config.initialization.sample_size);
    read(node, "fallback_sample_size", config.initialization.fallback_sample_size);
    read(node, "minimum_sampled_points", config.initialization.minimum_sampled_points);
    read(node, "minimum_directional_support", config.initialization.minimum_directional_support);
    read(node, "relative_cost_tolerance", config.initialization.relative_cost_tolerance);
    if (node) {
        readRegistration(node["registration"], config.initialization.registration);
        readStartupOptimization(node["optimization"], config.initialization.optimization);
    }
    std::array<double, 3> bias{0, 0, 0};
    read(node, "accel_bias", bias);
    config.initial_accel_bias = Eigen::Map<Vec3>(bias.data());
    std::array<double, 3> deviations;
    std::copy_n(config.initial_accel_bias_std.data(), 3, deviations.begin());
    read(node, "accel_bias_std", deviations);
    config.initial_accel_bias_std = Eigen::Map<Vec3>(deviations.data());
    if (node)
        readUncertainty(node["uncertainty"], config.initial_uncertainty);
}

/// 读取运行滤波参数。
void readFilter(const YAML::Node& node, Config& config) {
    checkKeys(node, {"maximum_iterations", "association_threads", "convergence_tolerance",
                     "gyro_variance", "accel_variance", "gyro_bias_psd", "accel_bias_psd"});
    read(node, "maximum_iterations", config.maximum_iterations);
    read(node, "association_threads", config.association_threads);
    read(node, "convergence_tolerance", config.convergence_tolerance);
    readImuNoise(node, config.noise);
}

/// 读取概率地图参数。
void readMap(const YAML::Node& node, MapConfig& map) {
    checkKeys(node, {"voxel_size", "range_sigma", "angle_sigma_degrees", "maximum_roots"});
    read(node, "voxel_size", map.voxel_size);
    read(node, "range_sigma", map.range_sigma);
    read(node, "angle_sigma_degrees", map.angle_sigma_degrees);
    read(node, "maximum_roots", map.maximum_roots);
}

/// 读取自适应采样参数。
void readSampling(const YAML::Node& node, SamplingConfig& sampling) {
    checkKeys(node, {"initial_size", "minimum_size", "maximum_size", "history_size",
                     "minimum_points", "maximum_points", "scale_threshold", "exponent",
                     "proportional_scale", "derivative_scale", "proportional_minimum",
                     "proportional_maximum", "derivative_minimum", "derivative_maximum"});
    read(node, "initial_size", sampling.initial_size);
    read(node, "minimum_size", sampling.minimum_size);
    read(node, "maximum_size", sampling.maximum_size);
    read(node, "history_size", sampling.history_size);
    read(node, "minimum_points", sampling.minimum_points);
    read(node, "maximum_points", sampling.maximum_points);
    read(node, "scale_threshold", sampling.scale_threshold);
    read(node, "exponent", sampling.exponent);
    read(node, "proportional_scale", sampling.proportional_scale);
    read(node, "derivative_scale", sampling.derivative_scale);
    read(node, "proportional_minimum", sampling.proportional_minimum);
    read(node, "proportional_maximum", sampling.proportional_maximum);
    read(node, "derivative_minimum", sampling.derivative_minimum);
    read(node, "derivative_maximum", sampling.derivative_maximum);
}

/// 读取连续时间调度的间隔边界和归一化局部误差预算。
void readWindow(const YAML::Node& node, WindowConfig& window) {
    checkKeys(node, {"maximum_interval", "minimum_interval", "normalized_error"});
    read(node, "maximum_interval", window.maximum_interval);
    read(node, "minimum_interval", window.minimum_interval);
    read(node, "normalized_error", window.normalized_error);
}

/// 校验采样尺度、目标数量和反馈增益。
void validateSampling(const SamplingConfig& s) {
    positive(s.minimum_size, "sampling.minimum_size");
    positive(s.maximum_size, "sampling.maximum_size");
    positive(s.initial_size, "sampling.initial_size");
    positive(s.minimum_points, "sampling.minimum_points");
    positive(s.maximum_points, "sampling.maximum_points");
    positive(s.scale_threshold, "sampling.scale_threshold");
    positive(s.proportional_scale, "sampling.proportional_scale");
    positive(s.derivative_scale, "sampling.derivative_scale");
    positive(s.proportional_minimum, "sampling.proportional_minimum");
    positive(s.proportional_maximum, "sampling.proportional_maximum");
    positive(s.derivative_minimum, "sampling.derivative_minimum");
    positive(s.derivative_maximum, "sampling.derivative_maximum");
    if (s.history_size == 0 || s.minimum_size > s.initial_size || s.initial_size > s.maximum_size ||
        s.minimum_points > s.maximum_points || !std::isfinite(s.exponent) || s.exponent <= 1 ||
        s.proportional_minimum > s.proportional_maximum ||
        s.derivative_minimum > s.derivative_maximum)
        throw std::invalid_argument("invalid sampling parameter range");
}

/// 校验启动窗口和粗配准设置。
void validateInitialization(const InitializationConfig& config) {
    if (config.window_frames < 2 || config.window_frames > std::numeric_limits<int>::max() / 15 ||
        config.seed_imu_samples == 0 || config.maximum_refinements < 4 ||
        config.minimum_sampled_points == 0)
        throw std::invalid_argument("invalid initialization window or iteration budget");
    positive(config.sample_size, "initialization.sample_size");
    positive(config.fallback_sample_size, "initialization.fallback_sample_size");
    positive(config.minimum_directional_support, "initialization.minimum_directional_support");
    if (config.fallback_sample_size > config.sample_size)
        throw std::invalid_argument(
            "initialization fine sampling size must not exceed the regular size");
    for (double tolerance : config.relative_cost_tolerance) {
        positive(tolerance, "initialization.relative_cost_tolerance");
        if (tolerance >= 1)
            throw std::invalid_argument(
                "initialization relative cost tolerance must be less than one");
    }
    const auto& registration = config.registration;
    positive(registration.voxel_size, "initialization.registration.voxel_size");
    positive(registration.plane_residual_tolerance,
             "initialization.registration.plane_residual_tolerance");
    positive(registration.prior_covariance_scale,
             "initialization.registration.prior_covariance_scale");
    positive(registration.rotation_tolerance, "initialization.registration.rotation_tolerance");
    positive(registration.position_tolerance, "initialization.registration.position_tolerance");
    positive(registration.gyro_variance, "initialization.registration.gyro_variance");
    positive(registration.accel_variance, "initialization.registration.accel_variance");
    positive(registration.bias_variance, "initialization.registration.bias_variance");
    positive(registration.initial_state_variance,
             "initialization.registration.initial_state_variance");
    positive(registration.initial_bias_variance,
             "initialization.registration.initial_bias_variance");
    if (registration.neighbors < 3 || registration.maximum_iterations < 1 ||
        registration.minimum_map_points < static_cast<std::size_t>(registration.neighbors))
        throw std::invalid_argument(
            "invalid coarse initialization registration point count or iteration budget");
}

/// 校验启动几何与优化资源设置。
void validateStartupOptimization(const StartupOptimizationConfig& config) {
    positive(config.voxel_size, "initialization.optimization.voxel_size");
    positive(config.imu_weight, "initialization.optimization.imu_weight");
    positive(config.plane_variance[0], "initialization.optimization.plane_variance[0]");
    positive(config.plane_variance[1], "initialization.optimization.plane_variance[1]");
    positive(config.plane_eigenvalue_ratio, "initialization.optimization.plane_eigenvalue_ratio");
    positive(config.plane_middle_ratio, "initialization.optimization.plane_middle_ratio");
    validateImuNoise(config.noise, "initialization.optimization");
    if (config.maximum_depth < 0 || config.maximum_depth > 8 || config.thread_count < 1 ||
        config.maximum_iterations < 1 || config.minimum_points < 3 || config.minimum_planes < 1 ||
        config.plane_variance[1] > config.plane_variance[0] || config.plane_eigenvalue_ratio >= 1 ||
        config.plane_middle_ratio >= 1)
        throw std::invalid_argument("invalid initialization plane or solver resource parameters");
}

/// 校验观测间隔的数值下限及上下界关系。
void validateWindow(const WindowConfig& window) {
    positive(window.maximum_interval, "window.maximum_interval");
    positive(window.minimum_interval, "window.minimum_interval");
    positive(window.normalized_error, "window.normalized_error");
    if (window.minimum_interval < 1e-9 || 2 * window.minimum_interval > window.maximum_interval)
        throw std::invalid_argument(
            "intervals must satisfy 1 ns <= minimum_interval and 2*minimum_interval <= "
            "maximum_interval");
}
/// 校验传感器几何、重力和启动先验。
void validateInitialConditions(const Config& config) {
    if (!std::isfinite(config.gravity) || config.gravity < 9.6 || config.gravity > 10.0)
        throw std::invalid_argument("sensor.gravity must be within [9.6, 10.0] m/s^2");
    if (!config.initial_accel_bias.allFinite())
        throw std::invalid_argument("invalid initial acceleration bias");
    for (int axis = 0; axis < 3; ++axis) {
        const double deviation = config.initial_accel_bias_std[axis];
        positive(deviation, "initialization.accel_bias_std");
        positive(1 / (deviation * deviation), "acceleration bias prior information");
    }
    positive(config.maximum_range, "sensor.maximum_range");
    if (!std::isfinite(config.minimum_range) || config.minimum_range < 0 ||
        config.minimum_range >= config.maximum_range)
        throw std::invalid_argument("invalid range limits");
    if (!config.body_from_lidar.position.allFinite() ||
        !config.body_from_lidar.rotation.coeffs().allFinite() ||
        std::abs(config.body_from_lidar.rotation.norm() - 1) > .01)
        throw std::invalid_argument("invalid rigid extrinsics");
    for (const Vec3& variance : config.initial_uncertainty.variance)
        for (int axis = 0; axis < 3; ++axis)
            positive(variance[axis], "variance component in initialization.uncertainty");
}

/// 校验惯性噪声、线程数及迭代预算。
void validateFilter(const Config& config) {
    validateImuNoise(config.noise, "filter");
    if (config.maximum_iterations < 1)
        throw std::invalid_argument("filter iteration count must be positive");
    if (config.association_threads < 1 || config.association_threads > 2)
        throw std::invalid_argument("point-to-plane association thread count must be one or two");
    positive(config.convergence_tolerance, "filter.convergence_tolerance");
}

/// 校验地图平面判据、测量噪声和资源上限。
void validateMap(const MapConfig& map) {
    positive(map.voxel_size, "map.voxel_size");
    positive(map.range_sigma, "map.range_sigma");
    positive(map.angle_sigma_degrees, "map.angle_sigma_degrees");
    const float resolution = static_cast<float>(map.voxel_size);
    if (!std::isnormal(resolution) || !std::isnormal(1.f / resolution))
        throw std::invalid_argument("map scale must be representable as a valid float");
    if (map.maximum_roots == 0)
        throw std::invalid_argument("map root voxel capacity must be positive");
}
}  // namespace

/// 加载并规范化 YAML 算法配置。
Config loadConfig(const std::string& path) {
    try {
        const auto root = YAML::LoadFile(path);
        checkKeys(root, {"sensor", "initialization", "filter", "map", "sampling", "window"});
        require(root, "sensor", "sensor");
        Config config;
        readSensor(root["sensor"], config);
        readInitialization(root["initialization"], config);
        readFilter(root["filter"], config);
        readMap(root["map"], config.map);
        readSampling(root["sampling"], config.sampling);
        readWindow(root["window"], config.window);
        return normalizeConfig(std::move(config));
    } catch (const YAML::BadFile&) {
        std::throw_with_nested(std::runtime_error("could not read configuration file: " + path));
    } catch (const YAML::ParserException& error) {
        std::throw_with_nested(std::invalid_argument("invalid configuration syntax: " + path +
                                                     configLocation(error.mark)));
    } catch (const YAML::Exception& error) {
        std::throw_with_nested(std::invalid_argument("invalid configuration structure: " + path +
                                                     configLocation(error.mark)));
    }
}

/// 校验配置并规范化外参旋转。
Config normalizeConfig(Config config) {
    validateConfig(config);
    // 将允许范围内的外参四元数归一化。
    if (std::abs(config.body_from_lidar.rotation.squaredNorm() - 1) >
        4 * std::numeric_limits<double>::epsilon())
        config.body_from_lidar.rotation.normalize();
    return config;
}

/// 校验算法参数、坐标变换与资源设置。
void validateConfig(const Config& config) {
    validateInitialConditions(config);
    validateFilter(config);
    validateMap(config.map);
    validateSampling(config.sampling);
    validateWindow(config.window);
    validateInitialization(config.initialization);
    validateStartupOptimization(config.initialization.optimization);
}
}  // namespace zlio
