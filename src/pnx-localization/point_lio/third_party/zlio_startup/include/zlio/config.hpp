/**
 * @file config.hpp
 * @brief 定义传感器、滤波、地图及初始化的配置类型。
 */
#pragma once

#include <array>
#include <cstddef>
#include <string>

#include "zlio/types.hpp"

namespace zlio {

/// 定义惯性测量方差与偏置随机游走强度。
struct ImuNoise {
    /// 原始角速度采样方差，rad²/s²。
    double gyro_sample_variance = .25;
    /// 原始比力采样方差，m²/s⁴。
    double accel_sample_variance = .5;
    /// 陀螺偏置随机游走强度，rad²/s³。
    double gyro_bias_psd = 1.33e-6;
    /// 加速度偏置随机游走强度，m²/s⁵。
    double accel_bias_psd = 2.15e-5;

    /// 将原始采样方差转换为当前区间使用的噪声强度。
    Eigen::Matrix<double, 12, 1> varianceRates(double original_interval) const;
};

/// 定义惯性历史容量和连续采样间隔上限。
struct ImuHistoryLimits {
    /// 可重放惯性历史的最大时长，s。
    double maximum_duration = 30;
    /// 可保留的惯性样本数上限。
    std::size_t maximum_samples = 65536;
    /// 相邻原始惯性样本的最大间隔，s。
    double maximum_interval = .1;
};

/// 定义概率地图的空间尺度、资源上限及测量噪声。
struct MapConfig {
    /// 地图根体素边长，m。
    double voxel_size = .5;
    /// 径向测距标准差，m。
    double range_sigma = .02;
    /// 切向测角标准差，度。
    double angle_sigma_degrees = .05;
    /// 根体素数量上限。
    std::size_t maximum_roots = 1000000;
};

/// 定义采样尺度和反馈控制参数。
struct SamplingConfig {
    /// 初始采样体素边长，m。
    double initial_size = .25;
    /// 采样体素边长下限，m。
    double minimum_size = .02;
    /// 采样体素边长上限，m。
    double maximum_size = 1;
    /// 场景尺度平滑所用的历史观测数。
    std::size_t history_size = 5;
    /// 目标采样点数下限。
    double minimum_points = 1000;
    /// 目标采样点数上限。
    double maximum_points = 4000;
    /// 场景距离中位数的归一化长度，m。
    double scale_threshold = 30;
    /// 场景尺度到目标点数的映射指数。
    double exponent = 2;
    /// 比例项的相对点数误差尺度。
    double proportional_scale = .1;
    /// 微分项的相对误差变化尺度。
    double derivative_scale = .2;
    /// 比例增益下限，m/点。
    double proportional_minimum = 1e-6;
    /// 比例增益上限，m/点。
    double proportional_maximum = 1e-4;
    /// 微分增益下限，m·s/点。
    double derivative_minimum = 1e-9;
    /// 微分增益上限，m·s/点。
    double derivative_maximum = 1e-7;
};

/// 定义观测间隔边界和局部运动外推误差预算。
struct WindowConfig {
    /// 观测间隔及采样历史跨度上限，s。
    double maximum_interval = .12;
    /// 观测间隔下限，s。
    double minimum_interval = .03;
    /// 点面噪声归一化后的运动外推 RMS 预算。
    double normalized_error = 2.;
};

/// 定义初始化导航误差的配置方差。
struct InitialUncertainty {
    /// 联合信息通过验收后，允许协方差数值恢复失败时使用配置方差。
    bool allow_configured_fallback = true;
    /// 位置、左旋转、速度和两偏置的各轴方差，使用对应 SI 单位。
    std::array<Vec3, 5> variance{Vec3::Ones(), Vec3::Ones(), Vec3::Ones(), Vec3::Constant(.0001),
                                 Vec3::Constant(.001)};
};

/// 定义启动粗配准及临时惯性传播参数。
struct RegistrationConfig {
    /// 配准质心体素边长，m。
    double voxel_size = .5;
    /// 局部平面拟合的最近邻点数。
    int neighbors = 5;
    /// 单次粗配准的迭代上限。
    int maximum_iterations = 4;
    /// 开始粗配准所需的地图点数。
    std::size_t minimum_map_points = 100;
    /// 平面方程 A·n=-1 的拟合残差上限。
    double plane_residual_tolerance = .1;
    /// 粗配准先验协方差的放大系数。
    double prior_covariance_scale = 1000;
    /// 旋转增量收敛限值，rad。
    double rotation_tolerance = .01 / 57.3;
    /// 位置增量收敛限值，m。
    double position_tolerance = .00015;
    /// 粗传播角速度采样方差，rad²/s²。
    double gyro_variance = .1;
    /// 粗传播比力采样方差，m²/s⁴。
    double accel_variance = 1;
    /// 原始 IMU 区间内两偏置变化率的有效方差。
    double bias_variance = 1e-4;
    /// 粗启动旋转、位置与速度的各轴先验方差，使用对应 SI 单位。
    double initial_state_variance = 1e-4;
    /// 粗启动两偏置的各轴先验方差，使用对应 SI 单位。
    double initial_bias_variance = 1e-5;
};

/// 定义联合窗口的平面构造与惯性求解参数。
struct StartupOptimizationConfig {
    /// 平面分组的根体素边长，m。
    double voxel_size = 1;
    /// 平面体素最大细分深度。
    int maximum_depth = 2;
    /// 几何求解线程数。
    int thread_count = 5;
    /// 单次联合优化的下降步数上限。
    int maximum_iterations = 3;
    /// 启用平面拟合的支持点数界限。
    std::size_t minimum_points = 5;
    /// 联合优化所需的有效平面数下限。
    std::size_t minimum_planes = 10;
    /// 粗、细阶段的法向散布方差上限，m²。
    std::array<double, 2> plane_variance{.02, .0025};
    /// 最小与最大散布特征值的比例上限。
    double plane_eigenvalue_ratio = .25;
    /// 最小与中间散布特征值的比例上限。
    double plane_middle_ratio = .12;
    /// 惯性代价相对几何代价的权重。
    double imu_weight = 1e-4;
    /// 帧间预积分的测量噪声与偏置随机游走参数。
    ImuNoise noise{.01, 1, 1e-4, 1e-4};
};

/// 定义启动数据积累、采样及窗口细化参数。
struct InitializationConfig {
    /// 参与联合估计的扫描帧数。
    std::size_t window_frames = 20;
    /// 初猜重力方向所需的惯性样本数。
    std::size_t seed_imu_samples = 31;
    /// 触发细采样的点数界限。
    std::size_t minimum_sampled_points = 1000;
    /// 重去畸变与联合优化的交替轮数上限。
    int maximum_refinements = 15;
    /// 原始回波的常规采样体素边长，m。
    double sample_size = .1;
    /// 支持不足时的细采样体素边长，m。
    double fallback_sample_size = .05;
    /// 平面法向最弱方向的支撑下限。
    double minimum_directional_support = 15;
    /// 粗、细阶段的相对代价变化收敛限值。
    std::array<double, 2> relative_cost_tolerance{.05, .01};
    /// 初始位姿生成所用的粗配准参数。
    RegistrationConfig registration;
    /// 联合窗口的平面与惯性优化参数。
    StartupOptimizationConfig optimization;
};

/// 汇集传感器与各算法阶段的配置。
struct Config {
    /// 将雷达坐标转换到 IMU 坐标的刚性外参。
    Pose body_from_lidar;
    /// 固定重力模长，m/s²。
    double gravity = 9.81;
    /// 有效测距下限，m。
    double minimum_range = .5;
    /// 有效测距上限，m。
    double maximum_range = 100;
    /// IMU 系加速度偏置初值及先验中心，m/s²。
    Vec3 initial_accel_bias = Vec3::Zero();
    /// 加速度偏置先验各轴标准差，m/s²。
    Vec3 initial_accel_bias_std = Vec3::Constant(.3);
    /// 初始化协方差恢复的配置方差。
    InitialUncertainty initial_uncertainty;
    /// 启动数据收集及联合估计参数。
    InitializationConfig initialization;
    /// 运行阶段的惯性噪声参数。
    ImuNoise noise;
    /// 单次滤波观测更新的迭代上限。
    int maximum_iterations = 5;
    /// 点面关联线程数。
    int association_threads = 2;
    /// 导航状态变化的收敛限值，使用对应 SI 单位。
    double convergence_tolerance = .001;
    /// 平面地图的空间、资源及噪声参数。
    MapConfig map;
    /// 回波采样与密度反馈参数。
    SamplingConfig sampling;
    /// 观测间隔调度参数。
    WindowConfig window;
};

/// 从 YAML 读取并规范化算法配置。
Config loadConfig(const std::string& path);
/// 校验参数数值、资源上限和外参。
void validateConfig(const Config& config);
/// 校验配置并归一化外参四元数。
Config normalizeConfig(Config config);

}  // namespace zlio
