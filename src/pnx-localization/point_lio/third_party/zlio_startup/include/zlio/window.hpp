/**
 * @file window.hpp
 * @brief 声明启动窗口状态、优化结果及有限预算的窗口求解接口。
 */
#pragma once

#include <memory>
#include <vector>

#include "zlio/types.hpp"

namespace zlio {

struct StartupOptimizationConfig;

/// 联合窗口状态，误差顺序为 IMU 右旋转、位置、速度及两偏置。
struct WindowState {
    Mat3 rotation;
    Vec3 position, velocity, gyro_bias, accel_bias, gravity;
    Mat15 covariance;
    /// 构造单位姿态和零值状态，重力与粗先验由初始化配置赋值。
    WindowState();
    /// 原位注入右旋转误差及位置、速度和偏置增量。
    WindowState& operator+=(const Vec15& increment);
    /// 计算相对参考状态的十五维右扰动差。
    Vec15 operator-(const WindowState& reference) const;
    /// 恢复单位姿态并清零导航量、重力及协方差。
    void setZero();
};

/// 联合窗口中的扫描观测及帧间 IMU 区间。
struct WindowFrame {
    Scan raw;
    Time end = 0;
    std::vector<Imu> imus;
};

enum class OptimizationStatus {
    InsufficientPlanes,
    DisconnectedWindow,
    NoProgress,  // 预算耗尽，无接受步，也不满足驻点条件。
    Progress,    // 已有下降，但内层预算结束时仍未满足停止条件。
    Converged    // 梯度驻点，或接受步达到代价收敛条件。
};

struct OptimizationStep {
    std::size_t planes = 0;
    double initial_cost = 0;
    double final_cost = 0;
    OptimizationStatus status = OptimizationStatus::InsufficientPlanes;
};

/// 对联合窗口状态执行惯性与平面约束优化。
class WindowOptimizer {
public:
    /// 保存优化配置及 IMU 系加速度偏置先验，单位为 m/s²。
    explicit WindowOptimizer(const StartupOptimizationConfig& config,
                             const Vec3& accel_bias = Vec3::Zero(),
                             const Vec3& accel_bias_std = Vec3::Constant(.3));
    /// 结束所有分区工作并释放窗口模型。
    ~WindowOptimizer();
    /// 重建当前几何和预积分，原位更新窗口状态并报告接受进展。
    OptimizationStep refine(std::vector<WindowState>& states,
                            const std::vector<WindowFrame>& frames,
                            const std::vector<std::vector<Vec3>>& local_points, bool refined);
    /// 返回当前有效平面的方向支撑矩阵。
    Mat3 normalSupport() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace zlio
