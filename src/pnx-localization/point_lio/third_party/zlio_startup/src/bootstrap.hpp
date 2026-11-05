/**
 * @file bootstrap.hpp
 * @brief 声明初始化联合约束及其协方差交接接口。
 */
#pragma once

#include <memory>
#include <string>

#include "zlio/window.hpp"

namespace zlio {

/// 将窗口旋转增量映射为右乘旋转矩阵。
Mat3 windowRotation(const Vec3& rotation_vector);

/// 将十五维右扰动注入参考导航量；保留输出对象的重力和协方差。
void applyWindowIncrement(const WindowState& reference, const Vec15& increment,
                          WindowState& result);

/// 持有一次启动尝试的偏置先验与当前外层的帧间预积分。
class StartupInertialModel {
public:
    /// 固定惯性配置及 IMU 系首帧偏置先验。
    StartupInertialModel(const StartupOptimizationConfig& config, const Vec3& bias,
                         const Vec3& deviations);
    /// 释放预积分与先验实现。
    ~StartupInertialModel();
    /// 按当前偏置重新积分所有相邻帧，重建冻结信息矩阵。
    void rebuild(const std::vector<WindowState>& states, const std::vector<WindowFrame>& frames);
    /// 清零并组装完整惯性系统，统一施加惯性权重，返回半平方代价。
    double linearize(const std::vector<WindowState>& states, Eigen::MatrixXd& hessian,
                     Eigen::VectorXd& gradient);
    /// 使用当前偏置修正及冻结信息矩阵求值，包含一次首帧先验。
    double evaluateCost(const std::vector<WindowState>& states);
    /// 累积本次试探的各帧偏置增量，并保存上次修正供拒绝时恢复。
    void applyBiasIncrement(const Eigen::VectorXd& increment);
    /// 恢复所有预积分在最近试探前的偏置修正。
    void rollbackBiasIncrement();

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

/// 持有当前外层的平面分组、充分统计量及求值缓存。
class StartupPlaneModel {
public:
    /// 固定几何构造配置，尚不建立平面约束。
    explicit StartupPlaneModel(const StartupOptimizationConfig& config);
    /// 释放平面支持统计量及缓存。
    ~StartupPlaneModel();
    /// 从当前去畸变点云重建粗阶段或细阶段平面约束。
    void rebuild(const std::vector<std::vector<Vec3>>& clouds,
                 const std::vector<WindowState>& states, bool refined);
    /// 返回提供帧间信息的有效平面数量。
    std::size_t size() const;
    /// 返回约束对应的窗口帧数。
    int frameCount() const;
    /// 检查有效平面是否连接窗口全部帧。
    bool connected() const;
    /// 返回当前有效平面的方向支撑矩阵。
    Mat3 normalSupport() const;
    /// 组装指定半开平面区间的位姿信息、梯度和代价。
    void linearize(const std::vector<WindowState>& states, int begin, int end,
                   Eigen::MatrixXd& hessian, Eigen::VectorXd& gradient, double& cost);
    /// 计算指定半开平面区间的代价并更新几何缓存。
    void evaluateCost(const std::vector<WindowState>& states, int begin, int end, double& cost);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

enum class StartupCovarianceFailure { InvalidInput, WeakInformation, NumericalFailure, None };
struct StartupCovariance {
    std::vector<Mat15> frames;
    std::string reason;
    // 去除规范后的联合信息是否具有完整数值秩。
    bool information_supported = false;
    StartupCovarianceFailure failure = StartupCovarianceFailure::InvalidInput;
    bool valid = false;
};
/// 恢复固定关联下各帧重力对齐后的导航协方差。
StartupCovariance recoverStartupCovariance(const std::vector<WindowState>& states,
                                           const std::vector<WindowFrame>& frames,
                                           const std::vector<std::vector<Observation>>& clouds,
                                           const StartupOptimizationConfig& config,
                                           const Vec3& bias, const Vec3& bias_std);
}  // namespace zlio
