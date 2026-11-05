/**
 * @file initialization.hpp
 * @brief 声明完整同步启动包的消费接口、状态说明及初始化交接结果。
 */
#pragma once

#include <memory>
#include <optional>
#include <string>

#include "zlio/config.hpp"
#include "zlio/plane.hpp"

namespace zlio {

/// 按配置的各轴方差构造十五维导航误差的对角先验。
Mat15 initialCovariance(const InitialUncertainty& uncertainty);

struct InitializationResult {
    Estimate estimate;
    Vec3 gravity = Vec3::Zero();
    std::vector<MapPoint> map;
};

/// 区分启动收集、优化进展、几何拒绝与最终交接状态。
enum class InitializationState {
    Collecting,          // 正在收集同步启动数据。
    NotConverged,        // 当前窗口尚未通过粗细阶段收敛检查。
    InsufficientPlanes,  // 有效跨帧平面数量不足。
    DisconnectedWindow,  // 跨帧几何约束没有连接整个窗口。
    NoProgress,          // 优化未取得有效进展，也未到达驻点。
    NonfiniteState,      // 窗口状态中出现非有限值。
    WeakGeometry,        // 最弱方向的几何支撑不足。
    Ready,               // 已构造可交接的状态与初始地图。
    Failed,              // 当前尝试因异常中止，具体原因见诊断说明。
    WeakInformation      // 去除规范后的联合信息不足。
};

/// 保存最近一次启动决策的机器状态与诊断说明。
struct InitializationStatus {
    std::string reason = "collecting initialization data";
    InitializationState state = InitializationState::Collecting;
};

/// 管理同步启动数据、窗口优化及状态交接。
class Initializer {
public:
    /// 规范化配置并建立独立拥有的初始化会话。
    explicit Initializer(const Config& config);
    /// 释放未完成的启动窗口及粗地图。
    ~Initializer();
    /// 消费完整扫描末端之间的同步包，只有通过接受检查才交接结果。
    std::optional<InitializationResult> add(const Scan& scan, Time end,
                                            const std::vector<Imu>& imus);
    /// 读取当前收集、拒绝或完成状态。
    const InitializationStatus& status() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace zlio
