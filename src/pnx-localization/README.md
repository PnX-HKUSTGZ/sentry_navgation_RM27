# PnX Localization

定位相关的 ROS 2 包集中存放在此目录：

- `point_lio`：激光惯性里程计，作为主仓库的普通目录维护。
- `backend`（包名 `rm27_backend`）：Point-LIO 原子帧适配、关键帧与点云管理、里程计因子及有界 iSAM2 位姿图；地图/回环测量接口已预留。
- `localization_interfaces`（包名 `rm27_localization_interfaces`）：前后端共用的原子扫描观测消息。
- `loam_interface`：将 Point-LIO 输出转换为导航使用的里程计与点云。
- `relocalization_manager`：先验点云匹配与 `map → odom` 变换管理。
- `sensor_scan_generation`：将雷达位姿转换为机器人里程计，发布底盘 TF 和雷达坐标系点云。

包名、可执行文件名和 ROS 接口保持原有约定。colcon 会递归发现这些包。后端当前旁路运行，保留原导航 TF 发布链；使用和测试见 [rm27_backend](backend/README.md)。

在工作区根目录构建：

```bash
colcon build --symlink-install --packages-select point_lio loam_interface relocalization_manager sensor_scan_generation
```

## Point-LIO IMU 更新数值保护

`use_imu_as_input: false` 的 IMU 观测更新使用 Cholesky 求解与 Joseph
协方差更新。原始 IMU 非有限值、非法加速度换算参数、有效轴的非正/非有限
测量方差、非法先验状态/协方差、求解溢出或非法后验都会使本次更新被拒绝。
计算在状态副本上完成，仅在全部检查通过后提交；拒绝时保留更新前的状态与
协方差。协方差检查允许零方差方向，但拒绝负主元。

有限的饱和读数仍按轴屏蔽；全部轴饱和时跳过观测更新，不收缩协方差。
每个 IMU 样本仅融合一次，不随激光迭代次数重复融合。

`/point_lio/diagnostics` 新增：

- `rejected_imu_updates`：本帧被拒绝的 IMU 更新次数。
- `last_imu_update_reason`：最近一次 IMU 更新的结果。
- `last_imu_rejection_reason`：本帧最后一次拒绝原因，后续成功更新不会覆盖它。

发生拒绝时诊断为 WARN / `imu_update_rejected`，输出节流日志，并沿用
`map_quality.impact_pause` 暂停入图、丢弃待提交地图点。该保护不负责修复
此前已损坏的状态，也不替代前后端重置协调。

回归测试：

```bash
colcon build --packages-select point_lio --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select point_lio --event-handlers console_direct+
colcon test-result --test-result-base build/point_lio --verbose
```

## 联合启动初始化

Point-LIO 默认 `initialization.method: joint`，使用 ZLIO 的多帧点云共面约束与
IMU 预积分联合估计启动位姿、速度、零偏和重力方向。静止和运动采用同一流程：
31 个 IMU 样本初猜、可配置帧数窗口、重去畸变与粗细优化、几何和联合信息验收，
随后交接末帧状态、协方差和初始局部地图。核心默认/实车配置保留 20 帧，
10 Hz 点云需要约 2.2～2.3 秒数据；仿真配置经参数对照改为 8 帧，约 1.0～1.1 秒。几何退化、饱和或同步异常时等待/重试，不强制输出初始化结果。

`initialization.method: legacy` 保留原来的约 100 样本均值初始化以便回退和 A/B。
参数更改后需重启。联合初始化要求启用 IMU、固定 LiDAR–IMU 外参，且
`prior_pcd.enable: false`；全局地图定位仍交给后端。加速度统一为 SI：
世界系重力通常 `[0,0,-9.81]`；原始 g 单位使用 `acc_norm: 1.0`，原始 m/s²
使用 `acc_norm: 9.81`。实车配置已按此统一，但尚未进行实车验证。

初始化进度在 `/point_lio/diagnostics` 的 `point_lio/initialization` 中输出，
包含状态、原因、样本包数、传感器经过时间与累计计算耗时。
启动核心的来源、提交和许可位于 `point_lio/third_party/zlio_startup`。

实现与完整测试对比见 [联合初始化验收报告](../../docs/pointlio_joint_initialization_2026-11-03.md)。

参数含义与 8 帧调优记录见 [初始化参数调优](../../docs/pointlio_initialization_tuning_2026-11-03.md)。

8 帧、51 个 IMU 初猜样本、4 线程的完整新旧对比见
[最终参数验证](../../docs/pointlio_final_initialization_2026-11-03.md)，包含静止、运动、
非零外参及退化场景，并记录了非零固定外参在运行期的坐标转换问题。
