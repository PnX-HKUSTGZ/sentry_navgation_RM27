# PnX Localization

定位相关的 ROS 2 包集中存放在此目录：

- `point_lio`：激光惯性里程计，作为主仓库的普通目录维护。
- `loam_interface`：将 Point-LIO 输出转换为导航使用的里程计与点云。
- `relocalization_manager`：先验点云匹配与 `map → odom` 变换管理。
- `sensor_scan_generation`：将雷达位姿转换为机器人里程计，发布底盘 TF 和雷达坐标系点云。

包名、可执行文件名和 ROS 接口保持原有约定。colcon 会递归发现这四个包。

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
