# Point-LIO 仿真导航修复（2026-10-28）

当前验证分支：`replace_gicp`。仿真：RMUC2026，MINCO，`use_ground_truth_odom:=false`。

## 修复内容

- `simulation_waking_robot.xacro` 中雷达和 IMU 均相对云台安装于 `(0, 0.126, 0.130) m`，roll 为 −30°。
- `loam_interface` 以 `T_odom_lio = T_base_sensor(t0) * inverse(T_lio_sensor(t0))` 对齐初始底盘原点，消除安装倾角重复叠加。挂载变换取自 URDF 发布的 TF，位姿协方差也随世界系变换旋转。
- 仿真使用 `/cloud_registered_full` 完整去畸变点云，接口订阅采用兼容其发布者的 best-effort QoS。原稀疏特征点不足以稳定完成此处的先验配准。
- GICP 新增 `gravity_aligned`，在优化中限制 roll/pitch；仿真开启，默认关闭。保留平移、yaw 求解和已有质量、增量门限。
- RMUC2026 初始 `map -> odom` 采用与出生配置一致的底盘位姿 `[-11.7, 3.1, 0.08, 0, 0, 0]`。
- Point-LIO 模式的 MPC 和速度转换节点统一采用 0.35 s 里程计时效预算，覆盖约 10 Hz 扫描输出、约 0.11 s 传输年龄及短时抖动。仍保留超时停车，未改写测量时间戳。

## 验证

- `loam_interface`、`relocalization_manager` 构建通过。
- 新增的两个坐标对齐测试通过；六项相关启动参数测试通过。
- 运行 `tools/stimulation/start_nav_stimulation.bash`，Gazebo GUI 与 RViz 均开启；本次无桌面终端使用临时虚拟显示。
- GICP 完成连续验证并进入 `LOCALIZED`，发出与故障复现相同的 `(5.50, 4.90)` goal，Nav2 返回 `SUCCEEDED`。
- Gazebo 真值仅用于统计物理运动，没有接入导航定位。

完整记录见 [验证 JSON](validation/pointlio_navigation_2026-10-28.json)。

本次物理净位移 17.027 m，终点真值到目标的距离 0.268 m。
最终运行未再出现地面支撑因世界系倾斜而关闭的告警。GUI 负载下仍触发过短时里程计超时停车，随后自动继续并到达目标；本验证不等于任意场景的精度或连续运行保证。
