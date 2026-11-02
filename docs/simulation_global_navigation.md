# 全局重定位接入 replace_gicp

本入口在最新 `replace_gicp` 的导航链上增加全局初始化。定位算法、比赛参数、参考 PCD 和 manifest 来自 `feat/global-relocalization`；导航控制、地图处理、Point-LIO、云台、碰撞模型与导航参数沿用目标分支。

## 构建与启动

先按仓库说明构建当前工作区及其导航依赖。已有导航构建时，可更新接入包：

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select \
  global_relocalization_interfaces global_relocalization \
  relocalization_manager launch_bringup rm_27_stimulation rm27_rendering
bash tools/global_relocalization/build.bash
source tools/global_relocalization/environment.bash
bash tools/global_relocalization/navigate.bash
```

在 RViz 使用 **2D Pose Estimate** 选择左半场开放位置与朝向。这次选点只用于 Gazebo 出生，不会直接作为定位结果。定位成功并完成确认后，使用 **Nav2 Goal** 导航。

启动过程依次执行：

1. 发布地图，等待出生点、LiDAR 和 IMU。
2. 调用全局定位 action，自转准备，进入 ARMED 后触发比赛开始及三个新扫描的停车确认。
3. 将确认位姿和原始时间戳交给 `relocalization_manager`。管理器使用相同时间戳的 `odom → base_footprint` 计算 `map → odom`，回送确认。
4. 恢复选定朝向，释放初始化速度控制，再激活导航。

`global_relocalization` 默认关闭。关闭时保持目标分支原有连续 GICP 修正行为。开启时使用全局分支的单次初始化、幂等确认和停车后局部 GICP 共识流程；`stationary_relocalization:=false` 可关闭全局初始化后的停车修正。

```bash
bash tools/global_relocalization/navigate.bash \
  stationary_relocalization:=false
```

初始化期间，协调器转发 `/relocalization/cmd_vel`；超时、失败时停车。成功交接后销毁速度发布者与订阅，后续速度控制交还目标分支原有导航链。GT 对照模式也仅通过确认位姿初始化 `map → odom`，不会把 RViz 出生点直接交给导航。

## 独立定位演示

```bash
bash tools/global_relocalization/visualize.bash
```

该入口不启动导航。在 `.build/robot_velocity.urdf` 生成独立演示模型，使用原全局分支的模型速度控制与固定传感器安装假设。由于目标分支新增了活动云台，临时模型会固定两个云台关节，并把 IMU 插件连接到固定关节合并后的底盘 link。正式 URDF、物理驱动与云台配置不会被修改。

raw 比赛参数仍按原分支要求使用理想瞬时扫描；这不代表已验证实车 MID360 去畸变或活动云台下的全局定位。算法、接口和历史实验边界见 [全局重定位说明](global_relocalization.md)，本次移植证据见 [合并验证记录](global_relocalization_merge_20261102.md)。
