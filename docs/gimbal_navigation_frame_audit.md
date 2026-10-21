# 实车旋转云台与定位坐标检查

## 本次已修复的软件问题

- `fake_vel_transform` 的 MINCO 最新里程计模式不再创建旧版路径同步器。原实现即使设置
  `use_latest_odom_for_cmd: true`，延迟路径仍能触发同步回调并用旧 yaw 覆盖新 yaw。
- 最新模式拒绝重复、乱序的有效里程计，保留过期停车保护；legacy 路径同步模式不变。
- MINCO 规划器先用完整三维姿态还原 child-frame twist，再按里程计时间戳查询
  `planning_frame <- odometry.header.frame_id`，最后投影到规划平面。
  不再把 `camera_init` 的速度直接与 `map` 中的轨迹比较。
- 缺少对应时间的 TF 或速度无效时，不再假定车辆静止并生成冷启动轨迹，而是发布 BLOCK。

这些修改有回归测试，但不代表已经在实车上证实消除了位置漂移。
当前 MINCO 本身不发布旧版 `/local_plan`，旧同步器问题是否在现场触发，还取决于是否有
其他节点发布该路径。上述两处修复也不直接修改 Point-LIO 的原始位姿估计。

## 仓库中实际存在的接口

`config/reality/minco_params.yaml` 中 `fake_vel_transform` 配置的输入是 `/lidar_odometry`，由 `loam_interface`
发布，父坐标系为 `odom`，child 为 `left_mid360`。`fake_vel_transform` 只发布
`gimbal_yaw -> gimbal_yaw_fake` 的零平移、负 yaw 变换。
MINCO 规划器另用 `/aft_mapped_to_init`，其父坐标系为 `camera_init`，并非 `map`。

默认实车描述中 `base_link -> gimbal_yaw`、`base_link -> left_mid360`、
`base_link -> imu_link` 都是固定关节。这不符合雷达随云台相对底盘独立旋转的实际结构。
`sensor_scan_generation` 虽会按点云时间查询机械 TF，但不能自行创造缺失的关节运动。

沿导航文档追查到的通信实现位于：

```text
/home/pnx/pnx_autoaim_sp/auto-aim-new/io/ros2/subscribe2nav.cpp
/home/pnx/pnx_autoaim_sp/auto-aim-new/io/gimbal/gimbal.hpp
/home/pnx/pnx_autoaim_sp/auto-aim-new/io/gimbal/gimbal.cpp
```

- 通信节点订阅 `/cmd_vel`，没有发布 `/joint_states` 或真实云台关节 TF。
- 上行 `GimbalToVision` 有小 yaw 四元数和 `yaw/pitch`，但现有接口没有明确这两个角是否为
  相对底盘的机械角；四元数和绝对姿态不能直接替代关节编码器。
- 现场通信源码实际对 `linear_x/y` 取反，与旁边注释及旧导航文档不同。本次未修改这些符号、
  串口协议或现场安装外参，避免覆盖已有调试。
- 导航仓库中的手柄 `JointState` 是云台控制命令，不是实测编码器反馈。

因此不能在没有相对角来源、单位、正方向和转轴位置的情况下，把 URDF 强行改成动态关节，
也不能声称已经完成真实云台到底盘的位姿与速度补偿。

## 实车对照记录

先禁用底盘驱动，保留雷达、云台和 ROS 导航；分别记录未发目标、发目标后同样转动云台的短测试。
不要只凭 RViz 中转动的机器人模型判断 Point-LIO 原始定位漂移。

```bash
ros2 bag record -o /tmp/gimbal_frame_check \
  /tf /tf_static /aft_mapped_to_init /lidar_odometry /odometry \
  /cmd_vel /joint_states /livox/imu /cloud_registered /rosout
```

| 对照结果 | 检查方向 |
| --- | --- |
| `camera_init` 下静态点云稳定，底盘模型却移动 | 动态机械 TF、旋转中心和雷达安装偏置 |
| `camera_init` 下静态墙面也漂移或重影 | Point-LIO、IMU/点云时序、导航负载导致的处理延迟 |
| 局部坐标下稳定，`map` 下跳变 | `map -> odom` 校正或重复 TF 发布 |
| 位置稳定但发目标后速度方向变化异常 | 里程计 yaw、机械相对角和下位机速度坐标约定 |

现有重定位代码依据 `/cmd_vel` 判断运动并暂停校正，所以“已下发速度”和“物理车在移动”
不能混为一谈。10 月 14 日旧日志还存在 GICP 高度门限拒绝，不能假定静止时全局校正一直有效。

真实机械关系补齐后，应让规划、MPC 和 footprint 使用同一底盘中心状态；仅修这个虚拟 TF
节点不能替代该工作。MID360 内部雷达与 IMU 一起转动，其内部外参仍应保持固定。
