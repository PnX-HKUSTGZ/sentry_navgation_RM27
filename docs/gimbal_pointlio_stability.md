# 云台持续旋转与 Point-LIO

## 仿真中已确认的问题

修复前，云台实际以约 1.52 rad/s 转动，但 `/livox/imu.angular_velocity` 三轴
一直为零。加速度也缺少传感器偏离转轴产生的分量。Point-LIO 收到的点云运动与
惯性数据不一致，继续调匹配门限或滤波器协方差无法恢复缺失的运动测量。

当前 Gazebo Harmonic 使用 Bullet Featherstone。该后端的
[KinematicsFeatures.cc](https://github.com/gazebosim/gz-physics/blob/gz-physics7/bullet-featherstone/src/KinematicsFeatures.cc)
从非基座连杆的 `m_absFrameTotVelocity` 缓存读取速度；
[Bullet 实现](https://github.com/bulletphysics/bullet3/blob/master/src/BulletDynamics/Featherstone/btMultiBody.cpp)
只在启用全局速度模式时更新该缓存。本机实际表现与这条代码路径吻合。
因此仅保留 IMU 的 fixed joint 或调整 ROS TF 不能补回这些读数。

另一处问题是 GPU lidar 的一帧点云来自同一时刻，但转换器曾按点序号赋予
0～100 ms 的时间。转动时 Point-LIO 按虚构时序处理这些点，产生错误的运动补偿。

## 仿真修改

- `rm_27_stimulation/src/pose_imu_system.cpp`：模型插件以 200 Hz 采样
  `gimbal_yaw` 的实际姿态和安装外参，计算 IMU 的角速度与比力。
  比力为 `R_sensor_world * (a_world - gravity)`，包含偏心旋转的向心加速度。
- 使用前后采样做中心差分，消息标注中间采样时刻，正常延迟为 5 ms；
  暂停时不发假数据，仿真时间倒退或中断后清除历史。
- `simulation_waking_robot.xacro` 只保留这个 `/livox/imu` 数据源，不再同时启动
  同一机器人原生 IMU 传感器。雷达和 IMU 的安装位置、物理碰撞及关节控制速度不变。
- 两份仿真导航参数启用 `instantaneous_scan: true`，采用实际的 `360 × 192`
  点云布局，`scan_line: 192`，`imu_time_inte: 0.005`。每点时间统一为 1 ns，
  因为本仓库 Point-LIO 把全部为零的时间解释为“没有时间字段”，会再次推算扫描时序。
- Point-LIO 仍用原来的 `use_imu_as_input: false`、一次迭代。未放宽定位质量门限。

这是仿真传感器模型的修复，不是用 Gazebo 真值替换 Point-LIO 里程计。
定位仍经过点云 + IMU + Point-LIO + GICP；真值只用于生成物理传感器测量及独立验收。
IMU 当前为理想模型，未额外加入随机偏置、温漂或振动噪声。

## 复测

重新编译后必须重启 Gazebo，使新插件加载：

```bash
source /opt/ros/jazzy/setup.bash
colcon build --base-paths src --packages-select \
  rm_27_stimulation ign_sim_pointcloud_tool pb2025_nav_bringup --symlink-install
source install/setup.bash
```

运行真实 Point-LIO 链路时必须使用 `use_ground_truth_odom:=false`。
默认真值定位模式不会检验 Point-LIO 是否稳定。

隔离验收工具：

```bash
export ROS_DOMAIN_ID=75
export GZ_PARTITION=rmul2027-map-validation
# 同设置启动 RMUL2027 世界和 Point-LIO 导航后：
python3 tools/mapping/check_rmul2027_localization.py --gimbal
python3 tools/mapping/check_sim_gimbal_imu.py --duration 60
```

后一个工具先做一次坐标对齐，此后不依赖 GICP 修正，直接对照原始
`/aft_mapped_to_init` 与同一时刻的物理传感器位置。覆盖连续旋转、边转边前后移动、
停转回正和倒头。测试输出中的 `sample_age_s` 包含为等待 joint TF 人为保留的
150 ms 比较窗口，不能直接当作 Point-LIO 计算延迟。

2026-10-21 最终配置实测：静止底盘持续转动 60 s（约 1.5 rad/s），随后转动中
前进 5 s、后退 5 s、回正 7 s、倒头 4 s；79 个检查样本的原始 Point-LIO 最大
位置误差为 5.36 mm，最大姿态误差为 0.205°。GICP 测试各阶段的中位 XY 误差
为 1.0～2.9 mm，其中转云台为 1.95 mm。结果仅代表当前理想仿真模型和测试场景，
不等于实车精度保证。

原始报告：[连续转动](validation/gimbal_pointlio_2026-10-21.json)、
[地图定位](validation/rmul2027_localization_2026-10-21.json)。
新增 IMU 数学测试 5 项、相关地图和启动回归测试 21 项通过。
完整旧启动测试集还存在此前车辆尺寸/调参期遗留的 12 项断言不一致；未为通过旧
断言而回退用户参数，不能声称整个仓库测试全绿。

## 实车的不同问题

仿真中的“角速度为零”不能直接推断到真实 MID360。现有代码和最近可找到的
实车日志提供以下排查依据；尚没有本次持续旋转的原始 rosbag，不能认定全部失稳
均由某一项造成。

| 项目 | 当前证据 | 应如何处理 |
| --- | --- | --- |
| 点云/IMU 外参坐标不一致风险 | `mid360_user_config.json` 设置 roll=45°；驱动 `pub_handler.cpp` 对点云及 IMU 都应用该旋转，但 `mapping.extrinsic_T` 仍为原始 `[-0.011,-0.02329,0.04412]` | 先确认这组平移是否来自未旋转的 MID360 原始坐标；如果是，应同步旋转平移，而不是只转点和角速度 |
| IMU 原点被当作雷达原点 | Point-LIO 的 `aft_mapped_to_init` 发布 IMU/body 位姿；`loam_interface` 将它直接标作 `left_mid360`，没有再乘 IMU→LiDAR 平移 | 明确整条链路的位姿原点；先转换 IMU→LiDAR，或统一按 IMU 位姿处理，并同步 URDF，避免重复补偿 |
| 缺少真实活动关节 TF | 实车 `gimbal_yaw_joint` 是 fixed，雷达和 IMU 也直接固定在 `base_link` 下 | 用实测机械 yaw 和正确转轴位置建立动态关节；雷达/IMU 挂到云台。角度需要与点云时间对齐 |
| IMU 数据不连续 | 10 月 18 日 21:27 实车记录中，初始化之后仍出现 31～116 ms 的 IMU 间隔警告 | 检查网络、时间戳、ROS 接收/计算负载；不要仅增大 30 ms 门限掩盖数据缺口 |
| 转动时初始化 | 初始化按平均加速度估计重力，转动的向心/切向加速度会混入 | 启动时先保持雷达静止至初始化完成，再持续转动 |

若 `R = Rx(45°)`，两种数据坐标均被 R 旋转，内部外参应满足：

```text
R_L_I_new = R * R_L_I_raw * R^-1
T_L_I_new = R * T_L_I_raw
```

在原始旋转为单位阵、平移确实是上述原始值的前提下，新的平移约为
`[-0.011,-0.047666,0.014729] m`。这不是建议在未核对坐标前直接粘贴的万能值。
两组平移相差约 3.8 cm，足以造成随转动变化的厘米级误差。

固定关节一定不能表达真实的底盘朝向。但若 yaw 轴恰好穿过底盘参考点，且
传感器到轴心偏置准确，仅利用传感器姿态也可还原轴心位置，所以不能简单地把
所有平移漂移都归咎于缺少机械角。当前需一起核对 IMU/LiDAR 原点、转轴位置和外参。

`fake_vel_transform` 只处理速度方向和虚拟 TF，不会修复 Point-LIO 内部点云/IMU
外参，也不能替代真实关节反馈。

## 实车如何区分原因

底盘保持静止，先静止初始化，再慢转、正常速度转、停转，各录一段：

```bash
ros2 bag record -o /tmp/gimbal_lio_check \
  /livox/lidar /livox/imu /aft_mapped_to_init /cloud_registered \
  /lidar_odometry /odometry /tf /tf_static /joint_states /cmd_vel
```

不存在的 joint_states 不会凭空产生，需要真实反馈节点提供。已有同名 bag 时
换一个输出目录。这个命令只录数据，不发送运动命令。

1. `/cloud_registered` 在固定 `camera_init` 中墙面稳定，只有底盘模型绕圈/摆动：
   优先修原点转换、轴心补偿和关节 TF。
2. 固定 `camera_init` 中墙面也扭曲或漂移：优先查点云/IMU 外参、每点时间和 IMU
   间断。不要先通过放宽导航障碍门限处理。
3. 只有 `map` 下跳变、`camera_init` 下稳定：单独检查 GICP 的 `map→odom` 更新，
   不应将其称为 Point-LIO 本体失稳。

本次仅修复仿真链路，保留实车参数与驱动现状，避免在缺少旋转原始数据时猜测修改。
