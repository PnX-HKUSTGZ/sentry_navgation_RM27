# RM27 Gazebo Harmonic Simulation

这个包是 RM27 导航工程内的 Gazebo Harmonic 仿真包，面向 Ubuntu 24.04 和
ROS 2 Jazzy。它复用了旧工程中可用的 world、mesh、RViz 和
`simulation_waking_robot.xacro`，启动链路已经从 Gazebo Classic / `gazebo_ros`
迁移到 `ros_gz_sim` / `ros_gz_bridge`。

## 目录

- `launch/rm_simulation.launch.py`: 启动 Gazebo Harmonic 和 ros_gz_bridge。
- `launch/spawn_robot.launch.py`: 发布机器人描述并按 `config/spawn_poses.yaml` 生成机器人。
- `launch/sim_with_nav.launch.py`: 同时启动 Gazebo、机器人和 `pb2025_nav_bringup` 仿真导航。
- `config/worlds.yaml`: world key 到 `.world` 文件、导航地图 key 的映射。
- `config/spawn_poses.yaml`: 每个 world 的默认出生点。
- `urdf/simulation_waking_robot.xacro`: RM27 仿真机器人描述，已适配 `base_footprint`、`gimbal_yaw`、`left_mid360`。
- `urdf/mid360.xacro`: 本包内置的 Mid360 近似 GPU LiDAR 仿真描述。
- `config/ros_gz_bridge.yaml`: Gazebo Transport 与 ROS 2 topic 的桥接配置。

## 构建

```bash
cd /home/pnx/nav_ws/sentry-navigation-RM27
source /opt/ros/jazzy/setup.bash
colcon build --packages-select rm_27_stimulation --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## 启动

只启动 Gazebo：

```bash
ros2 launch rm_27_stimulation rm_simulation.launch.py sim_world:=RMUC2026 gui:=true
```

另开终端生成机器人：

```bash
ros2 launch rm_27_stimulation spawn_robot.launch.py sim_world:=RMUC2026
```

一键启动 Gazebo、机器人和 RM27 导航：

```bash
ros2 launch rm_27_stimulation sim_with_nav.launch.py world:=RMUC2026 slam:=False gui:=true use_rviz:=true
```

当前默认 world 是 `RMUC2026`，并且已有对应地图。仿真导航默认使用 Gazebo
真值里程计，避免简化车辆模型的 IMU / Point-LIO 漂移进入导航闭环：

```bash
ros2 launch rm_27_stimulation sim_with_nav.launch.py world:=RMUC2026 slam:=False gui:=false use_rviz:=true
```

只在单独调试 Point-LIO 和 small_gicp 时关闭真值模式：

```bash
ros2 launch rm_27_stimulation sim_with_nav.launch.py world:=RMUC2026 slam:=False use_ground_truth_odom:=false
```

## 可用 world

- `rmuc_2024`
- `rmuc_2025`
- `RMUC2026`
- `rmul_2024`
- `rmul_2024_dynamic`
- `rmul_2026`
- `rmul_2026_wave`
- `rmul_2026_wave_y`
- `rmul_2026_wave_y_neg`

当前旧工程资源里没有 `RMUL2025_world`，所以本包没有伪造一个同名 world。RM27
导航侧虽然已有 `rmul_2025` 地图，但如果要跑严格匹配的 RMUL2025 仿真，后续需要补入
对应的 `.world` 和 mesh，再在 `config/worlds.yaml`、`config/spawn_poses.yaml` 里加一项。

## 注意

- 默认推荐单机器人无 namespace 启动，即 `namespace:=` 保持空值。当前 Gazebo
  Harmonic 传感器发布 `/livox/lidar/points`、`/livox/imu`，并订阅 `/cmd_vel`。
  `ros_gz_bridge` 会把点云桥接回 ROS 侧的 `livox/lidar`。
- 旧的 Classic `ros2_livox` 自定义扫描插件不再编译。当前 Mid360 使用 Harmonic
  原生 `gpu_lidar` 近似 3D 点云，不再发布 `livox_ros_driver2/msg/CustomMsg`。
- 默认真值链路是 `Gazebo OdometryPublisher -> ros_gz_bridge ->
  rm27_ground_truth_localizer`。该节点统一发布 `/odometry`、`map -> odom ->
  base_footprint`，并把 `/livox/lidar` 转换为真值坐标下的 `/registered_scan`；
  `terrain_analysis` 和 `terrain_analysis_ext` 仍正常处理地形点云。
- `use_ground_truth_odom:=false` 才会恢复原有 `/livox/lidar ->
  ign_sim_pointcloud_tool -> Point-LIO -> loam_interface -> sensor_scan_generation`
  和 small_gicp 重定位链路。
- 导航控制链路为 `cmd_vel_nav2_result -> fake_vel_transform -> cmd_vel -> ros_gz_bridge -> Gazebo 底盘控制插件`。
- 默认 `physics_engine:=gz-physics-dartsim-plugin`。底盘控制器只施加平面力和偏航力矩，
  DART 负责 STL mesh collision、重力和地形接触，因此 `z/roll/pitch` 会随坡面变化。
- 仿真 Mid360 逻辑 frame 为 `base_link -> left_mid360`: `xyz="0.0 0.18 0.14"`、
  `rpy="0 0 0"`；`imu_link` 使用同一逻辑位姿，Point-LIO 仿真外参为
  `extrinsic_T=[0.0, 0.0, 0.0]`、`extrinsic_R=I`。
- `meshes/mid360.stl` 目前是 Git LFS 指针文件，因此 URDF 暂用简单盒子作为 Mid360
  可视模型，避免 RViz 加载坏 STL。
- 目前只有 `RMUC2026_world/meshes/RMUC2026.stl` 是可直接使用的真实 mesh；其他旧 world
  下的 `.stl` 多数还是 Git LFS 指针文件，不能作为当前定位问题的验证基准。
- 仿真 Point-LIO 参数按当前 URDF 配置：Gazebo IMU 为 100Hz，仿真直接输出 roll
  补偿后的逻辑雷达/IMU frame，重力为 `[0.0, 0.0, -9.81]`。如果后续改 IMU 与雷达之间的相对位姿，需要同步更新
  `pb2025_nav_bringup/config/simulation/nav2_params.yaml`。
- `rmul_2024_dynamic` 中的 Classic 动态障碍物插件已从 world 加载链路移除；障碍物模型
  仍会静态加载。若需要恢复动态障碍物，需要后续单独实现 gz-sim system plugin 或 SDF 动画。
- 这个包不包含射击、官方 RMOSS/GZ 仿真器或 `sentry_robot_description` 依赖。
