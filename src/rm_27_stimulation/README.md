# RM27 Gazebo Simulation

这个包是 RM27 导航工程内的独立 Gazebo Classic 仿真包，结构参考
`sentry-navigation2/src/rm_simulation/pb_rm_simulation`。它复用了旧工程中可用的
world、mesh、RViz 和 `simulation_waking_robot.xacro`，并把 Livox Mid360 Gazebo
插件源码合入本包构建，避免继续依赖旧 workspace 的 `ros2_livox_simulation` 包。

## 目录

- `launch/rm_simulation.launch.py`: 只启动 Gazebo server/client 和指定 world。
- `launch/spawn_robot.launch.py`: 发布机器人描述并按 `config/spawn_poses.yaml` 生成机器人。
- `launch/sim_with_nav.launch.py`: 同时启动 Gazebo、机器人和 `pb2025_nav_bringup` 仿真导航。
- `config/worlds.yaml`: world key 到 `.world` 文件、导航地图 key 的映射。
- `config/spawn_poses.yaml`: 每个 world 的默认出生点。
- `urdf/simulation_waking_robot.xacro`: RM27 仿真机器人描述，已适配 `base_footprint`、`gimbal_yaw`、`left_mid360`。
- `urdf/mid360.xacro`、`scan_mode/mid360.csv`: 本包内置的 Livox Mid360 仿真描述和扫描表。

## 构建

```bash
cd /home/pnx/nav_ws/sentry-navigation-RM27
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
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

当前默认 world 是 `RMUC2026`，并且已有对应的地图和 PCD。使用 small_gicp 重定位模式：

```bash
ros2 launch rm_27_stimulation sim_with_nav.launch.py world:=RMUC2026 slam:=False gui:=false use_rviz:=true
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

- 默认推荐单机器人无 namespace 启动，即 `namespace:=` 保持空值。当前 Gazebo 插件发布
  `/livox/lidar`、`/livox/imu` 并订阅 `/cmd_vel`，这与 RM27 仿真参数默认话题最匹配。
- Mid360 插件在 `/livox/lidar` 发布 `sensor_msgs/msg/PointCloud2`，在
  `/livox/lidar/custom` 保留 `livox_ros_driver2/msg/CustomMsg` 调试输出。
- RM27 仿真点云链路是 `/livox/lidar -> ign_sim_pointcloud_tool -> /velodyne_points
  -> Point-LIO -> /cloud_registered -> loam_interface -> /registered_scan`。
- `slam:=False` 会走 RM27 原有 small_gicp 重定位路径，需要
  `pb2025_nav_bringup/pcd/simulation/<nav_world>.pcd` 存在；当前 RMUC2026 对应
  `pb2025_nav_bringup/pcd/simulation/RMUC2026.pcd`。
- Gazebo `planar_move` 只负责接收 `/cmd_vel` 并推动机器人，不发布 `/odometry`
  或 `odom -> base_footprint`。RM27 仿真导航链使用
  `Point-LIO -> loam_interface -> sensor_scan_generation` 生成 `/odometry` 和
  `odom -> base_footprint`，再由 small_gicp 发布 `map -> odom`。
- 导航控制链路为 `cmd_vel_nav2_result -> fake_vel_transform -> cmd_vel -> Gazebo planar_move`。
- 仿真 Mid360 逻辑 frame 为 `base_link -> left_mid360`: `xyz="0.0 0.18 0.28"`、
  `rpy="0 0 0"`；`imu_link` 使用同一逻辑位姿，Point-LIO 仿真外参为
  `extrinsic_T=[0.0, 0.0, 0.0]`、`extrinsic_R=I`。
- `meshes/mid360.stl` 目前是 Git LFS 指针文件，因此 URDF 暂用简单盒子作为 Mid360
  可视模型，避免 RViz 加载坏 STL。
- 目前只有 `RMUC2026_world/meshes/RMUC2026.stl` 是可直接使用的真实 mesh；其他旧 world
  下的 `.stl` 多数还是 Git LFS 指针文件，不能作为当前定位问题的验证基准。
- 仿真 Point-LIO 参数按当前 URDF 配置：Gazebo IMU 为 100Hz，仿真直接输出 roll
  补偿后的逻辑雷达/IMU frame，重力为 `[0.0, 0.0, -9.81]`。如果后续改 IMU 与雷达之间的相对位姿，需要同步更新
  `pb2025_nav_bringup/config/simulation/nav2_params.yaml`。
- 这个包不包含射击、官方 RMOSS/GZ 仿真器或 `pb2025_robot_description` 依赖。
