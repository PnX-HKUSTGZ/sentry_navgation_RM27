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
- `tools/import_rm27_robot_cad.py`: 从 SolidWorks 分件 STL 压缩包生成轻量整车视觉网格。
- `config/ros_gz_bridge.yaml`: Gazebo Transport 与 ROS 2 topic 的桥接配置。

## 构建

```bash
cd /home/pnx/nav_ws/sentry-navigation-RM27
source /opt/ros/jazzy/setup.bash
colcon build --packages-select rm_27_stimulation --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## RMUC2026 场地资源

Gazebo 使用 `RM27_Gazebo_Harmonic_part1.tar.xz` 和 `part2.tar.xz` 中的详细场地：
85 份 DAE、185 个视觉对象，以及 548 份凸 STL 组成的 607 个碰撞体。视觉与碰撞
分别放在 `meshes/RM27_battlefield_visual` 和
`meshes/RM27_battlefield_collision`，以相同 pose 加载，避免渲染模型参与物理计算。

原始 `meshes/RMUC2026_world/meshes/RMUC2026.stl` 保留给先验高程和地图工具，
不再作为 Gazebo 场地碰撞。需要从压缩包重新生成两套模型时运行：

```bash
cd /home/pnx/nav_ws/sentry-navigation-RM27
python3 src/rm_27_stimulation/tools/import_rm27_battlefield_visual.py --force
```

源场地以 `y=0` 为中心，world 对视觉和碰撞统一使用 `y=1.624344 m` 的平移恢复
现有地图坐标；地面仍为 `z=0`。不得只调整其中一个 pose，否则画面、碰撞、雷达
量测和导航地图会错位。

## RM27 整车 CAD

仿真车使用仓库根目录 `哨922stl.zip` 中的新版 SolidWorks 分件 STL。导入器排除地面、
放置、干涉和安装要求等设计辅助体，并以 2 mm 网格聚类合并为 body、wheels、armor、
sensors 四组。当前输出约 50.9 万个三角面；旧雄安车 mesh 已直接被这版覆盖。重新生成命令：

```bash
cd /home/pnx/nav_ws/sentry-navigation-RM27
python3 src/rm_27_stimulation/tools/import_rm27_robot_cad.py --force
```

源 CAD 已经是 roll 轴翻下姿态。实体包围盒为约
`0.592 x 0.594 x 0.248 m`，CAD `Y` 轴映射到 ROS `Z` 轴；导入器以四个轮盘中心自动
确定车体中心和轮轴高度。源文件的大、小 yaw 保留了装配时的偏转；导入器根据雷达支架和相机侧板
分别归零到车头 `+X`，并把雷达及支架恢复到原安装位置，使用 `-30 deg` roll。
归零角和安装坐标记录在 `import_metadata.json` 的 `gimbal_alignment` 中。
Xacro 默认 `use_cad_visual:=true`、
`cad_visual_scale:=1.0`，不再显示旧方盒/圆柱占位外观。

物理和导航模型同步使用 CAD 提取值：

- 四个轮心位于 `(+/-0.218, +/-0.218) m`，含滚子的等效接触半径为 `0.080 m`。
- 车体用缩小后的中心圆柱和上层方盒近似碰撞，轮地接触用四个球体；
  高面数视觉 STL 不参与 Bullet 碰撞。
- Nav2 与 MINCO 使用同一个 CAD 凸包外扩 15 mm 的 16 点安全轮廓，最大半径为
  `0.392 m`。
- Mid360 相对 `base_link` 恢复为 `[0.0, 0.126, 0.130] m`，roll 为 `-30 deg`，
  雷达原点离地高度约 `0.210 m`；
  ROG 折叠车高为 `0.25 m`。
- ROG 的 `projection.sensor_mount_rpy: [-0.523598775598, 0.0, 0.0]` 从雷达里程计
  扣除固定安装角，再计算地面切平面和车体方向；点云变换和射线起点仍使用真实雷达姿态。
  不能把该参数当作地面坡度或直接从点云坐标里去掉 roll。
- STL 不包含密度、整车质量、质心或轮胎摩擦。当前保留已验证的 `10.2 kg` 总质量和
  `0.05` 全向底盘等效滑动摩擦，惯量按 CAD 尺寸重新计算。拿到称重与推行试验数据后，
  集中修改 Xacro 顶部的 `body_mass`、`wheel_mass`、`omni_friction`，并重新验证控制器力限幅。

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

### 核显加速

两个仿真启动入口默认启用 `hardware_acceleration:=true`。`gui:=false` 时使用
Gazebo 原生 `--headless-rendering`，通过 EGL 渲染雷达；`gui:=true` 时优先寻找
`vglrun`，包括 NoMachine 自带的 `/usr/NX/scripts/vgl/vglrun`，通过 VirtualGL
的 EGL 后端渲染 Gazebo 窗口。没有 VirtualGL 的机器保留桌面 OpenGL 启动方式。
该设置只作用于 Gazebo，不向导航节点注入 VirtualGL，也不改变物理、碰撞或雷达采样。

远程桌面下启动 MINCO 仿真：

```bash
ros2 launch rm_27_stimulation sim_with_nav.launch.py world:=RMUC2026 navigation_mode:=minco gui:=true use_rviz:=true
```

启动后查看 `~/.gz/rendering/ogre2.log` 中的 `GL_RENDERER`。当前 Intel 机器应显示
`Mesa Intel(R) Graphics (ADL GT2)`；`llvmpipe` 表示仍在软件渲染。核显驱动或权限
不可用时，EGL 也可能回退到软件渲染，不能仅凭启动参数判断加速已经生效。
需要对比原启动方式时加 `hardware_acceleration:=false`。

实车的 Point-LIO、ROG 地图、MINCO 规划和 MPC 控制当前使用 CPU，不依赖核显。
实车、仿真和单独启动 RViz 都默认启用 `rviz_hardware_acceleration:=true`，
使用同样的 VirtualGL / EGL 后端减少显示占用的 CPU。该开关独立于 Gazebo 的
`hardware_acceleration`，只作用于 RViz 进程；需要恢复桌面 OpenGL 时加
`rviz_hardware_acceleration:=false`。没有 VirtualGL 时使用桌面 OpenGL。
不需要本机显示时可关闭 RViz。

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
- 默认 `physics_engine:=gz-physics-bullet-featherstone-plugin`。Bullet Featherstone 能加载
  场地的凸 STL 碰撞代理；底盘控制器只施加平面力和偏航力矩，`z/roll/pitch` 由重力
  和真实地形接触决定。DART 不支持从当前 SDF 创建这些 mesh collision，不能用于该场地。
- 机器人四个球形轮面同时显式配置 ODE/Bullet 摩擦系数 `0.05`，导入器也会把场地源模型
  的 ODE 摩擦系数 `0.8` 同步到 Bullet。URDF 的 `mu1/mu2` 只会生成 ODE 参数；如果遗漏
  Bullet 参数，Bullet 会使用自身默认摩擦，使低速 `/cmd_vel` 存在但机器人无法起步。
  四个 `base_to_wheel*` 固定关节设置 `preserveFixedJoint=true`：Bullet Featherstone
  对同一个 link 的复合碰撞体只采用第一个碰撞体的摩擦参数。保留独立轮链接才能让轮子的
  `0.05` 生效，避免 URDF 合并固定关节后用底盘默认摩擦覆盖轮地接触。
- 仿真 Mid360 逻辑 frame 为 `base_link -> left_mid360`:
  `xyz="0.0 0.126 0.130"`、`rpy="-0.523598775598 0 0"`；`imu_link` 使用同一安装位姿，Point-LIO 仿真外参为
  `extrinsic_T=[0.0, 0.0, 0.0]`、`extrinsic_R=I`。
- `meshes/mid360.stl` 仍是 Git LFS 指针文件；默认 CAD 外观已经包含真实 Mid360，只有
  `use_cad_visual:=false` 时才显示简单盒子回退模型。
- `RM27_battlefield_collision` 和 `RMUC2026_world/meshes/RMUC2026.stl` 是可直接使用的
  真实 mesh；其他旧 world 下的 `.stl` 多数还是 Git LFS 指针文件，不能作为当前定位
  问题的验证基准。
- 仿真 Point-LIO 参数按当前 URDF 配置：Gazebo IMU 为 100Hz，雷达/IMU frame 均保留
  真实的 `-30 deg` 安装 roll，重力在世界坐标中为 `[0.0, 0.0, -9.81]`。如果后续改 IMU 与雷达之间的相对位姿，需要同步更新
  `pb2025_nav_bringup/config/simulation/minco_params.yaml` 和 legacy 的 `nav2_params.yaml`。
- `rmul_2024_dynamic` 中的 Classic 动态障碍物插件已从 world 加载链路移除；障碍物模型
  仍会静态加载。若需要恢复动态障碍物，需要后续单独实现 gz-sim system plugin 或 SDF 动画。
- 这个包不包含射击、官方 RMOSS/GZ 仿真器或 `sentry_robot_description` 依赖。
