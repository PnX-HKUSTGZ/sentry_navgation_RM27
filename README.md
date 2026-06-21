
## 1. Overview

哨兵导航RM-27赛季主仓（ROS 2 Humble / Livox MID360 / Nav2）

基于PB25导航框架

本项目基于 [NAV2 导航框架](https://github.com/ros-navigation/navigation2) 并参考学习了 [autonomous_exploration_development_environment](https://github.com/HongbiaoZ/autonomous_exploration_development_environment/tree/humble) 的设计。

- 关于坐标变换：

    本项目大幅优化了坐标变换逻辑，考虑雷达原点 `lidar_odom` 与 底盘原点 `odom` 之间的隐式变换。

    Mid360 逻辑坐标系按 `base_link -> left_mid360` 为 `xyz="0.0 0.18 0.28"`、`rpy="0 0 0"` 维护，实车物理 `roll=-45°` 在 Livox driver 层补偿。系统使用 [point_lio](https://github.com/SMBU-PolarBear-Robotics-Team/point_lio/tree/RM2025_SMBU_auto_sentry) 里程计，[small_gicp](https://github.com/SMBU-PolarBear-Robotics-Team/small_gicp_relocalization) 重定位，[loam_interface](./src/loam_interface/) 会将 point_lio 输出的 `/cloud_registered` 从 `lidar_odom` 系转换到 `odom` 系，[sensor_scan_generation](./src/sensor_scan_generation/) 将 `odom` 系的点云转换到 `left_mid360` 系，并发布变换 `odom -> chassis`。

    ![frames_2025_03_26](https://raw.githubusercontent.com/LihanChen2004/picx-images-hosting/master/frames_2025_03_26.67xmq3djvx.webp)

- 关于路径规划：

    使用 NAV2 默认的 Global Planner 作为全局路径规划器，pb_omni_pid_pursuit_controller 作为路径跟踪器。

- namespace：

    当前 RM27 单机器人仿真默认使用空 namespace，因为 `rm_27_stimulation` 中的 Gazebo 插件发布 `/livox/lidar`、`/livox/imu` 并订阅 `/cmd_vel`。如后续要扩展多机器人，需要统一改 Gazebo 插件、点云转换、Point-LIO、Nav2 和 TF 的 namespace/remap。

- LiDAR:

    Livox Mid360 位于 `base_link` 左侧 0.18 m、上方 0.28 m。RM27 采用旧工程同款坐标方案：TF 中的 `left_mid360` 是 roll 补偿后的逻辑雷达 frame，实车物理 `roll=-45°` 写在 Livox driver 的 `extrinsic_parameter.roll`，不要再在 TF 里重复 roll。

    注：仿真环境中，Gazebo Mid360 插件先发布 `/livox/lidar`，再由 [ign_sim_pointcloud_tool](./src/ign_sim_pointcloud_tool/) 转为带 `ring/time` 字段的 `/velodyne_points` 供 Point-LIO 使用。

- 文件结构

    ```plaintext
    .
    └── src
        ├── fake_vel_transform              # 虚拟速度参考坐标系，以应对云台扫描模式自旋，详见子仓库 README
        ├── ign_sim_pointcloud_tool         # 仿真器点云处理工具
        ├── livox_ros_driver2               # Livox 驱动
        ├── loam_interface                  # point_lio 等里程计算法接口
        ├── pb_teleop_twist_joy             # 手柄控制
        ├── pb2025_nav_bringup              # 启动文件
        ├── pb2025_sentry_nav               # 本仓库功能包描述文件
        ├── pb_omni_pid_pursuit_controller  # 路径跟踪控制器
        ├── point_lio                       # 里程计
        ├── pointcloud_to_laserscan         # 将 terrain_map 转换为 laserScan 类型以表示障碍物（仅 SLAM 模式启动）
        ├── rm_27_stimulation               # RM27 Gazebo Classic 独立仿真包
        ├── sensor_scan_generation          # 点云相关坐标变换
        ├── small_gicp_relocalization       # 重定位
        ├── terrain_analysis                # 距车体 4m 范围内地形分析，将障碍物离地高度写入 PointCloud intensity
        └── terrain_analysis_ext            # 车体 4m 范围外地形分析，将障碍物离地高度写入 PointCloud intensity
    ```

## 2. Quick Start

### 2.1 Option 1: Docker

#### 2.1.1 Setup Environment

- [Docker](https://docs.docker.com/engine/install/)

- 允许 Docker Container 访问宿主机 X11 显示

    ```bash
    xhost +local:docker
    ```

#### 2.1.2 Create Container

```bash
docker run -it --rm --name pb2025_sentry_nav \
  --network host \
  -e "DISPLAY=$DISPLAY" \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /dev:/dev \
  ghcr.io/smbu-polarbear-robotics-team/pb2025_sentry_nav:1.3.2
```

### 2.2 Option 2: Build From Source

#### 2.2.1 Setup Environment

- Ubuntu 22.04
- ROS: [Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
- Gazebo Classic 仿真依赖：本仓库内置 `src/rm_27_stimulation`，当前不依赖 `rmu_gazebo_simulator`。
- Install [small_icp](https://github.com/koide3/small_gicp):

    ```bash
    sudo apt install -y libeigen3-dev libomp-dev

    git clone https://github.com/koide3/small_gicp.git
    cd small_gicp
    mkdir build && cd build
    cmake .. -DCMAKE_BUILD_TYPE=Release && make -j
    sudo make install
    ```

#### 2.2.2 Create Workspace

```bash
mkdir -p ~/ros_ws
cd ~/ros_ws
```

```bash
git clone --recursive https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_sentry_nav.git src/pb2025_sentry_nav
```

下载先验点云:

先验点云用于 point_lio 和 small_gicp，由于点云文件体积较大，故不存储在 git 中，请前往 [FlowUs](https://flowus.cn/lihanchen/share/87f81771-fc0c-4e09-a768-db01f4c136f4?code=4PP1RS) 下载。

> 当前 point_lio with prior_pcd 在大场景的效果并不好，比不带先验点云更容易飘，待 Debug 优化

#### 2.2.3 Build

```bash
rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
```

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

> [!NOTE]
> 推荐使用 --symlink-install 选项来构建你的工作空间，因为 pb2025_sentry_nav 广泛使用了 launch.py 文件和 YAML 文件。这个构建参数会为那些非编译的源文件使用符号链接，这意味着当你调整参数文件时，不需要反复重建，只需要重新启动即可。

### 2.3 Running

可使用以下命令启动，在 RViz 中使用 `Nav2 Goal` 插件发布目标点。

#### 2.3.1 仿真

单机器人：

推荐一键启动 Gazebo、机器人和 RM27 导航：

```bash
ros2 launch rm_27_stimulation sim_with_nav.launch.py \
world:=RMUC2026 \
slam:=False \
gui:=false \
use_rviz:=true
```

如果 Gazebo 和机器人已经单独启动，也可以只启动导航：

```bash
ros2 launch pb2025_nav_bringup rm_navigation_simulation_launch.py \
world:=RMUC2026 \
slam:=False \
namespace:=
```

建图模式：

```bash
ros2 launch rm_27_stimulation sim_with_nav.launch.py \
world:=RMUC2026 \
slam:=True \
gui:=false \
use_rviz:=true
```

保存栅格地图：`ros2 run nav2_map_server map_saver_cli -f <YOUR_MAP_NAME>`

当前 RM27 仿真和实车启动链均按单机器人维护，默认保持空 namespace；多机器人启动入口暂不作为当前稳定版本使用。

#### 2.3.2 实车

建图模式：

```bash
ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py \
slam:=True
```

保存栅格地图：`ros2 run nav2_map_server map_saver_cli -f <YOUR_MAP_NAME>`

导航模式：

注意修改 `world` 参数为实际地图的名称

```bash
ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py \
world:=<YOUR_WORLD_NAME> \
slam:=False
```

### 2.4 Launch Arguments

启动参数在仿真和实车中大部分是通用的。以下是所有启动参数表格的图例。

| 符号 | 含义                       |
| ---- | -------------------------- |
| 🤖    | 适用于实车           |
| 🖥️    | 适用于仿真                 |

| 可用性 | 参数 | 描述 | 类型  | 默认值 |
|-|-|-|-|-|
| 🤖 🖥️ | `namespace` | 顶级命名空间；当前单机器人仿真保持空值 | string | 仿真: ""; 实车: "" |
| 🤖🖥️ | `use_sim_time` | 如果为 True，则使用仿真（Gazebo）时钟 | bool | 仿真: True; 实车: False |
| 🤖 🖥️ | `slam` | 是否启用建图模式。如果为 True，则禁用 small_gicp 并发送静态 tf（map->odom）。然后自动保存 pcd 文件到 [PCD](./src/point_lio/PCD/)| bool | False |
| 🤖 🖥️ | `world` | 在仿真模式，当前优先验证 `RMUC2026`；其他旧 world 需要确认 mesh/map/pcd 是否齐全 | string | 仿真: "RMUC2026" |
|  |  | 在实车模式，`world` 参数名称与栅格地图和先验点云图的文件名称相同 | string | "" |
| 🤖 🖥️ | `map` | 要加载的地图文件的完整路径。默认路径自动基于 `world` 参数构建 | string | 仿真: [RMUC2026.yaml](./src/pb2025_nav_bringup/map/simulation/RMUC2026.yaml); 实车: 自动填充 |
| 🤖 🖥️ | `prior_pcd_file` | 要加载的先验 pcd 文件的完整路径。默认路径自动基于 `world` 参数构建 | string | 仿真: [RMUC2026.pcd](./src/pb2025_nav_bringup/pcd/simulation/RMUC2026.pcd); 实车: 自动填充 |
| 🤖 🖥️ | `params_file` | 用于所有启动节点的 ROS2 参数文件的完整路径 | string | 仿真: [nav2_params.yaml](./src/pb2025_nav_bringup/config/simulation/nav2_params.yaml); 实车: [nav2_params.yaml](./src/pb2025_nav_bringup/config/reality/nav2_params.yaml) |
| 🤖🖥️ | `rviz_config_file` | 要使用的 RViz 配置文件的完整路径 | string | [nav2_default_view.rviz](./src/pb2025_nav_bringup/rviz/nav2_default_view.rviz) |
| 🤖 🖥️ | `autostart` | 自动启动 nav2 栈 | bool | True |
| 🤖 🖥️ | `use_composition` | 是否使用 Composable Node 形式启动 | bool | False |
| 🤖 🖥️ | `use_respawn` | 如果节点崩溃，是否重新启动。本参数仅 `use_composition:=False` 时有效 | bool | False |
| 🤖🖥️ | `use_rviz` | 是否启动 RViz | bool | True |
| 🤖 | `use_robot_state_pub` | 是否使用 `robot_state_publisher` 发布机器人的 TF 信息。当前 `rm_27_stimulation` 仿真包会在 `spawn_robot.launch.py` 中启动自己的机器人描述；实车建议由完整机器人系统或 RM27 专用描述包维护 TF。只有确认实车描述文件已经按 `left_mid360`、`gimbal_yaw` 等 RM27 frame 接好后才设为 True。 | bool | False |

> [!TIP]
> 关于本项目更多细节与实车部署指南，请前往 [Wiki](https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_sentry_nav/wiki)

### 2.5 手柄控制

默认情况下，PS4 手柄控制已开启。键位映射关系详见 [nav2_params.yaml](./src/pb2025_nav_bringup/config/simulation/nav2_params.yaml) 中的 `teleop_twist_joy_node` 部分。

![teleop_twist_joy.gif](https://raw.githubusercontent.com/LihanChen2004/picx-images-hosting/master/teleop_twist_joy.5j4aav3v3p.gif)
