# pb2025_sentry_nav

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![Build and Test](https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_sentry_nav/actions/workflows/build_and_test.yml/badge.svg)](https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_sentry_nav/actions/workflows/build_and_test.yml)
[![pre-commit](https://img.shields.io/badge/pre--commit-enabled-brightgreen?logo=pre-commit)](https://github.com/pre-commit/pre-commit)

深圳北理莫斯科大学 北极熊战队 2025 赛季哨兵导航仿真/实车包

![PolarBear Logo](https://raw.githubusercontent.com/SMBU-PolarBear-Robotics-Team/.github/main/.docs/image/polarbear_logo_text.png)

[BiliBili: 谁说在家不能调车！？更适合新手宝宝的 RM 导航仿真](https://www.bilibili.com/video/BV12qcXeHETR)

https://github.com/user-attachments/assets/d9e778e0-fa43-40c2-96c2-e71eaf7737d4

https://github.com/user-attachments/assets/ae4c19a0-4c73-46a0-95bd-909734da2a42

## 1. Overview

本项目基于 [NAV2 导航框架](https://github.com/ros-navigation/navigation2) 并参考学习了 [autonomous_exploration_development_environment](https://github.com/HongbiaoZ/autonomous_exploration_development_environment/tree/humble) 的设计。

- 关于坐标变换：

    本项目大幅优化了坐标变换逻辑，考虑雷达原点 `lidar_odom` 与 底盘原点 `odom` 之间的隐式变换。

    mid360 倾斜侧放在底盘上，使用 [point_lio](https://github.com/SMBU-PolarBear-Robotics-Team/point_lio/tree/RM2025_SMBU_auto_sentry) 里程计，[small_gicp](https://github.com/SMBU-PolarBear-Robotics-Team/small_gicp_relocalization) 重定位，[loam_interface](./loam_interface/) 会将 point_lio 输出的 `/cloud_registered` 从 `lidar_odom` 系转换到 `odom` 系，[sensor_scan_generation](./sensor_scan_generation/) 将 `odom` 系的点云转换到 `front_mid360` 系，并发布变换 `odom -> chassis`。

    ![frames_2025_03_26](https://raw.githubusercontent.com/LihanChen2004/picx-images-hosting/master/frames_2025_03_26.67xmq3djvx.webp)

- 关于路径规划：

    使用 NAV2 默认的 Global Planner 作为全局路径规划器，pb_omni_pid_pursuit_controller 作为路径跟踪器。

- namespace：

    为了后续拓展多机器人，本项目引入 namespace 的设计，与 ROS 相关的 node, topic, action 等都加入了 namespace 前缀。如需查看 tf tree，请使用命令 `ros2 run rqt_tf_tree rqt_tf_tree --ros-args -r /tf:=tf -r /tf_static:=tf_static -r  __ns:=/red_standard_robot1`

- LiDAR:

    Livox mid360 倾斜侧放在底盘上。

    注：仿真环境中，实际上 point pattern 为 velodyne 样式的机械式扫描。此外，由于仿真器中输出的 PointCloud 缺少部分 field，导致 point_lio 无法正常估计状态，故仿真器输出的点云经过 [ign_sim_pointcloud_tool](./ign_sim_pointcloud_tool/) 处理添加 `time` field。

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
- 配套仿真包（Option）：[rmu_gazebo_simulator](https://github.com/SMBU-PolarBear-Robotics-Team/rmu_gazebo_simulator)
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

导航模式：

```bash
ros2 launch pb2025_nav_bringup rm_navigation_simulation_launch.py \
world:=rmuc_2025 \
slam:=False
```

建图模式：

```bash
ros2 launch pb2025_nav_bringup rm_navigation_simulation_launch.py \
slam:=True
```

保存栅格地图：`ros2 run nav2_map_server map_saver_cli -f <YOUR_MAP_NAME>  --ros-args -r __ns:=/red_standard_robot1`

多机器人 (实验性功能) :

当前指定的初始位姿实际上是无效的。TODO: 加入 `map` -> `odom` 的变换和初始化

```bash
ros2 launch pb2025_nav_bringup rm_multi_navigation_simulation_launch.py \
world:=rmul_2024 \
robots:=" \
red_standard_robot1={x: 0.0, y: 0.0, yaw: 0.0}; \
blue_standard_robot1={x: 5.6, y: 1.4, yaw: 3.14}; \
"
```

#### 2.3.2 实车

建图模式：

```bash
ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py \
slam:=True \
use_robot_state_pub:=True
```

保存栅格地图：`ros2 run nav2_map_server map_saver_cli -f <YOUR_MAP_NAME>  --ros-args -r __ns:=/red_standard_robot1`

导航模式：

注意修改 `world` 参数为实际地图的名称

```bash
ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py \
world:=<YOUR_WORLD_NAME> \
slam:=False \
use_robot_state_pub:=True
```

### 2.4 Launch Arguments

启动参数在仿真和实车中大部分是通用的。以下是所有启动参数表格的图例。

| 符号 | 含义                       |
| ---- | -------------------------- |
| 🤖    | 适用于实车           |
| 🖥️    | 适用于仿真                 |

| 可用性 | 参数 | 描述 | 类型  | 默认值 |
|-|-|-|-|-|
| 🤖 🖥️ | `namespace` | 顶级命名空间 | string | "red_standard_robot1" |
| 🤖🖥️ | `use_sim_time` | 如果为 True，则使用仿真（Gazebo）时钟 | bool | 仿真: True; 实车: False |
| 🤖 🖥️ | `slam` | 是否启用建图模式。如果为 True，则禁用 small_gicp 并发送静态 tf（map->odom）。然后自动保存 pcd 文件到 [PCD](./src/point_lio/PCD/)| bool | False |
| 🤖 🖥️ | `world` | 在仿真模式，可用选项为 `rmul_2024` 或 `rmuc_2024` 或 `rmul_2025` 或 `rmuc_2025` | string | "rmuc_2025" |
|  |  | 在实车模式，`world` 参数名称与栅格地图和先验点云图的文件名称相同 | string | "" |
| 🤖 🖥️ | `map` | 要加载的地图文件的完整路径。默认路径自动基于 `world` 参数构建 | string | 仿真: [rmuc_2025.yaml](./src/pb2025_nav_bringup/map/simulation/rmuc_2025.yaml); 实车: 自动填充 |
| 🤖 🖥️ | `prior_pcd_file` | 要加载的先验 pcd 文件的完整路径。默认路径自动基于 `world` 参数构建 | string | 仿真: [rmuc_2025.pcd](./src/pb2025_nav_bringup/pcd/reality/); 实车: 自动填充 |
| 🤖 🖥️ | `params_file` | 用于所有启动节点的 ROS2 参数文件的完整路径 | string | 仿真: [nav2_params.yaml](./src/pb2025_nav_bringup/config/simulation/nav2_params.yaml); 实车: [nav2_params.yaml](./src/pb2025_nav_bringup/config/reality/nav2_params.yaml) |
| 🤖🖥️ | `rviz_config_file` | 要使用的 RViz 配置文件的完整路径 | string | [nav2_default_view.rviz](./src/pb2025_nav_bringup/rviz/nav2_default_view.rviz) |
| 🤖 🖥️ | `autostart` | 自动启动 nav2 栈 | bool | True |
| 🤖 🖥️ | `use_composition` | 是否使用 Composable Node 形式启动 | bool | True |
| 🤖 🖥️ | `use_respawn` | 如果节点崩溃，是否重新启动。本参数仅 `use_composition:=False` 时有效 | bool | False |
| 🤖🖥️ | `use_rviz` | 是否启动 RViz | bool | True |
| 🤖 | `use_robot_state_pub` | 是 是否使用 `robot_state_publisher` 发布机器人的 TF 信息 <br> 1. 在仿真中，由于支持的 Gazebo 仿真器已经发布了机器人的 TF 信息，因此不需要再次发布。 <br> 2. 在实车中，**推荐**使用独立的包发布机器人的 TF 信息。例如，`gimbal_yaw` 和 `gimbal_pitch` 关节位姿由串口模块 [standard_robot_pp_ros2](https://github.com/SMBU-PolarBear-Robotics-Team/standard_robot_pp_ros2) 提供，此时应将 `use_robot_state_pub` 设置为 False。 <br> 如果没有完整的机器人系统或仅测试导航模块（此仓库）时，可将 `use_robot_state_pub` 设置为 True。此时，导航模块将发布静态的机器人关节位姿数据以维护 TF 树。 <br> *注意：需额外克隆并编译 [pb2025_robot_description](https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_robot_description.git)* | bool | False |

> [!TIP]
> 关于本项目更多细节与实车部署指南，请前往 [Wiki](https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_sentry_nav/wiki)

### 2.5 手柄控制

默认情况下，PS4 手柄控制已开启。键位映射关系详见 [nav2_params.yaml](./src/pb2025_nav_bringup/config/simulation/nav2_params.yaml) 中的 `teleop_twist_joy_node` 部分。

![teleop_twist_joy.gif](https://raw.githubusercontent.com/LihanChen2004/picx-images-hosting/master/teleop_twist_joy.5j4aav3v3p.gif)
