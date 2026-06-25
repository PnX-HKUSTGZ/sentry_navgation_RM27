# pb2025_sentry_nav

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![Build and Test](https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_sentry_nav/actions/workflows/ci.yml/badge.svg)](https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_sentry_nav/actions/workflows/ci.yml)
[![pre-commit](https://img.shields.io/badge/pre--commit-enabled-brightgreen?logo=pre-commit)](https://github.com/pre-commit/pre-commit)

Shenzhen MSU-BIT University PolarBear Robotics Team's Sentry Navigation Simulation/Reality Robot Package For RoboMaster 2025.

![PolarBear Logo](https://raw.githubusercontent.com/SMBU-PolarBear-Robotics-Team/.github/main/.docs/image/polarbear_logo_text.png)

[BiliBili: RM navigation simulation for beginners](https://www.bilibili.com/video/BV12qcXeHETR)

https://github.com/user-attachments/assets/d9e778e0-fa43-40c2-96c2-e71eaf7737d4

https://github.com/user-attachments/assets/ae4c19a0-4c73-46a0-95bd-909734da2a42

## 1. Overview

This project is based on the [NAV2 Navigation Framework](https://github.com/ros-navigation/navigation2) and references the design of [autonomous_exploration_development_environment](https://github.com/HongbiaoZ/autonomous_exploration_development_environment/tree/humble).

- Coordinate Transformation：

    This project has optimized coordinate transformation logic significantly, considering the implicit transformation between the radar origin `lidar_odom` and the chassis origin `odom`.

    The left Livox Mid360 is compensated in the Livox driver and exposed as `left_mid360`. It uses [point_lio](https://github.com/SMBU-PolarBear-Robotics-Team/point_lio/tree/RM2025_SMBU_auto_sentry) as odometry, [relocalization_manager](./src/relocalization_manager/) to manage relocalization while using small_gicp internally for point cloud verification, and [loam_interface](./loam_interface/) will transform PointCloud from `lidar_odom` frame to `odom` frame. The [sensor_scan_generation](./sensor_scan_generation/) transform PointCloud from `odom` frame to `left_mid360` frame and publishes the transform `odom -> chassis`.

    ![frames_2025_03_26](https://raw.githubusercontent.com/LihanChen2004/picx-images-hosting/master/frames_2025_03_26.67xmq3djvx.webp)

- Path Planning：

    The NAV2 default Global Planner is used as the global path planner, with the [pb_omni_pid_pursuit_controller](https://github.com/SMBU-PolarBear-Robotics-Team/pb_omni_pid_pursuit_controller) as the path follower.

- Namespace:

    The current RM27 setup is maintained as a single-robot system and keeps the namespace empty by default. If multi-robot support is added later, Gazebo plugins, Point-LIO, Nav2, TF, and topic remaps must be updated together.

- LiDAR:

    The Livox Mid360 is mounted 0.18 m to the left and 0.28 m above `base_link`. The physical `roll=-45 deg` mounting is compensated in the Livox driver, while TF exposes the compensated logical sensor frame as `left_mid360` with zero roll.

    Note: In the simulation environment, the point cloud pattern is actually of the Velodyne-style mechanical scan. Additionally, the simulator's output point cloud lacks some fields, preventing point_lio from estimating the state correctly. Thus, the simulator’s output point cloud is processed by [ign_sim_pointcloud_tool](./ign_sim_pointcloud_tool/) to add the `time` field.

- File Structure

    ```txt
    .
    ├── fake_vel_transform                  # Virtual velocity reference frame to handle gimbal scanning mode, see sub-repository README
    ├── ign_sim_pointcloud_tool             # Simulator point cloud processing tool
    ├── livox_ros_driver2                   # Livox driver
    ├── loam_interface                      # Point_lio and other odometry interfaces
    ├── pb_teleop_twist_joy                 # Gamepad control
    ├── pb2025_nav_bringup                  # Launch files
    ├── pb2025_sentry_nav                   # This repository's package description
    ├── pb_omni_pid_pursuit_controller      # Path tracking controller
    ├── point_lio                           # Odometry
    ├── pointcloud_to_laserscan             # Convert terrain_map to LaserScan type to represent obstacles (only launched in SLAM mode)
    ├── sensor_scan_generation              # Point cloud related coordinate transformation
    ├── relocalization_manager              # Relocalization manager; uses small_gicp for verification and publishes map -> odom
    ├── terrain_analysis                    # Terrain analysis within a 4m range of the vehicle, writing obstacle height above ground into the PointCloud intensity field.
    └── terrain_analysis_ext                # Terrain analysis beyond a 4m range of the vehicle, writing obstacle height above ground into the PointCloud intensity field.
    ```

## 2. Quick Start

### 2.1 Option 1: Docker

#### 2.1.1 Setup Environment

- [Docker](https://docs.docker.com/engine/install/)

- Allow Docker Container to access the host's X11 display

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
- Simulation package（Option）：[rmu_gazebo_simulator](https://github.com/SMBU-PolarBear-Robotics-Team/rmu_gazebo_simulator)
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

Download prior point cloud:

Prior point clouds are used for point_lio and small_gicp. Due to large file size, they are not stored in Git. Please download them from [FlowUs](https://flowus.cn/lihanchen/share/87f81771-fc0c-4e09-a768-db01f4c136f4?code=4PP1RS).

> Note: The performance of point_lio with prior_pcd in large scenes is not optimal, and it is more prone to drift than without prior point clouds. Debugging and optimization are ongoing.

#### 2.2.3 Build

```bash
rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
```

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

> [!NOTE]
> We highly recommend building your workspace using the symlink-install option since pb2025_sentry_nav extensively utilizes launch_file and YAML resources. This option installs symbolic links to those non-compiled source files meaning that you don't need to rebuild again and again when you're for example tweaking a parameter file. Instead, your changes take effect immediately and you just need to restart your application.

### 2.3 Running

You can start the project with the following commands. Use the `Nav2 Goal` plugin in RViz to publish goal pose.

#### 2.3.1 Simulation

Navigation mode：

```bash
ros2 launch rm_27_stimulation sim_with_nav.launch.py \
world:=RMUC2026 \
slam:=False \
gui:=true \
use_rviz:=true
```

SLAM mode：

```bash
ros2 launch pb2025_nav_bringup rm_navigation_simulation_launch.py \
slam:=True
```

Save map：`ros2 run nav2_map_server map_saver_cli -f <YOUR_MAP_NAME>`

#### 2.3.2 Physical Robot

SLAM mode：

```bash
ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py \
slam:=True \
use_robot_state_pub:=False
```

Save map：`ros2 run nav2_map_server map_saver_cli -f <YOUR_MAP_NAME>`

Navigation mode:

Remember to change the `world` parameter to the actual map name.

```bash
ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py \
world:=<YOUR_WORLD_NAME> \
slam:=False \
use_robot_state_pub:=False
```

### 2.4 Launch Arguments

Launch arguments are largely common to both simulation and physical robot. However, there is a group of arguments that apply only to hardware or only to the simulator. Below is a legend to the tables with all launch arguments.

| Symbol | Meaning                      |
| ------ | ---------------------------- |
| 🤖      | Available for physical robot |
| 🖥️      | Available in simulation      |

| Available | Argument | Description | Type  | Default |
|-|-|-|-|-|
| 🤖 🖥️ | `namespace` | Top-level namespace. Keep empty for the current single-robot setup. | string | Simulation: ""; Reality: "" |
| 🤖🖥️ | `use_sim_time` | Use simulation (Gazebo) clock if True | bool | Simulation: True; Reality: False |
| 🤖 🖥️ | `slam` | Whether run a SLAM. If True, it will disable small_gicp and send static tf (map->odom). Then automatically save the pcd_file in [./point_lio/PCD/](./point_lio/PCD/)| bool | False |
| 🤖 🖥️ | `world` | In simulation, `RMUC2026` is the currently verified world. Older worlds require matching mesh/map/PCD checks before use. | string | Simulation: "RMUC2026" |
|  |  | In reality, the `world` parameter name is the same as the file names of the grid map and prior pointcloud map | string | "" |
| 🤖 🖥️ | `map` | Full path to map file to load. The path is constructed based on the `world` parameter | string | Simulation: [RMUC2026.yaml](./src/pb2025_nav_bringup/map/simulation/RMUC2026.yaml); Reality: AUTO_FILL |
| 🤖 🖥️ | `prior_pcd_file` | Full path to prior pcd file to load. The path is constructed based on the `world` parameter | string | Simulation: [RMUC2026.pcd](./src/pb2025_nav_bringup/pcd/simulation/RMUC2026.pcd); Reality: AUTO_FILL |
| 🤖 🖥️ | `params_file` | Full path to the ROS2 parameters file to use for all launched nodes | string | Simulation: [nav2_params.yaml](./src/pb2025_nav_bringup/config/simulation/nav2_params.yaml); Reality: [nav2_params.yaml](./src/pb2025_nav_bringup/config/reality/nav2_params.yaml) |
| 🤖🖥️ | `rviz_config_file` | Full path to the RViz config file to use | string | [nav2_default_view.rviz](./src/pb2025_nav_bringup/rviz/nav2_default_view.rviz) |
| 🤖 🖥️ | `autostart` | Automatically startup the nav2 stack | bool | True |
| 🤖 🖥️ | `use_composition` | Whether to use composed bringup | bool | False |
| 🤖 🖥️ | `use_respawn` | Whether to respawn if a node crashes. Applied when composition is disabled. | bool | False |
| 🤖🖥️ | `use_rviz` | Whether to start RViz | bool | True |
| 🤖 | `use_robot_state_pub` | Whether to start `robot_state_publisher`. The `rm_27_stimulation` simulator starts its own robot description in `spawn_robot.launch.py`; on the physical robot, TF should be provided by the complete robot system or an RM27-specific description package. Keep this disabled until the physical robot description is explicitly wired to RM27 frames. | bool | False |

> [!TIP]
> For more details about this project and the deployment guide for the physical robot, please visit the [Wiki](https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_sentry_nav/wiki).

### 2.5 Joy teleop

By default, PS4 controller support is enabled. The key mapping can be found in the `teleop_twist_joy_node` section of [nav2_params.yaml](./pb2025_nav_bringup/config/simulation/nav2_params.yaml).

![teleop_twist_joy.gif](https://raw.githubusercontent.com/LihanChen2004/picx-images-hosting/master/teleop_twist_joy.5j4aav3v3p.gif)
