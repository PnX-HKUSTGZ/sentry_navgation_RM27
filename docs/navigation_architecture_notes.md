# sentry-navigation-RM27 导航架构说明

这份文档说明当前导航栈里的主要 topic 数据流，以及 TF 变换分别由哪个节点或功能包负责。

相关图文件：

- `docs/navigation_topics.png`
- `docs/navigation_tf.png`
- `docs/navigation_topics.dot`
- `docs/navigation_tf.dot`

## 1. 整体主线

整个导航工程可以先按下面这条流水线理解：

```text
传感器 / 仿真点云
  -> Point-LIO
  -> loam_interface
  -> relocalization_manager
  -> terrain_analysis / terrain_analysis_ext
  -> Nav2 规划与控制
  -> fake_vel_transform
  -> cmd_vel 输出到底盘
```

这套栈里最重要的三个 topic 是：

```text
/cloud_registered      Point-LIO 输出的已配准点云
/registered_scan       loam_interface 转换后的 odom 系点云
/odometry              sensor_scan_generation 发布、Nav2 使用的里程计
```

## 2. 传感器输入

### 仿真模式

仿真里，`rm_27_stimulation` 会发布模拟 LiDAR 和 IMU topic：

```text
/livox/lidar
/livox/lidar/custom
/livox/imu
```

但是在 `config/simulation/nav2_params.yaml` 里，Point-LIO 配置成消费：

```yaml
point_lio:
  common:
    lid_topic: "velodyne_points"
    imu_topic: "livox/imu"
```

所以仿真模式中间多了一个转换节点：

```text
rm_27_stimulation
  -> /livox/lidar
  -> ign_sim_pointcloud_tool
  -> /velodyne_points
  -> point_lio
```

`ign_sim_pointcloud_tool` 的作用是把仿真点云转换成 Point-LIO 在当前仿真配置下需要的 Velodyne 风格点云，补出类似 `ring`、`time` 这样的字段。

### 实车模式

实车模式里，`livox_ros_driver2` 发布：

```text
/livox/lidar
/livox/lidar/pointcloud   # 只有驱动格式启用 PointCloud2 输出时才会有
/livox/imu
```

在 `config/reality/nav2_params.yaml` 里，Point-LIO 配置成消费：

```yaml
point_lio:
  common:
    lid_topic: "livox/lidar"
    imu_topic: "livox/imu"
```

所以实车模式更直接：

```text
livox_ros_driver2
  -> /livox/lidar
  -> /livox/imu
  -> point_lio
```

简单总结：

```text
仿真：
  /livox/lidar -> ign_sim_pointcloud_tool -> /velodyne_points -> Point-LIO

实车：
  /livox/lidar -> Point-LIO
```

## 3. Point-LIO 输出

`point_lio` 负责 LiDAR-inertial odometry。它消费点云和 IMU，然后发布自己的里程计和配准后的点云。

主要输出：

```text
/aft_mapped_to_init      nav_msgs/Odometry
/cloud_registered        sensor_msgs/PointCloud2
/cloud_registered_body   sensor_msgs/PointCloud2，可选
/cloud_effected          sensor_msgs/PointCloud2
/Laser_map               sensor_msgs/PointCloud2
/path                    nav_msgs/Path，可选
```

最重要的是：

```text
/cloud_registered
/aft_mapped_to_init
```

`/cloud_registered` 是 Point-LIO 在自己世界坐标系下输出的已配准点云。当前代码里它的 frame 是：

```text
camera_init
```

`/aft_mapped_to_init` 是 Point-LIO 的里程计 topic。当前代码里：

```text
header.frame_id = camera_init
child_frame_id  = body
```

Point-LIO 也可以发布这个 TF：

```text
camera_init -> aft_mapped
```

但是当前参数里是：

```yaml
point_lio:
  publish:
    tf_send_en: False
```

所以默认情况下，Point-LIO 不广播主 TF 链，它主要发布 topic。

## 4. loam_interface

`loam_interface` 是一个非常关键的坐标整理节点。

它订阅：

```text
/cloud_registered
/aft_mapped_to_init
```

它的参数：

```yaml
loam_interface:
  state_estimation_topic: "aft_mapped_to_init"
  registered_scan_topic: "cloud_registered"
  odom_frame: "odom"
  base_frame: "base_footprint"
  lidar_frame: "left_mid360"
```

它发布：

```text
/registered_scan
/lidar_odometry
```

它的职责是把 Point-LIO 输出的已配准点云，从 Point-LIO 自己的坐标约定转换到导航栈统一使用的 `odom` 坐标约定。

所以：

```text
/cloud_registered
```

表示：

```text
Point-LIO 原生输出的已配准点云
```

而：

```text
/registered_scan
```

表示：

```text
给 Nav2、terrain_analysis、relocalization_manager 使用的 odom 系点云
```

`/lidar_odometry` 是 `loam_interface` 整理后发布的 LiDAR 里程计 topic。概念上它表示：

```text
odom -> left_mid360
```

但它是 `nav_msgs/Odometry` topic，不是 TF 广播。

## 5. relocalization_manager

`relocalization_manager` 是当前的全局定位节点。

它订阅：

```text
/registered_scan
/initialpose
```

它还从 launch 里加载：

```text
prior_pcd_file
```

这就是先验 PCD 地图。

关键参数：

```yaml
relocalization_manager:
  map_frame: "map"
  odom_frame: "odom"
  base_frame: "base_footprint"
  robot_base_frame: "gimbal_yaw"
  lidar_frame: "left_mid360"
```

它的工作流程是：

```text
累计 /registered_scan
  -> 使用 small_gicp 和 prior_pcd_file 做配准
  -> 估计 map -> odom
  -> 广播 TF map -> odom
```

所以责任关系是：

```text
正常 localization 模式下，relocalization_manager 负责 map -> odom
```

它还订阅 `/initialpose`。这个 topic 通常由 RViz 的 “2D Pose Estimate” 工具发布。收到初始位姿后，节点会尝试把这个位姿、当前点云 frame 和 TF 组合起来，重置 `map -> odom` 的初值。

## 6. sensor_scan_generation

`sensor_scan_generation` 订阅：

```text
/registered_scan
/lidar_odometry
```

它发布：

```text
/sensor_scan
/odometry
```

它还广播：

```text
odom -> base_footprint
```

它的参数：

```yaml
sensor_scan_generation:
  lidar_frame: "left_mid360"
  base_frame: "base_footprint"
  robot_base_frame: "gimbal_yaw"
```

它的职责是使用 `/lidar_odometry` 加上机器人静态 TF，估计底盘在 `odom` 下的位姿。

它会发布 `odom -> base_footprint` TF，同时发布 `/odometry`。`/odometry` 是 Nav2 实际使用的里程计 topic。

多个 Nav2 节点都使用：

```yaml
odom_topic: odometry
```

例如：

```yaml
bt_navigator:
  odom_topic: odometry

controller_server:
  odom_topic: odometry

velocity_smoother:
  odom_topic: "odometry"
```

所以 Nav2 并不直接消费 Point-LIO 的 `/aft_mapped_to_init`，它消费的是：

```text
sensor_scan_generation -> /odometry
```

## 7. terrain_analysis 和 terrain_analysis_ext

这两个节点负责把点云转换成 Nav2 costmap 使用的地形点云。

`terrain_analysis` 订阅：

```text
/registered_scan
/lidar_odometry
/joy
/map_clearing
```

它发布：

```text
/terrain_map
```

`terrain_analysis_ext` 订阅：

```text
/registered_scan
/lidar_odometry
/joy
/cloud_clearing
/terrain_map
```

它发布：

```text
/terrain_map_ext
```

这两个地形点云输出都使用：

```text
frame_id = odom
```

概念上可以这样理解：

```text
/terrain_map      给 local_costmap 使用的局部地形点云
/terrain_map_ext  给 global_costmap 使用的扩展地形点云
```

Nav2 costmap 参数里是这样接的：

```yaml
local_costmap:
  intensity_voxel_layer:
    observation_sources: terrain_map
    terrain_map:
      topic: <robot_namespace>/terrain_map

global_costmap:
  intensity_voxel_layer:
    observation_sources: terrain_map_ext
    terrain_map_ext:
      topic: <robot_namespace>/terrain_map_ext
```

也就是：

```text
/terrain_map     -> local_costmap
/terrain_map_ext -> global_costmap
```

## 8. map_server

`map_server` 从 launch 传入的 yaml/pgm 加载 2D 地图：

```text
map yaml/pgm
```

它发布：

```text
/map
/map_metadata
```

正常 localization 模式下：

```text
map_server 发布静态 2D 地图
relocalization_manager 发布 map -> odom
```

这两者合起来，让 Nav2 能知道机器人在 `map` 坐标系里的位置。

## 9. Nav2 规划与控制链路

Nav2 的主链路是：

```text
RViz / 上层客户端
  -> bt_navigator
  -> planner_server
  -> smoother_server
  -> controller_server
  -> velocity_smoother
  -> fake_vel_transform
  -> /cmd_vel
```

`bt_navigator` 接收导航 action，例如：

```text
NavigateToPose
NavigateThroughPoses
```

这些是 action，不是普通 topic。

`planner_server` 使用：

```text
/map
global_costmap
TF: map -> odom -> base
```

来生成全局路径。

`controller_server` 使用：

```text
/odometry
local_costmap
global path
TF
```

并通过你的自定义控制器插件：

```text
pb_omni_pid_pursuit_controller::OmniPidPursuitController
```

控制器插件发布：

```text
/local_plan
/lookahead_point
/curvature_points_marker_array
/cmd_vel_controller
```

在 `navigation_launch.py` 里，`controller_server` 有这个 remap：

```python
remappings=[("cmd_vel", "cmd_vel_controller")]
```

所以 controller 原本要发的 `cmd_vel` 被重映射成：

```text
/cmd_vel_controller
```

然后 `velocity_smoother` 订阅：

```text
/cmd_vel_controller
```

并发布：

```text
/cmd_vel_nav2_result
```

launch 里的 remap 是：

```python
("cmd_vel", "cmd_vel_controller")
("cmd_vel_smoothed", "cmd_vel_nav2_result")
```

所以速度指令链路是：

```text
controller_server
  -> /cmd_vel_controller
  -> velocity_smoother
  -> /cmd_vel_nav2_result
```

## 10. fake_vel_transform

`fake_vel_transform` 是最后一个速度转换节点。

它订阅：

```text
/cmd_vel_nav2_result
/cmd_spin
/odometry
/local_plan
```

它发布：

```text
/cmd_vel
```

它还广播：

```text
gimbal_yaw -> gimbal_yaw_fake
```

参数：

```yaml
fake_vel_transform:
  odom_topic: "odometry"
  robot_base_frame: "gimbal_yaw"
  fake_robot_base_frame: "gimbal_yaw_fake"
  input_cmd_vel_topic: "cmd_vel_nav2_result"
  output_cmd_vel_topic: "cmd_vel"
  cmd_spin_topic: "cmd_spin"
```

它的职责是处理云台/控制 frame 和底盘真实运动方向之间的差异。

Nav2 使用下面这个 frame 做规划和控制：

```text
gimbal_yaw_fake
```

但最终发给底盘的速度指令是：

```text
/cmd_vel
```

## 11. 为什么 Nav2 使用 gimbal_yaw_fake

Nav2 参数里把机器人 base frame 配成：

```yaml
bt_navigator:
  robot_base_frame: gimbal_yaw_fake

local_costmap:
  robot_base_frame: gimbal_yaw_fake

global_costmap:
  robot_base_frame: gimbal_yaw_fake

behavior_server:
  robot_base_frame: gimbal_yaw_fake
```

所以 Nav2 认为机器人本体 frame 是：

```text
gimbal_yaw_fake
```

而不是 `base_footprint` 或 `gimbal_yaw`。

这个 TF 由 `fake_vel_transform` 发布：

```text
gimbal_yaw -> gimbal_yaw_fake
```

这种设计是为了把 Nav2 的控制 frame 和真实云台/底盘方向关系解耦。

## 12. 正常 TF 链

正常 localization 模式下，主 TF 链应该是：

```text
map
  -> odom
  -> base_footprint
  -> gimbal_yaw
  -> gimbal_yaw_fake
```

还需要 LiDAR 安装外参：

```text
base_footprint / gimbal_yaw
  -> left_mid360
```

各段责任如下：

```text
map -> odom
  relocalization_manager

odom -> base_footprint
  sensor_scan_generation

base_footprint -> gimbal_yaw
  robot_state_publisher 或外部机器人启动模块

gimbal_yaw -> gimbal_yaw_fake
  fake_vel_transform

base_footprint / gimbal_yaw -> left_mid360
  robot_state_publisher 或外部机器人启动模块
```

一个重要细节：`loam_interface` 不广播 TF。它只是查 TF：

```text
base_frame -> lidar_frame
```

通常就是：

```text
base_footprint -> left_mid360
```

然后用这个静态/外部变换去转换 `/cloud_registered` 和 `/aft_mapped_to_init`。

`relocalization_manager` 在处理 `/initialpose` 时也会查 TF：

```text
robot_base_frame -> current_scan_frame
```

通常相关的是：

```text
gimbal_yaw -> registered_scan frame
```

## 13. SLAM 模式下的 TF 差异

当 launch 参数 `slam=True` 时，`bringup_launch.py` 不走 `localization_launch.py`，而是走 `slam_launch.py`。

`slam_launch.py` 启动：

```text
point_lio
pointcloud_to_laserscan
slam_toolbox
map_saver
static_transform_publisher_map2odom
```

它会发布一个静态 identity TF：

```text
map -> odom
```

当前 `slam_toolbox` 参数里：

```yaml
transform_publish_period: 0.0
```

注释说明 `0` 表示不发布 odometry transform。

所以在当前配置下，SLAM 模式里的 `map -> odom` 不是 `slam_toolbox` 发布的，而是由下面这个节点发布：

```text
tf2_ros static_transform_publisher
```

发布的是 identity 变换。

## 14. 常见混淆点

### `/cloud_registered` 和 `/registered_scan`

它们不是同一个东西。

```text
/cloud_registered
  Point-LIO 原生输出的已配准点云。

/registered_scan
  loam_interface 输出，已经转换到导航栈的 odom 坐标约定。
```

### `/aft_mapped_to_init` 和 `/odometry`

它们也不是同一个东西。

```text
/aft_mapped_to_init
  Point-LIO 输出。

/odometry
  sensor_scan_generation 输出，供 Nav2 使用。
```

### Point-LIO 默认不负责主 TF 链

当前参数是：

```yaml
tf_send_en: False
```

所以正常 TF 链不是 Point-LIO 提供的，而是由这些部分共同提供：

```text
relocalization_manager
sensor_scan_generation
fake_vel_transform
robot_state_publisher / 外部机器人启动模块
```

### behavior_server 的 cmd_vel remap 差异

在非 composed launch 路径里，`behavior_server` 没有像 `controller_server` 那样显式 remap `cmd_vel`。

在 composed launch 路径里，它有：

```python
("cmd_vel", "cmd_vel_nav2_result")
```

所以如果 recovery 行为看起来直接往 `/cmd_vel` 发速度，要重点检查这个启动路径差异。

## 15. 最简 topic 链

仿真：

```text
/livox/lidar
  -> ign_sim_pointcloud_tool
  -> /velodyne_points
  -> point_lio
```

实车：

```text
/livox/lidar
  -> point_lio
```

共同的后续链路：

```text
point_lio
  -> /cloud_registered + /aft_mapped_to_init
  -> loam_interface
  -> /registered_scan + /lidar_odometry
  -> relocalization_manager 发布 map->odom
  -> sensor_scan_generation 发布 /odometry 和 odom->base_footprint
  -> terrain_analysis 发布 /terrain_map
  -> terrain_analysis_ext 发布 /terrain_map_ext
  -> Nav2 costmap / planner / controller
  -> /cmd_vel_controller
  -> velocity_smoother
  -> /cmd_vel_nav2_result
  -> fake_vel_transform
  -> /cmd_vel
```

## 16. 最简 TF 链

```text
map
  -- relocalization_manager -->
odom
  -- sensor_scan_generation -->
base_footprint
  -- robot_state_publisher / 外部机器人启动 -->
gimbal_yaw
  -- fake_vel_transform -->
gimbal_yaw_fake
```

LiDAR 安装外参：

```text
base_footprint / gimbal_yaw
  -- robot_state_publisher / 外部机器人启动 -->
left_mid360
```
