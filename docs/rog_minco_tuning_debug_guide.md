# ROG-map + MINCO + MPC 调试、调参与发布指南

本文面向 `sentry-navigation-RM27` 当前代码，给出从仿真到实车的可执行流程。命令默认在 ROS 2 Jazzy、仓库根目录 `/home/pnx/nav_ws/sentry-navigation-RM27` 下执行。

端到端所有权、数据结构和源码索引见配套文档
[`rog_minco_navigation_architecture.md`](rog_minco_navigation_architecture.md)；本文重点是实际操作、
验收和故障定位。

本文有六个必须先明确的边界：

1. 当前新链路不是全状态三维轨迹规划。ROG-map 维护三维概率占据，随后按 XY 列提取地面和净空，MINCO 优化的是平面 `x/y/yaw` 轨迹，MPC 跟踪的是 SE(2) 参考。因此它是“3D 感知 + 2.5D 可通行投影 + 平面轨迹优化”，不会显式规划车身俯仰、侧倾或完整三维扫掠体积。
2. 系统采用 fail-closed：地图、TF、时间戳、净空、轨迹或 MPC 输入不可确认时，应该拒绝轨迹并输出零速。调试目标是找到拒绝发生在哪一层，不是绕开拒绝。
3. 仿真和实车 MINCO profile 都启用
   `projection.prior_map.require_ground_support: true`。候选地面必须与已测绘的地面高程相符；
   无地面回波的盲区列也必须同时具备二维地图 known-free、地面高程和已观测车身净空。
   `RMUC2026.yaml` 当前带 5 cm 全图高程栅格，实车 `highbay.yaml` 带 map-frame `z=0` 平地支撑，
   `tunnel_ramp_test.yaml` 另带坡面/平台 patch。新增实车坡道必须先测绘并写入 grid 或 patch，不能
   通过放宽 unknown 或沿用平地高度处理。
4. 仿真 `projection.min_headroom_known_ratio: 0.25` 用于空列，
   `projection.min_observed_overhead_headroom_known_ratio: 0.0` 只用于已测到且最低边界高于车体的顶板列；
   实车两项都保持 `0.80`。车身体积内的 occupied 和静态先验 occupied 始终硬否决。
   **禁止把两个仿真补偿值直接下放实车。**
5. active `minco` 当前只支持 `NavigateToPose`。`NavigateThroughPoses` 未加载；运动型
   behavior action（spin/backup/drive-on-heading/assisted-teleop）也未加载，避免绕过
   ROG + planning-token + MincoMpc 安全链。
6. 本文把“单次开发联调结果”和“发布验收”分开记录。单次到达目标也不代表 P2/P3/P6 已通过；
   正反向重复、浮空障碍、取消、陈旧数据以及实车标定仍必须按第 10、14 节完成。

## 1. 当前链路和三种模式

### 1.1 旧链路

`legacy` 模式保留原工程的权威导航链：

```text
点云
  -> terrain_analysis
  -> /terrain_map
  -> terrain_analysis_ext
  -> /terrain_map_ext
  -> Nav2 global/local costmap
  -> IntensityVoxelLayer
  -> GridBased(ThetaStar)
  -> FollowPath
  -> /cmd_vel_controller
  -> velocity_smoother
  -> /cmd_vel_nav2_result
  -> fake_vel_transform
  -> /cmd_vel
```

它仍是正式回退通道，不会因为接入 ROG-map 而被删除。

### 1.2 新链路

`minco` 模式的主链是：

```text
三维点云 + 里程计 + TF + 静态 OccupancyGrid
  -> ROG-map 三维概率占据
  -> 测绘地面高程匹配、头顶净空、浮空障碍判断
  -> 2.5D fused layer + signed distance field
  -> MincoPlanner: 搜索种子、走廊、x/y/yaw 连续轨迹优化
  -> 离散 + 连续动力学极值检查、必要时 retiming
  -> 轨迹安全复核
  -> /minco/opt_path (NORMAL 或 BLOCK)
  -> MincoMpc
     |-> /minco/cmd_vel_mpc（旁路 raw debug，不是控制输入）
     `-> 返回 TwistStamped 给 controller_server
         -> /cmd_vel_controller
         -> velocity_smoother
         -> /cmd_vel_nav2_result
         -> fake_vel_transform
         -> /cmd_vel
```

active `minco` 的主 Nav2 server 只注册 `MincoPlanner` 与 `MincoMpc`，外部 action 也不能通过指定
旧 plugin ID 绕过 ROG 安全链。`legacy` 和 `minco_shadow` 的主栈仍注册原有插件，回退必须停止
当前 bringup 后以 `navigation_mode:=legacy` 明确重启；运行中不会悄悄切换。

active 行为树的规划请求 RateController 是 `3 Hz`。参数中的 `expected_planner_frequency: 20.0`
用于 Nav2 期望/告警，`MincoPlanner.minco_optimizer.opt_freq: 20.0` 当前也不是独立调度器；实际
`createPlan()` 调用不能仅凭这两个 `20.0` 推断。测量实际输出必须以 action、日志和 topic 为准。

active profile 只加载 `navigate_to_pose` navigator 和 `wait` behavior。这不是功能精简，而是
安全边界：其他 motion behavior 的速度由 behavior server 直接进入 smoother，不经
MincoMpc token gate，也无法使用 ROG 的浮空障碍结果。

### 1.3 `shadow` 的准确含义

`minco_shadow` 是旁路对照，不是控制器混合：

- 主 `/planner_server`、`/controller_server`、`/navigate_to_pose` 仍走旧链并负责车辆运动。
- 另起 `/minco_shadow/planner_server`，只运行 MincoPlanner，不启动 shadow controller，也不接管底盘。
- sidecar 使用独立的 8 线程 `minco_planner/planner_server_mt`，避免 lifecycle/ROG 初始化阻塞
  TF、点云和地图更新回调；主链仍是原生 legacy planner server。
- 行为树以约 3 Hz 检查目标；relay 对已接受目标去重，每个目标/session 只创建一次 action。
- `/minco_shadow/opt_path`、`/minco_shadow/backup_path` 只用于观察。

relay 不以 3 Hz 重发 action；goal 被接受后，当前 profile 的 sidecar 内部 FSM 会在同一 session
内约 2 Hz 成功重规划，失败生成最多 4 Hz，并持续发布轨迹，直到到达目标、目标改变或 lifecycle
停止。拒绝、发送异常、非成功 result、
server 不可用和 lifecycle 变化会使 accepted-cache 失效，重新 active 后同目标可重派。因此应把
shadow 理解为“每目标一次 action、内部连续观测”，而不是一次规划快照，也不能把输出频率误写成
BT 的 3 Hz。

重启检测有两个明确边界：某次 BT tick 若观察到 action readiness 的 false edge，会清缓存，并在
ready 恢复后重派；正常 launch 的 managed sidecar 即使快速重启发生在两个 tick 之间，也会用
`/minco_shadow/planner_server/transition_event` 使 generation/缓存失效，inactive 时延迟派发，
active 后重派。手工替换非 lifecycle server 时，如果 readiness=false 从未被 tick 采样、也没有
transition event，relay 无法仅凭“仍然 ready”识别换进程。

所以，在 `minco_shadow` 下发送普通 `/navigate_to_pose`，车仍会被旧链驱动。要做真正静态的 shadow 检查，必须先从物理或仿真接口禁用驱动，再直接调用 `/minco_shadow/compute_path_to_pose`。

shadow 的 `NavigateThroughPoses` 仍由 legacy 主链执行，但不向 MINCO sidecar 派发整组
航点；当前单目标异步 planner 会丢失中间点，因此这种结果不能当作有效 A/B 对照。

### 1.4 模式切换规则

允许值只有：

```text
legacy
minco_shadow
minco
```

`navigation_mode` 在 launch 展开时决定参数文件、行为树和节点关系，不能通过 `ros2 param set` 在线切换。切换模式必须停止整套导航再重启。

当前要求：

- 不传 `namespace` launch 参数，使用已核验的空默认值；新链尚不支持非空 namespace。Jazzy CLI
  会把命令行中的空赋值写法折叠成 malformed launch argument，因此不要显式传空字符串。
- `slam:=true` 只支持 `legacy`。
- MINCO 模式必须有可加载的静态 OccupancyGrid YAML。
- `enable_legacy_terrain:=auto` 时，`legacy/minco_shadow` 自动为 true，`minco` 自动为 false。

主要参数和入口文件：

| 文件 | 用途 |
|---|---|
| `src/pb2025_nav_bringup/config/simulation/minco_params.yaml` | 仿真 active MINCO/ROG/MPC 基线，也是仿真 shadow sidecar 的主体参数 |
| `src/pb2025_nav_bringup/config/reality/minco_params.yaml` | 实车 active 与 shadow sidecar 的主体参数 |
| `src/pb2025_nav_bringup/config/*/minco_shadow_params.yaml` | shadow 主 Nav2 的旧链/BT overlay |
| `src/pb2025_nav_bringup/config/*/minco_shadow_sidecar_params.yaml` | shadow planner 的 topic 与性能标签 overlay |
| `src/pb2025_nav_bringup/config/*/nav2_params.yaml` | 原有 Nav2、costmap、旧 planner/controller 参数 |
| `src/pb2025_nav_bringup/launch/navigation_launch.py` | 三种模式装配和 shadow sidecar |
| `src/rm_27_stimulation/launch/sim_with_nav.launch.py` | Gazebo 与导航总入口 |
| `src/rm_27_stimulation/config/worlds.yaml` | 仿真 world 到导航 map 的映射 |
| `src/rm_27_stimulation/config/spawn_poses.yaml` | 仿真起始位姿 |

只修改 `src/` 下的源参数；随后重新 `colcon build` 并 `source install/setup.bash`。不要直接改
`install/` 中的副本，否则下一次构建会覆盖且无法可靠复现。

## 2. 构建与单元测试

### 2.1 首次构建

```bash
cd /home/pnx/nav_ws/sentry-navigation-RM27
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash
```

每个新终端都需要：

```bash
cd /home/pnx/nav_ws/sentry-navigation-RM27
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

### 2.2 只构建新链相关包

```bash
colcon build --symlink-install \
  --packages-up-to \
  ros_interfaces rog_map minco_planner minco_controller fake_vel_transform \
  sensor_scan_generation point_lio pb2025_nav_bringup rm_27_stimulation \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash
```

### 2.3 修改后最低测试集

```bash
colcon test --packages-select \
  ros_interfaces rog_map minco_planner minco_controller fake_vel_transform \
  sensor_scan_generation point_lio pb2025_nav_bringup rm_27_stimulation
colcon test-result --verbose
```

本次集成验收使用独立目录时，对应的完整命令是：

```bash
cd /home/pnx/nav_ws/sentry-navigation-RM27
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon --log-base log_rog_minco_release build --symlink-install \
  --build-base build_rog_minco_release \
  --install-base install_rog_minco_release \
  --packages-up-to \
  ros_interfaces rog_map minco_planner minco_controller fake_vel_transform \
  sensor_scan_generation point_lio pb2025_nav_bringup rm_27_stimulation \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install_rog_minco_release/setup.bash

colcon --log-base log_rog_minco_release_test test \
  --build-base build_rog_minco_release \
  --install-base install_rog_minco_release \
  --packages-select \
  ros_interfaces rog_map minco_planner minco_controller fake_vel_transform \
  sensor_scan_generation point_lio pb2025_nav_bringup rm_27_stimulation \
  --event-handlers console_cohesion+
colcon test-result --test-result-base build_rog_minco_release --verbose
```

默认 `build/` 和独立 `build_rog_minco_release/` 不可交叉使用；直接运行二进制时选择与本次构建
相同的目录。以下先以默认 `build/` 为例，名称均来自当前 CMake target：

```bash
./build/rog_map/test_projection_clearance
./build/rog_map/test_latest_value_mailbox
./build/minco_planner/test_map_query_adapters
./build/minco_planner/test_trajectory_safety_checker
./build/minco_planner/test_minco_optimizer_dynamics
./build/minco_planner/test_planning_session_state
./build/minco_planner/test_planning_request_contract
./build/minco_planner/test_planning_request_lease
./build/minco_planner/test_smac_global_search_policy
./build/minco_controller/test_input_validation
./build/minco_controller/test_mpc_solver
./build/minco_controller/test_trajectory_session_gate
./build/minco_controller/test_reference_progress
./build/fake_vel_transform/test_fake_vel_transform
./build/sensor_scan_generation/test_twist_estimator
./build/sensor_scan_generation/test_transform_sample_gate
./build/point_lio/test_adaptive_lio
./build/pb2025_nav_bringup/test_minco_shadow_goal_relay
./build/rm_27_stimulation/test_ground_truth_twist_transform
./build/rm_27_stimulation/test_organized_lidar_miss_rays
./build/rm_27_stimulation/test_planar_velocity_pi
python3 -m pytest -q src/pb2025_nav_bringup/test/test_navigation_launch.py
```

任何测试失败都不应进入 Gazebo 自动运动，更不能进入实车。

## 3. 启动仿真和实车

### 3.1 洞口加短坡仿真

**本任务的洞口、no-return 和短坡验收必须在命令行显式传
`world:=tunnel_ramp_test`。** `sim_with_nav.launch.py` 的默认 world 不是该测试场景；省略这个参数时，
即使 action 成功，也不是本任务的证据。总入口的参数名是 `world`，不要写成下层 launch 才使用的
`sim_world`。启动后可用下面两条命令核对实际世界与配套导航地图：

```bash
ros2 launch rm_27_stimulation sim_with_nav.launch.py --show-args | \
  rg 'world|nav_world|use_ground_truth_odom'
gz service -s /gazebo/worlds --reqtype gz.msgs.Empty \
  --reptype gz.msgs.StringMsg_V --timeout 2000 --req ''
```

第二条的响应必须包含 `tunnel_ramp_test`。若同一 `GZ_PARTITION` 中出现多个 world，先停止旧 Gazebo，
不要继续发目标。

先用 ground-truth odom 排除 Point-LIO 初始化和漂移：

```bash
ros2 launch rm_27_stimulation sim_with_nav.launch.py \
  world:=tunnel_ramp_test \
  nav_world:=auto \
  use_ground_truth_odom:=true \
  navigation_mode:=minco_shadow \
  enable_legacy_terrain:=auto \
  gui:=true \
  use_rviz:=true
```

完成 shadow 验收后，停止并改为：

```bash
ros2 launch rm_27_stimulation sim_with_nav.launch.py \
  world:=tunnel_ramp_test \
  nav_world:=auto \
  use_ground_truth_odom:=true \
  navigation_mode:=minco \
  enable_legacy_terrain:=auto \
  gui:=true \
  use_rviz:=true
```

为了复现 CI/联调问题，建议另做一次无 GUI、隔离 DDS 与 Gazebo transport 的运行，避免旧进程和
同机其他仿真串话。启动终端：

```bash
cd /home/pnx/nav_ws/sentry-navigation-RM27
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=153
export GZ_PARTITION=rm27_tunnel_153
ros2 launch rm_27_stimulation sim_with_nav.launch.py \
  world:=tunnel_ramp_test \
  nav_world:=auto \
  use_ground_truth_odom:=true \
  navigation_mode:=minco \
  enable_legacy_terrain:=auto \
  gui:=false \
  use_rviz:=false \
  2>&1 | tee /tmp/rm27_tunnel_active.log
```

动作终端必须设置相同的 domain 和 partition（Gazebo create/remove 也一样）：

```bash
cd /home/pnx/nav_ws/sentry-navigation-RM27
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=153
export GZ_PARTITION=rm27_tunnel_153
ros2 action send_goal \
  /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 3.5, y: 0.0}, orientation: {w: 1.0}}}}" \
  > /tmp/rm27_tunnel_goal.log
```

另开探针终端检查最终姿态和首失败：

```bash
export ROS_DOMAIN_ID=153
ros2 topic echo /odometry --once --qos-profile sensor_data
rg -n 'Goal|result|Trajectory safety rejected|ground-elevation|fail-closed|Failed to make progress' \
  /tmp/rm27_tunnel_goal.log /tmp/rm27_tunnel_active.log
```

运行结束用启动终端 `Ctrl-C`，并确认没有同 domain 的旧节点后再开始下一轮；不要用上一轮仍在发布的
`/clock`、TF 或点云做下一组参数对照。

`tunnel_ramp_test` 没有配套 PCD，不能用它验证 Point-LIO 路线。要验证 Point-LIO 启动与消息链，改用已有 PCD 的世界：

```bash
ros2 launch rm_27_stimulation sim_with_nav.launch.py \
  world:=RMUC2026 \
  nav_world:=auto \
  use_ground_truth_odom:=false \
  slam:=false \
  navigation_mode:=minco_shadow \
  enable_legacy_terrain:=auto \
  gui:=true \
  use_rviz:=true
```

### 3.2 旧链基线

同一世界、同一起点和目标只改模式：

```bash
ros2 launch rm_27_stimulation sim_with_nav.launch.py \
  world:=tunnel_ramp_test nav_world:=auto \
  use_ground_truth_odom:=true navigation_mode:=legacy \
  enable_legacy_terrain:=auto gui:=true use_rviz:=true
```

不要同时启动两个完整 bringup。录制 legacy、shadow、minco 三次独立运行，才能对比相同输入下的路径、拒绝原因和控制效果。

### 3.3 实车 shadow

实车第一阶段只能使用 shadow：

```bash
ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py \
  world:=highbay \
  slam:=false \
  navigation_mode:=minco_shadow \
  enable_legacy_terrain:=auto \
  use_rviz:=true
```

该 launch 会启动 Livox 驱动。确认现场没有另一个驱动实例，避免 `/cloud_registered_full` 重复发布。

当前 `map/reality/highbay.yaml` 已配置 map-frame `z=0` 平地支撑，因此 shadow MINCO 不会再仅因
缺少 `ground_elevation` 全图 BLOCK。仍应先在 shadow 录制原始点云、里程计、TF、地图和 ROG 结果，
确认实际地面回波与 `z=0` 的误差处于 `ground_support_tolerance` 内，再做静态墙、浮空障碍和地图边界
负例。若现场含坡道，必须按第 6.5 节测绘 patch；不能把坡面 BLOCK 误诊为优化器问题。

仿真的 no-return 重建只属于 `rm27_ground_truth_localizer` 对 Gazebo organized GPU lidar 的适配。
**实车不得在 reality 参数中增加或开启 `reconstruct_no_return_rays`，也不得把实车无回波按仿真的
360 x 320 行列角度合成。** 实车驱动的点排列、ring 和无回波编码必须从实际消息合同验证；未经验证的
合成会把错误方向写成 free 证据。实车 shadow 原始包可直接这样录：

```bash
mkdir -p bags
ros2 bag record --storage mcap \
  --compression-mode file --compression-format zstd \
  -o "bags/highbay_shadow_raw_$(date +%Y%m%d_%H%M%S)" \
  /cloud_registered_full /aft_mapped_to_init /odometry \
  /tf /tf_static /map /rosout
```

结束后先用 `ros2 bag info` 确认每个输入都有消息，再开始调 `ground_elevation`、自滤除或投影参数。

完成 shadow 和本文第 14 节 release 门槛后，才允许停止 shadow、低速重启为：

```bash
ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py \
  world:=highbay slam:=false \
  navigation_mode:=minco enable_legacy_terrain:=auto use_rviz:=true
```

## 4. 启动后五分钟验收

### 4.1 节点与生命周期

```bash
ros2 node list | sort
ros2 lifecycle get /planner_server
ros2 lifecycle get /controller_server
ros2 lifecycle get /bt_navigator
ros2 lifecycle get /velocity_smoother
```

shadow 还应存在：

```bash
ros2 lifecycle get /minco_shadow/planner_server
```

期望 Nav2 管理节点为 `active`。ROG-map 是 MincoPlanner 内部对象，不会出现独立 `/rog_map` 节点；不能以“找不到 rog_map node”判定它没运行。

shadow 重启合同可在仿真且运动输出已隔离时检查。一个终端先观察 lifecycle 事件：

```bash
ros2 topic echo \
  /minco_shadow/planner_server/transition_event \
  lifecycle_msgs/msg/TransitionEvent --field goal_state
```

另一个终端执行受管理停启：

```bash
ros2 lifecycle set /minco_shadow/planner_server deactivate
ros2 lifecycle set /minco_shadow/planner_server activate
```

预期每个 transition 都使 relay 的旧 generation 失效，inactive 期间不派发，active 后同一导航目标
重新建立 shadow session。若测试 action readiness 路径，则必须让 sidecar 端点消失足够久，使至少
一次 BT tick 观察到 `action_server_is_ready()==false`；在两个 tick 之间快速替换非 lifecycle
进程而始终显示 ready，不是 readiness 路径能够证明的测试。

### 4.2 接口清单

```bash
ros2 action list -t | sort
ros2 service list -t | sort
ros2 topic list -t | sort
```

关键 action：

```text
/navigate_to_pose                         nav2_msgs/action/NavigateToPose
/compute_path_to_pose                    nav2_msgs/action/ComputePathToPose
/compute_path_through_poses              nav2_msgs/action/ComputePathThroughPoses
/minco_shadow/compute_path_to_pose       nav2_msgs/action/ComputePathToPose
/minco_shadow/compute_path_through_poses nav2_msgs/action/ComputePathThroughPoses
```

shadow action 只在 `minco_shadow` 中存在。

active `minco` 中应存在 `/navigate_to_pose`，不应存在 `/navigate_through_poses`。
PlannerServer 框架仍创建 `/compute_path_through_poses`，但当前 MINCO 不支持这个直接入口；
不得调用它或把返回的多段占位 Path 当作有效规划。active 中 `/spin`、`/backup`、
`/drive_on_heading`、`/assisted_teleop` 也不应出现，`/wait` 仍存在。

active 启动后直接验证：

```bash
ros2 action list | sort | \
  rg '^/(navigate_to_pose|navigate_through_poses|spin|backup|drive_on_heading|assisted_teleop|wait)$'
```

预期只打印 `/navigate_to_pose` 和 `/wait`。若出现任意 motion behavior 或
`/navigate_through_poses`，说明 active overlay 未生效，禁止发目标。legacy/shadow 会保留
这些旧入口，不适用此预期。

还要确认最终速度变换节点没有订阅旧旋转旁路：

```bash
ros2 param get /fake_vel_transform enable_cmd_spin
ros2 topic info -v /cmd_spin
```

active `minco` 中参数必须为 `False`，`/cmd_spin` 的订阅者列表中不得有
`/fake_vel_transform`。legacy/shadow 默认仍为 `True`，这是刻意保留的旧接口兼容性，不应把
两种模式的预期混用。

Nav2 PlannerServer 发布的 `plan` 是相对名称，因此主栈为 `/plan`，shadow sidecar 为
`/minco_shadow/plan`。MincoPlanner 自己的 `/opt_path_vis` 等绝对可视化名称不遵循这个规则。

关键 topic：

```text
/map
/registered_scan                         仿真 ROG 点云
/lidar_odometry                          仿真 ROG 里程计
/cloud_registered_full                   实车 ROG 点云
/aft_mapped_to_init                      实车 ROG 里程计
/ground_truth/odometry                   Gazebo 原始 3D 真值，仅仿真
/odometry                                MPC 状态
/rog_map/layer_type
/rog_map/layer_value_dynamic
/rog_map/layer_value_static
/rog_map/layer_value
/rog_map/clearance_status
/rog_map/headroom_known_ratio
/rog_map/headroom
/rog_map/layer_height_delta
/rog_map/layer_confidence
/rog_map/field
/rog_map/occupied
/rog_map/raw_occupied
/rog_map/self_filter_box
/minco/opt_path
/minco/backup_path
/plan
/opt_path_vis
/minco_candidate_path_vis
/astar_path_vis
/mpc_predict_path
/mpc_real_path
/minco/cmd_vel_mpc
/cmd_vel_controller
/cmd_vel_nav2_result
/cmd_vel
```

当前没有单独发布 `ground_support_z` 网格；高程来源以实际加载的 map YAML 为准，授权结果编码在
`/rog_map/clearance_status` 的 3/4 中。调试坡面时必须同时保存 map YAML、TF 和 status，不能只录
status 后反推当时使用了哪版高程。

shadow 下只有轨迹 topic 明确改为 `/minco_shadow/opt_path` 和
`/minco_shadow/backup_path`，action 也位于 `/minco_shadow/...`。ROG visualizer 和
MincoPlanner visualizer 当前在源码中使用绝对 `/rog_map/...`、`/opt_path_vis`、
`/minco_candidate_path_vis`、`/astar_path_vis`，不会自动带 shadow 前缀。正常 launch 通过模式互斥
避免 active 与 shadow 混流；不要手工同时启动两套后再用这些绝对可视化 topic 做 A/B 归因。

### 4.3 QoS 正确的探针

ROG 可视化发布器使用 best-effort，并且部分数据只有存在订阅者时才组装。诊断探针必须使用 sensor-data QoS：

```bash
ros2 topic echo /rog_map/layer_type --once --qos-profile sensor_data
ros2 topic echo /rog_map/layer_value --once --qos-profile sensor_data
ros2 topic echo /rog_map/clearance_status --once --qos-profile sensor_data
ros2 topic echo /rog_map/headroom_known_ratio --once --qos-profile sensor_data
ros2 topic info -v /rog_map/layer_type
```

仿真 `/registered_scan` 同样是 best-effort。terrain 节点的订阅端必须显示 `BEST_EFFORT`，并且
legacy/shadow 下两个 terrain 输出必须持续更新：

```bash
ros2 topic info -v /registered_scan
ros2 topic echo /registered_scan --once --qos-profile sensor_data --no-arr
ros2 topic echo /terrain_map --once --no-arr
ros2 topic echo /terrain_map_ext --once --no-arr
ros2 param get /fake_vel_transform terrain_guard_enabled
ros2 param get /fake_vel_transform terrain_timeout
```

仿真 guard 预期为 `true/1.5`，实车为 `true/0.5`；active MINCO 预期为 `false`。日志出现
`Rejecting velocity command because no terrain map` 或 `Stopping because no terrain map` 时，必须先恢复
`registered_scan -> terrain_analysis -> terrain_map`，不能关闭 guard 让静态地图单独放行。

静态 `/map` 是 transient-local/reliable，单独按它的持久化 QoS 探测：

```bash
ros2 topic echo /map --once \
  --qos-durability transient_local --qos-reliability reliable
```

`ros2 topic hz` 没有通用的 QoS profile 参数。先用上述兼容订阅确认能收到，再用：

```bash
ros2 topic hz /rog_map/layer_type
ros2 topic hz /minco/opt_path
ros2 topic hz /odometry
```

若 `hz` 看不到 best-effort topic，但 sensor-data `echo` 能看到，不要误判发布器死亡；使用 RViz 或自写 sensor-data 订阅器统计频率。

### 4.4 参数快照

```bash
mkdir -p /tmp/rog_minco_params
ros2 param dump /planner_server > /tmp/rog_minco_params/planner_server.yaml
ros2 param dump /controller_server > /tmp/rog_minco_params/controller_server.yaml
ros2 param dump /velocity_smoother > /tmp/rog_minco_params/velocity_smoother.yaml
```

shadow 使用：

```bash
ros2 param dump /minco_shadow/planner_server \
  > /tmp/rog_minco_params/minco_shadow_planner_server.yaml
```

搜索实际生效值：

```bash
rg -n "physical_base_frame|ground_support|observed_empty|bridge_observed|headroom|near_field|footprint_clear|filter_mode|box_size|box_padding|keep_time|clear_time|map_timeout|safe_dist|collision_dist|traj_goal_tolerance|max_vel|max_acc|max_yaw|sample_dt|reference_progress_max_lead_time" \
  /tmp/rog_minco_params
```

直接核对几个关键值：

```bash
ros2 param get /planner_server MincoPlanner.rog_map.cloud_filter.z_offset
ros2 param get /planner_server MincoPlanner.frames.physical_base_frame
ros2 param get /planner_server \
  MincoPlanner.rog_map.projection.ground_seed_tolerance
ros2 param get /planner_server \
  MincoPlanner.rog_map.projection.min_headroom_known_ratio
ros2 param get /planner_server \
  MincoPlanner.rog_map.projection.min_observed_overhead_headroom_known_ratio
ros2 param get /planner_server \
  MincoPlanner.rog_map.projection.prior_map.require_ground_support
ros2 param get /planner_server \
  MincoPlanner.rog_map.projection.prior_map.ground_support_tolerance
ros2 param get /planner_server \
  MincoPlanner.rog_map.projection.clearance_check_enable
ros2 param get /planner_server \
  MincoPlanner.rog_map.projection.unknown_as_occupied
ros2 param get /planner_server \
  MincoPlanner.rog_map.projection.observed_empty_as_free
ros2 param get /planner_server \
  MincoPlanner.rog_map.projection.bridge_observed_empty_for_ground_connectivity
ros2 param get /planner_server \
  MincoPlanner.rog_map.projection.clear_robot_footprint_unknown
ros2 param get /planner_server \
  MincoPlanner.rog_map.projection.near_field_prior_fill_enable
ros2 param get /planner_server MincoPlanner.rog_map.decay.keep_time
ros2 param get /planner_server MincoPlanner.rog_map.decay.clear_time
ros2 param get /planner_server MincoPlanner.rog_map.cloud_filter.filter_mode
ros2 param get /planner_server MincoPlanner.rog_map.cloud_filter.box_size.x
ros2 param get /planner_server MincoPlanner.rog_map.cloud_filter.box_size.y
ros2 param get /planner_server MincoPlanner.rog_map.cloud_filter.box_size.z
ros2 param get /planner_server MincoPlanner.rog_map.cloud_filter.box_padding
ros2 param get /planner_server MincoPlanner.safety.map_timeout
ros2 param get /planner_server MincoPlanner.minco_optimizer.safe_dist
ros2 param get /planner_server MincoPlanner.minco_optimizer.collision_dist
ros2 param get /planner_server MincoPlanner.minco_optimizer.max_velocity
ros2 param get /planner_server MincoPlanner.minco_optimizer.max_acceleration
ros2 param get /planner_server MincoPlanner.minco_optimizer.max_yaw_dot
ros2 param get /planner_server MincoPlanner.minco_optimizer.lookahead_dist
ros2 param get /controller_server MincoMpc.dt
ros2 param get /controller_server MincoMpc.lookahead_time
ros2 param get /controller_server MincoMpc.trajectory_timeout
ros2 param get /controller_server MincoMpc.odom_timeout
ros2 param get /controller_server MincoMpc.deadzone_speed_threshold
ros2 param get /controller_server MincoMpc.control_delay_compensation
ros2 param get /controller_server MincoMpc.reference_progress_max_lead_time
ros2 param get /planner_server MincoPlanner.minco_optimizer.traj_goal_tolerance
ros2 param get /controller_server general_goal_checker.xy_goal_tolerance
```

真值仿真还要核对 no-return 重建的实际生效值；这些参数在 localizer，不在 planner：

```bash
ros2 param get /rm27_ground_truth_localizer reconstruct_no_return_rays
ros2 param get /rm27_ground_truth_localizer lidar_horizontal_samples
ros2 param get /rm27_ground_truth_localizer lidar_vertical_samples
ros2 param get /rm27_ground_truth_localizer no_return_horizontal_stride
ros2 param get /rm27_ground_truth_localizer no_return_vertical_stride
ros2 param get /rm27_ground_truth_localizer no_return_ray_length
ros2 param get /rm27_ground_truth_localizer rog_raycast_max_range
```

当前洞坡基线依次应为 `True, 360, 320, 1, 4, 10.5, 10.0`。参数在构造时读入；运行中
`ros2 param set` 即使返回成功也不会重建 localizer 内部配置。修改源 YAML 后必须重建、source 对应
install 并重启整套仿真。

当前 active/shadow MINCO profile 的安全组合应为：

```text
require_ground_support = true
ground_support_tolerance = 0.08
clearance_check_enable = true
unknown_as_occupied = true
observed_empty_as_free = false
bridge_observed_empty_for_ground_connectivity = false
clear_robot_footprint_unknown = true
near_field_prior_fill_enable = false
decay.keep_time = 3.0
decay.clear_time = 5.0
```

再检查启动日志是否真正加载了高程块：

```bash
ros2 topic echo /rosout rcl_interfaces/msg/Log \
  --qos-profile sensor_data \
  | rg --line-buffered \
  'loaded ground-elevation support|has no ground_elevation|ground support is fail-closed|prior map TF'
```

`tunnel_ramp_test` 应看到 `default=0.000 m, patches=2`；RMUC2026 和 highbay 应看到
`default=0.000 m, patches=0`。若仍出现 `has no ground_elevation`，先检查是否 source 了错误的
install、选错 map 或误用了旧参数文件。

核对当前 shell 实际使用的安装来源与已安装地图：

```bash
ros2 pkg prefix pb2025_nav_bringup
BRINGUP_SHARE="$(ros2 pkg prefix pb2025_nav_bringup)/share/pb2025_nav_bringup"
sed -n '/^ground_elevation:/,$p' \
  "$BRINGUP_SHARE/map/simulation/tunnel_ramp_test.yaml"
```

输出为空而源码文件有高程块，说明尚未重建或 source 了另一套 workspace。先修环境，不要在线 set
参数；地面高程由 map YAML 加载，不能通过单个 ROS parameter 动态补齐。

shadow 的所有 `MincoPlanner.*` 查询把节点名替换为 `/minco_shadow/planner_server`；shadow 没有
`MincoMpc`，不要执行 controller 参数对照。

优先改仓库 YAML 后重启并再次 dump。只有节点明确支持动态参数回调时才用 `ros2 param set`；“set successful”不等于内部对象已安全重建。

## 5. 目标发送与模式对照

### 5.1 正常导航目标

洞口后正向目标：

```bash
ros2 action send_goal \
  /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 3.5, y: 0.0}, orientation: {w: 1.0}}}}" \
  --feedback
```

回程目标：

```bash
ros2 action send_goal \
  /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: -2.0, y: 0.0}, orientation: {w: 1.0}}}}" \
  --feedback
```

### 5.2 触发异步规划诊断

`MincoPlanner::createPlan()` 会立即返回携带 planning token 的两点占位 Path，真正的
ROG 查询、全局 seed、MINCO 优化与安全复核在内部 FSM 异步进行。因此下面
`ComputePathToPose` 的 action `SUCCEEDED` 只说明请求被接收，**不说明 MINCO 已经规划成功**。

主 planner（只在 active `minco` 中注册 `MincoPlanner`）的接口握手检查：

```bash
ros2 action send_goal \
  /compute_path_to_pose nav2_msgs/action/ComputePathToPose \
  "{goal: {header: {frame_id: map}, pose: {position: {x: 3.5, y: 0.0}, orientation: {w: 1.0}}}, planner_id: MincoPlanner, use_start: false}" \
  --feedback
```

shadow planner（只在 `minco_shadow` 中存在）：

```bash
ros2 action send_goal \
  /minco_shadow/compute_path_to_pose nav2_msgs/action/ComputePathToPose \
  "{goal: {header: {frame_id: map}, pose: {position: {x: 3.5, y: 0.0}, orientation: {w: 1.0}}}, planner_id: MincoPlanner, use_start: false}" \
  --feedback
```

判定规划结果必须继续看同一 token 的 `/minco*/opt_path` `command_flag`、
`/opt_path_vis`、first-failure 日志和最终零速。shadow sidecar 将 request lease 关闭，使一个
已接受目标可由内部 FSM 持续异步重规划；active 的单次直接 ComputePath 不存在 10 Hz BT 心跳，会在
`request_lease_timeout` 到期后撤权并发 BLOCK，所以 active 运动验收必须使用
`/navigate_to_pose`。

调用 shadow action 前应在底盘侧确认运动输出已隔离。不要只依赖“shadow 不带 controller”这一点，因为同一进程中的旧 Nav2 主链仍可能有正在执行的目标。

### 5.3 legacy、shadow、minco 对照表

| 检查项 | legacy | minco_shadow | minco |
|---|---|---|---|
| 权威 planner | GridBased | GridBased | MincoPlanner |
| 权威 controller | FollowPath | FollowPath | MincoMpc |
| terrain_analysis | 开 | 开 | 关 |
| ROG/MINCO 旁路 | 无 | 有 | 无 |
| `/navigate_to_pose` 会动车 | 会 | 会，由旧链驱动 | 会，由新链驱动 |
| `/navigate_through_poses` | legacy 多点 | legacy 多点，不做 MINCO 旁路 | 不加载，fail-closed |
| motion behavior action | 原有集合 | 原有集合 | 仅无运动 `wait` |
| 适合首轮实车 | 旧基线 | 是 | 否 |
| MINCO 失败自动回旧链 | 不适用 | 主链本来就是旧链 | 不会 |

## 6. 分层调试：从输入到车轮

必须按本节顺序排查。下层输入没有验收时，禁止通过放宽上层阈值让车动起来。

### 6.1 第 0 层：时间、消息和 TF

检查仿真时钟：

```bash
ros2 param get /planner_server use_sim_time
ros2 param get /controller_server use_sim_time
ros2 topic echo /clock --once --qos-profile sensor_data
```

检查输入类型和发布者数量：

```bash
ros2 topic info -v /registered_scan
ros2 topic info -v /lidar_odometry
ros2 topic info -v /cloud_registered_full
ros2 topic info -v /aft_mapped_to_init
ros2 topic info -v /odometry
ros2 topic info -v /map
```

仿真 ROG frame 是 `odom`，规划 frame 是 `map`。`tf2_echo target source` 显示的是把 source 中的点变到 target 的变换，所以检查规划点送入 ROG 的方向应使用：

```bash
ros2 run tf2_ros tf2_echo odom map
ros2 run tf2_ros tf2_echo map odom
```

实车 ROG frame 是 `camera_init`：

```bash
ros2 run tf2_ros tf2_echo camera_init map
ros2 run tf2_ros tf2_echo map camera_init
```

地面高程写在 prior 的 `map` frame，投影运行在 `odom`（仿真）或 `camera_init`（实车）。实现使用
`T_map_rog` 的 `x/y/yaw/z` 做换算，其中：

```text
z_support_in_rog = z_support_in_map - T_map_rog.translation.z
```

该高程模型只支持水平 frame 之间的变换；`T_map_rog` 的 roll 或 pitch 绝对值超过 `1e-4 rad`、
四元数非法、TF 缺失/过期/来自未来，支撑缓存都会失效并全局闭锁。用下面命令同时观察平移和姿态，
不能只核对 XY：

```bash
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo map camera_init
ros2 topic echo /rosout --field msg | rg --line-buffered \
  'prior map TF|non-horizontal|ground support is fail-closed'
```

只执行与你当前部署匹配的一条 TF 命令。车辆上坡造成 `base_link` pitch 是正常现象；这里要求水平的
是定位世界 frame `map` 与 ROG 世界 frame `odom/camera_init` 的关系，而不是车体 frame。

prior transform 的 `x/y/z/yaw` 一旦变化，旧高程缓存会失效；required-support 模式随即强制整层
重投影，避免继续使用错位支撑。这是安全设计，但若定位不断发布微小的 `map -> odom/camera_init`
抖动，就可能每帧重建缓存并拉高 projection 延迟。当前变更比较容差约 `1e-9`，应把持续微抖当作
性能风险检查。复现时把 launch 输出和 TF 同时保存：

```bash
timeout 15s ros2 run tf2_ros tf2_echo map odom \
  | tee /tmp/rm27_map_odom_tf.txt
rg --line-buffered '\[PriorMap\] updated local projection cache|\[ROGMapPerf\]' \
  /tmp/rm27_tunnel_active.log
ros2 param get /planner_server MincoPlanner.rog_map.performance.print_enable
```

实车将第一条改为 `tf2_echo map camera_init`。`timeout` 正常结束时返回 124，不代表 TF 失败。
当前 performance print 每秒给出 `map_update_hz`，以及 `total`、`raycast`、`parallel`、`merge`、
`endpoint_dedup`、`prob`、`projection`、`field`、`snapshot` 分段耗时；若
`updated local projection cache` 接近点云频率、projection 延迟明显抬升或 map update 掉频，先修
上游世界 frame 稳定性。不要为了降 CPU 禁止 transform 变化后的全刷新，也不要未经误差评估对 TF
粗暴取整；正确做法是让定位只发布真实 map correction，并保证 ROG 世界 frame 水平、连续。

需要逐帧统计时，在对应 minco YAML 中临时打开
`MincoPlanner.rog_map.performance.detailed_csv_enable: true` 后重启。默认 CSV 为仿真
`/tmp/rm27_rog_map_simulation_detailed.csv`、实车
`/tmp/rm27_rog_map_reality_detailed.csv`：

```bash
head -n 1 /tmp/rm27_rog_map_simulation_detailed.csv
tail -n 10 /tmp/rm27_rog_map_simulation_detailed.csv
```

重点比较 `map_update_hz`、`projection_time_ms`、`total_update_time_ms` 与配置的 `20 ms` 更新周期；
实车改读 reality 文件。诊断结束应关闭逐帧 CSV，避免磁盘 I/O 反过来干扰实时性。

若 `raycast merge` 持续接近百毫秒，先确认没有运行旧的静态链接插件：本工程 `rog_map` 生成
`librog_map.a`，仅重编 `rog_map` 不会刷新已嵌入 `libminco_planner.so` 的代码。修改 ROG 后必须执行：

```bash
colcon build --symlink-install --packages-select rog_map minco_planner \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

2026-09-07 在 16 逻辑核开发机、RMUC2026、`360 x 320 / 1 x 8`、4 线程 raycast 下，修复参数
覆盖顺序并重链 `minco_planner` 后，静置稳态约为 `37800-40000` 个 ROG 输入、`8700` 个唯一终点，
`raycast=13-16 ms`、`prob=6-7 ms`、`projection=24-31 ms`、单帧总更新 `48-57 ms`，snapshot
复制通常 `0.6-1.1 ms`，地图约 `8-10 Hz`。运动时随有效点和投影分布变化，总更新通常约
`55-70 ms`。这是本机量级参考，不是跨机器硬阈值。若同一机器回到 `195-234 ms` 量级，先确认
现代 `raycasting.*` 没有被旧 `performance.*` 覆盖、静态链接插件已重链，再用 detailed CSV
定位阶段；不要靠增大 `map_timeout` 掩盖。

无论仿真还是实车，当前 MPC/local costmap 的 global frame 都是 `odom`，所以实车也必须另外检查：

```bash
ros2 run tf2_ros tf2_echo odom map
ros2 topic echo /odometry --field header.frame_id \
  --once --qos-profile sensor_data
```

预期 `/odometry.header.frame_id` 严格为 `odom`；ROG 的 `camera_init` 合同不能替代 MPC 的这个合同。

真值仿真还要核对 child frame 和 twist：

```bash
ros2 topic echo /odometry --once --qos-profile sensor_data
ros2 topic echo /lidar_odometry --once --qos-profile sensor_data
```

真值 localizer 不是从“回调时最新 GT”直接拼点云。它缓存原始 GT，按 LiDAR scan stamp 取精确
状态或两侧插值。首个有效真值只发布一次静态 `map -> odom`；动态 `odom -> base_footprint` 仍按原始
GT 高频发布。扫描进入 worker 时还会预发布一次 scan-time 动态 TF；no-return 重建、有限点压缩和
点云坐标变换完成后，再连续发布一次同 stamp TF、`/lidar_odometry` 与 `/registered_scan`。ROG 正常
路径直接使用配对 `/lidar_odometry` pose，不需要等待这个动态 TF；它保留给其他消费者和兼容回退。
worker mailbox 淘汰的扫描即使留下 TF，也只是合法的历史机器人位姿，不会留下没有对应点云的 ROG
odometry。默认合同是
GT 缓存 `2.0 s`、最大插值包围间隔 `0.10 s`、scan 等待 steady timeout `0.20 s`、最多 20 帧。
分别在两个终端观察以下字段，配对消息的 stamp 必须逐帧相同：

```bash
ros2 topic echo /registered_scan --field header.stamp --qos-profile sensor_data
ros2 topic echo /lidar_odometry --field header.stamp --qos-profile sensor_data
```

同时检查 ROG 每秒摘要：正常配对时 `cloud_odom_stamp_delta=0.00`；
`cloud_to_callback` 是点云到达 ROG callback 时相对 acquisition stamp 的总延迟，不能与前者混为一谈：

```bash
ros2 topic echo /rosout --field msg | rg --line-buffered \
  'ROGMapPerf|CloudFilter.*TF|Dropped lidar scans'
```

`cloud_odom_stamp_delta` 应接近 0；仿真 `cloud_to_callback` 的本机稳态基线约 `30-40 ms`。CloudFilter
使用配对 odometry pose 构造点云时刻的动态部分，只查询 `base_link <- lidar/imu` 静态外参。若日志出现
`matched odometry cannot provide ...; falling back to exact-time TF`，先核对 cloud frame、odometry
parent/child 和 `cloud_filter.pose_child_frame`，不要先加 timeout。`transform_timeout` 只覆盖初次静态
外参和兼容回退；它不会改变消息 stamp，也不能修复错误 frame 或持续断流。

`/odometry` 及 `map -> odom -> base_footprint` TF 保持原始 GT 更新率和原始 stamp，所以它们通常
比 LiDAR 配对输出频率高；不要把 `/odometry` 与每一帧 scan 不同 stamp 误报为同步故障。出现
`Dropped lidar scans: timeout=... overflow=... stale=... duplicate=... invalid=...` 时，先检查 GT/scan
时间源与调度；不要先增大 ROG 的 `cloud_odom_sync_tolerance`。

`/odometry` 的 pose/twist 对应 base 原点与 base 轴；`/lidar_odometry` 对应 LiDAR 原点与
LiDAR 轴。后者不是只换 `child_frame_id`，而是应满足：

```text
v_lidar = R_lidar_base * (v_base + omega_base x p_base_lidar)
omega_lidar = R_lidar_base * omega_base
```

当前传感器在 base `y=+0.18 m`；原地旋转时 `/lidar_odometry.twist.linear` 出现符合
`omega x p` 的切向分量是正常现象。若两条 odom 的 twist 数值始终完全相同，或者杆臂项符号相反，
停止规划并检查 `base_to_lidar` 方向。

#### Gazebo organized LiDAR 的 no-return 重建

这一适配只在 `use_ground_truth_odom:=true` 的仿真 localizer 中启用。Gazebo 的 organized GPU lidar
保留 `360 x 320` 射线栅格和 ring，但无回波点的 XYZ 是 Inf/NaN。若直接送入 PCL/ROG，这些点会被
跳过，射线经过的体素不会得到 miss/free 证据；在 `unknown_as_occupied=true` 和净空 known-ratio
门下，真实空旷区域会继续保持 UNKNOWN。

当前实现先验证完整 cloud layout、`x/y/z/ring` 字段和每行 ring，再按行列角重建**有方向信息的
Inf no-return**。纯 NaN 没有可信方向，保留非有限并计入 `unsupported_nonfinite`；任何 layout/ring
校验失败都会在修改 cloud 前拒绝整帧重建。有限回波先于 stride 判断，因而障碍 hit 无论位于哪一行
哪一列都原样保留。合成 miss 进入 ROG 后必须被最大射程截断并设置 `update_hit=false`，只沿射线更新
free，不能在末端制造假障碍。

参数源是
`src/pb2025_nav_bringup/config/simulation/nav2_params.yaml` 的
`rm27_ground_truth_localizer.ros__parameters`：

```yaml
reconstruct_no_return_rays: true
lidar_horizontal_samples: 360
lidar_vertical_samples: 320
lidar_horizontal_min_angle: 0.0
lidar_horizontal_max_angle: 6.283185307179586
lidar_vertical_min_angle: -0.1260127724939906
lidar_vertical_max_angle: 0.9637708129512688
no_return_ray_length: 10.5
rog_raycast_max_range: 10.0
no_return_horizontal_stride: 1
no_return_vertical_stride: 2
```

代码在节点构造时直接执行以下硬校验：

```text
no_return_ray_length >= rog_raycast_max_range + 2 * rog_map_resolution
10.5                    >= 10.0                  + 2 * 0.05
```

因为 `rog_map_resolution` 必须为正，该公式也保证 miss 端点严格位于 raycast 上限之外。等于最大
射程、只大一个浮点 epsilon 或不足两个 ROG 体素裕量都会让 localizer 启动失败。当前 `10.5 m`
满足 `10.5 >= 10.0 + 2 * 0.05`。

启动必须显式选择本任务 world：

```bash
ros2 launch rm_27_stimulation sim_with_nav.launch.py \
  world:=tunnel_ramp_test nav_world:=auto \
  use_ground_truth_odom:=true navigation_mode:=minco \
  enable_legacy_terrain:=auto gui:=false use_rviz:=false \
  2>&1 | tee /tmp/rm27_no_return_launch.log
```

启动后执行：

```bash
ros2 param dump /rm27_ground_truth_localizer \
  > /tmp/rm27_ground_truth_localizer_params.yaml
rg -n 'reconstruct_no_return|lidar_.*samples|no_return_.*stride|no_return_ray_length|rog_raycast_max_range' \
  /tmp/rm27_ground_truth_localizer_params.yaml
ros2 topic echo /registered_scan --once --qos-profile sensor_data --no-arr
rg -n 'organized no-return rays|No-return rays|reconstruction rejected|Dropped lidar scans' \
  /tmp/rm27_no_return_launch.log
```

正常启动先出现 `organized no-return rays=enabled`，随后每约 5 s 出现类似：

```text
No-return rays: synthesized=42956 finite_returns=29436 \
unsupported_nonfinite=0 stride_skipped=42808 length=10.50m stride=1x2
```

对一帧有效 `360 x 320` cloud，应满足：

```text
synthesized + finite_returns + unsupported_nonfinite + stride_skipped = 115200
```

计数会随车辆姿态和可见障碍变化，不能硬编码上面示例的四个分项。出现
`reconstruction rejected scan: INVALID_LAYOUT/INVALID_FIELDS/INVALID_RING_LAYOUT` 时，当前帧会保留非有限
no-return，使 ROG 继续 UNKNOWN；这是 fail-closed，不得改成“猜测角度后继续”。先核对
`width/height/point_step/row_step/ring` 和传感器 xacro。

stride 调参必须分两阶段：

1. 新传感器模型或修改重建代码后，先用 `horizontal_stride=1`、`vertical_stride=1` 跑语义基线。
   此时 `stride_skipped=0`，检查低梁、墙、坡面等 finite hit 全部仍在
   `/rog_map/raw_occupied`，同时纯 no-return 只产生 free ray。
2. 语义通过后才降低 miss 密度。当前部署基线是横向 `1`、纵向 `2`；它只抽取 no-return，所有
   finite hit 仍不降采样。近地约 `1.6 m` 盲环处，2 行纵向间距约 `0.011 m`，小于一个
   `0.05 m` ROG 体素；
   横向保持 1，避免方位采样跨过体素。
3. 每次只增加一个方向的 stride，并同时比较 known-ratio、首次 NORMAL 时间、ROG P95/P99、
   `STALE_SNAPSHOT` 和所有低梁/浮板负例。stride 过大会减少 free 证据，正确表现应是更保守的 UNKNOWN，
   不是“障碍更少”；若 finite hit 计数随 stride 改变，立即停止并回归单测。
4. 恢复部署基线 `1 x 2` 后重新构建、source、重启并用 parameter dump 留证。禁止在线 `param set`
   后认为 cached 配置已经更新。

针对性单测明确覆盖“stride 只跳 miss，finite hit 永远保留”和不安全射程/错误 layout 拒绝：

```bash
./build/rm_27_stimulation/test_organized_lidar_miss_rays
```

**实车不执行本小节。** reality profile 不应出现这些参数；实车先按第 3.3、6.5 和 12 节在 shadow
录包并验证真实点云组织形式，不能把 Gazebo 栅格角度套到 `/cloud_registered_full`。

Point-LIO 路线中的 `sensor_scan_generation` 另有一个更靠前的闭锁：它对每组同步输入按点云
stamp 查询 `lidar_frame <- robot_base_frame` 和 `lidar_frame <- base_frame`。任意一个 TF 缺失，
整组 `/sensor_scan`、`/odometry` 和派生 TF 都不发布；日志会明确列出两个查询的 `ok/missing`，
不存在 identity fallback。恢复后的首帧清空速度历史并以零 twist 冷启动，第二帧才有正常有限
差分速度。调试时可用：

```bash
ros2 topic hz /registered_scan
ros2 topic hz /lidar_odometry
ros2 topic hz /odometry
ros2 run tf2_ros tf2_echo left_mid360 base_footprint
ros2 run tf2_ros tf2_echo left_mid360 gimbal_yaw
ros2 topic echo /rosout --field msg | rg \
  'Required TF lookup|Dropping synchronized scan/odometry sample|cold-start twist'
```

同时检查机器人链：

```bash
ros2 run tf2_ros tf2_echo map base_link
ros2 run tf2_ros tf2_echo odom base_link
```

还要把“Nav2 平面定位基座”和“真实车体朝向”分开核对。两套 Nav2 costmap 的
`robot_base_frame` 当前都是 `gimbal_yaw_fake`；这个合成 frame 用于抵消云台/底盘 yaw，位置可正确，
但不能作为 MINCO 当前安全 footprint 的车体朝向。ROG 当前车体引导区和
`cloud_filter.position_frame=base_link` 都按真实 `base_link` 姿态建图，因此 PRIORMAP 中 MINCO 获取
当前位置后，必须显式取得 `planning_frame <- physical_base_frame` 的真实车体姿态。当前两套 profile
都设置 `MincoPlanner.frames.physical_base_frame=base_link`；PRIORMAP 保留 costmap control-point
的位置，只用该 TF 的 orientation 覆盖合成 frame orientation，EXPLORATION fallback 则使用完整的
physical-base pose。曾经直接
使用 `costmap_ros_->getRobotPose()` 的完整姿态，会令车体转过角度后安全 footprint 仍以约 0 yaw
查询，进而把真实已引导区域之外的格误判为 unknown/lethal。

一次隔离仿真已给出直接证据：`/lidar_odometry` 和 `map <- gimbal_yaw` 的真实 yaw 约为
`-0.286 rad`，`map <- gimbal_yaw_fake` 与 safety diagnostic yaw 都约为 `0.000 rad`；日志同时反复出现
`t=nan/nan sample=footprint`。这种“位置继续变化、诊断 yaw 恒零、转向后当前足迹闭锁”的组合应先
判为姿态源错误，不是 ROG 分辨率或 footprint margin 太大。

原地慢速转向时，在两个终端同时运行：

```bash
timeout 15 ros2 run tf2_ros tf2_echo map base_link
timeout 15 ros2 run tf2_ros tf2_echo map gimbal_yaw_fake
```

预期两者平移一致；`map <- base_link` yaw 跟随真实车体变化，而 `map <- gimbal_yaw_fake` yaw 可保持
接近固定值。此时若 first-failure 是 `t=nan/nan sample=footprint`，日志中的 `yaw` 必须和前者一致，
不能和后者一致。再检查实际生效的两个 frame：

```bash
ros2 param get /global_costmap/global_costmap robot_base_frame
ros2 param get /planner_server MincoPlanner.frames.physical_base_frame
ros2 param get /planner_server MincoPlanner.rog_map.cloud_filter.position_frame
```

预期依次为 `gimbal_yaw_fake`、`base_link`、`base_link`。如果 physical-base TF 缺失、非有限或无法
转换到 planning frame，MINCO 应 fail-closed 并先修 TF；禁止退回
固定 yaw、把 footprint 改成圆或扩大当前车体 unknown 引导区来掩盖姿态源错误。

停止条件：

- TF 跳变、反向、长期缺失或时间戳超前时停止后续调参。
- 点云/odom 频率不稳定或有多个非预期发布者时停止。
- 日志出现 cloud/odom reorder、sync、timeout、future stamp reject 时先修时间与来源。

当前 ROG 输入门限为 cloud/odom 同步约 `0.10 s`、odom timeout `0.25 s`、cloud timeout
`0.50 s`、future tolerance `0.05 s`。ROG 自身使用最近 odom，不在内部插值；真值模式由上游
localizer 先生成同 stamp 配对，Point-LIO 路线则仍依赖该同步门。不能靠无限放宽 timeout 掩盖不同步。

### 6.2 第 1 层：原始点云、坐标和自滤除

当前基线：

| 项目 | 仿真 | 实车 |
|---|---:|---:|
| cloud | `/registered_scan` | `/cloud_registered_full` |
| odom | `/lidar_odometry` | `/aft_mapped_to_init` |
| ROG frame | `odom` | `camera_init` |
| cloud z offset | `-0.45 m` | `-0.55 m` |
| vehicle height | `0.20 m` | `0.42 m` |
| headroom margin | `0.0 m` | `0.05 m` |
| body bottom clearance | `0.03 m` | `0.04 m` |
| origin ground offset | `0.20 m` | `0.28 m` |
| 兼容路径 ground seed radius（active support 模式不用） | `1.8 m` | `2.5 m` |
| self-filter box position | `(0, 0, 0.07) m` | `(0, 0, 0.05) m` |
| self-filter mode | `transform_cloud` | `transform_cloud` |
| self-filter box size | `(0.34, 0.28, 0.18) m` | `(0.34, 0.24, 0.14) m` |
| self-filter padding | `0.02 m` | `0.02 m` |

在 RViz 同时显示：

```text
/rog_map/raw_occupied
/rog_map/occupied
/rog_map/self_filter_box
```

判断：

- `raw_occupied` 中地面高度必须与机器人实际接地点一致。
- `occupied` 中车体自身回波应消失，洞顶、墙和坡面不能被一起删掉。
- `/rog_map/self_filter_box` 应包住车体回波，但不能侵入洞顶或坡面。
- 自滤除框尺寸、杆臂和 z offset 错误时，先标定，不能先改 headroom。

当前两套 profile 都使用 `transform_cloud`：点先按当时 TF 变换到 `base_link`，再和车体坐标系中的轴对齐框比较。因此 RViz 固定在 `map`/`odom` 时，
`/rog_map/self_filter_box` 必须跟随 `base_link` 的 yaw，上坡时也必须跟随 roll/pitch。如果 marker
仍始终与 odom 轴对齐，先查实际生效的 `filter_mode`、`position_frame`、点云 stamp 和
`cloud_frame <- base_link` TF，不得用扩大 box 来掩盖旋转错误。`transform_center` 只移动框中心，不能表示车体旋转，不是当前坡道验收基线。

本轮仿真曾出现一个典型自点闭锁：URDF 车轮中心位于 `y=+/-0.13 m`，轮宽 `0.05 m`，
因此横向外沿为 `+/-0.155 m`。旧 `box_size.y=0.24 m` 加两侧 `0.02 m` padding 只覆盖
`+/-0.14 m`，后轮回波持续进入 ROG，最终在当前 footprint 角点报
`COSTMAP_LETHAL`。仿真基线现为 `box_size.y=0.28 m`，加 padding 后覆盖
`+/-0.16 m`。这个数值只由仿真 URDF 得到，不能复制到实车。

遇到 `t=nan/nan sample=footprint` 的融合层 lethal 时，先把失败点换到车体系：

```text
dx = query_x - center_x
dy = query_y - center_y
x_body =  cos(yaw) * dx + sin(yaw) * dy
y_body = -sin(yaw) * dx + cos(yaw) * dy
```

再把 `(x_body,y_body)` 与 URDF/CAD 外形、`raw_occupied`、`occupied` 和 self-filter marker 对照。
`t=nan/nan` 在这里表示当前实际 footprint 检查，不是候选轨迹的未来时刻。框应只覆盖已测得的机器人自身包络；物理外形之外的点必须保留给 ROG 和 safety footprint。

建议一次只改 `MincoPlanner.rog_map.cloud_filter.z_offset` `0.01 m`，总搜索范围不超过当前值上下 `0.02 m`。自滤除 padding 当前为 `0.02 m`，在 `0-0.04 m` 内以 `0.01 m` 步进验证。如果需要更大值才能清掉自身点，优先怀疑物理外形、外参、时间同步或过滤框姿态。

### 6.3 第 2 层：三维概率占据

ROG 对每个体素累计 hit/miss 概率，并区分原始、动态和最终融合结果。当前基线和允许的首轮探索范围为：

| 参数 | 当前值 | 首轮探索 | 调法 |
|---|---:|---:|---|
| `raycasting.p_hit` | 0.90 | 0.86-0.94 | 漏障碍时增加，噪点固化时降低 |
| `raycasting.p_miss` | 0.45 | 0.40-0.48 | 用固定动态障碍移除 bag 验证，不凭视觉印象修改 |
| `raycasting.p_occ` | 0.85 | 0.80-0.90 | 确认 hit/miss 正常后再调判占阈值 |
| `decay.keep_time` | 3.00 s | 先不改 | 命中在这段时间内保持；不是地面支撑授权 |
| `decay.clear_time` | 5.00 s | 先不改且必须大于 keep | 从 keep 到 clear 逐步衰减，达到 clear 后清除 |
| `projection.min_observed_voxels` | 2 | 2-4 | 禁止改成 1 作为正式配置 |

`3.00/5.00 s` 是当前仿真和实车实际生效值，不是旧文档中的 `0.3-0.8/0.8-2.0 s`。
它最初覆盖约 `2.5 s` 的 MID360 近地盲环穿越时间，但接入地面高程后，decay 只负责占据证据的
时序稳定，不能拿“旧地面点还没衰减”当作车轮下方仍有支撑。调短前必须做移动障碍移除试验；调长前
必须确认最坏制动距离仍能容忍残影。任何值都不能把 UNKNOWN 或缺少高程支撑的列变成 free。

每次只调一项，固定点云 bag 回放，对比浮空障碍首次阻塞时间、移除后解除时间、孤立噪声数量和
静态墙稳定性。停止条件是静态墙体闪烁、真实障碍被 miss 清空、动态障碍解除慢于场景预算，或
`clear_time <= keep_time` 导致配置拒绝。

### 6.4 第 3 层：地面、坡度和净空

当前 active/shadow MINCO profile 都处于 required-support 模式。投影不是简单取一列最低点，而是：

1. 把每个 XY 列的体素分成 UNKNOWN、FREE、OCCUPIED，提取连续 occupied run。
2. 由同一张 prior map 做保守 `3 x 3` 采样；九点都为 known-free，且 YAML 有该 XY 的
   `ground_elevation`，才得到 trusted support 高程。
3. 每个可能放行的 support 列还检查八邻域高程连续性；与任一已知 support 邻格的高度差不能超过
   `max(max_ground_step, tan(max_ground_slope_deg) * planar_step)`。陡直 patch 跳变的两侧都闭锁。
4. 最低薄 occupied run 可成为地面候选，但其上表面与 trusted support 的差必须不大于
   `ground_support_tolerance`。相差过大的斜板、箱体或浮空薄片保持 `GROUND_UNVERIFIED`。
5. 没有 occupied run 的列不能仅凭“激光打空”放行。只有 trusted support 存在、列观测数达标，
   并且从支撑高程向上的完整车身带 known ratio 达标，才成为 support-verified empty。
6. 高位顶棚/前缘也从该列 trusted support 计算保守净空；低于
   `vehicle_height + headroom_margin` 或车身带观测不足都阻塞。
7. 车身净空带内任何 OCCUPIED、静态 prior OCCUPIED、TF/高程缺失或 UNKNOWN 证据不足均硬否决。

required-support 模式不会从机器人附近的薄表面自行播种，也不通过 BFS 把一个表面的“地面身份”传播
给另一个表面。这一点正是防止近场浮空斜板自封为地面的安全边界。`ground_seed_tolerance`、
`ground_seed_radius` 仍只用于非 required-support 兼容路径；但 `max_ground_step` 和
`max_ground_slope_deg` 也用于 active 的八邻域 support 连续性门。当前坡度首先应准确写进测绘
patch，再用这两个上限拒绝过陡坡/垂直台阶，不能靠放宽 BFS 或 support continuity 让错误 patch 通过。

源码仍保留一个非 required-support 的实验 bridge：它可以在最大长度内跨多个已观测为空且净空
验证的格，并对较长桥要求落地区间继续呈现一致坡度。它不跨 UNKNOWN/occupied，也禁止连续使用多座
bridge。然而“上方空气已观测为空”仍不能证明轮下有实体，斜浮板和真实坡面的 LiDAR 端点也可能
相似。因此仿真和实车 active 配置都将
`bridge_observed_empty_for_ground_connectivity: false`，并且 required-support 代码路径根本不使用
bridge。不要把 bridge 长度调到约 `2.35 m` 去覆盖 MID360 盲环；那会同时放行同长度的坑。

RViz/探针重点：

```bash
ros2 topic echo /rog_map/clearance_status --once --qos-profile sensor_data
ros2 topic echo /rog_map/headroom_known_ratio --once --qos-profile sensor_data
ros2 topic echo /rog_map/headroom --once --qos-profile sensor_data
ros2 topic echo /rog_map/layer_height_delta --once --qos-profile sensor_data
ros2 topic echo /rog_map/layer_confidence --once --qos-profile sensor_data
```

编码语义：

| topic | 值 | 含义 |
|---|---:|---|
| `clearance_status` | 0 | 没有地面候选，净空也未验证 |
| `clearance_status` | 4 | 无地面回波；二维 known-free + 高程支撑 + 车身带观测均验证 |
| `clearance_status` | 25 | 无地面候选但高位顶棚/前缘净空通过；active 中仍要求 trusted support |
| `clearance_status` | 100 | 地面未验证 |
| `clearance_status` | 50 | 地面已验证、净空未验证 |
| `clearance_status` | 3 | 地面候选与高程支撑匹配，且净空验证通过；当前 active 地面正例的主状态 |
| `clearance_status` | 5 | 仅当前自车 footprint：二维 known-free、匹配且连续的支撑、零 occupied，当前位置净空成立 |
| `clearance_status` | 2 | 非 required-support 兼容路径中的 bridge 地面；active 配置不应出现 |
| `clearance_status` | 1 | 非 required-support 兼容路径中的直接连通地面与净空均通过 |
| `headroom` intensity | `-1` | 没观察到上方占据，不等于净空已经验证 |
| `headroom_known_ratio` | 0-100 | 所需净空带内已知体素百分比 |

`clearance_status` 是 OccupancyGrid，坐标在消息 `header.frame_id`。把 first-failure 点先变到该 frame，
再按下面公式取格；不要直接拿 map 坐标索引 odom 网格：

```text
ix = floor((x - info.origin.position.x) / info.resolution)
iy = floor((y - info.origin.position.y) / info.resolution)
index = iy * info.width + ix
```

先用 `ros2 topic echo /rog_map/clearance_status --once --qos-profile sensor_data` 保存完整消息，确保
`0 <= ix < width`、`0 <= iy < height` 后再读 `data[index]`；越界不是 status 0，而是查询点不在滑窗内。

required-support 的地面参数建议顺序：

1. 先修 prior map 的 `ground_elevation`、二维 known-free 区域和 `map <-> ROG frame` TF。
2. 再验证点云 z offset、体素分辨率和地面候选上表面。
3. `ground_support_tolerance` 当前 `0.08 m`；只按测绘、TF Z、点云和体素量化误差预算小步调整。
4. 仿真 `obstacle_hold_time=0.0 s`、`hysteresis_count=2`：规则射线只保留两帧确认，不再叠加
   wall-clock 残影。实车仍为 `0.50 s/2`，必须用现场 bag 得到连续漏检上界后才可缩短。
5. 最后才看 `min_headroom_known_ratio`，不得用它掩盖错误的地面高程。

相邻列实际允许的高度差为：

```text
allowed_height_step = max(max_ground_step,
                          tan(max_ground_slope_deg) * planar_step)
```

两项取较大值但绝不相加。该公式既用于兼容 BFS，也用于当前 active required-support 的八邻域
高程连续性。提高任意一项都会放宽允许的测绘地面跳变；降低它们则可能在坡脚、坡顶或 patch 边界
形成闭锁带。它们只校验几何连续性，不能替代 `ground_support_tolerance` 对实际回波高度的匹配。

净空比例基线：

| 环境 | 当前值 | 允许探索范围 | 硬约束 |
|---|---:|---:|---|
| 仿真 | `0.25` | 0.25-0.50 | 只可向更严格方向验证；不得低于 0.25 |
| 实车 | `0.80` | 0.75-0.90 | 首次必须从 0.80 开始；不得复制仿真值 |

仿真 0.25 只补偿规则射线对车侧近地列的离散覆盖不足；所需车体净空带内的 occupied 检查仍然硬拒绝。实车若要从 0.80 降低，至少需要 30 次真实正例观测统计、完整负例拦截和评审记录。任何浮空障碍漏检都立即回退原值。

### 6.5 第 4 层：动态层、静态先验与最终融合

用四张图一起判断：

```text
/rog_map/layer_type
/rog_map/layer_value_dynamic
/rog_map/layer_value_static
/rog_map/layer_value
```

值的准确语义：

| topic | 值 | 含义 |
|---|---:|---|
| `layer_type` | 0 | UNKNOWN |
| `layer_type` | 33 | FREE |
| `layer_type` | 66 | PASSABLE |
| `layer_type` | 100 | OCCUPIED |
| `layer_value_dynamic` | 0/100 | 动态投影可通行/阻塞 |
| `layer_value_static` | 100 | 静态先验占据 |
| `layer_value_static` | 0 | 静态非占据；这里无法区分先验已知 free 与 unknown |
| `layer_value` | 0/100 | ROG field 和安全查询实际使用的融合结果 |

静态先验现在包含两部分：PGM 提供二维 occupied/known-free，YAML 的 `ground_elevation` 提供
在 map frame 中测绘的支撑平面或逐格高程。它仍不直接提供“当前净空”，净空必须来自本次三维观测。
当前规则是：

- 任一静态先验 occupied 均否决。
- 只有保守的 9 点采样都确认静态 known-free，才可能取得该列地面高程支撑。
- 地面候选必须在 `ground_support_tolerance` 内匹配；无地面回波时还必须观测到车身高度带为空。
- 先验不可用或 TF 无效时，融合结果保持阻塞。
- `prior_map.free_fills_unknown` 保持 false，禁止全局用二维 free 填满三维未知。

这也是为什么必须查看 `layer_value`，只看 `layer_type` 会误判规划器实际看到的地图。

#### `ground_elevation` YAML 格式

RMUC2026 使用与占据图严格对齐的 16 位栅格：

```yaml
ground_elevation:
  default_height: 0.0
  grid:
    image: RMUC2026_elevation.pgm
    scale: 0.001
    offset: -1.0
    no_data: 65535
  patches: []
```

有效值按 `z=offset+scale*value` 解码。grid 使用占据图的尺寸、分辨率和 origin；尺寸不一致会拒绝
启动。`no_data` 始终缺少支撑，即使存在 `default_height` 也不会静默回退；经复核的 patch 可以局部
覆盖 grid 或 no-data 单元。生成器对重叠洞体先保留常规最高坡面，仅在该面不连续时从相邻支撑沿
坡度最连续的候选相交面恢复洞底或中间坡面。RMUC2026 当前有 `113792/113794` 个 known-free
单元具备支撑，有效范围为 `0.000..0.310 m`，不是全图 `z=0`。

仿真现有文件 `src/pb2025_nav_bringup/map/simulation/tunnel_ramp_test.yaml` 是可执行示例：

```yaml
image: tunnel_ramp_test.pgm
mode: trinary
resolution: 0.1
origin: [-3.0, -3.5, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25

ground_elevation:
  default_height: 0.0
  patches:
    - name: ramp
      bounds: [0.442, -0.675, 1.641, 0.675]
      reference: [0.442, 0.0, 0.039]
      slope: [0.2085, 0.0]
    - name: platform
      bounds: [1.65, -0.675, 4.55, 0.675]
      reference: [1.65, 0.0, 0.25]
      slope: [0.0, 0.0]
```

所有数字都在 `projection.prior_map.frame_id`，当前为 `map`。每个 patch 的平面是：

```text
z(x,y) = reference[2]
       + slope[0] * (x - reference[0])
       + slope[1] * (y - reference[1])
```

`bounds=[min_x,min_y,max_x,max_y]` 为含边界矩形，`slope` 是无量纲 `dz/dx, dz/dy`，不是角度。
多个 patch 重叠时后写的覆盖前写的；`name` 仅供人阅读。patch 外使用 `default_height`，但只有 PGM
九点都 known-free 的列才成为 trusted support。不要在多层地面、地坑或尚未测量区域随意填写一个
覆盖全图的默认高度；应把不可信区域在 PGM 中保留为 unknown/occupied，或拆分专用导航地图。

#### 实车高程标定流程

1. 保持底盘失能或使用 `legacy`，记录当前 map、定位版本和 `map <- camera_init` TF。
2. 在 map frame 中测量平地、坡脚、坡顶、平台四角及横向两侧，单位统一为米；不要使用相对车体高度。
3. 每个连续平面拟合 `reference` 和 `slope`，边界只覆盖确实测量且 PGM known-free 的区域。坡角
   `theta` 转换为沿坡方向斜率时使用 `slope=tan(theta)`。
4. 把块写入实际 launch 选中的 `src/pb2025_nav_bringup/map/reality/<map>.yaml`，重新构建/安装并
   source；不要改 `install/` 副本。
5. 先启动 `minco_shadow`，确认加载日志、TF Z 和 `clearance_status=3/4/25` 的空间位置；随后做
   错高浮空板、静态墙和地图边界负例。
6. 保存测点、拟合残差、参数 diff、bag 和 RViz 截图。残差预算不能靠不断增大 tolerance 吞掉。

实车模板如下，尖括号必须替换成测量值，不能原样启动：

```yaml
ground_elevation:
  default_height: <measured_flat_ground_z_in_map>
  patches:
    - name: <surveyed_ramp_name>
      bounds: [<min_x>, <min_y>, <max_x>, <max_y>]
      reference: [<x_ref>, <y_ref>, <z_ref>]
      slope: [<dz_dx>, <dz_dy>]
```

只改地图后可重建 bringup 资源并刷新当前 shell：

```bash
cd /home/pnx/nav_ws/sentry-navigation-RM27
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select pb2025_nav_bringup
source install/setup.bash
```

RMUC2026 的 STL/world/PGM 任一改变后，必须重生成并核对高程图：

```bash
cd /home/pnx/nav_ws/sentry-navigation-RM27
python3 src/pb2025_nav_bringup/tools/generate_rmuc2026_elevation.py
python3 src/pb2025_nav_bringup/tools/generate_rmuc2026_elevation.py --check
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select rog_map pb2025_nav_bringup
```

生成输出必须仍显示 `support=113792/113794` 和 `z=[0.000, 0.310] m`。数量或范围变化说明场景几何、
二维图或筛选阈值已经变化，应先检查差异，不能直接把新文件用于发布。

RMUC2026 连续两级坡面的 headless 往返验收命令如下。目标 `(-3.0,-3.0)` 已确认同时位于二维
known-free 和 `0.309 m` 高程平台；不要仅按 STL 包围盒猜目标，二维 PGM occupied 始终有否决权。

```bash
# shell A
cd /home/pnx/nav_ws/sentry-navigation-RM27
source install/setup.bash
ros2 launch rm_27_stimulation sim_with_nav.launch.py \
  world:=RMUC2026 navigation_mode:=minco use_ground_truth_odom:=true \
  gui:=false use_rviz:=false

# shell B：确认新 grid 确实已由 planner 插件加载后再发目标
cd /home/pnx/nav_ws/sentry-navigation-RM27
source install/setup.bash
ros2 lifecycle get /planner_server
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: -3.0, y: -3.0}, orientation: {w: 1.0}}}}"
ros2 topic echo /ground_truth/odometry --once --field pose.pose.position

# 下坡回到默认出生区
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: -11.7, y: 2.9}, orientation: {w: 1.0}}}}"
```

当前基准结果中两次 action 都应为 `SUCCEEDED`；ground-truth Z 从约 `0.069 m` 经第一段坡面的
`0.165 m`、第一层平台的 `0.269 m` 到中央平台 `0.369 m`，返回后约 `0.069 m`。车体参考点比
支撑面高约 0.06 m，因此它们分别对应 grid 的 `0.000/0.10/0.209/0.309 m`，不是高程偏移错误。

核验命令：

```bash
MAP_YAML=src/pb2025_nav_bringup/map/reality/highbay.yaml
sed -n '/^ground_elevation:/,$p' "$MAP_YAML"
ros2 param get /minco_shadow/planner_server \
  MincoPlanner.rog_map.projection.prior_map.require_ground_support
ros2 param get /minco_shadow/planner_server \
  MincoPlanner.rog_map.projection.prior_map.ground_support_tolerance
ros2 run tf2_ros tf2_echo map camera_init
ros2 topic echo /rog_map/clearance_status --once --qos-profile sensor_data
```

当前 `highbay.yaml` 已包含平地 `z=0` 高程块；实车仍须先在 shadow 中证明该高度与点云、TF 和地图
一致。新增坡面必须添加经测量的 patch。禁止把 `require_ground_support` 改成 false、打开
`observed_empty_as_free` 或 bridge 来绕过错误的高程/TF。

静态高程是“现场在测绘时存在地面”的先验，不是下视传感器。若先验记着地面，而运行时突然出现新坑，
且 MID360 下视盲区没有地面回波也没有坑底/边缘证据，那么“坑”和“有地面但没回波”在观测上不可区分。
当前实现可能依照先验与上方空闲观测继续放行。存在坠落风险的实车必须增加下视 LiDAR/深度/ToF 或
触地/悬空检测，并把它接入支撑否决；在此之前把潜在坑区标成非 known-free。调任何 MINCO/MPC 参数
都不能解决这个可观测性限制。

### 6.6 第 5 层：近场补全与机器人包络

required-support 接管了近地盲区授权。observed-empty 和 bridge 旁路在仿真、实车均关闭；实车
near-field prior fill 也保持关闭。RMUC2026 仿真只在车前短扫掠区启用 near-field 补偿，并继续强制
known-free、连续匹配高程和零 occupied。current-footprint 则保留严格、随车移动的启动特例：

```yaml
projection:
  unknown_as_occupied: true
  clearance_check_enable: true
  observed_empty_as_free: false
  bridge_observed_empty_for_ground_connectivity: false
  clear_robot_footprint_unknown: true
  near_field_prior_fill_enable: false  # 实车；RMUC2026 仿真为 true
  prior_map:
    free_fills_unknown: false
    require_ground_support: true
    ground_support_tolerance: 0.08
```

这不是按矩形无条件“清 UNKNOWN”。bootstrap cell 只有同时位于随车局部包络、二维 prior
known-free、本格支撑高度与当前参考地面相差不超过 tolerance、八邻域支撑连续且整柱无 occupied
证据时，才以 `clearance_status=5` 释放。它每帧按当前 pose 重算，旧位置不会留下 free trail；
包络栅格化包含旋转后半个 cell 的投影 padding，避免连续 footprint 边界相交格遗漏，但 padding
不会绕过支撑或 occupied 门。包络外的 empty column 仍须满足支撑和车身带观测，才以 status 4
释放。

实车 current-footprint 几何为 `0.40 x 0.30 m`。RMUC2026 仿真因规则射线近场栅格空洞，使用
`0.52 x 0.51 m`：它覆盖 `0.32 x 0.31 m` 硬 footprint 外的一个 `0.10 m` ROG seed 步长，且仍受
known-free、连续高程和零 occupied 三重限制。下面是实车值；仿真 offset 相同、长宽分别为
`0.52/0.51 m`：

```yaml
projection:
  robot_footprint_clear_length: 0.40
  robot_footprint_clear_width: 0.30
  robot_footprint_clear_offset_x: 0.0
  robot_footprint_clear_offset_y: -0.18
  near_field_prior_fill_length: 0.80
  near_field_prior_fill_width: 0.50
```

仿真 near-field 为 `1.40 x 1.00 m`，覆盖 `0.16 m` 前悬与 1 m/s、1 m/s2 下约 `0.50 m` 的理论制动
距离；它只能修复有完整静态支撑先验的规则射线盲带。实车保持 `near_field_prior_fill_enable:false`。

当前 required-support 诊断步骤：

1. 车静止在已知 free 平地，显示 dynamic、static、fused。
2. 用 `clearance_status` 区分实测地面 `3`、当前 footprint `5`、有先验支撑的空列 `4` 和顶棚净空
   `25`；不接受无来源的 `1/2`。
3. 在盲区路径上放一个小箱体或错高斜板，确认 dynamic 和 fused 都 blocked，地面候选不得变成 `3`。
4. 把箱体移除，检查残影按设计消退而非立即被补全吞掉。
5. 把车靠近静态墙，确认 prior occupied 始终阻塞。
6. 暂时使用缺少 `ground_elevation` 的地图或破坏 prior TF，仅在仿真中确认整层闭锁和最终零速。

不要通过打开 near-field/bridge/observed-empty 旧旁路解决 `status=0/100`，也不要扩大 footprint
包络去“探路”。应依次检查 PGM known-free、ground-elevation patch、
TF Z、候选地面 Z、tolerance 和 body known ratio。小障碍、墙、错高浮空板或先验边界外区域被释放
都是 release blocker。

### 6.7 第 6 层：signed distance field 与规划输入

```bash
ros2 topic echo /rog_map/field --once --qos-profile sensor_data
ros2 topic echo /rog_map/layer_value --once --qos-profile sensor_data
```

`/rog_map/field` 点强度是 signed distance。当前 field inflation 为 `0`，轨迹最终碰撞阈值
`collision_dist=0`。安全检查仍必须执行 field query，并对 TF、快照 freshness、非有限距离和 query
失败保持 fail-closed；但零阈值下以先行检查的精确投影 cell 为硬碰撞结论，不用相邻 occupied 造成
的轻微负插值距离额外扩大 footprint。`collision_dist>0` 时才再要求
`distance > collision_dist`。安全裕量主要来自：

- footprint 多点查询；
- footprint margin，当前约 `0.05 m`；
- optimizer 的 `safe_dist`，当前 `0.30 m`，它是优化软约束而非最终碰撞阈值；
- corridor radius 与 extra margin。

当前配置 `rog_map.esdf.enable: false`。这里实际用于 MINCO 的是三维柱投影生成的二维 signed distance
field；first-failure 字段仍名为 `esdf_distance`，但不能据此把它理解为可供 SE(3) 规划的三维 ESDF。

不要同时增加 field inflation、collision distance、footprint margin 和 safe distance。那会重复膨胀，让短洞口无解且无法判断是哪一层导致。

如果日志里的 planning frame 与 ROG frame 不同，先按第 6.1 节核对 TF。轨迹点从 `map` 查询到仿真 `odom` 使用 `odom <- map`，实车使用 `camera_init <- map`。

ROG field 的网格采样点位于：

```text
cell_center = origin + (index + 0.5) * resolution
```

当前 `field.interpolation` 是 `quadratic`。因此不能拿日志中的任意连续坐标，直接和
`OccupancyGrid` 中“看起来最近的 x=0 格”作一对一比较；应先按 origin/resolution 算格索引，再同时
检查相邻格和 `/rog_map/field` 的 cell-center 点。轨迹安全使用的是 `/rog_map/layer_value` 对应的
fused snapshot，不是只看动态分类的 `/rog_map/layer_type`。

## 7. MINCO first-failure 安全日志

轨迹安全检查只报告每条候选轨迹的首个失败样本，并对重复日志按约 1 秒节流，格式包含：

```text
[MincoPlanner] Trajectory safety rejected:
reason=...
t=.../...
sample=center|footprint
center=(x,y,z)
query_point=(x,y,z)
footprint_offset=(x,y)
yaw=...
esdf_distance=...
safe_distance=...
cost=...
query_status=...
snapshot_age=...
snapshot_commit_age=...
pipeline_age_ms=...
cost_source=...
cost_cause=...
cell_type=... raw_type=... candidate_type=... base_type=...
pending_type=... pending_count=... hole_filled=... hold_remaining=...
raw_reason=... candidate_reason=...
planning_frame='...'
rog_frame='...'
```

抓取方式：

```bash
ros2 topic echo /rosout rcl_interfaces/msg/Log \
  --qos-profile sensor_data \
  | rg --line-buffered "Trajectory safety rejected|MincoPlanner|ROG"
```

或直接在 launch 终端保存：

```bash
ros2 launch rm_27_stimulation sim_with_nav.launch.py \
  world:=tunnel_ramp_test nav_world:=auto \
  use_ground_truth_odom:=true navigation_mode:=minco_shadow \
  2>&1 | tee /tmp/minco_shadow.log
```

注意：`center` 和 `query_point` 坐标都按 `planning_frame` 打印，`rog_frame` 用来说明底层查询地图所在 frame；不要看到 `query_point` 就把它当作已经转换后的 odom/camera_init 坐标。

一个具体换算例子：若日志 `planning_frame=map`、`rog_frame=odom`、中心
`x_map=-2.220`，而 `map <- odom` 的 x 平移是 `-2.25 m`，则应查询的 ROG 坐标约为
`x_odom=+0.030 m`。用于直接验证的命令是 `tf2_echo odom map`，即请求
`odom <- map`；把变换方向写反会造成约两倍平移量级的错误判断。

常见 reason：

| reason | 解释 | 第一动作 |
|---|---|---|
| `QUERY_UNAVAILABLE` | ROG/适配器没有可用快照 | 查点云、odom、ROG 初始化 |
| `OUT_OF_MAP` | 点超出当前安全查询地图范围 | active MINCO 先查 ROG origin、滑窗尺寸和 TF |
| `COSTMAP_UNKNOWN` | 查询适配器返回 unknown cost | active MINCO 先查 ROG fused layer、观测和 TF |
| `COSTMAP_LETHAL` | 查询适配器返回 lethal cost | active MINCO 先查 ROG fused occupied 与实际障碍 |
| `COSTMAP_INSCRIBED` | 查询适配器返回 inscribed cost | 查 ROG field、footprint 和 margin |
| `QUERY_FAILED` | ROG field 查询失败 | 查快照、frame、坐标范围 |
| `INVALID_SNAPSHOT_STAMP` | 地图时间戳非法 | 查 `use_sim_time` 与上游 stamp |
| `INVALID_ROS_TIME` | 当前 ROS time 非法 | 查 `/clock` |
| `STALE_SNAPSHOT` / `FUTURE_SNAPSHOT` | 地图过期或来自未来；边界 stale 也可能是 planner callback 自饿 | 按 7.1 同时查输入、ROG perf、CPU 和失败时刻 |
| `NONFINITE_DISTANCE` | ESDF 返回 NaN/Inf | 保存 layer/field 输入并停止运动 |
| `INSUFFICIENT_CLEARANCE` | signed distance 不够 | 用 query point 在 field/layer 上定位真实原因 |

`COSTMAP_LETHAL` 不能只看 `raw_reason`。最终 lethal 可能来自静态先验、原始分类、unknown
fail-closed、时间保持、迟滞或二维补洞；新日志中的 `cost_source + cost_cause` 已把这些阶段拆开：

| `cost_source / cost_cause` | 真正含义 | 应调哪一层 |
|---|---|---|
| `PRIOR_MAP / PRIOR_MAP` | PGM 先验该格占据 | 地图原点、分辨率、TF、PGM 像素；不要调点云阈值 |
| `DYNAMIC_PROJECTION / RAW_OCCUPIED` | 当前帧三维柱直接判障碍 | 看 `raw_reason`、高度、净空、ground/support 字段 |
| `DYNAMIC_PROJECTION / UNKNOWN_AS_OCCUPIED` | 观测不足按 fail-closed 转 lethal | 先修视场、miss ray、TF 和 ground support；禁止直接关闭实车 fail-closed |
| `DYNAMIC_PROJECTION / OBSTACLE_HOLD` | 当前 raw 已可通行，但旧占据仍在保持时间内 | 看 `hold_remaining`，再按实测漏检窗口调 `obstacle_hold_time` |
| `DYNAMIC_PROJECTION / HYSTERESIS` | 当前 raw 已变化，仍等待连续确认 | 看 `pending_type/pending_count`，调 `hysteresis_count` |
| `DYNAMIC_PROJECTION / MASK_HOLE_FILL` | 当前格被 8 邻域补洞抬成占据 | 看 `hole_filled=1`，调 `fill_occ_min` 或上游邻格分类 |
| `DYNAMIC_PROJECTION / MASK_DENOISE_TO_UNKNOWN` | 孤立 base occupied 被去噪为 UNKNOWN，但 fail-closed 又把 UNKNOWN 编成 254 | 看 `base_type=OCCUPIED, cell_type=UNKNOWN` 及上游 hold/hysteresis；不要关闭 unknown fail-closed |
| `DYNAMIC_AND_PRIOR / ...` | 动态层与先验同时占据 | 两套证据都要处理，不能只放开其中一个 |

`cell_type` 是 mask 后最终类型，`base_type` 是 mask 前的时间滤波类型，`raw_type/raw_reason` 是本帧
直接分类。比如 `raw_type=FREE, base_type=OCCUPIED, hold_remaining>0` 就是保持；
`base_type=FREE, cell_type=OCCUPIED, hole_filled=1` 才是二维补洞。这样不再需要从一个
`cost=254` 猜是哪层造成的。

排查首碰撞点：

1. 记下 `t`、`sample`、`center`、`query_point`、frames。
2. 若 frames 不同，用 `tf2_echo` 验证该时刻附近的变换方向和数值。
3. 在 RViz 同时叠加 `/opt_path_vis`、`/rog_map/layer_value`、`/rog_map/field`。
4. `sample=footprint` 时检查 offset 旋转后的角点，而不是只看中心线。
5. 对照 `cost` 与 `esdf_distance`：适配器先检查离散 cost，再查询距离。若离散 cost 已是 unknown/lethal/inscribed，日志出现 `query_status=NOT_QUERIED` 和非有限 distance 是预期的短路结果。当前 PRIORMAP 的最终动态安全查询是 `FrameAwareRogQuery`，因此这里的 `COSTMAP_*` 是沿用的枚举名，首先对应 ROG fused projection，不应直接归因于 Nav2 global costmap。Nav2 global costmap 主要在更早的全局搜索阶段使用。
6. 若 snapshot age 接近 timeout，先按下文区分上游真断流、计算过载和 safety callback
   自身阻塞地图更新；禁止直接提高 timeout。

仿真当前 `map_timeout=2.20 s`，实车 `0.50 s`。这两个值不是性能调节旋钮；超过正常发布周期数倍仍 stale，说明数据链有故障。仿真值用于容纳已观测到的约 2.1 s Gazebo 规则射线调度尖峰，并严格小于 3.0 s occupied 保留期；实车不得照抄。

### 7.1 footprint 批量快照与“自饿” `STALE_SNAPSHOT`

先明确当前实现的快照粒度，否则很容易把一次批量查询误认为整条轨迹的原子快照：

- 每个 trajectory time 的完整旋转 footprint 会组成一次 `queryBatch()`。当前仿真有效
  footprint 为 `0.32 x 0.31 m`、ROG 分辨率 `0.05 m`、硬检查
  `collision_dist=0`，因而间距为 `0.025 m`，一批约为 `13 x 13 + 1 = 170`
  个查询点。最后的 `+1` 是显式中心点。
- `FrameAwareRogQuery` 对这一批只做一次 `planning_frame -> rog_frame` TF 查询；
  `QueryAdapter` 在批次开头捕获一份 immutable `MapSnapshot`。该批内的 projected cost、
  distance、snapshot stamp 和 sequence 来自同一快照，gradient 也用同一 TF 旋回规划系。
- 批次查询后先用同一 `query_time` 判断所有点，解析完整批次后又用
  `completion_time` 复查 freshness；批处理本身耗尽时间预算也会 fail-closed。
- **整条轨迹不是一次 batch**。它仍以 `sample_dt=0.05 s` 逐个 footprint 查询，并强制检查终点。
  `TrajectorySafetyChecker::querySnapshot()` 当前只捕获 query-interface 指针，不等于冻结整条轨迹所用的
  `MapSnapshot`。不得在评审或测试报告中声称“trajectory-wide atomic snapshot”。

默认非 composition 的 active `minco` 启动使用 `minco_planner/planner_server_mt` 的 8 线程
executor；composition 使用 `component_container_mt`。ROG odom、cloud、map-update 和可视化各有独立
callback group；MINCO odom、FSM、safety、lease 和轨迹可视化也分别使用独立 callback group。
这样长时间 global search/MINCO 优化不会占住默认组并连带延迟 odom、安全 watchdog 或可视化。
当前线程数覆盖主要并发组，但它不是硬实时保证。

ROG 可视化读取与地图写入仍需要同一个地图锁，但 map-update 申请锁后会阻止新的可视化进入；地图锁
已占用时本轮可视化也不会排队。所有可视化 DDS publish，以及 occupied/unknown/inflated 等体素点云
的 ROS 序列化，都在地图解锁后执行。超过 250 ms 的剩余采样/复制锁持有仍会输出
`visualization held the map lock`。真值 `/registered_scan` 使用 `best_effort + keep_last(1)`，负载过高
时丢旧帧而不是排队回放。批量查询已经消除了逐点重复 TF/快照锁的主要开销，但完整轨迹扫描仍必须
做 P99 验收。

| 观测 | 上游真断流/真过期 | safety 自饿或 planner 计算过载 |
|---|---|---|
| `/registered_scan` 或实车 cloud | 频率消失，header delay 持续增长 | DDS 输入仍稳定 |
| `ROGMapPerf cloud_cb_hz/map_update_hz` | 降到 0 或与输入同时中断 | 目标/安全扫描期间窗口拉长，callback 结束后立即恢复 |
| first-failure `t` | 常在当前 footprint 或轨迹起点就 stale | 常在轨迹中段 `t>0` 才跨过 timeout |
| `snapshot_age` 序列 | 随时间继续增长，明显超过阈值 | 重复卡在阈值边缘，例如 `0.751` 对 `0.750 s` |
| 无目标静置 | 仍 stale | 取消扫描负载后地图 age 恢复正常 |

不要只看一个 `topic hz`。在发目标前启动下列探针，然后对照无目标、规划中和 BLOCK 后三个时段：

```bash
# 仿真输入；实车替换为 /cloud_registered_full
ros2 topic hz /registered_scan
ros2 topic delay /registered_scan --use-sim-time

# 可视化 topic 只在有订阅者时组装，一次只开必要的一个
ros2 topic echo /rog_map/layer_value --field header.stamp \
  --qos-profile sensor_data

pgrep -af 'planner_server'
pidstat -p <planner_server_PID> 1

rg -n 'ROGMapPerf|STALE_SNAPSHOT|Planning-request heartbeat lease|Failed to make progress' \
  /tmp/rm27_tunnel_active.log
```

启用 no-return 重建时，再把 localizer 与 planner 分开看。先用 `pgrep -af` 得到并人工核对两个精确
PID，再执行：

```bash
pgrep -af 'rm27_ground_truth_localizer|planner_server'
LOCALIZER_PID=<exact_localizer_pid>
PLANNER_PID=<exact_planner_pid>
pidstat -p "$LOCALIZER_PID","$PLANNER_PID" 1

ros2 topic hz /livox/lidar
ros2 topic hz /registered_scan
ros2 topic hz /lidar_odometry
rg --line-buffered \
  'No-return rays|reconstruction rejected|Dropped lidar scans|ROGMapPerf|STALE_SNAPSHOT|FUTURE_SNAPSHOT' \
  /tmp/rm27_no_return_launch.log
```

判断顺序如下：

1. `/livox/lidar` 稳定而 `/registered_scan` 掉频，且 localizer CPU 高：先查 full no-return 重建、
   PCL transform 和 publisher QoS 是否仍为 `best_effort/keep_last(1)`；在语义 `1 x 1` 已通过后恢复
   部署 stride `1 x 2`。
2. `/registered_scan` 稳定而 `ROGMapPerf cloud_cb_hz/map_update_hz` 降低，planner CPU 高：负载在
   ROG 更新、投影、field 或 safety callback，不应继续调 localizer stride 猜原因。
3. 两者都稳定但 first-failure 在非零轨迹时刻跨过 `map_timeout`：按本节前述“自饿”路径查完整
   footprint 扫描和 executor 调度。
4. 日志出现 no-return layout/ring reject：该帧保持 UNKNOWN 后导致 BLOCK/`COSTMAP_UNKNOWN` 是正确
   fail-closed。先修传感器消息合同，禁止打开 `observed_empty_as_free`、关闭
   `unknown_as_occupied` 或延长 map stamp。
5. synthesized 数量很大本身不是错误；要同时看输入频率、每帧总计数、ROG 更新率和 P95/P99。
   当前 full `1 x 1` 每帧最多把 115200 个栅格点送入下游，`1 x 2` 只减少 miss，不能减少 finite hit。

`map_timeout` 仍是安全 watchdog，不是吞吐调节器。先降低**已验证可抽样的 miss**、关闭非必要可视化、
确认 Release 构建并定位 callback 热点；不得通过增大 timeout、伪造 sensor stamp、跳过 completion
freshness 或把 invalid no-return 当 free 来消除 fail-closed。

如果日志持续出现 `path has fewer than 2 poses`，并且 start 到 goal 的距离小于 SMAC `tolerance`，
这不是障碍物或点云自滤失败，而是搜索起点已经满足全局搜索容差。当前实现会把这种单节点搜索结果
补成“起点栅格中心到精确目标”的短路径，随后仍执行完整 ROG footprint 检查；禁止用反复发送目标、
关闭安全检查或扩大 self-filter 来掩盖该问题。

坡面上的当前车体脚印放行会优先采用 `ground_elevation` 在车体中心处的高程作为参考地面，
`robot_origin_to_ground` 只在高程先验或其 TF 不可用时回退使用。雷达存在水平安装偏移，车体倾斜后
该偏移会改变雷达的世界坐标高度；若始终用 `lidar_z - robot_origin_to_ground`，下坡侧脚印可能被
误判为 `HEADROOM_UNVERIFIED`。该修复只对“静态地图已知可通行、连续高程匹配、且没有占据体素”的
当前脚印生效，不会绕过真实障碍物。

`pidstat` 中持续高 CPU、ROG perf 窗口在 safety 运行期间拉长，且 stale 总在非零
trajectory time 以近乎相同的边界 age 出现，应先归因计算/调度。反之，输入频率为 0 且 age
一直增长，才先查传感器、localizer、QoS 和 stamp。两者可以同时发生：输入频率正常但
`cloud_cb_hz` 明显下降，就是 planner 回调已经接不住输入。

修复顺序是：先确认当前二进制已包含 footprint `queryBatch()` 与单批一次 TF，再用性能数据减少重复安全扫描/可视化负载，必要时重新设计 callback 调度。不得伪造快照 stamp、跳过 completion freshness 复查，也不得以提高
`map_timeout` 作为首个修复。

2026-09-06 的最终回归从 `map(-11.70, 2.90)` 导航到 `map(3.042, 7.586)`，同时持续订阅
`/rog_map/occupied` 模拟 RViz 负载。action 为 `SUCCEEDED`，最终真值约
`map(2.934, 7.575)`；完整目标执行期间没有 `STALE_SNAPSHOT/FUTURE_SNAPSHOT`，也没有超过 250 ms
的可视化地图锁告警。启动段紧凑 ROG 输入约 `3.8-4.0 万点/frame`，occupied 可视化约 1 Hz。
该回归只证明周期性快照过期制动已消除，不替代 P1-P6。

同一路线早期日志在 `map(-1.09, 7.56)` 附近出现一次 `COSTMAP_LETHAL` 紧急 BLOCK，之后安全重规划
并到达。当时日志只有最终 `cost=254`，曾不足以证明它就是“当前帧新障碍”。2026-09-07 加入不可变
快照分类证据后，邻近复现点显示 `cost_source=DYNAMIC_PROJECTION`，但
`raw_reason=EMPTY_COLUMN`、`prior_free=1`、`empty_support=1`、`clearance_verified=1`。因此该次最终
lethal 是 raw 分类之后的保持/迟滞/补洞阶段，不是先验墙，也不是当前柱的实体障碍。新
`cost_cause` 字段用于把这三者继续精确拆开；不得再只凭 `cost=254` 推断成真实障碍。

### 7.2 历史案例：洞顶分类修复后仍在坡前闭锁

修复前的一次 active 仿真中，机器人从 `map(-2.25, 0)` 向 `map(3.5, 0)` 能正常起步，随后首个
拒绝为 ROG fused projection 返回的 `COSTMAP_LETHAL`。first-failure 点为：

```text
map(-0.398, -0.185) -> odom(约 1.852, -0.185)
```

这里 `map <- odom` 的 x 平移约 `-2.25 m`，所以必须用 `odom <- map` 加回 `2.25 m`，不能直接在
odom 的 `x=-0.398` 查图。对同一位置的观测为：

| 层 | 前缘两列，map x 约 -0.35/-0.30（odom 约 1.90/1.95） | 洞内，map x 约 -0.25..0.35（odom 约 2.00..2.60） |
|---|---|---|
| `layer_value_dynamic` | 100 | 0 = 可通行 |
| `layer_value_static` | 0，即静态投影未否决 | 0 |
| `layer_type` | 100 = OCCUPIED | 66 = PASSABLE |
| `clearance_status` | 结合 ratio 判断 ground/clearance 失败阶段 | 顶棚列 25；有回波地面在新模式中为 3 |

该空间跳变说明拒绝来自洞顶的动态柱分类，而不是 Nav2 static costmap。
进一步随车前进后的逐列探针还显示，水平下表面在某些列只留下一个薄 occupied run；
若直接将“最低薄 run”当地面，它会因高度不连通而误阻塞。所以同一洞顶需要覆盖两种激光离散形态：

- 前缘立面：同列形成较厚的竖直 run；
- 水平下表面：同列可能只有高位薄 run。

当前分类先用最低 run 的保守下边界与 trusted support 高程求净空，再决定它是顶棚还是
地面/墙；不再先按薄厚度猜地面。只有下列条件同时成立才返回
`OVERHEAD_CLEARANCE_OK` 和 `PASSABLE`：

1. 该列静态地图是保守 known-free，且存在测绘高程支撑；
2. 保守下边界与该列 support Z 之差不小于
   `vehicle_height + headroom_margin`；
3. 同一 XY 列中所需车身高度带达到
   `min_observed_overhead_headroom_known_ratio`。RMUC2026 的规则激光仿真允许该值为 0；空列仍必须
   达到 `min_headroom_known_ratio=0.25`，实车两项都为 0.80。

不做跨 XY 列的顶棚“桥接放行”；低于所需净空、同列 body band 观测不足或高程支撑缺失时均
fail-closed。顶棚放行列的诊断组合应为 `layer_type=66`、`headroom_known_ratio`
不低于阈值、`clearance_status=25`；与测绘高程匹配且净空通过的地面列为
`clearance_status=3`。

上面只解决了洞顶分类。引入高程支撑前最后一次 bridge 基线中，目标仍是
`map(3.5,0)`，action 最终 ABORTED，机器人停在约
`map(-0.185,-0.025)`，恢复计数达到 13。典型 first-failure 包括：

```text
COSTMAP_LETHAL: center=(0.457,-0.050), footprint query=(0.658,-0.199)
INSUFFICIENT_CLEARANCE: center=(0.433,-0.050), query=(0.634,-0.199), distance=-0.021
```

这些日志坐标是 planning frame `map`。同一快照换到 ROG `odom` 后，机器人约在 `x=2.065`；
`y=0` 一行的直接地面证据只到 `odom x=0.58`，随后约 `0.58..2.93` 是已观测车身空气但没有地面
回波，坡面候选从约 `2.93` 才重新出现。也就是说 MID360 下视盲区在该时刻造成约 `2.35 m` 的
支撑证据断层。把 bridge 从 `0.45 m` 扩到 `2.35 m` 虽可能让正例运动，却同样无法区分长坑和
真实坡面，因此被否决。当前修复是给仿真地图加入 floor/ramp/platform 的测绘高程，并在
required-support 模式逐列核验；bridge 在 active 中保持关闭。

这一历史数据用于说明根因和安全决策，不代表最终发布验收。最终新配置的单次 live 结果记录在
第 10.2 节；P2/P3/P6 是否通过仍以完整矩阵为准。

按以下顺序定位：

1. 用 first-failure 的 planning point 和 `tf2_echo odom map` 换算 ROG 坐标。
2. 在同一 odom fixed frame 叠加 dynamic、static、fused 和 `layer_type`。
3. 查看 `clearance_status`：0 无 candidate/support，4 是 support-verified empty，25 是顶棚净空，
   100 是候选地面未获授权，50 是地面已验证但净空失败，3 是高程匹配地面，2/1 只属于兼容路径。
4. 对照 `headroom_known_ratio`，区分“body volume 观测不足”与“已知 occupied 净空阻塞”。
5. 沿前缘到洞内逐列查看，确认问题是否只发生在 roof leading edge，而不是整段洞体。

可同时录下这些层，避免顺序 `echo --once` 取到不同快照：

```bash
ros2 bag record --storage mcap --use-sim-time \
  -o "bags/roof_edge_$(date +%Y%m%d_%H%M%S)" \
  --regex '(^/tf$|^/tf_static$|^/clock$|^/rog_map/layer_value_dynamic$|^/rog_map/layer_value_static$|^/rog_map/layer_value$|^/rog_map/layer_type$|^/rog_map/headroom_known_ratio$|^/rog_map/clearance_status$|^/rog_map/headroom$|^/rog_map/field$|^/opt_path_vis$|^/rosout$)'
```

这种症状应修高程地图、TF 或投影分类，并同时回归低浮空横梁和错高斜板负例。禁止通过降低
`min_headroom_known_ratio`、设置负 `collision_dist` 或关闭 unknown-as-occupied 让前缘通过；这些做法会
把真实低梁一起放行。修复后的最低验收是“高位洞顶前缘且所需 body volume 已知为空时连续通过，低于
所需净空的 occupied 前缘仍硬拒绝”。

### 7.3 本轮 `COSTMAP_LETHAL` 与时间链调参法

2026-09-07 的 RMUC2026 长路线复现同时暴露了两类问题：

- lethal 点约为 `map(-1.300, 7.320)`，`source=DYNAMIC_PROJECTION`，本帧 raw 证据却是
  `EMPTY_COLUMN + prior_free + empty_support + clearance_verified`。加入完整诊断后的复现分别给出
  `OBSTACLE_HOLD` 和 `MASK_DENOISE_TO_UNKNOWN`；后一种组合是
  `raw/candidate=PASSABLE, base=OCCUPIED, final=UNKNOWN`，不是先验墙或当前帧实体障碍。
- 一次 CloudFilter 请求 `73.700 s` 时，最新 TF 只有 `73.455 s`。旧实现要求动态 TF 精确覆盖点云
  stamp，而重建 worker 会让 TF/odom 与最终点云错开，且共用 buffer 的阻塞查询存在回调自饿。
  现在 matcher 保存配对 odometry 的 pose 和 parent/child frame，CloudFilter 用该 pose 加静态传感器
  外参直接构造点云时刻变换；`/registered_scan` 与 `/lidar_odometry` stamp 仍严格相同。仿真
  `map -> odom` 也改为初始化一次的静态 TF，消除了固定变换因动态更新时间变旧而外推失败的问题。

最终发布配置将仿真 `obstacle_hold_time` 从 `0.50 s` 改为 `0.0 s`，保留
`hysteresis_count=2`；实车仍保持 `0.50 s/2`。隔离回归从 `map(-11.70,2.90)` 到
`map(3.042,7.586)` 最终 `SUCCEEDED`，`cloud_odom_stamp_delta=0.00 ms`，没有 CloudFilter TF
外推、`STALE_SNAPSHOT` 或 `FUTURE_SNAPSHOT`。整条路线只剩一次
`cost_cause=HYSTERESIS, pending_count=1` 的单帧拒绝，没有旧 hold/mask 连续拒绝。该次还出现 Nav2
`Failed to make progress` 恢复；它会令 planning lease 到期并发布 BLOCK，但不是 ROG lethal，必须
另按 controller/progress checker 调试，不能继续放松 ROG 分类来处理。

先用隔离 ROS domain 回归，防止仿真 `/cmd_vel` 被实车接收：

```bash
cd /home/pnx/nav_ws/sentry-navigation-RM27
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=77
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export GZ_PARTITION=rm27_minco_debug

ros2 launch rm_27_stimulation sim_with_nav.launch.py \
  world:=RMUC2026 navigation_mode:=minco \
  use_ground_truth_odom:=true gui:=true use_rviz:=true
```

另开终端，使用同样三个环境变量，然后执行：

```bash
ros2 topic hz /registered_scan
ros2 topic hz /lidar_odometry
ros2 topic delay /registered_scan --use-sim-time
ros2 run tf2_ros tf2_echo base_link odom

planner_log=$(ls -t ~/.ros/log/planner_server_mt_*.log | head -n 1)
rg 'Trajectory safety rejected|ROGMap CloudFilter|STALE_SNAPSHOT|FUTURE_SNAPSHOT' \
  "$planner_log"
rg -o 'cost_source=[^ ]+ cost_cause=[^ ]+' "$planner_log" | sort | uniq -c
```

RViz 在 `ROG Map` 分组启用 `Projection Reason`、`Fused Traversability`、`Clearance Status`、
`Headroom` 和 `Occupied`。`Projection Reason` 的 OccupancyGrid 值等于 reason enum 乘 8：
0 observation 不足、8 空列、16 当前 footprint 放行、24 薄面、32 墙、40 tunnel、48 ambiguous、
56 ground 未验证、64 headroom 未验证、72 headroom blocked、80 overhead 通过、88 clearance 通过、
96 bridge 通过。

调参按下列顺序，每次只改一组并跑至少 10 次同路线：

1. **先修时间合同。** 当前正常路径不查询 exact-time 动态 TF：点云必须与 odometry 配对，且
   `pose_parent_frame == cloud.header.frame_id`；`pose_child_frame` 必须表示该 pose 的真实物理 child。
   实车因 Point-LIO 的 `body` 与 URDF 的 `imu_link` 命名不同，使用
   `cloud_filter.pose_child_frame=imu_link`。`transform_timeout` 只影响首次静态外参查询和兼容回退，
   增大它不能修复动态 TF future extrapolation。`cloud_odom_sync_tolerance=0.10 s` 是匹配误差，
   不是延迟预算；真值仿真应看到 `cloud_odom_stamp_delta=0.00`，不要用加大 tolerance 掩盖错配。
2. **再修 ROG 吞吐。** `raycasting.parallel_enable=true`、`num_threads=4` 是 16 逻辑核起点；分别试
   2/4/6，比较 detailed CSV 的 P95/P99 `total/raycast/projection`，选择不挤占 controller 的最小总耗时。
   兼容参数 `performance.parallel_raycast_enable` 也应保持同值；代码已规定显式 `raycasting.*` 优先。
3. **最后调致命保持阶段。** 仿真当前不加 wall-clock hold；若动态仿真障碍需要保持，从
   `0.10 s` 起按 `0.05 s` 增加。实车 `0.50 s` 的下限不得短于 bag 验证的连续漏检窗口。
   `HYSTERESIS` 且 raw 已连续可靠可通行时，仿真可对照 `hysteresis_count 2 -> 1`；
   `MASK_HOLE_FILL` 才试 `fill_occ_min 7 -> 8`。`MASK_DENOISE_TO_UNKNOWN` 要回看它之前的
   base hold/hysteresis，不能通过关闭 `unknown_as_occupied` 消除。每一步必须回归真实墙和低梁，
   确认不会出现一格穿透。
4. **最后才评估 watchdog。** 日志近似满足
   `snapshot_age = pipeline_age_ms/1000 + snapshot_commit_age`。若 pipeline 约 0.25 s 而 commit age
   超过 1 s，瓶颈是新快照长期未提交，不是单帧 snapshot 复制。先降计算/可视化负载；只有 P99 已稳定
   且偶发越界才把 `map_timeout` 设为 `P99(snapshot_age)+0.10 s`，并按最大车速制动距离审核。当前仿真
   为 2.20 s、实车 0.50 s，且仍小于 ROG occupied `keep_time=3.00 s`；实车禁止照抄仿真值。

修改位置是 `src/pb2025_nav_bringup/config/simulation/minco_params.yaml` 或
`src/pb2025_nav_bringup/config/reality/minco_params.yaml`。改完重启并确认
生效值；ROG 参数目前按 configure 阶段读取，不建议用 `ros2 param set` 当成永久调参：

```bash
ros2 param get /planner_server MincoPlanner.rog_map.cloud_filter.transform_timeout
ros2 param get /planner_server MincoPlanner.rog_map.cloud_filter.pose_child_frame
ros2 param get /planner_server MincoPlanner.rog_map.raycasting.parallel_enable
ros2 param get /planner_server MincoPlanner.rog_map.raycasting.num_threads
ros2 param get /planner_server MincoPlanner.rog_map.projection.obstacle_hold_time
ros2 param get /planner_server MincoPlanner.safety.map_timeout
```

通过标准：配对 stamp 差为 0（实车应在配置 tolerance 内）、正常路径无 CloudFilter exact-time TF
fallback 或 future/past 错误、
无 `STALE/FUTURE_SNAPSHOT`、同一无障碍格不再重复 `OBSTACLE_HOLD/HYSTERESIS/MASK_HOLE_FILL` 制动，
且墙与低浮空横梁负例仍稳定 `RAW_OCCUPIED` 拒绝。

## 8. 连续动力学极值和 retiming

优化器不会只检查离散采样点。每一段多项式会求速度平方和、加速度平方和导数的实根，并把区间端点一起纳入，从而得到连续时间峰值。

LBFGS 得到轨迹后，若峰值超限，会统一拉伸时间：

```text
scale = clamp(1.02 * max(v_ratio, sqrt(a_ratio)), lower > 1, upper = 4)
```

最多 retime 15 次。出现以下情况直接失败并清空候选：

- 边界 P/V/A 已经超限；
- 极值、系数或时间非有限；
- 15 次仍超限；
- retiming 后任一多项式分段时长超过 `1000 s` 保护上限；
- 最终安全检查失败。

日志重点：

```text
Minco optimization time ... raw cost ... retime_iters N,
peak |v| ..., peak |a| ...
```

解释方式：

- `retime_iters=0-2`：通常正常。
- 经常 5 次以上：时间分配、路径曲率或动力学权重不合理。
- 接近 15：不要继续提高最大迭代次数；先降低速度/加速度目标、增加时间分配或改善搜索路径。
- 离散最终检查还会按 `sample_dt` 检查非有限值、目标和碰撞，并对严重的约 `1.5x` 动力学超限 fail-closed。

规划器的 `max_velocity/max_acceleration/max_yaw_dot` 必须小于等于 MPC/底盘可实现限制。否则 planner 发布“数学可行但底盘不可跟踪”的轨迹。

当前基线：

| 参数 | 仿真 | 实车 |
|---|---:|---:|
| `max_velocity` | 1.00 m/s | 0.50 m/s |
| `max_acceleration` | 1.00 m/s² | 0.80 m/s² |
| `max_yaw_dot` | 1.20 rad/s | 0.80 rad/s |
| MPC `max_planar_speed` | 1.00 m/s | 0.50 m/s |
| 配置 `minco_optimizer.opt_freq` | 20 Hz | 20 Hz |
| time-allocation/retiming 最大迭代数 | 15 | 15 |
| trajectory safety sample dt | 0.05 s | 0.05 s |

## 9. MPC 和最终速度链

### 9.1 轨迹消息

查看接口：

```bash
ros2 interface show ros_interfaces/msg/MpcPositionCommand
ros2 topic echo /minco/opt_path --once --qos-profile sensor_data
ros2 topic echo /minco/backup_path --once --qos-profile sensor_data
```

`command_flag` 关键值：

```text
NORMAL_COMMAND = 1
BLOCK_COMMAND  = 137
```

会话授权使用显式 token：`nav_msgs/Path.header.stamp` 必须等于
`MpcPositionCommand.planning_stamp`，且 token 非零。新的 token、空 Path、完成和 lifecycle
边界会撤销旧轨迹并保持 BLOCK；取消由下述请求租约在有界时间内补齐撤权。匹配的新
NORMAL 到达前不会恢复运动；低于已公告 token 的旧 Path、BLOCK 或
NORMAL 都不能回滚当前会话。

Nav2 Jazzy 不会把 `NavigateToPose` 取消事件回调给已异步返回的 GlobalPlanner
插件。active 因此使用 `request_lease_timeout: 0.75 s` 的 steady-clock 心跳租约：
3 Hz `createPlan()` 刷新当前会话；取消、BT 停止或请求链断后，最迟在租约到期时推进
新 token、清除 pending/缓存轨迹、锁止并发布 BLOCK。它用单调时钟，即使 `/clock`
暂停或回拨也会到期。timer 仍由 ROS executor 调度，所以工程时序是 `0.75 s + executor
调度延迟`，不是硬实时的 0.75 s 截止；CPU 饱和或回调长时间占用时必须把实际延迟纳入
P99 验收。shadow 为无控制权的持续观测 session，显式设为 `0.0` 禁用租约。

`/minco/backup_path` 是 authoritative `/minco/opt_path` 上紧急 BLOCK 的诊断镜像；两个
topic 收到的该消息具有完全相同的 header、planning token、trajectory ID 和内容。
controller 只订阅 opt topic，backup topic 不具有独立控制权。

同一目标在 active 行为树中可按 3 Hz 重新 `createPlan()/setPlan()`。相同 token 的 Path 是
`REFRESHED`，不会反复清轨迹或周期性输出零速。由于 DDS 回调异步，NORMAL 可能先于匹配 Path 到达；
controller 会暂存它但保持零速，Path 授权到达且消息仍新鲜后才提升为当前轨迹。会话 token 与外层
`MpcPositionCommand.header.stamp`/本地接收年龄是两套独立检查；任一过期、future、错 frame、错
token 或非有限值都 fail-closed。

若本轮新 seed、optimizer 或候选轨迹失败，日志可能出现
`New trajectory rejected (...); republished ... after fresh ROG footprint validation`。这是受限的连续性
续发：planner 先用最新物理 pose 和最新 ROG snapshot 复核旧轨迹完整剩余 footprint，再截取 suffix、
重置时间并发布新 ID。它能避免一次优化抖动拖过 controller 的 `trajectory_timeout=1.50 s`；ROG stale、
会话/代次变化、BLOCK latch 或剩余段太短时不会续发，仍然制动。若本轮是碰撞类拒绝，仿真剩余段
还必须不超过 `0.75 s`，实车不超过 `0.40 s`；超限直接发布 emergency BLOCK，不再让长旧轨迹带着坡上
跟踪误差继续运动。调试时把这行与
`TRAJECTORY_RX_STALE` 对照：偶发续发可以接受，持续续发说明优化器或 seed 仍有根因，不能靠增大
controller timeout 隐藏。

实时对照 token：

```bash
ros2 topic echo /plan --field header.stamp --qos-profile sensor_data
ros2 topic echo /minco/opt_path --field planning_stamp --qos-profile sensor_data
ros2 topic echo /minco/opt_path --field command_flag --qos-profile sensor_data
```

### 9.2 `trajectory_id` 与 MPC 参考时间推进

调试时必须分清三个标识：

| 标识 | 作用 | 何时改变 |
|---|---|---|
| `planning_stamp` token | 证明 Path 和 NORMAL/BLOCK 属于同一导航会话 | 新目标、取消、租约过期或 lifecycle 边界 |
| `cmds[*].trajectory_id` | 标识一条具体的 MINCO 离散轨迹 | planner 发布新 NORMAL/escape 轨迹 |
| controller `trajectory_generation` | 防止 QP 求解期间旧结果越代提交 | 接受/清除轨迹或 BLOCK 时，只在进程内部使用 |

`Passing new path to controller` 只说明 Nav2 再次调用了 `setPlan()`。同 token 的 10 Hz Path
会返回 `REFRESHED`，不清轨迹，也不重置参考索引；因此重复出现这行日志不是“controller
不断从轨迹起点重启”的证据。

当前参考进度规则是“单调空间游标 + 有上界的时间推进”：

```text
新 trajectory_id:
  spatial_index = spatial_nearest_index
  reference_index = spatial_index

同一 trajectory_id:
  spatial_index = max(previous_spatial_index, spatial_nearest_index)
  time_progress = previous_reference_index + elapsed / planner_dt
  lead_limit = spatial_index + reference_progress_max_lead_time / planner_dt
  reference_index = clamp(
      max(spatial_index, previous_reference_index,
          min(time_progress, lead_limit)),
      0, last_index)

随后才加 control_delay_compensation，构造 MPC horizon。
```

当前 `planner_dt=0.05 s`、`reference_progress_max_lead_time=0.25 s`，基础参考游标最多领先
单调空间游标 5 个 planner sample。即使小车完全静止、spatial nearest 始终为 0，同一
`trajectory_id` 的参考索引也应先按 20 Hz controller 每周期约前进 1 个 sample，从而越过
零速度起始点，但到达 5 个 sample 的上界后必须等待真实空间进度。新 ID 必须从它自己的空间最近点
接入，不能继承上一条轨迹的时间索引。

这个上界约束的是加延迟补偿**之前的基础游标**；MPC horizon 仍会按
`control_delay_compensation + k * MincoMpc.dt` 查看未来参考，不能把 horizon 最远点误判为游标越界。
旧的无上界 `max(nearest, previous + elapsed/dt)` 会在短停车前缀上把参考跑到终点，而真实车辆仍可能
落后约 `0.17 m`，随后表现为提前零速或 `Failed to make progress`。若调试输出仍呈现这种行为，说明
运行的是旧 install、参数未加载或游标实现回退，本轮不能验收。

先用这个只读探针同时打印 flag、token、outer stamp、长度和该消息中的 ID 集合：

```bash
python3 - <<'PY'
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from ros_interfaces.msg import MpcPositionCommand

rclpy.init()
node = Node('minco_trajectory_id_probe')

def stamp(value):
    return value.sec * 1_000_000_000 + value.nanosec

def callback(msg):
    ids = sorted({cmd.trajectory_id for cmd in msg.cmds})
    print(
        f'flag={msg.command_flag} token_ns={stamp(msg.planning_stamp)} '
        f'outer_ns={stamp(msg.header.stamp)} count={len(msg.cmds)} ids={ids}',
        flush=True)

node.create_subscription(
    MpcPositionCommand, '/minco/opt_path', callback, qos_profile_sensor_data)
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass
finally:
    node.destroy_node()
    rclpy.shutdown()
PY
```

每条非空 NORMAL 的 `ids` 必须只有一个值。在 ID 不变的时段，查看 raw command、参考速度与真值位姿：

```bash
ros2 topic echo /minco/cmd_vel_mpc --qos-profile sensor_data
ros2 topic echo /ground_truth/odometry --field pose.pose.position --csv
ros2 topic hz /cmd_vel_controller
ros2 param get /controller_server MincoMpc.reference_progress_max_lead_time
ros2 param get /controller_server MincoMpc.control_delay_compensation
ros2 param get /planner_server MincoPlanner.minco_optimizer.traj_goal_tolerance
ros2 param get /controller_server general_goal_checker.xy_goal_tolerance

# detailed CSV 已在启动前开启时，它包含 ref_vx/ref_vy/ref_wz
head -n 1 /tmp/rm27_minco_mpc_simulation.csv
tail -f /tmp/rm27_minco_mpc_simulation.csv
```

controller CSV 不含索引和 ID，`ref_v*` 只是时间推进的旁证。它在 plugin `configure()` 时根据
`MincoMpc.performance.detailed_csv_enable` 打开文件；默认是 `false`。启动后才 `ros2 param set`
不会补开文件，必须先在当前 deployment YAML 启用，重建/source 正确 install 并重启导航。

要得到精确索引，只在隔离的仿真域内用 Debug 构建并附加 `controller_server`。设置断点时进程会暂停，watchdog 导致零速是预期行为：

```bash
colcon --log-base log_mpc_ref_debug build --symlink-install \
  --build-base build_mpc_ref_debug \
  --install-base install_mpc_ref_debug \
  --packages-up-to minco_controller pb2025_nav_bringup rm_27_stimulation \
  --cmake-args -DCMAKE_BUILD_TYPE=Debug
source install_mpc_ref_debug/setup.bash
pgrep -af controller_server
gdb -q -p <controller_server_PID>
```

```gdb
set pagination off
break /home/pnx/nav_ws/sentry-navigation-RM27/src/navigation/minco_controller/src/minco_mpc_controller.cpp:972
commands
  silent
  printf "id=%u same=%d nearest=%.3f spatial_prev=%.3f spatial=%.3f ref_prev=%.3f ref=%.3f lead_s=%.3f elapsed=%.6f\n", current_traj_id, same_opt_traj, nearest_idx_float, tracked_spatial_idx, current_spatial_idx, tracked_ref_idx, current_idx_float, (current_idx_float-current_spatial_idx)*planner_dt, elapsed_since_update
  continue
end
continue
```

验收看四件事：同 ID 时 `same=1`，`spatial` 和 `ref` 都单调不减；车不动时 `ref` 先按 elapsed
前进；基础游标的 `lead_s=(ref-spatial)*planner_dt` 始终不大于生效的
`reference_progress_max_lead_time`（只允许浮点误差）；ID 改变后首帧 `same=0` 且
`spatial=ref=nearest`。若每个 controller 周期都换 ID，时间推进会不断重置，应查 planner 为何高频
生成新 NORMAL，不应降低 deadzone 掩盖。

最小回归命令：

```bash
./build/minco_controller/test_reference_progress
# 独立 Debug 构建时则用：
./build_mpc_ref_debug/minco_controller/test_reference_progress
```

回归测试至少应覆盖：静止时能离开 `t=0`、空间最近点抖动时不后退、时间推进被空间上界限制、新 ID
重置，以及离终点尚远时不能仅靠 elapsed 把基础游标推到终点。若零速起始前缀本身长于 `0.25 s`，
先查 planner 为什么生成过长停车段；只有明确测得起步前缀、控制周期抖动和允许跟踪滞后后，才以
`0.05 s` 小步增加该参数，并保持不大于当前 `lookahead_time=0.50 s`。固定空间前视可作为对照实验，
但它会把不同速度下的时间前视混在一起，不是当前默认策略。

如果轨迹首点速度为 0，参考又始终锁在第 0 点，MPC 可持续给出小于
`deadzone_speed_threshold=0.05 m/s` 的速度，下游 smoother 还有 `0.03 m/s` deadband，最终形成“不动 -> 最近点仍是 0 -> 继续不动”的自锁。正确修复是证明同 ID 时间推进；把两层死区都改成 0 只会隐藏问题并引入静止抖动。

反过来，`reference_index==last` 或末端参考速度为 0 也不能单独决定停车。当前还存在两个到点门：
MINCO FSM 的 `traj_goal_tolerance=0.15 m` 可触发清轨迹/BLOCK，Nav2
`general_goal_checker.xy_goal_tolerance=0.20 m` 决定 FollowPath/action 成功。前者必须严格小于后者；
否则车辆可能在 `traj_goal_tolerance` 内、却仍在 Nav2 成功区外时被提前撤权，随后每条新轨迹又立刻
被清掉。当前 `0.15 < 0.20 m` 给 Nav2 留出判定余量。需要更高到点精度时，应先解决跟踪与制动，
再同时审查这两个阈值并重跑坡上停车/取消测试，不能用无上界参考推进、反向阈值或扩大 progress
checker 时间制造“到点”。

### 9.3 MPC 分层探针

```bash
ros2 topic echo /minco/opt_path --once --qos-profile sensor_data
ros2 topic hz /minco/opt_path
ros2 topic echo /odometry --once --qos-profile sensor_data
ros2 topic echo /minco/cmd_vel_mpc --once --qos-profile sensor_data
ros2 topic echo /cmd_vel_controller --once --qos-profile sensor_data
ros2 topic echo /cmd_vel_nav2_result --once --qos-profile sensor_data
ros2 topic echo /cmd_vel --once --qos-profile sensor_data
```

RViz 显示：

```text
/opt_path_vis
/mpc_predict_path
/mpc_real_path
```

`/minco/cmd_vel_mpc` 是 MincoMpc 在成功计算后发布的 raw debug 镜像；ControllerServer 不订阅它，
而是接收插件函数返回的 `TwistStamped` 并发布 `/cmd_vel_controller`。按以下观测关系定位：

1. 有 NORMAL `/minco/opt_path`，没有 `/minco/cmd_vel_mpc`：查 controller lifecycle、odom 和 MPC reject reason。
2. `/minco/cmd_vel_mpc` 有非零 debug、但 `/cmd_vel_controller` 没有对应输出：查 ControllerServer action 状态、取消/停止门和发布端；不要尝试把两个 topic 手工串接。
3. 有 `/cmd_vel_controller`，没有 `/cmd_vel_nav2_result`：查 velocity smoother lifecycle、输入 remap 和限制。
4. 有 `/cmd_vel_nav2_result`，没有 `/cmd_vel`：查 fake_vel_transform 输入、odom freshness 和 frame。
5. 有 `/cmd_vel` 但车不动：问题已到 Gazebo bridge、通信或底盘侧，不应再调 ROG/MINCO。

当前安全门限：

- trajectory receive timeout `1.5 s`；
- odom timeout `0.25 s`；
- future tolerance `0.05 s`；
- odom frame 必须与 MPC global frame 一致；
- fake velocity 使用 latest odom yaw，把全局速度旋转到底盘系；odom 超过约 `0.20 s` 时输出零速。
- active 的 `fake_vel_transform.enable_cmd_spin=false`，因此不会订阅或叠加 `/cmd_spin`；
  legacy/shadow 默认保持原来的启用状态。

常见 MPC fail-closed 原因包括 `NEW_PLAN_WAITING_FOR_TRAJECTORY`、`BLOCK_COMMAND`、`NO_TRAJECTORY`、`NO_ODOMETRY`、接收或 stamp 过期、frame mismatch、`NO_REFERENCE`、`QP_FAILED`、trajectory generation changed。失败时 raw debug topic 也应该看到零速。

### 9.4 取消和陈旧输入测试

运动测试前准备急停。先在两个终端记录会话和最终速度：

```bash
ros2 topic echo /minco/opt_path --qos-profile sensor_data
```

```bash
ros2 topic echo /cmd_vel --qos-profile sensor_data
```

第三个终端发送目标，看到运动后用 `Ctrl+C` 请求取消：

```bash
ros2 action send_goal \
  /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 3.5, y: 0.0}, orientation: {w: 1.0}}}}" \
  --feedback
```

取消后确认：

- Nav2/controller 立即进入停止，`/cmd_vel` 很快稳定为全零；
- 最迟在 `request_lease_timeout=0.75 s` 加一个 timer 调度余量后，`/minco/opt_path`
  出现 `BLOCK_COMMAND=137`；
- 该 BLOCK 的 `planning_stamp` 严格大于被取消会话的 token；
- `/minco/backup_path` 收到完全相同 token/header/trajectory ID 的诊断镜像；
- 再次发送语义相同的目标时，Path 和 NORMAL 使用另一个更大的新 token，不复用
  已取消会话。

单元测试还会暂停仿真 ROS time，确认租约仍按 steady clock 到期。不要用
`goal.header.stamp` 代替这个心跳；该 stamp 常为 0，也不是 Navigate action UUID。

active 入口隔离另做一次静态检查：

```bash
ros2 action list | sort | \
  rg '^/(navigate_through_poses|spin|backup|drive_on_heading|assisted_teleop)$' || true
```

预期无输出。若任一端点存在，不要通过“不去调用它”接受风险；应先修正
active overlay 装配。

再发布一条非零 spin 探针，active 最终速度不得变化：

```bash
ros2 topic pub --once /cmd_spin example_interfaces/msg/Float32 '{data: 1.2}'
timeout 2 ros2 topic echo /cmd_vel --qos-profile sensor_data
```

这项测试只能在静止、无导航目标且仿真或架空轮条件下执行。更强的装配判据仍是
`ros2 topic info -v /cmd_spin` 中不存在 `/fake_vel_transform` 订阅，而不是仅凭某一次零速采样。

断开或暂停 odom、轨迹输入属于侵入性测试，只允许仿真或架空轮测试；预期超时后所有下游速度为零，恢复输入后也不能复用旧 generation 轨迹。

## 10. 洞口加短坡测试矩阵

### 10.1 场景几何

`tunnel_ramp_test` 关键几何：

- 机器人起点约 `(-2.25, 0, 0.08)`。
- 通道墙中心线两侧约 `y=+/-0.8 m`，纵向约 6 m。
- 短洞顶中心约 `x=0`，长度约 `0.7 m`，底面高度约 `0.55 m`。
- 坡道在洞口后约 `0.1 m` 开始，长度约 `1.2 m`，抬升约 `0.25 m`。
- 正向目标 `(3.5, 0)`；反向目标 `(-2.0, 0)`。

“地图上坡道连通”不等于“Gazebo 坡道连续”。同一场景有三份必须同时对齐的几何：

1. world SDF 中的 `collision`，决定轮地接触和车体姿态。
2. world SDF 中的 `visual`，只决定 GUI 看到什么。
3. map YAML 中的 `ground_elevation`，决定 ROG required-support 相信的支撑高程。

三者任一断开都不得进入坡道物理验收。地面平面在洞口到坡底之间连续，所以约
`0.10 m` 的平地段是预期几何；而坡顶与 `z=0.25 m` 平台之间不应露出下方
`z=0` 地面。导航栅格分辨率为 `0.10 m`，可能在格中心采样中隐藏毫米级缝隙，但物理引擎仍会看到它，因而不能只靠 PGM/RViz 判定连续。

先验证 SDF 语法：

```bash
gz sdf -k \
  src/rm_27_stimulation/world/tunnel_ramp_test/tunnel_ramp_test.world
xmllint --noout \
  src/rm_27_stimulation/world/tunnel_ramp_test/tunnel_ramp_test.world
```

再用下面的静态门同时核对坡面上表面端点、平台上表面和两个 high-elevation patch。
脚本中的 `1 mm` join tolerance 和 `2 mm` 对齐 tolerance 只用于斜面角度/小数截断，不是允许
轮子跨越可见缝隙。正式资源应让接触面在该数值精度内接触或微量重叠：

```bash
python3 - <<'PY'
import math
import xml.etree.ElementTree as ET

import yaml

world_path = 'src/rm_27_stimulation/world/tunnel_ramp_test/tunnel_ramp_test.world'
map_path = 'src/pb2025_nav_bringup/map/simulation/tunnel_ramp_test.yaml'
root = ET.parse(world_path).getroot()

def model(name):
    value = root.find(f".//model[@name='{name}']")
    if value is None:
        raise RuntimeError(f'missing model: {name}')
    return value

def numbers(text):
    return [float(value) for value in text.split()]

def box_size(value, kind):
    return numbers(value.findtext(f'./link/{kind}/geometry/box/size'))

ramp = model('short_ramp')
platform = model('ramp_platform')
ramp_pose = numbers(ramp.findtext('pose'))
platform_pose = numbers(platform.findtext('pose'))
ramp_size = box_size(ramp, 'collision')
platform_size = box_size(platform, 'collision')

if ramp_size != box_size(ramp, 'visual'):
    raise SystemExit('FAIL: ramp visual and collision sizes differ')
if platform_size != box_size(platform, 'visual'):
    raise SystemExit('FAIL: platform visual and collision sizes differ')

cx, cz, pitch = ramp_pose[0], ramp_pose[2], ramp_pose[4]
half_length, half_height = 0.5 * ramp_size[0], 0.5 * ramp_size[2]

def ramp_top(local_x):
    world_x = cx + math.cos(pitch) * local_x + math.sin(pitch) * half_height
    world_z = cz - math.sin(pitch) * local_x + math.cos(pitch) * half_height
    return world_x, world_z

ramp_low = ramp_top(-half_length)
ramp_high = ramp_top(half_length)
platform_left = platform_pose[0] - 0.5 * platform_size[0]
platform_top = platform_pose[2] + 0.5 * platform_size[2]
collision_gap = platform_left - ramp_high[0]
collision_dz = platform_top - ramp_high[1]

with open(map_path, encoding='utf-8') as stream:
    config = yaml.safe_load(stream)
patches = {entry['name']: entry for entry in config['ground_elevation']['patches']}
ramp_patch = patches['ramp']
platform_patch = patches['platform']
support_gap = platform_patch['bounds'][0] - ramp_patch['bounds'][2]
ramp_support = lambda x, y=0.0: (
    ramp_patch['reference'][2]
    + ramp_patch['slope'][0] * (x - ramp_patch['reference'][0])
    + ramp_patch['slope'][1] * (y - ramp_patch['reference'][1])
)
ramp_support_low = ramp_support(ramp_patch['bounds'][0])
ramp_support_high = ramp_support(ramp_patch['bounds'][2])
support_dz = platform_patch['reference'][2] - ramp_support_high

print(f'ramp top: low={ramp_low}, high={ramp_high}')
print(f'platform: left={platform_left:.9f}, top={platform_top:.9f}')
print(f'collision join: gap={collision_gap:.9f} m, dz={collision_dz:.9f} m')
print(f'support join: gap={support_gap:.9f} m, dz={support_dz:.9f} m')

failures = []
if abs(ramp_low[1]) > 0.001:
    failures.append('ramp low top does not meet z=0 ground')
if collision_gap > 0.001:
    failures.append('physical ramp/platform collision surfaces have a gap')
if abs(collision_dz) > 0.002:
    failures.append('physical ramp/platform heights do not meet')
if support_gap > 0.0:
    failures.append('ground_elevation ramp/platform patches have a gap')
if abs(support_dz) > 0.002:
    failures.append('ground_elevation ramp/platform heights do not meet')
if abs(ramp_patch['bounds'][0] - ramp_low[0]) > 0.002:
    failures.append('ramp patch start is not aligned with physical ramp')
if abs(ramp_patch['bounds'][2] - ramp_high[0]) > 0.002:
    failures.append('ramp patch end is not aligned with physical ramp')
if abs(ramp_support_low - ramp_low[1]) > 0.002:
    failures.append('ramp patch low height is not aligned with physical ramp')
if abs(ramp_support_high - ramp_high[1]) > 0.002:
    failures.append('ramp patch high height is not aligned with physical ramp')
if abs(platform_patch['bounds'][0] - platform_left) > 0.002:
    failures.append('platform patch start is not aligned with physical platform')
platform_right = platform_pose[0] + 0.5 * platform_size[0]
if abs(platform_patch['bounds'][2] - platform_right) > 0.002:
    failures.append('platform patch end is not aligned with physical platform')
if abs(platform_patch['reference'][2] - platform_top) > 0.002:
    failures.append('platform patch height is not aligned with physical platform')
if failures:
    raise SystemExit('FAIL:\n- ' + '\n- '.join(failures))
print('PASS: collision, visual, and support joins are continuous')
PY
```

修复前 `platform pose.x=3.10` 令物理前缘位于 `x=1.650`，相对
`ramp_high_x=1.641466...` 留下约 `8.5 mm` 缝，YAML 的 `1.641 -> 1.65` 也留下约 `9 mm`
support 缝。当前基线把 platform pose.x 改为 `3.09147`，并把 platform patch 的
`bounds/reference.x` 同步改为 `1.641`（右界 `4.541`）；上述门应输出约 `3.4e-6 m` collision gap、
`0 m` support gap 和最终 `PASS`。若仍得到旧 gap，先查 source/install/world 选择；不得只用更大轮子、
更高摩擦或更宽 support tolerance 跨缝。

### 10.2 正例

每项先 shadow，再 minco；每次保存 bag 和日志。

引入高程支撑前的 bridge 基线已经明确失败：正向 action ABORTED，停车约
`map(-0.185,-0.025)`，根因是约 `2.35 m` 的下视盲区无法证明坡前支撑。该结果不能通过扩大 bridge
处理。required-support 最终 live 结果必须在第 10.1 节几何门通过后按第 10.8 节重新采集，并写入
独立日志/bag；任何一次偶然通过都不能替代 P3/P6 和以下完整发布矩阵。

2026-09-06 在旧 `360 x 720` 传感器基线上留下了两次**单向正例观测**，用于比较 full no-return
与 miss stride；它们是历史数据，不是当前 `360 x 320` 配置的完整发布验收：

| 配置 | no-return 日志样例 | action/真值结果 | 性能与证据边界 |
|---|---|---|---|
| full `1 x 1` | 起点一帧 `synthesized=222087, finite_returns=37113, unsupported=0`，总计 259200 | 单次 `/navigate_to_pose` 为 `SUCCEEDED`，约 `13.291 s`；真值最终 `(3.4134, 0.0111, 0.3100)`，相对 `(3.5,0)` 平面误差约 `0.0873 m`，最大相邻 z 变化约 `0.00208 m`，记录末 2 s 平面速度为 0 | 本轮没有形成可发布的 full 配置速度链采样或 ROG P95/P99 统计，因此不能把这次单次运行当作性能验收 |
| miss stride `1 x 8` | 起点一帧 `synthesized=27617, finite_returns=37113, unsupported=0, stride_skipped=194470`，总计 259200 | 单次 action 为 `SUCCEEDED`，约 `13.088 s`；真值最终 `(3.4088, 0.0129, 0.3100)`，平面误差约 `0.0921 m`，最大相邻 z 变化约 `0.00201 m`，记录末 2 s 平面速度为 0 | 合并日志 45 个 `ROGMapPerf` 窗口中，`map_update_hz` 时间加权均值约 `4.33 Hz`、范围 `1.33-7.00 Hz`；cmd CSV 有 266 个数值样本、末 19 个为零。最低窗口仍需 P99/重复性分析，不能据一次成功扩大 `map_timeout` 或直接发布 |

随后修正了 perf 时间单位，并将重复终点射线与 miss 合并改为 hash 位图：相同 `1 x 8` 输入下，
旧实现约 `total=181-189 ms, merge=123-132 ms`，新实现约
`total=69-80 ms, merge=9-12 ms`。修复后一次 `map(3.5,0)` action 在约 `13.7 s` 返回
`SUCCEEDED`，无 recovery，车体相对起点抬升约 `0.25 m`。这组数字只证明热点已消除和单次正例可达；
仍须按 P1-P6 重新采集正式日志/bag。

本轮临时日志已在交付清理时删除，不属于代码框架。正式 P2-P6 必须按第 10.8 节重新录入带
metadata 的 bag、完整 launch/action/cmd/GT 文件，并复制到版本化验收目录。
目前只能结论为“两个配置各观察到一次单向到达”；反向 P3、坡中停车 P4、取消/stale P5、十轮 P6、
障碍负例和 full 配置性能统计仍未由这两次运行证明。

| 编号 | 测试 | 通过条件 |
|---|---|---|
| P1 | 原地静置 60 s | fused 近场稳定，无自身障碍闪烁，无 stale |
| P2 | 洞前到洞后，正向 | 规划通过洞口，洞顶保持 occupied，车身净空 verified |
| P3 | 洞后回到起点，反向 | 反向也可通过，不依赖单向地面传播偶然性 |
| P4 | 洞后坡上停车再规划 | 坡面/平台 patch 连续，状态 3/4 正确，不把坡顶或浮空薄面当支撑 |
| P5 | 洞中取消 | 规定时间内速度归零，不复用旧轨迹 |
| P6 | 连续十次正反向 | 10/10 无碰撞、无非法穿障、无长时间 retime 饱和 |

记录每次：首规划耗时、重规划次数、first-failure reason、最小 ESDF、retime 次数、速度峰值、跟踪横向误差、停止距离。

### 10.3 负例：静态墙

发送墙内目标，例如：

```bash
ros2 action send_goal \
  /compute_path_to_pose nav2_msgs/action/ComputePathToPose \
  "{goal: {header: {frame_id: map}, pose: {position: {x: 0.0, y: 0.8}, orientation: {w: 1.0}}}, planner_id: MincoPlanner, use_start: false}" \
  --feedback
```

该 ComputePath action 可能因两点占位 Path 而显示 `SUCCEEDED`，不得把它作为负例
判据。预期的真正结果是：无匹配 token 的安全 NORMAL，first-failure 指向静态/融合占据，
发布 BLOCK，车辆全程零速。任何获授权轨迹穿墙都是 release blocker。

### 10.4 负例：低浮空横梁

只在 shadow 或驱动已物理隔离时生成低横梁：

```bash
LOW_GATE='<sdf version="1.7"><model name="low_gate"><static>true</static><link name="link"><collision name="collision"><geometry><box><size>0.20 1.20 0.10</size></box></geometry></collision><visual name="visual"><geometry><box><size>0.20 1.20 0.10</size></box></geometry></visual></link></model></sdf>'
ros2 run ros_gz_sim create \
  -world tunnel_ramp_test \
  -name low_gate \
  -string "$LOW_GATE" \
  -x -0.65 -y 0.0 -z 0.27
```

横梁底面约 `0.22 m`，低于仿真所需约 `0.25 m` 净空。预期：

- `/rog_map/occupied` 可见横梁；
- 相应列净空不通过；
- fused layer 阻塞；
- MINCO first-failure 指向中心或 footprint 查询点；
- shadow：ComputePath action 可因占位 Path 返回 `SUCCEEDED`；验收看
  `/minco_shadow/opt_path` 为 BLOCK、无安全 NORMAL，旧主链不受影响。由于 shadow 没有 MPC，
  不能在这一模式声称“BLOCK 到达 MPC”。
- active minco：BLOCK 被 MincoMpc 接收，所有下游速度为零。

删除测试实体：

```bash
gz service \
  -s /world/tunnel_ramp_test/remove \
  --reqtype gz.msgs.Entity \
  --reptype gz.msgs.Boolean \
  --timeout 2000 \
  --req 'name: "low_gate", type: MODEL'
```

不要在该 world 使用 `ros2 run ros_gz_sim delete_entity --name low_gate`：当前 Jazzy 二进制固定请求
`/world/default/remove`，且没有 world 参数。

### 10.5 负例：错高斜浮板不能伪装成坡

这个用例专门验证“薄、倾斜、接近机器人”的浮空表面不会自播种为地面。只在 shadow、仿真 active
且急停已准备，或底盘输出已隔离时执行：

```bash
FLOATING_PLATE='<sdf version="1.7"><model name="sloped_floating_plate"><static>true</static><link name="link"><pose>0 0 0 0 0.18 0</pose><collision name="collision"><geometry><box><size>0.50 1.00 0.08</size></box></geometry></collision><visual name="visual"><geometry><box><size>0.50 1.00 0.08</size></box></geometry></visual></link></model></sdf>'
ros2 run ros_gz_sim create \
  -world tunnel_ramp_test \
  -name sloped_floating_plate \
  -string "$FLOATING_PLATE" \
  -x -1.0 -y 0.0 -z 0.18
```

发送正向目标并同步观察：

```bash
ros2 topic echo /rog_map/clearance_status --qos-profile sensor_data
ros2 topic echo /rog_map/layer_value --qos-profile sensor_data
ros2 topic echo /minco/opt_path --field command_flag --qos-profile sensor_data
ros2 topic echo /cmd_vel --qos-profile sensor_data
```

预期板体出现在 `/rog_map/raw_occupied` 和 `/rog_map/occupied`，其最低薄面与 floor support 的高度差
超过 `0.08 m`，相关候选通常为 `clearance_status=100`，fused layer 阻塞且没有获授权的 NORMAL
穿过板体。即使具体柱被分类为低净空而不是 `100`，最终也必须阻塞；不能把某一个诊断码写成唯一
通过条件。

删除实体：

```bash
gz service \
  -s /world/tunnel_ramp_test/remove \
  --reqtype gz.msgs.Entity \
  --reptype gz.msgs.Boolean \
  --timeout 2000 \
  --req 'name: "sloped_floating_plate", type: MODEL'
```

对应投影与 prior 单元测试可单独执行：

```bash
./build/rog_map/test_projection_clearance \
  --gtest_filter='GroundSupportProjection.*:PriorMapLoader.GroundElevationAppliesMapToRogZTranslation:PriorMapLoader.RejectsMalformedGroundElevation:PriorMapLoader.SlidingProjectionKeepsGroundSupportAligned:PriorMapFusion.RequiredSupportDisablesEveryTwoDimensionalFreeFill'
```

当前完整 `rog_map` gtest 共 `77 tests, 0 failures`。其中 `GroundSupportProjection.*` 覆盖孤立
浮板不可自播种、无支撑空坑闭锁、正确坡面、垂直台阶边界、错高浮板、静态占据否决、低动态障碍
否决、局部支撑净空、关闭 clearance 也不得绕过 required gate，以及 legacy fallback 不得绕过。
正式验收仍要执行不带 filter 的完整测试二进制和
`colcon test-result --verbose`；单元测试不能替代 Gazebo 与实车观测。

### 10.6 负例边界：新坑不是当前传感器可证明的项目

地面高程先验能阻止“错高度薄板被当作地面”，但不能独立发现测绘后新出现且完全落在 MID360 下视
盲区内的坑。若 PGM 仍是 known-free、YAML 仍声明 support、车身上方射线又显示为空，当前系统可能
产生 `clearance_status=4`。因此：

- 单元测试必须证明“先验缺少 support 的空列”和“未知列”会阻塞；
- 仿真坑测试必须使用真正移除支撑面的专用 world，不能只在无限 `ground_plane` 上盖黑色贴图；
- 现场有跌落风险时，release 条件必须增加下视传感器的实时支撑否决测试；
- 在下视链路完成前，把坑、台阶边缘及可能变化区域在 PGM 中标为 unknown/occupied，并禁止驶入；
- 不能把“静态 support prior 单测通过”写成“能实时识别新坑”。

### 10.7 负例验收原则

以下任何一项出现即停止调速度并回到感知层：

- 低梁或小箱体在 raw cloud 可见但 fused layer free；
- 错高斜浮板得到 status 3/4，或静态墙被 support/prior 释放；
- 缺少 `ground_elevation`、support TF 非水平/过期时仍发布 NORMAL；
- TF/地图过期时仍发布 NORMAL；
- 取消目标后继续输出非零速度；
- 新 plan 到来后 controller 继续跟踪旧 generation；
- footprint 角点碰撞但中心线显示 free 时仍通过。

### 10.8 一次完整的隔离仿真验收

下面是一套可直接复制执行的 active 正例流程。先完成第 10.1 节静态几何门和第 2.3 节完整单元测试；任一失败时不启动自动运动。下列终端必须使用同一
`ROS_DOMAIN_ID` 和 `GZ_PARTITION`。

终端 A：启动独立进程、关闭 respawn 并保存完整日志：

```bash
cd /home/pnx/nav_ws/sentry-navigation-RM27
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=153
export GZ_PARTITION=rm27_tunnel_153

ros2 launch rm_27_stimulation sim_with_nav.launch.py \
  world:=tunnel_ramp_test nav_world:=auto \
  use_ground_truth_odom:=true navigation_mode:=minco \
  enable_legacy_terrain:=auto \
  use_composition:=false use_respawn:=false \
  gui:=true use_rviz:=true \
  2>&1 | tee /tmp/rm27_tunnel_acceptance.log
```

终端 B：启动后立即录制原始真值、地图快照、轨迹和全速度链：

```bash
cd /home/pnx/nav_ws/sentry-navigation-RM27
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=153
export GZ_PARTITION=rm27_tunnel_153
mkdir -p bags

ros2 bag record --storage mcap --use-sim-time \
  --compression-mode file --compression-format zstd \
  --include-hidden-topics \
  -o "bags/tunnel_acceptance_$(date +%Y%m%d_%H%M%S)" \
  --regex '(^/clock$|^/tf$|^/tf_static$|^/map$|^/ground_truth/odometry$|^/odometry$|^/lidar_odometry$|^/registered_scan$|^/rog_map/.*|^/plan$|^/minco/.*|^/opt_path_vis$|^/minco_candidate_path_vis$|^/astar_path_vis$|^/mpc_predict_path$|^/mpc_real_path$|^/cmd_vel_controller$|^/cmd_vel_nav2_result$|^/cmd_vel$|^/rosout$|.*/_action/.*)'
```

终端 C：发目标前做装配与数据健康检查：

```bash
cd /home/pnx/nav_ws/sentry-navigation-RM27
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=153
export GZ_PARTITION=rm27_tunnel_153

ros2 lifecycle get /planner_server
ros2 lifecycle get /controller_server
ros2 lifecycle get /bt_navigator
ros2 lifecycle get /velocity_smoother
ros2 topic echo /clock --once
ros2 topic echo /ground_truth/odometry --once --qos-profile sensor_data
ros2 topic echo /registered_scan --once --qos-profile sensor_data --no-arr
ros2 topic echo /rog_map/layer_value --once --qos-profile sensor_data --no-arr
ros2 param get /planner_server MincoPlanner.rog_map.cloud_filter.filter_mode
ros2 param get /planner_server MincoPlanner.rog_map.cloud_filter.box_size.y
ros2 param get /planner_server MincoPlanner.frames.physical_base_frame
ros2 param get /planner_server MincoPlanner.safety.map_timeout
ros2 param get /planner_server MincoPlanner.safety.collision_cache_reuse_max_duration
ros2 param get /controller_server MincoMpc.max_planar_speed
ros2 param get /controller_server MincoMpc.deadzone_speed_threshold
ros2 param get /controller_server MincoMpc.reference_progress_max_lead_time
ros2 param get /planner_server MincoPlanner.minco_optimizer.traj_goal_tolerance
ros2 param get /controller_server general_goal_checker.xy_goal_tolerance
ros2 param get /global_costmap/global_costmap robot_base_frame
ros2 param get /planner_server MincoPlanner.rog_map.cloud_filter.position_frame
ros2 param get /fake_vel_transform use_latest_odom_for_cmd
```

预期 `filter_mode=transform_cloud`、仿真 `box_size.y=0.28`、`physical_base_frame=base_link`、`map_timeout=2.20`、
`collision_cache_reuse_max_duration=0.75`、`max_planar_speed=1.0`、`reference_progress_max_lead_time=0.25`、
`traj_goal_tolerance=0.15 < xy_goal_tolerance=0.20`，所有 lifecycle 节点 active。再用 RViz 固定在 `odom`，显示 robot model、`/rog_map/raw_occupied`、
`/rog_map/occupied` 和 `/rog_map/self_filter_box`；原地 yaw 变化及上坡 roll/pitch 期间，marker 必须和
`base_link` 一起旋转。

原地慢速转向时还应并行运行 `tf2_echo map base_link` 和
`tf2_echo map gimbal_yaw_fake`。前者 yaw 必须随车体变化；后者是 Nav2 的合成平面 frame，可保持
接近固定 yaw。此时 MINCO 当前 footprint 日志必须使用前者的 yaw，详见第 6.1 节。

终端 D：记录物理真值和定位输出。第一条是 Gazebo 原始 3D odometry，第二条直接显示
`odom -> base_footprint` 的 roll/pitch/yaw；可分别在两个终端运行：

```bash
export ROS_DOMAIN_ID=153
timeout 180 ros2 topic echo /ground_truth/odometry --csv \
  > /tmp/rm27_ground_truth_3d.csv
```

```bash
export ROS_DOMAIN_ID=153
timeout 180 ros2 run tf2_ros tf2_echo odom base_footprint \
  > /tmp/rm27_odom_base_3d.txt
```

终端 E：在 `/cmd_vel` 类型已经可发现后记录最终底盘命令。先检查类型，避免得到一个只含
`topic does not appear to be published yet` 的无效“CSV”：

```bash
export ROS_DOMAIN_ID=153
ros2 topic type /cmd_vel
ros2 topic hz /cmd_vel
timeout 180 ros2 topic echo /cmd_vel --csv \
  > /tmp/rm27_cmd_vel.csv
```

类型必须为 `geometry_msgs/msg/Twist`。`timeout` 正常到期返回 124；CSV 内必须存在数值行，warning
文本不算样本。GT、cmd 和下述 action 三个录制器都应在发目标前启动。

正向目标：

```bash
export ROS_DOMAIN_ID=153
ros2 action send_goal \
  /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 3.5, y: 0.0}, orientation: {w: 1.0}}}}" \
  --feedback | tee /tmp/rm27_goal_forward.log
```

到达平台后先保持静止，保存当前位姿，再在平台上发一个短距离目标验证“坡上停车后重规划”：

```bash
ros2 topic echo /ground_truth/odometry --once --qos-profile sensor_data
ros2 action send_goal \
  /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 2.5, y: 0.0}, orientation: {w: 1.0}}}}" \
  --feedback | tee /tmp/rm27_goal_platform.log
```

反向回到起点附近：

```bash
ros2 action send_goal \
  /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: -2.0, y: 0.0}, orientation: {w: 1.0}}}}" \
  --feedback | tee /tmp/rm27_goal_reverse.log
```

不要用无人值守的 shell loop 直接完成 P6。每一轮都重复上述正向/反向命令，并在下一轮前确认无
`STALE_SNAPSHOT`、无自身 `COSTMAP_LETHAL`、无 `Failed to make progress`、最终速度已稳定归零。

一次正例后执行陈旧数据负例。只暂停**精确核对过的** ground-truth localizer PID，
Gazebo 和 `/clock` 保持运行；不得对模糊 `pgrep` 结果直接执行 `kill`：

```bash
pgrep -af rm27_ground_truth_localizer
GT_LOCALIZER_PID=<exact_numeric_pid>
kill -STOP "$GT_LOCALIZER_PID"
sleep 2
ros2 topic echo /cmd_vel --once --qos-profile sensor_data
kill -CONT "$GT_LOCALIZER_PID"
```

预期 MPC 先因 `odom_timeout=0.25 s` 零速，planner 随后因地图 age 超过
`2.20 s` 发布 BLOCK；全程 `/ground_truth/odometry` 仍由 Gazebo bridge 更新且车不动。恢复后必须由新鲜输入和匹配的安全 NORMAL 重新授权，不能继续执行旧 generation。若 launch
终端已退出，先确认 PID 仍存在再 `CONT`；测试结束不得遗留 stopped 进程。

再分别执行第 9.4 节取消、第 10.3 节静态墙、第 10.4 节低梁和第 10.5 节错高斜浮板。
每个负例都要单独记录日志/bag，不在同一轮里叠加多个障碍后猜测首失败来源。

一轮结束后用 `Ctrl-C` 正常停止 bag，再执行：

```bash
ros2 bag info bags/<tunnel_acceptance_bag>

rg -n \
  'Trajectory safety rejected|STALE_SNAPSHOT|COSTMAP_LETHAL|BLOCK_COMMAND|NO_ODOMETRY|ODOMETRY_STAMP_STALE|TRAJECTORY_CHANGED_DURING_SOLVE|Failed to make progress|Goal succeeded|Goal failed|Goal was canceled' \
  /tmp/rm27_tunnel_acceptance.log \
  /tmp/rm27_goal_forward.log \
  /tmp/rm27_goal_platform.log \
  /tmp/rm27_goal_reverse.log

rg -n 'ROGMapPerf|Passing new path|Minco MPC fail-closed' \
  /tmp/rm27_tunnel_acceptance.log | tail -n 200
```

先验收 action 本身；两项必须同时存在：

```bash
if rg -q 'error_code: 0' /tmp/rm27_goal_forward.log && \
   rg -q 'Goal finished with status: SUCCEEDED' /tmp/rm27_goal_forward.log; then
  echo 'ACTION_RESULT_OK'
else
  echo 'ACTION_RESULT_FAIL_OR_INCOMPLETE'
  exit 1
fi
```

再对当前 ROS 2 Jazzy `nav_msgs/msg/Odometry --csv` 字段顺序运行下面的 kinematic gate。这里使用
位置列 5-7、平面速度列 48-49；升级 ROS 后先用 `ros2 interface show nav_msgs/msg/Odometry` 和一行
CSV 重新核对列号：

```bash
awk -F, '
function abs(v) { return v < 0 ? -v : v }
NF >= 53 && $1 ~ /^[0-9]+$/ {
  n++
  t[n] = $1 + $2 / 1e9
  x[n] = $5; y[n] = $6; z[n] = $7
  speed[n] = sqrt($48 * $48 + $49 * $49)
  if (n == 1 || x[n] > max_x) max_x = x[n]
  if (n == 1 || z[n] > max_z) max_z = z[n]
  if (n > 1 && abs(z[n] - z[n-1]) > max_dz) max_dz = abs(z[n] - z[n-1])
}
END {
  if (n < 2) {
    print "GT_KINEMATIC_FAIL: no numeric odometry samples"
    exit 2
  }
  terminal_speed = 0
  for (i = 1; i <= n; i++)
    if (t[i] >= t[n] - 2.0 && speed[i] > terminal_speed)
      terminal_speed = speed[i]
  goal_error = sqrt((x[n] - 3.5)^2 + y[n]^2)
  crossed_ramp = (max_x >= 1.70 && max_z >= 0.28)
  pass = crossed_ramp && goal_error <= 0.20 && \
         terminal_speed <= 0.02 && max_dz <= 0.03
  printf("GT samples=%d end=(%.4f,%.4f,%.4f) goal_error=%.4f " \
         "terminal_2s_vmax=%.5f max_dz=%.5f crossed_ramp=%d => %s\n", \
         n, x[n], y[n], z[n], goal_error, terminal_speed, max_dz, \
         crossed_ramp, pass ? "GT_KINEMATIC_OK" : "GT_KINEMATIC_FAIL")
  exit(pass ? 0 : 1)
}' /tmp/rm27_ground_truth_3d.csv
```

这组阈值对应当前目标容差和场景几何，只检查“到达、跨上平台、末端停稳、没有大 z 跳变”；它不能证明
没有擦碰、没有穿障或 ROG 负例正确。修改目标或场景后必须同步修改判据，不能沿用到实车。

最终 Twist 没有 header，当前 CSV 每个数值行依次是 `linear xyz, angular xyz`。下面要求至少 20 个
数值样本，且末尾至少连续 10 个样本为零；同时打印峰值供调参，不把峰值自动当作通过：

```bash
awk -F, '
NF >= 6 && $1 ~ /^[-+0-9.eE]+$/ {
  n++
  planar = sqrt($1 * $1 + $2 * $2)
  yaw = $6 < 0 ? -$6 : $6
  if (planar > max_planar) max_planar = planar
  if (yaw > max_yaw) max_yaw = yaw
  if (planar > 0.001 || yaw > 0.001) last_nonzero = n
}
END {
  trailing_zero = n - last_nonzero
  pass = (n >= 20 && trailing_zero >= 10)
  printf("CMD samples=%d max_planar=%.4f max_yaw=%.4f trailing_zero=%d => %s\n", \
         n, max_planar, max_yaw, trailing_zero, \
         pass ? "CMD_STOP_OK" : "CMD_STOP_FAIL_OR_INCOMPLETE")
  exit(pass ? 0 : 1)
}' /tmp/rm27_cmd_vel.csv
```

若文件只有 warning 或零个样本，该命令必须失败；不能用 GT 已停住替代“最终控制链明确发过零速”。

最后从同一 launch log 汇总 no-return 与 ROG 更新窗口：

```bash
rg -n 'No-return rays|reconstruction rejected|Dropped lidar scans' \
  /tmp/rm27_tunnel_acceptance.log

awk '
/ROGMapPerf/ {
  window = 0; hz = 0
  for (i = 1; i <= NF; i++) {
    if ($i ~ /^win=/) { split($i, a, "="); window = a[2] + 0 }
    if ($i ~ /^map_update_hz=/) { split($i, b, "="); hz = b[2] + 0 }
  }
  n++; sum += hz; weighted += hz * window; total_window += window
  if (n == 1 || hz < min_hz) min_hz = hz
  if (n == 1 || hz > max_hz) max_hz = hz
}
END {
  if (n == 0) { print "ROG_PERF_MISSING"; exit 2 }
  printf("ROG windows=%d mean=%.3f weighted_mean=%.3f min=%.3f max=%.3f\n", \
         n, sum/n, weighted/total_window, min_hz, max_hz)
}' /tmp/rm27_tunnel_acceptance.log
```

均值不能掩盖最低窗口和 P99；还要结合 detailed CSV、输入 `hz/delay`、CPU 和
`STALE_SNAPSHOT/FUTURE_SNAPSHOT`。若合并日志没有 perf 行，本轮只能判为性能证据不完整。

最终判定不能只看 action result：

| 项目 | 通过条件 |
|---|---|
| 静态几何 | 第 10.1 节脚本 PASS；collision/visual/support 在坡底和坡顶都连续 |
| 物理运动 | `/ground_truth/odometry` 连续；坡段总抬升约 `0.25 m`，坡面名义倾角约 `0.205 rad`；无瞬时 z 台阶、自由落体或传送 |
| 自滤除 | marker 跟随车体 yaw/roll/pitch；车轮外沿无持续自点，车体外障碍仍可见 |
| 当前足迹姿态 | 转向时 MINCO 当前 footprint yaw 与 `map <- base_link` 一致，不使用 yaw-cancelled `gimbal_yaw_fake` 姿态；TF 缺失时闭锁 |
| 正向/反向 | P2/P3 action 成功且真值到达，无碰撞、无越界、无 positive-run safety reject |
| 坡上重规划 | P4 从真实停稳位姿生成新 ID，不继承上一 ID 的参考时间 |
| MPC 起步/跟踪 | 同 ID 基础参考在车静止时能离开 t0，且相对单调空间游标领先不超过 `0.25 s`；raw/final cmd 越过合理死区，真值位置响应 |
| 停车 | 以实际 odom 和 goal checker 判定，不以参考先到终点代替；终点无提前零速/进度 abort |
| 取消/陈旧 | P5 和暂停 localizer 都在规定 watchdog/租约边界内归零、BLOCK，不复用旧轨迹 |
| 重复性 | P6 正反向 `10/10`，每轮独立保存 result、first-failure 分布和性能数据 |
| 障碍负例 | 静态墙、低梁、错高浮板全部 fail-closed，车辆零速 |

当前 Gazebo odometry 为 50 Hz，最大平面速度为 `1.0 m/s`，坡度约 `0.2085`，理想坡面每帧高度变化约只有 `0.0042 m`。实际阈值要包含物理步进、轮子几何和发布抖动，但不应出现接近整个 `0.25 m` 抬升的单帧跳变。对托底、轮地接触力和摩擦余量有严格要求时，还必须增加 contact/force 传感器和独立机械验算；仅凭 odometry 连续不能证明这三项。

## 11. 调参顺序、范围与停止条件

### 11.1 固定顺序

只按以下顺序推进：

```text
传感器是否能观测目标风险（尤其新坑）
  -> 物理尺寸/外参
  -> 时钟、topic、TF
  -> 自滤除
  -> 三维占据
  -> 二维地图 known-free + 地面高程测绘
  -> 地面支撑匹配与净空
  -> fused field 与轨迹安全
  -> MINCO 动力学和时间分配
  -> MPC 跟踪
  -> velocity smoother/最终 cmd
```

如果传感器根本看不到一种危险，先加传感器或限制运行域，不能进入参数优化。每轮只修改一个参数组，
用同一个 bag 或同一初始状态重复至少 5 次；release 统计使用独立数据，不把调参集当验收集。记录
commit/diff、改前值、改后值、理论理由、预期、实测分位数、负例结果和是否回退。

### 11.2 感知与投影

下表 `cloud_filter/raycasting/projection/decay` 键均相对
`MincoPlanner.rog_map`；写入 YAML 或 `ros2 param get` 时要带完整前缀。表中明确写为
`rm27_ground_truth_localizer.*` 的两行是 Gazebo localizer 参数，不属于 ROG，也禁止用于实车。

| 参数组 | 当前/起点 | 步长 | 停止条件 |
|---|---:|---:|---|
| `cloud_filter.filter_mode` | 仿真/实车均为 `transform_cloud` | 不作数值调参 | marker 不跟随车体姿态时先修 TF/stamp |
| `cloud_filter.pose_child_frame` | 仿真留空取 odom child；实车 `imu_link` | 不作数值调参 | 必须与 odometry pose 的真实物理 child 一致 |
| `cloud_filter.box_size` | 仿真 `[0.34,0.28,0.18]`；实车 `[0.34,0.24,0.14]` m | 按 URDF/CAD/静态扫描重新测量 | 车体外障碍被删除立即回退 |
| `cloud_filter.z_offset` | 仿真 -0.45，实车 -0.55 m | 0.01 m，最多基线上下 0.02 | 地面仍系统偏差时回查外参 |
| `cloud_filter.box_padding` | 0.02 m | 0.01，范围 0-0.04 | 洞顶/墙/坡被滤掉立即回退 |
| `global_costmap.inflation_layer.inflation_radius/cost_scaling_factor` | 仿真 MINCO `0.35 m / 5.0`；实车继承现场 profile | 每次 `0.05 m / 1.0`，先只调仿真 | 洞口变得无全局路径，或路径仍擦边时停止并核对 PGM/STL |
| `raycasting.p_hit` | 0.90 | 0.01-0.02，范围 0.86-0.94 | 孤立噪声固化 |
| `raycasting.p_miss` | 0.45 | 0.01，范围 0.40-0.48 | 真实障碍被过早清空 |
| `raycasting.p_occ` | 0.85 | 0.01-0.02，范围 0.80-0.90 | 墙体闪烁或小障碍漏检 |
| `rm27_ground_truth_localizer.no_return_horizontal_stride/vertical_stride` | 仿真部署 `1/2`；语义基线 `1/1` | 语义通过后只逐级增加 miss stride | finite hit 计数变化、known-ratio 不足或负例漏检立即回退 |
| `rm27_ground_truth_localizer.no_return_ray_length/rog_raycast_max_range` | `10.5/10.0 m` | 固定；且 `length >= max + 2*0.05 m` | 相等、非有限或不足两个 ROG 体素裕量禁止启动 |
| `decay.keep_time/clear_time` | 3.00/5.00 s | 先固定 | 移动物体残影超预算或静态占据闪烁 |
| `projection.min_observed_voxels` | 2 | 1，范围 2-4 | 禁止以 1 发布 |
| `projection.prior_map.ground_support_tolerance` | 0.08 m | 0.01 m | 错高薄板被授权，立即回退 |
| `projection.max_ground_step` | 0.06 m | 0.01 m | 危险台阶通过，或合法 patch 边界闭锁 |
| `projection.max_ground_slope_deg` | 28 deg | 1 deg | 只按车辆坡度能力和测绘误差设定 |
| `projection.obstacle_hold_time` | 仿真 0.0 s；实车 0.50 s | 仿真动态障碍从 0.10 起；实车按 bag 漏检上界 | 移动障碍产生长残影或实车漏检窗口未覆盖 |
| `projection.hysteresis_count` | 2 帧 | 1 帧步进，范围 1-4 | 放行延迟超过制动预算或障碍闪烁 |

required-support=true 时，`ground_seed_tolerance/ground_seed_radius` 服务于兼容 BFS，不是当前坡道
旋钮。`max_ground_step/max_ground_slope_deg` 则仍校验八邻域 support 连续性：先把坡道高度和斜率
准确写进 map YAML，再确认邻格差满足几何门；地面回波相对测绘面的误差由
`ground_support_tolerance` 单独控制。

### 11.3 地面高程、净空和盲区

| 参数 | 仿真策略 | 实车策略 |
|---|---|---|
| `projection.prior_map.require_ground_support` | 必须 true | 必须 true；无高程时保持闭锁 |
| `projection.prior_map.ground_support_tolerance` | 0.08 m 起 | 由测绘/TF/点云误差预算，0.01 m 小步验证 |
| `projection.min_headroom_known_ratio` | 0.25；约束空列，不应降为 0 | 0.80，通常保持 0.75-0.90 |
| `projection.min_observed_overhead_headroom_known_ratio` | 0.0；仅限已测顶板列 | 0.80，不得照抄仿真值 |
| `projection.headroom_voxel_inset_fraction` | 0.0；规则仿真射线按 voxel center | 0.5；按 occupied voxel 边界保守估计 |
| `projection.prior_map.free_fills_unknown` | 保持 false | 必须为 false |
| `projection.observed_empty_as_free` | 保持 false | 必须为 false |
| `projection.bridge_observed_empty_for_ground_connectivity` | 保持 false | 必须为 false |
| `projection.clear_robot_footprint_unknown` | true；只允许严格 current-footprint bootstrap | true；测绘支撑完成前仍会闭锁 |
| `projection.near_field_prior_fill_enable` | RMUC2026 仿真为 true，限严格支撑的 `1.40 x 1.00 m` 短扫掠区 | 必须为 false |

先从数据估计支撑容差，而不是反复增大直到能走：

```text
e_support = e_survey_P99 + e_tf_z_P99 + e_cloud_z_P99 + e_voxel_quantization
要求：ground_support_tolerance >= e_support
同时：ground_support_tolerance < h_min_dangerous_false_surface - e_support
```

若两个不等式没有共同区间，说明当前地图/定位/传感器精度无法可靠区分地面与危险薄面；停止调参并
提高测绘或感知质量。对每个 patch 至少分别统计平地、坡脚、坡中、坡顶和横向边缘的
`ground_z - support_z`。只有分布存在一致偏置时才修 reference/z offset；若误差随 x 线性增长，
应修 slope；若只在 frame 切换时跳变，应修 TF，不能用 tolerance 一次吞掉。

patch 边界还必须满足八邻域连续性。以 `0.05 m` 投影分辨率、`max_ground_step=0.06 m`、
`max_ground_slope_deg=28 deg` 为例，轴向邻格允许差为 `max(0.06,tan(28 deg)*0.05)=0.06 m`；
不要用一个从 0 直接跳到 0.25 m 的平台 patch 代替真实坡面。若真实环境确有垂直台阶，它本来就应
形成阻塞边界，而不是靠扩大 `max_ground_step` 放行。

净空比例停止条件：

- 正例无法通过但已知比例低：先改善视角、射线覆盖和传感器安装，不先降低阈值。
- 任一低梁负例被放行：立即恢复更严格值，停止 release。
- 仿真 0.25 当前只适用于已验证的 360x320 规则射线，任何分辨率或传感器模型变化都要重做统计。
- 实车降到 0.75 也需要至少 30 次正例、全部负例和人工审查，绝不允许直接设 0.25。
- `status=100` 且候选 Z 与 support Z 有固定偏差：先修高程/TF/z offset，不改 known ratio。
- `status=0`：先查 PGM known-free、ground_elevation 与 TF；它不是 MPC 问题。
- `status=4`：只证明静态先验支撑和上方空闲证据，不证明测绘后没有出现新坑。

`headroom_voxel_inset_fraction` 只补偿 occupied voxel 表面量化，范围固定为 `[0, 0.5]`：`0` 使用
voxel center，`0.5` 使用最保守的上下边界。它不是洞口通行开关。实车只有在标定板/洞顶 bag 能证明
回波相对真实表面的单侧误差分布，并且机械高度、姿态误差和制动振动余量已单独计入
`vehicle_height + headroom_margin` 后，才可按 `0.05` 逐步减小；每一步都必须回归最低允许洞顶与低梁
负例。没有这组测量时保持 `0.5`。

### 11.4 field 和轨迹安全

| 参数 | 当前/范围 | 建议 |
|---|---:|---|
| `rog_map.field.inflation_radius` | 0 | 先保持 0，避免重复膨胀 |
| `minco_optimizer.collision_dist` | 0 | 先保持 0，occupied/unknown 仍硬拒绝 |
| `minco_optimizer.safe_dist` | 0.30 m | 以 0.05 m 小步调整软代价 |
| `safety.footprint_margin` | 0.05 m | 建议 0.05-0.10 m，必须来自外形/定位误差预算 |
| `corridor.extra_margin` | 0.05 m | 建议 0.05-0.10 m |
| `safety.sample_dt` | 0.05 s | 高速或高曲率只能减到 0.02，不可增大避开碰撞 |
| `lookahead_dist` | 1.50 m | 可在 1.00-2.00 m 内验证，但必须覆盖制动距离、完整 footprint，并留在可观测地图内 |
| `safety.collision_cache_reuse_max_duration` | 仿真 0.75 s；实车 0.40 s | 只能缩短；不要增大来掩盖碰撞类重规划失败 |

修改前计算最小制动距离：

```text
d_stop = v_max^2 / (2 * a_brake) + v_max * total_latency + position_error
```

局部可观测距离、轨迹 horizon 和已验证 free 区域必须大于 `d_stop` 加车身前悬和裕量。否则降低速度，不要放宽 unknown 或 stale 检查。

### 11.5 MINCO

当前仿真 `max_velocity=1.00 m/s`，实车保持 `0.50 m/s`；仿真加速度/角速度为
`1.0 m/s²`、`1.2 rad/s`，实车为 `0.8 m/s²`、`0.8 rad/s`。仿真速度提高不代表实车已经
完成同速放行；实车参数仍按下面的步骤独立验证。
实车 active 首次应在 YAML 中降到 `0.20 m/s`，重启后用参数 dump 证明生效：

```bash
ros2 param get /planner_server MincoPlanner.minco_optimizer.max_velocity
ros2 param get /planner_server MincoPlanner.minco_optimizer.max_acceleration
ros2 param get /planner_server MincoPlanner.minco_optimizer.max_yaw_dot
ros2 param get /planner_server MincoPlanner.minco_optimizer.time_allocation_iters
ros2 param get /controller_server MincoMpc.max_planar_speed
```

| 参数 | 调整方式 | 停止条件 |
|---|---|---|
| `max_velocity` | 0.20 起，每次 +0.10，最高不超过实车基线 0.50 | 制动距离/视距不足、跟踪误差增大 |
| `max_acceleration` | 0.30 起，每次 +0.10，最高不超过 0.80 | 轮滑、姿态扰动、MPC 饱和 |
| `max_yaw_dot` | 每次 +0.10，最高不超过 0.80 | 洞口横向误差或角速度饱和 |
| `traj_goal_tolerance` | 0.15 m；必须 `< xy_goal_tolerance(0.20 m)` | FSM 提前 BLOCK 或 Nav2 到点环振荡 |
| `time_allocation_iters` | 10-20，保持 15 优先 | 经常 retime 到上限时先查路径/限制 |
| `integral_res` | 16-24 | CPU 超预算或收益不明显 |
| 代价权重 | 每次乘 1.5 或除 1.5 | 禁止一次跨数量级且多项同调 |

推荐单变量循环：

1. 固定 ROG bag/地图/起终点，确认 first-failure 已经不在感知层。
2. 先看搜索 seed 和 corridor；它们穿障或贴边时，不调 MINCO 权重。
3. 保持 `safe_dist=0.30`、`collision_dist=0` 和完整 footprint 硬检查，先只降低速度/加速度。
4. 观察连续峰值与 `retime_iters`。经常超过 5 次时先改善 seed 曲率或时间分配，不直接把上限从 15 增大。
5. 轨迹可行后再调平滑/时间/位置权重，每轮只改一项乘或除 1.5；每次重跑全部障碍负例。
6. 最后逐级提速，并重新计算制动距离、地图视距和 controller 上限。

开启配置中的 planner CSV 后，当前默认文件为仿真
`/tmp/rm27_minco_planner_simulation.csv`、实车 `/tmp/rm27_minco_planner_reality.csv`。查看时使用：

```bash
tail -n 5 /tmp/rm27_minco_planner_simulation.csv
tail -n 5 /tmp/rm27_minco_planner_reality.csv
```

验收不能只看 `NavigateToPose` succeeded。至少记录：规划耗时 P50/P95/P99、重规划成功率、
first-failure 分布、最小 fused distance、连续峰值 v/a/yaw-rate、retime 次数、轨迹代数值、
footprint 最小净空和所有负例拒绝率。

### 11.6 MPC

| 参数 | 当前值/建议 | 停止条件 |
|---|---|---|
| `MincoMpc.dt` | 0.05 s，对应 20 Hz，先固定 | 实际循环达不到 20 Hz 先修负载 |
| `MincoMpc.lookahead_time` | 0.50 s；验证范围 0.4-0.8 | 过短不补偿延迟，过长放大模型误差 |
| `reference_progress_max_lead_time` | 0.25 s；从 0.20-0.25 验证，且不大于 lookahead | 太小无法越过停车前缀；太大则参考脱离真实空间进度 |
| `Q` | `[3,3,2]` | 不先于 frame/delay 调整 |
| `q_along/q_cross` | `3/12` | 横向振荡或狭窄通道饱和 |
| `R` | `[1.5,1.5,1.0]` | 过大跟不上，过小控制抖动 |
| `control_delay_compensation` | 0.05 s；0.01 s 步进 | 必须由时间对齐数据确定 |
| `deadzone_speed_threshold` | 0.05；验证范围 0.02-0.08 | 低速爬行或目标附近极限环 |

先把 MINCO 速度固定为实车 `0.20 m/s`，再按以下顺序调 MPC：

1. frame/stamp：确认 `/odometry` 是 `odom`，轨迹 token 匹配，数据都不 stale/future。
2. 方向/模型：架空轮或仿真给低速直线、横移、正负角速度，确认实测方向与预测一致。
3. 参考游标：按第 9.2 节证明同 ID 的 t0 推进和空间 lead cap；基础游标提前到终点、ID 每周期变化或
   真实车辆落后时，先修游标/轨迹生成，禁止用 Q、deadzone 或 progress timeout 补偿。
4. 延迟：从 bag 对齐 `/cmd_vel_controller` 变化与 `/odometry.twist` 响应，先调
   `control_delay_compensation`，不要用更大 Q 掩盖相位滞后。
5. 横向：洞口横向稳态误差大且未饱和时，将 `q_cross` 每次乘 1.25；出现左右摆动则回退，或把对应
   R 每次乘 1.25。
6. 沿迹/速度：响应迟钝先确认 smoother 限制，再在未饱和前提下小调 along Q/R。
7. yaw：航向稳态误差再调 yaw Q/omega R；先排除角度 wrap、frame 和轮胎侧滑。
8. deadzone：只在目标附近爬行/极限环时以 0.01 修改，不能用它消除全程跟踪误差。
9. 每次提高速度 0.10 m/s 后重做急停、取消、BLOCK、低梁、浮板和坡上停车。

规划器限制必须不高于 MPC 盒约束和底盘物理上限。当前仿真 MPC 是
`vx/vy +/-1.5 m/s`、`omega +/-1.2 rad/s`、`ax/ay +/-1.0 m/s²`；实车分别为
`+/-0.5`、`+/-0.8`、`+/-0.8`。任一 planner 上限更大都属于配置错误，不是“让 MPC 自己限住”即可。

开启 controller CSV 后默认文件为 `/tmp/rm27_minco_mpc_simulation.csv` 或
`/tmp/rm27_minco_mpc_reality.csv`：

```bash
tail -n 5 /tmp/rm27_minco_mpc_simulation.csv
tail -n 5 /tmp/rm27_minco_mpc_reality.csv
ros2 topic hz /cmd_vel_controller
ros2 topic hz /odometry
```

若 QP 偶发失败、预测与实测反相、横向误差持续增大、控制长期顶到 box constraint，立即归零/降速，
回到 frame、时间同步、模型和底盘能力检查。

### 11.7 仿真坡面 PI 执行器

这一组参数只属于 Gazebo `CmdVelPoseControlSystem`，不属于 MINCO/MPC，也不进入实车。插件对 world-XY
速度误差施加平面力，DART 仍负责重力、坡面接触、`z/roll/pitch`。默认
`linear_velocity_integral_gain=0`、`max_planar_integral_force=0` 保持旧 P 行为；当前仿真车 xacro 显式
使用：

```text
linear_velocity_gain          80.0 N/(m/s)
linear_velocity_integral_gain 45.0 N/(m)
max_planar_integral_force     35.0 N
max_planar_force              120.0 N
stall_assist_command_threshold 0.08 m/s
stall_assist_engage_velocity   0.02 m/s
stall_assist_release_velocity  0.10 m/s
stall_assist_delay             0.20 s
stall_assist_ramp_rate        100.0 N/s
max_stall_assist_force         45.0 N
command_timeout               0.5 s
```

当前约 `10.2 kg`、`11.8 deg` 坡面的水平静态补偿约为
`m*g*tan(theta) ~= 21 N`，所以 `35 N` 积分上限保留稳态余量但仍低于总力上限。该计算只说明起始量级，不能
替代仿真测量。插件对积分力做矢量限幅；总力饱和时冻结继续推向饱和的积分分量，反向误差仍可退积分。
命令 timeout、非有限命令/状态、非法 `dt` 会清积分。因而要测试“零速保持”，上游必须仍以正常频率
发布有效零速；通信断流后保存旧重力补偿不是安全行为。

停滞辅助不参与正常速度环。只有目标速度不小于 `0.08 m/s`，且沿目标方向速度持续低于
`0.02 m/s` 超过 `0.20 s` 时才线性增加；恢复到 `0.10 m/s` 立即归零。它用于克服 Gazebo 网格接缝
和坡面小凸起的静接触，不属于实车参数，也不能用来穿越 ROG 已判定的障碍。

核对 xacro 展开值与单测：

```bash
xacro src/rm_27_stimulation/urdf/simulation_waking_robot.xacro \
  > /tmp/rm27_sim_robot.urdf
rg -n 'linear_velocity_(integral_)?gain|max_planar_(integral_)?force|stall_assist|command_timeout' \
  /tmp/rm27_sim_robot.urdf
./build/rm_27_stimulation/test_planar_velocity_pi
```

同一轮启动日志还必须出现类似：

```text
CmdVelPoseControlSystem controlling model [...] ... with planar Kp=80, Ki=45, integral-force limit=35, stall-assist limit=45 N
```

检查命令：

```bash
rg -n 'CmdVelPoseControlSystem controlling|planar Kp|integral-force limit|failed to subscribe' \
  /tmp/rm27_tunnel_acceptance.log
```

没有该行、仍显示 `Ki=0` 或插件订阅失败时，本轮只是旧 P 执行器，不得用它评价坡上停车。

坡中定点测试必须仍显式使用
`world:=tunnel_ramp_test use_ground_truth_odom:=true`。发一个位于坡面中段的低速目标并等待 action
结束，确认车辆物理落在坡面而不是平台；然后先检查 `/cmd_vel` 仍由预期单一链路持续发布有效零速。
若发布中断超过 `command_timeout=0.5 s`，插件会主动清积分，这测到的是通信 timeout 后的 P 阻尼，
不是 PI 零速保持。不得另起第二个 `/cmd_vel` publisher 与 Nav2 抢写来制造结果：

```bash
ros2 topic info -v /cmd_vel
ros2 topic hz /cmd_vel
timeout 30 ros2 topic echo /ground_truth/odometry --csv \
  > /tmp/rm27_slope_hold_ground_truth.csv
timeout 30 ros2 topic echo /cmd_vel --csv \
  > /tmp/rm27_slope_hold_cmd.csv
```

两个 `echo` 要在两个终端同时启动。当前场景的建议筛选门是 30 s 内平面漂移不超过 `0.02 m`、最大
平面速度不超过 `0.02 m/s`，且输入命令持续为零。先把阈值写入本轮验收记录，再运行：

```bash
awk -F, '
NF >= 53 && $1 ~ /^[0-9]+$/ {
  n++
  if (n == 1) { x0 = $5; y0 = $6 }
  x1 = $5; y1 = $6
  speed = sqrt($48 * $48 + $49 * $49)
  if (speed > vmax) vmax = speed
}
END {
  if (n < 100) { print "PI_HOLD_FAIL: insufficient GT samples"; exit 2 }
  drift = sqrt((x1-x0)^2 + (y1-y0)^2)
  pass = drift <= 0.02 && vmax <= 0.02
  printf("PI hold samples=%d drift=%.5f vmax=%.5f => %s\n", \
         n, drift, vmax, pass ? "PI_HOLD_KINEMATIC_OK" : "PI_HOLD_FAIL")
  exit(pass ? 0 : 1)
}' /tmp/rm27_slope_hold_ground_truth.csv

awk -F, '
NF >= 6 && $1 ~ /^[-+0-9.eE]+$/ {
  n++
  planar = sqrt($1*$1 + $2*$2)
  yaw = $6 < 0 ? -$6 : $6
  if (planar > vmax) vmax = planar
  if (yaw > wmax) wmax = yaw
}
END {
  pass = n >= 100 && vmax <= 0.001 && wmax <= 0.001
  printf("PI hold cmd samples=%d planar_max=%.6f yaw_max=%.6f => %s\n", \
         n, vmax, wmax, pass ? "ZERO_COMMAND_STREAM_OK" : "ZERO_COMMAND_STREAM_FAIL")
  exit(pass ? 0 : 1)
}' /tmp/rm27_slope_hold_cmd.csv
```

上面只验证执行器在零速输入下抵消坡向稳态扰动；还需检查车轮接触、姿态、无高频振荡和 timeout
恢复行为。第 10.2 节两次单向运行都停在坡顶平台，并非专门的坡中 30 s 保持试验，因此不能据其
末端速度为零宣称 PI 坡中保持已经验收。

调参固定顺序：先令 `Ki=0` 调 `Kp` 到无明显高频振荡；再从 `Ki=20` 起每次增加 `10-20`，观察至少
30 s 的坡向稳态速度、过冲和总力饱和；最后把积分力上限设为实测所需保持力的 `1.5-2.0` 倍，且必须
小于总平面力上限。坡向漂移仍有稳定偏差才增加 Ki；往返振荡、坡顶冲出、频繁总力饱和或 timeout
恢复后突跳则回退。不得通过提高摩擦、冻结重力、直接写 pose 或把 PI 参数复制到实车来制造通过。

### 11.8 通用停止条件

出现以下任一项，本轮调参停止并回退最后一次改动：

- 未知或 stale 被当作 free；
- occupied 负例漏检一次；
- 错高浮空薄面得到 support verified，或缺高程/错误 TF 时仍放行；
- TF 跳变或 frame mismatch；
- planner/controller 发布 NaN、Inf 或旧 generation；
- 取消、BLOCK 或输入超时后速度不归零；
- 规划 P95 超过控制任务允许的重规划周期；
- MINCO 经常耗尽 15 次 retime；
- MPC 连续 QP_FAILED、输出饱和或实测轨迹离开 footprint 安全带；
- CPU、点云队列或 DDS 延迟导致地图 age 接近 timeout。

## 12. rosbag 录制、回放和复现

### 12.1 原始输入 bag

仿真：

```bash
mkdir -p bags
ros2 bag record --storage mcap \
  --use-sim-time \
  --compression-mode file --compression-format zstd \
  -o "bags/tunnel_raw_$(date +%Y%m%d_%H%M%S)" \
  --regex '(^/tf$|^/tf_static$|^/clock$|^/map$|^/ground_truth/odometry$|^/registered_scan$|^/lidar_odometry$|^/cloud_registered_full$|^/aft_mapped_to_init$|^/odometry$)'
```

同一个仿真正则同时覆盖 ground-truth localizer 和 RMUC2026 Point-LIO 输入；当前未发布的 topic 不会写入 bag。

实车：

```bash
mkdir -p bags
ros2 bag record --storage mcap \
  --compression-mode file --compression-format zstd \
  -o "bags/real_raw_$(date +%Y%m%d_%H%M%S)" \
  --regex '(^/tf$|^/tf_static$|^/map$|^/cloud_registered_full$|^/aft_mapped_to_init$|^/odometry$)'
```

若需要观察 action feedback/status 或 lifecycle event，可加 `--include-hidden-topics`，但先确认磁盘带宽。
action goal/result 依赖 service，请求内容若未启用 service introspection 和 service 录制，不能只凭 topic bag
完整复现；本流程要求另外记录目标 pose，并在回放时重新发送 action。

### 12.2 结果和诊断 bag

```bash
ros2 bag record --storage mcap \
  --use-sim-time \
  --compression-mode file --compression-format zstd \
  --include-hidden-topics \
  -o "bags/tunnel_result_$(date +%Y%m%d_%H%M%S)" \
  --regex '(^/tf$|^/tf_static$|^/clock$|^/rosout$|^/ground_truth/odometry$|^/odometry$|^/rog_map/.*|^/minco/.*|^/minco_shadow/.*|^/plan$|^/opt_path_vis$|^/minco_candidate_path_vis$|^/astar_path_vis$|^/mpc_predict_path$|^/mpc_real_path$|^/cmd_vel_controller$|^/cmd_vel_nav2_result$|^/cmd_vel$|.*/_action/.*)'
```

ROG 可视化数据按需生成。开始录制后，在另一终端确认目录增长：

```bash
du -sh bags/<bag_name>
```

用 `Ctrl-C` 正常结束、metadata 写完后再检查：

```bash
ros2 bag info bags/<bag_name>
```

### 12.3 安全回放

禁止在连接真实底盘和驱动的 ROS domain 内回放控制结果。使用隔离 domain：

```bash
export ROS_DOMAIN_ID=71
source /opt/ros/jazzy/setup.bash
source /home/pnx/nav_ws/sentry-navigation-RM27/install/setup.bash
ros2 bag play bags/<raw_bag_name> --clock
```

只回放 raw bag，让当前代码重算结果。不要同时回放旧 `/cmd_vel`、旧 `/minco/opt_path` 和新节点输出。若必须检查 result bag，使用 topic remap 或不开任何控制/驱动节点：

```bash
ROS_DOMAIN_ID=71 ros2 bag play bags/<result_bag_name> \
  --remap /cmd_vel:=/replay/cmd_vel \
          /cmd_vel_controller:=/replay/cmd_vel_controller \
          /cmd_vel_nav2_result:=/replay/cmd_vel_nav2_result
```

复现记录至少包含：git commit、dirty diff、launch 完整命令、参数 dump、bag 路径、目标 pose、首次失败日志和 RViz 截图。

## 13. 症状速查表

| 症状 | 最可能层 | 先执行 | 禁止的处理 |
|---|---|---|---|
| 没有 `/rog_map/*` | planner 未 active、无订阅或输入未初始化 | lifecycle、sensor-data echo、点云/odom/TF | 盲目重装依赖 |
| 仿真日志报 no-return `INVALID_LAYOUT/FIELDS/RING_LAYOUT` | Gazebo 点云组织形式与配置不一致 | 查 width/height/fields/ring 和 localizer 参数 dump | 猜角度重建或把 invalid 当 free |
| `/livox/lidar` 稳定但 `/registered_scan`/ROG 掉频 | full no-return/PCL transform 或后续 ROG 过载 | 分开看 localizer/planner CPU、两级 hz 和 `ROGMapPerf` | 增大 `map_timeout` 或伪造 stamp |
| 实车出现 `organized no-return rays=enabled` | 误启动仿真 localizer或误下放 simulation 参数 | 停车，查节点、参数来源和 publisher 数量 | 继续采集“free”结果 |
| `layer_type` 全 UNKNOWN | 点云、odom、时间或投影未初始化 | 看 reject log、raw occupied、TF | 降低 collision distance |
| 启动报 `no ground_elevation` | 实车地图尚未标定 | 查实际 map YAML 和 install 来源 | 关闭 `require_ground_support` |
| `clearance_status` 大片 0 | PGM 非 known-free、高程缺失或 prior TF 无效 | 查 YAML、九点地图边界、两向 TF | 打开 observed-empty |
| 坡面大片 status 100 | 候选 Z 与 support Z 不匹配 | 对照 patch/ref/slope、TF Z、z offset | 直接增大 tolerance |
| active 出现 status 2 | 错 profile 或 bridge 被误开 | 参数 dump、launch mode | 接受为正常 |
| dynamic blocked、fused free | 当前安全配置不应出现的旧 prior-fill 行为 | 查四个 fallback 开关和 `free_fills_unknown` | 继续 active 测试 |
| fused 大范围 free | 参数 overlay/地图 support 范围错误 | 查 status、PGM 九点先验、YAML patch、TF | 继续 active 测试 |
| 洞顶可见但净空通过 | 高度、z offset、车辆高度或投影 bug | raw occupied、headroom、clearance | 降低 known ratio |
| 平地净空总是未验证 | 射线覆盖或 known ratio | known ratio 分布、自滤除、传感器模型 | 把实车改成 0.25 |
| 低梁漏检 | occupied/自滤除/投影 | raw 与 filtered occupied、fused | 放宽任何参数 |
| 首失败 `OUT_OF_MAP` | ROG origin/滑窗尺寸/TF | fused layer 边界、两向 tf2_echo | 增大 timeout |
| 首失败 `STALE_SNAPSHOT/FUTURE_SNAPSHOT` | 真断流、时钟错误或 safety callback 自饿 | 第 7.1 节：输入 delay、ROG perf、CPU、失败 t/age | 无限提高 map timeout |
| 中心 free、footprint collision | 正常的车体角点保护 | 看 offset/yaw/cost | 改成仅查中心 |
| `t=nan/nan` footprint 持续 lethal，点落在车轮外沿 | 自滤框尺寸/姿态或外参 | 失败点转车体系，对照 URDF/CAD、raw/filtered/marker | 缩小安全 footprint 或照抄仿真 box 到实车 |
| self-filter marker 不随车体 yaw/roll | `transform_center`、TF/stamp 或错误 overlay | 查生效 filter mode、position frame 和 stamped TF | 继续扩大过滤框 |
| 转向后当前 footprint 查到 unknown，日志 yaw 近 0 | 错把 `gimbal_yaw_fake` 当真实车体朝向 | 并行 `tf2_echo map base_link` 与 `map gimbal_yaw_fake` | 圆化 footprint 或扩大 unknown 引导区 |
| optimizer 经常 retime 15 次 | 动力学限值、曲率、时间分配 | peak v/a、搜索路径 | 再增加 retime 次数 |
| 有 path 无 NORMAL opt path | 优化或安全复核失败 | first-failure、retime log | 让 MPC 跟 backup 运动 |
| 有 NORMAL 但 MPC 零速 | session、stamp、odom、QP | MPC fail reason、generation | 绕过 session gate |
| 同 ID 起点零速且参考锁在 t0 | MPC 参考时间未推进或运行旧 install | 第 9.2 节索引/ID/lead 调试 | 把 controller 和 smoother deadzone 都清零 |
| 参考先到终点，车仍落后并 progress abort | 时间游标未按空间进度限幅 | 查 `lead_s` 和 `reference_progress_max_lead_time` | 增大 progress timeout 或以参考终点判成功 |
| `/minco/cmd_vel_mpc` 有值但最终无值 | Nav2/smoother/fake vel 链 | 逐 topic echo | 调 ROG 参数 |
| `/cmd_vel` 有值但车不动 | ros_gz transport/通信/底盘 | Gazebo topic bridge 或硬件接口 | 调 MINCO 权重 |
| 仿真坡上给零速仍持续下滑 | Gazebo P 环有重力稳态差或 PI 未装配 | 展开 xacro、查插件加载日志、记录真值坡向速度 | 放宽 progress checker 或复制 PI 到实车 |
| BLOCK 后仍有自旋 | `/cmd_spin` 旁路或参数 overlay 未生效 | 查 `enable_cmd_spin`、订阅者和最终速度 | 只调 MPC 权重 |
| shadow 下车移动 | 普通主链仍是 legacy | 取消主 action、物理禁用驱动 | 认为 shadow 自动静止 |
| 清 costmap 后 ROG 仍阻塞 | 两套地图独立 | 查 ROG 数据源和衰减 | 重复清 Nav2 costmap |

### 13.1 从现象开始的故障树

```text
NavigateToPose 不动/失败
|
+-- 没有 /plan
|   +-- action/BT/lifecycle 未 active
|
+-- 有 /plan，但没有安全 NORMAL /minco/opt_path
|   +-- 没有 ROG snapshot -> cloud/odom/TF/time/QoS
|   +-- layer_value 阻塞
|   |   +-- status 0   -> PGM known-free / elevation / prior TF
|   |   +-- status 100 -> candidate Z vs support Z / z offset
|   |   +-- status 50  -> body known ratio / 低梁 occupied
|   |   `-- static=100 -> PGM occupied 或九点边界
|   +-- ROG 可通行 -> seed/corridor/MINCO/retime/footprint safety
|
+-- 有匹配 NORMAL，但 /minco/cmd_vel_mpc 为零
|   +-- token/stamp/frame/odom timeout/QP fail
|
+-- raw MPC 非零，但 /cmd_vel_controller 或 smoother 输出为零
|   +-- ControllerServer action/cancel/velocity smoother
|
`-- 最终 /cmd_vel 非零但车不动
    `-- ros_gz transport、底盘通信、使能、急停或硬件故障

出现错误放行
|
+-- raw_occupied 中没有障碍 -> 传感器/FOV/外参/自滤除/时间同步
+-- raw 有、filtered 无       -> self-filter
+-- occupied 有、layer free   -> 柱分类/support tolerance/body band
+-- dynamic blocked、fused free -> 错误 prior-fill/参数 overlay
`-- fused blocked、轨迹穿过   -> query frame/snapshot/token/footprint 安全门
```

一次性采集故障树所需最小证据：

```bash
mkdir -p /tmp/rog_minco_debug
ros2 param dump /planner_server > /tmp/rog_minco_debug/planner.yaml
ros2 param dump /controller_server > /tmp/rog_minco_debug/controller.yaml
ros2 topic echo /rog_map/clearance_status --once --qos-profile sensor_data \
  > /tmp/rog_minco_debug/clearance_status.txt
ros2 topic echo /rog_map/layer_value --once --qos-profile sensor_data \
  > /tmp/rog_minco_debug/layer_value.txt
ros2 topic echo /minco/opt_path --once --qos-profile sensor_data \
  > /tmp/rog_minco_debug/opt_path.txt
ros2 run tf2_ros tf2_echo odom map \
  > /tmp/rog_minco_debug/tf_odom_from_map.txt
```

实车把最后一条另加 `tf2_echo camera_init map`。这些命令应与第 12 节 bag 同时运行；顺序执行的
`--once` 不是同一原子快照，不能用于毫秒级时序归因。

### 13.2 源码级调试、回溯和 sanitizer

普通行为错误先用 topic/log/bag 复现；只有崩溃、死锁、非有限值或无法从 first-failure 定位时再进
调试器。所有源码级调试只在仿真或断开底盘输出的环境进行，并保持 `use_respawn:=false`，否则崩溃
进程会被自动拉起并污染现场。

独立的带符号构建：

```bash
cd /home/pnx/nav_ws/sentry-navigation-RM27
source /opt/ros/jazzy/setup.bash
colcon --log-base log_rog_minco_debug build --symlink-install \
  --build-base build_rog_minco_debug \
  --install-base install_rog_minco_debug \
  --packages-up-to rog_map minco_planner minco_controller pb2025_nav_bringup rm_27_stimulation \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install_rog_minco_debug/setup.bash
```

默认 launch 是非 composition，planner/controller 为独立进程。先启动
`use_composition:=false use_respawn:=false`，然后找 PID：

```bash
pgrep -af 'planner_server|controller_server'
gdb -p <planner_server_PID>
```

GDB 中常用命令：

```gdb
set pagination off
set breakpoint pending on
info threads
thread apply all bt
break minco_planner::MincoPlanner::createPlan
continue
```

调 controller 时附加它自己的 PID，并把断点改成：

```gdb
break minco_controller::MincoMpcController::computeVelocityCommands
```

附加被系统 ptrace 策略拒绝时，不要直接改全机安全设置；优先在隔离开发机上用 launch prefix 或
检查本机调试权限。需要保存崩溃现场时，在启动导航的同一 shell 先执行：

```bash
ulimit -c unlimited
coredumpctl list | rg 'planner_server|controller_server'
coredumpctl gdb <PID-or-executable>
```

`coredumpctl` 只适用于启用 systemd-coredump 的主机；不可用时保留 launch 完整 stderr 和 bag。

内存越界/未定义行为可用独立 ASan+UBSan 构建，禁止覆盖正常 install：

```bash
colcon --log-base log_rog_minco_asan build --symlink-install \
  --build-base build_rog_minco_asan \
  --install-base install_rog_minco_asan \
  --packages-up-to rog_map minco_planner minco_controller pb2025_nav_bringup rm_27_stimulation \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=Debug \
    -DCMAKE_CXX_FLAGS='-fsanitize=address,undefined -fno-omit-frame-pointer' \
    -DCMAKE_EXE_LINKER_FLAGS='-fsanitize=address,undefined'
source install_rog_minco_asan/setup.bash
export ASAN_OPTIONS=halt_on_error=1:abort_on_error=1:detect_leaks=1
export UBSAN_OPTIONS=halt_on_error=1:print_stacktrace=1
```

随后按第 3.1 节启动 headless 仿真。sanitizer 会显著改变时序，因此它用于找内存/UB 问题，不能用
其规划耗时做 release 性能结论。若插件加载时报 ASan runtime 顺序问题，应给对应 Nav2 进程显式
预加载本机 `gcc -print-file-name=libasan.so`，不要把该环境变量带入正常实车 shell。

Nav2 costmap 清理服务：

```bash
ros2 service call \
  /global_costmap/clear_entirely_global_costmap \
  nav2_msgs/srv/ClearEntireCostmap '{}'
ros2 service call \
  /local_costmap/clear_entirely_local_costmap \
  nav2_msgs/srv/ClearEntireCostmap '{}'
```

这些服务只清 Nav2 costmap，不清 ROG 三维占据、投影或 field。ROG 异常必须处理其输入、概率衰减或重启对应 planner 生命周期，不能把 costmap clear 当成 ROG reset。

## 14. 实车 release 门槛与回退

### 14.1 release 门槛

全部满足才允许从 `minco_shadow` 转 `minco`：

1. 本文第 2.3 节全部测试通过，无 flaky failure。
2. 仿真正向和反向各 10/10 通过。
3. 静态墙、低浮空横梁、错高斜浮板、小箱体、stale、取消和旧 generation 负例全部 fail-closed。
4. 实车 map 已加入经复核的 `ground_elevation`；测点、拟合残差、patch 边界和 TF Z 归档。
5. `require_ground_support/clearance_check_enable/unknown_as_occupied=true`；observed-empty、bridge、
   near-field 和 `free_fills_unknown` 均为 false；current-footprint bootstrap 为 true，但尺寸、offset、
   支撑匹配/连续性和 occupied veto 已由参数 dump、单测与负例共同证明。
6. 仿真 0.25 与实车 0.80 参数分离已由第二人复核。
7. 实车 raw 点云高度、外参、自滤除框和车辆完整包络已测量记录。
8. 若运行域有坑/坠落边缘，下视支撑否决传感器已接入并完成动态坑负例；否则相关区域已在地图封闭。
9. 实车 shadow 连续运行至少 30 分钟，无错误放行、无地图长时间 stale、无 TF 跳变。
10. shadow 期间至少采集 30 次洞口/坡面正例统计，并保存所有 first-failure。
11. 规划耗时 P95、地图 age、MPC loop rate 和 CPU 峰值满足实时预算。
12. 底盘急停、遥控接管和 `legacy` 重启流程现场演练通过。
13. 发布参数、bag、日志、commit 和回退命令归档。
14. 仿真 no-return 已先以 `1 x 1` 证明语义，再以部署 `1 x 2` 完成性能与全部负例；finite hit 不受
    stride 影响，`10.5 >= 10.0 + 2*0.05 m` 有参数 dump/日志留证。reality profile 和实车节点中未
    启用该 Gazebo 重建。

### 14.2 实车逐级放行

按以下顺序，每级通过后才能进入下一级：

1. 轮子架空，速度上限 `0.10 m/s`，验证方向、取消、BLOCK、超时归零。
2. 开阔平地，速度 `0.20 m/s`，验证路径和 MPC 跟踪。
3. 洞口前停车，触发 shadow 异步规划 session 并人工复核 ROG 各层。
4. 入口低速进出，不上坡。
5. 洞口加短坡全程 `0.20 m/s`，正反向重复。
6. 每次只加 `0.10 m/s`，最高不超过当前实车 `0.50 m/s` 基线。

现场必须有独立于 ROS 的急停人员；进入狭窄洞口时禁止无人值守。

### 14.3 回退到旧链

模式不能热切换。回退步骤：

1. 取消当前 action，并用硬件急停或底盘使能确保零速。
2. 停止整个 navigation launch。
3. 保留 `/tmp` 日志、参数 dump 和 bag，不覆盖失败现场。
4. 用相同 `world/map` 重启 `legacy`：

```bash
ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py \
  world:=highbay slam:=false \
  navigation_mode:=legacy enable_legacy_terrain:=auto use_rviz:=true
```

5. 重新检查 planner/controller lifecycle 和最终 `/cmd_vel`，不要假设旧进程已经完全退出。

回退不等于问题关闭。必须保留触发回退的首个失败样本、地图层、TF、参数和输入 bag，离线复现后才能重新申请 active。

## 15. 推荐的一次完整执行流程

```text
构建 + 单测
  -> 启动 legacy，记录基线
  -> 实车测绘 ground_elevation；仿真核对 RMUC2026 grid 或洞坡 patch
  -> 启动 minco_shadow，验证节点、QoS、TF、ROG 各层
  -> 触发 shadow 异步规划 session，对照 first-failure 和连续极值
  -> 做静态墙、低梁、错高斜浮板、小箱体、stale、取消负例
  -> 有坠落风险时完成下视传感器和真实坑负例
  -> shadow 正反向 10 次并录 bag
  -> 停止并重启 minco，0.2 m/s active 仿真
  -> 验证 MPC 到最终 cmd 的每一跳
  -> 仿真 release 统计
  -> 实车 shadow 30 分钟和 30 次观测
  -> 架空轮 0.1 m/s
  -> 开阔地 0.2 m/s
  -> 洞口入口
  -> 洞口 + 短坡 0.2 m/s
  -> 按 0.1 m/s 增量提速，最高 0.5 m/s
```

整个过程最重要的原则是：先证明障碍、未知、旧数据和错误 frame 都会被拒绝，再证明正例能够稳定通过，最后才提高速度。ROG-map 提供的是三维观测证据，MINCO/MPC 只能在证据可靠的前提下优化运动；任何上游证据缺失都不能靠下游调参补救。

## 16. 坡面长停与图形负载专项排查

### 16.1 2026-09-08 RMUC2026 运行结论

这次运行从约 `1788843231` 到 `1788843270` 在坡面附近反复停走。首个失败点不是 MPC
求解器，而是 ROG 动态投影：安全脚印边缘的零回波列反复为
`raw_reason=HEADROOM_UNVERIFIED`。其中多次下一帧已经变成
`raw_reason=ROBOT_FOOTPRINT_CLEAR`，但旧实现仍显示 `cost_cause=HYSTERESIS`、
`pending_count=1`，导致安全门在等待第二个投影帧期间继续发布 BLOCK。

同时运行 Gazebo GUI 和 RViz 后，日志中的 `pipeline_age_ms` 常见值由 headless 基线约
`0.10-0.14 s` 上升到 `0.30-0.52 s`，并出现 `snapshot_age=1.7-2.7 s` 的
`STALE_SNAPSHOT`。进入 `GENERATE_TRAJ` 后，旧 FSM 还会每 `50 ms` 重跑一次 SMAC，形成
“规划失败 -> CPU 抢占 -> ROG 更慢 -> 快照过期 -> 再失败”的反馈环。

对应修复如下：

1. `ROBOT_FOOTPRINT_CLEAR` 在保留实车 `obstacle_hold_time` 优先级的前提下，不再等待通用
   free-space hysteresis；真实 occupied voxel 仍立即否决。
2. 仿真随车 bootstrap 证明区最终为 `0.52 x 0.51 m`，覆盖 `0.32 x 0.31 m` 安全脚印及每侧一个
   `0.10 m` ROG seed 步长；它只能释放 known-free、连续高程支撑且零占据回波的当前近场。
3. `minco_optimizer.failed_replan_retry_period=0.25` 将失败重试限制为最多 `4 Hz`；独立
   `20 Hz` 轨迹安全监控不变。
4. 默认 RViz 不订阅高带宽 `/registered_scan`，仍显示低频局部 `/rog_map/occupied`；需要看
   原始点云时临时勾选 `ROG-map/Registered Scan`。
5. Gazebo no-return 的纵向 miss stride 最终从 `8` 加密为 `2`。有限障碍回波仍全部保留，只增加坡面
   车体高度范围内的 free-ray 证据；实车和 Point-LIO 路线不使用这项仿真重建。
6. 仿真 MINCO 全局 costmap 的软膨胀由 `radius=0.25 m, scaling=10.0` 改为
   `radius=0.35 m, scaling=5.0`。第二洞口边缘在 STL 中存在真实立面，旧 SMAC 路径中心到其边界
   约 `0.262 m`，恰好位于旧软代价区之外，安全框角点会擦到 5 cm ROG 边界体素。新值只引导
   全局 seed 走洞口中部，不绕过 ROG 硬碰撞检查。
7. 第二洞口的后续复现发现，若每次重规划按新的路径切线重建 goal yaw，矩形角点会逐步转入
   `HEADROOM_UNVERIFIED` 列并形成反复制动。现在所有轨迹都从当前实测 yaw 优化到 NavigateToPose
   请求中的显式目标 orientation；position path 只决定平移，不再改变车头终点朝向。

同一 RMUC2026 起点 `(-11.7, 2.9)` 到目标 `(4.739, 5.488)` 的旧 `0.5 m/s` headless 对比为：问题版本约
`103 s` 到达且坡面同点连续闭锁约 `30 s`；只提高 miss 密度后约 `69.1 s` 到达；再加入全局
软膨胀后约 `53.4 s` 到达。最终复测没有 `STALE_SNAPSHOT`、`HYSTERESIS`、
`HEADROOM_BLOCKED` 或 `Failed to make progress`。坡段仍有 9 次 `HEADROOM_UNVERIFIED` 安全裁剪，
但拒绝点随车辆连续前移，只产生 3 个约 1 s 的短 BLOCK 脉冲，没有长时间停在同一位置。

针对用户实际第二洞口目标 `(5.154, 4.732)` 的同起点 headless 对照中，锁 yaw 前虽然最终成功，
但约 `96 s`、发生 4 次 `Failed to make progress`；锁 yaw 后约 `32.6 s` 到达，Nav2 action 报告
`number_of_recoveries: 0`。过程中仍有保守的观测前缘裁剪和一次短 `BLOCKED`，但不再固定在同一
投影列等待 progress recovery。该对照没有修改 RMUC2026 地图、高程或实车参数。

随后使用 `ROG raycasting.num_threads=6`、Gazebo GUI 与 RViz 同开的运行，从同一起点驶向
`(7.335, 1.648)` 时又在第二洞口下坡段停住。该次日志有 36 次轨迹安全拒绝，其中 30 次为
`HEADROOM_UNVERIFIED`、20 次固定命中约 `(4.904, 6.337)` 的同一安全脚印角点，并出现 5 次
`STALE_SNAPSHOT` 和 1 次 `Failed to make progress`。高程栅格在这段坡面连续且与生成器一致；
问题是 MID360 近场下视盲区叠加当时的 `no_return_vertical_stride=4`，使顶棚下方某些 5 cm 车体高度列
没有被任何保留的 miss ray 穿过。线程数只缩短计算时间，不能补回被 stride 丢掉的观测方向。

仿真真值链现将 miss stride 调为 `1 x 2`；有限 hit 仍不降采样，只有可确定方向的无回波射线加密。
同一 `(7.335, 1.648)` 目标的隔离 headless 回归约 36 秒成功到达：安全拒绝降为 6 次，其中 5 次
为随车辆前移的 `HEADROOM_UNVERIFIED` 前缘裁剪；固定角点重复、`STALE_SNAPSHOT` 和 progress
failure 均为 0。ROG 在第二洞口附近的 `pipeline_age_ms` 约 `102-117 ms`。这项参数只属于
Gazebo organized GPU lidar 重建，不得复制到实车或 Point-LIO 输入链。

### 16.2 推荐启动方式

日常规划、过洞和坡面验收只开 RViz，Gazebo 保持无 GUI：

```bash
cd /home/pnx/nav_ws
source /opt/ros/jazzy/setup.bash
source sentry-navigation-RM27/install/setup.bash
ros2 launch rm_27_stimulation sim_with_nav.launch.py \
  world:=RMUC2026 navigation_mode:=minco \
  use_ground_truth_odom:=true gui:=false use_rviz:=true
```

需要观察 Gazebo 接触或车体姿态时，反过来只开 Gazebo GUI：

```bash
ros2 launch rm_27_stimulation sim_with_nav.launch.py \
  world:=RMUC2026 navigation_mode:=minco \
  use_ground_truth_odom:=true gui:=true use_rviz:=false
```

做纯性能基线时两者都关：

```bash
ros2 launch rm_27_stimulation sim_with_nav.launch.py \
  world:=RMUC2026 navigation_mode:=minco \
  use_ground_truth_odom:=true gui:=false use_rviz:=false
```

不要在一次验收运行中同时打开 Gazebo GUI、`/registered_scan`、ROG raw/unknown/field 和多个
costmap。需要逐层排障时每次只打开一个重层，记录后关闭。

当前 RViz 配置已经把渲染帧率从 30 降为 20，并默认关闭 `/registered_scan`，只保留 1 Hz、
`3 x 3 x 1 m` 范围内的 `/rog_map/occupied`。若必须同时开 Gazebo 和 RViz，按以下顺序减载：

1. 保持 `Registered Scan`、`Raw Occupied`、`Unknown`、`Field` 和两个完整 costmap 关闭，只在定位
   某一层时短时打开；`Decay Time` 保持 0、点云 queue size 保持 1。
2. Gazebo 相机只用于接触/姿态检查时再打开；日常导航优先使用上面的 `gui:=false use_rviz:=true`。
3. 仍卡顿时，在测试副本中关闭 world 的 `<shadows>`/`<cast_shadows>`，并降低 Gazebo 相机窗口分辨率；
   GPU lidar 的几何量测不依赖光照阴影，但修改后仍应复跑点云计数和碰撞负例。
4. 不要降低有限 hit 密度、增大 `map_timeout` 或降低 ROG 安全频率来换画面流畅。先查 CPU/GPU：

```bash
pgrep -af 'gz sim|rviz2|planner_server_mt|rm27_ground_truth_localizer'
pidstat -p <gz_pid>,<rviz_pid>,<planner_pid>,<localizer_pid> 1
nvidia-smi dmon -s pucm
ros2 topic bw /registered_scan
```

没有 NVIDIA GPU 时跳过 `nvidia-smi`。若 planner 单核高而 GPU 余量充足，关闭 ROG/Costmap 重层；
若 `gz sim` 和 GPU 同时满载，关闭 Gazebo GUI/阴影；若 DDS 带宽高，首先确认 RViz 没订阅
`/registered_scan`。修改传感器分辨率或频率属于最后手段，必须重新做低梁、浮空板和坡面负例。

### 16.3 复测时的判据与调参边界

```bash
# 首个 ROG/轨迹失败原因
rg -n 'Trajectory safety rejected|No safely observed|emergency stop' \
  ~/.ros/log/planner_server_mt_*.log | tail -80

# Nav2 进度超时和 MPC BLOCK
rg -n 'Failed to make progress|BLOCK|TRAJECTORY_CHANGED_DURING_SOLVE' \
  ~/.ros/log/controller_server_*.log | tail -80

# 当前带宽；registered_scan 仅在专项观察时才应有 RViz 订阅
ros2 topic bw /registered_scan
ros2 topic hz /rog_map/occupied
```

复测目标是：坡面不再出现 `ROBOT_FOOTPRINT_CLEAR + cost_cause=HYSTERESIS`；规划失败段的
`Nav2 costmap global search input` 间隔不小于约 `0.25 s`；正常负载下
`snapshot_age < safety.map_timeout`。`failed_replan_retry_period` 推荐范围为 `0.20-0.50 s`：
CPU 仍满载时增大，动态绕障响应过慢时减小，但不要用减小该值来解决地图分类错误。

若仍是 `HEADROOM_UNVERIFIED`，先检查高程、当前脚印 offset 和 `/registered_scan` 的低角度
free ray，不能直接关闭 `clearance_unknown_as_occupied`。若仍是 `STALE_SNAPSHOT`，先关闭图形
重层并检查 CPU/显卡；也不能先放大 `safety.map_timeout` 掩盖持续掉帧。实车
`obstacle_hold_time=0.50` 与 `map_timeout=0.50` 的保守边界保持不变。

### 16.4 仿真巡航提速

原仿真 `minco_optimizer.max_velocity=0.50 m/s` 是整条新链的实际硬上限；velocity smoother
允许 `2.5 m/s`，MPC 仿真盒约束允许 `vx/vy +/-1.5 m/s`，因此继续放大后两者不会让现有轨迹
变快。当前仿真 MINCO 上限为 `1.00 m/s`，加速度仍为 `1.0 m/s²`。理论纯制动距离为
`1.0^2/(2*1.0)=0.50 m`，仍位于 `1.5 m` 滚动验证距离内；ROG footprint、unknown、stale 和
碰撞门均未放宽。实车仍保持 `0.50 m/s`。

相同 RMUC2026 起终点的 `0.80 m/s` headless 单次回归约 `34.6 s` 成功到达，相比修复后的
`0.50 m/s` 基线 `53.4 s` 缩短约 35%。本轮没有 `STALE_SNAPSHOT` 或 Nav2 progress recovery；
坡口出现 2 次短 `BLOCK_COMMAND`，对应 `HEADROOM_UNVERIFIED`，均自动恢复。

启动后先确认真正加载的限制：

```bash
ros2 param get /planner_server MincoPlanner.minco_optimizer.max_velocity
ros2 param get /planner_server MincoPlanner.minco_optimizer.max_acceleration
ros2 param get /controller_server MincoMpc.vx_max
ros2 param get /controller_server MincoMpc.ax_max
```

仿真当前应依次得到 `1.0 / 1.0 / 1.5 / 1.0`。需要继续试验时，每次最多提高
`0.10 m/s`，并重新执行洞口正反向、坡顶停车、低梁、浮空障碍、取消和 stale 负例；不能同时
提高速度、缩短 lookahead 或放宽 unknown。实车提速仍执行第 14.2 节的独立逐级放行流程。

## 17. 洞口投影稳定性与净空掉帧保持

最新 RMUC2026 日志中的绕行不是静态 PGM 把洞口封死，而是 ROG 柱投影在稀疏射线边界发生了
两种瞬态闭锁：已得到 `EMPTY_COLUMN/OVERHEAD_CLEARANCE_OK` 的列仍被两帧滞回保留为致命，
以及前一帧已验证的空列在下一帧没有占据命中、但车体高度 free ray 覆盖不足时立刻变成
`HEADROOM_UNVERIFIED`。后者代表“本帧证据不足”，不等价于“本帧看到了障碍”。

当前实现保留 fail-closed 主规则，并增加三项窄范围稳定化：

1. `EMPTY_COLUMN`、`OVERHEAD_CLEARANCE_OK`、`CLEARANCE_OK`、
   `GROUND_BRIDGE_CLEARANCE_OK` 和 `ROBOT_FOOTPRINT_CLEAR` 已携带明确净空证明，不再额外等待
   `hysteresis_count`；实车的 `obstacle_hold_time` 仍有更高优先级。
2. `projection.clearance_dropout_hold_time` 只保持“上一帧已验证、本帧
   `HEADROOM_UNVERIFIED`、本列零占据体素、先验高程连续”的单列。保持期间 reason 为
   `CLEARANCE_DROPOUT_HOLD`。任何占据回波、低梁、地面不连续或超时都会在当帧闭锁。
3. 仿真可用 `projection.clearance_hole_fill_enable` 修补规则化射线之间从未单独获得完整 free-ray
   覆盖的窄条带。整段必须是 known-free、具有可信且连续的高程支撑、没有任何占据体素，并在水平、
   垂直或对角方向由两个高程一致的已验证可通行端点封闭；reason 为
   `CLEARANCE_BOUNDED_HOLE_FILL`。候选先统一收集再提交，因此补出的格不能继续作为端点扩散，
   超过配置宽度或没有两端证明的观测前缘仍保持闭锁。

掉帧保持初值是仿真 `0.50 s`、实车 `0.15 s`；窄带补洞在仿真启用且最大宽度为 `0.10 m`
（两个 5 cm ROG 栅格），
实车默认关闭。前者不是障碍消失保持，后者也不是普通形态学 free-space 膨胀；两者都不能替代
`obstacle_hold_time` 或修复 stale snapshot。检查实际加载值：

```bash
ros2 param get /planner_server \
  MincoPlanner.rog_map.projection.clearance_dropout_hold_time
ros2 param get /planner_server \
  MincoPlanner.rog_map.projection.clearance_hole_fill_enable
ros2 param get /planner_server \
  MincoPlanner.rog_map.projection.clearance_hole_fill_max_width
ros2 param get /planner_server \
  MincoPlanner.rog_map.projection.obstacle_hold_time
ros2 param get /planner_server \
  MincoPlanner.rog_map.projection.min_headroom_known_ratio
```

在 RViz 打开 `Projection Reason`（`/rog_map/projection_reason`）、`Projection Type` 和
`ROG Occupied`，重点看洞口中心而不是墙边膨胀区。日志中真正的障碍应是
`HEADROOM_BLOCKED` 或带有限 `occupied_z=[min,max]` 的占据分类；稀疏观测抖动会在
`EMPTY_COLUMN/OVERHEAD_CLEARANCE_OK`、`CLEARANCE_DROPOUT_HOLD` 和
`CLEARANCE_BOUNDED_HOLE_FILL` 之间切换，不应再产生 `COSTMAP_LETHAL`。

调参只按以下顺序进行：

1. 仿真先保持 `0.50 s`、窄带补洞开启且宽度 `0.10 m`，正向和反向各过洞 5 次，统计
   `HEADROOM_UNVERIFIED`、`CLEARANCE_DROPOUT_HOLD`、`CLEARANCE_BOUNDED_HOLE_FILL` 和
   `COSTMAP_LETHAL`。
2. 仍偶发单帧误判时，每次增加 `0.05 s`，仿真上限建议 `0.50 s`；连续未知超过该时间时应查
   no-return 射线、TF 或高程，不应继续增加保持时间。
3. 实车从 `0.15 s` 开始用 bag 验证，建议范围 `0.10-0.25 s`，且必须小于
   `safety.map_timeout=0.50 s`。单格补洞保持关闭；只有现场 bag 的洞口正例和低梁、浮空板、墙角、
   动态小障碍和无端点未知带负例都通过后才可单独启用；实车初次验证最大宽度只允许一个 ROG
   分辨率 `0.05 m`。
4. `clearance_hole_fill_max_width` 是允许插值的观测缺口上限，不是洞口宽度参数。仿真不得超过
   `0.10 m`；需要更大值说明 no-return 射线、视场、TF 或高程仍有上游问题。
5. 不要为消除绕行把 `min_headroom_known_ratio` 降到 0、关闭
   `clearance_unknown_as_occupied` 或增大车体自滤框；这些会改变真实障碍的放行边界。

全局 SMAC 的 `use_esdf_cost` 在实车和仿真都保持 `false`。这是因为 ROG 是局部滚动安全场，边界的
UNKNOWN 会随车辆和射线相位移动，不应改变全局拓扑。洞口前缘仍可能在日志中出现保守的
`HEADROOM_UNVERIFIED` 和 local seed clipping，但它应随车前移、短时停车等待新观测，而不应让全局
路径改走另一条路。需要动态全局绕障时，应接入只含明确 occupied evidence 的独立代价层；不要重新
开启当前含 UNKNOWN 的 ROG ESDF 软偏置。

检查实际全局策略：

```bash
ros2 param get /planner_server MincoPlanner.smac_2d.use_esdf_cost
```

当前必须返回 `False`。

运行时临时对照参数（节点重启后恢复 YAML）：

```bash
ros2 param set /planner_server \
  MincoPlanner.rog_map.projection.clearance_dropout_hold_time 0.40

rg -n 'COSTMAP_LETHAL|HEADROOM_UNVERIFIED|CLEARANCE_(DROPOUT_HOLD|BOUNDED_HOLE_FILL)|HYSTERESIS|Failed to make progress' \
  ~/.ros/log/planner_server_mt_*.log ~/.ros/log/controller_server_*.log | tail -120
```

## 18. 坡后短停的最终检查与调参

RMUC2026 坡后长时间停止的直接原因不是 MPC 求解慢，也不是地图 TF 的 Z 偏移。早期失败点附近的
`ground_elevation` 从坡面高度过渡到侧边低平面，相邻高度差超过当时的 `max_step=0.06 m`；局部安全门
还会把该断边向连续支撑区域扩展一个栅格。此前全局种子的 `0.30 m` 缓冲全部只是软代价，
路径中心仍可穿过缓冲带，但 `0.32 x 0.31 m` 仿真安全足迹的角点会落到
`GROUND_UNVERIFIED`，形成持续的规划/裁剪冲突。

当前仿真使用 `ground_edge_avoidance.lethal_clearance_radius=0.28 m`，实车使用 `0.30 m`，两者的
软外圈 `clearance_radius` 均为 `0.30 m`。计算硬内圈下限时必须使用安全包络而不是 URDF 视觉
尺寸：

```text
lethal_radius_min = hypot(length / 2 + safety_margin,
                          width  / 2 + safety_margin) + elevation_resolution
```

本次 headless 回归使用冷启动正向穿洞上坡并反向返回，两次目标均成功。修复前反向坡段会在
`(-5.0,-3.5)` 附近因 footprint 角点跨过 `0.16 m` 高程断边触发运行中 emergency/BLOCK；加入硬内圈
后该 `GROUND_UNVERIFIED` 与运行中 BLOCK 均未复现，反向动作命令侧墙钟约 `9.6 s`。仍有边界外
`HEADROOM_UNVERIFIED` 局部种子裁剪，但没有中断控制；这是未知净空的保守行为，不应通过放宽
ROG near-field 或关闭 unknown 闭锁消除。

当前 RMUC2026 仿真还把全局 `ground_edge_avoidance.max_step/max_slope_deg` 收紧为
`0.02 m / 20 deg`，局部 ROG projection 保持 `0.06 m / 28 deg`。前者让 SMAC 绕开已测高程中的
`3 cm` 坡面局部凸起，后者仍按车体净空语义判断当前柱是否可通过。全局阈值更保守是安全的；反方向
（全局比最终局部门更宽松）会持续生成局部必拒路径，禁止使用。实车当前两组仍保持相同，只有在完整
现场高程能证明局部粗糙度位置和可绕拓扑后，才允许采用仿真的保守分层。

### 18.1 现场诊断命令

先记录失败点和真实加载参数：

```bash
ros2 topic echo /odometry --once
ros2 param get /planner_server MincoPlanner.priormap.ground_edge_avoidance.enable
ros2 param get /planner_server MincoPlanner.priormap.ground_edge_avoidance.lethal_clearance_radius
ros2 param get /planner_server MincoPlanner.priormap.ground_edge_avoidance.clearance_radius
ros2 param get /planner_server MincoPlanner.priormap.ground_edge_avoidance.clearance_cost
ros2 param get /planner_server MincoPlanner.rog_map.projection.max_ground_step
ros2 param get /planner_server MincoPlanner.rog_map.projection.max_ground_slope_deg
```

再按原因分类，不要把所有停车都当成 MPC block：

```bash
rg -n 'Trajectory safety rejected|Clipped local seed|GROUND_UNVERIFIED|HEADROOM_UNVERIFIED|occupied_z|STALE_SNAPSHOT|TF_|emergency stop|BLOCK' \
  ~/.ros/log/planner_server_mt_*.log ~/.ros/log/controller_server_*.log | tail -160

ros2 topic hz /registered_scan
ros2 topic hz /lidar_odometry
ros2 topic hz /rog_map/occupied
ros2 run tf2_ros tf2_echo map odom
```

判断标准如下：

| 现象 | 优先检查 | 处理方向 |
| --- | --- | --- |
| `GROUND_UNVERIFIED`，紧邻高度跳变 | 对应高程像素、全局/局部 `max_step`、坡边半径 | 修正高程；保证全局阈值不宽松于局部门；按足迹公式增大缓冲 |
| `HEADROOM_UNVERIFIED`，`occupied_z` 为空 | 首次/暖图差异、free ray、雷达下视角 | 改善 no-return 射线或传感器覆盖；保留 unknown 闭锁 |
| `HEADROOM_BLOCKED` 或有有限 `occupied_z` | 点云、外参、车体自滤、真实低梁 | 先确认实际障碍或自体点，再改感知参数 |
| `STALE_SNAPSHOT` 或 TF 超时 | CPU/GPU、点云和里程计频率、时间戳 | 关闭 GUI/RViz 重图层并修复上游时序 |
| 规划持续发布但控制器 `BLOCK` | trajectory token、参考新鲜度、QP 状态 | 再进入 MPC 链路排查 |

### 18.2 调参顺序和禁区

1. 先校验失败坐标附近的高程值；错误的断边不能靠增大缓冲解决。
2. `ground_edge_avoidance.max_step/max_slope_deg` 必须小于等于
   `rog_map.projection.max_ground_step/max_ground_slope_deg`。相等表示统一通行能力；全局更严格可让 seed
   绕开已测局部粗糙度；全局更宽松则会发送局部必拒路径。
3. `lethal_clearance_radius` 从上述几何下限开始，每次最多增加一个高程栅格，并始终满足
   `lethal_clearance_radius <= clearance_radius`。硬半径过大会封窄路，不能把它当作通用障碍膨胀。
4. `clearance_cost` 只作用于硬内圈外的软缓冲，建议保持 `220-252`，当前为 `240`。
5. 不要降低 `min_headroom_known_ratio` 到 0、关闭 `clearance_unknown_as_occupied`、扩大车体自滤框，
   也不要让裁剪轨迹以非零末速度结束。这些改动会直接削弱浮空障碍物或轨迹终点安全性。

最终验收应在相同起终点分别做冷启动和暖图，正向、反向各至少 5 次，并同时保留低梁、浮空板、
墙角和坡边负例。只有暖图仍在同一坐标反复 `HEADROOM_UNVERIFIED`，才继续查射线覆盖；只有
`GROUND_UNVERIFIED` 与高程断边稳定重合，才继续调整高程或坡边缓冲。

### 18.3 剩余风险

- 坡边覆盖层在 planner 启动时由静态 `ground_elevation` 构建；更新高程文件后必须重启 planner。
- 高程先验及其 `frame_id` 必须和二维地图严格对齐。现场坡体变化或测绘偏移会造成保守绕行或漏掉断边。
- 动态障碍和浮空障碍仍完全依赖滚动 ROG；静态高程覆盖层不能替代实时点云负例验证。
- 单雷达首次进入短洞/坡组合仍可能因下视盲区短暂停车，暖图流畅不代表冷启动已经覆盖所有视角。
- Gazebo GUI、RViz 点云/体素图层会争用 CPU/GPU；出现 stale 时应先 headless 复测，不能放大
  `map_timeout` 掩盖计算或时间戳抖动。

## 19. 另一处洞口下坡卡住：自体点诊断与调参

本轮故障不是高程缺失、ROG 线程不足或 TF 过期。卡住时 `snapshot_age` 约 `0.13-0.38 s`、
`pipeline_age_ms` 约 `92-98 ms`，但同一车体角点连续出现：

```text
cost_cause=RAW_OCCUPIED
raw_reason=GROUND_UNVERIFIED
footprint_offset=(约 0.06, 0.15)
occupied_z=[0.325,0.375]
support_z=0.138
prior_free=1 support_known=1
```

RMUC2026 高程生成器在相应 map 坐标 `(-4.30,-3.72)` 给出的连续坡面约为 `0.200 m`。结合当时
`map -> odom` 的 Z 偏移和车体真值姿态，回波位于 `base_link` 上方约 `0.145-0.195 m`；仿真
Mid360/IMU 位于 `y=0.18 m,z=0.14 m`，而旧底盘自滤盒只到 `y=0.16 m`。因此这是传感器组件漏滤，
不是坡体或浮空障碍物。

当前仿真配置使用两盒自滤：

```yaml
cloud_filter:
  position_frame: base_link
  filter_mode: transform_cloud
  box_padding: 0.02
  positions:
    x: [0.0, 0.0]
    y: [0.0, 0.18]
    z: [0.07, 0.14]
  box_sizes:
    x: [0.34, 0.10]
    y: [0.28, 0.08]
    z: [0.18, 0.08]
```

第一盒覆盖底盘/车轮，第二盒只覆盖雷达和 IMU 外壳。启动日志必须显示 `boxes=2`，RViz 的
`/rog_map/self_filter_box` 应显示两个随 `base_link` 俯仰、横滚和偏航的盒：

```bash
rg -n 'ROGMap CloudFilter' ~/.ros/log/planner_server_mt_*.log | tail -5
ros2 topic echo /rog_map/self_filter_box --once
ros2 run tf2_ros tf2_echo base_link left_mid360
```

### 19.1 如何区分自体点、真实障碍和观测不足

先查失败日志，不要先改 `min_headroom_known_ratio`：

```bash
rg -n 'Trajectory safety rejected|occupied_z|support_z|footprint_offset|STALE_SNAPSHOT|TF_' \
  ~/.ros/log/planner_server_mt_*.log | tail -160
```

判断顺序如下：

| 日志特征 | 含义 | 首选处理 |
| --- | --- | --- |
| `occupied_z` 有限，固定在车体局部坐标和固定相对高度 | 高概率自体回波 | 对照 URDF/CAD，增加独立部件盒 |
| `occupied_z` 有限，固定在世界坐标，RViz 与墙/梁重合 | 真实障碍 | 保持闭锁，修路径或目标位姿 |
| `occupied_z=[nan,nan]`、`HEADROOM_UNVERIFIED` | 没有 hit，但 free-ray 覆盖不足 | 查视场、miss ray、近场证明和快照新鲜度 |
| `GROUND_UNVERIFIED` 与高程断边重合 | 高程或坡边问题 | 校验高程，并检查全局坡边缓冲 |
| `STALE_SNAPSHOT/FUTURE_SNAPSHOT/TF_*` | 时序或 frame 问题 | 查发布频率、时间戳和算力，不改几何盒 |

高程文件先做确定性一致性检查：

```bash
cd /home/pnx/nav_ws/sentry-navigation-RM27
python3 src/pb2025_nav_bringup/tools/generate_rmuc2026_elevation.py --check
```

当前应报告 `support=113792/113794 known-free cells` 和 `z=[0.000, 0.310] m`。若检查失败，先重新
生成并重启 planner；若检查通过且有限回波随车体移动，再进入自滤排查。

### 19.2 自滤盒调参边界

1. 每个刚性部件单独计算 `base_link` 坐标系下的 AABB，优先增加小盒，不要整体放大底盘盒。
2. `box_padding` 只补偿点噪声、外参和同步残差。仿真建议 `0.01-0.03 m`；当前 `0.02 m` 已覆盖
   `2 mm` 雷达噪声和几何边界。超过 `0.03 m` 前必须做近车障碍负例。
3. 盒必须随车体旋转，因此保持 `position_frame=base_link` 和 `filter_mode=transform_cloud`。坡上
   盒不随俯仰旋转时，平地正常、上下坡漏点是必然结果。
4. 若点位明显在盒内仍未滤掉，先查 paired odometry 的 parent/child、静态雷达外参和
   `cloud_odom_sync_tolerance`；继续扩大盒只会制造感知盲区。
5. 实车不得复制上述坐标。实车先静止采 bag，分别在平地、坡上和最大转向姿态拟合车体点包络，
   再用纸箱/细杆贴近包络外缘验证真实障碍仍保留。

禁止通过关闭 `clearance_unknown_as_occupied`、提高 `max_ground_step`、降低净空要求或把整车外扩
`0.10 m` 来解决自体点。这些参数会同时削弱洞顶、浮空板和坡边保护。

### 19.3 本轮回归结果与剩余风险

最终 headless 回归从 `(-5.186,-1.488)` 与 `(-3.24,-4.232)` 连续往返 4 轮，共 8 次穿越，全部
`SUCCEEDED`；单程 `17.1-20.3 s`，记录到的最大连续停滞为 `0.26 s`，最大车体倾角约 `9.2 deg`。
另一处历史下坡目标 `(7.335,1.648)` 也完成双向穿越，墙钟分别为 `37.6 s` 和 `39.2 s`，最大停滞
均为 `0.18 s`。

同一回归进程累计只有 8 次单帧轨迹安全拒绝，其中 3 次为随车辆前移的
`HEADROOM_UNVERIFIED`；`Failed to make progress`、`STALE_SNAPSHOT` 和 planning lease timeout 均为
0。ROG 常见更新约 `7-10 Hz`，`pipeline_age_ms` 约 `77-96 ms`。短 `BLOCK_COMMAND` 仍会在 action
切换和单帧保守拒绝时出现，但没有形成坡上长停。

这轮稳定性修复包含三个互补边界：仿真轮碰撞从固定轴圆柱改为安全 footprint 内的全轮径球形接触；
RMUC2026 的 DART 动力学改用 Bullet collision detector；全局高程阈值用 `0.02 m / 20 deg` 绕开
已测 `3 cm` 局部凸起，同时保留 `0.28/0.30 m` 坡边膨胀。停滞辅助只处理碰撞网格接缝的静接触，
不会改变 ROG 障碍判定。

回归命令：

```bash
ros2 launch rm_27_stimulation sim_with_nav.launch.py \
  world:=RMUC2026 navigation_mode:=minco use_ground_truth_odom:=true \
  gui:=false use_rviz:=false

ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: -5.186, y: -1.488}, orientation: {w: 1.0}}}}"
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: -3.24, y: -4.232}, orientation: {w: 1.0}}}}"
```

正式验收仍应冷启动和暖图各做正反向 5 次，并加入贴近传感器盒外缘的低矮障碍、洞顶、浮空板和
坡边负例。若后续再次长停，只有“同一局部点、同一有限高度带、随车体移动”的签名才继续调整
自滤盒；世界坐标固定的有限回波应按真实环境处理。仿真接触参数和 `0.02 m` 全局粗糙度阈值都不得
直接复制到实车，实车必须由轮径、可跨台阶能力和现场高程重新标定。

## 20. 2026-09-09 坡口重复卡死：全局静态硬净空

最近一次从 `(5.86,3.70)` 到 `(-3.28,6.22)` 的运行不是 MPC QP、里程计或 TF 先失效。最终目标
会话共有 56 次安全拒绝，其中 55 次为 `DYNAMIC_AND_PRIOR`；49 次是 `GROUND_UNVERIFIED`，6 次
是 `HEADROOM_BLOCKED`。失败中心固定在约 `(4.475,6.748)`，足迹角点 `(4.278,6.851)` 落入
静态 prior occupied。只有会话末尾出现一次 `STALE_SNAPSHOT`，所以继续放大 `map_timeout`、增加
ROG 线程或放宽 MPC 门都不能修复这个位置相关故障。

根因是全局与局部使用了不同几何语义：SMAC 只拒绝代价 `>=253`，旧路径所在格代价为 `171`，
因此软 inflation 允许车体中心穿过；局部 ROG safety 随后按完整矩形足迹检查，角点触到 prior
occupied 就必然停车。现在全局查询按以下顺序组合：

```text
Nav2 static/global costmap
  -> StaticObstacleClearanceQuery（只从 cost=254 扩张硬净空）
  -> SurveyedGroundEdgeQuery（坡边、无高程支撑、坡边缓冲）
  -> SMAC 2D centerline search

ROG fused 3D query
  -> local seed / corridor / MINCO
  -> swept rectangular footprint safety
  -> MPC
```

静态硬半径自动使用：

```text
hypot(footprint_length/2 + footprint_margin,
      footprint_width/2  + footprint_margin)
+ max(global costmap resolution, ROG resolution)
```

当前仿真为约 `0.273 m`，实车为 `0.300 m`。它没有单独的运行时参数；需要改变时必须先实测车体
外廓，然后修改 `MincoPlanner.safety.footprint_length/width/margin`，使全局和局部同时变化。
`ground_edge_avoidance.lethal_clearance_radius` 仍只负责高程断边，不能用来补静态墙体净空。

### 20.1 启动与日志验收

仿真必须与实车 DDS 域隔离。两个终端都使用同一个非实车 domain，例如：

```bash
cd /home/pnx/nav_ws/sentry-navigation-RM27
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=73
ros2 launch rm_27_stimulation sim_with_nav.launch.py \
  world:=RMUC2026 navigation_mode:=minco use_ground_truth_odom:=true \
  gui:=false use_rviz:=false
```

启动日志应先后出现自动半径和覆盖统计，activate 阶段还会再构建一次：

```bash
rg -n 'Global static hard-clearance radius|Static-obstacle global hard clearance|Surveyed ground-edge global overlay' \
  ~/.ros/log/planner_server_mt_*.log | tail -20
```

仿真期望半径接近 `0.273 m`，实车接近 `0.300 m`。激活阶段的
`newly_hardened_cells` 应大于 0；若配置和激活两次都为 0，先检查 `/map`、global costmap static
layer 和实际加载的 overlay，不要放宽 safety。`unsupported_cells` 大于 0 表示二维地图声称可走，
但对应高程没有有效支撑；这类格现在会失败关闭，应修高程文件或二维地图，不应改代码绕过。

重新发送相同目标后，重点确认旧失败坐标不再进入输出路径：

```bash
export ROS_DOMAIN_ID=73
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: -3.28, y: 6.22}, orientation: {w: 1.0}}}}"

rg -n 'start_cell|goal_cell|Trajectory safety rejected|COSTMAP_LETHAL|GROUND_UNVERIFIED|HEADROOM_BLOCKED|STALE_SNAPSHOT|Failed to make progress' \
  ~/.ros/log/planner_server_mt_*.log ~/.ros/log/controller_server_*.log | tail -200
```

验收标准不是完全没有一次短时 BLOCK，而是：路径中心不再进入 `(4.475,6.748)` 附近的硬格；同一
世界坐标不再连续出现 prior occupied 拒绝；没有 15 秒一次的 `Failed to make progress`；最终动作
成功。若路径改道后在新坐标出现有限 `occupied_z`，回到第 19 节区分真实障碍和自体点。

### 20.2 调参边界与回归

1. `footprint_length/width` 填真实完整外形，不是视觉模型或轮距；`footprint_margin` 只覆盖定位、控制
   和外参误差。仿真当前 `0.05 m`，实车未经负例验证不要降低。
2. 洞口因硬净空完全断路时，先核对 PGM/STL/车体尺寸是否确实有几何通路。不得通过降低硬半径让
   全局重新生成局部 safety 必拒的路线。
3. `inflation_radius/cost_scaling_factor` 只调路径偏好。它们不能替代硬净空，也不能解决固定坐标的
   prior collision。
4. `ground_edge_avoidance.max_step/max_slope_deg/lethal_clearance_radius` 只在高程不连续时调整；墙体
   卡死不应改这些值。
5. `map_timeout`、ROG 线程数和 MPC 权重只有在日志分别证明 stale、计算饥饿或 QP 问题后才调整。

代码回归命令：

```bash
cd /home/pnx/nav_ws/sentry-navigation-RM27
source /opt/ros/jazzy/setup.bash
colcon build --packages-select minco_planner --symlink-install --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select minco_planner --event-handlers console_direct+
colcon test-result --verbose --test-result-base build/minco_planner
python3 src/pb2025_nav_bringup/tools/generate_rmuc2026_elevation.py --check
```

本次静态地图连通性检查把自动静态硬净空、当前坡边硬内圈和无支撑闭锁共同投影后，最近任务的
起点、目标仍位于同一连通域；旧卡死中心被正确排除。这个结果只证明全局拓扑没有被新层封死，
不替代冷/暖启动、双向各 5 次以及低梁、浮空障碍和贴墙负例的动态仿真验收。
