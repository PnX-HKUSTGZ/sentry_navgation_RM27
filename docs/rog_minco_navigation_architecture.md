# RM27 ROG-map + MINCO + MPC 导航架构

本文只描述当前工作树中已经实现的接口、权威关系和安全边界。调试命令、验收顺序和调参方法见
[ROG-map + MINCO 调试与调参指南](./rog_minco_tuning_debug_guide.md)。

> 重要结论：当前实现是“3D 占据感知 + 可信地面高程约束的垂直柱通行判断 + 2D 距离场 +
> 平面 MINCO 轨迹 + SE(2) MPC”，不是完整三维空间规划。active real/sim 已启用
> `require_ground_support`；RMUC2026 使用与二维图对齐的 5 cm 高程栅格，highbay 使用 map-frame
> `z=0` 平地支撑，洞口测试地图另含短坡和平台 patch。实车 MINCO 可以启动，但新增坡面必须先测绘
> 后写入高程栅格或 patch；代码接入不等于实车洞口已经标定或获准通行。

## 1. 先回答新旧链路是什么关系

### 1.1 “权威”是什么意思

本文中的“权威链路”指最终能够决定：

1. 哪一条路径被 Nav2 接受；
2. 哪一个控制器产生运动命令；
3. 动态障碍是否让车辆继续运动；
4. 哪一条命令最终到达 `/cmd_vel`。

只发布地图、轨迹或诊断话题，但不进入控制输出链的组件，不是权威组件。

### 1.2 三种启动模式

`navigation_mode` 只接受 `legacy`、`minco_shadow`、`minco`。它们是启动时选择的三个互斥
profile，不是运行中的状态机。

| 模式 | 主 planner | 主 controller | 动态障碍权威 | terrain 默认状态 | 旁路 MINCO | 最终控制权 |
| --- | --- | --- | --- | --- | --- | --- |
| `legacy` | `GridBased` | `FollowPath` | `terrain_analysis -> IntensityVoxelLayer` | 开 | 无 | 旧链 |
| `minco_shadow` | `GridBased` | `FollowPath` | `terrain_analysis -> IntensityVoxelLayer` | 开 | 独立 `/minco_shadow/planner_server` | 旧链 |
| `minco` | BT 指定 `MincoPlanner` | BT 指定 `MincoMpc` | ROG 融合层和 MINCO 安全门 | 关 | 无 | 新链 |

active `minco` 参数只把新插件注册到主 server：

~~~yaml
planner_plugins: ["MincoPlanner"]
controller_plugins: ["MincoMpc"]
~~~

非 composition 的 active `minco` 和独立 shadow sidecar 都使用
`minco_planner/planner_server_mt`，由 8 线程 executor 调度 Nav2、ROG cloud/odom/map-update、
可视化和 MINCO timer 回调；legacy 与 shadow 的权威主栈仍使用 Nav2 原生 `planner_server`。
composition 模式的权威主栈使用 `component_container_mt`，shadow sidecar 仍保持独立进程。
MINCO odom、FSM、安全检查、lease 和轨迹可视化各自使用独立 callback group；ROG odom、cloud、
map-update 和可视化也各自独立。ROG 地图写入与体素可视化采样仍由同一地图互斥锁保护，但有地图
更新等待或正在执行时，本轮可视化直接跳过；可视化 ROS publish 和体素点云序列化在解锁后执行。

同一 active profile 还收紧了另两类可运动入口：

~~~yaml
bt_navigator.navigators: ["navigate_to_pose"]
behavior_server.behavior_plugins: ["wait"]
fake_vel_transform.enable_cmd_spin: false
~~~

当前异步 `MincoPlanner` 一个 session 只保留一个目标，因此 active 不创建
`/navigate_through_poses` action server；防御性的多目标 BT 文件也固定返回失败。
`spin` / `backup` / `drive_on_heading` / `assisted_teleop` 会从 behavior server 直接产生速度，
绕过 `MincoMpc` 的 planning token 和 ROG 安全门，所以 active 不加载它们；无运动的
`wait` 保留给默认恢复树。`fake_vel_transform` 在旧链中还可以把 `/cmd_spin` 的角速度叠加到
最终指令；active 将该入口关闭后不创建订阅，并把内部 spin 值强制为零，避免 BLOCK/零速在
最后一跳又变成旋转命令。legacy/shadow 仍保留原有 navigator、behavior 和默认启用的
`/cmd_spin` 兼容行为。

但 active BT 显式使用：

~~~xml
<ComputePathToPose planner_id="MincoPlanner"/>
<FollowPath controller_id="MincoMpc"/>
~~~

这样即使外部直接调用 planner/controller action，也不能在 active 进程中通过指定旧 plugin ID
绕过 ROG 安全链。旧插件只在 `legacy` 和 `minco_shadow` 的主栈中加载；它们仍是重启后的明确
回退路径，**不是故障后的自动 fallback**。当前没有在线仲裁器，也没有“MINCO 失败便切回
GridBased/FollowPath”的逻辑。

### 1.3 切换模式时的硬约束

- `navigation_mode` 在 launch 展开参数时决定整套权威关系；不能靠运行期改参数热切换。
- 应先取消导航目标、确认 `/cmd_vel` 为零、停止当前 bringup，再以另一个模式重启。
- 当前单车合同要求 namespace 保持 launch 的默认空值；命令行中直接省略 `namespace` 参数。
  非空 namespace 会在 launch 阶段拒绝，避免只命名空间化一部分全局 topic/TF。不要写
  `namespace:=''`，ROS 2 Jazzy 会把空赋值解析为非法参数。
- `enable_legacy_terrain:=auto` 在 `legacy`、`minco_shadow` 中解析为 `true`，在 `minco`
  中解析为 `false`。
- 显式在 `minco` 中设置 `enable_legacy_terrain:=true` 只会额外启动旧 terrain producer；
  active costmap 已覆盖为 static + ROG dynamic obstacle + inflation，它不会因此恢复旧
  IntensityVoxelLayer 动态避障权威。
- 反过来，在 `legacy` 中强制关闭 terrain，会让 IntensityVoxelLayer 缺少预期输入，不能视为
  正常 legacy 配置。
- `slam:=true` 当前只允许 `legacy`；`slam:=true` 与 `use_ground_truth_odom:=true` 互斥。

## 2. Legacy：原有导航链路

旧链保留完整，作为基线和 shadow 模式的实际执行链：

~~~text
LiDAR + IMU
  -> Point-LIO
     -> /cloud_registered
     -> /aft_mapped_to_init
  -> loam_interface
     -> /registered_scan
     -> /lidar_odometry
  +-> sensor_scan_generation
  |    -> /sensor_scan
  |    -> /odometry
  |    -> odom/body 相关 TF
  |
  +-> terrain_analysis
  |    -> /terrain_map
  |
  +-> terrain_analysis_ext
       -> /terrain_map_ext

/terrain_map     -> local costmap IntensityVoxelLayer
/terrain_map_ext -> global costmap IntensityVoxelLayer
  -> GridBased / ThetaStarPlanner
  -> FollowPath / OmniPidPursuitController
  -> /cmd_vel_controller
  -> velocity_smoother
  -> /cmd_vel_nav2_result
  -> fake_vel_transform
  -> /cmd_vel
~~~

两张 legacy costmap 的插件顺序均为：

~~~yaml
[static_layer, intensity_voxel_layer, denoise_layer, inflation_layer]
~~~

`terrain_analysis` 和 `terrain_analysis_ext` 直接消费 `/registered_scan` 与
`/lidar_odometry`。`sensor_scan_generation` 则负责生成底盘位姿/速度合同 `/odometry` 和 TF；
其 `/sensor_scan` 是另一条输出，不应误画成 terrain 的唯一输入。

IntensityVoxelLayer 仍是二维 Nav2 costmap layer。它把 terrain 点云按 intensity、高度和体素门限
投进二维代价地图，适合继续作为原系统基线；但它没有把同一 XY 柱内的地面支撑、车体净空、
洞顶/横梁和地面连通性统一成一个明确合同。

legacy 与 shadow 主链都使用
`fake_vel_transform.use_latest_odom_for_cmd: false`，保留旧的 `local_plan + odometry` 同步方式。
`enable_cmd_spin` 未显式设置时默认 `true`，因此旧模式仍可按原合同叠加 `/cmd_spin`；这个兼容
入口不是 ROG/MINCO 安全链的一部分。

legacy terrain 的 `/registered_scan` 订阅使用 `SensorDataQoS/keep_last(1)`，同时兼容仿真
best-effort 发布器和实车 reliable 发布器。local/global IntensityVoxelLayer 的 observation source
设置了 `expected_update_rate`，并在最终 `/cmd_vel` 输出边缘另设 `terrain_guard_enabled`：启动后从未
收到 `/terrain_map`、或最后一次接收超过仿真 `1.5 s`/实车 `0.5 s` 时，所有非零命令被拒绝，已经
运动时主动发布零速。因此 legacy/shadow 不再在 terrain 链断开时静默退化成“只看静态地图”。active
MINCO 不启动 terrain 节点，明确关闭这个 guard，继续由 ROG freshness 和 trajectory token 安全门
负责 fail-closed。

## 3. Shadow：只观察，不控制

### 3.1 shadow 的准确含义

`minco_shadow` 不是“隐藏的控制器”、不是热备控制链，也不是自动回退机制。它表示：

- 旧链照常规划、避障和控制；
- 同一个导航目标被旁路发送给一个独立 MINCO planner server；
- 旁路可以运行 ROG、搜索、优化并发布轨迹和日志；
- 旁路没有 controller server、没有 MincoMpc、没有 smoother、没有通向 `/cmd_vel` 的边。

~~~mermaid
flowchart LR
  G[Nav2 goal] --> BT[主 bt_navigator]
  BT --> R[shadow goal relay]
  R -. 异步复制 .-> SP["/minco_shadow/planner_server<br/>MincoPlanner only"]
  SP -. 观测输出 .-> ST["/minco_shadow/opt_path<br/>/minco_shadow/backup_path"]
  BT --> LP["/planner_server<br/>GridBased"]
  LP --> LC["/controller_server<br/>FollowPath"]
  LC --> S[velocity_smoother]
  S --> F[fake_vel_transform]
  F --> C[/cmd_vel]
~~~

### 3.2 为什么旁路不会阻塞旧链

shadow 单目标 BT 中的 `SendMincoShadowGoalToPose` 是同步 BT 叶节点，但内部只做：

1. 对语义上相同的目标去重；
2. 检查 sidecar action server 是否 ready、planner lifecycle 是否 active；
3. 每个 relay 实例最多保留一个在途 action，ready 时调用 `async_send_goal()`；
4. 不在 BT tick 内等待 goal response 或规划结果；
5. 只有 goal response 接受后才缓存目标；拒绝、异常、非成功 result 会清缓存并在后续 tick 重试；
6. server 不可用或 lifecycle 变化时递增 generation，使旧异步回调失效，并在重新 active 后重派；
7. 每个分支都立即返回 `SUCCESS`，随后主 BT 执行 `GridBased`。

shadow BT 外层 `RateController hz="3.0"` 表示这段 Sequence 以 3 Hz 被 tick，也就是 relay 检查和
legacy 全局重规划的节拍；它**不表示同一目标以 3 Hz 重发给 sidecar**。goal 被接受后，relay
把它保存为 `last_accepted_goal`，后续 tick 对 frame、position、orientation 逐项比较，等价目标
不会重复创建 action。当前 profile 中 sidecar 的 MINCO FSM 会在这个 action/session 内约 2 Hz
成功重规划；失败生成独立限频为最多 4 Hz，并重复
发布观测轨迹，直到到达目标、被新目标替换或 lifecycle 停止。因此 shadow 是“每目标一次 action、
sidecar 内持续重规划”，不是一次规划快照。

relay 有两条不同的重启检测路径，不能混为一谈：

- 若某次 BT tick 实际观察到 `action_server_is_ready()==false`，relay 清除已接受目标；端点恢复
  ready 后，同一目标会重派。这要求 readiness 的 false edge 确实被采样到。
- 正常 launch 中 sidecar 是 lifecycle managed node；
  `/minco_shadow/planner_server/transition_event` 的每次状态变化都会使 generation 和目标缓存失效，
  inactive 期间不派发，重新 active 后重派同一目标。因此，即使停启完整发生在两个 BT tick 之间、
  action discovery 看起来始终 ready，受管理的快速重启仍由 lifecycle 事件覆盖。

手工替换非 lifecycle action 进程，并且既没有被 tick 观察到 readiness=false、又没有发布上述
transition event，不在可检测合同内；不能把 action readiness 当成 server identity。

`NavigateThroughPoses` 在 shadow 主栈仍由 `GridBased + FollowPath` 正常执行，但不再把
整组航点派发到 sidecar。原因是当前 MINCO 插件的单段 Nav2 API 无法在异步 FSM
中保存整组顺序航点；只比较最后一点会产生虚假的 shadow 结论。多目标 shadow 因此是
纯 legacy 基线，不能记为 MINCO 多航点验证。

sidecar 由独立的 `/minco_shadow/lifecycle_manager` 管理，节点列表只有 `planner_server`。
即使主 Nav2 设置 `use_composition:=true`，sidecar 仍作为独立
`minco_planner/planner_server_mt` 进程启动。因此 ROG 初始化和 costmap lifecycle 等待不会饿死
sidecar 自己的 TF/cloud 回调；sidecar 计算慢、规划失败或退出，也不会直接占用主 planner server
的 executor，或更改旧链控制权。

### 3.3 shadow 的 topic 隔离边界

sidecar 自定义轨迹明确写到：

~~~text
/minco_shadow/opt_path
/minco_shadow/backup_path
~~~

它接收的 action 是：

~~~text
/minco_shadow/compute_path_to_pose
/minco_shadow/compute_path_through_poses
~~~

PlannerServer 框架仍会创建 `compute_path_through_poses` 端点，但它对当前异步
MINCO 属于明确不支持的诊断入口，不得直接调用并把返回的占位 Path 当成多点规划结果。

但 ROG visualizer 当前在源码中使用绝对 `/rog_map/...` 话题，不会自动变成
`/minco_shadow/rog_map/...`。正常 launch 的三种模式互斥，因此不会同时启动 active ROG 和
shadow ROG；如果手工同时启动两套，绝对可视化话题会混流，不能据此做 A/B 归因。

### 3.4 shadow 与 active 的参数装配

shadow 主栈只得到“legacy base + shadow BT overlay”，所以主 planner/controller/costmap 仍是旧链。
独立 sidecar 按以下顺序叠加参数：

~~~text
base nav2 params
  -> deployment minco_params.yaml
  -> simulation Point-LIO input overlay（仅需要时）
  -> minco_shadow_sidecar_params.yaml（最后覆盖）
~~~

最后一层把 sidecar 的 planner plugins 收敛为仅 `MincoPlanner`，并改写两条轨迹 topic。

## 4. Active MINCO：端到端权威链路

~~~mermaid
flowchart LR
  PC[注册点云 + LiDAR odometry] --> R["MincoPlanner 内嵌 ROGMapROS"]
  R --> O[3D 概率占据]
  O --> P["垂直柱分类<br/>地面 + 净空 + 连通"]
  M["map YAML/PGM"] --> N[Nav2 static global costmap]
  M --> PR[ROG static prior]
  P --> FU[动态/静态三态融合]
  PR --> FU
  FU --> DF[2D signed distance field]
  N --> GS[全局 SMAC/A* seed]
  GS --> CL["ROG 边界裁剪<br/>1.5 m 局部视距"]
  DF --> CL
  CL --> MI[MINCO position + yaw]
  DF --> MI
  MI --> SG[发布前及在线 footprint 安全门]
  SG -->|minco/opt_path| MPC[MincoMpc]
  MPC --> CS[controller_server]
  CS -->|/cmd_vel_controller| VS[velocity_smoother]
  VS -->|/cmd_vel_nav2_result| FV[fake_vel_transform]
  FV -->|body-frame /cmd_vel| BASE[底盘]
~~~

active 模式中：

- 主 local/global costmap 的完整 `plugins` 数组被覆盖为
  `[static_layer, rog_dynamic_obstacle_layer, inflation_layer]`；
- `IntensityVoxelLayer` 不再是动态障碍权威；
- ROG 中具有实测 occupied 证据的动态障碍会经 `/rog_map/dynamic_obstacles` 注入 Nav2 costmap，
  再由后置 inflation layer 生成软代价；ROG 的未知地面、未知净空、三维距离场和最终放行仍由
  MincoPlanner 内部查询和安全门负责；
- active BT 指定 `MincoPlanner + MincoMpc`；
- `fake_vel_transform.use_latest_odom_for_cmd` 改为 `true`。

因此，active 模式下旧 terrain 链不是“第二重动态保护”。ROG 输入失效、融合图失效或轨迹检查
失败时，设计行为是 fail closed 并停止，而不是悄悄改走旧 IntensityVoxelLayer。

## 5. 部署输入、topic 和 TF 合同

### 5.1 三套输入 profile

| 部署 | ROG frame | ROG 点云 | Planner/ROG odom | MPC odom | 规划输出 frame |
| --- | --- | --- | --- | --- | --- |
| 实车 | `camera_init` | `/cloud_registered_full` | `/aft_mapped_to_init` | `/lidar_odometry` | `map` |
| 仿真真值 | `odom` | `/registered_scan` | `/lidar_odometry` | `/odometry` | `map` |
| 仿真 Point-LIO | `camera_init` | `/cloud_registered_full` | `/aft_mapped_to_init` | `/odometry` | `map` |

> 实车 active MINCO 的 MPC、fake-vel 和 legacy 控制器都订阅 `loam_interface` 生成的
> `/lidar_odometry`；MincoPlanner/ROG 直接订阅 `/aft_mapped_to_init`。实车 `/odometry` 是旧版
> sensor-scan 兼容链路的历史名称，不能作为当前 active MINCO 的输入前提；现场调试
> 应以 `src/pb2025_nav_bringup/config/reality/minco_params.yaml` 和
> `config/reality/nav2_params.yaml` 为准。

所有当前 MINCO profile 都是 `planner_mode: PRIORMAP`。`FrameAwareRogQuery` 在规划时把
`map` 中的查询变换到 ROG frame。

实车和仿真的 planner 还共同配置 `frames.physical_base_frame: base_link`。Nav2 costmap 的
`robot_base_frame` 仍可保持 `gimbal_yaw_fake`，但这个 yaw-cancelled frame 不能代表车体 footprint
朝向；MINCO 会单独查询 `planning_frame <- base_link`，取得真实物理姿态。该 TF 缺失、查询失败或
包含非有限 pose 时，当前 pose 获取失败并 fail-closed，不以 `gimbal_yaw_fake` 朝向兜底。

### 5.2 实车与仿真 Point-LIO

Point-LIO 现在除原有降采样/显示点云外，还发布每帧完整注册点云
`/cloud_registered_full`，frame 为 `camera_init`。ROG 直接消费：

~~~text
/cloud_registered_full + /aft_mapped_to_init
~~~

旧的定位整理分支仍然需要：

~~~text
Point-LIO
  -> loam_interface
  -> /registered_scan + /lidar_odometry
  -> sensor_scan_generation
  -> /odometry + TF

relocalization_manager
  -> map -> odom
~~~

因此 active 并不是删除所有旧前端节点；它删除的是旧 terrain/costmap 的动态障碍权威。
MPC 依赖 `/lidar_odometry`，PRIORMAP 仍依赖连通的
`map -> odom -> camera_init` TF。

Point-LIO 的 twist 按 ROS odometry 合同表达在 `child_frame_id=body`；`loam_interface` 在当前
同轴安装合同下将其保留到 `child_frame_id=left_mid360`，MPC 再将其旋转到 `odom` 并补偿
base-to-lidar 杆臂。这与仅修改 header 名称不是一回事。

`sensor_scan_generation` 对每组同步的 `/registered_scan + /lidar_odometry` 还要求在该点云 stamp
同时取得 `lidar_frame <- robot_base_frame` 和 `lidar_frame <- base_frame` 两个 TF。任意一个查询
失败都会丢弃整组样本：不发布 `/sensor_scan`、`/odometry` 或由该样本派生的 TF，也不再用 identity
transform 兜底。TF 恢复后的第一组有效样本会清空有限差分历史并以零 twist 冷启动，下一组才恢复
正常速度估计，避免跨越 TF 中断计算出伪速度。

### 5.3 仿真真值

`use_ground_truth_odom:=true` 时跳过 Point-LIO、relocalization、loam_interface 和
sensor_scan_generation。`rm27_ground_truth_localizer` 把原始状态输出与 LiDAR 配对输出分开：

~~~text
/ground_truth/odometry
  -> 有序 GT 状态缓存
  -> 静态 TF map -> odom（首个有效真值确定后只发布一次）
  -> /odometry + 动态 TF odom -> base_footprint（保持原始 GT 更新率和 GT stamp）

/livox/lidar(scan stamp) + GT 状态缓存
  -> exact sample，或由 scan stamp 两侧状态插值
  -> worker 入队前预发布 scan-time odom -> base_footprint TF
  -> 360 x 96/ring 强校验；合格的 any-inf no-return 恢复为 miss ray
  -> 再发布 scan-time TF + /lidar_odometry + /registered_scan
     （同一插值位姿、同一 scan stamp）
~~~

缓存默认保留 `2.0 s` 状态；只有精确时间样本，或两侧 GT 间隔不超过 `0.10 s` 的严格包围插值，
才允许生成 LiDAR 配对输出。等待包围状态的 scan 最多保留 `0.20 s` steady time，队列最多 20 帧；
超时、溢出、重复、倒序或无有效包围时整帧 fail-closed 丢弃。`/odometry` 和两条 TF 仍由原始 GT
回调高频发布，不应要求它们与每个 LiDAR scan 一一同 stamp；必须同 stamp 的是
`/registered_scan` 与 `/lidar_odometry`。

ROG CloudFilter 不再依赖动态 TF 在点云时刻恰好到达。matcher 保存配对 odometry 的完整合同
`pose + parent_frame + child_frame`，先用该 pose 得到 `T_parent_child`，再只从专用 TF buffer 取
`T_filter_child` 静态外参，组合出 `T_filter_cloud`。仿真 child 是 `left_mid360`；实车 Point-LIO
消息把同一物理 IMU frame 命名为 `body`，因此配置显式用 URDF 中的 `imu_link`。只有 frame 合同不完整
或静态外参不可用时才回退到有界 exact-time TF；回退也失败则整帧丢弃，不使用 latest 动态 TF。

`/ground_truth/odometry` 的 twist 原本是在 base 原点测量并以 base 轴表达。
`/odometry.child_frame_id` 仍为 base，因此直接保留；`/lidar_odometry.child_frame_id`
改为 LiDAR 后，twist 必须连同六维 covariance 一起变换：

~~~text
v_lidar = R_lidar_base * (v_base + omega_base x p_base_lidar)
omega_lidar = R_lidar_base * omega_base
Sigma_lidar = J * Sigma_base * J^T
~~~

当前 MID360 与 base 有 `y=+0.18 m` 杆臂，所以原地转动时 LiDAR 原点必然有
`omega x p` 的线速度。只改 pose/child frame 而原样复制 twist 会让 planner 对这个杆臂运动
再补偿一次。

真值 localizer 还承担一个**仅限 Gazebo organized GPU LiDAR** 的 no-return 语义恢复步骤。当前仿真
配置是：

~~~yaml
reconstruct_no_return_rays: true
lidar_horizontal_samples: 360
lidar_vertical_samples: 96
no_return_horizontal_stride: 1
no_return_vertical_stride: 1
no_return_ray_length: 10.5
rog_raycast_max_range: 10.0
rog_map_resolution: 0.05
~~~

Gazebo 保留了每条射线的 raster 行列和 `ring`，但无回波方向的 XYZ 是非有限值。重建器的合同是：

1. 在修改任何点之前，严格要求 little-endian organized cloud 为 `width=360`、`height=320`，XYZ
   为 `FLOAT32`、ring 为 `UINT16`，并逐点验证 `ring == row`；尺寸、字段、row step、data size 或
   任一 ring 不匹配时整次重建 fail-closed，不猜测射线方向，原 non-finite 数据保留，因此 ROG
   继续把该空域视为 UNKNOWN；
2. 三个 XYZ 全部有限的真实回波一律原样保留，不受 stride 影响；只有 XYZ 中**至少一个分量为
   Inf** 的 any-inf no-return 才有方向恢复资格，纯 NaN 或其他没有 Inf 的非有限组合不重建；
3. `horizontal_stride=1`、`vertical_stride=2` 只对合成 miss 射线降采样，所有有限 hit 仍全部保留；
4. 参数硬约束为 `R_miss >= R_max + 2 * resolution`；当前即
   `10.5 >= 10.0 + 2 * 0.05 m`。合成端点因此位于 ROG raycast 上限之外，ROG 把射线裁到
   10 m 并只写 free/miss 证据，不把 10.5 m 合成端点插成 occupied hit。

没有这一步时，无回波射线在点云转换中不能提供 free-space 证据，洞口之后已经看见的空气仍可能因
体素未被穿过而保持 UNKNOWN，继而让 `min_headroom_known_ratio` 和安全查询闭锁。恢复 miss 射线后，
可见空域获得 free 证据；有限的洞顶、坡面和障碍物 hit 仍保留，再与二维 known-free 和
`ground_elevation` 支撑门共同决定通行。它只补“射线明确未命中”的空域证据，不把 UNKNOWN 全局
改成 free，也不证明轮下存在地面。

该重建只适用于当前 `use_ground_truth_odom:=true` 的 360 x 96 Gazebo raster，不能复制到实车或
仿真 Point-LIO 路径。实车必须先确认驱动是否保存 no-return 的方向/ring/方位俯仰或等价 range
语义，再按真实扫描模式实现并验证 miss；下视盲区和负障碍还需要独立下视补盲/支撑否决能力，且
`ground_elevation` 必须现场测量。不得照搬仿真的分辨率、FOV、stride 或 10.5 m 数值来制造 free。

真值模式用于先隔离感知/规划问题，再引入 Point-LIO 漂移、PCD 对齐和重定位误差。

### 5.4 Point-LIO 输入覆盖层

仿真基础 `minco_params.yaml` 默认是真值输入。仅当
`use_ground_truth_odom:=false` 时，`minco_pointlio_params.yaml` 覆盖：

~~~yaml
frames.rog_frame: camera_init
odom_topic: /aft_mapped_to_init
rog_map.frame_id: camera_init
rog_map.ros_callback.cloud_topic: /cloud_registered_full
rog_map.ros_callback.odom_topic: /aft_mapped_to_init
rog_map.visualization.frame_id: camera_init
~~~

### 5.5 ROG 输入时间和 frame 门

ROG 不会把“刚收到”当作“可用”。一帧点云进入 raycasting 前必须满足：

1. cloud 与 odom 的 `header.frame_id` 非空，且严格等于当前 `rog_map.frame_id`；
2. stamp 有限、非零，且不旧于各自 timeout；
3. 相对当前 ROS time 超前不超过 `0.05 s`；
4. odom 倒序被拒绝，cloud 重复或倒序被拒绝；
5. ROG 使用当前最新 odom，不在 ROG 内再次插值；真值模式的上游 localizer 已保证
   `/registered_scan` 与 `/lidar_odometry` 来自同一 scan stamp 的同一插值状态；
6. cloud 与 odom stamp 差不超过 `0.10 s`；
7. 最新 odom 的接收年龄不超过 `0.25 s`。

~~~yaml
odom_timeout: 0.25
cloud_timeout: 0.50
future_tolerance: 0.05
cloud_odom_sync_tolerance: 0.10
~~~

有效地图更新、decay、投影和 query snapshot 使用**传感器 cloud stamp**，不是回调处理时间。
队列中积压的旧点云不会在晚到时伪装成新地图。当前 decay 随有效更新推进；输入停止后，轨迹
safety freshness gate 会按 snapshot age 负责阻断。

### 5.6 车体自回波过滤

cloud filter 在点云 stamp 查询 `base_link <- cloud_frame` TF。通过 Z 截断的每个注册点先用该完整
刚体变换进入 `base_link`，再与 `base_link` 中定义的轴对齐车体盒比较；因此车体在注册点云 frame
中的 roll、pitch 和 yaw 都会自然反映到过滤体姿态。TF 失败时整帧拒绝，不绕过滤波继续建图。

| 参数 | 实车 | 仿真 |
| --- | ---: | ---: |
| `position_frame` | `base_link` | `base_link` |
| `filter_mode` | `transform_cloud` | `transform_cloud` |
| `remove_inside` | `true` | `true` |
| `position` | `[0, 0, 0.05] m` | `[0, 0, 0.07] m` |
| `box_size` | `[0.34, 0.24, 0.14] m` | `[0.34, 0.28, 0.18] m` |
| `box_padding` | `0.02 m/side` | `0.02 m/side` |
| `z_offset` | `-0.55 m` | `-0.45 m` |

inside 判定只使用变换到 `base_link` 的临时坐标；保留下来的点仍以原始注册点云坐标写回 cloud，
所以后续 ROG raycasting 的输入 frame 和点坐标合同不变。`/rog_map/self_filter_box` 在
`base_link` 显示实际裁剪体，用于核对完整旋转后的车体包络。短坡上仍必须分别核对真实车壳回波、
洞沿回波和 TF 时序，不能把仿真盒尺寸直接照搬到实车；盒过大或 `base_link` 标定错误仍可能把
真实障碍误删。

## 6. ROG-map：从三维占据到二维通行域

ROG-map 作为 `MincoPlanner` 插件内的 `ROGMapROS` 对象存在，不是独立 ROS node。插件
configure/activate/deactivate 也决定 ROG 的订阅、更新、可视化和 query 生命周期。

当前共同基础参数：

~~~yaml
resolution: 0.05
map_size: [10.0, 10.0, 2.0]
map_sliding.enable: true
raycasting.ray_range: [0.10, 10.0]
raycasting.p_hit: 0.90
raycasting.p_miss: 0.45
raycasting.p_occ: 0.85
raycasting.p_free: 0.499
decay.keep_time: 3.00
decay.clear_time: 5.00
projection.scan_z_min_abs: -0.45
projection.scan_z_max_abs: 1.40
projection.unknown_as_occupied: true
projection.min_observed_voxels: 2
projection.surface_height_delta_max: 0.12
projection.clearance_check_enable: true
~~~

`scan_z_min_abs` / `scan_z_max_abs` 是 ROG frame 中的绝对 Z，不是相对底盘或相对地面的高度。
LiDAR 安装高度、坡上姿态、TF 和 2 m 滑窗高度共同决定它是否覆盖了地面与洞顶。

### 6.1 垂直柱分类

每个 XY cell 对应一列三维体素。投影层扫描该列中的
`UNKNOWN / KNOWN_FREE / OCCUPIED` 状态，先生成 candidate，再做地面连通性和最终融合。

#### A. 观测不足

若 `observed_count < min_observed_voxels`：

- reason 为 `INSUFFICIENT_OBSERVATION`；
- candidate 为 `UNKNOWN`；
- 当前 `unknown_as_occupied: true`，所以动态 mask 阻塞；
- 静态先验默认也不会全局填开它。

#### B. 没有 occupied return 的空柱

若已有足够观测但 `occupied_count == 0`，reason 为 `EMPTY_COLUMN`。active profile 中，空柱只有
同时满足以下条件才会成为 `FREE`：

1. 该格在二维 prior 中经 `3 x 3` 保守采样后全部 known-free；
2. `ground_elevation` 为该 XY 提供可信支撑高度；
3. 以该支撑高度为起点，车底到 `vehicle_height + headroom_margin` 的体素已知比例达到
   `min_headroom_known_ratio`。

通过时设置 `clearance_verified=1` 和 `empty_support_verified=1`。这解决的是 MID360 下视盲环内
“地面回波暂时看不到，但场地已测绘”的情况。它依赖静态场地未发生变化；如果先验声明地面存在，
而现场后来出现一个又恰好处于 LiDAR 盲区内的坑，单靠本链路仍不能在线发现。

#### C. 有 occupied return 的柱

1. 把相邻 occupied 体素组成多个竖直 run；
2. 先用最低 run 的下边界与本格 `ground_support_z_abs` 计算净空。边界相对 voxel center 的偏移由
   `headroom_voxel_inset_fraction * resolution` 决定：默认/实车为 `0.5` 的保守体素边界，规则
   仿真射线为 `0.0` 的中心估计；
3. 若该下边界已高于 `vehicle_height + headroom_margin`，则无论回波是
   水平薄层还是前缘厚立面，都只有在本格存在可信支撑、同列车身高度带已知比例达标时才作为
   顶棚通过，reason 为
   `OVERHEAD_CLEARANCE_OK`；
4. 高位 run 下方观测不足为 `HEADROOM_UNVERIFIED`，未开启上述授权为
   `GROUND_UNVERIFIED`，二者都不放行；
5. 若最低 run 进入所需车身高度带且厚于 `surface_height_delta_max`，则它是
   墙或低梁，直接 `HEADROOM_BLOCKED`；
6. 其余最低薄 run 才是 ground candidate；在 `body_bottom_clearance` 内相邻的
   低位 run 合并进地面；
7. 第一个更高的 run 被视为洞顶或悬空障碍；
8. 若 `ceiling_z - ground_z < vehicle_height + headroom_margin`，reason 为
   `HEADROOM_BLOCKED`；
9. 即使没有低顶，车体高度区间的已知比例不足也会成为 `HEADROOM_UNVERIFIED`；
10. 最低薄 run 的表面高度还必须与本格先验支撑高度之差不超过
    `ground_support_tolerance`，才能成为 `PASSABLE`。

所以悬空障碍的处理不是“看到高点就把整列永远封死”：高位 run 在所需净空以上时可以保留
通行；进入车体所需净空时则阻塞。系统仍只输出该 XY 能否通过，不生成改变车身 Z 的轨迹。

### 6.2 active 的可信地面高程门

active real/sim 都配置：

~~~yaml
projection:
  unknown_as_occupied: true
  bridge_observed_empty_for_ground_connectivity: false
  clear_robot_footprint_unknown: true
  near_field_prior_fill_enable: false
  prior_map:
    enable: true
    free_fills_unknown: false
    require_ground_support: true
    ground_support_tolerance: 0.08
~~~

`require_ground_support` 是硬安全模式，不是一个调参提示。启用后：

- ground candidate 不再通过“离机器人近、接近当前参考高度”自举为 seed；
- 每个 candidate 必须与本格已测高程相符，孤立浮板即使处于旧 seed 半径内也不能自证为地面；
- observed-empty 只有同时拥有本格支撑先验和已验证车体净空才可放行；
- `UNKNOWN` 即使有人误配 `unknown_as_occupied:false`，代码仍强制输出阻塞 mask；
- near-field fill、全局 `free_fills_unknown` 和 blind bridge 均不能绕过支撑门；
- 唯一的随车 bootstrap 特例也必须先通过二维 known-free、当前支撑高度匹配、八邻域支撑连续和
  零 occupied 证据四道门。实车包络只覆盖硬 footprint；RMUC2026 仿真包络额外覆盖一个
  `0.10 m` ROG seed 步长，用于跨越规则射线近场栅格空洞，不能复制到实车。

配置加载器还会拒绝 `require_ground_support:true` 与 `unknown_as_occupied:false` 或
`free_fills_unknown:true` 的组合。所选地图没有 `ground_elevation` 时节点可以启动用于观察，但不会
形成可执行通行域。当前 `highbay.yaml` 已提供已测平地的 `z=0` 支撑，因此 active MINCO 不再因缺少
高程块全局闭锁；该默认值不覆盖未来新增的坡道标定责任。

### 6.3 高程先验的含义

仿真地图按 map frame 写入：

~~~yaml
ground_elevation:
  default_height: 0.0
  patches:
    - name: ramp
      bounds: [0.442, -0.675, 1.641, 0.675]
      reference: [0.442, 0.0, 0.0]
      slope: [0.2085, 0.0]
    - name: platform
      bounds: [1.641, -0.675, 4.541, 0.675]
      reference: [1.641, 0.0, 0.25]
      slope: [0.0, 0.0]
~~~

patch 内的平面为：

~~~text
z = reference_z
  + slope_x * (x - reference_x)
  + slope_y * (y - reference_y)
~~~

这里的高程描述的是**可接触上表面**，不是 SDF box 的中心或局部厚度偏移。仿真坡道是
`1.225 x 1.35 x 0.08 m` 的倾斜 box，world pose 为
`(x,z,pitch)=(1.05,0.0857,-0.2054)`；变换后的上表面约从
`(x,z)=(0.442,0.0)` 连续上升到 `(1.641,0.25)`。平台 box 的 world pose x 为 `3.09147 m`，长度
为 `2.9 m`，所以几何前后缘约为 `x=1.64147 m` 和 `x=4.54147 m`；地图以毫米精度写成
`[1.641, 4.541] m` 的 platform support bounds，并以 `[1.641, 0.0, 0.25]` 为 reference。平台中心
高度为 `0.20 m`、厚度为 `0.10 m`，所以上表面为 `0.25 m`。SDF 坡顶与平台前缘、ramp/platform
support 的边界和高度差都限制在 `1 mm` 内，不再留下会被 required-support 门判作断裂的缝隙。
因此 ramp patch 必须在 `x=0.442` 以 `z=0.0` 为 reference；
把 box 的半厚度约 `0.039 m` 写入 reference 会令支撑先验整体高于真实坡面，并在平地到坡道的
入口制造虚假的高度跳变。

后出现的 patch 覆盖 `default_height`。高程只对二维 PGM 的 known-free 区有效；static unknown 或
occupied 没有支撑资格。因此 `default_height` 的范围必须已经测量，未测区、坑、断崖和不可跨越
台阶必须在 PGM 中保持 unknown/occupied，不能先画成 free 再期待高程层自动否决。

对所有准备放行的 required-support cell，包括有地面回波的 candidate、EMPTY 和 overhead，代码还
检查其 8 邻域中每个同样拥有可信支撑的 cell：

~~~text
planar_step = resolution * hypot(dx, dy)
allowed_step = max(max_ground_step,
                   tan(max_ground_slope_deg) * planar_step)
abs(support_z - neighbor_support_z) <= allowed_step
~~~

任一已知支撑邻边超过限制，本格即改为 `GROUND_UNVERIFIED/OCCUPIED`；因此测绘 patch 的陡跳不会
被平面 MINCO 直接跨过。没有支撑的相邻格自身仍按 required 规则阻塞。

该 YAML 表达的是“经测量并确认可行驶的静态支撑面”，不是传感器在线估出来的地形。它应由场地
测量/建图工具生成并做版本管理，不能凭肉眼手填后直接上车。

### 6.4 legacy seed、坡面 BFS 与 blind bridge

当 `require_ground_support:false` 时，兼容逻辑仍可使用机器人附近的多源 seed、8 邻域坡面 BFS
以及 bounded observed-empty bridge。bridge 实现可累计跨越连续空柱，受
`ground_connectivity_bridge_max_length` 限制；较长桥接还要求落点后的连续高度证据。它并非“只能
跨一格”。

这套逻辑只能验证回波几何连续性，不能区分短坡与形状相同的悬空斜板，也不能证明空柱下有轮地
支撑。实测仿真 MID360 在坡前出现的下视盲带约 `2.35 m`，把 bridge 增大到这个量级会同时打开
坑洞和浮板风险，所以 active real/sim 都明确将其关闭。bridge 参数保留用于 legacy 实验和负例
测试，不是本洞口方案的安全依据。

### 6.5 footprint 与近场补全的兼容边界

`near_field_prior_fill_enable` 在实车 active 保持 `false`。RMUC2026 仿真将它限制在车前
`1.40 x 1.00 m` 的短扫掠区，用于补偿规则射线被车体遮挡形成的零命中条带；即使开启，required-support
代码仍要求二维 prior known-free、测绘高程匹配且八邻域连续、整柱零 occupied，因此它不是二维 prior
无条件清 UNKNOWN。该补偿是仿真传感器模型特例，不得复制到实车。

`clear_robot_footprint_unknown:true` 则是一个范围受限的启动特例，用于处理车身自滤除后当前位置没有
足够 body-volume 射线的问题。required-support 下，一个格只有同时满足以下条件才会得到
`ROBOT_FOOTPRINT_CLEAR/FREE`：

1. 位于当前配置的自车包络内（实车有效包络 `0.40 x 0.30 m`，仿真 bootstrap 包络
   `0.52 x 0.51 m`）；
2. 二维 prior 为 known-free；
3. 本格测绘支撑高度与机器人当前参考地面高度之差不超过 `0.08 m`；
4. 八邻域支撑连续；
5. 该柱没有任何 occupied voxel，occupied 证据在进入本分支前已经否决资格。

ROG odom 表示 MID360 原点，因此使用 `robot_footprint_clear_offset_y:-0.18` 把包络移回底盘中心。
栅格化还加入旋转后半个 cell 的投影 padding，确保连续矩形边界相交的格不会因“格心刚好在矩形外”
而漏清。padding 只扩大待审查格集合，不绕过上述支撑和 occupied 门。

这个特例每帧按当前 pose 重算，旧位置不会保留 free trail；它只证明“机器人已经实际占据的当前
车身空间”，不证明前方地面，也不允许 MINCO 越过未观测净空。相应单测覆盖高度匹配、错高支撑、
支撑不连续、occupied 否决、随车移动不留轨迹和边界 cell 栅格化。

### 6.6 时间稳定与二维 mask filter

动态 occupied 立即进入阻塞。实车在 raw classification 转回非 occupied 后用
`obstacle_hold_time: 0.50 s` 保守保持，仿真规则射线则为 `0.0 s`；两者都保留
`hysteresis_count: 2` 的类型切换确认。实车 hold 只能依据现场 bag 的连续漏检上界缩短。

投影后还启用二维 mask filter：`fill_occ_min: 7` 会把被至少 7 个 occupied 邻居包围的
UNKNOWN/FREE 小孔保守填上；`denoise_occ_max: 0` 只把完全孤立的 occupied type 改成 UNKNOWN。
由于当前 `unknown_as_occupied: true`，后者在最终 mask 中仍然阻塞，并不是把孤立点直接变成自由。
`ROBOT_FOOTPRINT_CLEAR` cell 不参与小孔填充。

### 6.7 静态二维图与高程先验

Nav2 map server 和 ROG prior-map 使用同一个顶层 `map:=...yaml`。launch 将参数模板中的
`<rog_prior_map_yaml>` 替换为最终选择的 YAML。

ROG loader 使用 YAML/PGM 中的：

- `mode: trinary`；
- `resolution`；
- `origin`，包括 yaw；
- `negate`；
- `free_thresh` 和 `occupied_thresh`。
- 可选的 `ground_elevation.default_height`、16 位 `grid` 与平面 `patches`。

YAML 中的相对 `image` 路径按 YAML 所在目录解析。实车
`map/reality/highbay.yaml` 的 `free_thresh: 0.196` 用来让灰度 205 保持 UNKNOWN，而不是被
当成 known-free。

`ground_elevation.grid` 与占据 PGM 共用尺寸、分辨率、origin 和行坐标方向。每个 16 位 PGM 值按
`z = offset + scale * value` 解码；`no_data` 表示该单元没有可信地面，不能回退到
`default_height`。patch 按文件顺序最后覆盖 grid/default，可用于经测量的局部修正。grid 尺寸不匹配、
`scale <= 0`、编码越界或文件损坏都会在 ROG-map 启动时直接报错。

每个 ROG cell 不只采一个中心点。它在中心、四边中点和四角附近执行 `3 x 3` 保守采样：

- 任一点 static occupied，整格 static occupied；
- 九点全部 known-free，整格才是 static known-free；
- 其他情况为 static unknown。

### 6.8 动态/静态/支撑融合优先级

active 中 `free_fills_unknown:false` 且 `require_ground_support:true`。融合规则为：

| 二维 prior | 高程/动态柱 | 融合结果 |
| --- | --- | --- |
| occupied | 任意 | 强制阻塞 |
| unknown | 任意 | 保持动态阻塞结果，不能获得支撑 |
| known-free | 无本格高程 | 强制缺支撑，不能放行 |
| known-free | 高程存在，但地面回波高度不吻合 | `GROUND_UNVERIFIED`，阻塞 |
| known-free | 高程存在，空柱车体净空观测不足 | `HEADROOM_UNVERIFIED`，阻塞 |
| known-free | 高程吻合或空柱有支撑，且三维净空通过 | 动态层可通行 |
| known-free | 车体高度内有动态 occupied | 动态障碍硬否决 |

在 required-support 模式中，即使直接调用融合 API 并传入 near-field 标记或
`free_fills_unknown:true`，代码也不执行二维自由填充。二维 static known-free 只是高程查询的必要
条件，不单独产生自由空间。

### 6.9 prior TF 与 fail-closed

prior 投影查询 `map <- rog_frame`。XY 坐标使用平移和 yaw；支撑高度按
`z_rog = z_map - transform.tz` 换算。实车 transform timeout 为 `0.50 s`，仿真为 `0.75 s`，
future tolerance 都是 `0.20 s`。

以下任一情况会让 prior transform/cache 失效，并把 fused layer 全部置为阻塞：

- prior 文件未加载或存储无效；
- TF 查询失败；
- TF stamp 无效、过旧或超前；
- TF 含非有限量、无效四元数，或 map 与 ROG frame 之间 roll/pitch 绝对值超过 `1e-5 rad`；
- 投影 cache 尺寸不匹配或未 ready。

XY/yaw/tz transform 变化会使 prior 投影 cache 失效并按当前滑窗重建；required-support 模式同时
强制整层重新分类，避免旧支撑高度残留。滑窗移动时复用重叠区并重算新区域。当前高程模型只支持
水平 map/ROG frame 关系；不能靠放大 timeout 或放宽姿态阈值掩盖 TF 建模错误。

### 6.10 二维 field

最终 fused mask 中 `0` 为障碍、`1` 为可通行。`DynamicLayer` 对其计算二维正/负 EDT，并提供
signed distance 和 XY gradient：

~~~yaml
rog_map.esdf.enable: false
rog_map.field.enable: true
rog_map.field.inflation_radius: 0.0
rog_map.field.max_distance: 4.0
rog_map.field.min_distance: -2.0
rog_map.field.interpolation: quadratic
~~~

距离场采样以 `origin + (index + 0.5) * resolution` 为 cell center；query adapter 对半格边界
做一致钳位。`gradient.z` 恒为零。

所以这里的 field 是“由 3D 柱分类生成的 2D signed distance field”，不是 active 3D ESDF。

### 6.11 ROG 诊断话题

主要全局绝对话题如下：

~~~text
/rog_map/raw_occupied
/rog_map/occupied
/rog_map/unknown
/rog_map/inflated_occupied
/rog_map/inflated_unknown
/rog_map/frontier
/rog_map/layer_value
/rog_map/layer_value_dynamic
/rog_map/layer_value_static
/rog_map/layer_type
/rog_map/layer_confidence
/rog_map/layer_height_delta
/rog_map/headroom
/rog_map/headroom_known_ratio
/rog_map/clearance_status
/rog_map/field
/rog_map/decay_cells
/rog_map/map_bound
/rog_map/self_filter_box
~~~

`layer_value_dynamic` 是柱分类/morphology 后、prior 融合前的结果；
`layer_value_static` 是保守投影的静态信息；`layer_value` 是 MINCO 实际查询的融合结果。
`headroom_known_ratio` 只表示车体高度区间的已知体素比例，不能单独证明有地面、无 occupied
或最终可通行。

`clearance_status` 的调试编码为：

| 值 | 含义 |
| ---: | --- |
| `0` | 无地面候选，净空/支撑未验证 |
| `1` | 非 required 模式中，地面和净空通过旧连通逻辑 |
| `2` | 非 required 模式中，经 bounded bridge 连接的地面 |
| `3` | ground candidate 与可信高程吻合，且净空通过 |
| `4` | 无地面回波，但可信高程存在且车体净空观测通过 |
| `5` | 当前自车 footprint 特例：known-free、匹配且连续的可信支撑、零 occupied，当前位置净空成立 |
| `25` | 无地面候选，但可信支撑上的顶棚/高位前缘净空通过；required 模式同样可出现 |
| `50` | 地面已验证但净空失败 |
| `100` | 地面候选没有可信支撑/未通过验证 |

该 topic 是诊断编码，不是 Nav2 cost。最终能否规划仍以 `/rog_map/layer_value`、field query 和
planner footprint 安全检查为准。

## 7. MINCO 规划

### 7.1 PRIORMAP 中三类 query 的职责

`PlannerModeContext` 在 `PRIORMAP` 下配置：

~~~text
globalQuery()   = Nav2 global costmap
dynamicQuery()  = FrameAwareRogQuery（ROG fused field）
sparsifyQuery() = Nav2 global costmap
~~~

query 角色与当前机器人 pose 的来源是两件事。`getRobotPose()` 总是先要求有效的
`planning_frame <- base_link` 物理 TF：

- `PRIORMAP` 在 costmap pose 可用时保留 Nav2 costmap 给出的控制点**位置**，以保持全局
  costmap/路径的平移语义，但用上述物理 TF 的 orientation 覆盖 `gimbal_yaw_fake` 的合成朝向；
  若只有 costmap pose 获取失败，则退回已经校验过的完整物理 pose；
- `EXPLORATION` 不经过 costmap 控制点，直接使用物理 TF 的完整 position 和 orientation；
- 两种模式都不会在物理 TF 失败时继续用 yaw-cancelled orientation 做 footprint、轨迹接续或
  当前位姿安全检查。

职责不是互相替代：

- `globalQuery` 负责静态全局拓扑和全局 SMAC/A* seed；
- `dynamicQuery` 负责局部实时三维柱投影、距离场、优化障碍代价和最终安全检查；
- `sparsifyQuery` 用静态全局代价地图检查全局 seed 的视线稀疏化；每条捷径还需通过完整 footprint
  的动态安全检查，不能将动态绕障折线重新拉直穿墙。

active Nav2 global costmap 按 `static -> rog_dynamic_obstacle -> inflation` 合并，且
`smac_2d.use_esdf_cost=false`。其中 ROG costmap plugin 只接受实测 occupied 障碍，不把未知支撑或
未知净空投影成二维墙；后置 inflation 的 `1..253` 软代价直接进入 SMAC 路径代价。当
`priormap.dynamic_global_obstacle.enable=true` 时，Astar/SMAC 还会额外查询
`FrameAwareRogQuery` 的局部 ESDF，在 `collision_distance` 内避开明确的 ROG occupied evidence。
该动态门只接受同一柱中存在有限 `occupied_z=[min,max]` 且分类原因为
`SOLID_VERTICAL_WALL`、`AMBIGUOUS_OCCUPIED` 或 `HEADROOM_BLOCKED` 的实测障碍证据，再用
`collision_distance` 对其周围搜索单元做硬门。`HEADROOM_UNVERIFIED`、`GROUND_UNVERIFIED`、
`UNKNOWN_AS_OCCUPIED`、去噪后未知和 ROG 窗口外查询失败不允许改写全局拓扑，否则实车激光盲区会将
起点周围封死。
占据证据和距离在同一轮搜索中按 cell 缓存，邻域膨胀不重复查询相同单元；下一轮搜索重新获取证据。
动态 ROG 的约束仍同时进入局部 corridor、MINCO 优化和最终安全门；全局 path 仍不是覆盖滑窗之外的
动态无碰证明。关闭该开关即可恢复原来的“静态全局 + 局部动态安全”行为。

SMAC 对软代价的当前配置为实车 `cost_penalty=4.0`、仿真 `2.0`，且
`use_quadratic_cost_penalty=false`。旧实现固定平方归一化代价，使外圈常见 cost=45 只增加约 6%
行程代价；实车线性配置将其提高到约 71%，使膨胀外圈能
实际推动路径离墙，同时不把窄洞的重叠软代价抬到默认就绕远路的程度。cost `>=253` 仍不可通行，
明确 occupied 的动态硬门也不受该软权重影响。

`Nav2CostmapQuery` 在读取 cell 和复制 charmap 时持有 `Costmap2D::getMutex()`。SMAC/A* 使用
同一次锁内复制出的自有快照；如果 costmap 在尺寸读取与复制合同间发生不一致，搜索失败，而不是
继续使用部分或悬空内存。

### 7.2 从全局 seed 到局部轨迹

全局路径生成后：

1. 按当前 `lookahead_dist: 1.5 m` 截成滚动局部 seed；
2. 按 ROG 滑窗边界裁剪，并保留 `rog_boundary_margin: 0.30 m`；
3. 沿 seed 以不大于半个 ROG cell 的步长，用当前实测 yaw 检查完整旋转 footprint；
4. 遇到第一个 occupied、UNKNOWN、越界、TF/field 失败或过期快照时，只保留其前方最后一个已验证
   安全点；若连两个安全点都没有则本轮规划 fail-closed；
5. 若终点由观测前缘裁剪得到，MINCO 的终端速度和加速度强制为零，时间分配也按停车端点处理；
   yaw 始终从当前实测车身朝向优化到 NavigateToPose 的显式目标朝向，不再从路径切线重建终点 yaw。
   因此全向横移不会在每次重规划时把矩形车身逐步转向 UNKNOWN 边缘；
6. 通过全局 costmap query 做可视线稀疏化；候选捷径的峰值/平均 cost 不得明显高于原 SMAC 子路径，
   并沿每条候选捷径以半个 ROG cell 的步长检查当前 yaw 的完整 footprint。失败时拆分回原始拐点，
   拆分后的两条边都必须检查；不允许补入未经检查的拐点/终点；
7. 结合 ROG 动态 field 构造 corridor 和障碍代价；
8. 生成分段五次 MINCO position trajectory 和独立 yaw trajectory；
9. 通过发布前安全检查后，发布 `MpcPositionCommand`。

滚动重规划会明确选择 HOT 或 COLD 起点，二者都必须经过同一套优化后安全门：

- 只有旧轨迹的当前时间仍落在有效区间、位置跟踪误差不超过
  `0.30 m + 0.20 s * ||current_speed_xy||`、速度误差不超过 `0.35 m/s`，并且旧轨迹速度与新 seed
  首段方向的点积不小于 `0.5` 时，才允许 HOT_START；
- 时间越界、位置/速度误差过大或方向不一致时，`determinePlanningState()` 实际返回
  `COLD_START`，不会只打印降级日志后继续复用旧轨迹；
- COLD_START 从当前实测位置和速度开始、加速度置零，并清空旧的 waypoint/duration warm-start
  guess；
- HOT_START 的边界状态也使用实测位置、实测速度和零加速度；旧轨迹只作为优化器内部的 waypoint/
  duration 初值，不再把预测动量伪装成当前车体状态。

position 的 HOT/COLD 选择不改变 yaw 起点合同。每条替换轨迹的 yaw seed 都重新取
`planning_frame <- base_link` 的当前实测物理朝向，yaw 角速度置零；不会从旧 position 速度或当前
平移速度方向推导车头。全向底盘可以横移，坡面也可能侧滑，这些速度方向都不是车体 yaw。

COLD 是 MINCO 内部的重规划初值回退，不是切回 legacy controller，也不会绕过 ROG、动力学、yaw
footprint 或 planning token 检查。

`1.5 m` 是局部滚动上限，不是“从当前位置一次验证到最终目标”，也不是要求单帧 LiDAR 必须证明
完整 1.5 m。实际终点可以在已观测净空前缘提前停车；车辆前进、点云补齐后 FSM 再向前延伸。
这个裁剪不会把 UNKNOWN 改成 free，也不会降低 `min_headroom_known_ratio`，因此坡道逐段展开与低
浮空障碍物硬阻塞使用同一份证据。

### 7.3 两点 Path、内部 generation 与显式 planning token

`MincoPlanner::createPlan()` 返回的两点 `nav_msgs/Path` 是 Nav2 的合同载体和 controller 授权载体，
**不是可执行 MINCO 轨迹**。内部 FSM 仍以 20 Hz 检查状态和安全条件，但当前 profile 的成功
滚动重规划由 `successful_replan_period=0.50 s` 限制为约 2 Hz，失败生成由
`failed_replan_retry_period=0.25 s` 限制为最多 4 Hz；全局搜索和局部优化都异步完成，
并通过 `MpcPositionCommand` 发布。

当前实现同时维护两个不同概念：

- 内部 `planning_session_` generation 是并发正确性屏障；pending goal、global path、优化结果、缓存
  轨迹和在线安全结果都核对它，旧后台任务晚完成也不能覆盖新状态；
- 对外 `planning_stamp_` 是显式规划会话 token，以 `builtin_interfaces/Time` 承载，在
  `Path.header.stamp` 与 `MpcPositionCommand.planning_stamp` 之间做精确匹配。

token 不是普通的消息时间戳。其更新规则等价于
`max(1 ns, ros_now, previous_token + 1 ns)`，所以即使仿真时间暂停或回退，仍保持非零、严格单调；
零值保留为非法 token。

一次 `createPlan()` 先记录本次 `request_stamp`、规范化 start/goal 到 planning frame，再按目标语义
决定是否建立新会话：

1. 若 frame 相同、三维位置差不超过 `1e-3 m`、yaw 差不超过 `1e-3 rad`，且已有目标会话仍有效，
   则判为同一目标；复用原 generation 和原 token，不清 position/yaw trajectory，不清 global path，
   不重新置 pending goal，也不发布 BLOCK。
2. active BT 的 `RateController hz="3.0"` 因而可以对同一目标持续调用 `createPlan()`。每次仍返回
   新的两点 Path 消息，但 `Path.header.stamp` 保持同一个 token；这只是 Nav2 plan refresh，不会令
   controller 周期性刹停。
3. 若目标改变，则开始新 generation、推进 token、使旧轨迹失效、清空旧 global path、设置新
   pending goal，并立即发布携带新 token 的 `BLOCK_COMMAND`。
4. 目标完成以及 planner 的 activate/deactivate/cleanup 边界会推进到新的非零单调 token，
   清除相应会话状态并发布 BLOCK。目标完成只接受当前 expected generation，过期 completion 无效。
5. Nav2 Jazzy 的 `GlobalPlanner` 插件没有 Navigate action 取消回调；active 因此以
   `request_lease_timeout` 的 steady-clock 心跳租约补齐这一生命周期。10 Hz BT 的每次
   `createPlan()` 刷新租约；取消、BT 退出或进程链断后无新请求，租约到期便只推进
   一次 token、清缓存并向 authoritative/debug 两个轨迹 topic 发布同一 BLOCK。

FSM 的内部到点门和 Nav2 的最终 action 到点门是两个不同阈值。当前两套 active profile 使用
`traj_goal_tolerance=0.15 m`，而 `general_goal_checker.xy_goal_tolerance=0.20 m`。内部 FSM 可能在
自己的阈值内清轨迹并发布 BLOCK，因此前者必须严格小于后者；若反向配置成例如 `0.30 > 0.20`，
车辆可能停在 Nav2 成功区外，后续会话又被 FSM 立即清掉，形成到点闭锁。

各时间字段必须按下面的语义读取，不能互换：

| 字段 | 语义 | 同一目标 10 Hz refresh 时是否变化 |
| --- | --- | --- |
| `nav_msgs/Path.header.stamp` | planning token；授权合同，不用于 freshness | 否 |
| 两点 Path 中每个 `PoseStamped.header.stamp` | 本次 `createPlan()` 的 `request_stamp`，用于 TF/调试时序 | 是 |
| `MpcPositionCommand.planning_stamp` | 与 Path 精确匹配的 planning token | 否，直到会话推进 |
| `MpcPositionCommand.header.stamp` | 本次 NORMAL/BLOCK 的实际发布时间 | 是 |
| 每个 `PositionCommand.header.stamp` | 复制外层消息 header，表示本次轨迹发布时刻 | 是 |

因此不能用 `Path.header.stamp` 判断 Path 是否“新鲜”，也不能用不断变化的
`MpcPositionCommand.header.stamp` 关联会话；会话匹配只看 `planning_stamp` token，NORMAL 轨迹的
新鲜度只看消息 header 和本地 steady-clock 接收年龄。

### 7.4 优化与动力学硬检查

MINCO 的速度、加速度障碍项在 L-BFGS 目标中首先是软积分惩罚，不能仅凭优化器“收敛”断言动力学
满足上限。当前实现随后使用每段多项式的连续极值计算：

~~~text
Trajectory::getMaxVelRate()
Trajectory::getMaxAccRate()
  -> Piece::getMaxVelRate()/getMaxAccRate()
  -> polynomial roots over the full piece interval
~~~

若峰值超限，统一放大各段时间，再重新生成多项式，最多
`time_allocation_iters: 15` 次。时间缩放依据速度比例和加速度比例平方根，并留有缩放 margin。

以下情况会拒绝轨迹：

- 起终点 PVA 已超过配置上限；
- 任一时间、系数或峰值非有限；
- piece duration 非法；
- 15 次后仍超限。

日志记录 `retime_iters`、peak velocity 和 peak acceleration，调参时应以这些连续极值为准，
不能只抽样曲线。

### 7.5 显式目标 yaw 不等于三维轨迹

planner 始终使用 NavigateToPose 请求中的显式 orientation 作为 goal yaw，并按当前实测物理 yaw
展开到最短角差。yaw optimizer 以 `free_start=false, free_goal=false` 运行：起点不会被 position
初段切向替换，目标也不会被 position 末段切向重新覆盖；内部 waypoint 只负责平滑连接这两个固定
边界。这避免把全向横移或坡面侧滑当作车头方向，也避免每次滚动重规划都按新的路径切线累积旋转。
任何 yaw 优化失败都退回从当前物理 yaw 开始的常值 yaw trajectory，并继续经过相同的旋转 footprint
安全检查。

position trajectory 仍为平面轨迹，发布命令中的 position `z=0`；pitch 不会进入一个
`x,y,z,roll,pitch,yaw` 的三维优化问题。

## 8. 轨迹安全边界

### 8.1 软距离、硬占据与 footprint

需要区分两个参数：

~~~yaml
MincoPlanner.minco_optimizer.safe_dist: 0.30
MincoPlanner.minco_optimizer.collision_dist: 0.0
~~~

- `safe_dist: 0.30 m` 是优化器的障碍软代价尺度；
- `collision_dist: 0.0 m` 被传给发布前/在线 `TrajectorySafetyChecker`；
- 硬检查仍先拒绝 out-of-map、UNKNOWN、LETHAL、INSCRIBED 和 query 失败；
- 即使阈值为 0，仍必须完成 ESDF query，并拒绝 TF、field、时间戳、freshness 和非有限距离错误；
- 阈值为 0 时，硬碰撞以已经检查过的精确投影 cell 为准，不再用邻接 occupied 导致的轻微负插值
  距离扩大 footprint；只有 `collision_dist > 0` 时才额外要求 `distance > collision_dist`；
- 因而设为 0 只是避免在矩形 footprint 外再叠加一圈重复径向膨胀，不是关闭碰撞检查。

优化后先执行 position centerline 检查；yaw 轨迹生成后，再执行最终旋转矩形 footprint 检查。

实车参数为 `footprint_length: 0.30`、`footprint_width: 0.20`、
`footprint_margin: 0.05`，最终硬 footprint 是 `0.40 x 0.30 m`；RMUC2026 仿真为
`footprint_length: 0.22`、`footprint_width: 0.21`、`footprint_margin: 0.05`，最终为
`0.32 x 0.31 m`，覆盖仿真轮组碰撞包络。两侧 `sample_dt` 均为 `0.05 s`。每个时间样本按规划 yaw 旋转并采
footprint 内部/边界，轨迹终点无论是否落在采样步上都会强制检查。

同一轨迹时刻的 footprint 不再通过 `worldToMap -> value -> query` 三次独立读取拼接结果，而是一次
coherent batch query：

1. checker 先生成该旋转矩形的全部内部/边界采样点，并显式加入中心点；
2. `FrameAwareRogQuery` 只解析一次 `planning_frame -> rog_frame` TF，并用同一变换处理整批点和梯度；
3. `QueryAdapter` 在批次开始时捕获一个不可变 `MapSnapshot`，同批所有点的 projected cost、ESDF、
   snapshot stamp 和 sequence 都来自这一个快照；
4. checker 用同一个 query time 解释整批结果，并在结果处理完成时再次按 completion time 检查
   freshness；检查本身耗尽 freshness 预算也会 fail-closed。

这个合同消除了 cost 与 ESDF 跨快照的 TOCTOU，也避免对 footprint 中每个点重复查 TF 和反复锁
snapshot。这里的原子单位是“一个轨迹时刻的完整 footprint batch”，不是宣称整条多秒轨迹只使用
一次永久冻结的地图。

### 8.2 地图 freshness

每个 ROG query result 带有 sensor snapshot stamp。安全检查拒绝：

- query 不存在；
- snapshot stamp 非有限、为零；
- snapshot 过旧；
- snapshot 超前 ROS time 超过 `0.05 s`。

| 参数 | 实车 | 仿真 |
| --- | ---: | ---: |
| `safety.map_timeout` | `0.50 s` | `2.20 s` |
| `safety.future_tolerance` | `0.05 s` | `0.05 s` |

仿真 timeout 较宽是为了容纳高密度 Gazebo ray 在主机负载下的更新延迟，不代表实车可照搬。
安全 timer 以 20 Hz 复查当前实际 footprint 和缓存 position+yaw trajectory 的剩余段；已经驶过的
历史前缀不会因为 current-footprint bootstrap 随车移动而被误判为新碰撞。每轮在线检查重新取得
当前实际 pose，并用与 controller 初次接管轨迹相同的规则寻找空间 pickup：先选 `0.05 s` 离散样本
中的二维最近点，再只在其前向线段投影满足 `-0.5 < projection < 1.0` 时细化时间。这个安全起点
**不使用 wall elapsed，也不再回退一个采样步**。

checker 随后先检查“实际 pose/yaw -> 空间 pickup pose/yaw”的 swept footprint；平移与旋转合成的
最远角点运动每次不超过半个 ROG cell。通过后，再从空间 pickup 起检查完整 position+yaw suffix 和
强制终点。controller 对新 `trajectory_id` 同样从空间 pickup 起步；对同一 trajectory，空间游标和
参考时间游标都不倒退，但时间推进会被限制为最多领先当前单调空间 pickup `0.25 s`。因此 safety
从纯空间 pickup 开始检查不会晚于 controller 当前参考，覆盖的是 controller 可能执行 suffix 的
超集，而 swept join 负责覆盖实际跟踪偏差。当前 pose、空间进度、yaw 或轨迹时间非有限，或者
`planning_frame <- base_link` 物理 TF 无法取得时均 fail-closed。失败会发布 BLOCK 并锁住运动授权。

首次失败日志包含 reason、轨迹时刻/总时长、center、实际 query point、footprint offset、yaw、
distance、safe distance、cost、query status、snapshot age 和 planning/ROG frame。常见 reason
包括 `OUT_OF_MAP`、`COSTMAP_UNKNOWN`、`COSTMAP_LETHAL`、
`INSUFFICIENT_CLEARANCE`、`STALE_SNAPSHOT`、`FUTURE_SNAPSHOT` 和 `QUERY_FAILED`。

### 8.3 当前 fail-closed 链

active 模式的运动授权依次经过：

~~~text
cloud/odom frame 与时间门
  -> 3D occupancy
  -> 地面/净空/连通投影
  -> 静态先验三态融合
  -> ROG snapshot freshness
  -> MINCO 连续动力学极值
  -> yaw 后矩形 footprint
  -> planner NORMAL/BLOCK（发布 header + planning_stamp token）
  -> MPC trajectory session gate（精确 token 授权）
  -> odom/trajectory freshness 与数值检查
  -> QP solve
  -> solve 后 generation 复核
  -> fake_vel_transform odom watchdog
  -> /cmd_vel
~~~

任一级失败的预期行为是零速/BLOCK/拒绝新轨迹，不是切回 legacy。
active 的 motion behavior action 不加载，`NavigateThroughPoses` navigator 不加载；这两条是
系统入口级安全边界，不只是默认 BT 约定。

周期重规划失败不再等价于立即让 controller 的接收 watchdog 过期。若新 seed、optimizer 或候选安全
复核失败，planner 只会在以下条件全部满足时截取并续发当前缓存轨迹的剩余段：取得最新物理车体 pose、
用当前 ROG snapshot 重新检查完整 position+yaw footprint、会话 token/轨迹 generation 仍一致、剩余
时长至少两个控制步且没有 BLOCK latch。续发段使用新 trajectory ID 和当前时间戳；任一条件失败仍按
原 fail-closed 逻辑 BLOCK。它解决的是“新优化偶发失败造成 `TRAJECTORY_RX_STALE`”，不是允许复用未经
当前地图验证的旧路径。碰撞类拒绝还受独立时长上限约束：仿真只允许续发不超过 `0.75 s` 的剩余段，
实车为 `0.40 s`；更长的旧尾段即使当前检查通过也立即 BLOCK，避免坡上跟踪误差继续累积到边界。

MINCO 内部的 escape recovery 也不是安全旁路。它每次构造 `0.5 s` 短轨迹后，使用与
正常 MINCO 轨迹相同的 position + yaw + 旋转 footprint + ROG snapshot freshness 检查，
强制采样当前位姿、预测段和终点。通过后写入 `last_traj_` / `last_yaw_traj_`，继续被
20 Hz safety timer 监视；失败则清缓存、推进 generation、锁止并发 BLOCK。

紧急 BLOCK 先只构造一份消息，再同时发布到 authoritative `/minco/opt_path` 和诊断
`/minco/backup_path`；两者的 header、planning token、trajectory ID 和轨迹内容一致。
controller 只订阅 opt topic，backup topic 不能驱动底盘。

## 9. MincoMpc 与底盘输出

### 9.1 坐标系合同

active local costmap `global_frame` 是 `odom`，MincoMpc 在 configure 时将其作为内部
`global_frame_`：

1. MincoPlanner 的 `MpcPositionCommand.header.frame_id` 是 `map`；
2. controller 将 position、velocity、acceleration、jerk 和 yaw 全部变换到 `odom`；
3. `/odometry.header.frame_id` 必须严格等于 `odom`；
4. odometry pose 表示 parent 到 child 的位姿，twist 按 ROS 合同位于 `child_frame_id`；
5. controller 用完整四元数把 twist 旋到 `odom`，并应用配置的 LiDAR 杆臂项；
6. MPC state、reference、QP control 全部位于 `odom`；
7. `computeVelocityCommands()` 返回 world/odom 对齐的 `TwistStamped`。

controller 不使用 Nav2 传入 pose 的 synthetic yaw 作为 MPC state，因为导航 base frame
`gimbal_yaw_fake` 消除了云台 yaw；将它与 odom-frame reference 混用会产生横向方向错误。

### 9.2 当前 MPC 配置

两套部署共同值：

~~~yaml
dt: 0.05
lookahead_time: 0.50
# horizon = ceil(lookahead_time / dt) = 10
control_delay_compensation: 0.05
reference_progress_max_lead_time: 0.25
Q: [3.0, 3.0, 2.0]
q_along: 3.0
q_cross: 12.0
R: [1.5, 1.5, 1.0]
odom_timeout: 0.25
trajectory_timeout: 1.50
future_stamp_tolerance: 0.05
max_planar_speed: 1.0  # 实车为 0.7
~~~

`q_cross > q_along` 让横向误差比沿轨误差更重。速度和加速度约束的实车/仿真差异见第 10 节。
`vx/vy` box constraint 只限制各轴，不能限制对角速度；solver 因此在预测和输出提交前把平面速度投影到
`max_planar_speed` 圆盘。该值必须与 planner `max_velocity` 一致，否则制动距离和近场授权范围的预算失效。
MINCO position 仍是 2.5D 平面多项式：坡上 odometry 的垂向速度不会写入优化边界，冷启动平面速度也会
先限到 planner `max_velocity`，避免以超限初速度直接触发 optimizer `ret=1`。

reference 进度同时使用空间和时间，但二者的角色不同。收到新的 `trajectory_id` 时，controller
以机器人到离散 command 的空间最近点为 pickup，并按前向线段投影细化；基础参考游标就从该空间
pickup 开始，不会按消息发布后的墙钟年龄跳过新轨迹。只要 `trajectory_id` 不变：

~~~text
spatial = max(current_spatial_nearest, previous_spatial)
time_candidate = previous_reference + elapsed / planner_dt
lead_limit = spatial + reference_progress_max_lead_time / planner_dt
reference = max(spatial, previous_reference, min(time_candidate, lead_limit))
~~~

随后所有索引限制在 `[0, cmds.size()-1]`。当前 `planner_dt=0.05 s`、
`reference_progress_max_lead_time=0.25 s`，所以同轨迹基础 reference 单调不倒退，但最多领先单调空间
pickup 5 个样本；这既能越过静止轨迹速度为零的 `t=0` command，又不会让短停车轨迹按墙钟独自跑到
终点。这里限制的是基础参考游标，后续 `control_delay_compensation` 和 MPC horizon 仍按各自参数从
该游标向前取预测参考。BLOCK、生命周期清理、无效输入或轨迹会话切换会清除 tracked reference
状态；下一条新轨迹重新从其空间最近点开始。

### 9.3 显式 planning token 的 controller 状态机

`MpcPositionCommand.command_flag`：

~~~text
NORMAL_COMMAND = 1
BLOCK_COMMAND  = 137
~~~

controller activate 后 gate 为空且保持阻塞。它把 `Path.header.stamp` 和
`MpcPositionCommand.planning_stamp` 都转换为纳秒整数，零值非法，并执行以下合同：

1. `setPlan()` 收到非空、非零 token 的新 Path 时，将该 token 授权，清除上一会话轨迹并保持
   BLOCK，直到收到匹配的 NORMAL；低于已经公告 token 的旧 Path 直接拒绝。
2. `setPlan()` 再次收到**已经授权的同一 token** 时返回 `REFRESHED`，保留当前轨迹、授权和运动
   状态，不推进 controller trajectory generation，也不输出新的零速。这正是同目标 10 Hz
   `createPlan()` 不造成周期性 BLOCK 的关键。
3. 空 Path 或零 token 关闭 gate、清轨迹并保持阻塞。
4. 携带新 token 的 BLOCK 先公告 `expected_token`，撤销旧 token 的 Path/轨迹授权并清轨迹；相同
   token 的安全 BLOCK 也立即令输出阻塞，但保留该 token 的 Path 授权，以便随后匹配的安全轨迹恢复。
   小于 `expected_token` 的旧 BLOCK 被拒绝，不能回滚新会话。
5. NORMAL 只有在 token 等于当前 `authorized_token` 时才能进入控制缓存；旧 token、未知 token、
   零 token 一律拒绝，且迟到的旧消息不能覆盖当前会话。

新目标建立时，planner 通常先发布 BLOCK，再从 `createPlan()` 返回 Path，但 DDS 回调和 FSM 是异步
的，因此同 token 的 NORMAL 可能先于 controller 的 `setPlan()` 到达。若 BLOCK 已公告该
`expected_token`，gate 会把这条 NORMAL 暂存为 pending、继续输出零速；匹配 Path 到达后，仅当
pending 的本地接收年龄和外层 `MpcPositionCommand.header.stamp` 仍在
`trajectory_timeout` 内，才将其提升为当前轨迹。Path 先到、BLOCK 后到的同 token 顺序也受支持；
后续仍须有匹配 NORMAL 才能解除 BLOCK。

这里有两条相互独立的检查：

- **会话归属**：只比较 `Path.header.stamp` 与 `MpcPositionCommand.planning_stamp` token；
- **NORMAL 消息 freshness**：继续检查外层 `MpcPositionCommand.header.stamp`、未来时间容差和本地
  steady-clock 接收年龄；BLOCK 作为撤权消息按 token 处理，不等待轨迹 freshness 判定。

未知 flag、空/非有限 command、NORMAL 的过期/未来外层 header、无效 token 都会 fail closed。也就是
说，显式 token 解决“属于哪次目标”的问题，NORMAL 的 outer header 与 receive age 解决“这条轨迹
现在还能不能用”的问题，二者缺一不可。

odom 过期、trajectory 接收超时、frame 错误、无 reference、QP 失败、命令非有限，或 QP 求解期间
trajectory generation 已变化，均输出零速。`/minco/cmd_vel_mpc` 是 controller 内部 raw command
诊断发布，不是绕过 Nav2 的另一条底盘控制线。

### 9.4 controller 到 `/cmd_vel`

~~~text
MincoMpc return TwistStamped
  -> controller_server
  -> /cmd_vel_controller
  -> velocity_smoother
  -> /cmd_vel_nav2_result
  -> fake_vel_transform
  -> /cmd_vel
~~~

active 中 `fake_vel_transform.use_latest_odom_for_cmd: true`。它用每一帧通过校验的最新 odom yaw
把 world/odom 对齐平移速度旋到底盘坐标，要求 odom：

- pose、orientation、twist 均有限；
- 四元数有效；
- stamp 非零且年龄不超过 `0.20 s`；
- 未来不超过源码默认 `0.05 s`。

没有 fresh odom 时，新命令直接变零；50 Hz watchdog 会在已经输出过非零命令后 odom 断流时主动
补发 stop。legacy/shadow 则保持 `use_latest_odom_for_cmd: false` 的历史同步路径。

实车随后由 `gimbal_navigation_bridge` 订阅 chassis-frame `/cmd_vel`，使用 NavToGimbalV2 原样发送
`linear.x/y`。坐标旋转和符号只允许在 `fake_vel_transform` 完成，串口层不得再次交换或取反。
普通模式的通信线速度比例为 `1.0`；`follow_mark=0` 的特殊路段仍可使用独立比例降速。

仿真中的 `/cmd_vel` 最终进入 `CmdVelPoseControlSystem`。它仍只施加 world-XY 平面力和 yaw 力矩，
`z/roll/pitch` 由 DART 重力和接触求解；平面速度环由 P 扩展为可参数化 PI。插件默认
`linear_velocity_integral_gain=0`、`max_planar_integral_force=0`，所以其他未显式配置的模型保持旧 P
行为；当前哨兵 xacro 对洞口短坡启用 `Kp=80`、`Ki=45`、积分力矢量上限 `35 N`、总平面力上限
`120 N`。命令超时、非有限命令/状态或非法仿真步长都会清积分，总力饱和使用条件积分避免 windup。
有效零速命令持续输入时，积分项用于补偿坡面重力造成的稳态速度误差，使仿真车能在坡上保持，而
不是靠冻结 pose 或关闭重力。这只是 Gazebo 执行器模型，不属于实车 MPC 参数，也不能复制到实车
底盘。仿真另有最大 `45 N` 的停滞辅助：仅在非零目标速度下沿命令方向连续低速时延迟启用，实测
前向速度恢复到 `0.10 m/s` 立即清零，不保存脱困冲量。

四个可视轮仍是原圆柱，但 DART 碰撞体使用内收的全轮径球形接触，物理外廓为 `0.28 x 0.30 m`，
位于 `0.32 x 0.31 m` 安全 footprint 内。RMUC2026 保持 DART 动力学，并使用 Bullet collision
detector 解析球体与三角网格接触；这同时保留重力/坡面姿态，又避免固定圆柱轴向锁死全向横移。

## 10. 实车与仿真参数差异

下表是当前 profile 值，不是已经完成实车标定的测量结论。

| 参数 | 实车 | 仿真真值/Point-LIO 基础值 | 说明 |
| --- | ---: | ---: | --- |
| ROG frame | `camera_init` | `odom` / Point-LIO 覆盖为 `camera_init` | 输入合同不同 |
| cloud | `/cloud_registered_full` | `/registered_scan` / Point-LIO 覆盖为 full cloud | 输入合同不同 |
| organized no-return 重建 | 无 | 仅真值：360 x 96，any-inf，miss stride 1 x 1 | Gazebo 专用，不适用于 Point-LIO/实车 |
| ROG odom | `/aft_mapped_to_init` | `/lidar_odometry` / Point-LIO 覆盖为 aft mapped | 输入合同不同 |
| `robot_origin_to_ground` | 0.28 m | 0.20 m | 必须按安装高度复测 |
| `vehicle_height` | 0.42 m | 0.17 m | 仿真碰撞体顶面约 0.165 m；仿真值不是实车尺寸证明 |
| `body_bottom_clearance` | 0.04 m | 0.03 m | 低位 run 合并范围 |
| `require_ground_support` | `true` | `true` | active 的硬门 |
| 地图 `ground_elevation` | highbay 平地 `z=0` | RMUC2026 5 cm 栅格 `z=0.000..0.310 m`；洞口测试含 ramp/platform | 新增实车坡面仍须现场测绘 |
| `ground_support_tolerance` | 0.08 m | 0.08 m | 回波表面与测绘高程容差 |
| blind bridge | 关闭 | 关闭 | 不作为轮下支撑证明 |
| current-footprint / near-field fill | 严格 bootstrap / near-field 关闭 | `0.52 x 0.51 m` bootstrap / `1.40 x 1.00 m` 仿真短扫掠区 | 均须 prior-free、连续匹配高程、零 occupied |
| `min_headroom_known_ratio` | **0.50** | **0.25** | 仿真离散 ray 专用补偿 |
| `min_observed_overhead_headroom_known_ratio` | **0.0** | **0.0** | 仅适用于已测到顶板回波的列；仍要求可信高程支撑且保守顶板下边界高于车体，空列继续用上一项闭锁 |
| `headroom_margin` | **0.05 m** | **0.02 m** | 净空余量；仿真仍保留 2 cm，不得复制体素模型到实车 |
| `headroom_voxel_inset_fraction` | **0.5** | **0.0** | 实车按 occupied voxel 边界保守估计；规则仿真射线按 voxel center 估计 |
| `obstacle_hold_time` | 0.50 s | 0.0 s | 仿真靠 2 帧 hysteresis；实车按漏检上界保守保持 |
| prior `transform_timeout` | 0.50 s | 0.75 s | 仿真负载容差 |
| safety `map_timeout` | 0.50 s | 2.20 s | Gazebo 调度尖峰容差；仍小于占据证据 3 s 保留期 |
| 碰撞类缓存续发上限 | 0.40 s | 0.75 s | 超限立即 BLOCK，不继续执行长旧轨迹 |
| planner `max_velocity` | 0.7 m/s | 1.0 m/s | MINCO 连续峰值；实车与仿真分别放行 |
| planner `max_acceleration` | 0.8 m/s2 | 1.0 m/s2 | MINCO 连续峰值 |
| planner `max_yaw_dot` | 0.8 rad/s | 1.2 rad/s | yaw trajectory |
| planner `traj_goal_tolerance` | 0.15 m | 0.15 m | 必须严格小于 Nav2 `xy_goal_tolerance=0.20 m` |
| MPC `vx/vy` | +/-0.5 m/s | +/-1.5 m/s | QP box constraint；另有 0.5/1.0 m/s 径向硬上限 |
| MPC `omega` | +/-0.8 rad/s | +/-1.2 rad/s | QP box constraint |
| MPC `ax/ay` | +/-0.8 m/s2 | +/-1.0 m/s2 | 已启用 |
| MPC `alpha` | +/-2.0 rad/s2 | +/-3.0 rad/s2 | 已启用 |

两边共同使用：

~~~text
resolution                  0.05 m
max_ground_height_delta     0.35 m
max_ground_step             0.06 m
max_ground_slope_deg        28 deg
ground_support_tolerance    0.08 m
lookahead_dist              1.5 m
optimizer safe_dist         0.30 m
hard collision_dist         0.0 m
~~~

### 10.1 为什么仿真 known ratio 是 0.25

Gazebo MID360 当前是 10 Hz、水平 `360`、垂直 `96` 的规则 gpu_lidar，总计
`34560` 条 ray/frame，垂直 FOV 为 `-7.22 ... +55.22 deg`。在当前仿真 `0.20 m` 车高、
`0.05 m` 体素和规则射线模型下，车侧部分近地柱可能只有 4 个车体高度体素中的 1 个被标成
known，所以仿真门限设为 `0.25`。

96 条垂直 ray 在当前安装高度和俯视边界处的相邻地面回波间距约 `1.8 cm`；当前 miss stride 为
1，小于一个 ROG 体素。stride 只作用于 no-return，有限的洞顶、坡面和障碍命中不会被抽样。
该选择用于把 Gazebo、DDS 和 ROG 输入负载限制
在当前开发机能持续处理的范围内，不是实车 MID360 扫描模式的替代品。

这只是 **Gazebo 规则离散射线模型补偿**：

- 它不改变“所需车体净空带内的任意 occupied return 都硬否决”的逻辑；
- 它不放开 `INSUFFICIENT_OBSERVATION` 的普通全局 unknown；
- 它不单独证明轮下支撑；支撑资格来自已测绘的 `ground_elevation`；
- 它绝不能复制到实车；实车当前为 `0.50`，仍需用真实 bag 完成正负例验证。

最低俯视角在水平地面上的首个回波约为 `1.55 m`；在坡前遮挡叠加后，实测未持续获得地面回波的
区间约可达到 `2.35 m`。旧 `ground_seed_radius` 即使覆盖首圈回波也不能证明盲区内支撑，所以
active 已改用测绘高程。提高规则射线密度只改善离散采样，不会消除 `-7.22 deg` 以下的近场
盲区。该模型也不是物理 Livox 扫描时序、噪声和遮挡的等价实现，且 CPU 开销较高。

## 11. 当前实现为什么仍是 2.5D

### 11.1 数据在各层保留了什么

| 层级 | 表达 |
| --- | --- |
| ROG occupancy | 三维体素概率占据 |
| projection | 每个 XY 垂直柱的 ground/headroom/connectivity 分类 |
| fused field | 二维通行 mask 和二维 signed distance |
| MINCO position | 平面 X/Y 多项式，发布 Z 固定为 0 |
| yaw | 独立平面 yaw trajectory |
| MPC | `x, y, yaw, vx, vy, omega` 的 SE(2) 控制 |

“能识别悬空障碍物”来自三维体素和逐柱净空检查；“怎么绕开”仍发生在二维 XY 平面。

### 11.2 明确不具备的能力

当前系统不能：

- 在同一 XY 上规划“从障碍上方”或“从障碍下方”的不同 Z 路线；
- 输出随坡变化的轨迹 Z、roll 或 pitch；
- 做 SE(3) 搜索、三维 corridor 或三维 MPC；
- 建模四轮/多轮接地点、轮胎是否都有支撑；
- 检查轴距、前后悬在坡折点处的托底和机械干涉；
- 建模洞口到坡面过渡时的完整三维车体 swept volume；
- 推断摩擦、打滑、侧翻或纵向稳定性；
- 仅靠 MID360 可靠在线识别其下视盲区中的新坑、断崖和地面消失；
- 证明静态高程先验发布后现场从未发生变化。

处于车体所需净空高度内的浮空障碍会阻断该 XY；位于所需净空以上且下方观测充分的高位障碍可以
允许穿过。但 planner 不会通过调整车身高度解决冲突。

`ground_z_abs` 和 `ground_support_z_abs` 只用于通行分类，不成为 MINCO 的轨迹 Z。pitch-aware
逻辑只修正 yaw 尾端方向，不改变上述结论。

### 11.3 高程先验解决什么、不解决什么

`ground_elevation` 解决的是一个可观测性问题：MID360 看见车体上方净空，却因下视盲环暂时看不见
轮下表面时，使用事先测量的静态支撑高度完成判定。它比 observed-empty、footprint clear、二维
near-field fill 或 bridge 更严格，因为浮板回波必须与预期地面高度相符，纯空柱也必须位于已知
支撑区域。

但它仍是先验，不是在线负障碍传感器：

- `default_height` 会给 PGM 中全部 known-free 像素声明支撑；未测区和已知坑必须在 PGM 中标为
  unknown/occupied；
- patch 必须描述已经确认可行驶的坡面，陡坎、坡折机械干涉和无支撑边界不能仅靠一个高度平面表达；
- 发布先验后新出现的坑若落在下视盲区，系统可能仍相信旧支撑；
- 支撑高程不能替代轮地接触、悬挂、摩擦和车体姿态模型。

若验收要求在线发现新坑/地面坍塌，必须加入下视深度/LiDAR、负障碍边缘检测或轮地接触传感器，
并让其结果进入同一个 fail-closed mask。不能通过放大 bridge 或降低 known ratio 来替代。

### 11.4 洞口后紧接短坡的专门边界

对目标场景，当前链路按设计能够提供的条件性能力是：

- 对洞顶、横梁和浮空物体形成 3D occupied run；
- 按车型高度与 margin 判断逐柱净空；
- 用测绘高程约束地面回波，并为下视盲环中的空柱提供有界静态支撑；
- 以 1.5 m 滚动局部轨迹尝试逐步穿洞和上坡；
- 在 ROG 新鲜、footprint 全部通过时才授权 MPC。

引入高程先验前的最后一次 active 基线从起点向 `(3.5, 0)` 行驶后停在约
`map x=-0.185 m`；对应 ROG/odom 中，从最后一段已验证地面到坡面回波之间约有 `2.35 m` 空柱带，
planner 以 `COSTMAP_LETHAL` / `INSUFFICIENT_CLEARANCE` fail-closed。该结果证明 `0.45 m` bridge
不是目标问题的解法，也是本次改为可信支撑先验的直接原因。本轮最终真值正例见第 12.1 节；即使
单次通过，P2/P3 正反向和重复通过率仍需单独验收。

仍需额外实测/验算：

- 进入坡面时车体 pitch 后，车顶前后角与洞口/顶板的真实最小间隙；
- 轴距、前后悬和底盘离地间隙在坡折点是否托底；
- 坡面宽度、轮胎接触、摩擦与制动余量；
- ROG frame 的竖直柱与真实倾斜车体 swept volume 的差异；
- `transform_cloud` 使用的 cloud-stamp `base_link <- cloud_frame` TF 是否准确，以及旋转后的车体盒
  是否既清除车壳又保留洞沿/坡面回波；
- 下视盲区内是否存在负障碍。

在完成这些检查前，不能仅因 RViz 中 `/rog_map/layer_value` 连通或仿真一次通过，就宣称实车
可安全通过。若任务最终要求真正的空间绕行或严格坡面车体姿态约束，需要引入三维距离场/通行图、
带 Z/姿态的轨迹状态、轮地支撑与稳定性约束，以及相应控制器；当前集成没有提供这些能力。

## 12. 仿真场景与启动前提

### 12.1 RMUC2026 全图高程先验

RMUC2026 使用以下四个相互绑定的资源：

~~~text
src/pb2025_nav_bringup/map/simulation/RMUC2026.pgm
src/pb2025_nav_bringup/map/simulation/RMUC2026_elevation.pgm
src/rm_27_stimulation/meshes/RMUC2026_world/meshes/RMUC2026.stl
src/rm_27_stimulation/world/RMUC2026_world/RMUC2026_world.world
~~~

高程图为 `583 x 300`、5 cm 分辨率的 16 位 PGM，与二维占据图逐像素对齐。生成器从 world 读取
`ground_plane` 范围、高度以及 STL model pose/scale，只栅格化坡度不超过 28 度且高度不超过
0.60 m 的朝上表面；候选表面还必须能从 world 地面沿相邻栅格连续坡度到达。这样会保留真实坡道和
平台，并排除孤立物体顶面。对于同一 XY 同时命中洞底和顶板的单元，生成器只在最高表面不连续时，
保留该格内全部 STL 相交高度，并从相邻已验证地面选择坡度最连续的候选面。这样不会把桥底、坡面、
顶板三层压缩成只有最高/最低两层。当前正式地图最终有 `113792 / 113794` 个 known-free 单元具备
可信支撑，高程为 `0.000..0.310 m`；其余 2 个孤立单元使用 `no_data=65535`，继续 fail-closed。

这是单值 2.5D 地面模型：同一 XY 只能有一个支撑高度。它适用于当前场地的地面、坡道和平台；若
同一 XY 同时要导航桥上与桥下，必须拆地图/分层，或把规划状态扩展为真正的三维拓扑，不能靠一个
高程栅格同时表达两层。

生成和一致性检查命令：

```bash
cd /home/pnx/nav_ws/sentry-navigation-RM27
python3 src/pb2025_nav_bringup/tools/generate_rmuc2026_elevation.py
python3 src/pb2025_nav_bringup/tools/generate_rmuc2026_elevation.py --check
```

### 12.2 当前洞口资源

当前工作树包含：

~~~text
src/rm_27_stimulation/world/tunnel_ramp_test/tunnel_ramp_test.world
src/pb2025_nav_bringup/map/simulation/tunnel_ramp_test.yaml
src/pb2025_nav_bringup/map/simulation/tunnel_ramp_test.pgm
~~~

`src/rm_27_stimulation/config/worlds.yaml` 将 `tunnel_ramp_test` 映射到同名 nav map，
`spawn_poses.yaml` 提供该 world 的出生位姿。

同名 map YAML 还包含与 world 几何一致的 `ground_elevation`：水平地面为 `0.0 m`，短坡使用
平面 patch，平台为 `0.25 m`。坡道 patch 的 reference 是 `[0.442, 0.0, 0.0]`，对应倾斜 box 的
真实上表面入口；按 `slope_x=0.2085` 到 `x=1.641` 时为约 `0.25 m`，其末端目标高度与平台上表面
相同。平台 world pose x 为 `3.09147 m`，其 `2.9 m` 长度给出约
`[1.64147, 4.54147] m` 的几何范围；YAML 使用 bounds `[1.641, -0.675, 4.541, 0.675]` 和 reference
`[1.641, 0.0, 0.25]`，坡顶到平台几何与支撑先验均在 `1 mm` 内连续。world 中的 `z=0.0857` 是
带厚度倾斜 box 的**中心 pose**，不能直接写成地面高程。该场景用于验证软件合同；实车不得复制
这些坐标。

当前工作树**没有**：

~~~text
src/pb2025_nav_bringup/pcd/simulation/tunnel_ramp_test.pcd
~~~

因此洞口场景应先使用 `use_ground_truth_odom:=true`。当
`use_ground_truth_odom:=false` 且 `slam:=false` 时，simulation launch 会要求对应 PCD 存在，
否则在 launch 阶段明确失败。

顶层 `rm_27_stimulation/sim_with_nav.launch.py` 当前没有透传自定义 `prior_pcd_file`。后续测试
Point-LIO 版本时，需要先制作与 world/map 对齐的 PCD，并通过
`pb2025_nav_bringup/rm_navigation_simulation_launch.py` 的入口提供绝对路径，同时标定
`relocalization_manager.init_pose` 和必要的 `prior_pcd_transform`。“PCD 文件存在”不代表坐标
已经对齐。

本轮最终 `tunnel_ramp_test` 真值正例中，`NavigateToPose` action 返回 `SUCCEEDED`。配套 GT 记录中
`base_footprint` 的停稳位置为
`(x, y, z)=(3.408787, 0.012935, 0.309999) m`，按毫米记录即
`(3.409, 0.013, 0.310) m`。这是当轮使用 organized miss-ray stride 8、物理 yaw seed、
`free_goal=false` 显式终端 yaw 和坡面 PI 执行器的一次端到端正例；它证明这一组仿真合同可以到达
平台目标，不等于 Point-LIO 模式、反向通过、障碍负例、P6 重复性或实车 release 已完成。

同日对 ROG-map 热点复测发现，旧并行路径虽然只用约 `6-9 ms` 完成射线步进，却把数百万个 miss
`Vec3i` 合并后全局排序，`merge` 约为 `125 ms`。当前实现先按终点体素去掉重复射线，再用线程局部
hash 位图和 touched-ID 列表合并 miss，保持“每帧每体素一次 miss、hit 优先”的概率语义。相同
`59952` 输入点下，实测 `merge=9-12 ms`、`raycast=22-26 ms`、ROG 总更新约 `69-80 ms`；重新发送
`map(3.5,0)` 后约 `13.7 s` 返回 `SUCCEEDED`，车体高度相对起点抬升约 `0.25 m`。这仍是单次开发
正例，不替代 P95/P99、反向、障碍和重复性验收。

### 12.3 不应从当前工程状态推断什么

- 场景文件存在，不等于该场景已经完成重复通过率测试；
- 上述单次 `SUCCEEDED` 不等于 P2/P3/P6 或负例验收完成；
- 引入高程前约在 `map x=-0.185 m` 的 fail-closed 停车是对 blind bridge 方案的否定证据；
- 单元测试覆盖分类和安全合同，不等于 Gazebo 物理接触正确；
- 仿真真值模式可运行，不等于 Point-LIO 模式已经对齐；
- 参数写入 reality YAML，不等于实车安装高度、尺寸、时延和制动已经验收；
- 参考工程 `navi_minco_bit` 只提供设计参考，RM27 当前代码和本文件才是接口事实来源。

## 13. 相关目录与关键文件索引

### 13.1 Bringup、模式和 BT

| 文件 | 责任 |
| --- | --- |
| `src/pb2025_nav_bringup/launch/navigation_launch.py` | 三模式解析、参数 overlay、terrain 开关、shadow sidecar 与 lifecycle |
| `src/pb2025_nav_bringup/launch/bringup_launch.py` | localization/navigation 组合及 SLAM 模式约束 |
| `src/pb2025_nav_bringup/launch/rm_navigation_reality_launch.py` | 实车入口、map/PCD/模式参数 |
| `src/pb2025_nav_bringup/launch/rm_navigation_simulation_launch.py` | 仿真定位选择、PCD 前置检查、真值 localizer |
| `src/pb2025_nav_bringup/config/reality/nav2_params.yaml` | 实车 legacy 基线 |
| `src/pb2025_nav_bringup/config/simulation/nav2_params.yaml` | 仿真 legacy 基线 |
| `src/pb2025_nav_bringup/config/reality/minco_params.yaml` | 实车 active MINCO/ROG/MPC overlay |
| `src/pb2025_nav_bringup/config/simulation/minco_params.yaml` | 仿真真值 active overlay |
| `src/pb2025_nav_bringup/config/simulation/minco_pointlio_params.yaml` | 仿真 Point-LIO 输入 frame/topic 覆盖 |
| `src/pb2025_nav_bringup/config/*/minco_shadow_params.yaml` | shadow 主栈保持 legacy 的 BT overlay |
| `src/pb2025_nav_bringup/config/*/minco_shadow_sidecar_params.yaml` | 独立 sidecar 最终覆盖 |
| `src/pb2025_nav_bringup/behavior_trees/navigate_to_pose_w_minco_replanning.xml` | active 单目标 BT，固定 planner/controller ID |
| `src/pb2025_nav_bringup/behavior_trees/navigate_through_poses_w_minco_replanning.xml` | active 多目标防御性 fail-closed BT；当前 navigator 未加载 |
| `src/pb2025_nav_bringup/behavior_trees/navigate_to_pose_w_minco_shadow.xml` | shadow 单目标旁路后运行 legacy |
| `src/pb2025_nav_bringup/behavior_trees/navigate_through_poses_w_minco_shadow.xml` | shadow 多目标纯 legacy 执行，不派发 MINCO 对照 |
| `src/pb2025_nav_bringup/src/minco_shadow_goal_relay.cpp` | 非阻塞 action relay、accepted-cache、generation 与 lifecycle 重派 |
| `src/pb2025_nav_bringup/test/test_navigation_launch.py` | 模式/overlay launch 测试 |
| `src/pb2025_nav_bringup/test/test_minco_shadow_goal_relay.cpp` | shadow relay 行为测试 |

### 13.2 ROG-map

| 文件 | 责任 |
| --- | --- |
| `src/perception/rog_map/include/rog_map/rog_map_core/config.hpp` | ROG 参数模型与校验 |
| `src/perception/rog_map/include/rog_map_ros/rog_map_ros2.hpp` | ROS2 订阅、时间/frame 门、prior TF、可视化装配 |
| `src/perception/rog_map/include/rog_map_ros/latest_value_mailbox.hpp` | 多线程下原子交付最新点云、位姿和合并帧数 |
| `src/perception/rog_map/include/rog_map_ros/cloud_registered_crop_filter.hpp` | 自回波过滤接口 |
| `src/perception/rog_map/src/rog_map_ros/cloud_registered_crop_filter.cpp` | `transform_cloud` 点变换、Z 截断与旋转车体自滤实现 |
| `src/perception/rog_map/include/rog_map/projection_layer.hpp` | 柱统计、cell type/reason 和投影接口 |
| `src/perception/rog_map/src/rog_map/projection_layer.cpp` | 净空、支撑高度门、兼容 seed/BFS/多格 bounded bridge 与代码级 fail-closed |
| `src/perception/rog_map/include/rog_map/prior_map.hpp` | 二维 prior、高程 patch、frame transform、cache 与融合接口 |
| `src/perception/rog_map/src/rog_map/prior_map.cpp` | trinary YAML/PGM、高程解析、3 x 3 保守采样、支撑 cache 与三态融合 |
| `src/perception/rog_map/include/rog_map/field_layer.hpp` | 2D field 接口 |
| `src/perception/rog_map/src/rog_map/field_layer.cpp` | 正/负 EDT、距离和梯度 |
| `src/perception/rog_map/include/rog_map/map_query_interface.hpp` | 单点与 batch query、projected cost 和 snapshot 元数据合同 |
| `src/perception/rog_map/include/rog_map/query_adapter.hpp` | 不可变 snapshot query 接口 |
| `src/perception/rog_map/src/rog_map/query_adapter.cpp` | 单快照 batch、projected cost、cell-center 坐标和边界插值 |
| `src/perception/rog_map/src/rog_map/rog_map.cpp` | occupancy、投影、prior、field 的总装 |
| `src/perception/rog_map/src/rog_map/rog_map_visualizer.cpp` | `/rog_map/...` publisher 创建 |
| `src/perception/rog_map/test/test_latest_value_mailbox.cpp` | 最新点云/位姿/合并帧计数的原子交付与并发测试 |
| `src/perception/rog_map/test/test_projection_clearance.cpp` | 净空、支撑/浮板/坑/坡、legacy bridge、prior TF/cache/fusion 回归测试 |

### 13.3 MINCO planner

| 文件 | 责任 |
| --- | --- |
| `src/navigation/minco_planner/src/minco_core/minco_planner.cpp` | Nav2 plugin、ROG 内嵌、session、异步规划、物理 base pose 选择、yaw 与安全门 |
| `src/navigation/minco_planner/src/minco_core/minco_fsm.cpp` | 20 Hz 搜索/重规划状态机 |
| `src/navigation/minco_planner/include/minco_core/components/planning_request_contract.hpp` | 同目标判定、非零单调 planning token |
| `src/navigation/minco_planner/src/minco_core/components/planner_mode_context.cpp` | PRIORMAP/EXPLORATION query 角色 |
| `src/navigation/minco_planner/src/minco_core/components/map_query_adapters.cpp` | Nav2 costmap snapshot、单 TF 的 map/ROG batch query |
| `src/navigation/minco_planner/src/minco_core/components/global_path_searcher.cpp` | 全局 SMAC/A* |
| `src/navigation/minco_planner/src/minco_core/components/local_path_processor.cpp` | ROG 边界裁剪、lookahead 和稀疏化 |
| `src/navigation/minco_planner/src/traj_opt/minco_optimizer.cpp` | L-BFGS、连续动力学检查与时间重分配 |
| `src/navigation/minco_planner/src/utils/piece.cpp` | 分段多项式连续极值 |
| `src/navigation/minco_planner/src/traj_opt/yaw_traj_opt.cpp` | yaw waypoint、显式起终端约束与多项式 trajectory |
| `src/navigation/minco_planner/test/yaw_traj_opt_test.cpp` | 全向底盘显式 goal yaw 不被 position tangent 覆盖的测试 |
| `src/navigation/minco_planner/src/minco_core/components/trajectory_safety_checker.cpp` | coherent footprint batch、freshness、空间 pickup、swept join 与旋转矩形检查 |
| `src/navigation/minco_planner/test/map_query_adapters_test.cpp` | query/frame/snapshot 合同 |
| `src/navigation/minco_planner/test/minco_optimizer_dynamics_test.cpp` | 连续峰值和重定时 |
| `src/navigation/minco_planner/test/planning_session_state_test.cpp` | generation/lifecycle |
| `src/navigation/minco_planner/test/planning_request_contract_test.cpp` | 同目标复用和 token 单调性测试 |
| `src/navigation/minco_planner/test/planning_request_lease_test.cpp` | active 请求心跳租约和过期 fail-closed 测试 |
| `src/navigation/minco_planner/test/smac_global_search_policy_test.cpp` | 全局搜索策略及失败边界测试 |
| `src/navigation/minco_planner/test/trajectory_safety_checker_test.cpp` | footprint/freshness/fail-closed |
| `src/navigation/minco_planner/src/minco_core/minco_fsm.cpp` | 20 Hz 状态检查、当前 profile 约 2 Hz 成功滚动重规划，以及最多 4 Hz 的失败生成限频，避免阻塞时重复搜索挤占 ROG 回调 |

### 13.4 MPC、消息和速度 bridge

| 文件 | 责任 |
| --- | --- |
| `src/navigation/minco_controller/src/minco_mpc_controller.cpp` | Nav2 controller plugin、frame 统一、输入门和零速策略 |
| `src/navigation/minco_controller/include/minco_controller/reference_progress.hpp` | 新轨迹空间 pickup、同轨迹单调且受空间领先上限约束的时间推进合同 |
| `src/navigation/minco_controller/src/mpc_solver.cpp` | QP 模型与求解 |
| `src/navigation/minco_controller/src/input_validation.cpp` | trajectory/odom/stamp 数值校验 |
| `src/navigation/minco_controller/include/minco_controller/trajectory_session_gate.hpp` | Path/trajectory token 授权、pending 与旧会话拒绝 |
| `src/navigation/minco_controller/test/test_mpc_solver.cpp` | MPC 约束与求解测试 |
| `src/navigation/minco_controller/test/test_input_validation.cpp` | 非有限值和时间戳测试 |
| `src/navigation/minco_controller/test/test_reference_progress.cpp` | 空间/时间 reference 进度与非法输入测试 |
| `src/navigation/minco_controller/test/test_trajectory_session_gate.cpp` | 旧 trajectory 隔离测试 |
| `src/ros_interfaces/msg/MpcPositionCommand.msg` | `planning_stamp`、NORMAL/BLOCK 和离散轨迹命令 |
| `src/ros_interfaces/msg/PositionCommand.msg` | position/velocity/acceleration/jerk/yaw 样本 |
| `src/fake_vel_transform/src/fake_vel_transform.cpp` | world-aligned 命令到底盘系、odom watchdog |
| `src/fake_vel_transform/include/fake_vel_transform/odom_validation.hpp` | odometry/stamp 校验 |
| `src/fake_vel_transform/test/test_fake_vel_transform.cpp` | latest-odom 模式和 fail-closed 测试 |

### 13.5 定位、旧感知与仿真

| 文件/目录 | 责任 |
| --- | --- |
| `src/point_lio/src/laserMapping.cpp` | `/cloud_registered_full`、`/aft_mapped_to_init` 和 twist 合同 |
| `src/loam_interface/src/loam_interface.cpp` | Point-LIO 输出整理为 registered scan/lidar odom |
| `src/sensor_scan_generation/src/sensor_scan_generation.cpp` | 底盘 `/odometry`、`/sensor_scan` 和 TF |
| `src/sensor_scan_generation/include/sensor_scan_generation/twist_estimator.hpp` | child-frame twist 估计 |
| `src/sensor_scan_generation/include/sensor_scan_generation/transform_sample_gate.hpp` | 必需 TF 任一失败时整组样本 fail-closed，恢复后 twist 冷启动 |
| `src/sensor_scan_generation/test/test_twist_estimator.cpp` | child-frame 有限差分 twist 测试 |
| `src/sensor_scan_generation/test/test_transform_sample_gate.cpp` | 双 TF 原子发布门、单边失败和恢复冷启动测试 |
| `src/relocalization_manager/src/relocalization_manager.cpp` | prior PCD 重定位和 `map -> odom` |
| `src/terrain_analysis/src/terrainAnalysis.cpp` | legacy local terrain |
| `src/terrain_analysis_ext/src/terrainAnalysisExt.cpp` | legacy global/extended terrain |
| `src/pb_nav2_plugins/src/layers/intensity_voxel_layer.cpp` | legacy terrain 到 costmap |
| `src/rm_27_stimulation/src/ground_truth_localizer.cpp` | 真值状态缓存、organized no-return 重建；按 scan stamp 插值并成对发布 lidar odom/registered scan |
| `src/rm_27_stimulation/include/rm_27_stimulation/ground_truth_state_buffer.hpp` | 有序 GT 缓存、严格时间包围、插值间隔和 steady timeout 合同 |
| `src/rm_27_stimulation/include/rm_27_stimulation/twist_transform.hpp` | `R(v + omega x p)` 与六维 covariance Jacobian |
| `src/rm_27_stimulation/test/test_ground_truth_twist_transform.cpp` | 杆臂、安装旋转、covariance、GT 有序缓存/插值与 steady timeout 单测（可执行名同文件主名） |
| `src/rm_27_stimulation/include/rm_27_stimulation/organized_lidar_miss_rays.hpp` | Gazebo organized any-inf miss-ray 的强校验与有限端点重建 |
| `src/rm_27_stimulation/test/test_organized_lidar_miss_rays.cpp` | layout/ring/Inf/NaN、range 和仅 miss stride 合同测试 |
| `src/rm_27_stimulation/src/cmd_vel_pose_control_system.cpp` | Gazebo 平面 PI 速度力伺服、yaw 力矩、命令 timeout 与非有限输入保护 |
| `src/rm_27_stimulation/include/rm_27_stimulation/planar_velocity_pi.hpp` | 矢量积分限幅和总力饱和 anti-windup |
| `src/rm_27_stimulation/test/test_planar_velocity_pi.cpp` | P 兼容、积分限幅、anti-windup、复位和非法输入测试 |
| `src/rm_27_stimulation/include/rm_27_stimulation/stiction_assist.hpp` | 非零命令下的延时、渐增、速度释放和换向复位停滞辅助 |
| `src/rm_27_stimulation/test/test_stiction_assist.cpp` | 启用延时、斜坡、上限、释放、换向和非法输入测试 |
| `src/rm_27_stimulation/urdf/mid360.xacro` | 360 x 96 gpu_lidar 模型 |
| `src/rm_27_stimulation/urdf/simulation_waking_robot.xacro` | 仿真车体、全向球形接触、驱动插件、短坡 PI 与停滞辅助参数 |
| `src/rm_27_stimulation/launch/sim_with_nav.launch.py` | Gazebo 与导航总入口 |
| `src/rm_27_stimulation/config/worlds.yaml` | world 到 nav map 映射 |
| `src/rm_27_stimulation/config/spawn_poses.yaml` | 仿真出生位姿 |
| `src/rm_27_stimulation/world/RMUC2026_world/RMUC2026_world.world` | RMUC2026 DART 动力学与 Bullet collision detector 配置 |
| `src/rm_27_stimulation/world/tunnel_ramp_test/` | 洞口短坡 world |
| `src/pb2025_nav_bringup/map/` | Nav2/ROG 共用二维 YAML/PGM |
| `src/pb2025_nav_bringup/pcd/` | Point-LIO 重定位先验；与二维 map 不是同一资源 |

## 14. 阅读与排障顺序

理解一次 active 运行时，建议按以下因果顺序定位，不要一上来只调 MPC：

~~~text
模式/overlay
  -> cloud + odom + TF
  -> raw occupancy
  -> 2D prior + ground_elevation + prior TF/cache
  -> 动态柱分类 + support/headroom/邻接连续性门
  -> static occupied 最终融合
  -> fused field
  -> global/local seed
  -> MINCO 动力学与 footprint
  -> Path token + MpcPositionCommand flag/planning_stamp/header freshness
  -> MPC odom/reference/QP
  -> smoother 与 fake_vel_transform
  -> /cmd_vel
~~~

具体命令、RViz 显示组合、参数优先级、仿真到实车的分阶段验收和故障树，统一放在配套
[调试与调参指南](./rog_minco_tuning_debug_guide.md) 中维护，避免架构事实与操作手册互相漂移。

## 15. 洞口净空稳定层

ROG 的二维投影仍以三维占据为硬否决：只要柱内存在占据体素，低梁和浮空障碍会在当前更新
立即进入闭锁。为了避免规则化 LiDAR 射线在洞口边缘形成一帧宽的假障碍，投影提交阶段增加了
一个有限状态：

~~~text
明确净空证明
  -> 本帧仍有净空证明：直接提交 FREE/PASSABLE
  -> 本帧零命中 + HEADROOM_UNVERIFIED + 高程连续：
       在 clearance_dropout_hold_time 内提交 CLEARANCE_DROPOUT_HOLD
  -> 出现任意占据命中 / 高程断裂 / 超时：立即提交 OCCUPIED

从未单独验证的窄带 HEADROOM_UNVERIFIED
  -> known-free + 零占据 + 连续可信支撑 + 两端已验证净空 + 宽度不超上限：
       提交 CLEARANCE_BOUNDED_HOLE_FILL
  -> 超过宽度 / 没有两端证明 / 任意占据命中：保持 OCCUPIED
~~~

这些规则只在 `prior_map.require_ground_support=true` 时生效。窄带补洞先收集全部候选再统一提交，
因此推断结果不能递归传播，也不能打开无端点的观测前缘；仿真默认开启并限定两个 5 cm 栅格，
实车默认关闭且初次现场验证最多只允许一个栅格。`obstacle_hold_time` 控制“障碍消失后多久清除”，
`clearance_dropout_hold_time` 控制“已证明净空后允许一次零命中观测抖动多久”，两者方向相反且
前者优先。具体初值、RViz 图层和正负例调参流程见调试指南第 17 节。

没有两端净空证明的 `HEADROOM_UNVERIFIED` 仍保持 occupied；它只会让局部轨迹裁剪、停车并等待
视场扩展，不再通过全局 SMAC 的 ROG ESDF 软代价诱导绕路。这个职责分离是洞口稳定性的另一半。

## 16. 坡边全局引导与局部足迹一致性

`ground_elevation` 不只是给局部 ROG 柱分类提供地面支撑。MINCO 启动时还会从完整高程先验中
提取相邻栅格的高度突变和超限坡度，形成只作用于全局路径查询的静态坡边覆盖层：

在坡边覆盖之前，全局查询现在还会先生成静态障碍硬净空层。Nav2 inflation 的 `1..252` 仍是
SMAC 的软偏好，`253` 保留 Nav2 原有的不可通行语义；只有原始 `LETHAL_OBSTACLE=254` 才作为新
硬层的膨胀源。硬半径不再另设 YAML 参数，而是由最终安全足迹自动计算：

~~~text
static_hard_radius = hypot(length / 2 + margin,
                           width  / 2 + margin)
                   + max(global_costmap_resolution, ROG_resolution)
~~~

因此修改 `safety.footprint_length/width/margin` 会同时改变局部三维安全检查和全局静态硬净空，二者
不会再因手工维护两套半径而漂移。未知格仍保持 unknown，不会被这层伪装成已知障碍。覆盖层在插件
配置时建立，并在 lifecycle activate 时从已经激活的静态 costmap 再建立一次，避免 transient-local
地图晚于插件 configure 到达。

~~~text
Nav2 static lethal
  -> 按安全足迹角点半径 + 一格离散误差膨胀
  -> static hard clearance

ground_elevation
  -> 相邻高度差/坡度检查
  -> 二维地图可通行但高程无有效支撑：lethal
  -> 坡边栅格：lethal
  -> lethal_clearance_radius 内：lethal（完整足迹不得跨越）
  -> clearance_radius 的剩余外圈：clearance_cost（高代价但可通行）
  -> 与 Nav2 静态代价查询取最大值
  -> SMAC 全局种子远离坡边

rolling ROG 3D field
  -> 当前传感器证据、低梁、浮空障碍、未知空间
  -> 局部种子和完整安全足迹逐点检查
  -> 最终放行/裁剪/停车
~~~

静态覆盖层不替代 ROG，也不把高程先验当成动态障碍物真值。坡边及 footprint 几何所需的内圈
是致命代价，内圈之外到 `clearance_radius` 的窄外圈保持 `clearance_cost=240`。这样不会让 SMAC
生成“中心可过、车角越出坡面”的路径，同时给已经贴近边缘的车辆保留高代价退出区。

全局路径以中心点查询，而局部安全门检查矩形足迹，因此缓冲半径不能只按半车宽设置。静态障碍
硬半径由代码按下式自动得到；坡边的手工硬内圈也不得小于同一几何下限：

~~~text
lethal_clearance_radius >= hypot(length / 2 + margin,
                                 width  / 2 + margin) + ground_elevation_resolution
clearance_radius >= lethal_clearance_radius
~~~

当前仿真安全包络为 `0.32 x 0.31 m`，实车为 `0.40 x 0.30 m`，高程分辨率均为 `0.05 m`，
对应自动静态硬半径约为 `0.273 m` 和 `0.300 m`。仿真的坡边
`lethal_clearance_radius` 为 `0.28 m`，实车为 `0.30 m`，两套配置的外圈
`clearance_radius` 均为 `0.30 m`。这避免了中心线看似可行、但足迹角点恰好落入坡边扩展格而被
持续裁剪的全局/局部矛盾。

RMUC2026 高程中存在约 `3 cm` 的局部坡面突起。仿真全局覆盖层因此使用
`max_step=0.02 m`、`max_slope_deg=20 deg`，使 SMAC 提前绕开该已测粗糙区；最终局部 ROG 分类仍为
`max_ground_step=0.06 m`、`max_ground_slope_deg=28 deg`。全局种子可以比局部安全门更保守，不能
反过来更宽松，否则会持续生成局部必拒的路径。实车两组阈值保持 `0.06 m / 28 deg`，除非现场完整
高程和可绕拓扑已经测量证明，不得直接复制仿真分层值。

日志中的两种停止需要分开判断：`GROUND_UNVERIFIED` 且查询点紧邻高程突变，通常是坡边引导
不足、高程错误或全局/局部坡度阈值不一致；`HEADROOM_UNVERIFIED` 且 `occupied_z` 为空，通常是
首次经过时车体高度带的 free-ray 证据尚未覆盖。前者可通过高程和几何缓冲修复，后者应等待视场
扩展或改善传感器布置，不能通过关闭 unknown 闭锁来消除。

## 17. 冷启动近场与坡面自回波闭环

坡面停车不能只根据 `COSTMAP_LETHAL` 判断成地图障碍。本轮另一处洞口的持续拒绝同时具有以下
特征：快照和 TF 时间正常，二维 prior 为 known-free，高程连续，但固定安全脚印点反复报告
`raw_reason=GROUND_UNVERIFIED`、`occupied_z=[0.325,0.375]` 和 `support_z=0.138`。把 ROG 的 odom
高度换回 map 后，该格先验坡面约为 `0.200 m`；有限回波相对 `base_link` 约位于
`z=0.145..0.195 m`，查询点又固定在车体局部 `y=0.15 m`。这与地图 STL 坡面不符，却与仿真
LiDAR/IMU 组件的包络重合。

仿真 URDF 中底盘/车轮和传感器组件是两个不同包络：

| 部件 | `base_link` 中的主要范围 | 旧自滤结果 |
| --- | --- | --- |
| 底盘与车轮 | `x=+/-0.160 m`、`y=+/-0.155 m`、`z=-0.060..0.140 m` | 已覆盖 |
| Mid360/IMU | `y=0.124..0.205 m`、`z=0.085..0.165 m` | 单个居中盒在 `y=0.160 m` 截止，部分漏出 |

`cloud_filter` 现在在仿真 overlay 中配置两个独立盒。共享 `0.02 m` padding 后，底盘盒实际范围为
`x=+/-0.19 m`、`y=+/-0.16 m`、`z=-0.04..0.18 m`；传感器盒实际范围为
`x=+/-0.07 m`、`y=0.12..0.24 m`、`z=0.08..0.20 m`。第二个盒只覆盖车上物理组件，不把整车
周围都扩成不可观测区。处理顺序仍是：

```text
registered cloud in odom
  -> 使用同帧 paired lidar odometry 得到 odom <- lidar
  -> 组合静态 base_link <- lidar 外参
  -> 把点变换到 base_link
  -> 分别剔除 chassis/wheels 与 lidar/imu 两个盒内的点
  -> 把保留点以原 odom 坐标交给 ROG raycasting
```

这项修复只进入 `config/simulation/minco_params.yaml`。实车外形、雷达支架和 Point-LIO 点云合同不同，
实车 `cloud_filter` 不能复制仿真盒，必须用实测 CAD/URDF 和静态 bag 标定。

冷启动时还有另一类没有有限占据回波的近场零命中。`SURVEYED_NEAR_FIELD_CLEAR` 只允许同时满足
以下条件的格成为通行：二维 prior 明确 free、高程支撑有效且与车体所在连续坡面一致、格内零 occupied
voxel、查询点位于配置的随车近场矩形内。它不覆盖 prior unknown、坡边、高程断层或任何真实 hit；
实车当前也启用这一有界补偿，但使用独立尺寸，并保留真实 occupied return 的一票否决。

MINCO 动力学提交也保持两级边界：每次重定时都以 `0.1%` 严格误差为目标；只有所有重定时次数耗尽
后，才允许连续极值求根在重复根附近留下最多 `2%` 的数值残差。非有限值、边界状态超限和更大的
速度/加速度违规仍失败关闭。失败日志会同时给出 L-BFGS 返回值、冷热启动、起点速度、首段方向、
连续峰值和重定时次数，用来区分数值残差与真正不可行轨迹。
