# MINCO 实车调参教程

适用范围：ROS 2 Jazzy，`navigation_mode:=minco`，以
`src/pb2025_nav_bringup/config/reality/minco_params.yaml` 为唯一实车 MINCO 参数来源。
仿真使用 `config/simulation/minco_params.yaml`，`legacy` 使用实车的
`nav2_params.yaml`；不要把仿真值或 legacy 插件参数直接复制到实车 MINCO。
本文的数值是当前实车配置快照，不是所有场地的通用建议值。先完成传感器、TF、车体尺寸及
`map/ground_elevation` 标定，再修改安全阈值。

## 1. 调参前：确认哪一层出了问题

实车数据链：Point-LIO `/aft_mapped_to_init` + `/cloud_registered_full` ->
`MincoPlanner` 内的 ROG-map -> 全局搜索种子 -> 局部 MINCO 轨迹 -> `/minco/opt_path`
-> `MincoMpc` (`/lidar_odometry`) -> velocity smoother -> `fake_vel_transform`
-> `/cmd_vel`。ROG-map 的三维占据/地面/净空判断，与 Nav2 的二维 local/global costmap
是不同的安全关卡；看见二维地图有空隙不代表头顶或地面已经验证通过。

在仓库根目录、确认底盘安全和急停可用后启动：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py \
  world:=highbay slam:=false navigation_mode:=minco use_rviz:=true
```

实车和仿真启动 RViz 时默认启用 `rviz_hardware_acceleration:=true`。在当前
NoMachine 远程桌面中，启动文件通过 VirtualGL / EGL 使用 Intel 核显，且只作用于
RViz 进程；Point-LIO、ROG、MINCO 和 MPC 仍在 CPU 上计算。需要对比原渲染方式时
加 `rviz_hardware_acceleration:=false`。没有 VirtualGL 的机器使用桌面 OpenGL，
实际是否使用 GPU 仍取决于驱动和桌面环境。

`world` 要换成现场**已有且与实际环境匹配**的地图；坡道须有正确的地面高程，不能沿用平地
`z=0` 假设。首次/每次改动后先确认参数真的进入对应节点（以下均为读取，不是在线修改）：

```bash
ros2 lifecycle get /planner_server
ros2 lifecycle get /controller_server
ros2 param get /planner_server planner_plugins
ros2 param get /controller_server controller_plugins
ros2 param get /controller_server MincoMpc.max_planar_speed
ros2 param get /local_costmap/local_costmap plugins
ros2 param get /global_costmap/global_costmap plugins
ros2 topic hz /aft_mapped_to_init
ros2 topic hz /lidar_odometry
ros2 topic hz /cloud_registered_full
ros2 topic hz /rog_map/dynamic_obstacles
```

预期插件只有 `MincoPlanner` / `MincoMpc`；两张 costmap 都按
`static_layer -> rog_dynamic_obstacle_layer -> inflation_layer` 合成。规划器使用
`/aft_mapped_to_init`（`camera_init`），控制器使用补偿后处在 `odom` 的
`/lidar_odometry`；不要为了消除报错把两者改成相同 topic。若节点名因实际启动方式不同，
先 `ros2 node list` 核对。每轮只修改一组参数、备份原值、记录同一起点/目标和日志，
重启对应 bringup 后再比较；不要把 `ros2 param set` 当成这份 YAML 的持久化修改。

RViz 中建议同时显示 `/astar_path_vis`（真正的全局搜索路径）、`/opt_path_vis`
（优化后的局部路径）、`/rog_map/dynamic_obstacles`、`/rog_map/clearance_status`、
`/minco/dynamic_costmap_inflation` 和两张 `*/costmap`。Nav2 的 `/plan` 可能只是
MINCO 异步规划的两点 token 占位，**不能据此判断全局规划只有一条直线**。
动态膨胀的点云仅用于诊断，最终决策以 costmap、轨迹和 ROG 安全检查为准。

## 2. `local_costmap`：近场障碍与避障缓冲

修改位置：`local_costmap.local_costmap.ros__parameters`。本层在 `odom` 中使用
5 x 5 m 滚动窗口；动态层只把 `/rog_map/dynamic_obstacles` 中超过阈值的
**已观测障碍**写成 lethal cell，随后 inflation 层扩展二维代价。
当前 `MincoMpc` 主要从 costmap 读取坐标系，不拿 local costmap 的软代价
直接计算局部轨迹；局部 MINCO 轨迹仍由 planner 的 ROG/安全检查决定。
所以调 local 膨胀不能替代调 planner，也不能把它当作单独的避障兜底。

| 参数（省略上述前缀） | 当前值 | 作用与调整方向 |
|---|---:|---|
| `plugins` | static, ROG dynamic, inflation | 顺序很重要；若动态障碍无膨胀，先核对三个插件均已加载，不要只调半径。 |
| `global_frame` / `robot_base_frame` | `odom` / `gimbal_yaw_fake` | 必须能 TF 连通；不得用云台的真实旋转角充当车体真实朝向。 |
| `rolling_window`, `width`, `height`, `resolution` | `true`, 5 m, 5 m, 0.05 m | 决定近场覆盖和栅格精度；窗口变大、格子变细都会增加计算和更新负担。 |
| `update_frequency` / `publish_frequency` | 10 / 5 Hz | 前者是地图更新频率，后者是外发给 RViz 的频率；提高发布频率不能让传感器变快。 |
| `robot_radius` | 0.20 m | Nav2 圆形近似；不是 MINCO 的长方形安全足迹，也不是实测车宽。 |
| `rog_dynamic_obstacle_layer.topic`, `obstacle_threshold` | `/rog_map/dynamic_obstacles`, 65 | 消息值达到阈值才标 lethal；无消息先查 ROG 和 TF，不能仅降低阈值制造障碍。 |
| `rog_dynamic_obstacle_layer.stale_timeout`, `tf_timeout` | 0.75 / 0.10 s | 动态栅格接收超时会清除旧快照并标为非 current；TF 查询失败也无法投影。优先排查延迟/丢帧，不能靠无限增大超时。 |
| `rog_dynamic_obstacle_layer.footprint_clearing_enabled` | `true` | 仅在动态二维层清理车体足迹内的自障碍；不会跳过 ROG 三维安全判断。 |
| `inflation_layer.inflation_radius`, `cost_scaling_factor` | 0.40 m, 4.0 | 半径控制膨胀影响范围；缩放系数越大，范围内的代价衰减越快。先在 RViz 观察再小幅调；不要把膨胀半径当成车体硬碰撞边界。 |

**先诊断，再改值**：动态障碍不出现时，检查 `/rog_map/dynamic_obstacles`、
costmap 中动态层投影日志的 `source/marked/outside/transform_failed`；有障碍点但无
膨胀，核对插件顺序、`inflation_layer.enabled` 和最终 costmap。
消息超过 0.75 s 未更新时，旧动态层会清空；这不表示现场已经安全。
不应关闭 ROG/规划器的独立新鲜度检查去掩盖这个问题。

## 3. `global_costmap`：搜索拓扑与选路

修改位置：`global_costmap.global_costmap.ros__parameters`。该图在 `map` 下组合
静态先验、ROG 动态障碍和 0.40 m 膨胀。与 local costmap 的关键区别是
`track_unknown_space: true`、不滚动以及 5 Hz 更新/2 Hz 发布。

| 参数 | 当前值 | 调整时要看什么 |
|---|---:|---|
| `plugins` | static, ROG dynamic, inflation | 只在 local 加动态层，global 搜索可能仍穿过动态障碍；两个表都要有。 |
| `global_frame`, `robot_base_frame` | `map`, `gimbal_yaw_fake` | 定位 TF 抖动时全局障碍投影和路线会漂移；先核对 `map -> odom`。 |
| `resolution`, `update_frequency`, `publish_frequency` | 0.05 m, 5 / 2 Hz | 门洞/细障碍分辨率和更新负载的折中；不要以 RViz 的 2 Hz 判断规划频率。 |
| `track_unknown_space` | `true` | 地图未知空间保留 unknown；与下文 `MincoPlanner.allow_unknown: false` 配合，不能为了通路直接改为 false。 |
| `robot_radius` | 0.20 m | Nav2 近似；MINCO 另按矩形足迹 + 边距做硬校验。 |
| `rog_dynamic_obstacle_layer.*` | 同 local costmap | 全局坐标系是 `map`；若 local 有障碍而 global 没有，重点查动态层是否加载、TF 和 `outside` 数。 |
| `inflation_layer.inflation_radius`, `cost_scaling_factor` | 0.40 m, 4.0 | 改变 SMAC 的绕行偏好，需同时看 `MincoPlanner.smac_2d.cost_penalty`，勿一次改两者。 |

静态地图的膨胀、动态 ROG 的膨胀都应该出现在最终 costmap。RViz 可叠加
`/minco/global_costmap_soft_costs` 和 `/minco/dynamic_costmap_inflation` 区分
软代价和动态障碍周围代价。某个洞不被选择时先分别检查洞口的静态占据、动态
lethal、膨胀软代价和 `astar_path_vis`，确认它到底是硬阻塞还是被路线代价避开；
无高程模式下 `ground_edge_avoidance` 固定关闭，坡面安全由 ROG 连通性判断；不要通过
缩小车体尺寸或放行 unknown 来解决卡住。

## 4. `planner_server`：全局种子、ROG 校验和局部轨迹

修改位置：`planner_server.ros__parameters.MincoPlanner`。`planner_plugins` 必须是
`["MincoPlanner"]`。`planner_mode: PRIORMAP` 从先验地图搜索全局路线，
再截取已观测的局部段供 MINCO 优化；导航安全还取决于 ROG 的地面、净空和动态障碍。

### 4.1 全局路径和重规划

| 参数（`MincoPlanner.` 后缀） | 当前值 | 含义与调整方向 |
|---|---:|---|
| `priormap.use_nav2_global_search`, `use_smac`, `allow_unknown` | true, true, false | 在已有地图上用 Nav2/SMAC 种子搜索，不走 unknown；别用放行未知空间解决卡住。 |
| `smac_2d.cost_penalty` | 8.0 | 放大膨胀软代价的绕行倾向；过大可能宁可走远洞，过小可能贴障碍/贴洞边。对照 `/astar_path_vis` 调整，必须保持硬碰撞检查。 |
| `smac_2d.use_esdf_cost` | false | 当前不把 ROG 未观测边界当作 SMAC 全局软代价；下方 `esdf_weight/decay/max_cost` 在 false 时不应视为当前选路旋钮。 |
| `priormap.dynamic_global_obstacle.enable`, `.collision_distance` | true, 0.34 m | 让已测 ROG 占据参与全局搜索硬遮罩；距离是障碍周围搜索中心线的缓冲，不等同 Nav2 0.40 m 软膨胀。距离过大会封窄洞，过小有扫碰风险。 |
| `priormap.ground_edge_avoidance.enable` | false | 当前实车不使用先验高程，因此全局搜索不生成测绘坡边层；二维静态墙体仍正常参与搜索。 |
| `rog_map.projection.max_ground_step`, `.max_ground_slope_deg` | 0.06 m, 28 deg | 实时点云地面的相邻台阶与坡度上限；这是无高程模式的局部硬门，不要为消除停顿直接放大。 |
| `priormap.clip_seed_by_rog_boundary`, `.rog_boundary_margin` | true, 0.30 m | 全局路径进入 ROG 未观察区域时截短当前局部种子；不应为消除停顿直接关掉。 |
| `local_path.observed_prefix_max_velocity` | 0.20 m/s | 已观测安全前缀到观察边界时的慢速推进上限；适合行进中缓慢摆动云台继续获取点云，不保证盲区可通行。 |
| `local_path.shortcut_peak_cost_slack`, `.shortcut_mean_cost_slack` | 10, 5 | 局部路径抄近路可接受的代价峰值/均值增量；奇怪的贴障碍捷径先尝试减小容忍度并对照全局种子。 |
| `minco_optimizer.lookahead_dist` | 1.5 m | 一次优化/校验的局部前视距离，不是全局目标距离；过长会要求更多已观测安全区域，过短易频繁滚动重规划。 |
| `minco_optimizer.successful_replan_period`, `.failed_replan_retry_period` | 0.50 / 0.25 s | 已成功/失败后的内部重规划节流时间；减小会增加 CPU 占用和路径抖动，不等同 BT 请求频率。 |
| `request_lease_timeout` | 0.75 s | 上层持续规划请求的会话租期；请求中断会撤销旧轨迹。先测 BT 和规划器实际更新间隔，不应单纯加大到数秒。 |

`planner_server.ros__parameters.expected_planner_frequency: 20.0` 是 Nav2 的频率
期望/告警值；`MincoPlanner.minco_optimizer.opt_freq: 20.0` 也不能直接解释为独立
`createPlan()` 调度周期。实际规划请求和轨迹发布速率应查日志及 topic。

### 4.2 局部 MINCO、车体安全和头顶高度

| 参数（`MincoPlanner.` 后缀） | 当前值 | 含义与调整方向 |
|---|---:|---|
| `minco_optimizer.max_velocity`, `.max_acceleration`, `.max_yaw_dot` | 0.8 m/s, 0.8 m/s², 0.8 rad/s | 规划轨迹的速度/加速度/偏航上限；不要只提高控制器速度上限，二者须按实测能力配套。 |
| `minco_optimizer.terminal_velocity_ratio`, `.max_trajectory_duration` | 0.80, 20 s | 局部终点速度比例与轨迹最大时长；20 s 接纳经动力学重定时的坡面轨迹，非有限值仍会拒绝。若车长期低速，检查优化器日志而不是继续增大。 |
| `minco_optimizer.safe_dist` | 0.30 m | 优化器避障偏好/代价距离；不是硬碰撞判定，也不能代替矩形车体检查。 |
| `minco_optimizer.collision_dist` | 0.0 m | 安全检查在矩形足迹之外的额外距离；本配置避免重复径向膨胀，不能理解为“关闭碰撞检查”。 |
| `safety.footprint_length`, `.footprint_width`, `.footprint_margin` | 0.30, 0.20, 0.05 m | 矩形轨迹硬校验的本体尺寸及额外边距；与车体外形、凸出物核对，不要为穿洞擅自减小。 |
| `safety.sample_dt`, `.map_timeout`, `.check_horizon` | 0.05 s, 0.50 s, 2.0 s | 轨迹采样、地图新鲜度与滚动安全校验时域。 |
| `safety.collision_cache_reuse_max_duration`, `.optimizer_failure_cache_reuse_max_duration` | 0.40 s, 2.0 s | 碰撞拒绝只允许极短尾段；数值优化失败可在每次通过最新 ROG 足迹校验后续行，且不得超过 `check_horizon`。 |
| `rog_map.projection.vehicle_height`, `.headroom_margin` | 0.42, 0.03 m | 高度是**距局部地面的车体高度**与头顶余量：净空判断约需 0.45 m；不是固定 map-frame Z，也不是 LiDAR 高度。 |
| `rog_map.projection.robot_origin_to_ground` | 0.28 m | 传感/车体原点到地面参考高度；与车体安装标定区分，不能拿它替代 `vehicle_height`。 |
| `rog_map.projection.min_headroom_known_ratio` | 0.15 | 车身体积内需要的已观测净空比例。降低可减少 MID360 启动时的 `HEADROOM_UNVERIFIED`，但会接受更多未观测体素；实际占用回波和低净空仍然会拒绝。 |
| `rog_map.projection.prior_map.require_ground_support` | false | 不使用先验高程；地面必须由车体接地点高度附近的实时候选种子及连续坡面传播确认。 |
| `rog_map.projection.ground_seed_tolerance`, `.ground_seed_radius` | 0.18 m, 2.50 m | 首个地面种子相对车体接地点的高度窗与搜索半径。当前车身下方净空已验证的薄地面会先用中位数校准姿态切平面的高度截距；包络外仍按 `max(max_ground_step, resolution)` 的严格误差门槛匹配校准后的平面，避免远处独立薄板自封为地面。 |
| `rog_map.projection.ground_connectivity_quantile` | 0.20 | 仅在普通地面连通失败时，对坡面横向窄带高度取较低分位数，抑制稀疏回波和体素离散造成的单点高度跳变。不要调到 0，否则一个异常低点就可能主导坡面。 |
| `rog_map.projection.ground_connectivity_quantile_lateral_radius` | 0.15 m | 坡面分位数的横向采样半径，总宽度约 0.30 m。过小对缺点敏感，过大会跨到坡边或相邻平台。 |
| `rog_map.projection.ground_connectivity_quantile_min_samples` | 3 | 每个坡面截面至少需要的净空已验证地面格数量；不足时保持阻塞。 |
| `rog_map.projection.ground_connectivity_fit_residual_tolerance` | 0.05 m | 分位数高度相对拟合坡面的最大残差，应与 ROG 体素分辨率同量级；增大可能把真实台阶拟合成坡。 |
| `rog_map.projection.observed_empty_as_free`, `.bridge_observed_empty_for_ground_connectivity` | true, true | 仅放行车身体积已观测为空的列，并允许一次最长 0.60 m、有落地点与连续坡面证据的盲带桥接；UNKNOWN 和占据列不能参与。 |
| `rog_map.projection.near_field_prior_fill_enable` | false | 该路径依赖先验高程，无高程模式必须关闭。 |
| `rog_map.decay.keep_time`, `.clear_time` | 3 / 5 s | ROG 占据证据的保留/清理时标；云台慢转时短暂消失的障碍不能立刻当作 free。 |

洞口净空不足时，先测**最高的实际车体部件及其运动姿态**，核对
`vehicle_height + headroom_margin`、LiDAR 外参和 `rog_map.projection` 可视化；
二维 costmap 的 `robot_radius/inflation_radius` 管不到头顶。规划路径已生成却不动，
应先区分 `/astar_path_vis` 已有但 `/opt_path_vis` 没有、ROG 净空未确认、
`/minco/opt_path` 被 BLOCK、还是 MPC 因输入陈旧停机；不能一律增大
`trajectory_timeout` 或把 unknown 当 free。

## 5. `controller_server`：MPC 跟踪与启停行为

修改位置：`controller_server.ros__parameters` 及其 `MincoMpc` 子项。

| 参数 | 当前值 | 含义与调整方向 |
|---|---:|---|
| `controller_frequency` | 20 Hz | 控制器调用频率；需与 CPU、里程计和 costmap 更新匹配，不是规划器轨迹发布频率。 |
| `failure_tolerance` | 0.30 s | 控制插件抛异常后允许的最长持续时间，超时 FollowPath 失败；不是传感器新鲜度阈值。 |
| `progress_checker.required_movement_radius`, `.movement_time_allowance` | 0.05 m / 20 s | SimpleProgressChecker 在时间窗内期望的位置变化；当前窗口允许坡上慢速净位移和短暂观测停顿，不应因尚未移动 0.10 m 就反复中止并清图。 |
| `general_goal_checker.xy_goal_tolerance`, `.yaw_goal_tolerance` | 0.10 m, 6.28 rad | Nav2 目标完成阈值；MINCO `minco_optimizer.traj_goal_tolerance: 0.08 m` 严格小于 XY 目标容差，避免内部 FSM 先停而 FollowPath 无法完成，也避免车辆仍在坡面时过早结束。 |
| `MincoMpc.odom_topic`, `.odom_timeout` | `/lidar_odometry`, 0.25 s | 控制器用补偿后 `odom` 里程计；接收时间**和消息时间戳**都需新鲜。超时先查 Point-LIO/loam_interface/时钟。 |
| `MincoMpc.trajectory_timeout`, `.future_stamp_tolerance` | 1.50 / 0.05 s | NORMAL 轨迹接收与时间戳的新鲜度、未来时间容忍；调大仅延迟停机，无法制造有效新轨迹。 |
| `MincoMpc.dt`, `.lookahead_time` | 0.05 / 0.50 s | MPC 离散步长和预测时域，当前约 10 步；提高预测时域或缩小步长会增加求解负载。 |
| `MincoMpc.q_along`, `.q_cross`, `.Q`, `.R` | 3, 12, [3,3,2], [1.5,1.5,1] | 沿参考朝向/横向的位置误差权重、状态权重和控制权重。当前实现用 `q_along/q_cross` 替代 XY 的 `Q[0:2]`，`Q[2]` 管 yaw；横向偏差大可谨慎增加 `q_cross`，指令振荡可谨慎增加 `R`。先核对路径与定位。 |
| `MincoMpc.max_planar_speed`, `.omega_min/max` | 1.0 m/s, ±0.8 rad/s | 平移合速度与角速度输出上限；`vx/vy_min/max: ±1.5 m/s` 是各轴边界，不代表允许合速度达到 1.5 m/s。 |
| `MincoMpc.use_acc_constraints`, `.ax/ay_min/max`, `.alpha_min/max` | true, ±0.8 m/s², ±2 rad/s² | MPC 动力学变化限制；起步拖沓先查死区、参考推进和底盘反馈，再按实测加速度调。 |
| `MincoMpc.control_delay_compensation` | 0.05 s | 外推车体状态/参考时刻以补偿控制延迟；改动前用时间戳和实际跟踪滞后估计，不要拿来修方向反转。 |
| `MincoMpc.reference_progress_max_lead_time` | 0.25 s | 时间参考允许领先空间进度的上限；可缓解零速冷启动，但过大可能要求车追逐尚未到达的轨迹。 |
| `MincoMpc.reference_startup_max_lead_time` | 1.50 s | 里程计平移速度不高于死区时使用的启动搜索窗口；与 `lookahead_time` 之和不得超过规划器 `safety.check_horizon`，车辆越过死区后恢复普通领先限制。 |
| `MincoMpc.reference_startup_target_speed` | 0.08 m/s | 低速启动时搜索的首个参考速度，必须高于 `deadzone_speed_threshold`，但不能高于安全前缀的速度上限。只有找到该参考点才允许启动补偿。 |
| `MincoMpc.reference_startup_min_command_speed` | 0.20 m/s | 实车克服静摩擦的启动命令下限，只在车速不高于死区且已有安全参考方向时生效；该值与观测前沿速度上限一致，使车辆能先接触短坡，再由姿态触发上坡助推。 |
| `MincoMpc.uphill_startup_min_grade`, `.uphill_full_assist_grade` | 0.03, 0.18 | 上坡平滑补偿的起点与满助推坡度，约为 1.7 deg 和 10.2 deg。两者之间使用两端斜率为零的平滑曲线，不再跨过阈值就硬跳速度。 |
| `MincoMpc.uphill_assist_min_reference_speed` | 0.30 m/s | 车辆离开启动死区后，轨迹峰值至少达到此值才持续上坡助推。它独立于最大助推速度，避免助推随车速反复开关，同时排除最高仅 0.20 m/s 的观测前缀停车轨迹。 |
| `MincoMpc.uphill_startup_min_command_speed` | 0.90 m/s | 参数名为兼容旧配置保留；当前含义是陡坡满助推目标。实际命令从 MPC 原始值随坡度平滑靠近该值，并继续受 Nav2 动态限速、`max_planar_speed` 和 `slope_speed_limit` 约束。 |
| `MincoMpc.deadzone_speed_threshold` | 0.05 m/s | 求解出的平移速度低于阈值时置零；若小速度指令被截断，先与底盘实际可动阈值比对。 |
| `MincoMpc.slope_slowdown_start_angle`, `.slope_full_slowdown_angle`, `.slope_speed_limit` | 0.08 rad, 0.18 rad, 0.90 m/s | 按实测 roll/pitch 倾角从平地速度逐步限到坡道速度。坡道卡住先看坡面支撑/限速是否触发，再调速度。 |

实车低速调试建议先检查 `/minco/cmd_vel_mpc`（原始控制调试输出）、
`/cmd_vel_nav2_result` 和最终 `/cmd_vel`，并与 `/lidar_odometry` 比较。
**车向与命令方向相反**时，先验证坐标轴、TF、底盘接口和
`fake_vel_transform` 的变换；不能靠取负 `Q/R`、放宽碰撞阈值或改 odom topic
“修复”方向。停止条件、急停与安全员始终优先于参数实验。

## 6. 常见症状的排查顺序

| 现象 | 先观察 | 确定原因后再考虑 |
|---|---|---|
| 只看到一条直线，车却有局部轨迹 | `/plan` 对比 `/astar_path_vis` 和 `/opt_path_vis` | token 占位不是全局搜索失败；检查 RViz Path 显示和话题订阅。 |
| 动态障碍无膨胀或全局仍穿障碍 | `/rog_map/dynamic_obstacles`、两张 `*/costmap`、动态层投影日志 | 检查插件/TF/过期/地图窗口，最后再调 `inflation_radius` 和 SMAC 软代价。 |
| 不走期望洞口、贴墙或局部捷径异常 | `/astar_path_vis`、洞宽、静态地图、动态遮罩、坡边和局部候选 | 分清硬遮罩与软代价，分别检查 `collision_distance`、`cost_penalty` 和 shortcut slack；不能减小实车尺寸凑通路。 |
| 路已规划但长时间犹豫，转云台后立刻移动 | `/opt_path_vis`、ROG `clearance_status`、`/minco/opt_path`、里程计频率和 MPC stop reason | 优先补观测/标定与已测地面；安全前缀慢行用 `observed_prefix_max_velocity`，不要将 unknown 改为 free。 |
| 上坡停住或重新规划不断变化 | 地面高程、`max_step/max_slope_deg`、支持/净空、坡道限速、规划失败日志 | 确认地图坡面与实车对齐，隔离定位抖动、观测波动和 MPC 倾角限制后一次只改一项。 |
| 有正常优化轨迹但输出为零 | `/minco/opt_path` 的状态、`/lidar_odometry` 的 frame/stamp、MPC 日志 | 按 `NO_TRAJECTORY`、`BLOCKED`、`ODOMETRY_RX_STALE`、`TRAJECTORY_RX_STALE`、`QP_FAILED` 分别排查。 |

建议记录同一目标的 `/tf`、`/tf_static`、`/map`、
`/cloud_registered_full`、`/aft_mapped_to_init`、`/lidar_odometry`、
`/rog_map/dynamic_obstacles`、`/rog_map/clearance_status`、
`/astar_path_vis`、`/opt_path_vis`、`/minco/opt_path`、两张 costmap
和 `/rosout`。修改前后均保留 YAML 副本及地图版本，在同一路段做低速正反向验证；
一旦出现占据物漏检、越过未观测区域、碰撞风险或里程计/TF 异常，立即停止试验并恢复
上一个已验证配置。

参数语义参考：本仓库 `MincoPlanner`、`MincoMpc`、
`RogDynamicObstacleLayer` 和 `ROG-map` 的实现；Nav2 Jazzy 官方
[Controller Server](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/core_servers/controller_server/)、
[Costmap 2D](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/core_servers/costmap_2d/)、
[Planner Server](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/core_servers/configuring_planner_server/)
及 [Inflation Layer](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/core_servers/costmap_2d/costmap_plugins/inflation/)。
