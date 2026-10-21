# 实车与仿真 MINCO 参数差异表

比较日期：2026-10-13。按当前工作区 YAML 解析结果逐项比较，不采用旧教程中的历史数值，也没有修改两份配置。

来源：[实车 minco_params.yaml](/home/pnx/nav_ws/sentry-navigation-RM27/src/pb2025_nav_bringup/config/reality/minco_params.yaml)、[仿真 minco_params.yaml](/home/pnx/nav_ws/sentry-navigation-RM27/src/pb2025_nav_bringup/config/simulation/minco_params.yaml)。

两份文件共有 **246 个字段差异**；排除 **35 个车辆几何与外参字段**后，下面完整列出 **211 个字段**。数量包括只在一侧存在的节点/参数、各节点的时钟差异和当前关闭分支中的设置；不代表有 211 处当前生效的算法差异。

排除范围是车身轮廓/圆半径/长宽高、离地高度、自车点云过滤包尺寸和位置、雷达/IMU安装外参。速度、加速度、过滤余量、避障距离等可调策略仍列出。

“未配置”表示这一侧 YAML 没有该字段。对于共同启动的节点，它可能使用默认值；对于实车或仿真专用节点，则可能根本不启动。因此不能把“未配置”直接当成 false 或 0。表中“R行/S行”是实车/仿真文件的当前行号。

参数前缀：

| 缩写 | 完整位置 |
| --- | --- |
| C | controller_server.ros__parameters.MincoMpc |
| P | planner_server.ros__parameters.MincoPlanner |
| R | planner_server.ros__parameters.MincoPlanner.rog_map |
| L | local_costmap.local_costmap.ros__parameters |
| G | global_costmap.global_costmap.ros__parameters |

其余节点省略中间的 ros__parameters。

## 最影响实际行为的差异

实车平面合速度上限为 1.0 m/s，仿真为 1.5 m/s；观测边界截短路径的速度上限分别为 0.2 和 1.0 m/s。实车还采用更严格的净空未知判定、更短的地图新鲜度门槛及更大的全局膨胀代价惩罚。这些差异会直接影响停车频率、平均速度以及绕行选择。

仿真全局动态层仅标记 2 m 内的实测障碍，实车不限范围；仿真关闭额外的 ROG 全局动态硬碰撞门，实车开启。但两边的 Nav2 动态障碍层和 MINCO 局部安全检查均仍开启。

两边当前都为无先验高程方案。以下共同值用于理解后面的“未生效”说明：

| 参数 | 两边共同值 | 含义 |
| --- | --- | --- |
| P.rog_map.projection.prior_map.require_ground_support | false | 不要求先验高程支撑；仍保留静态 PGM 和实测地面/净空检查。 |
| P.allow_unknown_motion | true | 允许局部轨迹经过 ROG UNKNOWN；不会自动放行已确认实体障碍。 |
| P.allow_unknown | false | 全局搜索的未知格策略，与上述局部运动开关不同。 |
| R.projection.unknown_as_occupied | false | 普通 ROG 未知状态不直接当作占据。 |
| R.projection.prior_free_uncertain_obstacle_confirmations | 4 | 先验白格中的不确定障碍需连续确认；此次两边相同。 |
| P.priormap.ground_edge_avoidance.enable | false | 关闭基于先验高程的全局断边避让。 |
| P.smac_2d.use_esdf_cost | false | 不使用额外 ESDF 全局软代价。 |
| R.esdf.enable | false | 关闭传统 3D ESDF；两边仍开启 2D field。 |
| R.projection.near_field_prior_fill_enable | false | 关闭近场先验补未知区域。 |
| fake_vel_transform.terrain_guard_enabled | false | MINCO 不用 legacy terrain_map 新鲜度作为停车门槛。 |

仿真启动默认 use_ground_truth_odom=true。若改成 false，launch 会从仿真文件的 minco_input_profiles.point_lio 覆盖主规划器的坐标系、点云/里程计输入、雷达位置以及 min_headroom_known_ratio；该比例会从默认 0.1 变为 **0.4**。因此文件中的主配置不一定就是该可选分支的最终运行参数。

## 1. 控制器与进度、到达判定

| 参数 | 实车 | 仿真 | 含义及当前适用条件 | R行/S行 |
| --- | --- | --- | --- | --- |
| `C.alpha_max` | `2` | `3` | 正向角加速度上限，rad/s²。 | 126/128 |
| `C.alpha_min` | `-2` | `-3` | 负向角加速度下限，rad/s²。 | 125/127 |
| `C.ax_max` | `0.8` | `1.8` | X 方向正加速度上限，m/s²。 | 122/124 |
| `C.ax_min` | `-0.8` | `-1.8` | X 方向负加速度下限，m/s²。 | 121/123 |
| `C.ay_max` | `0.8` | `1.8` | Y 方向正加速度上限，m/s²。 | 124/126 |
| `C.ay_min` | `-0.8` | `-1.8` | Y 方向负加速度下限，m/s²。 | 123/125 |
| `C.control_delay_compensation` | `0.05` | `0.12` | 控制延迟补偿时间，s；将参考推进到预计指令生效时刻。 | 71/73 |
| `C.lookahead_time` | `0.5` | `1` | MPC 预测时域，s；仿真看得更远，dt 相同条件下使用更多预测步。 | 67/67 |
| `C.max_planar_speed` | `1` | `1.5` | 平面合速度上限，m/s；限制 sqrt(vx²+vy²)，不是单轴上限。 | 117/119 |
| `C.odom_topic` | `/lidar_odometry` | `/odometry` | 控制器运动状态输入：实车使用补偿到 odom 的雷达里程计，仿真使用车体里程计。 | 61/59 |
| `C.omega_max` | `0.8` | `1.2` | 正向角速度上限，rad/s。 | 119/121 |
| `C.omega_min` | `-0.8` | `-1.2` | 负向角速度下限，rad/s。 | 118/120 |
| `C.performance.mpc_csv_path` | `/tmp/rm27_minco_mpc_reality.csv` | `/tmp/rm27_minco_mpc_simulation.csv` | 性能 CSV 输出文件路径；相关 detailed/summary CSV 开关当前为 false，此路径未用于持续写入。 | 139/139 |
| `C.performance.scenario` | `reality` | `simulation` | 性能记录中的场景标签；不改变规划/控制行为。 | 143/143 |
| `C.q_cross` | `12` | `120` | 横向偏离参考路径的惩罚权重；仿真为实车的 10 倍，更强调贴合路径。 | 111/109 |
| `C.recovery_max_distance` | `0.15` | `0.2` | 控制器允许的脱困最大位移，m；应和规划器脱困范围一致。 | 92/89 |
| `C.recovery_max_speed` | `0.15` | `0.2` | 控制器脱困速度上限，m/s；脱困时不使用普通起步/上坡提速。 | 91/88 |
| `C.reference_progress_max_lead_time` | `0.25` | `0.05` | 正常跟踪时，参考时间游标最多领先车体空间进度的时间，s；仿真更严格限制参考超前。 | 74/76 |
| `C.reference_startup_max_lead_time` | `1.5` | `0.85` | 静止起步时向前搜索可运动参考速度的最大时间窗口，s。 | 81/83 |
| `C.reference_startup_min_command_speed` | `0.4` | `0.6` | 普通静止起步的最小指令速度幅值，m/s；有有效轨迹方向时用于克服低速起步死区。 | 89/86 |
| `C.slope_slowdown_start_angle` | `0.08` | `0.06` | 车体倾角达到此值后开始逐渐执行坡面限速，rad；0.08≈4.6°，0.06≈3.4°。 | 106/101 |
| `C.slope_speed_limit` | `0.8` | `1.15` | 充分触发倾斜限速后的速度上限，m/s；不是所有坡面瞬间采用的固定速度。 | 108/105 |
| `C.uphill_assist_min_reference_speed` | `0.3` | `0.25` | 进入上坡速度辅助所需的参考速度门槛，m/s；低速短前缀不会无条件被提速。 | 102/96 |
| `C.uphill_startup_min_command_speed` | `0.9` | `1` | 上坡辅助的目标最低指令速度，m/s；按坡度渐进混合，受速度限幅及有效轨迹条件限制。 | 103/97 |
| `controller_server.enable_stamped_cmd_vel` | 未配置 | `false` | 是否发布带时间戳的 TwistStamped；仿真显式 false，实车使用其安装版本/启动配置的默认值。 | —/148 |
| `controller_server.general_goal_checker.xy_goal_tolerance` | `0.1` | `0.2` | Nav2 最终到达的平面距离容差，m；实车要求更靠近目标。 | 166/162 |
| `controller_server.odom_topic` | `lidar_odometry` | `odometry` | 该节点使用的里程计话题；实车和仿真的输入链不同。 | 147/147 |
| `controller_server.progress_checker.movement_time_allowance` | `20` | `15` | 达到 required_movement_radius 所允许的最长时间，s；超时会判定缺少运动进展。 | 160/158 |
| `controller_server.progress_checker.required_movement_radius` | `0.05` | `0.1` | 被认定为有导航进展所需的最小移动距离，m。 | 159/157 |

## 2. 全局与局部代价地图

| 参数 | 实车 | 仿真 | 含义及当前适用条件 | R行/S行 |
| --- | --- | --- | --- | --- |
| `G.inflation_layer.enabled` | `true` | 未配置 | Nav2 膨胀层开关；仿真未显式写出不表示关闭，应使用插件默认/运行参数判定。 | 228/— |
| `G.inflation_layer.inflation_radius` | `0.4` | `0.56` | Nav2 膨胀代价影响半径，m；作用于静态和合并后的动态障碍，不等于车身碰撞尺寸。 | 230/227 |
| `G.rog_dynamic_obstacle_layer.obstacle_marking_max_range` | `0` | `2` | 向全局代价地图标记动态障碍的车体距离上限，m；0 不限范围。仿真只将 2 m 内障碍用于全局绕行，局部层该值两边均为 0。 | 223/218 |
| `G.rog_dynamic_obstacle_layer.stale_timeout` | `0.75` | `2.75` | ROG 动态栅格接收后允许继续使用的时长，s；过期会清除该层旧快照，仿真容忍更长更新间隔。 | 219/213 |
| `G.track_unknown_space` | `true` | 未配置 | 是否在全局代价地图中保留未知格；仿真未显式配置，该字段需结合运行默认值，不能等同 planner allow_unknown_motion。 | 239/— |
| `L.inflation_layer.enabled` | `true` | 未配置 | Nav2 膨胀层开关；仿真未显式写出不表示关闭，应使用插件默认/运行参数判定。 | 188/— |
| `L.inflation_layer.inflation_radius` | `0.4` | `0.56` | Nav2 膨胀代价影响半径，m；作用于静态和合并后的动态障碍，不等于车身碰撞尺寸。 | 190/199 |
| `L.rog_dynamic_obstacle_layer.stale_timeout` | `0.75` | `2.75` | ROG 动态栅格接收后允许继续使用的时长，s；过期会清除该层旧快照，仿真容忍更长更新间隔。 | 180/176 |

## 3. 全局搜索、热启动、轨迹安全与脱困

| 参数 | 实车 | 仿真 | 含义及当前适用条件 | R行/S行 |
| --- | --- | --- | --- | --- |
| `P.exploration.boundary_margin` | `0.3` | `0.5` | 探索模式裁剪局部 ROG 边界时的内缩量，m；当前两边 planner_mode=PRIORMAP，探索分支未启用。 | 297/292 |
| `P.frames.rog_frame` | `camera_init` | `odom` | ROG 内部地图坐标系。实车为 Point-LIO 的 camera_init；仿真默认真值输入为 odom。 | 262/260 |
| `P.hot_start.direction_cosine_threshold` | `0.5` | `0.35` | 实测速度方向与缓存轨迹方向的相似度阈值；较小值允许更大的方向偏差继续热启动。 | 610/673 |
| `P.hot_start.tracking_error_base` | `0.3` | `0.4` | 热启动允许的基础位置跟踪误差，m；仿真更容易保留当前运动边界状态。 | 608/670 |
| `P.hot_start.velocity_error_threshold` | `0.35` | `0.45` | 热启动允许的速度误差，m/s；仿真更容忍跟踪速度偏差。 | 609/671 |
| `P.local_path.observed_prefix_max_velocity` | `0.2` | `1` | 路径被观测边界截短时，该短前缀的最高速度，m/s；不是全局速度上限。 | 605/666 |
| `P.odom_topic` | `/aft_mapped_to_init` | `/lidar_odometry` | 规划起点状态输入。仿真默认真值路径用 /lidar_odometry；切换 Point-LIO 时会被 launch 改为 /aft_mapped_to_init。 | 310/303 |
| `P.performance.minco_csv_path` | `/tmp/rm27_minco_planner_reality.csv` | `/tmp/rm27_minco_planner_simulation.csv` | 性能 CSV 输出文件路径；相关 detailed/summary CSV 开关当前为 false，此路径未用于持续写入。 | 341/351 |
| `P.performance.scenario` | `reality` | `simulation` | 性能记录中的场景标签；不改变规划/控制行为。 | 345/355 |
| `P.priormap.dynamic_global_obstacle.collision_distance` | `0.34` | `0.4` | 上述额外 ROG 动态硬碰撞约束的距离，m；仿真该分支已关闭，此值当前不生效。 | 277/270 |
| `P.priormap.dynamic_global_obstacle.enable` | `true` | `false` | 是否额外用 ROG 实测障碍构造全局搜索硬碰撞约束；仿真 false 仍保留 Nav2 的 ROG 动态障碍层和局部安全检查。 | 272/269 |
| `P.priormap.ground_edge_avoidance.clearance_radius` | `0.3` | `0.46` | 高程断边周围添加软代价的范围，m；两边 enable=false，当前不生效。 | 294/289 |
| `P.priormap.ground_edge_avoidance.lethal_clearance_radius` | `0.3` | `0.44` | 高程断边周围的硬禁止通行范围，m；两边 enable=false，当前不生效。 | 293/288 |
| `P.priormap.ground_edge_avoidance.max_slope_deg` | `28` | `20` | 先验高程断边判定的最大可通行坡度，度；两边 enable=false，当前不生效。不要与当前 ROG 坡度阈值混淆。 | 289/284 |
| `P.priormap.ground_edge_avoidance.max_step` | `0.06` | `0.05` | 先验高程邻格允许的最大台阶高度，m；两边 enable=false，当前不生效。 | 288/283 |
| `P.priormap.rog_boundary_margin` | `0.3` | `0.5` | PRIORMAP 模式将局部种子路径裁剪到 ROG 有效区域时的边界余量，m。 | 279/272 |
| `P.recovery_server.escape_speed` | `0.15` | `0.2` | 规划器生成脱困轨迹采用的速度，m/s。 | 666/739 |
| `P.recovery_server.overlap_max_distance` | `0.15` | `0.2` | 允许在受控起点重叠脱困中的最大位移，m。 | 670/743 |
| `P.recovery_server.overlap_max_speed` | `0.15` | `0.2` | 允许在受控起点重叠脱困中的最大速度，m/s。 | 668/741 |
| `P.safety.collision_cache_reuse_max_duration` | `0.4` | `2` | 碰撞/安全检查拒绝新轨迹后，允许继续复用经重新检查的旧轨迹尾段的最大时长，s；不意味着沿已知碰撞轨迹继续走。 | 331/342 |
| `P.safety.map_timeout` | `0.5` | `2.5` | 安全检查接受 ROG 地图快照的最大年龄，s；实车更容易因地图不新鲜而停下。 | 326/328 |
| `P.smac_2d.cost_penalty` | `8` | `2` | SMAC 全局搜索对合并代价地图代价的惩罚强度；实车更倾向远离膨胀带，仿真更愿意走狭窄通道。 | 585/645 |
| `P.smac_2d.esdf_decay` | `0.3` | `0.15` | ESDF 软代价随障碍距离衰减的尺度，m；两边 use_esdf_cost=false，当前不生效。 | 593/656 |
| `P.smac_2d.esdf_max_cost` | `4` | `8` | ESDF 软代价上限；两边 use_esdf_cost=false，当前不生效。 | 594/657 |
| `P.smac_2d.esdf_weight` | `4` | `8` | ESDF 软代价权重；两边 use_esdf_cost=false，当前不生效。 | 592/655 |
| `P.tolerance` | `0.1` | `0.3` | 全局搜索允许用目标附近可通行点替代目标的范围，m；不是控制器最终到达容差。 | 303/296 |

## 4. MINCO 轨迹优化

| 参数 | 实车 | 仿真 | 含义及当前适用条件 | R行/S行 |
| --- | --- | --- | --- | --- |
| `P.minco_optimizer.failed_replan_retry_period` | `0.5` | `0.25` | 优化/规划失败后的重试间隔，s；仿真重试更快，不改变独立安全监测频率。 | 615/680 |
| `P.minco_optimizer.guide_lateral_tolerance` | `0.01` | `0.003` | 轨迹偏离引导线后才开始额外惩罚的容差，m；实车 guide_lateral_weight=0 时此项不起作用。 | 658/733 |
| `P.minco_optimizer.guide_lateral_weight` | `0` | `1000000` | 整段轨迹横向偏离引导线的惩罚；实车关闭，仿真用较大权重抑制洞口切角。 | 657/732 |
| `P.minco_optimizer.max_acceleration` | `0.8` | `1.8` | 规划轨迹加速度上限，m/s²；与控制器的加速度约束分别生效。 | 633/709 |
| `P.minco_optimizer.max_trajectory_duration` | `15` | `20` | 单条局部轨迹允许的最长时长，s；超过该值的优化结果会被拒绝。 | 641/712 |
| `P.minco_optimizer.max_velocity` | `1.2` | `1.5` | 轨迹优化平移速度上限，m/s；实车最终指令还受 MPC 的 1.0 m/s 上限约束。 | 632/704 |
| `P.minco_optimizer.max_yaw_dot` | `0.8` | `1.2` | 规划姿态轨迹的最大偏航角速度，rad/s。 | 642/713 |
| `P.minco_optimizer.obstacle_slowdown_distance` | `0` | `0.5` | 引导路径点离障碍距离小于此值时降低整条局部轨迹的速度上限，m；0 表示关闭。仿真触发后使用两边均为 1.0 m/s 的 obstacle_max_velocity。 | 659/707 |
| `P.minco_optimizer.opt_accuracy` | `0.001` | `0.00001` | 数值优化收敛精度；仿真更严格，可能需要更多优化计算。 | 650/721 |
| `P.minco_optimizer.penalty_weight_att` | `100` | `800` | 航点吸引惩罚权重，att 指 attractor；增大后更重视引导航点位置，不是车身倾斜惩罚。 | 656/729 |
| `P.minco_optimizer.safe_dist` | `0.3` | `0.33` | 优化器到障碍的软距离目标，m；不等于硬碰撞盒尺寸或车高。 | 624/697 |
| `P.minco_optimizer.successful_replan_period` | `0.5` | `0.35` | 成功规划后的常规滚动重规划间隔，s。 | 616/683 |
| `P.minco_optimizer.terminal_velocity_ratio` | `0.8` | `0.85` | 滚动局部轨迹末端速度相对当前轨迹速度上限的比例；增大有利于连续接段。 | 637/710 |
| `P.minco_optimizer.traj_goal_tolerance` | `0.08` | `0.15` | 规划内部 FSM 判断到达终点的距离，m；与 Nav2 goal_checker 分开。 | 620/687 |

## 5. ROG 地面、洞顶净空与未知格

| 参数 | 实车 | 仿真 | 含义及当前适用条件 | R行/S行 |
| --- | --- | --- | --- | --- |
| `R.projection.clearance_dropout_hold_time` | `0.2` | `1.5` | 已验证净空因短暂缺测丢失时的保持时间，s；当前代码要求 require_ground_support=true，两边当前 false，此分支未启用。 | 521/579 |
| `R.projection.clearance_hole_fill_enable` | `false` | `true` | 启用基于已知高程支撑的净空小孔填补；还要求 require_ground_support=true，因此当前两边均不执行。 | 524/583 |
| `R.projection.clearance_hole_fill_max_width` | `0.05` | `0.2` | 上述净空孔洞填补允许的最大宽度，m；当前不生效。 | 525/584 |
| `R.projection.clearance_unknown_as_occupied` | `true` | `false` | 净空已观测比例不足时，是否按占据/不可通行处理；实车更保守，仿真可保留为 UNKNOWN。 | 508/559 |
| `R.projection.ground_connectivity_bridge_landing_min_length` | `0.15` | `0.25` | 确认盲区对面为连续实测坡面所需的最短落地区长度，m。 | 442/470 |
| `R.projection.ground_connectivity_bridge_max_length` | `0.6` | `0.45` | 连续性检查最多允许一次跨过的已观测空白盲区长度，m；不是无条件跨越 UNKNOWN 或断崖。 | 438/466 |
| `R.projection.ground_connectivity_hold_time` | `0` | `0.8` | 短暂观测波动时保持已确认地面连通性的时长，s；仿真减少连通性闪断，实车为 0。 | 494/532 |
| `R.projection.ground_seed_radius` | `2.5` | `2.6` | 地面连通搜索的种子范围半径，m。 | 501/545 |
| `R.projection.headroom_voxel_inset_fraction` | `0.5` | `0` | 计算净空时按体素尺寸保守内缩的比例；0.5 为半体素，0 为不内缩。当前 0.05 m 分辨率下半体素为 0.025 m。 | 485/523 |
| `R.projection.max_ground_step` | `0.08` | `0.1` | 局部实测地面连通性允许的最大台阶高度，m；仿真允许更大的相邻高度变化。 | 499/540 |
| `R.projection.min_headroom_known_ratio` | `0.15` | `0.1` | 车体竖向净空区间要求的最低已观测比例；实车 15%，仿真默认 10%，仿真 Point-LIO 分支覆盖成 40%。 | 502/551 |
| `R.projection.near_field_prior_fill_length` | `1.2` | `1.4` | 近场先验填未知区的长度，m；两边 near_field_prior_fill_enable=false，当前不生效。 | 479/516 |
| `R.projection.near_field_prior_fill_width` | `1.2` | `1` | 近场先验填未知区的宽度，m；两边 near_field_prior_fill_enable=false，当前不生效。 | 480/517 |
| `R.projection.observed_ground_support_bridge_enable` | `false` | `true` | 基于已观测支撑邻居填补先验支撑小孔；代码还要求 require_ground_support=true，当前两边均不执行。 | 456/486 |
| `R.projection.observed_ground_support_bridge_hysteresis_count` | `3` | `2` | 上述支撑桥接的确认次数；当前不生效。 | 459/489 |
| `R.projection.observed_ground_support_bridge_max_height_delta` | `0.04` | `0.06` | 上述支撑桥接允许的邻居高度差，m；当前不生效。 | 458/488 |
| `R.projection.observed_ground_support_bridge_min_neighbors` | `3` | `2` | 上述支撑桥接要求的最少已观测支撑邻格数；当前不生效。 | 457/487 |
| `R.projection.obstacle_hold_time` | `0.5` | `0` | 实测障碍变为可通行证据后仍保留障碍的时间，s；实车减少闪烁，但清除瞬时误判较慢。 | 518/574 |
| `R.projection.prior_map.free_fills_unknown` | `false` | `true` | 是否用原始 PGM 白格填补 ROG 的动态 UNKNOWN；仿真开启。该分支不会直接覆盖已确认实测障碍。 | 534/593 |
| `R.projection.prior_map.ground_support_tolerance` | `0.08` | `0.1` | 启用先验地面支撑时，实测地面与先验高度的容差，m；两边 require_ground_support=false，当前支撑授权分支未启用。 | 538/597 |
| `R.projection.prior_map.transform_timeout` | `0.5` | `0.75` | 先验地图与 ROG 地图之间 TF 查询等待上限，s。 | 539/598 |

## 6. ROG 地图更新、输入与可视化

| 参数 | 实车 | 仿真 | 含义及当前适用条件 | R行/S行 |
| --- | --- | --- | --- | --- |
| `R.cloud_filter.box_padding` | `0.02` | `0.005` | 自车点云过滤包的额外扩张量，m/侧；增大会滤掉更多边缘回波，也可能误滤近处环境。 | 390/405 |
| `R.cloud_filter.pose_child_frame` | `imu_link` | 未配置 | 自车过滤使用的配对里程计子坐标系覆盖项；仿真未配置时使用消息自带子坐标系。 | 381/— |
| `R.cloud_filter.transform_timeout` | `0.1` | `0.2` | 自车点云过滤的 TF 查询等待上限，s。 | 385/399 |
| `R.esdf.local_update_box` | `[10,10,2]` | `[6,6,2]` | 传统 3D ESDF 局部更新盒尺寸，m；两边 esdf.enable=false，此设置当前不生效。不要与已开启的 2D field 混淆。 | 357/370 |
| `R.frame_id` | `camera_init` | `odom` | ROG 内部地图坐标系；仿真切换 Point-LIO 时 launch 会改为 camera_init。 | 348/358 |
| `R.map_size` | `[10,10,2]` | `[6,6,2]` | ROG 滑动地图的三轴范围，m；实车窗口更大、数据保留范围及计算量也更大。 | 351/364 |
| `R.performance.detailed_csv_path` | `/tmp/rm27_rog_map_reality_detailed.csv` | `/tmp/rm27_rog_map_simulation_detailed.csv` | 性能 CSV 输出文件路径；相关 detailed/summary CSV 开关当前为 false，此路径未用于持续写入。 | 556/615 |
| `R.performance.raycast_num_threads` | `4` | `6` | 射线线程数的后加载兼容别名；与 raycasting.num_threads 各自保持一致。 | 569/627 |
| `R.performance.scenario` | `reality` | `simulation` | 性能记录中的场景标签；不改变规划/控制行为。 | 559/618 |
| `R.performance.summary_csv_path` | `/tmp/rm27_rog_map_reality_summary.csv` | `/tmp/rm27_rog_map_simulation_summary.csv` | 性能 CSV 输出文件路径；相关 detailed/summary CSV 开关当前为 false，此路径未用于持续写入。 | 557/616 |
| `R.raycasting.local_update_box` | `[10,10,2]` | `[6,6,2]` | 点云射线更新的局部空间范围，m。 | 397/422 |
| `R.raycasting.num_threads` | `4` | `6` | 射线更新线程数；仿真更多。实际会被后加载的 performance.raycast_num_threads 别名覆盖。 | 406/434 |
| `R.ros_callback.cloud_timeout` | `0.5` | `1.25` | ROG 接受点云的最大年龄，s；实车对旧点云更严格。 | 368/385 |
| `R.ros_callback.cloud_topic` | `/cloud_registered_full` | `/registered_scan` | ROG 输入的已配准点云话题；仿真 Point-LIO 分支会覆盖为 /cloud_registered_full。 | 365/378 |
| `R.ros_callback.odom_timeout` | `0.25` | `0.5` | ROG 接受里程计的最大年龄，s。 | 367/384 |
| `R.ros_callback.odom_topic` | `/aft_mapped_to_init` | `/lidar_odometry` | ROG 用于同步点云的里程计话题；仿真 Point-LIO 分支会覆盖为 /aft_mapped_to_init。 | 366/379 |
| `R.visualization.frame_id` | `camera_init` | `odom` | ROG 可视化坐标系；仿真 Point-LIO 分支会覆盖为 camera_init。 | 572/630 |
| `R.visualization.range` | `[10,10,2]` | `[3,3,1]` | ROG 可视化的三轴空间范围，m；不改变实际地图大小。 | 574/635 |
| `R.visualization.rate` | `5` | `0.5` | ROG 可视化发布频率，Hz；实车更新更频繁，仿真更省显示开销。 | 573/634 |

## 7. 行为树、速度接口与记录

| 参数 | 实车 | 仿真 | 含义及当前适用条件 | R行/S行 |
| --- | --- | --- | --- | --- |
| `bt_navigator.default_server_timeout` | `100` | `500` | 行为树动作/服务节点默认响应超时，ms；不是整条导航允许的总时间。 | 15/16 |
| `bt_navigator.odom_topic` | `lidar_odometry` | `odometry` | 该节点使用的里程计话题；实车和仿真的输入链不同。 | 13/14 |
| `fake_vel_transform.odom_topic` | `/lidar_odometry` | `odometry` | 该节点使用的里程计话题；实车和仿真的输入链不同。 | 684/762 |
| `fake_vel_transform.terrain_timeout` | `0.5` | `1.5` | Legacy 地形消息新鲜度门槛，s；两边 terrain_guard_enabled=false，当前 MINCO 不用此门槛停车。 | 699/770 |
| `velocity_smoother.enable_stamped_cmd_vel` | 未配置 | `false` | 速度平滑器使用的速度消息格式开关；仿真显式 false，实车未显式设置。 | —/1163 |
| `velocity_smoother.odom_topic` | `lidar_odometry` | `odometry` | 该节点使用的里程计话题；实车和仿真的输入链不同。 | 1066/1171 |

## 8. Point-LIO 与仿真可选输入覆盖

| 参数 | 实车 | 仿真 | 含义及当前适用条件 | R行/S行 |
| --- | --- | --- | --- | --- |
| `minco_input_profiles.point_lio.frames.rog_frame` | 未配置 | `camera_init` | 仿真切换 Point-LIO 时，将主 ROG/规划帧或可视化帧覆盖为 camera_init；不是额外常驻节点。 | —/1186 |
| `minco_input_profiles.point_lio.odom_topic` | 未配置 | `/aft_mapped_to_init` | 仿真切换 Point-LIO 时，将规划/ROG 同步输入覆盖为 /aft_mapped_to_init。 | —/1187 |
| `minco_input_profiles.point_lio.projection.min_headroom_known_ratio` | 未配置 | `0.4` | 仿真切换 use_ground_truth_odom=false 时，将主 ROG 净空最低已观测比例覆盖成 0.4；默认真值分支不使用此覆盖。 | —/1184 |
| `minco_input_profiles.point_lio.rog_map.frame_id` | 未配置 | `camera_init` | 仿真切换 Point-LIO 时，将主 ROG/规划帧或可视化帧覆盖为 camera_init；不是额外常驻节点。 | —/1191 |
| `minco_input_profiles.point_lio.rog_map.ros_callback.cloud_topic` | 未配置 | `/cloud_registered_full` | 仿真切换 Point-LIO 时，将主 ROG 点云输入覆盖为 /cloud_registered_full。 | —/1193 |
| `minco_input_profiles.point_lio.rog_map.ros_callback.odom_topic` | 未配置 | `/aft_mapped_to_init` | 仿真切换 Point-LIO 时，将规划/ROG 同步输入覆盖为 /aft_mapped_to_init。 | —/1194 |
| `minco_input_profiles.point_lio.rog_map.visualization.frame_id` | 未配置 | `camera_init` | 仿真切换 Point-LIO 时，将主 ROG/规划帧或可视化帧覆盖为 camera_init；不是额外常驻节点。 | —/1196 |
| `point_lio.common.lid_topic` | `livox/lidar` | `velodyne_points` | Point-LIO 输入点云话题；仅启动 Point-LIO 时适用。 仿真默认真值定位时 Point-LIO 不启动。 | 768/872 |
| `point_lio.mapping.acc_norm` | `1` | `9.81` | IMU 加速度单位换算尺度；实车输入以 g 为单位，仿真以 m/s² 为单位，不能当作算法松紧参数互抄。 仿真默认真值定位时 Point-LIO 不启动。 | 795/899 |
| `point_lio.mapping.gravity` | `[0,0,-1]` | `[0,0,-9.81]` | 滤波器使用的重力向量，需与 acc_norm 和 IMU 数据单位匹配。 仿真默认真值定位时 Point-LIO 不启动。 | 810/914 |
| `point_lio.mapping.gravity_init` | `[0,0,-1]` | `[0,0,-9.81]` | 初始化时 IMU 坐标系下的重力向量，需与传感器单位及坐标约定匹配。 仿真默认真值定位时 Point-LIO 不启动。 | 811/915 |
| `point_lio.mapping.gyr_cov_input` | `0.1` | `0.01` | 采用 IMU 输入模型时的角速度噪声协方差；当前两边 use_imu_as_input=false，对应输入模型分支未启用。 仿真默认真值定位时 Point-LIO 不启动。 | 803/907 |
| `point_lio.mapping.imu_time_inte` | `0.005` | `0.01` | IMU 时间积分步长，s。 仿真默认真值定位时 Point-LIO 不启动。 | 791/895 |
| `point_lio.mapping.lidar_meas_cov` | `0.01` | `0.001` | 雷达观测噪声协方差；仿真更小，滤波器更相信其雷达观测。 仿真默认真值定位时 Point-LIO 不启动。 | 796/900 |
| `point_lio.mapping.satu_acc` | `4` | `30` | 加速度饱和检测门槛；应与传感器输入单位匹配。 仿真默认真值定位时 Point-LIO 不启动。 | 793/897 |
| `point_lio.odometry.publish_odometry_without_downsample` | `false` | `true` | 是否发布未经下采样节流的里程计输出；影响输出更新节奏。 仿真默认真值定位时 Point-LIO 不启动。 | 816/920 |
| `point_lio.preprocess.lidar_type` | `1` | `2` | 点云解析类型；1 为 Livox，2 为 Velodyne 格式。 仿真默认真值定位时 Point-LIO 不启动。 | 783/887 |
| `point_lio.preprocess.scan_line` | `4` | `32` | 点云解析使用的线数；需与输入点云格式匹配。 仿真默认真值定位时 Point-LIO 不启动。 | 784/888 |
| `point_lio.preprocess.timestamp_unit` | `3` | `0` | 点字段时间戳单位；3 为纳秒，0 为秒。 仿真默认真值定位时 Point-LIO 不启动。 | 785/889 |

## 9. 重定位与场地点云

| 参数 | 实车 | 仿真 | 含义及当前适用条件 | R行/S行 |
| --- | --- | --- | --- | --- |
| `relocalization_manager.init_pose` | `[0,0,0,0,0,0]` | `[-11.7,2.9,0.26,0,0,0]` | 初始重定位位姿 [x,y,z,roll,pitch,yaw]；仿真对应出生位置，不是车辆外参。 仿真默认真值定位时该重定位链不启动。 | 925/1030 |
| `relocalization_manager.max_delta_xy` | `0.5` | `0.6` | 单次重定位修正每个水平分量允许的最大幅度，m。 仿真默认真值定位时该重定位链不启动。 | 911/1016 |
| `relocalization_manager.max_delta_yaw` | `0.3490658504` | `0.1745329252` | 单次重定位允许的偏航修正，rad；实车约 20°，仿真约 10°。实车 YAML 行内的 10 deg 注释与数值不符。 仿真默认真值定位时该重定位链不启动。 | 913/1018 |
| `relocalization_manager.max_mean_error` | `0.35` | `0.3` | 重定位配准结果允许的最大平均误差；仿真质量门槛更严格。 仿真默认真值定位时该重定位链不启动。 | 908/1012 |
| `relocalization_manager.min_inlier_ratio` | `0.7` | `0.75` | 重定位结果要求的最低内点比例；实车 70%，仿真 75%。 仿真默认真值定位时该重定位链不启动。 | 909/1013 |
| `relocalization_manager.min_inliers` | `800` | `300` | 重定位结果要求的最低内点数；实车要求更多。 仿真默认真值定位时该重定位链不启动。 | 910/1014 |
| `relocalization_manager.prior_neighbor_max_distance` | 未配置 | `0.5` | 配准前保留距离先验点云不超过此范围的源点，m；实车未显式设置，但代码默认值也是 0.5，属于写法差异。 仿真默认真值定位时该重定位链不启动。 | —/1015 |
| `relocalization_manager.prior_pcd_transform` | `[0,0,0,0,0,0]` | `[0,0,1.65,0,0,0]` | 先验 PCD 整体刚体变换 [x,y,z,roll,pitch,yaw]；仿真将场地点云抬高 1.65 m 以对齐世界。 仿真默认真值定位时该重定位链不启动。 | 926/1031 |

## 10. 仿真专用点云与真值定位

| 参数 | 实车 | 仿真 | 含义及当前适用条件 | R行/S行 |
| --- | --- | --- | --- | --- |
| `ign_sim_pointcloud_tool.ang_bottom` | 未配置 | `7.22` | 仿真点云格式转换所用下边缘垂直角，度；仅仿真 Point-LIO 转换链适用。 | —/778 |
| `ign_sim_pointcloud_tool.ang_res_y` | 未配置 | `2.01419` | 仿真点云格式转换所用垂直角间隔，度。 | —/779 |
| `ign_sim_pointcloud_tool.horizon_scan` | 未配置 | `1875` | 仿真点云格式转换的每圈水平采样配置。 | —/777 |
| `ign_sim_pointcloud_tool.n_scan` | 未配置 | `32` | 仿真点云格式转换的线数。 | —/776 |
| `ign_sim_pointcloud_tool.pcd_topic` | 未配置 | `livox/lidar` | 仿真点云转换器的输入点云话题。 | —/775 |
| `rm27_ground_truth_localizer.cycle_no_return_stride_phase` | 未配置 | `false` | 是否逐帧改变无回波重建的抽样相位；当前 false 且两个 stride=1，为每帧完整重建。 仅仿真 use_ground_truth_odom=true 时使用。 | —/814 |
| `rm27_ground_truth_localizer.ground_truth_odom_topic` | 未配置 | `ground_truth/odometry` | 输入 Gazebo 真值里程计话题。 仅仿真 use_ground_truth_odom=true 时使用。 | —/784 |
| `rm27_ground_truth_localizer.lidar_cloud_topic` | 未配置 | `livox/lidar` | 输入仿真雷达点云话题。 仅仿真 use_ground_truth_odom=true 时使用。 | —/785 |
| `rm27_ground_truth_localizer.lidar_frame` | 未配置 | `left_mid360` | 雷达点云的坐标系名。 仅仿真 use_ground_truth_odom=true 时使用。 | —/792 |
| `rm27_ground_truth_localizer.lidar_horizontal_max_angle` | 未配置 | `6.283185307179586` | 无回波射线重建所用水平角上限，rad。 仅仿真 use_ground_truth_odom=true 时使用。 | —/802 |
| `rm27_ground_truth_localizer.lidar_horizontal_min_angle` | 未配置 | `0` | 无回波射线重建所用水平角下限，rad。 仅仿真 use_ground_truth_odom=true 时使用。 | —/801 |
| `rm27_ground_truth_localizer.lidar_horizontal_samples` | 未配置 | `360` | 无回波射线重建所用水平采样数。 仅仿真 use_ground_truth_odom=true 时使用。 | —/799 |
| `rm27_ground_truth_localizer.lidar_odometry_topic` | 未配置 | `lidar_odometry` | 输出用于规划/ROG 同步的雷达原点里程计话题。 仅仿真 use_ground_truth_odom=true 时使用。 | —/787 |
| `rm27_ground_truth_localizer.lidar_vertical_max_angle` | 未配置 | `0.9637708129512688` | 无回波射线重建所用垂直角上限，rad。 仅仿真 use_ground_truth_odom=true 时使用。 | —/804 |
| `rm27_ground_truth_localizer.lidar_vertical_min_angle` | 未配置 | `-0.1260127724939906` | 无回波射线重建所用垂直角下限，rad。 仅仿真 use_ground_truth_odom=true 时使用。 | —/803 |
| `rm27_ground_truth_localizer.lidar_vertical_samples` | 未配置 | `192` | 无回波射线重建所用垂直采样数。 仅仿真 use_ground_truth_odom=true 时使用。 | —/800 |
| `rm27_ground_truth_localizer.map_frame` | 未配置 | `map` | 输出全局地图坐标系。 仅仿真 use_ground_truth_odom=true 时使用。 | —/789 |
| `rm27_ground_truth_localizer.no_return_horizontal_stride` | 未配置 | `1` | 无回波重建水平抽样间隔；1 表示不跳过水平采样。 仅仿真 use_ground_truth_odom=true 时使用。 | —/812 |
| `rm27_ground_truth_localizer.no_return_ray_length` | 未配置 | `10.5` | 重建无回波射线的长度，m；超过 ROG 最大射线距离，以便截断为 free ray。 仅仿真 use_ground_truth_odom=true 时使用。 | —/805 |
| `rm27_ground_truth_localizer.no_return_vertical_stride` | 未配置 | `1` | 无回波重建垂直抽样间隔；1 表示不跳过垂直采样。 仅仿真 use_ground_truth_odom=true 时使用。 | —/813 |
| `rm27_ground_truth_localizer.odom_frame` | 未配置 | `odom` | 输出里程计坐标系。 仅仿真 use_ground_truth_odom=true 时使用。 | —/790 |
| `rm27_ground_truth_localizer.odometry_topic` | 未配置 | `odometry` | 输出车体里程计话题。 仅仿真 use_ground_truth_odom=true 时使用。 | —/786 |
| `rm27_ground_truth_localizer.reconstruct_no_return_rays` | 未配置 | `true` | 是否将 Gazebo 无返回的 inf/nan 样本重建成有方向的未命中射线，供 ROG 更新已观测空闲空间。 仅仿真 use_ground_truth_odom=true 时使用。 | —/798 |
| `rm27_ground_truth_localizer.registered_scan_topic` | 未配置 | `registered_scan` | 输出已配准仿真点云话题。 仅仿真 use_ground_truth_odom=true 时使用。 | —/788 |
| `rm27_ground_truth_localizer.robot_base_frame` | 未配置 | `base_footprint` | 车体参考坐标系名。 仅仿真 use_ground_truth_odom=true 时使用。 | —/791 |
| `rm27_ground_truth_localizer.rog_map_resolution` | 未配置 | `0.05` | 无回波重建使用的 ROG 分辨率，m，应与实际地图一致。 仅仿真 use_ground_truth_odom=true 时使用。 | —/807 |
| `rm27_ground_truth_localizer.rog_raycast_max_range` | 未配置 | `10` | 配合 ROG 射线更新的最大量程，m。 仅仿真 use_ground_truth_odom=true 时使用。 | —/806 |

## 11. 实车雷达驱动

| 参数 | 实车 | 仿真 | 含义及当前适用条件 | R行/S行 |
| --- | --- | --- | --- | --- |
| `livox_ros_driver2.cmdline_input_bd_code` | `livox0000000001` | 未配置 | Livox 设备广播码/设备选择字符串。 仿真用 Gazebo/bridge，未配置此实车驱动节点。 | 710/— |
| `livox_ros_driver2.data_src` | `0` | 未配置 | 驱动输入来源；0 为真实雷达。 仿真用 Gazebo/bridge，未配置此实车驱动节点。 | 705/— |
| `livox_ros_driver2.frame_id` | `left_mid360` | 未配置 | 实车雷达驱动输出消息的坐标系名。 仿真用 Gazebo/bridge，未配置此实车驱动节点。 | 708/— |
| `livox_ros_driver2.lvx_file_path` | `""` | 未配置 | 离线 LVX 文件路径；当前为空，实车使用实时雷达。 仿真用 Gazebo/bridge，未配置此实车驱动节点。 | 711/— |
| `livox_ros_driver2.multi_topic` | `0` | 未配置 | 多雷达话题分配模式；0 为共享话题。 仿真用 Gazebo/bridge，未配置此实车驱动节点。 | 704/— |
| `livox_ros_driver2.output_data_type` | `0` | 未配置 | 输出目的地；0 为 ROS。 仿真用 Gazebo/bridge，未配置此实车驱动节点。 | 707/— |
| `livox_ros_driver2.publish_freq` | `10` | 未配置 | 实车雷达发布频率，Hz。 仿真用 Gazebo/bridge，未配置此实车驱动节点。 | 706/— |
| `livox_ros_driver2.user_config_path` | `$(find-pkg-share pb2025_nav_bringup)/config/reality/mid360_user_config.json` | 未配置 | 实车 MID360 网络、设备及数据等驱动设置文件。 仿真用 Gazebo/bridge，未配置此实车驱动节点。 | 709/— |
| `livox_ros_driver2.xfer_format` | `4` | 未配置 | 驱动消息格式；当前 4 为本仓库的 AllMsg 输出模式。 仿真用 Gazebo/bridge，未配置此实车驱动节点。 | 703/— |

## 12. 各节点时钟

| 参数 | 实车 | 仿真 | 含义及当前适用条件 | R行/S行 |
| --- | --- | --- | --- | --- |
| `behavior_server.use_sim_time` | `false` | `true` | ROS 时钟来源：实车使用系统时间，仿真使用 /clock；节点是否启动由 launch 分支决定，启动也可覆盖该值。 | 28/29 |
| `bt_navigator.use_sim_time` | `false` | `true` | ROS 时钟来源：实车使用系统时间，仿真使用 /clock；节点是否启动由 launch 分支决定，启动也可覆盖该值。 | 10/11 |
| `controller_server.use_sim_time` | `false` | `true` | ROS 时钟来源：实车使用系统时间，仿真使用 /clock；节点是否启动由 launch 分支决定，启动也可覆盖该值。 | 146/146 |
| `fake_vel_transform.use_sim_time` | `false` | `true` | ROS 时钟来源：实车使用系统时间，仿真使用 /clock；节点是否启动由 launch 分支决定，启动也可覆盖该值。 | 688/761 |
| `G.use_sim_time` | `false` | `true` | ROS 时钟来源：实车使用系统时间，仿真使用 /clock；节点是否启动由 launch 分支决定，启动也可覆盖该值。 | 232/230 |
| `ign_sim_pointcloud_tool.use_sim_time` | 未配置 | `true` | ROS 时钟来源：实车使用系统时间，仿真使用 /clock；节点是否启动由 launch 分支决定，启动也可覆盖该值。 | —/774 |
| `loam_interface.use_sim_time` | `false` | `true` | ROS 时钟来源：实车使用系统时间，仿真使用 /clock；节点是否启动由 launch 分支决定，启动也可覆盖该值。 | 831/935 |
| `L.use_sim_time` | `false` | `true` | ROS 时钟来源：实车使用系统时间，仿真使用 /clock；节点是否启动由 launch 分支决定，启动也可覆盖该值。 | 192/182 |
| `map_saver.use_sim_time` | `false` | `true` | ROS 时钟来源：实车使用系统时间，仿真使用 /clock；节点是否启动由 launch 分支决定，启动也可覆盖该值。 | 1029/1133 |
| `planner_server.use_sim_time` | `false` | `true` | ROS 时钟来源：实车使用系统时间，仿真使用 /clock；节点是否启动由 launch 分支决定，启动也可覆盖该值。 | 675/750 |
| `point_lio.use_sim_time` | 未配置 | `true` | ROS 时钟来源：实车使用系统时间，仿真使用 /clock；节点是否启动由 launch 分支决定，启动也可覆盖该值。 | —/818 |
| `pointcloud_to_laserscan.use_sim_time` | `false` | `true` | ROS 时钟来源：实车使用系统时间，仿真使用 /clock；节点是否启动由 launch 分支决定，启动也可覆盖该值。 | 933/1038 |
| `relocalization_manager.use_sim_time` | `false` | `true` | ROS 时钟来源：实车使用系统时间，仿真使用 /clock；节点是否启动由 launch 分支决定，启动也可覆盖该值。 | 901/1005 |
| `rm27_ground_truth_localizer.use_sim_time` | 未配置 | `true` | ROS 时钟来源：实车使用系统时间，仿真使用 /clock；节点是否启动由 launch 分支决定，启动也可覆盖该值。 | —/783 |
| `sensor_scan_generation.use_sim_time` | `false` | `true` | ROS 时钟来源：实车使用系统时间，仿真使用 /clock；节点是否启动由 launch 分支决定，启动也可覆盖该值。 | 840/944 |
| `smoother_server.use_sim_time` | `false` | `true` | ROS 时钟来源：实车使用系统时间，仿真使用 /clock；节点是否启动由 launch 分支决定，启动也可覆盖该值。 | 1037/1141 |
| `velocity_smoother.use_sim_time` | `false` | `true` | ROS 时钟来源：实车使用系统时间，仿真使用 /clock；节点是否启动由 launch 分支决定，启动也可覆盖该值。 | 1058/1162 |
| `waypoint_follower.use_sim_time` | `false` | `true` | ROS 时钟来源：实车使用系统时间，仿真使用 /clock；节点是否启动由 launch 分支决定，启动也可覆盖该值。 | 1047/1151 |

## 13. 默认 MINCO 不启用的地形与 SLAM 配置

| 参数 | 实车 | 仿真 | 含义及当前适用条件 | R行/S行 |
| --- | --- | --- | --- | --- |
| `slam_toolbox.transform_publish_period` | `0.05` | `0` | SLAM 的 map→odom TF 发布周期，s；0 为关闭发布。当前带静态先验的 MINCO 启动不使用 SLAM 分支。 | 975/1079 |
| `terrain_analysis.decayTime` | `0.8` | `1` | 局部地形保留点的时间，s；当前 MINCO 默认关闭 legacy 地形链。 | 848/952 |
| `terrain_analysis.minBlockPointNum` | `8` | `10` | 地形栅格参与障碍点输出需要的最低点数；不是 ROG 障碍确认次数，默认 MINCO 地形链关闭。 | 866/970 |
| `terrain_analysis.noDecayDis` | `0` | `1` | 此距离内的点不按 decayTime 衰减，m；实车 0 表示无该免衰减近圈。 | 849/953 |
| `terrain_analysis.voxelTimeUpdateThre` | `2` | `0.3` | 地形体素重新整理/滤除过期点的时间门槛，s；值越大，整理越不频繁。实车旧注释写着更频繁，与 2.0 数值方向不符。 | 872/976 |
| `terrain_analysis_ext.decayTime` | `0.2` | `1` | 扩展地形点保留时长，s；默认 MINCO 地形链关闭。 | 880/984 |
| `terrain_analysis_ext.voxelTimeUpdateThre` | `0.8` | `0.5` | 扩展地形体素重新整理的时间门槛，s；默认 MINCO 地形链关闭。 | 889/993 |

## 核对依据与说明

- 净空缺测保持、先验支撑桥接和高程净空孔洞填补均在代码中要求 require_ground_support=true，不能因仿真 YAML 写着 enable=true 就认为当前无高程模式正在使用这些机制。见 [projection_layer.cpp](/home/pnx/nav_ws/sentry-navigation-RM27/src/perception/rog_map/src/rog_map/projection_layer.cpp:823)。
- 原始 PGM 白格补 UNKNOWN 由 free_fills_unknown 独立控制，在无先验高程模式仍可生效。见 [prior_map.cpp](/home/pnx/nav_ws/sentry-navigation-RM27/src/perception/rog_map/src/rog_map/prior_map.cpp:942)。
- obstacle_slowdown_distance 对引导路径检查后，可限制整条当前局部轨迹，不只是某个采样点。见 [minco_planner.cpp](/home/pnx/nav_ws/sentry-navigation-RM27/src/navigation/minco_planner/src/minco_core/minco_planner.cpp:2230)。
- 全局动态层 obstacle_marking_max_range=0 表示无限制，超过 stale_timeout 会清除该层旧快照。见 [rog_dynamic_obstacle_layer.cpp](/home/pnx/nav_ws/sentry-navigation-RM27/src/pb_nav2_plugins/src/layers/rog_dynamic_obstacle_layer.cpp:156)。
- 仿真切换 Point-LIO 的参数覆盖见 [navigation_launch.py](/home/pnx/nav_ws/sentry-navigation-RM27/src/pb2025_nav_bringup/launch/navigation_launch.py:132)。
- 实车 relocalization_manager.max_delta_yaw 数值 0.3490658504 实际为 20°，原注释写成 10°；本文以数值为准。terrain_analysis.voxelTimeUpdateThre=2.0 也不表示比 0.3 更频繁整理体素，相关旧注释方向不准确。此次只记录差异，没有改配置或注释。

所有差异字段均由 YAML 结构比较产生，包含数组整体值与单侧缺失项；仅注释不同、字段排列不同不会计入。
