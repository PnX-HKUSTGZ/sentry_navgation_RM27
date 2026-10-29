# RViz 选择出生点 → 全局重定位 → 手动导航

当前默认 PCD 是仿真扫描结合 Gazebo 真值位姿融合的理想先验，导航栅格也是现有仿真地图，**不是 Point-LIO 实际建图产物**。

默认流程：在 RViz 选出生点 → Gazebo 生成机器人 → 自动全局重定位 → 恢复所选朝向并停稳 → 等待手动 Goal。
选择的位置用于生成仿真实体，朝向用于生成及定位后恢复；均不作为定位初值。默认导航为 **MINCO**，使用现有 `config/simulation/minco_params.yaml`。
规划器、MPC、代价地图、速度平滑和底盘坐标转换使用当前工作区的仿真配置。
仿真 MINCO 的目标采用位置到达判定，行驶和到达时保持任务起步朝向，不再在临近目标时转向 Goal 箭头；RViz 初始位姿朝向仍会在全局定位后恢复。这样可避免未完成的末端转向让下一段以斜向车身进入窄洞。详见 [本次慢速过洞修复](tunnel_heading_fix_20261029.md)。

默认导航现已切换为 **Point-LIO**：LiDAR/IMU → Point-LIO → 底盘里程计和点云 → MINCO。
仿真 MINCO 的 ROGMap 现在接收原始扫描与同一时刻 Point-LIO 位姿生成的
`/pointlio/registered_scan`，保留地面、洞顶及无回波射线；前端自身 `/cloud_registered`
继续供现有定位链使用。RMUC2026 的过洞修复和验证见
[观测与路径恢复记录](tunnel_observation_fix_20261028.md)。雷达采样变为 288 线，需要完整重启 Gazebo。
RMUC2026 的 Point-LIO 地图体素为 0.10 m、平面拟合残差门槛为 0.03 m，用于减少窄净空场景的高度不一致。
仿真 MINCO 还启用了 `mapping.deduplicate_map`，在 IVox 内强制执行地图体素去重，避免往返后重复点积累造成里程计延迟。
终端每 5 秒输出 `Frontend health`，可检查单帧耗时、数据年龄及队列长度；详见
[运行后段数据积压修复](pointlio_backlog_fix_20261029.md)。
全局重定位使用原始 LiDAR/IMU 获取起始位姿；`relocalization_manager` 使用该结果对应时刻的
`odom→base_footprint` 计算 `map→odom`。协调器恢复所选朝向并停稳后启动导航。
现在由同一个 `relocalization_manager` 继续执行**停车后的局部 GICP 校正**，独占发布动态
`map→odom`；不再启动一次性 `global_pose_handoff` 节点。Point-LIO 的局部里程计保持连续。
此模式不启动 Gazebo 真值定位节点；Gazebo 真值仅供独立验收程序测量误差。

## 构建与运行

需要原 MINCO 仿真依赖和全局重定位构建；首次安装执行：

```bash
bash tools/global_relocalization/build.bash
source tools/global_relocalization/environment.bash
colcon build --base-paths src --packages-select \
  point_lio loam_interface sensor_scan_generation relocalization_manager rog_map minco_planner pb2025_nav_bringup rm_27_stimulation \
  --symlink-install --cmake-args -DBUILD_TESTING=OFF
```

两种方式都启动 RMUC2026、Gazebo、RViz、全局重定位和 MINCO，使用同一个入口切换。
默认仍为 Point-LIO。在工作区根目录执行以下任一命令。

Point-LIO 里程计：

```bash
bash tools/global_relocalization/navigate.bash use_ground_truth_odom:=false
```

Gazebo 真值相对里程计：

```bash
bash tools/global_relocalization/navigate.bash use_ground_truth_odom:=true
```

Point-LIO 模式使用 LiDAR/IMU 估计运动；真值模式由 `rm27_ground_truth_localizer`
提供相对里程计和注册点云，不启动 Point-LIO。两者都先用原始 LiDAR/IMU 进行一次全局定位，
再初始化 `map→odom`、恢复所选朝向并激活导航。真值模式适合对照定位误差对导航的影响；
它也保留全局初始化误差，不等于把全程位姿直接强制设为地图真值。

切换模式前用 Ctrl+C 结束上一组启动进程，然后执行另一条命令。省略参数时使用 Point-LIO。

Point-LIO 模式默认 `stationary_relocalization:=true`。需要做关闭后端的对照时：

```bash
bash tools/global_relocalization/navigate.bash use_ground_truth_odom:=false stationary_relocalization:=false
```

停车校正要求收到停止指令，且新鲜的 `/odometry` 连续至少 0.75 秒低于线速度 0.05 m/s、
角速度 0.08 rad/s。原地转动也算运动；指令消失不等于停车。每次运动后的静止期间，
需 3 个独立点云窗口通过 GICP 质量门限，且候选之间平移差不超过 0.08 m、旋转差不超过
0.035 rad，才提交一次校正。匹配期间 TF 和运动监测继续运行，重新运动会废弃待处理结果。
日志 `STATIONARY_CORRECTION committed` 表示实际应用了校正；单独出现 `GICP converge`
还不代表已应用。完成校正后保持地图对齐，下一次运动后停车才重新估计。

这属于有初值的局部地图匹配，并非每次停车都进行全地图搜索，也没有给 Point-LIO 增加回环优化。
RMUC2026 当前仍保留原局部修正限幅（变换 X/Y 各 0.60 m、Z 0.25 m、旋转约 10°）；
还检查候选的 `map→camera_init` 横滚/俯仰不超过 0.009 rad，留在 ROG 地面投影的 0.01 rad
水平姿态门限内，避免校正后禁用地面支撑。超限、点云不足或匹配不稳定时保留原定位，
不强制跳到候选位置。该后端不能保证恢复严重漂移或大幅倾斜的 LIO 坐标系。停车到校正通常需要数秒，
连续不停驶时不会执行校正。真值模式不运行这套 GICP 后端。


入口在需要时会构建全局重定位器，定位确认后继续使用当前修复后的 MINCO 仿真参数。

1. 等待 RViz 显示地图。此时 Gazebo 尚未生成机器人，导航未激活。
2. 点击 RViz 工具栏 **2D Pose Estimate**，在左半场空地按住鼠标并拖动，指定出生位置和朝向。
3. Gazebo 在该点生成机器人，随后自动静置、自转并全局重定位；`ALIGNING_HEADING` 阶段转回拖动箭头指定的朝向并停稳。
4. 日志出现 **READY** 后，点击 **Nav2 Goal** 选择目标；机器人通过正常导航到达。
5. 到达后可继续发送 **Nav2 Goal**，也可通过原有 Nav2 action 接口取消或更换目标。

默认不发送任何导航目标。`READY` 表示启动协调已完成，后续导航状态及到达结果查看原有 Nav2 / RViz 界面。再次使用 **2D Pose Estimate** 不会传送正在运行的机器人或覆盖定位；重新选择出生点需要重启本次仿真。

当前定位器使用 `negative_x` 左半场搜索先验，因此出生点要求 `x < 0`。
选点会检查地图范围、0.5 m 障碍间隙和有效地面高度；不合适的点会拒绝，可重新选择。
出生 Z 根据地面高程加车轮接触高度自动计算，不使用 RViz 的二维 Z 值。
这些检查依据栅格地图，不代表已对任意点击点完成实体碰撞验证。

## 可选参数

- `gui:=false use_rviz:=false` 用于通过 ROS 接口进行无界面测试。

当前集成入口支持 RMUC2026、MINCO，默认 `use_ground_truth_odom:=false`；
需要真值里程计对照时显式传入 `use_ground_truth_odom:=true`。

默认入口加载工作区和全局重定位构建。
机器人使用正常 xacro 中的物理驱动：在物理步之前向四个轮子施加水平力，用力偶产生转向；Z、roll、pitch 由重力和碰撞决定。
导航入口不再使用独立重定位试验的 `robot_velocity.urdf`。该模型中的三维 `VelocityControl` 会约束竖直速度，不能用于自由落体及地形导航验收。
轮子仍是简化的全向接触球，并非逐个麦克纳姆滚子的动力学模型；这不是实车驱动验证。
每次默认使用独立 Gazebo partition；ROS domain 默认 87，可通过环境变量 `ROS_DOMAIN_ID` 覆盖。

## 启动顺序与接口

- `spawn_from_rviz` 订阅 `/initialpose`，只用于一次 Gazebo 生成；通过 `/simulation/spawned` 发布完成标志，`/simulation/spawn_pose` 保留实际接受的选点。其朝向只用于定位完成后的转向目标，不输入定位搜索。
- 协调器等待地图服务启动后直接通过 `/map_server` 生命周期服务配置并激活地图，使 RViz 在机器人出生前就能显示地图，然后不限时等待选点。此流程不依赖定位 lifecycle manager 的启动服务，因此 Nav2 面板中的 Localization 状态不作为全局定位完成的判断依据。
- 生成机器人且传感器就绪后，调用 `/global_relocalization/run`；收到 `ARMED` 反馈后调用 `/global_relocalization/start_match` 完成独立停稳确认。
- 最终确认位姿通过 `/relocalization/initialpose` 一次性交接。Point-LIO 模式由 `relocalization_manager` 使用匹配时间的底盘 TF 计算 `map→odom`；真值模式由 `rm27_ground_truth_localizer` 使用匹配时间的相对里程计计算。两者均通过 `/relocalization/initialpose_accepted` 回送确认，只接受第一次位姿，同一消息的重试仅回送确认，不重置导航。
- 确认后进入 `ALIGNING_HEADING`，使用已确认的 `map→base_footprint` 和相对里程计恢复所选朝向。地图必须允许半径 0.45 m 的旋转包络，位置漂移不得超过 0.10 m；传感器、里程计、TF 失效时停车。转速上限 0.6 rad/s，角度误差小于 0.01 rad 且连续停稳 0.3 s 后交接。
- 交接时销毁协调器的 `/cmd_vel` 发布者和重定位速度订阅，再激活导航 lifecycle，进入 `READY`；看门狗不再发送命令。标准 `/navigate_to_pose` 接口保持原样，后续手动 Goal、取消和连续导航均由 MINCO 原路径处理。

`/relocalize_then_navigate/status` 是 transient-local JSON 启动状态，
`/spawn_from_rviz/status` 发布选点提示、拒绝原因和生成结果。
全局重定位与默认导航反馈均不订阅 Gazebo ground truth。

初始化期间，协调器转发 `/relocalization/cmd_vel`，0.5 秒没有新指令则停车。
交接后，速度完整沿用 `MINCO MPC → velocity_smoother → cmd_vel_nav2_result → fake_vel_transform → cmd_vel → Gazebo`。
协调器不再转发、重采样或发布停车指令；即使状态节点继续存在，也没有底盘发布者。
定位失败或交接失败不会激活导航。

也可先 `source tools/global_relocalization/environment.bash`，然后在原启动命令中增加
`global_relocalization:=true`；地图及 manifest 默认使用仿真安装目录中的 RMUC2026 文件，亦可通过
`relocalization_map` 和 `relocalization_manifest` 覆盖。此流程目前支持 RMUC2026 单机器人，不支持 `slam:=true`。

## 当前集成复测与已知限制

2026-10-28 停车校正集成的定向回归共 **24 项通过**：3 项静止判定、3 项真实 C++/DDS/GICP
集成、8 项启动与配置契约、8 项朝向交接和 2 项真值里程计交接。真实后端测试覆盖匹配时刻
插值、禁止最新 TF 替代、初值幂等、转动/滑行禁止提交、连续三次确认、再次停车重新校正，
以及成功匹配但姿态不满足导航约束时拒绝更新。

最终版本使用 Point-LIO 完整无界面仿真，出生点 `(-9.272, 0.819, yaw=0)`，每段成功后
观察静止 6 秒。全局初始化成功，第一、第二目标分别用时 26.799 s、33.649 s，均通过真实
目标距离检查；第三段 120.001 s 超时，第四段未执行。第三段按 TF 时间戳插值对齐真值后的
XY 误差最大约 **1.03 m**，末段约 **0.95 m**。取消后后端进行了匹配，但修正量超出局部
安全门限而未应用。**整条路线尚未通过；接入停车校正不能视为大幅漂移和过洞问题已经解决。**

第一段停车的候选横滚约 -0.0094 rad，超过后端的 0.009 rad 门限，原定位被保留；
第二段停车实际提交了校正。最终版本没有出现校正导致的 ROG `non-horizontal` 失效。
更早的诊断版本曾完成四段，但应用倾斜候选后触发地图保护；这正是加入姿态门限的原因，
该记录不能作为最终版本完整路线通过的证据。

日志、参数快照、导航/静止采样与验收摘要在
`tools/global_relocalization/.build/stationary_backend_20261028/`：
`guarded_launch.log`、`guarded_run/`、`acceptance.json` 为最终版本；
`launch.log`、`run/` 为补姿态门限之前的诊断版本。这些是观测日志/JSON，**没有录制完整 rosbag**。


Point-LIO 默认入口、漂移实测及尚未通过的隧道回归见[最新跟进报告](pointlio_tunnel_followup_20261028.md)。以下其余运行记录是此前配置下的历史验证，不能作为当前 Point-LIO 连续导航通过的证明。

重定位后朝向恢复、任务内巡航朝向保持及过洞复测，见
[2026-10-28 修复报告](navigation_relocalization_tunnel_20261028.md)。下列联通性和配置对照记录属于此前的初始集成验证。

2026-10-28 的无界面实测以 ROS 消息模拟 RViz 选点，完整走过选点、传感器准备、自动旋转匹配、位姿确认、导航激活和一个手动导航目标。该次定位结果与选定出生点的 XY 差约 0.0022 m，目标状态为 `SUCCEEDED`；确认 `/cmd_vel` 只由原 `fake_vel_transform` 发布，MPC 平滑权重为 `[6,6,0.5]`。这是单个选点的联通性结果，不能推广到所有出生点；本次没有操作图形窗口。

之前的实现依赖定位 lifecycle manager 的启动服务，曾因服务发现卡住而让 `STARTING_MAP` 超时；即使随后手动激活地图，已失败的协调器也不会在选点后启动定位。现已改为在地图服务启动后直接配置、激活地图，并推迟协调器启动以避免与导航节点同时初始化。修改后的两次独立仿真均完成地图发布、选点、重定位和导航；第二次使用用户选点附近 `(-8.978, -0.308)`，定位结果的 XY 差约 0.0045 m。ROS 服务发现本身仍可能受环境影响；若看到 `FAILED`，导航不会启动，应查看终端原因。成功和历史失败日志保存在 `tools/global_relocalization/.build/global_navigation_repair_20261028/`。

## 一次性交接与 MINCO 原配置验证（2026-10-28）

- 配置回归逐项比较原 YAML 的各节点参数；默认仿真除了定位后端的一次性初始化设置，其余均一致。
- 真实 ROS 测试验证：初值之前不发布 `map→odom`、使用初值对应时刻的里程计而非最新值、重复初值不能重置导航；普通仿真未启用该功能时行为保持原样。
- 实际运行确认规划插件为 `MincoPlanner`，控制插件为 `MincoMpc`。
- `READY` 后 `/cmd_vel` 唯一发布者为 `fake_vel_transform`，协调器已不订阅任何速度流。
- 连续两个手动 action 目标均 `SUCCEEDED`，停车真值 XY 误差分别为 0.1573 m、0.0818 m；到达后仍可继续正常导航。
- 两段导航的平滑输出与底盘输出消息数分别为 104/104、108/108，保留原配置的约 20 Hz 频率，没有协调器二次转发。

以上验证保存在 `.build/minco_handoff_validation_20261028/`。单元/真实 ROS 回归共 33 项通过。

## 物理回归验证（2026-10-28）

```bash
source tools/global_relocalization/mapping_environment.bash
python3 src/rm_27_stimulation/test/check_drive_physics.py \
  --output tools/global_relocalization/.build/drive_physics_validation_20261028
```

回归使用实际机器人的碰撞、惯量和驱动，在 Bullet 中自动验证：

- 从 1 m 高处落到 0.30 m 平台，车体 Z 稳定在 0.38 m。
- 驶离平台后自然落到平地，Z 回到 0.0800 m，而非保持平台高度。
- 左右自转指令 ±1 rad/s，实测稳态约 +0.9999 / -1.0000 rad/s。
- 停止发送速度后触发超时停车，末态水平速度小于 0.001 m/s。

驱动修复包含 `Update` 改为 `PreUpdate`、轮端水平力/转向力偶、显式关闭接触球的扭转摩擦，以及有界转向 PI。
力偶避免依赖当前 Bullet 后端对显式世界力矩的惯量坐标转换；参见
[Gazebo Physics 的实现](https://github.com/gazebosim/gz-physics/blob/gz-physics7_7.8.0/bullet-featherstone/src/LinkFeatures.cc)。

物理驱动下还运行了完整双向导航：RViz 初始位姿接口选择 `(-9.288, -0.376, yaw=0)`，
完成全局重定位后静置 4 秒，真值位置未发生移动，再通过标准 Nav2 action 发送目标。

| 手动目标 XY（m） | action 结果 | 停车 XY 误差（m） | 停车车体 Z（m） | 该处碰撞地面 Z（m） |
|---|---|---:|---:|---:|
| (-0.07, 4.93) | SUCCEEDED | 0.1571 | 0.3800 | 0.3000 |
| (-9.288, -0.376) | SUCCEEDED | 0.1459 | 0.0800 | 0.0000 |

第一个目标位于 0.30 m 高的平台，已独立核对可视网格与碰撞网格；不能仅凭车体 Z=0.38 m 判断悬空。
最初验收脚本错误地把所有目标当作零高程，导致去程 JSON 的固定 Z 断言失败；保留该原始记录，以上按实际地面高度复核。
回程实际驶下平台回到平地，两次目标均未使用真值驱动或定位。
本机完整日志、轨迹、地面复核和近景落地截图保存在
`tools/global_relocalization/.build/physical_navigation_validation_20261028/`。

## 早期链路验证（修复物理驱动之前）

三次均从随机出生开始，经全局重定位、时间戳交接、自动发目标，最终
NavigateToPose 返回 `SUCCEEDED`。目标均为 `(-11.7, 3.1)`。

| 模式 | 随机种子 | 出生 XY（m） | 协调器启动至到达（s） | 停车后真值 XY 误差（m） |
|---|---:|---|---:|---:|
| legacy | 2 | (-9.075, 1.115) | 34.50 | 0.1594 |
| minco | 2 | (-9.075, 1.115) | 22.25 | 0.0292 |
| minco | 1 | (-13.525, -0.735) | 25.74 | 0.0330 |

耗时不包含 Gazebo 初始化和生成机器人；真值仅由独立验收进程读取，不进入
在线定位/导航。这三次使用旧的速度约束模型，仅说明接口链路打通，**不能作为物理地形导航通过的依据**，也不是所有随机点的成功率验收。
早期 Point-LIO 实验为 MINCO 改用了 `/registered_scan`、`/lidar_odometry` 和 `odom`，与定位管理器的
输入一致，避免混用当前隔离 Point-LIO 不发布的 `/cloud_registered_full` 和
机器人 TF 中不存在的 `body`。

早期 27 项测试覆盖 RViz 选点/拒绝无效点、阶段顺序、失败停车、未确认不激活导航、
随机采样复现及两种导航模式的启动接线。其中真实 ROS 管理器测试验证初值前不发布 map→odom、拒绝错误 frame，并使用旧扫描对应的里程计而非最新位姿。原有 `test_navigation_launch.py`
结果为 41 通过、9 失败，失败涉及未改动的地图几何/参数一致性检查，尚未处理。

本机日志和 JSON 结果保存在 `tools/global_relocalization/.build/navigation_validation_20261028/`。
