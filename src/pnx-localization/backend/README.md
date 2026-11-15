# rm27_backend

Point-LIO 原子帧适配、关键帧与点云、里程计/地图/回环因子、GTSAM iSAM2、定位状态机和可信导航输出。
2026-11-12 起，先验地图模式链接 `relocalization_manager` 的计算库，在同一个进程内使用后台线程完成几何任务。
旧版数值与资源基线保留在 `VALIDATION.md`，不能作为新恢复实现的验收结果。

## 数据与职责

- 输入 `/point_lio/backend_frame`：同一扫描的世界系完整点云、IMU 位姿、instance/epoch/sequence 和质量字段。
- `body` 是 IMU 原点。适配后将点云变换到该帧 IMU 局部系；不消费 `/registered_scan`。
- `KeyframeManager` 保存原始里程计和优化位姿，分段不跨前端重置连接里程计边。
- `CloudStore` 有界缓存与可选诊断归档仍保留；全局重定位地图来自离线 PCD 预处理，不能把诊断归档当地图包。
- 公共类型、Scan Context、几何信息构造与配准实现在 `relocalization_manager`；旧 `rm27_backend/*.hpp` 路径通过兼容头继续可用。
- `LocalizationController` 是生产地图模式的统一提交入口；旧 `MapLocalizer` 保留用于历史回归，生产节点不再实例化它。

## 三种模式

| constraint.mode | 约束 | 图与输出 |
|---|---|---|
| prior_map | 里程计、先验地图配准、独立恢复锚 | 固定窗口；锚定后 map 系；可接管 TF |
| loop | 里程计、SC 历史回环 | 历史图有节点上限；局部段坐标；不发布导航 TF |
| odometry | 里程计链 | 固定窗口；局部段坐标；基线调试 |

先验地图与运行时回环保持互斥。回环保留既有 GICP 和 PCM/NIS；新地图跟踪、恢复使用鲁棒点到面优化。
`prior_map.gravity_aligned=true` 在优化中保持初值 roll/pitch，适配水平地图与 Point-LIO 的重力对齐世界系。
不能在配准结束后直接抹掉旋转，也不能把地图倾角错误靠放宽导航地形门限隐藏。

## 状态与恢复

五状态为 INITIALIZING / NORMAL / OBS_DEGRADED / SUSPECT / LOST。普通观测只有成功入图后才能恢复可信跟踪；
可疑或丢失必须经过独立恢复验收。没有候选、后台忙、过期结果不累计观测冲突。

恢复积累同代连续源帧；可疑阶段围绕最后可信位置检索，初始化和丢失允许全库检索。SC 按地点分散召回候选，
逐航向验证；结果在收敛位置重新选子图评分，残差差异不足以区分不同地点时拒绝唯一定位。
恢复锚要求收敛、几何支撑、重叠率和满秩信息；不跨越不可信区间补里程计边。
后台请求绑定前端身份、源时刻、段号及状态 generation，提交时复核并传播到最新源帧。

地图加载错误保持未定位。前端断流由独立墙钟定时器发现；协调重置服务的响应仅代表受理，新的 epoch 原子帧才确认完成。
每个前端 epoch 最多自动请求一次，避免重置循环。全局匹配失败本身不自动重置健康前端。

## 地图输入

```bash
python3 tools/mapping/prepare_relocalization_map.py \
  --pcd /path/map.pcd --grid /path/map.yaml --output /path/map.pcd.rm27
```

工具支持 ASCII/binary PCD，binary_compressed 需先转换。输出 map.pcd、虚拟参考位姿/局部云、manifest 和坐标叠加报告。
参考云以角度最近返回近似可见性；SC 根据运行参数在加载时构造并只读保存于内存。
源 PCD、导航 YAML/图像和生成云都有哈希；map←PCD 变换只应用一次。
参考视点覆盖 P95 超过匹配对应距离时加载拒绝，避免稀疏视点遗漏窄通道。
坐标报告不能单凭边界重叠证明物理场地严格对齐；部署前应核对控制点及传感器安装、视场和参考高度。

地图路径由 `recovery.map_bundle` 指定，空值使用 `prior_map.file + ".rm27"`。
`recovery.navigation_grid` 在导航 launch 中由 map 参数传入，核对导航栅格来源；不使用旧定位 TF 作为启动种子。

## ROS 接口

- `~/odometry`：可信阶段的当前 IMU 地图位姿、模型协方差和前端速度。
- `~/keyframes`：当前图关键帧路径。
- `~/map_alignment`：map←camera_init，不等同于导航 map←odom。
- `~/localization_status`：结构化状态、原因、身份、有效源时间、全局可信和 TF 标志。
- `~/diagnostics`：输入、图、匹配、缓存和耗时统计。
- `/prior_pcd_map`：transient-local 背景地图，供现有车辆障碍识别模块使用。
- `/initialpose`：map 系 `base_footprint` 提示，以原始时刻 TF 转到 IMU；配准验证前不会改变可信 TF。
- `~/relocalize`（std_srvs/Trigger）：无位置提示的全局恢复请求。
- `~/reset_frontend`（std_srvs/Trigger）：请求重置当前 Point-LIO 身份。
- `/point_lio/reset`（rm27_localization_interfaces/ResetFrontend）：前端受理合同；新 epoch 确认完成。

先验地图模式 `output.publish_tf=true` 时，使用同刻 world←odom 和 IMU 位姿生成 map→odom。
可疑/丢失冻结最后可信底盘位姿；状态明确不可信。TF 查询稍滞后时由维护定时器重试，不改用最新 TF 冒充源时刻。
导航最终速度出口检查定位状态和时间；恢复后要求新规划路径，不重放旧速度。

普通 Point-LIO 导航 launch 强制 prior_map 与后端 TF；不再启动旧管理器节点。独立 backend.launch.py 按 backend.yaml 运行。
GT 和 SLAM 模式保留原 TF 所有者，定位速度门由 launch 关闭。
参数位置：`launch_bringup/config/{reality,simulation}/{nav2_params,minco_params}.yaml`，每项带中文注释。

## 构建与测试

```bash
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build --packages-up-to rm27_backend point_lio fake_vel_transform --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
colcon test --packages-select rm27_backend point_lio
python3 -m pytest -q tools/mapping/test_prepare_relocalization_map.py
```

几何基准 `tools/stimulation/benchmark_rm27_global_recovery.py`；完整隔离仿真 `validate_rm27_recovery.py`。
完整开发与本轮实验报告在仓库 [docs/backend_recovery_2026-11-12.md](../../../docs/backend_recovery_2026-11-12.md)。

## 固定出生位姿的自动启动

`startup_pose.enabled=true` 时，`startup_pose.map_from_base` 表示地图系中的底盘参考点（`output.base_frame`）位姿，
以 `[x,y,z,roll,pitch,yaw]` 米/弧度填写。后端查询同刻前端世界←底盘，转为 IMU 几何初值，匹配验证后才发布全局 TF。
不需要测试脚本或 RViz 发送 `/initialpose`。仅用于首次 epoch，后续恢复不自动复用出生点。
此参数与 `prior_map.seed_source=parameter` 互斥；后者是 map←camera_init，坐标含义不同。

RMUC2027 / RMUL2027 固定出生场景在 simulation_world_profiles 中分别启用
`[-11.7,3.1,0.08,0,0,0]` / `[-4.37,3.03,0.093,0,0,0]`；
它是预先配置的位置先验，不读取仿真真值。更改出生点或启用随机出生时必须更新或关闭。
实车默认关闭，填写实际比赛出生区后才启用。先验不能保证拒绝错误但几何相同的人工提示，仍须保证配置来源正确。

## 人工初值与恢复交互（2026-11-13）

`/initialpose` 仍为带原始时间戳的底盘位置提示。启用
`initialpose.preview_tf` 后，后端先撤销运动许可并取消当前
`NavigateToPose` 目标，再临时发布待验证的 `map→odom`。这只改变 RViz 定位显示，
不移动 Gazebo 或实车，也不向图中写入未经验证的先验。

默认 RViz 的 **Backend Recovery** 显示订阅
`/rm27_backend/recovery_markers`：黄色表示等待/验证，红色表示失败；
显示阶段及墙钟耗时。恢复成功后主动删除箭头与文字，启动时也发送删除消息清理旧显示。提示附近成功后使用几何结果更新 TF；人工请求默认 5 秒截止，
失败退回最后可信底盘显示并保持未定位，等待新的请求。恢复后需要重新发送导航目标。

主要参数为 `initialpose.{preview_tf,timeout,tf_wait_timeout,max_translation,max_rotation_deg}`、
`recovery.{hint_global_fallback,hint_yaw_samples,hint_yaw_step_deg,ambiguity_retry_period}`。
实车与仿真、MINCO 与 legacy 的完整参数文件均已显式配置并附中文注释；
独立包的 `config/backend.yaml` 只提供默认配置，普通导航仍使用 launch 解析的完整配置。

恢复源复用同代近期扫描，源几何每轮只预处理一次；优先验证 SC 航向，重复收敛位置
不重复完整地图复验。两个不同地点均达到残差噪声下限时，提前报告歧义；不会凭首个
好结果宣布全局唯一。静止歧义降低重试频率，新视角或新点击可以提前触发。
新请求通过原子取消版本打断旧点到面迭代，旧代次与旧取消版本的结果均不能提交。
诊断增加 `map_registration_attempts`、`map_cancelled_jobs`。

测试工具：`tools/stimulation/validate_rm27_manual_recovery.py` 与
`tools/stimulation/summarize_rm27_manual_recovery.py`。详细记录见
`docs/manual_recovery_optimization_2026-11-13.md`。


### 行驶连续性恢复（2026-11-13 补齐）

SUSPECT 与 LOST 下，若前端仍健康且同一 instance/epoch，恢复中心使用
`T_map_imu_last * inverse(T_world_imu_last) * T_world_imu_now`。
位置和姿态包络随锚点年龄、累计前端路程和转角增大，既限制候选检索与初值，
也限制最终配准结果。默认从 1.5 m、15° 起步，最长 15 s，超过 8 m 或 60° 则解除。
先验证运动预测附近的满秩点到面结果，成功后仍按正常图锚流程提交，不能直接拿前端预测作全局测量。

低质量、前端重置、图异常、断流、时间回退或不合理运动会解除约束；显式人工初值也解除，
使用户能纠正错误定位或处理被搬动的机器人。无提示的恢复服务可继续利用有效连续性。
无可靠运动范围时保留全局搜索和对称解拒绝，不会永远限定在旧半场。
参数见完整配置中的 `recovery.motion.*`，增长量是工程误差包络，不是统计标定的协方差。
诊断含 `motion_recovery_active`、`motion_recovery_radius_m`、
`motion_recovery_angle_deg` 与 `motion_recovery_reason`。验证记录见
`docs/motion_recovery_completion_2026-11-13.md`。
