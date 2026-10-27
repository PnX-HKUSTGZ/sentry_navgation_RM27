# 全局重定位：最终实现与使用说明

当前交付入口是 raw LiDAR + IMU 比赛模式。机器人在赛前静置标定、原地自转并完成全局搜索；开始事件到来后立即停车，用三个不同时间戳的新扫描确认定位，成功后释放速度控制。

本模块是独立定位器，输出地图坐标系下的底盘位姿。现有导航中的 relocalization_manager 仍负责 map -> odom，需要初值或 /initialpose；当前没有自动接入该管理器。仿真演示不会启动 Point-LIO、规划器或导航控制器。

## 1. 保留的实现

| 位置 | 用途 |
|---|---|
| src/global_relocalization/src/node.cpp | ROS action、开始事件、旋转与停车状态机 |
| src/global_relocalization/src/core.cpp | Scan Context 检索、分批 GICP、歧义检查和独立确认 |
| src/global_relocalization/src/stationary_raw_frontend.cpp | IMU 标定、旋转先验、局部 SE(3) 配准和 20 帧子图 |
| src/global_relocalization/include/ | 有界 KD 查询、输入队列、安全保护、可视化 |
| src/global_relocalization/config/competition.yaml | 唯一交付参数文件 |
| src/global_relocalization/launch/relocalization.launch.py | 定位节点及可选 RViz 窗口 |
| src/global_relocalization_interfaces/action/Relocalize.action | 定位 action 接口 |
| tools/global_relocalization/visualize.bash | 一键启动、恢复窗口和演示 |
| tools/global_relocalization/competition_trial.py | 单次仿真评分、参数审计、资源记录 |
| tools/global_relocalization/assets/RMUC2026/ | 冻结参考 PCD 和地图来源 manifest |

旧 LIO 实验入口、过渡 profile、消融脚本和分散文档已移除。底层一次性 Core::search() 与分阶段 prepare()/confirm() 共用同一套求解器，供组件测试和离线诊断使用；不维护第二套定位算法。节点保留已测试的输入兼容接口，交付配置统一选择 raw_imu 和比赛模式。

历史批测日志和旧二进制不再纳入当前版本，原文件保留在本机被忽略的 tools/global_relocalization/data/，旧提交也保留记录。参考地图独立保留，启动不依赖旧实验目录或 /tmp 构建。

## 2. 算法流程

1. **静置标定**：检查 IMU 时间连续性、静止角速度和重力，估计陀螺仪偏置及重力方向。
2. **局部运动估计**：按实际采样间隔积分 IMU，结合固定外参预测雷达运动，再用完整六自由度局部 GICP 对齐扫描。只把通过质量和运动门限的扫描放入子图。
3. **旋转建查询云**：自转满一圈后冻结累计点云；继续跟踪局部运动，查询云不再增长。
4. **全局搜索**：对左半场数据库计算 Scan Context，取 SC 前 10 与 ring key 前 5 的去重并集，进行粗、细 GICP。首批全部不合格时，再评估尚未测试的 SC 第 11–20 名。两个候选 worker 并行，保留质量排序和歧义检查。
5. **准备完成**：进入 ARMED，继续自转等待开始事件。准备结果有本轮来源和一次性消费约束，不能跨轮复用。
6. **开始与停车**：start_match 锁存事件并发布零速。停车前采集、停车后才处理的旧扫描不能重启停稳窗口。实际运动会重置停稳计时。
7. **独立确认**：停稳后取得三个不同时间戳的新扫描，逐帧检查 GICP 质量、与全局解的一致性及帧间一致性。冻结输入并排空 worker 后再次核验。
8. **输出**：返回完整底盘位姿。成功后停止持续发布速度；失败或取消保留停车保护。

~~~text
SETTLING -> SPINNING -> PREPARING -> ARMED
                                     |
                                 start_match
                                     v
                         STOPPING -> CONFIRMING -> SUCCEEDED
任一阶段的故障、超时或取消 -> 停车 -> FAILED / CANCELED
~~~

可恢复的局部数值不收敛最多使用一次后续扫描恢复；质量、几何、时间连续性或运动门限失败不会通过重试绕过。提前开始会停车并失败；无活动轮次、重复开始事件会明确拒绝。

## 3. 环境与构建

当前验证环境为 Ubuntu / WSL2、ROS 2 Jazzy、Gazebo Harmonic、Bullet Featherstone。WSL2 GUI 需要 WSLg 的 DISPLAY；关闭 Gazebo GUI 仍需要可用的 GPU 传感器渲染环境。

依赖包括 PCL、Eigen、OpenMP、OpenSSL、yaml-cpp、nlohmann-json、ROS action/TF/消息包，以及 Python 的 numpy、scipy、psutil、PyYAML、pytest。仿真与窗口还需要仓库中的 rm_27_stimulation、rm27_rendering、ros_gz 和 RViz。

在仓库根目录执行：

~~~bash
bash tools/global_relocalization/build.bash
source tools/global_relocalization/environment.bash
ros2 pkg prefix global_relocalization
~~~

最后一条应指向本仓库的 tools/global_relocalization/.build/ros/install/global_relocalization。环境脚本把隔离构建及其 core 库放在工作区安装之后，避免选择旧节点或混用动态库。

- ROS setup 默认为 /opt/ros/jazzy/setup.bash，可用 GLOBAL_RELOCALIZATION_ROS_SETUP 覆盖。
- 默认加载仓库 install/ 的依赖；外部依赖可设置 GLOBAL_RELOCALIZATION_DEPENDENCY_INSTALL。
- small_gicp 固定 revision 为 fa0cfc983417c044b6036f71dfa448038bb9c820。构建优先使用已有源码缓存，可设置 GLOBAL_RELOCALIZATION_SMALL_GICP_SOURCE；无缓存时由 CMake 获取固定版本。
- 构建记录源码与二进制 SHA-256。一键可视化会检查记录，源码变化或构建缺失时先重新构建，并清理安装目录中已删除的旧 profile、launch 和脚本。

## 4. 一键打开 Gazebo 和两个 RViz

~~~bash
bash tools/global_relocalization/visualize.bash
~~~

命令会启动或恢复仿真、定位节点、显示辅助节点和两个 RViz，等待 LiDAR/IMU 就绪，再演示一次定位。ARMED 后特意等待 5 秒，便于观察准备状态，再自动调用开始服务。再次执行会复用存活窗口、恢复缺失窗口，再开始新一轮，不会只发送一个等待数据的定位请求。

~~~bash
# 只打开或恢复窗口，不自动触发定位
bash tools/global_relocalization/visualize.bash --no-demo

# 关闭本入口所属的窗口、仿真和节点
bash tools/global_relocalization/visualize.bash --stop

# 指定渲染模式
bash tools/global_relocalization/visualize.bash --render-mode gpu
~~~

--map、--manifest、--params 可以覆盖默认路径。默认使用正式资源目录中的 RMUC2026 地图和 competition.yaml。更换输入时会停止本入口的旧会话，再重新启动。

会话状态、日志和每次演示结果写到 .build/visualization/。进程身份用 boot ID 和内核启动 ticks 核验，避免 WSL 墙钟校正或 PID 复用导致误判、重复启动或误清理。不会按进程名批量结束其他仿真。自动选择独立 ROS domain 和 Gazebo partition；ROS_DOMAIN_ID 可指定 domain。

### 画面含义

| 窗口 / 颜色 | 含义 |
|---|---|
| Gazebo | 实际仿真机器人自转与停车 |
| 地图视图：灰色 | 冻结先验地图 |
| 地图视图：青色 | 按最近候选变换后的累计查询云，尚不表示整次定位接受 |
| 地图视图：紫色 | 按确认结果变换后的独立停稳扫描 |
| 黄色 / 红色 / 绿色箭头 | 候选初值 / 未通过检查 / 通过该阶段检查 |
| CURRENT RUN ACCEPTED | 本轮最终接受的底盘位姿 |
| 参考视图：橙色 / 紫色 | 动作起始坐标系下的累计查询云 / 独立扫描 |

地图窗口使用 map；参考窗口使用 relocalization_reference。显示辅助节点只添加各自独立的叶子 TF anchor 并压缩诊断文字，不连接这两个坐标系，不发布 map -> odom、速度或导航目标。未定位前不存在参考系到地图的已知变换。

可视化默认关闭。开启时有界采样和诊断队列会增加开销；一键演示使用最多 12000 点、3 Hz。正式性能批测应关闭 GUI 和可视化。候选绿色箭头仍须通过歧义检查及三帧确认，不能单独当作定位成功。

## 5. 手动运行和接口

独立终端启动 headless 仿真，所有终端需要相同的 domain 和 partition：

~~~bash
source tools/global_relocalization/environment.bash
export ROS_DOMAIN_ID=174
export GZ_PARTITION=rm27_manual_relocalization
bash tools/global_relocalization/start_simulation.bash gui:=false
~~~

另一个终端加载相同环境后启动算法：

~~~bash
ros2 launch global_relocalization relocalization.launch.py \
  map_path:=$PWD/tools/global_relocalization/assets/RMUC2026/map.pcd \
  manifest_path:=$PWD/tools/global_relocalization/assets/RMUC2026/manifest.json
~~~

rviz:=true 会同时打开两个窗口，并默认启用诊断 publisher；use_sim_time 默认 true。

| 接口 | 类型 / 用途 |
|---|---|
| /global_relocalization/run | global_relocalization_interfaces/action/Relocalize，空 goal 开始准备 |
| /global_relocalization/start_match | std_srvs/srv/Trigger，ARMED 后触发停车确认 |
| /global_relocalization/pose | geometry_msgs/msg/PoseStamped，成功输出 |
| /global_relocalization/status | JSON 字符串：阶段、输入健康、候选、队列、确认 stamp、计时 |
| /global_relocalization/visual/* | 显式开启的点云、marker 和诊断事件 |

~~~bash
ros2 action send_goal /global_relocalization/run \
  global_relocalization_interfaces/action/Relocalize '{}' --feedback

# 等待 ARMED，再执行
ros2 service call /global_relocalization/start_match std_srvs/srv/Trigger '{}'
~~~

使用一键窗口后手动发命令，应先加载 tools/global_relocalization/.build/visualization/environment.bash，从而使用实际会话的 domain 和 partition。

## 6. 主要参数与接受条件

| 参数 | 当前值 | 含义 |
|---|---:|---|
| raw_scan_model | instantaneous | 仅支持理想瞬时扫描 |
| raw_calibration_sec / settle_sec | 2.0 s | 初始标定与静止要求 |
| spin_speed | 2.25 rad/s | 仿真指令值；实测约 3 s/圈 |
| raw_submap_scans | 20 | 只保留接受扫描的局部子图 |
| raw_local_max_fitness | 0.18 m | 局部最近邻 RMS 门限 |
| raw_local_min_inlier_ratio | 0.65 | 局部内点比例 |
| raw_range_filter | true，0.5–30 m | 过滤车体近点和远点 |
| raw_deterministic / raw_imu_rotation_prior | true | 确定性预处理与 IMU 旋转约束 |
| bounded_global_correspondences | true | KD 查询阶段精确距离剪枝 |
| candidate_workers | 2 | 候选并行任务数 |
| max_fitness / min_inlier_ratio | 0.2 m / 0.65 | 全局和独立确认的质量门限 |
| ambiguity_margin | 0.15 | 空间上不同的相近质量候选导致歧义拒绝 |
| search_side | negative_x | 固定左半场先验，底盘 x 必须小于 0 |
| stop_settle_sec | 0.2 s | 开始事件后的墙钟与采集时间停稳窗 |
| raw_confirmation_frames | 3 | 恰好三个不同 stamp 的新扫描 |
| prestart_timeout_sec / post_start_timeout_sec | 120 / 5 s | 赛前、开始事件后的独立预算 |

完整 SE(3) 配准还要求收敛、有限变换、足够内点和有效几何。局部运动检查不会把真实平移投影为零。确认解须与累计解及其他确认解分别满足默认 0.15 m、5° 一致性门限。

`confirmation_gravity_prior` 在通用 Core 中默认关闭；比赛 profile 显式打开，仅作用于三个独立停稳扫描的确认配准。它要求 `raw_imu` 初始静止重力建立的参考系，且 manifest 的 `T_map_world` 将世界 Z 轴映射到显式 `confirmation_map_up`（当前 `[0,0,1]`）。节点启动会核验这两个前提，缺少 manifest 或方向不符会拒绝启动。更换任意地图或非重力对齐输入时应关闭此项，或提供经过验证的地图重力方向。

该项使用方向残差 `R * [0,0,1] - map_up`，标准差 `confirmation_gravity_sigma=0.005 rad`，与局部 IMU 姿态不确定度的保守下限相同。它是两个倾斜方向的软约束，不约束 XYZ 或绕重力轴的 yaw；准备配准和全部质量、一致性门限保持原值。通过确认后仍返回完整一圈的参考位姿，再合成最新原始前端运动，因此当前底盘真实 roll/pitch 和平移会保留。状态诊断记录重力来源、方向、标准差及逆方差。


仿真验收的 **3D ≤0.10 m、SO(3) ≤2°** 是独立评估器的 GT 评分标准，不进入定位器。局部 fitness 门限曾从 0.12 放宽到 0.18，当前结果包含这一调整的收益。

## 7. 评分、测试与诊断

完整检查：

~~~bash
bash tools/global_relocalization/check.bash
~~~

检查包含 C++ 求解器、bounded KD、局部前端、队列、停稳保护、准备状态、观察器和 ROS action/开始服务合约；另检查完整位姿评分、GT 时间插值、Gazebo reset 和进程身份。组件检查不等于真实定位成功率。

单次 headless 正确性试验需先运行独立仿真，不要同时运行同 domain 的其他定位器：

~~~bash
source tools/global_relocalization/environment.bash
ros2_prefix="$(ros2 pkg prefix global_relocalization)"
python3 tools/global_relocalization/competition_trial.py \
  --node "$ros2_prefix/lib/global_relocalization/global_relocalization_node" \
  --params src/global_relocalization/config/competition.yaml \
  --map tools/global_relocalization/assets/RMUC2026/map.pcd \
  --manifest tools/global_relocalization/assets/RMUC2026/manifest.json \
  --trial p13_yaw3 --output /tmp/rm27_relocalization_new_trial
~~~

输出目录必须不存在。客户端核对地图 hash、运行时参数和实际 core 库，只在客户端读取 GT，按输出 pose stamp 插值位置和四元数后评分。保存估计矩阵、状态、控制命令、确认 stamp、实测转速和 CPU/RSS。退出码 0 要求严格正确且内部开始到 pose ≤1 s；--early 单独验证提前开始被拒绝并安全停车。

trigger_to_pose_sec 从服务器锁存开始事件计时到成功完成，**不包含赛前准备，也不是客户端收到 pose 的完整通信时延**。prestart_prepare_sec 只统计赛前全局搜索阶段，整轮 action 还包含初始标定、自转、等待事件和确认。

离线诊断保留两个工具：

~~~bash
ros2 run global_relocalization global_relocalization_offline_query \
  MAP.pcd CAPTURE_DIR PROFILE.yaml RESULT.json

ros2 run global_relocalization global_relocalization_raw_replay \
  RAW_DIAGNOSTIC_DIR 20
~~~

diagnostic_output_dir 生成全局 query、三个 independent PCD 和 retrieval metadata；raw_diagnostic_directory 生成逐帧 PCD、实际参数、外参、IMU 和迭代 trace。raw replay 按捕获参数和实际停车恢复授权重放，不提供旧 baseline/消融覆盖入口。重复一致只证明数值重放，不证明实时性能或全局定位成功。

## 8. 已有结果及边界

以下是不同阶段的独立批次，不能合并成最终版本的一次全通过验收：

| 批次 | 严格正确 | 内部开始到 pose ≤1 s |
|---|---:|---:|
| 扩大子图后的三个困难点 | 3/3 | 3/3 |
| 固定 120 项首轮，局部门限 0.12 | 98/120 | 98/120 |
| 上述 22 项改为 0.18 后复测 | 22/22 | 22/22 |
| 新随机 30 项 | 29/30 | 24/30 |
| 停车修复后的五点针对复测 | 5/5 | 4/5 |
| 最终第二批检索修复，r01 单点 | 1/1 | 1/1 |

120 项首轮含 21 项前端拒绝和 1 项参数审计失败。98 个成功样本的内部确认耗时均值 0.635 s、P95 0.793 s、最大 0.897 s，最大三维位置误差约 7.37 mm。停车修复后仍出现一次约 1.046 s；r27 在新仿真会话复测约 0.641 s，第二批检索修复后的 r01 约 0.498 s。交互 GUI 演示也成功，但不计入 headless 验收。

旧普通流程冻结 45 项全正确，平均 action 27.95 s，15 s 内正确 0/45。它只描述历史基线，不能代表当前比赛模式的统一稳定性。

参考地图由同一仿真场景的 GT 融合建立。在线定位器不订阅 GT，但地图和瞬时扫描限制了毫米级精度结论的外推。真实 MID360 逐点时间、旋转去畸变、实车转速与误差、Point-LIO 建图误差、导航接入及最终版本的统一完整批测仍需要验证。
