# 建图异常与重定位测试交接记录

更新日期：2026-10-26，时区 UTC+8。

用户已明确要求停止测试，整理日志和交接文档，后续工作由其他人处理。本轮工作到文档整理为止，不自动恢复在线批测、离线对照或建图任务。

## 1. 当前状态

- 已运行仓库完整建图启动链：Point-LIO、loam_interface、地形分析、障碍扫描转换、SLAM Toolbox、地图保存。
- 正式采集目录为 `tools/global_relocalization/data/full_slam_20261026/run08/`。此前 `run01`—`run07` 为初始化、传输或控制排查及失败采集，全部保留，不混入正式结果。
- `run08` 连续完成 199/199 个路径点，路线约 32.15 m；保存了三维先验、二维栅格、原生位姿图和原始录包。
- **采集完成不代表地图合格。当前三维先验存在已证实的 Point-LIO 发散，不应作为合格先验使用。** 质量拒绝标记另存在 `run08/MAP_QUALITY_REJECTED.json`，未改写冻结地图或 manifest。
- 在线测试原计划 120 组，停止时检查点为 86 组。离线双地图对照保存了 38 组记录，其中 36 组有效、2 组运行超时。
- 整理时环境已重启，原仿真、在线测试、离线对照进程均已不存在。本次没有重启它们。旧日志中的 PID 已失效，不可用于后续进程操作。
- `batch120/p22_yaw2/` 是未完成目录，没有 `result.json` 和完整 bag 元数据；不进入已完成统计。剩余 34 组未完成，不计为算法失败。

## 2. 可以确定的结论

**当前失败主要受建图地图影响；不能将全部失败都归因于地图，也不能将基础设施异常视为重定位算法失败。**

建图前段已存在约 0.16 m 的位置偏差，超过评估要求的 0.10 m。随后原始 Point-LIO 的三维位姿快速发散，最终高度误差约为 -13 m。SLAM Toolbox 仅修正 XY 和航向，无法纠正 Z、横滚和俯仰。

同云对照进一步确认了地图输入的影响：查询云、可用的独立确认云、参数及核心库完全一致，仅替换地图。36 组有效对照中，理想地图有 35 组准备通过且参考位姿达到严格误差要求，车辆建图地图只有 15 组准备通过。另有 20 组在车辆建图地图上准备失败，换理想地图后准备通过。

两种地图均通过准备的相同 15 组，其查询参考位姿误差为：

| 地图 | 位置均值 m | 位置 P95 m | 完整姿态均值 ° | 完整姿态 P95 ° |
|---|---:|---:|---:|---:|
| 车辆建图地图 | 0.147403 | 0.199400 | 0.706703 | 0.918359 |
| 理想地图 | 0.003541 | 0.007431 | 0.053696 | 0.076875 |

按测试编号最早的两组完整确认对照，合成到原输出时刻的底盘位置误差为：

| 样本 | 建图地图 m | 理想地图 m |
|---|---:|---:|
| `p00_yaw0` | 0.114400 | 0.000879 |
| `p00_yaw1` | 0.205146 | 0.000493 |

已接受在线样本的离线复现与在线位姿一致，13 组检查的最大平移差小于 `9e-16 m`。对照估计器没有读取真值。

需要保留的判断边界：

- 原在线流程在准备阶段拒绝的样本，没有独立停车确认云。换图后准备通过不能记成完整重定位成功。
- 理想地图上仍有 `p06_yaw3` 未通过准备，原因为候选未通过 GICP 质量和搜索侧门限。因此重定位/查询链路也不是完全无失败。
- 换图对照能确认地图输入的影响，尚未分离地图形变、覆盖不足和采样密度差异的贡献。
- 低配准残差、独立确认通过，不保证输出位姿在真值坐标下满足 0.10 m 精度。已有 30 组在线输出属于这一情况。

## 3. 建图异常的关键证据

以下时间都是消息头的仿真时间，不是系统时钟。

- 第一个位姿图节点时间：`4017.099946670 s`。
- 原始 Point-LIO 最大相邻位姿跳变：`4146.070097778 → 4146.119360000 s`，约为采集起点后 129 s。
- 跳变量：`1.023611 m / 0.049262 s`。随后高度迅速进入错误范围，并进一步偏至约 -13 m；不是单帧直接跳变 13 m。
- 跳变已存在于 `/aft_mapped_to_init`，发生在 SLAM Toolbox 平面修正之前。
- 核查窗口：`4144.070097778—4148.119360000 s`。

| 原始数据 | 窗口内消息数 | 最大头时间间隔 | 时间倒退 |
|---|---:|---:|---:|
| LiDAR | 41 | 0.100 s | 0 |
| IMU | 404 | 0.010 s | 0 |
| 真值 | 202 | 0.020 s | 0 |

窗口内真值高度变化范围约 `4.09e-11 m`，不存在对应的真实下坠。IMU 加速度分量变化仅为约 `1e-6 m/s²`，最大角速度约 `2.01e-5 rad/s`。这些是原始观测，不能单独证明数值发散的唯一原因。

按最大跳变时刻分段，仅用于分析，**没有从地图中删除异常帧**：

| 建图轨迹分段 | 样本数 | 三维位置均值 m | 三维位置 P95 m | 姿态均值 ° | 绝对 Z 误差均值 m |
|---|---:|---:|---:|---:|---:|
| 跳变前 | 2383 | 0.163296 | 0.194798 | 0.841657 | 0.029353 |
| 跳变后 | 885 | 13.057508 | 14.135044 | 23.185291 | 12.322077 |

SLAM Toolbox 最终图包含 267 个节点、395 条边。扫描匹配、回环检测和 `map→odom` 发布均已启用并通过运行参数审计。但最终图与同时间戳在线节点对照，历史节点回写变化数为 0；不能宣称本轮成功通过回环纠正了历史轨迹。14 条节点 ID 间隔超过 10 的边也不能单独证明回环成立。

返回起点时，最终图内首尾位置仍相距约 `5.635 m`，配置的 `loop_search_maximum_distance` 为 `3.0 m`。漂移可能限制回环候选搜索，但尚未取得逐次回环拒绝原因，不能将其认定为唯一解释。

## 4. 实际建图方式与已处理问题

启动入口为仓库的：

```text
rm_navigation_simulation_launch.py
slam:=true use_ground_truth_odom:=false navigation_mode:=legacy use_rviz:=false
```

使用原仓库仿真参数叠加 `config/full_slam_mapping.yaml`。本轮 `run08` 的 Point-LIO 保留仓库原始仿真参数，`use_imu_as_input=false`，没有沿用早期实验的 IMU 输入模式覆盖。

已处理且有审计记录的问题：

1. 原仿真参数关闭了 SLAM Toolbox 的扫描匹配、回环和变换发布。覆盖配置启用 `use_scan_matching=true`、`do_loop_closing=true`、`transform_publish_period=0.02`。
2. SLAM Toolbox 未显式使用仿真时钟。覆盖配置增加 `use_sim_time=true`，运行时核对。
3. 重力对齐后的 Point-LIO 初始姿态已经包含安装角，原接口又叠加安装角。新增默认关闭的 `loam_interface.anchor_initial_base_frame`，本次启用，按 `T_base_lidar × inverse(T_camera_lidar_initial)` 锚定初始底盘。
4. 完整导航启动链的停车保护与采集器同时发布 `/cmd_vel`。本次将导航输出转为 `mapping_nav2_cmd_vel`，核对采集器为 `/cmd_vel` 唯一发布者。
5. 大点云在默认 DDS 共享内存缓冲下明显丢帧。增加仅用于传输的 64 MiB 配置 `fastdds_large_cloud.xml`，不改变消息内容和话题 QoS。正式采集已恢复接近 10 Hz 的注册点云。
6. 所有新录包使用 `--use-sim-time`，避免 WSL 系统时钟校正造成录包接收时间倒退。早期旧包另有时间索引规范化工具，不覆盖原包。

为避免漂移同时导致采集路线无法完成，正式采集显式使用 `--control-source simulator`：独立测试控制器读真值驱动车辆连续完成路线，**真值没有输入 Point-LIO/SLAM Toolbox，也没有逐帧参与点云融合**。采集结束后仅按一个共同时间戳做初始坐标对齐；没有参考地图 ICP、整段轨迹拟合或逐帧真值校正。

三维 PCD 按最终原生位姿图重建：

```text
C(t) = T_graph_corrected(t) × inverse(T_graph_odometry(t))
P_world = T_world_slam_map × C(t) × T_odom_camera_init × P_camera_init
```

相邻图节点之间插值，不在图时间范围外外推，超过 3 s 的节点间隙不插值。最终融合 1635 帧、305956 点。原始 Z/roll/pitch 仍来自 Point-LIO。

冻结地图 SHA256：

```text
6fcb156378155cd494d6baa8ab079788ccfa2aa5ab47ee01e06f75d21007ea15
```

## 5. 停止时的在线测试结果

这是部分批次记录，不是 120 组完整验收。

| 类别 | 数量 |
|---|---:|
| 严格正确 | 1 |
| 输出位姿但真值误差超标 | 30 |
| 全局准备/候选质量拒绝 | 36 |
| 独立停车确认拒绝 | 6 |
| 传感器/里程计/TF 数据过期 | 2 |
| 参数审计超时、服务不可用等基础设施异常 | 11 |
| 已落盘检查点合计 | 86 |
| 未完成，不计为算法失败 | 34 |

严格正确要求三维位置误差 ≤0.10 m 且完整姿态 SO(3) 误差 ≤2°。31 个有有效同步真值的输出，包含误定位，统计如下；拒绝、异常和未完成项不填零误差。

| 指标 | 均值 | RMSE | 中位数 | P95 | 最大值 |
|---|---:|---:|---:|---:|---:|
| 三维位置误差 m | 0.148562 | 0.151957 | 0.142985 | 0.193957 | 0.205146 |
| 完整姿态误差 ° | 0.673317 | 0.682901 | 0.648745 | 0.872905 | 0.930868 |

原始汇总的 `1/86` 包含基础设施异常作为完成尝试的分母，不能作为剔除基础设施影响后的纯算法成功率。73 组有算法接受/拒绝结果，另有上述 13 组数据过期或基础设施异常，三者应保持分开。

停止后的冻结输入核对通过，匹配 680 项实际执行 hash，没有发现冻结输入被改写。11 组缺少有效运行时参数审计的异常样本已单独列出，不能支撑算法性能结论。

## 6. 日志与产物入口

所有路径相对于仓库根目录。数据目录被 Git 忽略，**不会随代码提交自动交接，迁移机器时需另行复制**。原始包、失败采集和未完成目录均未删除。

| 路径 | 内容 |
|---|---|
| `tools/global_relocalization/data/full_slam_20261026/capture_index.json` | 正式采集与早期失败采集索引 |
| `.../run08/mapper.log`、`mapping_params.yaml` | 完整启动链日志及实际输入配置 |
| `.../run08/full_mapping_audit.json`、`mapping_runtime_parameters.json` | SLAM 配置、订阅和 Point-LIO 可执行文件审计 |
| `.../run08/mapping_binary_provenance.json` | 实际二进制、loam 库路径、SHA256 和 DDS 环境 |
| `.../run08/raw_bag/` | 已封包原始录包，MCAP 约 8.77 GB |
| `.../run08/registered_scans/`、`capture.json` | 逐帧三维注册点云、原始估计/真值轨迹、控制命令和在线图位姿 |
| `.../run08/slam.posegraph`、`slam.data`、`graph_optimized.json` | 原生最终图、数据集、原始扫描时间、图位姿和图边 |
| `.../run08/occupancy.yaml`、`occupancy.pgm` | SLAM Toolbox 原生局部坐标二维地图 |
| `.../run08/map.pcd`、`manifest.json` | 单次起点对齐后、应用图修正的冻结三维地图 |
| `.../run08/mapping_failure_analysis.json` | 最大跳变前后矩阵、原始传感器时间核查和真值高度范围 |
| `.../run08/mapping_error_strata.json` | 跳变前后分段统计，仅分析，不筛掉地图帧 |
| `.../run08/graph_correction_audit.json` | 最终图/在线节点对照、原始 LIO 误差 |
| `.../run08/prior_geometry_audit.json` | 冻结地图与理想地图的只读距离比较，没有做地图配准拟合 |
| `.../run08/MAP_QUALITY_REJECTED.json` | 当前地图不能作为合格先验的标记 |
| `.../batch120/termination.json` | 用户停止指令、86 项分类、未完成数和停止状态 |
| `.../batch120/summary.json`、`summary_at_stop.json`、`results.csv` | 停止时汇总及逐组真值、估计和误差 |
| `.../batch120/<trial>/result.json` | 完整矩阵、四元数、RPY、同步评分、状态、资源和控制记录 |
| `.../batch120/<trial>/node.log`、`runtime_parameters.json` | 逐组节点日志与参数审计；异常项可能缺审计文件 |
| `.../batch120/<trial>/raw_bag/`、`raw_diagnostics/`、`query_diagnostics/` | 逐组原始包、局部配准 trace、查询及独立确认云 |
| `.../batch120/integrity_audit.json`、`failure_analysis.json` | 冻结输入核对、未通过阶段分类 |
| `.../batch120/report.html`、`evaluation.*`、`mapping_drift.*` | 离线报告、误差分布、XYZ 分量和地图 XZ 投影 |
| `.../paired_maps/termination.json`、`summary.json`、`p*_yaw*.json` | 已停止的同云双地图对照、逐组结果和超时记录 |
| `.../tests.log`、`ctest.log` | 停止前已有的 21 项 Python、8 项 CTest 通过记录；没有在用户停止后重跑测试 |

表中的 `.../` 指 `tools/global_relocalization/data/full_slam_20261026/`。

`mapper.log` 在地图/原生图保存完成后的 SIGINT 退出阶段包含 SLAM Toolbox 关闭异常。原生图已成功重新读取；不要把退出阶段异常当作约 129 s 处 Point-LIO 发散的发生原因。

早期 `data/sensor_mapping_20261026/pointlio_prior_replay` 及 `pointlio_batch120` 是未使用 SLAM Toolbox 图修正的旧实验，已被本轮替代，不能混合统计。`full_uc_mapping.py`、`mapping_session.py` 及 `data/full_uc_20261026/` 属于另一套采集工作，不是本交接记录的正式批次。

## 7. 交给后续处理者的排查项

以下尚未完成，不代表已经确定的修复方案；本轮没有继续执行。

1. **先复现 Point-LIO 的原始三维发散。** 使用 `run08/raw_bag`、其实际 `mapping_params.yaml` 和同一二进制。需要从首次 IMU 初始化开始播放到异常，单独截取 4 s 窗口会丢掉历史状态，不能证明问题是否可复现。不要播放已记录的 odometry、注册点云或 TF 来替代估计输入。
2. **核对仿真点云逐点时间。** Gazebo GPU LiDAR 是瞬时扫描，原转换器却按 `(point_id % horizon_scan) × 0.1 / horizon_scan` 分配合成 0—100 ms 时间。本场景水平采样为 360，转换器 `horizon_scan=1875`。需要核实 Point-LIO 对这些时间的预处理/去畸变是否符合实际。不能直接把所有时间设零后就认为已关闭去畸变，Point-LIO 对末点时间非正存在时间推断回退。
3. **核对驱动与 IMU 的物理一致性。** 使用 Gazebo VelocityControl，直接设定模型速度。异常窗口实际 IMU 加速度接近恒定重力、角速度接近零；需与真值速度导数、车辆实际转动和安装变换核对。采集命令角速度不等于实际角速度，不能仅凭控制指令认定完成了旋转。
4. **检查 Point-LIO 的数值和几何约束。** 重点看异常前后平面对应、可观性、滤波创新及更新量、协方差和姿态更新；保存足以解释约 49 ms 内 1.02 m 更新的内部诊断。原始头时间连续只排除了该窗口录包头时间缺口，不能排除时间模型、物理模型或数值问题。
5. **在三维里程计可靠后核查回环。** 当前只证实开关启用，未观测到历史节点回写；需要取得实际回环候选和拒绝原因。二维回环不能代替修复三维高度和倾斜估计。

不要用每帧真值修正、理想地图 ICP 或删除异常帧来掩盖建图问题；若后续修改建图流程，使用新的输出目录及新的来源记录，不改写本轮冻结输入。用户已停止本轮重定位矩阵，不能自动接着跑剩余样本。

## 8. 代码与工作区注意事项

- 完整建图采集/导出：`tools/global_relocalization/full_slam_mapping.py`。
- 覆盖参数及传输配置：`config/full_slam_mapping.yaml`、`config/fastdds_large_cloud.xml`。
- 隔离构建：`build_mapping.bash`、`build_full_slam.bash`；原生图读取：`slam_graph_export.cpp`、`slam_export/`。
- 原接口的可选起点锚定修改位于 `src/loam_interface/`，默认关闭，本轮通过隔离 overlay 启用。
- 异常核查：`audit_mapping_failure.py`、`audit_slam_graph.py`；同云对照：`offline_map_compare.cpp`、`paired_map_comparison.py`。
- 流程说明见 [sensor_map_relocalization.md](sensor_map_relocalization.md)，包含构建和运行方法；这里不表示应当立即恢复测试。
- 仓库 HEAD：`baf92a6a449a6c234e7d75d8d5e6efff512a49ff`。Point-LIO 来源：`e85e79558cf746f6699888a54285fe48b3b0ac71`。
- Point-LIO 子模块工作树在本任务前已有暂存删除状态。本轮从其 HEAD 提取源码到 `.build/mapping_source` 隔离构建，没有恢复或改写该子模块工作树/index。
- 工作区还有用户原有修改和其他采集工作的文件，尚未提交。本轮没有提交、推送或清理这些文件；后续不能使用 `git reset --hard` 或 `git clean` 整理工作区。

本记录用于交接已经停止的实验与尚未解决的建图问题，不代表建图已修复或重定位已完成验收。
