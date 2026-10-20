# 全局重定位仿真评估报告

**当前阶段：**55-key bounded 固定 45 项已完成；2026-10-19 复核已有产物，未启动新测试或构建。结果和停止边界见[恢复检查点](global_relocalization_resume.md)。

## 当前 bounded 候选（55 keys；固定回归 45/45）

当前冻结 profile SHA-256 为 `1c1bba993a6aae1d9ab9854748839098a31863d1aa943e45e92c1497309db0c0`，55 个参数键，`candidate_workers=2`，server timeout 60 s。冻结 manifest SHA-256 为 `6a12676661f03b7fdfe846f2d7ee67f3e46b3b185281ecef7971b9db7b2927b4`；对应 core/node SHA-256 分别为 `5ac470d46553176b61928400a7672e6963cf2d722ae8aba8cc482e4f65dc592b` / `c662e64e61aa3eb3307100d4e20e8f7c8f9225fb255f18dedc068f5ea4674109`。live binary、source snapshot、输入地图与配置来源见[冻结 manifest](../tools/global_relocalization/data/resume_parallel_validation_20261014T123937Z/bounded_frozen_manifest.json)、[live freeze provenance](../tools/global_relocalization/data/resume_parallel_validation_20261014T123937Z/bounded_live_p16_20261014T154541Z/freeze_provenance.json)和[run manifest](../tools/global_relocalization/data/resume_parallel_validation_20261014T123937Z/bounded_live_p16_20261014T154541Z/run_01/batch_manifest.json)。共享 main 目前还含其他任务的 SearchObserver/visualization 修改；headless 定位验收以归档 source snapshot 和实际加载 binary 为准，observer GUI 不计入验收。

### 已证实的实现与离线结果

审计定位到旧查询会先做无界 KD 最近邻搜索，再按距离拒绝远点。新 adapter 在 KD 查询阶段做精确半径剪枝，同时保留原树结构、tie 处理、距离门限、全部候选、歧义检查和三次独立确认；并行配置仍为 2 candidate workers × 4 OpenMP threads，60 s 服务端 timeout 不变。STOPPING 修复也已合入：逐帧运动会重启完整 2 s wall/ROS 静止窗口；freeze/drain 输入后复核 stage、帧数和稳态；raw Core 至少要求 3 个独立确认帧。

bounded build 的 44 个检查全通过。对相同冻结 p16 capture，原版与 bounded 版完成相同 13 候选和 3 次确认，593 项比较均在绝对误差 1e-12 内，最大差异 2.22e-15。单次同二进制离线查询 wall 为 18.79→5.87 s，CPU 为 74.56→31.46 core-s，峰值 RSS 为 115.60→115.45 MiB。独立微基准的远区查询快约 5.8–6.0 倍，近区基本持平。该 capture 无 GT，这些单样本数据只证明查询行为与耗时变化，不代表 live 正确率、全场景性能或 action P95。原始证据见[bounded comparison](../tools/global_relocalization/data/resume_parallel_validation_20261014T123937Z/bounded_validation/comparison.json)、[build/test manifest](../tools/global_relocalization/data/resume_parallel_validation_20261014T123937Z/bounded_frozen_manifest.json)和[微基准日志](../tools/global_relocalization/data/resume_parallel_validation_20261014T123937Z/bounded_micro.log)。

### 完整 45 项结果与验收口径

2026-10-14 UTC 的冻结回归已完成，2026-10-19 只复核归档：run id `accuracy45_08792fbbe785`，三轮各 15/15，共 **45/45 严格正确**（3D ≤0.10 m、SO(3) ≤2°），40 项无恢复、5 项一次恢复后正确，失败 action / 错误接受均为 0，15 s 内正确 **0/45**。15 秒目标未达到。45/45 只描述固定理想仿真集合，含恢复的系统通过不代表局部配准 100% 成功。

45 项完整 3D 误差均值/RMSE/P95/最大为 **1.165732 / 1.301647 / 2.227641 / 2.846049 mm**；SO(3) 为 **0.029225 / 0.036750 / 0.060048 / 0.122663°**。action elapsed wall 均值/P95/最大为 **27.947963 / 32.315618 / 34.985493 s**（最短 24.840043 s）。P95 对全部 45 项线性插值，不平均每轮 P95。

算法 45 个有效资源窗口总计 **1488.44 core-s**，按窗口 wall 加权 CPU **118.1981%**，trial 窗口平均 CPU 的均值/P95/最大 **114.9132 / 202.3304 / 216.6429%**，最大 RSS **219.984375 MiB**；Gazebo 仿真单列 **1642.36 core-s**、加权 CPU **130.3919%**、最大 RSS **2999.910156 MiB**。CPU 100% 为一个逻辑核，trial CPU 最大是窗口平均值的最大，不是瞬时峰值。外部 host 的独立 sim/RViz/algorithm 负载不计入算法；归档环境负载不能写成重启后的当前进程状态。

5 次恢复发生于 round 1 `p13_yaw3`、`p16_yaw2`，round 2 `p13_yaw1`、`p01_yaw2`，round 3 `p13_yaw1`；原因均为局部数值不收敛，下一扫描仅恢复一次。45 个终态的 queue drop/overflow/stale/oversized/canceled 均为 0；不能据此宣称任意负荷下无丢帧。45 个 `(run_id,repetition,trial_id)` 唯一键及 exact pose-stamp 评分联接全部通过；实际冻结矩阵第 9 项是 `p10_yaw3`。

原始[回归 manifest](../tools/global_relocalization/data/resume_parallel_validation_20261014T123937Z/bounded_fresh45_20261014T154902Z/accuracy45/regression_manifest.json)和三轮 [round 1](../tools/global_relocalization/data/resume_parallel_validation_20261014T123937Z/bounded_fresh45_20261014T154902Z/accuracy45/round_01/results.summary.json)、[round 2](../tools/global_relocalization/data/resume_parallel_validation_20261014T123937Z/bounded_fresh45_20261014T154902Z/accuracy45/round_02/results.summary.json)、[round 3](../tools/global_relocalization/data/resume_parallel_validation_20261014T123937Z/bounded_fresh45_20261014T154902Z/accuracy45/round_03/results.summary.json)保持不动；完整[结果分析](../tools/global_relocalization/data/resume_parallel_validation_20261014T123937Z/bounded_fresh45_20261014T154902Z/analysis.md)、[重算 summary](../tools/global_relocalization/data/resume_parallel_validation_20261014T123937Z/bounded_fresh45_20261014T154902Z/summary.json)和[恢复审计](../tools/global_relocalization/data/resume_parallel_validation_20261014T123937Z/bounded_fresh45_20261014T154902Z/resumption_audit_20261019.md)提供逐项证据。

本 45 项批次的历史清理记录缺失：归档临时 driver 在读取 `/proc/18288/environ` 时抛出 `psutil.AccessDenied`，`cleanup.json` 未落盘；不能宣称历史 zero Twist 成功或完整清理。重启后无所属仿真/ROS 进程仅证明当前状态。正式 batch 已处理该读取异常，归档临时 `bounded_live_runner.py` 不得原样复用；详见[审计](../tools/global_relocalization/data/resume_parallel_validation_20261014T123937Z/bounded_fresh45_20261014T154902Z/resumption_audit_20261019.md)。

地图由同一理想仿真场景的 GT 融合得到；在线估计器不订阅 GT，但参考地图和同源传感器限制了精度结论的外推。raw scan 为 instantaneous、无逐点时间，不能证明真实旋转扫描去畸变或实车可靠性。当前结果没有验证共享 main 中其他用户的 SearchObserver/visualization GUI 版本。timed-scan、Point-LIO 地图/速度、加速旋转、GUI、导航、长时稳定性及 120 项后续测试继续暂停。 GPU headless domain 142 的独立 p16 三次诊断全正确且无恢复（15 秒内 0/3），不计入本批分母；domain 140 的原 15-cell 物理预检为 15/15。旧 53-key 29/30、15 项未试和 54-key p16 60.227 s timeout 均为独立历史记录。

## 历史鲁棒恢复回归（2026-10-14；53-key 首轮中止）

首个鲁棒 profile [`raw_imu_robust_left_half.yaml`](../src/global_relocalization/config/raw_imu_robust_left_half.yaml)（53 个参数键；本次冻结 SHA-256 `7b257f9cccf437a59d9bca777bb874ca2ce917edf0640671f7db261c3470de26`）的固定 15-cell × 3 轮回归，在 fresh session `robust_correctness_fresh_20261014T101242Z_87dd84d4`、ROS domain 139 中第 2 轮发生全局搜索超时并停止：计划 45 项、执行 30 项，其中 29 项 action 成功且严格全姿态评分通过；1 项 action 超时，第三轮 15 项未尝试。session 实际 15/15 物理预检有效。分轮为 round 1：15/15 正确，round 2：14/15 正确并有 1 次超时，round 3：未启动。该 run 的 `passed_45_of_45=false`，29/30 只是已执行部分的条件结果，不能写成 45/45 或完整成功率。旧正式 120 次和旧固定 15 次结果仍是其他配置/矩阵的历史数据。

超时发生在 `p16_yaw2`：server 60 秒预算结束时，14 个检索候选已测试 12 个；与目标区域接近的候选 fitness 为 0.08648、inlier ratio 为 1.0，但还没有完成独立停稳确认，因此没有输出 pose 或评分。raw 前端 queue drop/overflow 为 0、队列深度为 0，单扫描 worker 最大时间约 20.4 ms；本次证据定位到全局候选搜索尚未完成，不支持把它归因于 raw 前端接收积压。该 60 秒门限未放宽。

同一位置的一次独立单 cell 诊断 action 在 54.153 秒内得到严格评分通过，完整 3D 误差为 3.151 mm，SO(3) 误差为 0.03831°。这条诊断记录不属于 45-cell 回归分母。随后曾冻结 54-key、`candidate_workers=2` 版本（profile SHA-256 `c45f008347edc394dc1b2a0cb09ec05550f222a2ef1408e2a4ea967ae2de238b`；core `f3d8063527e4dad6ed815c08e049e4f7fb54acb3a84a3be4a806d7fcb1d355ea`；node `5255eb8295d35ff7935a2b3666ad42dfe79b9221f665a0ea8e13a309f4da88e2`）。该版本的独立 live p16 在 60.227 s 服务端 timeout，未输出 pose，且未进入三次确认；这是历史诊断，不属于当前 55-key bounded build。当前 build 的离线 bounded 对照和 live 状态见本报告顶部。

| 已执行轮次 | 严格正确 | 3D 位置误差 RMSE / 最大 | SO(3) 误差 RMSE / 最大 | action wall 均值 / P95 / 最大 | 15 秒内正确 | 恢复分类 |
|---|---:|---:|---:|---:|---:|---|
| round 1（15 项） | 15/15 | 1.525 / 3.017 mm | 0.03868 / 0.09864° | 29.741 / 35.913 / 42.523 s | 0/15 | 首次正确 14；一次恢复后正确 1 |
| round 2（15 项） | 14/15 | 1.364 / 2.711 mm（14 个有效评分） | 0.04034 / 0.09904°（14 个有效评分） | 30.955 / 40.479 / 60.218 s | 0/15 | 首次正确 14；超时 1 |

这 30 项里有 28 次首次正确、1 次一次内部恢复后正确、1 次 action 超时；没有错误接受或未评分成功 action。以已观察样本计算，15 秒内正确为 0/30，成功 action 条件率为 0/29；另有 15 项未观察，不能当作通过。round 1/2 的算法 CPU 均值/P95 分别为 78.42%/122.13% 与 79.21%/116.10%，peak RSS 最大值分别为 217.6 MiB 与 223.6 MiB；Gazebo CPU 均值/P95 为 85.76%/88.82% 与 86.51%/90.69%，peak RSS 最大值为 2,101.3 MiB 与 2,643.9 MiB。CPU 100% 表示一个逻辑核全占用。

完整逐轮 ledger、计时/资源表和 frozen-input 证明见 [首轮 regression manifest](../tools/global_relocalization/data/robust_correctness_fresh_20261014T101242Z_87dd84d4/regression45/regression_manifest.json)、[round 1 summary](../tools/global_relocalization/data/robust_correctness_fresh_20261014T101242Z_87dd84d4/regression45/round_01/results.summary.json)、[round 2 summary](../tools/global_relocalization/data/robust_correctness_fresh_20261014T101242Z_87dd84d4/regression45/round_02/results.summary.json)、[p16 超时诊断](../tools/global_relocalization/data/robust_correctness_fresh_20261014T101242Z_87dd84d4/p16_timeout_diagnosis.json) 与 [p16 独立诊断结果](../tools/global_relocalization/data/robust_correctness_fresh_20261014T101242Z_87dd84d4/p16_search_capture/live/results.summary.json)。本轮没有启动 timed-scan、Point-LIO 地图/速度比较、长时稳定性或 120 次稳定性评估；这些后续项目仍暂停。

## 上一版 raw LiDAR + IMU 正式批测（2026-10-14）

最新正式批次覆盖 30 个静置位置、每点 4 个初始航向，共 120 次；定位成功 58 次（48.33%），未达到 ≥114/120（95%）验收目标。58 个成功姿态的 3D 位置误差均值/RMSE/P95/最大为 1.451/1.828/3.969/4.982 mm，SO(3) 为 0.0374/0.0555/0.1213/0.2005°。成功动作墙钟均值/P95/最大为 31.331/41.748/47.617 s。51 次失败为 raw scan coverage gap，且全部伴随队列丢帧；详细精度、计时、负荷、来源 hash 与失败分析见 [正式测试报告](../tools/global_relocalization/data/session_20261013T160806Z_0f2b982a/formal_raw120_run01/analysis.md)、[批次 summary](../tools/global_relocalization/data/session_20261013T160806Z_0f2b982a/formal_raw120_run01/results.summary.json) 和 [完整性核验](../tools/global_relocalization/data/session_20261013T160806Z_0f2b982a/formal_integrity_check.json)。

本段结果来自 raw LiDAR + IMU 路线。下方 2026-10-13 章节记录的是此前 Point-LIO 输入路线的独立实验，运行配置和失败原因不同，两组数据分别保留。

## 历史正式左半场批测结果（2026-10-13，Point-LIO 输入路线）

这是隔离仿真的静置 → 实际自转 → 停稳 → Scan Context 检索 → 独立停稳扫描 GICP 确认流程。只启动评估所需的 Point-LIO 和全局重定位节点；不启动 Nav2、旧 relocation manager 或实车。Point-LIO 使用 `/registered_scan` 与 `/odometry`，真值 topic 只供评估器读取，定位器不订阅真值、不发布 localization TF。

固定搜索先验为 `x < 0`（`search_side: negative_x`，边界 `x = 0`），来自用户指定的粗半场范围，不根据在线真值选择候选。正式矩阵覆盖 30 个不同位置、每点 4 个初始航向，共 120 次；完整保留所有失败和超时。批次成功定义为动作成功并且全姿态评估同时满足 3D 位置误差 ≤0.10 m、SO(3) 角误差 ≤2°。

结果为 **68/120 成功（56.7%）**，52 次失败，其中 6 次超时。68 个成功输出均按动作内部 `action.pose_stamp_ns` 精确联接到唯一评分记录，全部使用 `map` 中的 `base_footprint` 姿态，并在真值时间序列中找到有效插值括区。失败动作没有误联接旧分数；失败分数为空值，不折算为零。68 个有效成功姿态均通过完整门限。下表精度仅描述这 68 个成功输出，不能替代 120 次总成功率。

| 误差指标（68 个成功姿态） | 均值 | RMSE | P95 | 最大值 |
|---|---:|---:|---:|---:|
| 完整 3D 位置误差 | 2.654 mm | 3.534 mm | 7.347 mm | 11.693 mm |
| SO(3) 姿态角误差 | 0.0519° | 0.0695° | 0.1053° | 0.2704° |

误差轴分量的偏差/RMSE 为：dx −0.698/2.372 mm、dy +0.220/2.476 mm、dz −0.118/0.855 mm；roll +0.0093/0.0553°、pitch +0.0037/0.0248°、yaw −0.0001/0.0340°。这些是条件精度统计；失败次数仍按 120 作为总成功率分母。

失败原因按 action ledger 分类：37 次因旋转期间 Point-LIO 里程计显示过大平移而拒绝；7 次独立停稳扫描确认不一致或质量门控拒绝；2 次没有候选同时通过 GICP 质量与半场先验门控；6 次达到墙钟超时。无评分成功动作以外的有效姿态输出。位置和航向的成功分布见 30×4 表，`✓` 为成功且全姿态门限通过，`L` 为里程计平移门控，`C` 为独立停稳扫描确认拒绝，`N` 为无合格候选，`T` 为超时；`*` 表示该位置后验真值检查判为物理前提偏离。

| 位置（x, y）m | 0° | +90° | 180° | −90° | 真值物理测量 | 成功数 |
|---|---:|---:|---:|---:|---|---:|
| p00 (−7.275, −0.285) | ✓ | ✓ | ✓ | ✓ | 有效 | 4/4 |
| p01 (−10.175, 1.165) | L | L | C | ✓ | 有效 | 1/4 |
| p02 (−11.775, −1.085) | ✓ | ✓ | ✓ | ✓ | 有效 | 4/4 |
| p03 (−5.775, 1.965) | ✓ | ✓ | ✓ | ✓ | 有效 | 4/4 |
| p04 (−13.525, −0.735) | ✓ | ✓ | ✓ | ✓ | 有效 | 4/4 |
| p05 (−5.975, 0.865) | ✓ | ✓ | ✓ | ✓ | 有效 | 4/4 |
| p06 (−4.625, −2.535) | N | L | L | N | 有效 | 0/4 |
| p07 (−4.725, 4.615) | ✓ | ✓ | ✓ | ✓ | 有效 | 4/4 |
| p08 (−5.525, −1.935) | ✓ | ✓ | ✓ | ✓ | 有效 | 4/4 |
| p09 (−12.675, 0.715) | L | C | ✓ | L | 有效 | 1/4 |
| p10 (−5.625, −2.985) | C | L | ✓ | L | 有效 | 1/4 |
| p11 (−10.725, −1.335) | L | L | L | L | 有效 | 0/4 |
| p12 (−11.125, −3.035) | L | L | L | L | 有效 | 0/4 |
| p13 (−9.575, −1.285) | L | L | L | T | 有效 | 0/4 |
| p14 (−9.425, 2.115) | ✓ | ✓ | ✓ | ✓ | 有效 | 4/4 |
| p15 (−3.625, 5.265) | L | ✓ | L | L | 有效 | 1/4 |
| p16 (−2.625, 6.665) | L | L | L | L | 有效 | 0/4 |
| p17 (−14.425, −2.135) | T | L | T | L | 偏离* | 0/4 |
| p18 (−6.025, −0.985) | ✓ | ✓ | ✓ | ✓ | 有效 | 4/4 |
| p19 (−6.675, 2.965) | ✓ | ✓ | ✓ | ✓ | 有效 | 4/4 |
| p20 (−8.575, 0.165) | ✓ | ✓ | ✓ | ✓ | 有效 | 4/4 |
| p21 (−12.625, −1.885) | ✓ | ✓ | ✓ | ✓ | 有效 | 4/4 |
| p22 (−10.475, 0.065) | ✓ | ✓ | ✓ | ✓ | 有效 | 4/4 |
| p23 (−11.575, 0.215) | L | C | ✓ | L | 有效 | 1/4 |
| p24 (−13.225, 2.915) | L | L | L | L | 有效 | 0/4 |
| p25 (−9.075, 3.415) | ✓ | ✓ | ✓ | ✓ | 有效 | 4/4 |
| p26 (−13.225, 1.665) | C | L | T | C | 有效 | 0/4 |
| p27 (−4.175, 6.315) | ✓ | C | ✓ | ✓ | 有效 | 3/4 |
| p28 (−9.075, 1.115) | ✓ | ✓ | ✓ | ✓ | 有效 | 4/4 |
| p29 (−14.525, −3.235) | T | L | L | T | 偏离* | 0/4 |

### 物理前提核验

`physics_validation.json` 是批后独立生成的真值轨迹分类，不接入定位器、不改变冻结算法门限，也不删除或改写原 120 次分母。判为“测量有效”的标准为至少 50 个真值样本、观测时长至少 1 s，且相对起始姿态最大平移不超过 0.10 m、绝对 roll/pitch 不超过 2°、高度范围不超过 0.02 m。120 次都有足量真值数据：112 次符合该测量标准，8 次偏离；按位置计为 28/30 个位置有效，p17 和 p29 的 8 个航向试验全部偏离且均未成功。

p17 的试验真值最大平移为 0.232 m、最大绝对 roll/pitch 为 81.96°/49.83°、高度范围为 0.128 m；p29 对应为 0.780 m、32.91°/24.49°、0.742 m。它们与“平坦自由空间”名义标签不符，必须作为原批失败保留。其余 112 个物理测量有效试验中 68 次成功、44 次失败（33 次里程计门控、7 次确认、2 次无候选、2 次超时）。这个子集用于解释试验前提，不取代主指标 68/120。

在 p11、p12、p16、p24 等物理测量有效位置，四个航向均被 LIO 平移门控拒绝；这些试验的真值位置保持原地，平移与倾斜稳定，同时机器人仍完成自转。证据支持局部里程计轨迹退化与这些拒绝同时出现，但不能单凭此实验确定其具体成因。每个位置的轨迹明细和判据保存在 [physics_validation.json](../tools/global_relocalization/data/batch_left120_domain91_20261013/physics_validation.json)，逐位置摘要在 `physics_validation.positions.csv`。

### 独立平地补充试验（不并入 120 次）

为单独检查物理前提，从相同随机种子的后续候选位置中，用 3 s 静置真值检查选择了 p30 `(-12.475, -3.135)` 和 p31 `(-5.575, 3.215)`，再对每点运行四个航向。该补充集是经真值静置物理条件筛选的点集，不是原始 30 点随机矩阵的延伸统计；不与正式 120 次合并。8 个活动试验的真值位置保持原地、平移与倾斜稳定，最大相对平移不超过 0.307 mm，绝对 roll/pitch 小于 8×10⁻⁶°；期间机器人执行了自转。

补充结果为 5/8 成功（62.5%），3 次均被 LIO 平移门控拒绝、没有超时；p30 成功 1/4（仅 180°），p31 成功 4/4。成功 5 姿态的 3D 误差均值/RMSE/P95/最大为 2.278/2.744/4.626/5.147 mm，SO(3) 为 0.0436/0.0463/0.0637/0.0654°。8 次动作墙钟均值/P95/最大为 21.639/33.922/37.964 s，成功子集 5 次为 28.190/35.654/37.964 s。详见 [补充结果 summary](../tools/global_relocalization/data/supplemental_left_8_domain91_20261013/results.summary.json)、[补充 trials CSV](../tools/global_relocalization/data/supplemental_left_8_domain91_20261013/results.trials.csv)；候选静置物理预检记录在 [checks.json](../tools/global_relocalization/data/supplemental_left_static_precheck_20261013/checks.json)。

### 计时

正式批次总墙钟时间为 58.9515 分钟。单次 `total_wall` 包含每次动作窗口；总体均值包含提前拒绝和超时，因此同时报告成功子集，避免把早停解释为定位更快。采集总时长为 static + sweep + stop-settle；计算总时长为 retrieval + GICP + confirmation。只有所有组成段都有有效数值时才统计组合项；样本数不足 120 是由于相应阶段没有执行或缺少有效计时，不以零补齐。

| 时长 | 子集/样本数 | 均值 | P50 | P95 | 最大 |
|---|---:|---:|---:|---:|---:|
| 总动作墙钟 | 全部 120 | 22.487 s | 23.658 s | 48.449 s | 60.040 s |
| 总动作墙钟 | 成功 68 | 24.664 s | 24.054 s | 27.057 s | 42.533 s |
| 采集总时长 | 可计算 77 | 23.115 s | 22.098 s | 29.586 s | 44.826 s |
| 采集总时长 | 成功 68 | 22.385 s | 22.078 s | 22.741 s | 38.867 s |
| 计算总时长 | 可计算 75 | 2.297 s | 1.970 s | 4.732 s | 6.656 s |
| 计算总时长 | 成功 68 | 2.231 s | 1.938 s | 4.518 s | 6.656 s |

全体可用计时的阶段墙钟均值/P95/最大值为：`search_total` 2.336/4.759/6.700 s（n=77），retrieval 0.033/0.020/1.113 s（n=77），GICP 2.275/4.703/6.635 s（n=77），confirmation 0.0019/0.0026/0.0074 s（n=75）。首个成功查询的 retrieval 为 1.113 s，包含首次数据库构建；数据库随后复用，热查询约 0.02 s。状态接口没有独立的数据库构建计时，故不推算单独的建库时长。分阶段 ROS 时间在状态接口未提供的项目中保留为空。全批 RTF 均值为 0.99982，P95 为 1.00004。

### 资源

资源以每个动作窗口为样本，测量算法进程、Point-LIO 进程组、Gazebo 进程组，以及单独的 trial evaluator 客户端。CPU 百分比为 `(user + system) / wall × 100`；100% 表示一个逻辑 CPU 核完全占用，允许多线程进程超过 100%。这里的“最大 CPU”是 120 个动作窗口平均 CPU 百分比中的最大值，不是瞬时峰值。RSS 为每个窗口采样到的进程组峰值。

| 进程组（120 个窗口） | CPU 均值 | CPU P95 | 最大单窗口均值 | RSS 最大实测峰值 |
|---|---:|---:|---:|---:|
| 全局重定位算法 | 34.21% | 69.64% | 103.78% | 281,792,512 B（268.8 MiB） |
| Point-LIO pipeline | 56.75% | 61.14% | 67.69% | 335,478,784 B（319.9 MiB） |
| Gazebo | 82.97% | 88.19% | 90.45% | 5,743,616,000 B（5.35 GiB） |
| Trial evaluator 客户端（单独统计） | 37.02% | 38.17% | 38.74% | 110,043,136 B（104.9 MiB） |

批后用 matplotlib `Agg` 静态生成了 [位置与姿态误差经验 CDF](../tools/global_relocalization/data/batch_left120_domain91_20261013/results.error_cdf.png) 及 [阶段时长和进程 CPU 分布](../tools/global_relocalization/data/batch_left120_domain91_20261013/results.performance.png)。

运行环境为 WSL2 Linux，32 个逻辑 CPU、16 个物理 CPU、约 19.4 GiB 内存；ROS domain 为 91，使用独立 Gazebo partition，`gazebo_gui=false`。正式结果不含 GUI/RViz observer。

## 冻结输入与实现来源

- 正式输出目录：`tools/global_relocalization/data/batch_left120_domain91_20261013/`。关键证据可直接打开：[batch manifest](../tools/global_relocalization/data/batch_left120_domain91_20261013/batch_manifest.json)、[原始 trials ledger](../tools/global_relocalization/data/batch_left120_domain91_20261013/trials.jsonl)、[按时间戳评分的 pose rows](../tools/global_relocalization/data/batch_left120_domain91_20261013/pose_errors.jsonl)、[正式 summary](../tools/global_relocalization/data/batch_left120_domain91_20261013/results.summary.json)、[逐试验 CSV](../tools/global_relocalization/data/batch_left120_domain91_20261013/results.trials.csv)、物理核验 JSON 见上方链接。
- 冻结地图：`tools/global_relocalization/data/station_map_20261013/map.pcd`，457,278 点，SHA-256 `2b9e08cda8a63ade7ec3e153fe56363a4effd6566ed4a066c764b369abda0570`。manifest 声明固定 `T_map_world = identity`、`map/world/base_footprint` 帧；地图由 1,434 个已融合扫描构成，另有 16 个丢弃扫描。
- 冻结先验与矩阵：`src/global_relocalization/config/left_half.yaml` SHA-256 `f1c51ee22febb45bbb4eda655029473fc5a69124ee0fd347239191fd811eea8a`；`tools/global_relocalization/config/trials_left_120.json` SHA-256 `d03599f4e946c14cab914c26d9a7a40fe2b33dbd83675da3d13cb7eaf949cc0f`。矩阵 30 个位置均满足 `x < 0`，每个位置按 0°、+90°、180°、−90°测试。批次 manifest 记录 ground truth 未用于候选筛选。
- 算法程序来自隔离安装 `tools/global_relocalization/.build/ros/install/global_relocalization`；可执行文件 SHA-256 `3714cdb5941cc0762f405b9cfdb8dd725630157653cb8b29e00f273969444cde`，core 库 SHA-256 `215d4d05d308c99b3891c6d68ca5cbf31a8de4e883e0241b3a179fb7f3809da0`。
- 实际 Point-LIO 可执行文件为 `/home/zhang/ros_ws_jazzy/sentry_navgation_RM27/build/point_lio/pointlio_mapping`，SHA-256 `eb0ed9afd05d419e9fd5e9232bb54d828f6c109baa836007e07cc8b772139aed`。使用的冻结 `lio.yaml` SHA-256 为 `1a70129f97fcb76c9d4afbdb40acaa3b9276f0fd781b0a4f36243ba545d35718`。该原版二进制不声明 `estimator.max_iterations`、`input_guard.*` 或 `adaptive.*` 参数；虽然 YAML 中有这些键，不能把它们解释为已生效的运行时调节，也不据此声称关闭了自适应改进。
- 批次执行代码与 scorer 的 SHA-256 保存在 manifest。正式运行所用 `evaluate_pose.py` SHA-256 为 `58697c33ab451432ef4ef6c627c3ac91e7d72b6c93179f441082f6ce90c5b6ba`；本次正式结束后未改动执行脚本。

## 评分和复现实验协议

评估器仅订阅估计 `/global_relocalization/pose` 和真值 `/ground_truth/odometry`。估计姿态按消息时间戳查询真值；平移线性插值、姿态最短弧四元数 SLERP，不外推，最大真值括区间隔 0.25 s；估计时间戳超出真值缓存时最多等待 1 ROS 秒以形成未来括区。真值经地图 manifest 固定的 `T_map_world` 变换，绝不在线拟合对齐。估计消息为 `map` 帧的 `base_footprint` 姿态。批次 join 只接受成功且非超时 action 中嵌套的整数 `action.pose_stamp_ns`，同一 trial stamp 或评分 stamp 重复会报错；失败 action、超时和缺分保留在分母且不复用旧分数。

每个有效姿态报告 signed dx/dy/dz、XY 和完整 3D 误差、roll/pitch/yaw 差，以及完整 SO(3) geodesic 角误差。位置门限始终为 3D ≤0.10 m，姿态门限始终为 SO(3) ≤2°；XY 和 yaw 只作为诊断量。

正式配置与工具命令如下。每次复跑使用新的输出目录，不覆盖本次证据；仿真保持 headless，与批处理使用相同 ROS domain 和独立 Gazebo partition。

```bash
tools/global_relocalization/build.bash
tools/global_relocalization/prepare_simulation.bash
export ROS_DOMAIN_ID=91
export GZ_PARTITION=global_relocalization_91_left120_run01
tools/global_relocalization/start_simulation.bash
```

在另一个终端设置相同环境变量，再执行：

```bash
export ROS_DOMAIN_ID=91
export GZ_PARTITION=global_relocalization_91_left120_run01
tools/global_relocalization/run_batch.bash \
  --map tools/global_relocalization/data/station_map_20261013/map.pcd \
  --manifest tools/global_relocalization/data/station_map_20261013/manifest.json \
  --trials tools/global_relocalization/config/trials_left_120.json \
  --params src/global_relocalization/config/left_half.yaml \
  --output /path/to/new-left-half-output
```

正式 120 次结束后，可用固定 scorer 重新聚合并生成静态 PNG：

```bash
/usr/bin/python3 src/global_relocalization/scripts/evaluate_pose.py --aggregate \
  --trials tools/global_relocalization/data/batch_left120_domain91_20261013/trials.jsonl \
  --pose-errors tools/global_relocalization/data/batch_left120_domain91_20261013/pose_errors.jsonl \
  --output-prefix tools/global_relocalization/data/batch_left120_domain91_20261013/results \
  --expected-trials 120 --plots
```

`--plots` 使用 matplotlib `Agg` 非交互后端，只保存 PNG，不打开窗口。

## 预检、测试与仿真控制器

五次 left-half 预检独立于正式批次：4/5 动作成功，无超时；唯一拒绝为旋转期间里程计平移过大。4 个有效 pose 的 3D 误差均值/RMSE/P95/最大为 2.074/2.176/2.831/2.877 mm，SO(3) 为 0.0466/0.0510/0.0701/0.0721°。预检结果不并入正式 120 次。其 ledger 和 summary 在 `tools/global_relocalization/data/pilot_left_domain91_5_20261013/`。

`python3 src/global_relocalization/test/test_evaluate_pose.py` 的 19 个 scorer 单元测试在批测前通过，覆盖四元数等价与旋转误差、角度环绕、SLERP、时间戳插值和禁止外推、延迟括区超时、完整 3D/SO(3) 门限、固定地图变换与哈希校验、失败/超时分母、重复 stamp 与严格 action stamp join。半场 prior 的隔离 CTest 共 2/2 target 通过：8 个 core case 用时 0.59 s，ROS action contract 用时 3.27 s，总计 3.86 s。日志 `tools/global_relocalization/data/half_field_prior_ctest.log` 的 SHA-256 为 `19d17ad6ba666d59460ec180750b3d68523ba47d050ef77cca0cc1b0ec9a4f76`。这些测试验证实现和 action 合约，不替代本报告实测成功率。

仿真专用机器人描述使用 Gazebo 内建 `VelocityControl` 订阅 `/cmd_vel`；没有验证实车力控。独立整圈试验的 22 s 仿真命令得到 7.848454 rad（449.68°）转角，净平移 0.216 mm；停止时真值姿态差分角速度为 0，IMU 原始角速度最大 1.45e−6 rad/s。Gazebo GT twist 的角速度字段停后仍缓存约 0.35653 rad/s，所以停止判据依据姿态轨迹和 IMU，不使用该陈旧 twist。记录为 `tools/global_relocalization/data/physical_rotation_builtin_full_turn.json` 及其 JSONL trace。

此前 ROS domain 86 时钟污染及全场诊断不属于正式左半场统计，证据留在其原目录；它们没有混入本次 120 次汇总。

## 原始 LiDAR+IMU 路线阶段记录（截至 2026-10-13；当时正式 120 次尚未运行）

本节记录一条独立的原始传感器路线：定位器直接接收瞬时 LiDAR 点云和 IMU，利用 IMU 重力/陀螺初始化和局部 6D GICP 估计静置自转运动，再执行全局候选检索和停稳扫描确认。配置明确使用 `raw_scan_model: instantaneous`；点云没有逐点采样时间，因此这批数据不能证明 timed-scan 去畸变效果，也没有验证实车力控。Ground truth 仅供离线物理前提检查和按时间戳评分，不进入定位或控制。

截至该阶段记录时，原正式基线仍是上文旧 30 点矩阵：68/120（56.7%）。新的候选 120 矩阵由 28 个旧位置和 p30、p31 两个替换位置组成；旧矩阵中 p17、p29 的物理前提检查不通过，因此新旧整体成功率不是严格同矩阵对照。新矩阵已完成 120/120 静置真值物理预检，结果为全通过；该预检不测定位成功率。证据在 [静态预检 summary](../tools/global_relocalization/data/raw_left120_static_precheck_20261013/summary.json) 和 [固定矩阵](../tools/global_relocalization/config/trials_left_physical_120.json)。随后进行的正式 raw LiDAR + IMU 120 次批测结果见本报告顶部。

### 五次路线预检

两次五样本运行均保留全部五次动作作为成功率分母；误差只统计有有效估计姿态并精确联接到离线评分的输出。失败动作没有姿态误差，不折算为零。表中的“全部动作”耗时包括成功和失败；“成功动作”只用于观察已成功流程耗时，不能代表批次总可用性。

| 批次 | 动作成功 | 有效评分姿态 | 位置误差（有效姿态）均值 / RMSE / P95 / 最大 | SO(3) 误差均值 / RMSE / P95 / 最大 |
|---|---:|---:|---|---|
| 首次原始路线 pilot | 3/5 | 3/5 | 1.340 / 1.518 / 2.123 / 2.210 mm | 0.04790 / 0.05231 / 0.07000 / 0.07216° |
| DAQ 修正后 pilot | 4/5 | 4/5 | 1.490 / 1.755 / 2.721 / 2.953 mm | 0.04683 / 0.05161 / 0.06958 / 0.07058° |

首次 pilot 的两次失败分别为 `IMU coverage gap` 和 `raw scan coverage gap`；DAQ 修正后这两类数据覆盖故障没有再出现。修正后的 p06 仍因“没有候选通过 GICP 质量与半场边界门控”失败。单次 p06 诊断记录显示目标描述子候选处于 ring rank 5、Scan Context rank 16，未进入当前 top-10 GICP 队列；候选覆盖方案仍在验证。该诊断记录包含额外 rank dump 和 rosbag，属于单次故障诊断，不并入性能统计。

| 批次 | 总动作墙钟（全部动作）均值 / P95 / 最大 | 总动作墙钟（成功动作）均值 / P95 / 最大 | 算法 CPU 均值 / 最大（全部；成功） | 算法 RSS 最大值（全部 / 成功） |
|---|---|---|---|---:|
| 首次 pilot（n=5；成功 n=3） | 20.929 / 29.884 / 30.394 s | 28.515 / 30.139 / 30.394 s | 47.38% / 82.51%；68.17% / 82.51% | 231,010,304 / 231,010,304 B（220.3 MiB） |
| DAQ 修正 pilot（n=5；成功 n=4） | 28.497 / 33.409 / 34.925 s | 26.890 / 27.290 / 27.345 s | 69.11% / 139.17%；51.59% / 60.26% | 237,637,632 / 237,637,632 B（226.7 MiB） |

DAQ 修正 pilot 的 Gazebo CPU 均值/最大为全部动作 83.59%/87.69%、成功动作 82.57%/86.43%，RSS 最大值在全部/成功子集中均为 2,797,985,792 B（2.61 GiB）。Trial evaluator CPU 均值/最大为 43.16%/44.57%（全部）、42.81%/43.69%（成功），RSS 最大值在两组中均为 106,913,792 B（102.0 MiB）。CPU 为 `(user + system) / wall × 100`，多线程时可以超过 100%。Point-LIO 已禁用，LIO CPU、RSS 保留为 null。原始前端运行在定位器算法进程内，CPU/RSS 计入算法组，不另行重复计数。DAQ pilot 中一个动作窗口还单独记录了诊断 rosbag 进程（13.12% 平均 CPU、178,589,696 B RSS）；该进程只覆盖 1/5 个样本，正式 120 批测不配置诊断 bag。

### 原始输入计数的口径与完整度

节点 terminal status 原生提供平铺字段。离线聚合器现已把这些字段映射到 `raw_frontend.inputs` 和 `raw_frontend.metrics`，同时原样保留 `terminal_status` 与 `status_events`。DAQ 修正 pilot 的五个终态都有 LiDAR/IMU 接收、使用或接受、拒绝计数：LiDAR received 均值 284.4、used 均值 223.8、rejected 均为 0；IMU received 与 accepted 均值均为 2,847.8、rejected 均为 0；显式 `queue_dropped` 均为 0。LiDAR 计数差不能当作丢帧：定位进入 SEARCHING 后仍收取新云以检查输入新鲜度，但停止局部前端处理。`queue_dropped` 才是有界 LiDAR 前端队列实际淘汰数。

DAQ pilot 中 `processing_sec` 五次都有测量，均值 4.035 ms、最大 5.027 ms；`max_worker_wall_sec` 均值 15.272 ms、最大 17.173 ms。节点未单独报告 stale 数、每传感器 drop 数或拒绝原因分解，这些字段保持 null；不从接收数与使用数之差推断。首次 pilot 的原生状态缺少大部分传感器计数，因此其对应字段的统计样本数为 0；只报告该批实际存在的字段。Trial client 的 raw topic 接收数是另一层观察，代表客户端收到了消息，不代表定位器接受或处理了这些消息。

两批重新按成功且非超时 action 的整数 `action.pose_stamp_ns` 与评分记录做精确联接。首次 pilot 5 条 trial 对应 3 条 pose score；DAQ pilot 对应 4 条。重复 stamp 会导致聚合报错；失败动作没有评分 stamp，不会继承上一个姿态的分数。全部成功仍按 `3D ≤0.10 m` 和完整 `SO(3) ≤2°` 共同判定，失败动作始终保留在 5 次分母中。

可复核的归一化汇总与逐试验记录分别在 [首次 pilot 汇总](../tools/global_relocalization/data/pilot_raw_failure5_20261013_48dc198665e4/schema_review.summary.json)、[首次 pilot trials](../tools/global_relocalization/data/pilot_raw_failure5_20261013_48dc198665e4/schema_review.trials.csv)、[DAQ pilot 汇总](../tools/global_relocalization/data/pilot_raw_failure5_daq_20261013_48dc198665e4/schema_review.summary.json) 和 [DAQ pilot trials](../tools/global_relocalization/data/pilot_raw_failure5_daq_20261013_48dc198665e4/schema_review.trials.csv)。p06 诊断记录见 `tools/global_relocalization/data/diagnostic_raw_p06_20261013_48dc198665e4/`，不属于上述两份性能汇总。

扁平 node-status 归一化新增了 evaluator schema 用例；`python3 src/global_relocalization/test/test_evaluate_pose.py` 共 21/21 通过。该检查只覆盖离线评分与汇总，不运行 ROS 或重新编译定位程序。

### 预检快照与下一步

DAQ pilot 的 [batch manifest](../tools/global_relocalization/data/pilot_raw_failure5_daq_20261013_48dc198665e4/batch_manifest.json) 记录了当次实际输入和程序来源：执行节点 SHA-256 `315541f397e8a247f5d05731ae0b47b1ea849d9b9afc1c8074167fa036ae8182`，core 库 `64dd03e2695092452f46bc7cffc45b538e32b1ee454f9f08284efaf44bedfbef`，地图 `2b9e08cda8a63ade7ec3e153fe56363a4effd6566ed4a066c764b369abda0570`，该 pilot 生效参数 `global.yaml` `e229c648e05cd03de2d5647af2d41eaf9d3d420b3e498ccdb5ff37820930ff14`，pilot 五次输入 `trials.json` `dd52d638fb5cd05702b907227e5567ab7e4e41a4ce06df05b0ae162d60d731dc`，Gazebo controller snapshot `2a2a6dc2182eb912843b69f12d92f697024d4ebfbe6cbf51b544430995f88638`。当次源文件逐文件哈希和 map manifest 哈希也在该 manifest 中。新增离线归一化评估器 SHA-256 为 `b3a6b44b47cf6167a036aa321471628a3dea385b8b6b6e8ba195c046a7d2cfd7`；它用于重聚合，不是当次定位节点二进制的一部分。

待测 120 矩阵 SHA-256 为 `3c3332f4b932ce89f34d3273106396ce5457ddf3f238730f31ae18aff946356c`；静态物理预检 manifest 记录同一哈希，且 120/120 物理检查通过。后续需先完成固定 5 次和 10 次路线预检，再决定启动正式 120 次。候选 top-k 覆盖修复尚未冻结；在正式批测前继续保持当前门限、固定矩阵和离线评分协议。


## 2026-10-14 前端修复固定15诊断

原58/120正式结果保留，本轮没有重跑120。使用预先冻结、保留physical120原ID/pose/index的15个诊断cells，仅执行一次：13/15严格成功、2个前端拒绝；六个原formal localfail cells均成功，但原pilot cell p13_yaw3出现convergence拒绝（same-pair15→30iterations retry一次仍不收敛），p26_yaw2仍触发linear_speed0.36215454m/s>原0.35门限。门限没有放松，也没有GT候选oracle。

实际预检15/15有效、runtime43/43全匹配，estimator不订阅GT；全部15terminal queue_drop/overflow0、worker_error为空、队列高水位1，独立可靠observer与node source stamp gap均0.1s、callback gap最大约0.122s。13有效输出的位置RMSE0.001643m、最大0.003179m，SO3最大0.113658°。成功action wall平均33.923s/p95 45.677s/最大49.351s；probe资源独立计入diagnostic recorder，这些诊断样本不能作为formal120性能结论。

build/core27+evaluator23+harness7共57checks通过。新session最终remaining owned=0，原artifacts全部保留。长时源输送burst未在本轮15中复现，所以本轮结果不能宣称所有问题修好。完整数据与限制见[固定15分析](../tools/global_relocalization/data/session_frontend_fix15_20261013T181313Z_64ccf218/pilot_raw_frontend_fix15_run01/analysis.md)、[summary](../tools/global_relocalization/data/session_frontend_fix15_20261013T181313Z_64ccf218/pilot_raw_frontend_fix15_run01/results.summary.json)、[成功cohort](../tools/global_relocalization/data/session_frontend_fix15_20261013T181313Z_64ccf218/pilot_raw_frontend_fix15_run01/results.success_cohort.json)、[cleanup](../tools/global_relocalization/data/session_frontend_fix15_20261013T181313Z_64ccf218/cleanup.json)；复现命令见test guide第12节。后续120或新算法试验需要独立授权，本轮未自动继续。
