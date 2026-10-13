# 全局重定位仿真评估报告

## 正式左半场批测结果（2026-10-13）

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

## 原始 LiDAR+IMU 路线阶段进展（2026-10-13；正式 120 次尚未运行）

本节记录一条独立的原始传感器路线：定位器直接接收瞬时 LiDAR 点云和 IMU，利用 IMU 重力/陀螺初始化和局部 6D GICP 估计静置自转运动，再执行全局候选检索和停稳扫描确认。配置明确使用 `raw_scan_model: instantaneous`；点云没有逐点采样时间，因此这批数据不能证明 timed-scan 去畸变效果，也没有验证实车力控。Ground truth 仅供离线物理前提检查和按时间戳评分，不进入定位或控制。

原正式基线仍是上文的旧 30 点矩阵：68/120（56.7%）。新的候选 120 矩阵由 28 个旧位置和 p30、p31 两个替换位置组成；旧矩阵中 p17、p29 的物理前提检查不通过，因此新旧整体成功率不是严格同矩阵对照。新矩阵已完成 120/120 静置真值物理预检，结果为全通过；该预检不测定位成功率。证据在 [静态预检 summary](../tools/global_relocalization/data/raw_left120_static_precheck_20261013/summary.json) 和 [固定矩阵](../tools/global_relocalization/config/trials_left_physical_120.json)。正式 120 次定位批测尚未启动，95% 目标尚无正式结果。

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
