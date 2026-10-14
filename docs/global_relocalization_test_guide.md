# Raw LiDAR + IMU 全局重定位：完整复现与批测指南

**当前已暂停以便关机。**冻结版本、验证证据、完整恢复顺序和仿真清理状态见[恢复检查点](global_relocalization_resume.md)。没有在这次暂停前启动新版本性能对照、p16×3 或 15×3 回归。

## 当前修复周期（2026-10-14）

第一个 53-key 鲁棒 profile 的 15×3 回归已在 round 2 超时后中止：planned 45、executed 30、严格正确 29、超时 1、未尝试 15，round 3 未启动。`passed_45_of_45=false`，该结果不是 45/45，也不代表普遍成功率。完整状态和 round links 见 [仿真结果报告顶部](global_relocalization_simulation.md)；原始记录见 [regression manifest](../tools/global_relocalization/data/robust_correctness_fresh_20261014T101242Z_87dd84d4/regression45/regression_manifest.json)、[round 1](../tools/global_relocalization/data/robust_correctness_fresh_20261014T101242Z_87dd84d4/regression45/round_01/results.summary.json)、[round 2](../tools/global_relocalization/data/robust_correctness_fresh_20261014T101242Z_87dd84d4/regression45/round_02/results.summary.json) 和 [p16 timeout](../tools/global_relocalization/data/robust_correctness_fresh_20261014T101242Z_87dd84d4/p16_timeout_diagnosis.json)。下文的旧 120 次和固定 15 次属于历史 profile/矩阵。

冻结目标仍是相同 15 个 cell 连续三轮，且每个输出完整 3D ≤0.10 m、SO(3) ≤2°。这只评估该冻结回归集，不代表普遍 100%。15 秒内正确定位比例另行报告，计时取原 action goal 的 `elapsed_sec`，内部恢复不重置计时。candidate-workers=2 并行版本现已通过 36 个 build/test checks 并冻结；需在 fresh session 完成相同输入对照、p16 三次验证后，再以新的输出目录重新完成三轮。保持 server 60 秒超时。timed-scan、Point-LIO 地图/速度路线、60 分钟长稳及 120 次稳定性评估仍暂停。

action-level recovery 结果要再经完整姿态评分分类：正确姿态且 `recovery_attempts=0` 为首次正确定位，正确姿态且 `recovery_attempts=1` 为一次内部恢复后正确定位；valid pose 超出完整门限、但 action 返回 success 时列为错误接受；缺姿态分数列为未评分 action 成功，action failure/timeout 独立计数。旧 ledger 没有恢复计数时保留 unknown，不推断首次正确定位。`registration_attempts` 只描述 terminal snapshot 的 latest processed scan，不能代替 action 全程累计的 `recovery_attempts`。`run_id`、`repetition`、冻结的 `trial_id` 和 `source_trial_index` 区分三轮；评分行必须带相同 `run_id`、`repetition`，只按唯一 action stamp 联接，混入旧轮、未标记或多余评分会被拒绝。诊断录制进程单列资源，不并入算法组；RSS 以 MiB 展示，CPU 100% 表示占满一个逻辑核。

此次首轮的 29/30 已评分结果保留在链接所示目录；p16 一次额外诊断 action 54.153 秒严格评分通过（3D 3.151 mm、SO(3) 0.03831°），不并入回归分母。新并行搜索版本和后续完整 45 项回归须另建冻结快照与 session，不覆盖或拼接此首轮数据。

更新：2026-10-14。本轮已按本指南完成物理预检、固定 5 次、拓展 10 次和正式 120 次。正式批测结果为 58/120（48.33%），未达到 ≥114/120 验收目标；成功姿态精度、耗时、CPU/RSS 与失败分析见 [正式批测报告](../tools/global_relocalization/data/session_20261013T160806Z_0f2b982a/formal_raw120_run01/analysis.md)。这是一项未通过验收的实测结果，不能用成功样本的高精度掩盖整体成功率。

## 1. 上一版测试记录（历史 raw_imu_left_half profile）

本节记录的是旧 43-key `raw_imu_left_half.yaml` 路线及原正式 120 矩阵，不是上方首个 53-key 鲁棒 profile 的回归结论，也不是正在验证的新候选并行版本。

- 本轮静置物理预检覆盖 120/120，均有效；正式仿真 audit 通过，运行时参数 43/43 键与预期一致。
- 最终版本固定 5 次预检为 5/5；旧失败点导向的 10 次预检为 8/10，p26 两个航向失败。该 10 次是按历史失败选择的诊断集，不与正式 120 次合并。
- 正式 raw LiDAR + IMU 批测已完成 120/120，58 次成功、62 次失败；51 次失败与 raw 前端队列丢帧同时出现。请优先查看 [完整分析](../tools/global_relocalization/data/session_20261013T160806Z_0f2b982a/formal_raw120_run01/analysis.md) 和 [完整性核验](../tools/global_relocalization/data/session_20261013T160806Z_0f2b982a/formal_integrity_check.json)。

- 最终 Release 构建完成，耗时 51.4 s；C++/ROS CTest 3/3 target、17 case 通过，耗时 11.73 s。包含 9 个 core、4 个 raw 数学、4 个 ROS 合约 case。
- scorer 的 21 个测试通过，包含完整 3D/SO(3) 门限、严格动作时间戳联接、失败分母以及实际 raw frontend 状态字段归一化。
- 原 raw 5 次试验为 3/5，存在 IMU/scan 接收缺口。源 bag 的 IMU 最大间隔 10 ms、LiDAR 最大间隔 100 ms，证明源流正常。修复接收链后，同 5 次为 4/5；剩余 p06 为候选检索失败，四个成功输出均通过完整姿态评分。**这两组均发生在最终检索修复之前。**
- p06 的近区数据库视点距起点约 0.312 m，ring rank 5、Scan Context rank 16、yaw 初值 0。旧 SC 前 10 截断了该视点。最终 raw 检索对固定左半场全部 272 视点评分，取 SC 前 10 与 ring 前 5 的去重并集，最多 15 个候选；质量、半场和三帧独立确认门限保持不变。
- 同一冻结 query + 三个独立停稳扫描离线重放：baseline 10 候选失败，15.386 s；covered 15 候选通过，20.443 s。后者 GICP 19.292 s、确认约 0.0208 s、fitness 0.0282364、inlier 1.0。此输出是 `map_from_reference` 校正，缺少冻结 current-base 元数据，**不是完整最终 base 姿态精度验证**。

已归档的 [最终修复证据与 SHA manifest](../tools/global_relocalization/data/coverage_fix_20261013/archive_manifest.json)、[构建日志](../tools/global_relocalization/data/coverage_fix_20261013/raw_core_coverage_build.log)、[CTest 日志](../tools/global_relocalization/data/coverage_fix_20261013/raw_core_coverage_ctest.log)、[离线对照](../tools/global_relocalization/data/coverage_fix_20261013/raw_p06_offline_comparison.json) 均保留；不依赖 `/tmp`。

最终版本 SHA-256：

| 输入/程序 | SHA-256 |
|---|---|
| raw profile（43 键） | `2c20576e65cf72ef3bce7600b2bb329687daff656956173a901b134a4918359e` |
| node | `7d9d91cf8f270f59dd10a599eb3530fc97e88030b6613ae1581e0a154546335c` |
| core 库 | `ca7747e84b07d8abef8529bcd77d14089e68912f07326bd38267ec065f0f29ca` |
| offline query 工具 | `b6616bfebdc05b850f68ef25dcffce83123e93e00682f5318d6f27b68d6f4b89` |
| 固定地图 | `2b9e08cda8a63ade7ec3e153fe56363a4effd6566ed4a066c764b369abda0570` |
| 物理有效 120 次矩阵 | `3c3332f4b932ce89f34d3273106396ce5457ddf3f238730f31ae18aff946356c` |

## 2. 环境与边界

在当前 Linux/WSL2 checkout 操作，使用 ROS 2 Jazzy、Gazebo Harmonic、Bullet Featherstone 后端。需要 `colcon`、CMake/G++、PCL、Eigen、OpenSSL、yaml-cpp、nlohmann-json、small_gicp，以及 ROS 的 rclcpp/rclpy、TF、action、ros_gz、xacro；Python 需要 numpy/scipy/psutil/PyYAML，绘图才需要 matplotlib，诊断录包需要 rosbag2/mcap。

`environment.bash` 依次加载 `/opt/ros/jazzy`、外部依赖安装、本 checkout 安装和隔离 `.build/ros/install`。外部依赖默认为 `/home/zhang/ros_ws_jazzy/sentry_navgation_RM27/install`，换机器应设置 `GLOBAL_RELOCALIZATION_DEPENDENCY_INSTALL` 为已有依赖安装路径。Gazebo 世界/机器人包必须可发现。`build.bash` 使用已有 small_gicp 源码缓存，固定 revision 为 `fa0cfc983417c044b6036f71dfa448038bb9c820`；缓存不存在时先准备该 revision 的源码缓存。

本流程只运行独立仿真、raw 定位与评估，不启动 Point-LIO、Nav2、RViz/GUI 或实车，也不修改既有 dirty 子模块。raw 只支持 `instantaneous` 点云；带逐点时间的 timed raw 未实现，不可套用此结果到实车。

GT 只用于静置物理前提核验与输出评分。定位器只接 raw LiDAR、IMU 和固定 TF，goal 无真值初始姿态，候选只受固定 `x < 0` 先验与传感器数据影响。原 120 次 Point-LIO 结果仍独立保留，不与新矩阵或新算法统计合并。

以下命令是**未来复现步骤**。本轮实际运行的冻结输入、结果和逐试验证据均保存在报告链接所指向的 session 目录；正式复跑使用新的输出目录，不覆盖这些记录。

## 3. 构建和单元测试（启动批测前完成）

```bash
cd /home/zhang/navi/sentry_navgation_RM27
export RELOC_ROOT="$PWD"
PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
  MAKEFLAGS=-j2 tools/global_relocalization/build.bash
source tools/global_relocalization/environment.bash
ctest --test-dir tools/global_relocalization/.build/ros/build/global_relocalization \
  --output-on-failure
/usr/bin/python3 src/global_relocalization/test/test_evaluate_pose.py
tools/global_relocalization/prepare_simulation.bash
```

Linux PATH 避免 WSL 的 Windows 路径扫描拖慢构建。首次缺依赖的构建时间会更长。`prepare_simulation.bash` 只生成隔离机器人 URDF/controller manifest，用 Gazebo 内建 VelocityControl 替换该独立描述中的旧驱动控制器。正式测性能时不运行构建、测试、离线重放或参数调整。

## 4. 建立全新 headless 仿真 session

每次选一个未占用 ROS domain，并生成唯一 Gazebo partition。示例用 91；先阅读本机进程/ROS graph，确认没有其他实验使用该 domain，不能按旧 PID 清理进程。所有后续终端必须 source 同一 session 环境文件。

```bash
export ROS_DOMAIN_ID=91
export RELOC_SESSION="$RELOC_ROOT/tools/global_relocalization/data/session_$(date -u +%Y%m%dT%H%M%SZ)_$RANDOM"
/usr/bin/python3 - <<'PY'
import json, os, pathlib, subprocess, uuid, psutil
root=pathlib.Path(os.environ['RELOC_ROOT'])
folder=pathlib.Path(os.environ['RELOC_SESSION']); folder.mkdir(exist_ok=False)
partition='global_relocalization_'+os.environ['ROS_DOMAIN_ID']+'_'+uuid.uuid4().hex
env={**os.environ,'GZ_PARTITION':partition}
command=['bash',str(root/'tools/global_relocalization/start_simulation.bash')]
with (folder/'simulation.log').open('w') as log:
    process=subprocess.Popen(command,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
record={'owner_pgid':process.pid,'create_time':psutil.Process(process.pid).create_time(),
        'ros_domain_id':env['ROS_DOMAIN_ID'],'gz_partition':partition,'command':command}
(folder/'session.json').write_text(json.dumps(record,indent=2)+'\n')
(folder/'environment.bash').write_text('export ROS_DOMAIN_ID='+env['ROS_DOMAIN_ID']+'\nexport GZ_PARTITION='+partition+'\n')
print(folder)
PY
source "$RELOC_SESSION/environment.bash"
/usr/bin/python3 tools/global_relocalization/audit_simulation.py \
  --output "$RELOC_SESSION/simulation_audit.json" --duration 6
```

默认 `gui=false`，不另开 observer。audit 必须 `passed=true`：`/clock`、GT、raw LiDAR、raw IMU 各有且仅有一个 publisher，时间戳推进且不回退。启动初期未完成 spawn 时失败，应等待日志中实体创建成功，再保存**新文件名**重做 audit；通过后传相应文件给 batch。audit 使用 best-effort 观察流，样本数不能作为源丢包率测量。

## 5. 固定矩阵与独立物理预检

```bash
export RELOC_MAP="$RELOC_ROOT/tools/global_relocalization/data/station_map_20261013/map.pcd"
export RELOC_MAP_MANIFEST="$RELOC_ROOT/tools/global_relocalization/data/station_map_20261013/manifest.json"
export RELOC_PARAMS="$RELOC_ROOT/src/global_relocalization/config/raw_imu_left_half.yaml"
export RELOC_PHYS="$RELOC_SESSION/physical_precheck"
/usr/bin/python3 tools/global_relocalization/validate_static_candidates.py \
  --candidates tools/global_relocalization/config/trials_left_physical_120.json \
  --validate-all --output "$RELOC_PHYS" --map-manifest "$RELOC_MAP_MANIFEST"
/usr/bin/python3 - <<'PY'
import json,os,pathlib
out=pathlib.Path(os.environ['RELOC_PHYS'])
summary=json.loads((out/'summary.json').read_text())
assert summary['observed_trials']==120 and summary['all_physical_valid']
assert json.loads((out/'manifest.json').read_text())['source_input_hashes_verified_at_end']
PY
```

预检脚本现在自动写入 controller/map/matrix 及前后字节hash核验的 manifest，不再手工覆盖。已有完整provenance的合法旧预检可复用：源矩阵hash仍匹配、controller/map与所有verified inputs不变，选中trial的ID/index/pose逐项匹配；缺少这些证据的旧格式须重新预检，不能补一个未经核验的手写manifest。

物理预检时不运行定位器/goal；该脚本持续发布零 Twist，每个固定 pose 静置 3 ROS s、观察 3 ROS s，总体约 12–15 min（RTF≈1）。120 次必须完整保留：至少 50 GT 样本/1 s，平移≤0.10 m、绝对 roll/pitch≤2°、高度范围≤0.02 m、落点 XY 误差≤0.10 m。失败应停止评估前提检查，不可依据定位成功率删点、换点或悄悄缩分母。

已有 [全 120 静置预检](../tools/global_relocalization/data/raw_left120_static_precheck_20261013/summary.json) 为 120/120 物理有效，矩阵/hash/controller 已核。换环境/控制器后应重新做上述预检。新矩阵保留原 28 个位置，仅以同 seed 的 p30/p31 替代原物理异常 p17/p29；旧 120 次及其异常失败仍原样保留。

## 6. 43 键 raw profile 与实际参数审计

完整参数在 [raw_imu_left_half.yaml](../src/global_relocalization/config/raw_imu_left_half.yaml)。关键设置：`input_mode=raw_imu`，raw topics 为 `/livox/lidar`、`/livox/imu`，`search_side=negative_x`，`side_boundary_x=0`，`top_k=10`，`ring_key_candidates=5`，`retrieval_all_views=true`，`diagnostic_output_dir=''`。后者必须为空，预检/正式均不 dump。

严格质量门限包括 fitness≤0.2、inlier≥0.65、歧义检查、三个不同 timestamp 停稳扫描相互确认。raw 接收为 reliable、IMU depth 200/cloud depth 20；35 ms IMU/350 ms scan 缺口门限未放宽。100 ms 局部计算预算用于诊断，不把超预算计成质量拒绝。

batch 启动算法后自动调用 `audit_runtime_parameters.py`，保存 `runtime_parameters.json/log`；对 **43 个 YAML 键实际 list/get**，任何未声明/值不符都拒绝进入正常试验。它也记录订阅 graph，raw estimator 不能订阅 GT 或 `/odometry`。本轮正式批次的 43 键审计通过，完整记录在 [runtime_parameters.json](../tools/global_relocalization/data/session_20261013T160806Z_0f2b982a/formal_raw120_run01/runtime_parameters.json)。

若单独启动算法做参数排查，可以执行下列命令；完成后 Ctrl-C 该 launch，确认其退出，才运行 batch，避免双算法：

```bash
# 终端 A（同 session 环境）：
ros2 launch "$RELOC_ROOT/src/global_relocalization/launch/raw_imu.launch.py" \
  map_path:="$RELOC_MAP" manifest_path:="$RELOC_MAP_MANIFEST" params_file:="$RELOC_PARAMS"
# 终端 B（同 session 环境）：
/usr/bin/python3 tools/global_relocalization/audit_runtime_parameters.py \
  --params "$RELOC_PARAMS" --output "$RELOC_SESSION/parameters_manual.json"
```

## 7. 先跑固定 5 次，再跑固定拓展 10 次

本轮已经执行以下两组。固定 5 保留 p06，拓展 10 根据旧 Point-LIO ledger 的失败选择 p11/p12/p16/p24 各两个航向与 p26 的旧平移拒绝/超时航向；不按新 raw 成功选点。固定 5 为 5/5；拓展 10 为 8/10。其合并集合经过旧失败记录定向挑选，只用于发现回归，不是代表性成功率样本。

```bash
tools/global_relocalization/run_batch.bash \
  --map "$RELOC_MAP" --manifest "$RELOC_MAP_MANIFEST" \
  --params "$RELOC_PARAMS" --input-mode raw_imu \
  --physical-precheck "$RELOC_PHYS" --simulation-audit "$RELOC_SESSION/simulation_audit.json" \
  --trials tools/global_relocalization/config/pilot_raw_failure_5.json \
  --output "$RELOC_SESSION/pilot_failure5"
tools/global_relocalization/run_batch.bash \
  --map "$RELOC_MAP" --manifest "$RELOC_MAP_MANIFEST" \
  --params "$RELOC_PARAMS" --input-mode raw_imu \
  --physical-precheck "$RELOC_PHYS" --simulation-audit "$RELOC_SESSION/simulation_audit.json" \
  --trials tools/global_relocalization/config/pilot_raw_legacy_failure_10.json \
  --output "$RELOC_SESSION/pilot_legacy_failure10"
```

复跑时预计 5 次约 3–5 min、10 次约 6–10 min，具体受候选/GICP耗时影响。检查每个失败、timeout、缺分与数据拒绝，而非只看进程退出码；聚合成功也不代表定位全通过。以下为复现命令；本轮结果见 [pilot_fixed5 summary](../tools/global_relocalization/data/session_20261013T160806Z_0f2b982a/pilot_fixed5_run01/results.summary.json) 与 [pilot_legacy_failure10 summary](../tools/global_relocalization/data/session_20261013T160806Z_0f2b982a/pilot_legacy_failure10_run01/results.summary.json)。

## 8. 冻结后正式 120 次（不录包/不 dump）

本轮正式批次已经完成，结果为 58/120（48.33%），低于验收线。失败以 51 次 raw scan coverage gap 为主；这些失败窗口均记录到 raw 前端队列丢帧。详见 [正式分析与性能数据](../tools/global_relocalization/data/session_20261013T160806Z_0f2b982a/formal_raw120_run01/analysis.md)。下方命令用于以新 session、新输出目录复现，不能把本轮失败重命名为成功或覆盖其原始数据。

```bash
tools/global_relocalization/run_batch.bash \
  --map "$RELOC_MAP" --manifest "$RELOC_MAP_MANIFEST" \
  --params "$RELOC_PARAMS" --input-mode raw_imu \
  --physical-precheck "$RELOC_PHYS" --simulation-audit "$RELOC_SESSION/simulation_audit.json" \
  --trials tools/global_relocalization/config/trials_left_physical_120.json \
  --output "$RELOC_SESSION/formal_raw120"
```

不要传 `--limit`、`--record-trials`；保持诊断输出目录为空。正式期间不编译、调参、运行离线重放或绘图，不启 GUI。预计约 1–1.5 h，失败/超时可能增加时长。每完成 5–10 次查看累计 ledger，报告完成数/成功数/失败类型；不能提前把未完成部分计作成功。

batch 保存来源 hash、源码快照、node/core 二进制快照、地图 manifest、矩阵/profile、预检/audit、机器人 URDF/controller manifest，以及同 session Gazebo 实际加载的 VelocityControl/Bullet 库路径/hash/二进制。地图使用原冻结文件并校验 SHA；正式期间不改任何冻结输入。

验收目标为 **≥114/120（≥95%）**，且每个成功输出同时满足 **完整 3D≤0.10 m、SO(3)≤2°**。失败、timeout、动作成功却缺有效评分都保留在 120 分母。XY/yaw 只是诊断，不能替代完整门限。

## 9. 评分、计时、资源与输出

每组目录包含 `trials.jsonl`、`pose_errors.jsonl`、`results.summary.json`、`results.trials.csv`、`results.success_cohort.json`、manifest 和各 trial 原始 JSON/log。GT 通过固定 `T_map_world=identity` 对齐；平移插值/最短弧 SLERP，不外推，不在线拟合地图对齐。只用嵌套整数 `action.pose_stamp_ns` 与唯一评分记录精确联接，失败不复用旧 pose/score。

batch 自动聚合并生成独立成功 cohort。若需批后重聚合，应保留原文件，使用新 prefix；正式完全结束后才绘图：

```bash
/usr/bin/python3 src/global_relocalization/scripts/evaluate_pose.py --aggregate \
  --trials "$RELOC_SESSION/formal_raw120/trials.jsonl" \
  --pose-errors "$RELOC_SESSION/formal_raw120/pose_errors.jsonl" \
  --output-prefix "$RELOC_SESSION/formal_raw120/review" --expected-trials 120 --plots
/usr/bin/python3 tools/global_relocalization/summarize_success_cohort.py \
  --report "$RELOC_SESSION/formal_raw120/review.summary.json" \
  --output "$RELOC_SESSION/formal_raw120/review.success_cohort.json"
```

同时报告全分母成功率，以及成功 pose 的 3D/SO(3) mean/RMSE/P95/max和各轴 bias/RMSE。计时要给全部 trial 与严格成功 cohort：static、sweep、stop-settle、retrieval、GICP、confirmation、总动作/采集/计算墙钟；未测到的 ROS 段时间保持 null，不用零补。原始 terminal 状态还包含候选数量、SC/ring 来源/rank/seed、raw DAQ间隔/拒绝/队列与局部质量。

CPU 为 `(user+system)/wall×100`，100% 为一个逻辑核，多线程可超过100%；RSS 为窗口采样峰值，RTF 为动作 ROS 时长/墙钟。raw frontend 在算法 PID 内，包含在 algorithm CPU/RSS，不重复相加；Point-LIO 未启用，其成本为 null。Gazebo、trial evaluator 客户端、诊断 recorder 分开计。正式无 recorder 样本。条件成功精度/时长不能替代总体成功率。

## 10. 失败诊断与离线重放

本轮 120 次中有 51 次 raw scan coverage gap；每次均记录 raw cloud 队列丢帧，累计 480 帧。前端处理扫描的 stamp 间隔保护是 350 ms，ROS 接收消息的 stamp 间隔仍为 100 ms。后续排查先记录回调、长度为 4 的 pending queue、worker 处理时长和实际处理 stamp；保留门限，不以放宽门限隐藏积压。完整证据见 [正式批测分析](../tools/global_relocalization/data/session_20261013T160806Z_0f2b982a/formal_raw120_run01/analysis.md)。

- readiness：先看逐源 unmet、计数/freshness、clock/server；raw 不等待 `/odometry`。检查同 domain/partition、实际 topics/frame 和唯一 publisher。
- IMU/scan gap：先区分源 header gap、callback墙钟延迟、队列丢弃与处理时长。保留35/350ms阈值，查接收/背压，不用放宽门限掩盖漏帧。
- no candidate：看 `retrieval_scored_views`、`retrieval_candidate_count`（最多15）、`retrieval_ring_added`、hypothesis rank/source/seed/fitness/inlier。检查召回覆盖和初值，不能用 GT 选择候选或降低质量门限来制造通过。
- confirmation：检查是否有3个不同 stamp 的停稳扫描、姿态/质量是否相互一致；不要回用累积旋转 query 冒充独立确认。
- timeout/缺分：检查 readiness/client/server/outer timeout来源、stage walltime和exact stamp join/GT插值括区。失败继续计入分母。

仅诊断时，复制 profile，设置绝对 `diagnostic_output_dir`；用固定 [p06 诊断矩阵](../tools/global_relocalization/config/diagnostic_raw_p06_1.json)，可给 batch 加 `--record-trials p06_yaw0`。raw replay bag 只含 raw LiDAR/IMU/TF静态/clock，GT另录到 scoring bag；诊断输出为 `search_N/query.pcd`、三个 independent PCD与完整 view rank JSON。不要在正式测性能时启用这些选项。

已有 query 可离线对照（无 ROS/控制/GT输入；使用新输出路径）：

```bash
source tools/global_relocalization/environment.bash
tools/global_relocalization/.build/ros/install/global_relocalization/lib/global_relocalization/global_relocalization_offline_query \
  "$RELOC_MAP" \
  tools/global_relocalization/data/diagnostic_raw_p06_20261013_48dc198665e4/estimator_diagnostics/search_1 \
  "$RELOC_PARAMS" "$RELOC_SESSION/offline_baseline.json" baseline
tools/global_relocalization/.build/ros/install/global_relocalization/lib/global_relocalization/global_relocalization_offline_query \
  "$RELOC_MAP" \
  tools/global_relocalization/data/diagnostic_raw_p06_20261013_48dc198665e4/estimator_diagnostics/search_1 \
  "$RELOC_PARAMS" "$RELOC_SESSION/offline_covered.json" covered
```

离线结果仅证明冻结 query 的召回/配准/确认机制，不替代真实动作评分或正式性能。

## 11. 正常停止与只清理自有进程

batch 正常结束会停其算法/evaluator/client/recorder；中途停止用 Ctrl-C，让 `finally` 清理。先等它们退出，再零速停仿真。所有命令在本 session 环境内操作，不用 `pkill` 或旧 session PGID。

```bash
source "$RELOC_SESSION/environment.bash"
timeout 15 ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{}'
/usr/bin/python3 - <<'PY'
import json,os,pathlib,psutil,signal,time
s=json.loads((pathlib.Path(os.environ['RELOC_SESSION'])/'session.json').read_text())
pid=s['owner_pgid']; p=psutil.Process(pid); e=p.environ()
assert os.getpgid(pid)==pid
assert e.get('ROS_DOMAIN_ID')==s['ros_domain_id'] and e.get('GZ_PARTITION')==s['gz_partition']
cmd=' '.join(p.cmdline())
expected_launch=str(pathlib.Path(os.environ['RELOC_ROOT'])/'src/global_relocalization/launch/simulation.launch.py')
assert expected_launch in cmd and 'robot_velocity.urdf' in cmd
create_time_delta_sec=p.create_time()-s['create_time']
def owned():
    members=[]
    for q in psutil.process_iter(['pid','status']):
        try:
            if os.getpgid(q.pid)!=pid or q.info['status']==psutil.STATUS_ZOMBIE: continue
            qe=q.environ()
            assert qe.get('ROS_DOMAIN_ID')==s['ros_domain_id']
            assert qe.get('GZ_PARTITION')==s['gz_partition']
            members.append(q.pid)
        except (psutil.NoSuchProcess,ProcessLookupError): pass
    return members
for sig,seconds in [(signal.SIGINT,8),(signal.SIGTERM,4),(signal.SIGKILL,1)]:
    if not owned(): break
    os.killpg(pid,sig); deadline=time.monotonic()+seconds
    while time.monotonic()<deadline and owned(): time.sleep(.2)
assert not owned()
print(f'Owned simulation stopped; artifacts preserved; launch create-time delta={create_time_delta_sec:.3f}s')
PY
```

身份核验失败应停止清理，先只读确认实际进程；不能因 PID 相同就发送信号。2026-10-14 本轮对 session PGID 2637 发布零 Twist，并核验 ROS domain 与唯一 partition 后停止了专用仿真进程；仅 Gazebo server 在 SIGINT/SIGTERM 后仍存活，随后对已验证的 PID 2719 发送 SIGKILL。记录见 [本轮 cleanup.json](../tools/global_relocalization/data/session_20261013T160806Z_0f2b982a/cleanup.json)。更早的独立 session 清理记录仍保留在 [旧 cleanup.json](../tools/global_relocalization/data/raw_session_20261013_48dc198665e4/cleanup.json)。


## 12. 2026-10-14 前端修复固定15次验证

本轮15次是诊断样本，不是正式120次性能评测。只验证冻结的 `config/pilot_raw_frontend_fix_15.json`，保留原 physical120 的 pose、ID、index 与源文件 SHA；五个原 pilot、六个 formal 局部 GICP 失败、一个 motion gate、两个 legacy failure 和首个 coverage failure 共15次。选择在新结果之前冻结，不按新成功率筛选。旧120次与历史 artifact 均保留。本轮现已完成一次实际15次验证：13/15成功，p13_yaw3 convergence retry一次至30iterations仍拒绝；p26_yaw2 linear speed0.36215454m/s超过原0.35门限而拒绝。没有通过重跑/换点改变分母。详见[本轮analysis](../tools/global_relocalization/data/session_frontend_fix15_20261013T181313Z_64ccf218/pilot_raw_frontend_fix15_run01/analysis.md)、[全15 summary](../tools/global_relocalization/data/session_frontend_fix15_20261013T181313Z_64ccf218/pilot_raw_frontend_fix15_run01/results.summary.json)及[成功13 cohort](../tools/global_relocalization/data/session_frontend_fix15_20261013T181313Z_64ccf218/pilot_raw_frontend_fix15_run01/results.success_cohort.json)。

所有代码冻结、核心与评估单测通过之后，只能在唯一 ROS domain、唯一 GZ partition 和本轮记录的 owner PGID 下运行 headless 仿真。先对这15个 pose 做实际物理预检，不能套用旧120预检而忽略 controller/map/matrix 身份；不重新做120次物理预检。沿用前述 session/environment 初始化命令，随后运行：

```bash
source "$RELOC_SESSION/environment.bash"
/usr/bin/python3 tools/global_relocalization/validate_static_candidates.py \
  --candidates tools/global_relocalization/config/pilot_raw_frontend_fix_15.json \
  --output "$RELOC_SESSION/physical_precheck15" --map-manifest "$RELOC_MAP_MANIFEST" --validate-all
/usr/bin/python3 tools/global_relocalization/audit_simulation.py \
  --output "$RELOC_SESSION/simulation_audit.json" --duration 6
/usr/bin/python3 tools/global_relocalization/batch_run.py \
  --map "$RELOC_MAP" --manifest "$RELOC_MAP_MANIFEST" \
  --params src/global_relocalization/config/raw_imu_left_half.yaml --input-mode raw_imu \
  --physical-precheck "$RELOC_SESSION/physical_precheck15" \
  --simulation-audit "$RELOC_SESSION/simulation_audit.json" \
  --trials tools/global_relocalization/config/pilot_raw_frontend_fix_15.json \
  --output "$RELOC_SESSION/pilot_raw_frontend_fix15_run01"
```

`RELOC_MAP_MANIFEST`指冻结 map manifest，不是 controller manifest。runtime audit 仍验证实际 profile 的全部43 keys，并检查 estimator 不订阅GT。simulation audit要求单发布者、时间戳递增和最近回调年龄小于1.5秒。可用 `passive_receipt_probe.py --output "$RELOC_SESSION/passive_receipt.jsonl"` 在本session的独立owned进程组记录源LiDAR接收与node状态对照；它使用可靠KEEP_LAST depth64 reader并记录QoS，按首次观察的run_id边界分段计数/最大gap，另留session累计。它没有GT订阅、command publisher或点云落盘，其CPU/RSS归入diagnostic recorder，不能当作正式性能。观察者与node共同出现gap只说明共同输送异常，上游具体原因仍未知；不能直接归因源发布端。

reset helper记录每次开始/结束、returncode、stdout/stderr、CLI10秒与outer15秒超时分类。只有超时允许同pose重试一次，Boolean false不重试；同planned trial始终只保留一条结果，`reset_attempts`完整保留首试失败，manifest的`reset_summary`分开报告首试成功与重试恢复，不能将恢复计作首试成功。日志落盘失败会终止该reset并保留原服务错误。完成后使用第11节身份核验与仅owned group清理流程，验证无本轮owned残留。


本轮build/core27、evaluator23、harness7共57项checks通过；实际物理预检15/15、runtime43keys全匹配。13输出严格3D/SO3全通过（位置RMSE0.001643m、最大0.003179m；SO3最大0.113658°）。全部15terminal queue drop/overflow为0、worker_error为空，queue高水位实测1；与独立可靠observer的source stamp gap均0.1s、callback gap最大约0.122s。成功13次action wall平均33.923s、p95 45.677s、最大49.351s；这些是诊断样本数据，不能作为正式120次性能。

本轮未复现旧120后半程长时0.7–1.0s回调burst，受控FIFO/burst停车由测试覆盖，不能宣称长期输送问题已完全修复。停止probe、发布零Twist并完成owned group清理，remaining owned=0，详见[cleanup](../tools/global_relocalization/data/session_frontend_fix15_20261013T181313Z_64ccf218/cleanup.json)与[integrity](../tools/global_relocalization/data/session_frontend_fix15_20261013T181313Z_64ccf218/integrity_check.json)。没有执行新120、GUI、实车或导航。

## 13. 当前前端恢复逻辑：诊断复现、20 次离线重放与 45 次正确率回归

以下命令记录首个 53-key 鲁棒 profile 的复现方式；该版的45项运行已经停止于round 2，不能据此声称45/45通过。后续候选并行版本须另建 frozen build/profile/session 和输出目录，不能覆盖此首轮 ledger。先按第 4 节创建全新的隔离仿真 session，并按第 12 节要求对固定15个cell重新做物理预检和simulation audit。本版参数文件固定为：

```bash
export RELOC_PARAMS="$RELOC_ROOT/src/global_relocalization/config/raw_imu_robust_left_half.yaml"
```

runtime audit 应精确核对本版53个参数键；server wall timeout 保持60秒。诊断复现和正确率回归分别使用新输出目录，不覆盖旧 ledger 或诊断输入。

### 单次诊断复现并保存原始输入

以下用先前固定集中的 `p13_yaw3` 作为可复现的诊断例子。它从冻结 15 矩阵按原 ID、原 pose 和 `source_trial_index` 派生一个单 cell 清单；物理有效性仍从同一固定 15 预检核对。诊断 profile 只设置 `raw_diagnostic_directory`，`diagnostic_output_dir` 继续留空：前者保存按扫描顺序编号的输入 `.pcd` 和包含精确 IMU/参数的 `.json`；后者是全局候选搜索诊断，和本次前端数值重放无关。

```bash
source "$RELOC_SESSION/environment.bash"
export RELOC_DIAG_TRIALS="$RELOC_SESSION/diagnostic_p13_yaw3.json"
export RELOC_DIAG_PARAMS="$RELOC_SESSION/raw_diagnostic.yaml"
export RELOC_DIAG_OUT="$RELOC_SESSION/raw_diagnostic_p13_yaw3"
export RELOC_CAPTURE_ROOT="$RELOC_SESSION/raw_diagnostic_capture"
/usr/bin/python3 - <<'PY'
import hashlib, json, os, pathlib, yaml
root=pathlib.Path(os.environ['RELOC_ROOT'])
src=root/'tools/global_relocalization/config/pilot_raw_frontend_fix_15.json'
data=json.loads(src.read_text())
trial=next(t for t in data['trials'] if t['trial_id']=='p13_yaw3')
one={'seed':data['seed'],'count':1,'search_prior':data['search_prior'],'trials':[trial],
     'purpose':'single-cell raw frontend diagnostic reproduction; not a correctness cohort',
     'source_matrix':str(src),'source_matrix_sha256':hashlib.sha256(src.read_bytes()).hexdigest()}
pathlib.Path(os.environ['RELOC_DIAG_TRIALS']).write_text(json.dumps(one,indent=2)+'\n')
profile=yaml.safe_load(pathlib.Path(os.environ['RELOC_PARAMS']).read_text())
params=profile['global_relocalization']['ros__parameters']
params['diagnostic_output_dir']=''
params['raw_diagnostic_directory']=os.environ['RELOC_CAPTURE_ROOT']
pathlib.Path(os.environ['RELOC_DIAG_PARAMS']).write_text(yaml.safe_dump(profile,sort_keys=False))
PY
test ! -e "$RELOC_DIAG_OUT"
test ! -e "$RELOC_CAPTURE_ROOT"
tools/global_relocalization/run_batch.bash \
  --map "$RELOC_MAP" --manifest "$RELOC_MAP_MANIFEST" \
  --params "$RELOC_DIAG_PARAMS" --input-mode raw_imu \
  --physical-precheck "$RELOC_SESSION/physical_precheck15" \
  --simulation-audit "$RELOC_SESSION/simulation_audit.json" \
  --trials "$RELOC_DIAG_TRIALS" --output "$RELOC_DIAG_OUT"
```

原始 scan dump 在算法 node 内生成；该次的节点 CPU/RSS 和 I/O 包含落盘开销，只用于复现诊断，不能用作算法性能数据。batch 中若另启 raw bag recorder，它会独立标为 `diagnostic_recorder_process`，资源不得并入算法统计。确认只生成一个动作目录后，直接用同一份捕获输入做确定性重放：

```bash
mapfile -t RELOC_CAPTURE_DIRS < <(find "$RELOC_CAPTURE_ROOT" -mindepth 1 -maxdepth 1 -type d -name 'action_*' | sort)
test "${#RELOC_CAPTURE_DIRS[@]}" -eq 1
source tools/global_relocalization/environment.bash
source "$RELOC_SESSION/environment.bash"
ros2 run global_relocalization global_relocalization_raw_replay \
  "${RELOC_CAPTURE_DIRS[0]}" 20 > "$RELOC_SESSION/raw_replay20.json"
/usr/bin/python3 - "$RELOC_SESSION/raw_replay20.json" <<'PY'
import json, sys
report=json.load(open(sys.argv[1]))
assert report['numerical_only'] and report['replays_receipt_latency'] is False
assert report['repetitions']==20 and report['bitwise_identical'] is True
assert len(report['runs'])==20
print(json.dumps({'scan_count':report['scan_count'],'repetitions':report['repetitions'],
                  'bitwise_identical':report['bitwise_identical']}))
PY
```

20 次是对同一份按时间顺序保存的 scan/IMU 输入重复运行前端，检查每次接受/拒绝与姿态输出是否逐位一致；它不是 20 次独立仿真、GT 精度评分或传感器接收时延测量。若输出不一致，保留 JSON 和捕获目录用于定位，不得把重放次数计入 45 次 cohort。

### 固定 15 个 cell 的三轮正确率回归

只有在代码、core、evaluator 与 harness 均冻结并通过各自单测后，由唯一仿真操作者在当前 session 执行。profile 的 `diagnostic_output_dir` 和 `raw_diagnostic_directory` 都应为空；45 个动作不启用点云 dump 或 bag recorder。脚本每轮重新启动 evaluator/node，三轮使用相同 15 个 trial ID 和 source index，以 `(run_id, repetition, trial_id, pose_stamp_ns)` 关联评分。任一轮失败即停止，未执行项仍显示为 unattempted：

```bash
source tools/global_relocalization/environment.bash
source "$RELOC_SESSION/environment.bash"
/usr/bin/python3 tools/global_relocalization/regression_45.py \
  --map "$RELOC_MAP" --manifest "$RELOC_MAP_MANIFEST" \
  --params "$RELOC_PARAMS" \
  --trials tools/global_relocalization/config/pilot_raw_frontend_fix_15.json \
  --physical-precheck "$RELOC_SESSION/physical_precheck15" \
  --simulation-audit "$RELOC_SESSION/simulation_audit.json" \
  --output "$RELOC_SESSION/accuracy45_run01"
```

新 session 下使用独立空输出路径；先检查 `test ! -e "$RELOC_SESSION/accuracy45_run01"`，保证失败记录不会覆盖旧目录。每轮固定使用 `config/pilot_raw_frontend_fix_15.json`，运行三轮共45项。

审核 `$RELOC_SESSION/accuracy45_run01/regression_manifest.json` 的 `planned_cells`、`executed_cells`、`not_attempted_cells`、`passed_45_of_45` 和冻结输入末尾哈希；每轮查看 `round_NN/results.summary.json`、完整 `trials.jsonl`、`pose_errors.jsonl`、`results.trials.csv` 与 `results.summary.json`。三维误差或 SO(3) 超门限仍是失败，即使 action 返回成功；失败、timeout 和缺姿态分数保留在原计划 45 分母，缺失 pose/error 显示 null，不从上一轮补分或重复 join。报告 `correct_localization_within_15s` 时同时给出全 cohort 率和成功 action 条件率；它使用从原 goal 发起时累计的 action elapsed，恢复阶段不能重置计时，也不替代 45/45 完整姿态正确率目标。

`45/45` 仅表示此冻结 15-cell 集合在三轮均通过 `3D≤0.10 m` 与 `SO(3)≤2°`，不外推到普遍 100%。本周期不做 timed-scan、Point-LIO 地图/速度比较、60 分钟稳定性或 120 次稳定性测试；只有冻结 45/45 达标后再进入那些后续阶段。CPU 百分比按一核为 100%，RSS 用 MiB；诊断 recorder 的资源单独列示，诊断文件写入期间的算法负荷不作为无诊断的性能基线。
