# Raw LiDAR + IMU 全局重定位：完整复现与批测指南

**当前状态：55-key bounded 固定回归已完成 45/45。**2026-10-14 UTC 的冻结回归已完成，2026-10-19 只复核归档：run id `accuracy45_08792fbbe785`，三轮各 15/15，共 **45/45 严格正确**（3D ≤0.10 m、SO(3) ≤2°），40 项无恢复、5 项一次恢复后正确，失败 action / 错误接受均为 0，15 s 内正确 **0/45**。15 秒目标未达到。45/45 只描述固定理想仿真集合，含恢复的系统通过不代表局部配准 100% 成功。 进度和冻结身份见[恢复检查点](global_relocalization_resume.md)，完整分布见[仿真结果报告](global_relocalization_simulation.md)。

当前 profile 有 55 个参数键、`candidate_workers=2`、server timeout 60 s；profile SHA-256 `1c1bba993a6aae1d9ab9854748839098a31863d1aa943e45e92c1497309db0c0`。冻结 manifest SHA-256 `6a12676661f03b7fdfe846f2d7ee67f3e46b3b185281ecef7971b9db7b2927b4`，installed core/node SHA-256 分别为 `5ac470d46553176b61928400a7672e6963cf2d722ae8aba8cc482e4f65dc592b` 和 `c662e64e61aa3eb3307100d4e20e8f7c8f9225fb255f18dedc068f5ea4674109`。具体使用的源快照与运行时 binary 以[冻结 manifest](../tools/global_relocalization/data/resume_parallel_validation_20261014T123937Z/bounded_frozen_manifest.json)、[live provenance](../tools/global_relocalization/data/resume_parallel_validation_20261014T123937Z/bounded_live_p16_20261014T154541Z/freeze_provenance.json)及每轮 source/binary snapshot 为准；共享 main 的其他 observer/visualization 功能不计入 headless 定位验收。

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

## 13. 历史 53-key 前端恢复逻辑：诊断复现、20 次离线重放与 45 次正确率回归

以下命令记录首个 53-key 鲁棒 profile 的历史复现方式；该版的45项运行已经停止于round 2，不能据此声称45/45通过。这些旧命令和结果不代表当前55-key bounded candidate。旧 53-key ledger 与新版本须分开保存，不能覆盖或拼接。首版参数文件当时固定为：

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

## 14. 当前 55-key bounded 候选：固定回归 45/45

当前冻结参数文件为 `raw_imu_robust_left_half.yaml`，55 个参数键，SHA-256 `1c1bba993a6aae1d9ab9854748839098a31863d1aa943e45e92c1497309db0c0`；`candidate_workers=2`、OpenMP=4，server wall timeout 60 s。精确 source/build/installed binary 身份见[冻结 manifest](../tools/global_relocalization/data/resume_parallel_validation_20261014T123937Z/bounded_frozen_manifest.json)，活动 session 的 core/node 映射见[live freeze provenance](../tools/global_relocalization/data/resume_parallel_validation_20261014T123937Z/bounded_live_p16_20261014T154541Z/freeze_provenance.json)。当前 main 的 SearchObserver/visualization 属于另一项共享工作；验收依据每个 live run 的 source snapshot、binary snapshot、runtime audit 和地图校验，不将 observer 纳入本轮算法计时。

查询路径变更针对远场开销：原逻辑做无界 KD 最近邻搜索后再以距离门限拒绝，新 adapter 在 KD 遍历中使用精确半径剪枝。树结构、tie 规则、距离门限、全量候选、歧义门限和三次独立确认保持一致。与此同时，STOPPING 中任何逐帧运动都会重启完整 2 s wall/ROS 静止窗口；freeze/drain 后再次核验 stage、帧数和稳态；raw Core 至少要求 3 个独立确认帧。

安装候选前完成 44 个 checks，全部通过：raw frontend 13、raw scan worker 8、bounded KD tree 5、core 11、action contract 7。相同 p16 capture 的原查询和 bounded 查询都完成 13 个候选及 3 次确认；593 项数值比较在绝对误差 1e-12 内等价，最大差异 2.22e-15。单次同二进制离线结果 wall 18.79→5.87 s、CPU 74.56→31.46 core-s、峰值 RSS 115.60→115.45 MiB；微基准远区约 5.8–6.0×、近区基本持平。capture 无 GT，因此以上只用于确认搜索路径和离线成本变化，不构成 live 正确率或端到端时延结论。证据见[比较 JSON](../tools/global_relocalization/data/resume_parallel_validation_20261014T123937Z/bounded_validation/comparison.json)与[微基准日志](../tools/global_relocalization/data/resume_parallel_validation_20261014T123937Z/bounded_micro.log)。

2026-10-14 UTC 的冻结回归已完成，2026-10-19 只复核归档：run id `accuracy45_08792fbbe785`，三轮各 15/15，共 **45/45 严格正确**（3D ≤0.10 m、SO(3) ≤2°），40 项无恢复、5 项一次恢复后正确，失败 action / 错误接受均为 0，15 s 内正确 **0/45**。15 秒目标未达到。45/45 只描述固定理想仿真集合，含恢复的系统通过不代表局部配准 100% 成功。

45 项完整 3D 误差均值/RMSE/P95/最大为 **1.165732 / 1.301647 / 2.227641 / 2.846049 mm**；SO(3) 为 **0.029225 / 0.036750 / 0.060048 / 0.122663°**。action elapsed wall 均值/P95/最大为 **27.947963 / 32.315618 / 34.985493 s**（最短 24.840043 s）。P95 对全部 45 项线性插值，不平均每轮 P95。

算法 45 个有效资源窗口总计 **1488.44 core-s**，按窗口 wall 加权 CPU **118.1981%**，trial 窗口平均 CPU 的均值/P95/最大 **114.9132 / 202.3304 / 216.6429%**，最大 RSS **219.984375 MiB**；Gazebo 仿真单列 **1642.36 core-s**、加权 CPU **130.3919%**、最大 RSS **2999.910156 MiB**。CPU 100% 为一个逻辑核，trial CPU 最大是窗口平均值的最大，不是瞬时峰值。外部 host 的独立 sim/RViz/algorithm 负载不计入算法；归档环境负载不能写成重启后的当前进程状态。

5 次恢复发生于 round 1 `p13_yaw3`、`p16_yaw2`，round 2 `p13_yaw1`、`p01_yaw2`，round 3 `p13_yaw1`；原因均为局部数值不收敛，下一扫描仅恢复一次。45 个终态的 queue drop/overflow/stale/oversized/canceled 均为 0；不能据此宣称任意负荷下无丢帧。45 个 `(run_id,repetition,trial_id)` 唯一键及 exact pose-stamp 评分联接全部通过；实际冻结矩阵第 9 项是 `p10_yaw3`。

原始[回归 manifest](../tools/global_relocalization/data/resume_parallel_validation_20261014T123937Z/bounded_fresh45_20261014T154902Z/accuracy45/regression_manifest.json)和三轮 [round 1](../tools/global_relocalization/data/resume_parallel_validation_20261014T123937Z/bounded_fresh45_20261014T154902Z/accuracy45/round_01/results.summary.json)、[round 2](../tools/global_relocalization/data/resume_parallel_validation_20261014T123937Z/bounded_fresh45_20261014T154902Z/accuracy45/round_02/results.summary.json)、[round 3](../tools/global_relocalization/data/resume_parallel_validation_20261014T123937Z/bounded_fresh45_20261014T154902Z/accuracy45/round_03/results.summary.json)保持不动；完整[结果分析](../tools/global_relocalization/data/resume_parallel_validation_20261014T123937Z/bounded_fresh45_20261014T154902Z/analysis.md)、[重算 summary](../tools/global_relocalization/data/resume_parallel_validation_20261014T123937Z/bounded_fresh45_20261014T154902Z/summary.json)和[恢复审计](../tools/global_relocalization/data/resume_parallel_validation_20261014T123937Z/bounded_fresh45_20261014T154902Z/resumption_audit_20261019.md)提供逐项证据。

本 45 项批次的历史清理记录缺失：归档临时 driver 在读取 `/proc/18288/environ` 时抛出 `psutil.AccessDenied`，`cleanup.json` 未落盘；不能宣称历史 zero Twist 成功或完整清理。重启后无所属仿真/ROS 进程仅证明当前状态。正式 batch 已处理该读取异常，归档临时 `bounded_live_runner.py` 不得原样复用；详见[审计](../tools/global_relocalization/data/resume_parallel_validation_20261014T123937Z/bounded_fresh45_20261014T154902Z/resumption_audit_20261019.md)。

地图由同一理想仿真场景的 GT 融合得到；在线估计器不订阅 GT，但参考地图和同源传感器限制了精度结论的外推。raw scan 为 instantaneous、无逐点时间，不能证明真实旋转扫描去畸变或实车可靠性。当前结果没有验证共享 main 中其他用户的 SearchObserver/visualization GUI 版本。timed-scan、Point-LIO 地图/速度、加速旋转、GUI、导航、长时稳定性及 120 项后续测试继续暂停。

### 下一次 55-key 复测入口（本轮未执行）

下一次若另行开展复测，应从正式 [regression_45.py](../tools/global_relocalization/regression_45.py) 的三轮流程进入；第 13 节的 53-key 命令仅属历史复现，不作为当前候选入口。归档临时 `bounded_live_runner.py` 有上述清理缺陷，不能原样复用。

1. 创建全新 output 目录、未占用 ROS domain 和唯一 Gazebo partition；沿用第 3–5 节的隔离启动、simulation audit 和物理预检流程，使用本次冻结的原 15-cell 矩阵（第 9 项 `p10_yaw3`），不得复用旧预检来证明新 session 的实际物理状态。
2. 明确选择冻结 `bounded_source_root` 与 `bounded_install`，核对 profile/map/core/node SHA-256；确认所有脚本启动时的 setup/LD_LIBRARY_PATH 实际指向该 install。每轮启动后保存 node/core `/proc/<pid>/maps`，不能仅凭工作树或 `ros2 pkg prefix` 推断实际加载 core。保留共享 main 的其他用户改动。
3. 实际地图路径、manifest/hash、传感器单发布者和物理预检通过后，batch 的 [runtime 参数审计](../tools/global_relocalization/audit_runtime_parameters.py) 必须核对本 profile 全部 **55/55 keys**，并检查 estimator 不订阅 GT。workers=2、OMP=4、server timeout=60 s；不开 GUI、bag 或 point dump。第 6 节的旧 43-key 例子只是历史批次。
4. 正式入口的必需参数为 `--map --manifest --params --trials --output --physical-precheck --simulation-audit`，三轮独立算法/evaluator 进程，始终保留 45 分母；任一轮失败停止，未执行项明确记 unattempted。内部一次恢复沿用同一 action 计时，不重置预算；按 `(run_id,repetition,action.pose_stamp_ns)` 精确联接唯一评分，按 `(run_id,repetition,trial_id)` 去重。
5. 启动器只清理自己创建并保留句柄的 `Popen` / process group；先发送零速并持久记录返回结果，再有界等待和升级停止，最后记录 remaining owned PIDs。身份读取遇 AccessDenied 应标为未知并保留证据，不能误杀外部进程或因此中断 cleanup 记录落盘；第 11 节描述拥有进程的身份与清理边界。

以上是下一次复测的准备和验收约束；2026-10-19 本次仅重算原始 45 项并更新文档，没有执行这些入口。
