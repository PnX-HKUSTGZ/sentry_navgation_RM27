# Raw LiDAR + IMU 全局重定位：完整复现与批测指南

更新：2026-10-13。本文供后续自行运行；本轮按额度要求停止实际复测，仿真已关闭。没有运行最终修复后的固定 5 次、拓展 10 次或正式 120 次。因此最终在线精度、总体成功率和正式资源占用仍待实测，不能把离线通过写成批测通过。

## 1. 当前已验证到哪一步

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

以下命令均是**未来复现步骤**，本轮不执行。

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
  --validate-all --output "$RELOC_PHYS"
/usr/bin/python3 - <<'PY'
import hashlib,json,os,pathlib
root=pathlib.Path(os.environ['RELOC_ROOT']); out=pathlib.Path(os.environ['RELOC_PHYS'])
summary=json.loads((out/'summary.json').read_text())
assert summary['observed_trials']==120 and summary['all_physical_valid']
matrix=root/'tools/global_relocalization/config/trials_left_physical_120.json'
controller=root/'tools/global_relocalization/.build/robot_velocity.json'
manifest={'matrix_path':str(matrix),'matrix_sha256':hashlib.sha256(matrix.read_bytes()).hexdigest(),
 'controller_provenance':json.loads(controller.read_text()),'expected_trials':120,
 'GT_usage':'physical preparation/assessment only; no estimator/goal running',
 'validation_script_sha256':hashlib.sha256((root/'tools/global_relocalization/validate_static_candidates.py').read_bytes()).hexdigest(),
 'ros_domain_id':os.environ['ROS_DOMAIN_ID'],'gz_partition':os.environ['GZ_PARTITION']}
(out/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
PY
```

物理预检时不运行定位器/goal；该脚本持续发布零 Twist，每个固定 pose 静置 3 ROS s、观察 3 ROS s，总体约 12–15 min（RTF≈1）。120 次必须完整保留：至少 50 GT 样本/1 s，平移≤0.10 m、绝对 roll/pitch≤2°、高度范围≤0.02 m、落点 XY 误差≤0.10 m。失败应停止评估前提检查，不可依据定位成功率删点、换点或悄悄缩分母。

已有 [全 120 静置预检](../tools/global_relocalization/data/raw_left120_static_precheck_20261013/summary.json) 为 120/120 物理有效，矩阵/hash/controller 已核。换环境/控制器后应重新做上述预检。新矩阵保留原 28 个位置，仅以同 seed 的 p30/p31 替代原物理异常 p17/p29；旧 120 次及其异常失败仍原样保留。

## 6. 43 键 raw profile 与实际参数审计

完整参数在 [raw_imu_left_half.yaml](../src/global_relocalization/config/raw_imu_left_half.yaml)。关键设置：`input_mode=raw_imu`，raw topics 为 `/livox/lidar`、`/livox/imu`，`search_side=negative_x`，`side_boundary_x=0`，`top_k=10`，`ring_key_candidates=5`，`retrieval_all_views=true`，`diagnostic_output_dir=''`。后者必须为空，预检/正式均不 dump。

严格质量门限包括 fitness≤0.2、inlier≥0.65、歧义检查、三个不同 timestamp 停稳扫描相互确认。raw 接收为 reliable、IMU depth 200/cloud depth 20；35 ms IMU/350 ms scan 缺口门限未放宽。100 ms 局部计算预算用于诊断，不把超预算计成质量拒绝。

batch 启动算法后自动调用 `audit_runtime_parameters.py`，保存 `runtime_parameters.json/log`；对 **43 个 YAML 键实际 list/get**，任何未声明/值不符都拒绝进入正常试验。它也记录订阅 graph，raw estimator 不能订阅 GT 或 `/odometry`。不要仅凭 YAML 声称参数生效。最终新 profile 的这项在线 audit 尚未执行。

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

以下两组在最终修复后**尚未执行**。固定 5 保留 p06，拓展 10 根据旧 Point-LIO ledger 的失败选择 p11/p12/p16/p24 各两个航向与 p26 的旧平移拒绝/超时航向；不按新 raw 成功选点。

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

预计 5 次约 3–5 min、10 次约 6–10 min，具体受候选/GICP耗时影响。检查每个失败、timeout、缺分与 DAQ 拒绝，而非只看进程退出码；聚合成功也不代表定位全通过。确认 15 次完成、43 参数匹配、无源缺口/重复发布、成功姿态全部通过严格评分，并评估剩余候选/确认失败后，再决定正式批测。未达到预检要求先诊断，修复后用新输出目录完整复跑，不覆盖旧结果。

## 8. 冻结后正式 120 次（不录包/不 dump）

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
assert os.getpgid(pid)==pid and abs(p.create_time()-s['create_time'])<.01
assert e.get('ROS_DOMAIN_ID')==s['ros_domain_id'] and e.get('GZ_PARTITION')==s['gz_partition']
def owned():
    members=[]
    for q in psutil.process_iter(['pid','status']):
        try:
            if os.getpgid(q.pid)!=pid or q.info['status']==psutil.STATUS_ZOMBIE: continue
            assert q.environ().get('GZ_PARTITION')==s['gz_partition']
            members.append(q.pid)
        except (psutil.NoSuchProcess,ProcessLookupError): pass
    return members
for sig,seconds in [(signal.SIGINT,8),(signal.SIGTERM,4),(signal.SIGKILL,1)]:
    if not owned(): break
    os.killpg(pid,sig); deadline=time.monotonic()+seconds
    while time.monotonic()<deadline and owned(): time.sleep(.2)
assert not owned()
print('Owned simulation stopped; artifacts preserved')
PY
```

身份核验失败应停止清理，先只读确认实际进程；不能因 PID 相同就发送信号。本轮已对原 PGID9034 验证 command/domain/唯一 partition，发零 Twist并仅清理该自有进程组，剩余为0，记录见 [cleanup.json](../tools/global_relocalization/data/raw_session_20261013_48dc198665e4/cleanup.json)。所有旧批测、两轮 raw pilot、p06 诊断、最终修复与预定矩阵均保留。
