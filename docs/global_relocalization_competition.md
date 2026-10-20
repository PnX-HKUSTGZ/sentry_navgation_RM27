# 赛前旋转准备与比赛开始确认模式

**状态：已按用户要求暂停（2026-10-19，额度不足）。**比赛模式和实测约3 s/圈已实现，但高速定位初始三点 **0/3**，端到端尚未通过。已发布零速并核实清理本轮独占仿真，未碰外部进程；所有未提交代码/原始结果保留。20帧子图只是未执行的建议，尚无对应修复。恢复入口见[暂停检查点](../tools/global_relocalization/data/competition_20261019T073851Z_6fc8e7c9/shutdown_checkpoint.md)，先在冻结原数据离线比较子图覆盖，再修复与小样本；不自动启动120项。

这是独立实验模式，默认 raw_imu 流程保持不变。新配置为 `src/global_relocalization/config/raw_imu_competition_left_half.yaml`。只使用 raw LiDAR、IMU 和固定外参；运行时 GT 仅由独立评估客户端读取；冻结先验地图仍由 GT 融合建立，并非 Point-LIO 建图，这限制了结果对实际建图误差的代表性。无导航、无 GUI。本轮测试范围为速度预检、固定困难三点各一次与早触发安全检查，120 次稳定性验收仍暂停。

模式在赛前完成 2 s 初始静置/IMU 标定、自转采集和全局候选预搜索。`ARMED` 后继续旋转等待事件。调用 `/global_relocalization/start_match`（`std_srvs/srv/Trigger`）锁存开始事件并立即发布零速；通过停稳保护与 `stop_settle_sec: 0.2` 后，使用恰好三个独立时间戳的新扫描执行精 GICP 确认。算法保留原质量、运动、半场、歧义与三个独立确认门限。完整 3D ≤.10 m、SO(3) ≤2° 是独立 GT 评分阈值，不进入定位器。成功后立即返回并释放控制，无额外成功后等待。预搜索未完成时提前触发会停车并失败；无活动、重复触发明确拒绝，取消和新一轮不复用上一轮候选。

`settle_sec: 2.0` 和 `raw_calibration_sec: 2.0` 保留初始标定语义；`.2 s` 只用于开始事件后的停止稳定窗。赛前超时 `prestart_timeout_sec: 120.0`，事件后 `post_start_timeout_sec: 5.0`。competition 允许指令角速度最高 3.5 rad/s，普通模式保持 1.5 上限。新 profile 其余参数继承现 55-key robust 配置，未更改旧 profile。

本机 2026-10-19 headless Gazebo 物理预检使用独立 ROS domain 103 和随机 Gazebo partition。命令 2.0943951024 rad/s 的 GT/IMU 中位数为 1.950879 rad/s，GT 姿态累计首圈 3.221574 s，未达到约 3 s/圈。经批准仅将新 profile 的命令校准为 2.25 rad/s：GT/IMU 中位数为 2.106452/2.106449 rad/s，实际首圈 2.982676 s；最大平移 0.3081 mm，停后 GT 姿态差分角速度为零、IMU 最大 1.7452e-6 rad/s。首圈由 GT 姿态连续解包到 2π 插值测得，不由指令推算。GT twist 停后缓存旧值，不用于停止判断。这个 2.25 指令只代表当前仿真校准，硬件必须重新实测。

原始证据：[2.094 测量](../tools/global_relocalization/data/competition_20261019T073851Z_6fc8e7c9/physical_rotation_2p094.json)、[2.25 测量](../tools/global_relocalization/data/competition_20261019T073851Z_6fc8e7c9/physical_rotation_2p25.json)，各有同名 `.jsonl` 原始 GT/IMU 轨迹。控制器和物理引擎保持原有 VelocityControl / Bullet Featherstone。

独立冻结构建与测试命令（新目录必须不存在）：

```bash
export COMPETITION_BUILD=/absolute/new/competition_build
bash tools/global_relocalization/build_competition.bash "$COMPETITION_BUILD"
source /opt/ros/jazzy/setup.bash
source /home/zhang/ros_ws_jazzy/sentry_navgation_RM27/install/local_setup.bash
source "$COMPETITION_BUILD/install/local_setup.bash"
export LD_LIBRARY_PATH="$COMPETITION_BUILD/install/global_relocalization/lib:$LD_LIBRARY_PATH"
export GLOBAL_RELOCALIZATION_EXPECTED_CORE_LIBRARY="$COMPETITION_BUILD/install/global_relocalization/lib/libglobal_relocalization_core.so"
ctest --test-dir "$COMPETITION_BUILD/build/global_relocalization" --output-on-failure
```

构建脚本一次复制当前完整 package 源码，保存逐文件 SHA-256，再在新 source/build/install/log 中 Release 构建。本次在首轮冻结后发现 ARMED 发布次序竞态，最终采用明确归档的 revision2，保留 revision1 manifest/node 和修订原因，并强制重编 node，真实试验仅使用最终版本。只读外部依赖，不写共享 install；Linux-only PATH 避免 Windows 路径拖慢。应在 core/node 均完成后执行冻结，不能先复制再漏掉后续修改。

真实仿真每点运行一次，固定 `p13_yaw3`、`p26_yaw2`、`p16_yaw2`，每次输出新目录。仿真需先在新 domain/随机 partition 启动并 audit 成功，之后所有命令使用相同环境；独立安装 overlay 必须最后 source。示例：

```bash
/usr/bin/python3 tools/global_relocalization/competition_trial.py \
  --node "$COMPETITION_BUILD/install/global_relocalization/lib/global_relocalization/global_relocalization_node" \
  --params "$COMPETITION_BUILD/source/global_relocalization/config/raw_imu_competition_left_half.yaml" \
  --map tools/global_relocalization/data/station_map_20261013/map.pcd \
  --manifest tools/global_relocalization/data/station_map_20261013/manifest.json \
  --trial p13_yaw3 --output /absolute/new/p13_yaw3
```

客户端在 `ARMED` 后观察 .3 s 再显式触发，记录全部状态、命令、GT、IMU、准备耗时、开始到姿态、三个确认 stamp、算法 CPU/RSS，并按输出 stamp 使用固定地图变换、GT 线性位置插值与四元数 SLERP 评分。门限为完整 3D ≤.10 m 且 SO(3) ≤2°，失败保留在分母。`--early` 单独测试提前开始停车失败。成功后额外 1 s 仅由观察客户端采样命令，不是服务器等待，不计入开始到 pose；可以检查节点是否释放控制。

当前 raw 点云只支持 `instantaneous` 理想瞬时扫描。真实 MID360 的高速逐点时间、去畸变和实车高速转动尚未支持，不能用这组仿真结果宣称实车达到速度或精度目标。组件/ROS 合约测试也不能替代实际仿真评分。三点结果只表示正确性小样本，不证明整体稳定性或 120 次通过率。

2026-10-19 首轮高速正确性小样本为 **0/3**，不得解释为模式可用性验收通过。三点各只运行一次，均在赛前 raw 前端保留的 `max_fitness=.12` 门限处失败，尚未进入 ARMED、没有发送 start、没有姿态输出或三帧确认样本，开始到 pose 时延无法测量。p13/p26/p16 的失败 fitness 分别为 .146785/.166971/.122116；动作持续 6.626/6.629/8.903 s，队列 drop 均为 0。各点 GT/IMU 实际角速度约 2.1064 rad/s，p16 完整首圈为 3.032691 s；另外两点在首圈完成前已安全失败。算法 CPU 平均 14.53/13.62/53.39%，峰值 RSS 146.48/143.43/158.56 MB；资源窗口明确包含 goal 提交到终态与额外 1 s 客户端被动观察。失败后的持续零速保护保留，成功释放控制因无成功样本尚未实测。详见[完整三点汇总](../tools/global_relocalization/data/competition_20261019T073851Z_6fc8e7c9/initial_three_summary.json)，各点 `result.json` 保留完整状态/命令/GT/IMU，`node_maps.txt` 证明实际加载最终隔离 core 库。

早触发独立真实仿真检查返回 Trigger false、action aborted，明确 “before preparation”，观察到的 cmd 均为零。新 profile 59/59 参数运行时审计通过，定位器没有 GT 订阅。C++ 六个 target 的 45 case 通过；初次两个 ROS target 分别遇到时钟/跨 topic 回调同步竞态，原始失败 log/XML 保留。revision3 只更改测试等待逻辑，生产 binary 未改，重新运行两个 ROS target 均通过（21.36 s），不会将这组组件/合约测试当作真实定位成功。

随后独立诊断 `diagnostic_newspeed_p13_run01` 仅覆盖 `raw_diagnostic_directory`，所有质量/运动参数及生产 binary 保持不变，并未纳入前三点分母。捕获 66 组扫描、原始 IMU、外参、初值、迭代 trace 与目标子图，共 60.77 MB；原始[失败扫描元数据](../tools/global_relocalization/data/competition_20261019T073851Z_6fc8e7c9/diagnostic_newspeed_p13_raw/action_1792396737253929419_1_1792396737263380603/000065.json)及同名 `.pcd` / `_target.pcd` 可供进一步分析。一次不改参离线 raw replay 精确复现 stamp 1264.1、fitness .12024896573051823 的原门限失败；这只是数值重现，不能替代实时成功率。完整 override 与 hash 见同 session 的 `diagnostic_override_manifest.json`；进一步修复与真实复验仍待完成，120 次验收保持暂停。
