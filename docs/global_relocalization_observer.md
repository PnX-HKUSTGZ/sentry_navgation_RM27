# 全局重定位 RViz 观察模式

本模式用于交互检查算法过程。默认 `visualization_enabled=false`、`rviz=false`，批测保持无 GUI；质量门限、候选策略和定位接受判定由原算法决定。GUI 验证不替代准确率回归。

## 构建和启动

在仓库根目录构建隔离模块：

```bash
bash tools/global_relocalization/build.bash
```

以下三个终端均先执行（使用同一个独立 ROS domain）：

```bash
cd /home/zhang/navi/sentry_navgation_RM27
source tools/global_relocalization/environment.bash
export ROS_DOMAIN_ID=174
```

终端 1 启动仿真。需要同时观察 Gazebo 时将 `gui` 改为 `true`：

```bash
ros2 launch global_relocalization simulation.launch.py gui:=false
```

终端 2 启动算法和 RViz。将地图/manifest 路径替换为已有地图；现有基线地图来自仿真采集，不能当作真实 Point-LIO 建图验收：

```bash
ros2 launch global_relocalization raw_imu.launch.py \
  map_path:=/absolute/path/map.pcd manifest_path:=/absolute/path/manifest.json \
  params_file:=$PWD/src/global_relocalization/config/raw_imu_robust_left_half.yaml \
  visualization_enabled:=true rviz:=true
```

终端 3 开始一次定位，机器人静置、自转、停稳并计算结果：

```bash
ros2 action send_goal /global_relocalization/run \
  global_relocalization_interfaces/action/Relocalize '{}' --feedback
```

registered/LIO 模式使用 `algorithm.launch.py` 的同名两个开关，先按原流程启动 `lio.launch.py`。关闭 RViz 窗口不会结束算法或仿真。只设置 `rviz:=true` 不会自动打开诊断 publisher，应同时指定两个开关。

已有算法进程开启观察 publisher 时，可单独打开窗口：

```bash
ros2 run rviz2 rviz2 -d "$PWD/src/global_relocalization/rviz/observer.rviz" \
  --ros-args -p use_sim_time:=true
```

需要在自转期间直接看累计查询点云时，可另外打开 reference 视图：

```bash
ros2 run rviz2 rviz2 -d "$PWD/src/global_relocalization/rviz/observer_reference.rviz" \
  --ros-args -p use_sim_time:=true
```

## 画面含义

默认地图视图的 Fixed Frame 是 `map`：

| 画面 | 含义 |
|---|---|
| 灰色点云 | 先验 PCD 的限点采样 |
| 青色点云 | 累计查询云按最近候选假设变换到地图；尚不表示定位被接受 |
| 紫色点云 | 独立停稳扫描按当前 confirmation 结果变换到地图 |
| 黄色候选箭头 | Scan Context 初值、正在测试或粗配准 seed |
| 红色候选箭头 | 细配准质量门限未通过，或 confirmation 一致性未通过 |
| 绿色候选箭头 | 该候选质量门限通过；全局歧义检查及三帧确认仍可能拒绝整次动作 |
| `CURRENT RUN ACCEPTED` 箭头 | 仅在本次 action 成功后出现的最终完整位置和姿态 |
| 场地上方文字 | 当前 run、动作阶段、已完成阶段指标及最终失败/成功原因 |

候选和最终箭头使用完整 SE(3) 位姿。观察器不读取 ground truth，不发布任何 TF、`/initialpose` 或速度指令。默认配置不订阅 `/global_relocalization/pose`，避免 RViz 保留前次成功姿态造成误判；最终结果由观察器 marker 显示，新动作清空它。

累计查询和原始独立停稳扫描的坐标系是动作起始 `relocalization_reference`，尚未定位到 map，不发布两者间的 TF。`observer_reference.rviz` 专门显示这两种云；也可在主窗口将 Fixed Frame 改为 `relocalization_reference`，关闭 Prior map、Candidate aligned query、Aligned independent confirmation scan、Stages/ranked seeds displays，开启 Accumulated query 和 Independent scan。切回 `map` 查看配准。

## 阶段和诊断事件

实际回调包括 `RETRIEVAL_COMPLETE`、`GICP_CANDIDATE_START`、`GICP_COARSE_SEED_COMPLETE`、`GICP_FINE_QUALITY_COMPLETE`、`CONFIRMATION_START/COMPLETE`。small_gicp 没有逐迭代回调，画面显示阶段边界；`coarse_iterations/fine_iterations` 是优化器完成后返回的计数。

粗配准仅用于产生初值，标记为 `usable/nonfinite/skipped`，没有粗配准质量接受判定。细配准逐项显示收敛、至少 20 个 inliers、变换有限、fitness 有限、fitness/inlier ratio 门限、半场先验及取消状态。通过这些门限仍需全局歧义检查和独立停稳确认；`COMPLETE` 表示阶段已完成，不能当作整次成功。

confirmation 完成事件提供完整 `xyz/quaternion_xyzw`、与最佳累计查询解及此前 confirmation 的最大平移/旋转差。地图视图同步显示该停稳扫描的对齐云。一致性结果只在 `CONFIRMATION_COMPLETE` 有效，其他阶段是 null。

```bash
ros2 topic echo /global_relocalization/visual/events std_msgs/msg/String \
  --qos-reliability best_effort
```

事件包含时间戳与动作计数构成的 `search_id`、单调 `sequence`、固定候选 `index`、SC/ring 排名、逐项门限及 `dropped_events`。并行候选的事件按完成顺序显示，排名不会随完成顺序改变。重置时清空旧事件与显示；旧 generation 的云/事件被丢弃，终态后不再接收新事件。

## 开销和测试

可视化开启后额外加载先验地图快照。云按固定 stride 采样，默认最多 3000 点；query/aligned query 默认墙钟 1 Hz，先验地图首次发布，并在新订阅者匹配时补发，confirmation 每阶段有一份快照。点云和 marker 使用 best effort、transient local、depth 1；事件使用 best effort depth 32。算法回调只入有界 256 条队列，满时丢旧诊断事件并计数，定位数据队列不受影响。独立 callback group 的 timer 发布快照，不持有节点控制锁或观察器数据锁。

`visualization_cloud_hz` 允许 (0,5]，`visualization_max_points` 允许 100–20000，在节点启动时设置。GUI 增加采样、发布和渲染成本，性能批测应关闭它。关闭时不创建 observer publisher、队列或额外 PCD 采样。

运行本模块测试：

```bash
source tools/global_relocalization/environment.bash
colcon --log-base tools/global_relocalization/.build/ros/log test \
  --base-paths src/global_relocalization_interfaces src/global_relocalization \
  --build-base tools/global_relocalization/.build/ros/build \
  --install-base tools/global_relocalization/.build/ros/install \
  --packages-select global_relocalization --event-handlers console_direct+
colcon test-result --test-result-base tools/global_relocalization/.build/ros/build --verbose
```

Core 测试检查候选阶段配对、并行固定排名、observer 异常隔离，以及观察模式与关闭模式结果一致。观察器测试检查逐项门限、箭头颜色、空/有限指标、confirmation 云坐标变换、重置隔离和最终姿态清理。ROS contract 测试检查默认无诊断 publisher、显式启用才发布限点地图，以及失败动作的阶段 marker。

2026-10-14 的隔离构建已通过；最终 `colcon test-result` 汇总为 43 tests、0 errors、0 failures、0 skipped，两个 launch 的观察/RViz 开关均验证为默认 false。这里包括既有模块测试和观察器检查，不能解释为 43 次定位成功。首次观察器 contract 暴露静态 map 首发早于 DDS 匹配的问题，已用订阅匹配后补发修复；原传感器健康测试另出现过一次 server ROS clock 尚为零的启动竞态，未修改生产保护，针对性复跑及完整 contract 9 项均通过。
