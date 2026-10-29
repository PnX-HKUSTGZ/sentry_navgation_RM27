# ROGMap / MINCO / MPC 可重复导航批测

每次重复均重新启动 Gazebo，固定世界、出生点、种子和目标序列。ROS domain 必须空闲；Gazebo 使用独立 partition。脚本只清理自己创建的进程组，不调用全局 `pkill`。失败不删除，某段失败后停止本次路线，下一次重复重新出生。

先构建当前源码，避免安装目录里的旧库与新参数混用：

```bash
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
CMAKE_BUILD_PARALLEL_LEVEL=4 colcon build --base-paths src \
  --packages-select rog_map minco_planner minco_controller pb_nav2_plugins \
  fake_vel_transform loam_interface point_lio ign_sim_pointcloud_tool \
  sensor_scan_generation relocalization_manager pb2025_nav_bringup rm_27_stimulation \
  --executor sequential --symlink-install \
  --cmake-args -DBUILD_TESTING=ON
source install/local_setup.bash

python3 tools/navigation_benchmark/batch.py \
  --output /tmp/rm27-baseline --domain 92 --repeats 3 --seed 42 \
  --params src/pb2025_nav_bringup/config/simulation/minco_params.yaml \
  --goals '[[5.231,5.213],[-3.535,6.374],[-4.56,-2.03],[-3.94,-4.89],[-4.56,-2.03]]'
python3 tools/navigation_benchmark/summarize.py --root /tmp/rm27-baseline \
  --scenario tools/navigation_benchmark/scenario.json
```

调整参数后以不同输出目录运行同一命令。比较时检查 `manifest.json` 中的参数、程序哈希和启动命令；程序不同的结果不能归因于单个参数。固定种子不能消除 ROS 调度与传感器送达顺序的变化，因此需要重复测试。本工具保留轨迹和日志，不声称传感器级确定性重放。

`--mode gt` 使用 Gazebo 真值定位。`--mode full_chain` 启动现有 Point-LIO、loam_interface 和重定位管理器；需要匹配的 PCD、初始化配置与已安装依赖。观察器独立读取真值，不发布初始位姿或控制速度。它检查运行拓扑，不能仅据此证明定位精度。全局重定位一次初始化后继续使用真值相对里程计的混合链，不属于 full_chain。

默认 RMUC2026 PCD 已提供在 `src/pb2025_nav_bringup/pcd/simulation/RMUC2026.pcd`，与 `tools/global_relocalization/assets/RMUC2026/map.pcd` 同字节；相邻 `RMUC2026_manifest.json` 记录来源和哈希。地图坐标与世界一致，MINCO 仿真配置的 RMUC2026 world profile 使用零先验变换。该先验来自仿真真值辅助的离线地图准备，不是 Point-LIO 实际建图产物，也不意味着在线使用真值。完整链配置使用 `/cloud_registered` 和 `/aft_mapped_to_init`，保留 `camera_init` 坐标系。

以下命令使用新构建的默认先验与当前仿真 MINCO 参数；DDS 配置可通过 `--dds-profile` 指定，并在 manifest 中记录路径和 SHA256。当前版本省略此参数时优先继承环境中的 XML，否则使用仓库内 64 MiB SHM + UDP 配置；传 `--dds-profile ''` 可保留中间件默认传输。历史记录中的空哈希不能视为使用了指定配置：

```bash
python3 tools/navigation_benchmark/batch.py --mode full_chain --domain 92 \
  --output /tmp/rm27-full-chain --repeats 1 --seed 42 \
  --params src/pb2025_nav_bringup/config/simulation/minco_params.yaml \
  --dds-profile src/rm_27_stimulation/config/fastdds_large_clouds.xml \
  --goals '[[5.231,5.213],[-3.535,6.374],[-4.56,-2.03],[-3.94,-4.89],[-4.56,-2.03]]'
```

当前批测也记录解析后的默认先验哈希；要指定先验文件，可显式加 `--prior-pcd src/pb2025_nav_bringup/pcd/simulation/RMUC2026.pcd`；不指定时 launch 使用安装包的默认路径，旧版批测 manifest 中该项可能为空。确认每次复跑的参数、DDS 配置和核心二进制哈希一致后再比较。

重新汇总本轮 follow-up 的已有记录，不启动仿真：

```bash
python3 tools/navigation_benchmark/followup_summary.py
```

输出 `results/20261028_followup/followup_summary.json` 是临时矩阵，运行中记录为 partial。南洞路线使用 `scenario_south.json`，其余使用 `scenario.json`。矩阵区分动作内事件与整个启动日志；旧观测中缺失的真值终点门限保留为 null，末次采样距离不替代该门限。

运行时保存 planner/controller 全参数、生命周期、话题发布者；每段成功还要求实际收到 MINCO 轨迹和非零 MPC 输出。`runtime.json`、`final.json`、`run_XX.json`、`launch.log` 和 `manifest.json` 一并保留。规划里程计与真值可能处于不同坐标系，不直接相减。

统计采用仿真时间，速度来自真值里程计：

- 主停车：水平速度 ≤0.05 m/s，连续 ≥1 s；长停 ≥5 s。另输出 ≥0.1 s 的敏感性统计。
- 排除开始的 2 s 和距目标 ≤0.35 m 的停车；边界事件拆分，避免把超长起步停顿全部排除。
- 间隔超过 0.3 s 的采样不连接，不将缺测误算为停车；不会合并短暂移动。
- 坡道由区域矩形或 roll/pitch 合成倾角 ≥0.06 rad 标记；洞口使用文档路线的近似矩形。区域标签用于筛查，不作为碰撞证据。
- 日志中的 BLOCK、碰撞/净空拒绝仅证明触发保护，不能证明停车必要。只有显式 `safety_reason` 覆盖至少 80% 的停车时间才排除；其余保留为待判定候选。不要把候选数量写成已确认的非预期停车数量。

`summary.csv` 主计数排除起终点和明确安全停车；`run_XX_metrics.json` 保留全部事件、相关日志、位置、最大指令速度和缺测情况。失败或超时不能因未完成后续路段而被视为“后续零停车”。

```bash
python3 -m unittest discover -s tools/navigation_benchmark -p 'test_*.py' -v
```

后续结果可统一重算（仅全部试验结束后使用 `--final`）：

```bash
python3 tools/navigation_benchmark/followup_summary.py --final
python3 tools/navigation_benchmark/audit_coverage.py \
  --root tools/navigation_benchmark/results/20261028_followup
```

`audit_coverage.py` 要求连续采样路径经过洞内且跨越两侧边界；绕行到达目标不会算作穿洞。目标坐标接受整数和小数，在启动前统一转换为 ROS 所需浮点数；非有限数值及非法形状会被拒绝。`observer_goal_serialization` 为早期工具故障，不能计作导航启动失败。超时、定位假到达与 ≥5 秒低速长停分别保留，运动中的大范围绕行不能写作零卡住。

当前 RMUC2026 MINCO world profile 对 Point-LIO 使用 `point_filter_num: 1`、`filter_size_surf: 0.1`；这只在该世界的 Point-LIO 链生效，GT 模式不启动 Point-LIO，其他世界与实车默认不随之改变。历史 4DoF 实验未采用，相关生产实现及测试已撤回。`full_chain_gravity*.yaml` 是历史实验配置，不能在当前代码中单独启用 4DoF；如需复现实验，先按[归档补丁说明](results/20261028_followup/validation/4d_experiment_patches.md)恢复实验代码、重建并测试。
