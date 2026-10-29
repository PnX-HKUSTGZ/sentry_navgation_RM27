# Point-LIO 仿真参数优化（2026-10-28）

分支 `replace_gicp`，RMUC2026，MINCO，`use_ground_truth_odom:=false`，Gazebo GUI 和 RViz 开启。
Gazebo 真值只用于评估物理运动，没有接入导航定位。按用户要求，`vehicle_height` 保持本轮开始时的 `0.05`。

## 验证结果

每轮依次执行北侧长路线、北侧下坡、北侧上坡、南侧接近、南侧上坡、南侧下坡，共六个目标。
“中途停车”定义为开始移动后实际平面速度低于 0.05 m/s、连续不少于 0.4 秒仿真时间；成功后的终点停车不计。
每段还设有独立的墙钟超时，因此仿真负载过高时可能超时，不能只看停车计数评价成功。

| 指标 | 基线 | 选定参数第 1 轮 | 独立重复第 2 轮 |
| --- | ---: | ---: | ---: |
| 成功目标 | 6/6 | 6/6 | 6/6 |
| 中途停车次数 | 4 | 3 | 2 |
| 累计停车（仿真秒） | 10.22 | 4.36 | 4.50 |
| 实际行驶路径长度（m） | 66.43 | 65.52 | 65.24 |
| GICP 首次验证到首次 LOCALIZED（墙钟秒） | 5.52 | 1.00 | 1.02 |
| 最后记录的定位状态 | LOCALIZED | LOCALIZED | LOCALIZED |

分段停车次数 / 时长：

| 路段 | 基线 | 第 1 轮 | 第 2 轮 |
| --- | --- | --- | --- |
| 北侧长路线 | 2 / 2.94 s | 1 / 0.76 s | 0 / 0 s |
| 北侧下坡 | 2 / 7.28 s | 2 / 3.60 s | 1 / 3.26 s |
| 北侧上坡 | 0 / 0 s | 0 / 0 s | 0 / 0 s |
| 南侧接近 | 0 / 0 s | 0 / 0 s | 0 / 0 s |
| 南侧上坡 | 0 / 0 s | 0 / 0 s | 0 / 0 s |
| 南侧下坡 | 0 / 0 s | 0 / 0 s | 1 / 1.24 s |

本轮减少了总体停顿，没有消除所有短停，也没有证明每一段每一次运行都会改善。

## 最终参数

主要修改在 `src/pb2025_nav_bringup/config/simulation/minco_params.yaml`：

| 参数 | 原值 | 最终值 |
| --- | ---: | ---: |
| MPC 与 MINCO 最大平面速度 | 1.5 m/s | 1.2 m/s |
| 近障碍 / 已观测前缀速度 | 1.0 m/s | 0.85 m/s |
| 坡道速度上限 | 1.15 m/s | 0.95 m/s |
| 控制延迟补偿 | 0.12 s | 0.20 s |
| 起步参考最大前移时间 | 0.85 s | 0.75 s |
| Point-LIO 模式里程计超时 | 0.35 s | 0.50 s |
| ROG 空闲射线 `p_miss` | 0.45 | 0.40 |
| 浅层动态边界容差 | 0.03 m | 0.04 m |
| GICP 最大迭代次数 | 30 | 60 |
| GICP 平移 / 旋转收敛步长 | 1 mm / 0.1° | 5 mm / 0.2° |
| GICP 平均误差上限 | 0.30 | 0.15 |
| GICP 单次 XY / Z 修正上限 | 0.60 / 0.25 m | 0.25 / 0.12 m |
| GICP 单次 yaw 修正上限 | 10° | 5° |
| RViz 默认刷新率 | 20 Hz | 10 Hz |

GICP 的两项底层收敛阈值此前没有 ROS 参数入口，本轮只增加参数声明、有效性检查和向 small_gicp 的传递。
默认值仍是原来的 1 mm / 0.1°。没有重写配准、规划或控制算法。

GICP 候选仍必须通过内点数、内点率、误差和位姿增量验收。最终门限更严格，因此候选拒绝数可能增加；
两轮启动和最终停车时均完成连续验证。不能把候选拒绝全部视为定位失败，或为追求放行率取消这些门限。

动态边界容差只作用于现有代码允许的浅层边缘单元；原始 PGM、多边形、墙体和车身内部障碍检查继续生效。
0.50 s 里程计超时和其他失效停车机制仍然启用。实车导航参数没有修改。

## 试验取舍及配置一致性

先进行了多组纯参数试验。增加 LIO 点数、修改体素尺寸以及部分投影保持参数的组合未通过全部路线，未保留。
双仿真争抢 GPU 的一轮以及重复 goal 发布者的一轮不用于单实例对比。

测试期间并行修改的四项投影参数曾被合并保留并单独复测；该组合在南侧下坡超时，所以最终恢复为两轮通过的组合：

- `prior_free_uncertain_obstacle_confirmations`：6 恢复为 4。
- `clearance_dropout_hold_time`：2.0 恢复为 1.5 s。
- `obstacle_hold_time`：0.3 恢复为 0.0 s。
- 顶层 `min_headroom_known_ratio`：0.05 恢复为 0.10；Point-LIO 专用覆盖值仍为 0.40。

并行修改的值、失败结果和两轮选定结果均保存在 [完整对比 JSON](validation/pointlio_parameter_tuning_2026-10-28.json)。
最终源 YAML 已与 [选定参数快照](validation/pointlio_tuning_selected_2026-10-28.yaml) 做语义一致性检查。
[基线快照](validation/pointlio_tuning_baseline_2026-10-28.yaml) 也已保留。

## 复测方法

先运行原仿真入口，另一个终端使用相同 ROS_DOMAIN_ID：

```bash
cd /home/pnx/nav_ws/sentry-navigation-RM27
source install/setup.bash
python3 tools/stimulation/benchmark_pointlio_navigation.py \
  --output /tmp/pointlio_route_benchmark.json
```

路线定义在 `tools/stimulation/pointlio_benchmark_routes.json`。测试脚本带单实例锁，使用 Gazebo 真值统计速度和路径，
只在定位及导航 action 就绪后发送目标；退出时会取消尚未结束的目标。

新增 3 项 GICP 集成测试通过，覆盖已知位姿恢复、收敛后仍拒绝过大修正、非法阈值拒绝。
6 项相关启动/参数契约测试通过，新增脚本语法检查通过。
扩大检查时发现一项原有实车恢复速度配置不一致：规划器 0.15 m/s、控制器 0.20 m/s，HEAD 中同样如此；本轮未修改该实车配置。
