# 完整 UC 场地建图误差与 Gazebo / RViz 可视化

测试使用完整 `RMUC2026` 世界，从 `(-11.7, 3.1)` 连续巡航，经过左右半场、中央通道和已记录高度的坡道，最后返回起点。路线约 147 m，包含 14 个采集站点；二维地图只用于规划路线和高度跟随，不作为 Point-LIO 先验。站点会吸附到膨胀后可通行的栅格，实际站点和吸附距离保存在 `route.json`。

## 运行与查看

在仓库根目录运行。使用已有隔离 Point-LIO 构建；缺失时自动调用 `build_mapping.bash`，从子模块 HEAD 构建，不恢复或改写子模块工作树。

```bash
bash tools/global_relocalization/test_full_uc_mapping.bash
```

默认打开 Gazebo、RViz，创建独立 ROS domain 和 Gazebo partition，保留其他正在运行的仿真。若显式设置了 `ROS_DOMAIN_ID`，应给本测试一个未占用的域。默认巡航速度为 0.3 m/s，不连续旋转；默认使用仓库 `nav2_params.yaml` 建图参数。

可指定新输出目录和速度，或使用之前已记录的仿真 IMU 输入覆盖参数。`--imu-input` 是一次独立实验条件，不是默认配置：

```bash
bash tools/global_relocalization/test_full_uc_mapping.bash \
  --output /data/full_uc_run01 --speed 0.3

# 可选的另一组实验，必须使用新目录：
bash tools/global_relocalization/test_full_uc_mapping.bash \
  --output /data/full_uc_imu_run01 --speed 0.3 --imu-input
```

采集结束后生成报告，Gazebo 和 RViz 保持打开。关闭本次会话，或显示已保存结果：

```bash
bash tools/global_relocalization/test_full_uc_mapping.bash --stop
bash tools/global_relocalization/test_full_uc_mapping.bash --view /data/full_uc_run01
```

`--view` 只显示保存结果，不重跑 SLAM 或驱动车辆；Gazebo 中的实体机器人位于新仿真的出生点，轨迹标记是保存结果。使用 `--headless` 可只运行测试与报告。会话状态在 `tools/global_relocalization/.build/full_uc_visualization/session.json`，其中记录结果目录、环境和进程身份。停止操作核对 boot ID、PID 和内核启动时刻，仅停止该会话拥有的进程。

## 实际建图链路

测试调用原有 `rm_navigation_simulation_launch.py slam:=true use_ground_truth_odom:=false navigation_mode:=legacy`：

- Point-LIO 接收转换后的雷达和 IMU，输出 `/aft_mapped_to_init` 与 `/cloud_registered`，不加载先验地图。
- 地形点云转为 `/obstacle_scan`，送给 **SLAM Toolbox** 生成 `/map`。仓库当前参数是 `use_scan_matching: false`、`do_loop_closing: false`、`transform_publish_period: 0.0`，因此默认实验中 SLAM Toolbox 负责二维栅格化，不进行扫描匹配或闭环修正。
- 巡航控制器独立读取 Gazebo 真值，发布连续速度，沿地面高度连续调整 Z 方向速度；不调用位置重置，不向 SLAM 里程计或 TF 注入真值。这样即使估计漂移，测试车辆仍能走完计划路线。
- `audit_mapping.py --include-slam` 核验实际 Point-LIO 参数、订阅、进程路径和二进制 SHA256，同时记录 SLAM Toolbox 的实际参数和订阅。

## 误差口径与失败处理

采集停止后，使用第一个同时具有真值和估计的时间戳确定一次固定 `T_world_camera_init`。不做整轨迹拟合、尺度修正或参考点云 ICP。真值在估计消息时间戳处进行平移线性插值和四元数 SLERP，最大真值间隔 0.25 s，不外推。

- **轨迹误差**：三维和平面位置误差、XYZ 分量、SO(3) 姿态角、航向误差，以及均值、RMSE、中位数、P95、最大值、终点误差。另记录左右半场分组、真值时长、有效评分时间跨度与缺失样本。
- **三维地图几何偏差**：建图点云与独立的真值参考点云在固定世界坐标下做双向最近邻统计。正向包含采样和可见性差异；反向还包含未覆盖区域，不能等同于纯配准误差。参考地图仅由报告程序读取。
- **二维地图**：SLAM Toolbox 保存的占据栅格与已知 UC 栅格在固定起点坐标基准下比较，记录占据 IoU、precision、recall 和已知区域覆盖。比较排除未知格；地形高度过滤和可见性也会影响这些数值。
- **完整性**：分别记录巡航是否完成、14 个站点的实际接近距离、是否到达左右半场、返回出生点距离。未完成运行明确标为部分结果；SLAM 输出停止后仍可继续真值巡航和原始录包，失败原因不会被清除。完整巡航不代表建图精度合格。

Gazebo 驱动复用现有测试用 `VelocityControl` 适配器，包含高度跟随，不能替代实车动力学验证。GPU LiDAR 是瞬时扫描，而原转换器分配 0–100 ms 的合成逐点时间；本测试保留该链路行为，不据此声称真实 MID360 去畸变精度。

## 可视化

RViz 固定坐标是独立的 `mapping_eval_world`。观察者不发布 SLAM TF 或速度命令：

| 显示 | 说明 |
| --- | --- |
| 灰线 | 完整计划路线 |
| 绿线、绿色箭头 | Gazebo 真值轨迹与朝向 |
| 青线、青色箭头 | 一次起点对齐后的 Point-LIO 轨迹与朝向 |
| 红线 | 同时间戳真值与估计之间的误差 |
| 高度着色点云 | 累计 Point-LIO 注册点云，显示数量有上限，完整数据独立导出 |
| 黄色点云 | 当前注册扫描 |
| 二维点阵 | SLAM Toolbox 占据栅格；使用 PointCloud2 显示，避免部分 WSLg 环境的 RViz Map 着色器问题 |
| 文字 | 当前误差、累计 RMSE、路线进度、完成或失败状态 |

Gazebo 全场俯视窗口显示相同的路线、真值、估计和误差标记，并以青色抽样点显示累计建图点云。标记由 GUI 的 `MarkerManager` 插件接收，使用独立命名空间，不进入雷达传感器场景。[Gazebo MarkerManager 接口](https://gazebosim.org/api/gui/9/classplugins_1_1MarkerManager.html)

`/mapping_eval/status` 和结果目录中的 `visualization_status.json` 提供实时统计与 Gazebo 标记确认状态。导出完成后观察者切换到完整保存轨迹；显示过程中使用的点云抽样不改变误差统计和最终 PCD。

## 结果文件

| 文件 | 内容 |
| --- | --- |
| `full_uc_summary.json` | 轨迹、地图、左右半场、实际覆盖和 SLAM Toolbox 参数汇总 |
| `report.html` | 可独立查看的报告，和同目录图片一起保留 |
| `mapping_evaluation.png` / `.svg` | 全场轨迹、误差曲线与地图几何偏差图 |
| `trajectory_errors.csv` | 带消息时间戳的轨迹误差 |
| `map.pcd` / `manifest.json` | 来自 Point-LIO 的冻结三维地图、固定坐标变换、来源与哈希 |
| `occupancy.yaml` / `.pgm` | SLAM Toolbox 最终二维地图，保存失败时报告明确说明 |
| `geometry_errors.npz` | 点云 XYZ 与各点最近参考表面的距离 |
| `capture.json` / `mapping_errors.json` | 完整真值、估计、控制记录与逐样本评分 |
| `raw_bag/` / `registered_scans/` | 原始传感器、估计、地图、TF 与逐帧注册点云 |
| `mapping_runtime_parameters.json` | 运行时参数和 Point-LIO 二进制审计 |
| `visual_occupancy.npz` | RViz 重新显示二维栅格所需的点阵 |
| `progress.json` | 最近完成的路线点、真实位置、点云数与时长 |

单独重新生成报告：

```bash
source tools/global_relocalization/environment.bash
python3 tools/global_relocalization/report_full_uc_mapping.py \
  --result /data/full_uc_run01 \
  --reference tools/global_relocalization/assets/RMUC2026/map.pcd
```

评估与路线回归检查：

```bash
source tools/global_relocalization/environment.bash
python3 -m pytest tools/global_relocalization/test_full_uc_mapping.py \
  tools/global_relocalization/test_evaluation.py -q
```
