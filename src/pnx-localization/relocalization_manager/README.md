# relocalization_manager

RM27 导航栈的重定位管理器。它使用
[small_gicp](https://github.com/koide3/small_gicp.git) 作为内部点云验证后端；
管理器节点负责维护重定位状态，并发布最终的 `map -> odom` 变换。

给定 `odom` 坐标系下的已配准点云，以及使用
[point_lio](https://github.com/SMBU-PolarBear-Robotics-Team/Point-LIO) 或类似工具构建的先验点云，
`RelocalizationManagerNode` 会计算两份点云之间的修正量。
`SmallGicpVerifier` 只返回经过验证的候选位姿；它不会直接做决策或发布 TF。
该节点不执行全局位姿搜索，启动时需要通过 `init_pose` 或 `/initialpose` 提供足够接近
真实位姿的初值，再由 small_gicp 做局部校准。

## 依赖

- ROS2 Humble
- small_gicp
- pcl
- OpenMP

## 构建

1. 安装依赖

    ```zsh
    rosdepc install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
    ```

2. 构建

    ```zsh
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=release
    ```

## 基本使用

1. 在 bringup launch 文件中通过 `prior_pcd_file` 设置先验点云文件；如果单独运行该包，
   则在 [launch/relocalization_manager_launch.py](launch/relocalization_manager_launch.py)
   中设置该参数。

2. 调整 `base_frame` 与 `lidar_frame` 之间的坐标变换

    `pointlio`、`fastlio` 等算法输出的 `global_pcd_map` 严格基于 `lidar_odom` 坐标系。
    但机器人初始位置通常由 `odom` 坐标系下的 `base_link` 坐标系定义。
    为了解决这一差异，代码会监听从 `base_frame`（velocity_reference_frame）到
    `lidar_frame` 的坐标变换，从而将 `global_pcd_map` 转换到 `odom` 坐标系下。

    如果未设置，则会使用空变换。

3. 运行

    ```zsh
    ros2 launch relocalization_manager relocalization_manager_launch.py
    ```

## 重力方向约束

`gravity_aligned` 默认关闭。若先验地图和 LIO 世界系均已对齐重力，可将其设为
`true`，使用 small_gicp 的 `RestrictDoFFactor` 约束 roll/pitch 增量，优化平移和 yaw。
质量判定直接使用约束优化的结果。该选项约束的是 `map -> odom`，机器人自身上坡、
倾斜仍由 LIO 姿态表达。仿真配置已开启此选项。

仿真输入使用完整去畸变点云，经 `loam_interface` 转为 odom 系后配准。
`init_pose` 对应初始底盘原点的地图位姿，不能再叠加雷达安装偏置。

## 配准收敛阈值

- `translation_epsilon`：每次优化平移增量的停止阈值，单位 m，默认 `0.001`。
- `rotation_epsilon`：每次优化旋转增量的停止阈值，单位 rad，默认约 `0.001745329`（0.1°）。

两者须为有限正值。仿真 MINCO 配置采用 5 mm / 0.2°，与点云采样精度相匹配。
收敛后仍须通过误差、内点数、内点率和局部位姿增量验收，不能用收敛标志替代定位质量判断。

## 全局搜索与自转耗时

`global_relocalization_node` 的比赛流程在 raw IMU 重力参考与传感器偏置确定后，
就异步构建虚拟视点数据库，与第一圈点云采集重叠。转满一圈后的 `PREPARING`
等待建库完成，再执行全局检索和 GICP；同一个 Core 的建库与搜索不会并发运行。
建库和描述子评分分别最多使用 4 个工作线程，结果按原始视点顺序汇总，保留全部
高度层、候选预算、歧义检查和停车后三帧独立确认。诊断输出复用检索评分。
取消或提前开赛会立即停车，未完成的数据库不会进入缓存，工作线程结束后才能重试。

`/global_relocalization/status` 的 `metrics` 提供 `database_warmup_sec` 和
`database_warmup_ready`，用于观察与采集重叠的建库；`database_sec` 是
`retrieval_sec` 内的冷建库部分，缓存命中时为零。`retrieval_sec`、`gicp_sec`
分别表示检索和候选配准墙钟耗时。建库就绪仅表示缓存可用，定位准备完成仍以
`ARMED` 为准。比赛模式在 `ARMED` 后继续自转，直到显式开赛事件才停车确认；
等待开赛的时间须与算法搜索耗时分开统计。

## 定位后的底盘朝向

启动协调器的 `heading_alignment_mode` 支持 `none`、`spawn`、`nearest_axis`。
未指定时兼容旧 `restore_spawn_heading`：true 对应 `spawn`，false 对应 `none`。
MINCO 仿真配置使用 `nearest_axis`：位姿交接确认后，从第一份新鲜且通过验收的
`map → base_footprint` TF 中选择最近的 0°/90°/180°/270°，初始转角不超过 45°。
恰好等距时选择逆时针方向；目标选定后固定，避免跨越分界线时来回切换。
这些方向按当前 RMUC2027 地图坐标轴定义，适用于当前近正方形底盘足迹。

实际旋转仍检查完整旋转净空、位置漂移、TF/传感器新鲜度，保持 0.6 rad/s 上限、
0.01 rad 朝向误差和静止 0.3 s 确认，再释放初始化速度入口并启动导航。
确认位姿及其原始时间戳不会被朝向目标覆盖。该策略只负责启动末尾对齐，之后的
行驶朝向由 MINCO 规划与控制决定。实车配置保持原有交接方式。
