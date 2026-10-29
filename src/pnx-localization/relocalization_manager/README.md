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
