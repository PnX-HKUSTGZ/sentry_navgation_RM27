# relocalization_manager

RM27 导航栈的重定位管理器。它使用
[small_gicp](https://github.com/koide3/small_gicp.git) 作为内部点云验证后端；
管理器节点负责维护重定位状态，并发布最终的 `map -> odom` 变换。

给定 `odom` 坐标系下的已配准点云，以及使用
[point_lio](https://github.com/SMBU-PolarBear-Robotics-Team/Point-LIO) 或类似工具构建的先验点云，
`RelocalizationManagerNode` 会计算两份点云之间的修正量。
`SmallGicpVerifier` 只返回经过验证的候选位姿；它不会直接做决策或发布 TF。

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

## 使用

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
