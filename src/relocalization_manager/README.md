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
- FLANN
- python3-yaml

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

## Scan Context 数据库

Scan Context 用于从当前点云中提取全局描述子，并在数据库中查找可能的重定位候选。
当前数据库保存为单个二进制 `.scdb` 文件，不保存每帧原始 PCD，也不会把每个 keyframe
的点云单独写到目录中。保存时会先写临时文件 `*.scdb.tmp`，再重命名为最终的 `*.scdb`。

数据库默认放在 `pb2025_nav_bringup` 中：

- 仿真：`src/pb2025_nav_bringup/scan_context/simulation/<world>.scdb`
- 实车：`src/pb2025_nav_bringup/scan_context/reality/<world>.scdb`

`scan_context_mode` 控制 Scan Context 是否启动：

- `off`：不启用 Scan Context。
- `build`：建库模式。`slam:=True` 且参数文件中写了 `scan_context_mode: "build"` 时，
  `slam_launch.py` 才会启动 `relocalization_manager` 作为 Scan Context builder。
- `query`：查询模式。加载已有 `.scdb`，从当前 `/registered_scan` 生成 descriptor，
  查询候选 keyframe，并把候选转换成 GICP 初值。最终仍由 small_gicp 验证通过后才更新
  `map -> odom`。

注意：`slam:=True` 只表示启动建图流程；它不会强制把 `scan_context_mode` 改成 `build`。
是否建库必须由参数文件显式决定。

### 仿真：从先验 PCD 离线建库

仿真通常使用 prior PCD 直接生成 Scan Context 数据库，不需要让机器人跑一圈采 keyframe。

1. 修改 `src/pb2025_nav_bringup/config/simulation/nav2_params.yaml`：

    ```yaml
    relocalization_manager:
      ros__parameters:
        scan_context_mode: "build"
        scan_context_build_source: "prior_pcd"
    ```

2. 启动仿真建图流程：

    ```zsh
    source install/setup.bash
    ros2 launch rm_27_stimulation sim_with_nav.launch.py \
      world:=RMUC2026  gui:=false use_rviz:=false
    ```

    如果只启动导航 bringup，也可以使用：

    ```zsh
    source install/setup.bash
    ros2 launch pb2025_nav_bringup rm_navigation_simulation_launch.py \
      world:=RMUC2026 slam:=True use_rviz:=false
    ```

3. 正常情况下会看到类似日志：

    ```text
    Scan Context build mode will create a fresh database
    Loaded prior PCD for Scan Context database
    Built Scan Context database from prior PCD
    Saved prior-built Scan Context database
    ```

4. 生成结果：

    ```zsh
    ls -lh src/pb2025_nav_bringup/scan_context/simulation/RMUC2026.scdb
    ```

### 实车：建图时 live 采 keyframe 建库

实车通常使用 live 模式：边跑 Point-LIO/SLAM，边从 `/registered_scan`
生成 Scan Context descriptor，并按位移、角度和时间间隔采 keyframe。

1. 修改 `src/pb2025_nav_bringup/config/reality/nav2_params.yaml`：

    ```yaml
    relocalization_manager:
      ros__parameters:
        scan_context_mode: "build"
        scan_context_build_source: "live"
    ```

2. 启动实车建图流程：

    ```zsh
    source install/setup.bash
    ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py \
      world:=rmul_2024 slam:=True use_rviz:=true
    ```

3. 按导航会用到的区域跑一圈。运行过程中可以观察日志：

    ```text
    Added Scan Context keyframe
    ```

4. 建图结束前手动保存 Scan Context 数据库：

    ```zsh
    ros2 service call /save_scan_context_database std_srvs/srv/Trigger {}
    ```

    如果使用了 ROS namespace，服务名要带 namespace，例如：

    ```zsh
    ros2 service call /<namespace>/save_scan_context_database std_srvs/srv/Trigger {}
    ```

5. 检查数据库文件：

    ```zsh
    ls -lh src/pb2025_nav_bringup/scan_context/reality/rmul_2024.scdb
    ```

6. Point-LIO 在建图模式下会保存 PCD 地图。当前 `slam_launch.py` 会覆盖：

    ```yaml
    pcd_save.pcd_save_en: True
    prior_pcd.enable: False
    ```

    Point-LIO 正常退出后会把累计地图保存到：

    ```text
    src/point_lio/PCD/scans.pcd
    ```

    导航默认会从 `pb2025_nav_bringup/pcd/reality/<world>.pcd` 读取先验 PCD，
    因此建议建图后复制并按 `world` 名称重命名：

    ```zsh
    cp src/point_lio/PCD/scans.pcd \
      src/pb2025_nav_bringup/pcd/reality/rmul_2024.pcd
    ```

7. 退出建库后，将参数改回普通导航需要的模式：

    ```yaml
    relocalization_manager:
      ros__parameters:
        scan_context_mode: "off"   # 或 query，取决于后续是否启用 Scan Context 查询
    ```

### 手动指定数据库路径

顶层 launch 会根据 `world` 自动生成数据库路径。也可以手动指定：

```zsh
ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py \
  world:=rmul_2024 slam:=True \
  scan_context_database_path:=/home/pnx/nav_ws/sentry-navigation-RM27/src/pb2025_nav_bringup/scan_context/reality/rmul_2024.scdb
```

### 查看服务和参数

```zsh
ros2 node list | grep relocalization_manager
ros2 service list | grep save_scan_context_database
ros2 param get /relocalization_manager scan_context_mode
ros2 param get /relocalization_manager scan_context_build_source
```

### 使用 Scan Context 查询辅助 GICP 重定位

建好 `.scdb` 后，可以在普通导航流程中启用 query 模式：

```yaml
relocalization_manager:
  ros__parameters:
    scan_context_mode: "query"
    scan_context_query_on_startup: true
    scan_context_query_on_gicp_failure: true
    scan_context_failure_trigger_count: 3
    scan_context_max_gicp_candidates: 5
    scan_context_yaw_delta_sign: 1.0
```

query 模式下，节点会：

1. 加载 `scan_context_database_path` 指向的 `.scdb`。
2. 从 `/registered_scan` 转到 `scan_context_input_frame`，默认是 `base_footprint`。
3. 生成当前 Scan Context descriptor。
4. 在 `.scdb` 中查找候选 keyframe。
5. 用候选的 `map -> base_footprint` 和当前 TF `odom -> base_footprint`
   计算 GICP 初始值：

    ```text
    map_to_odom_guess = map_to_base_candidate * inverse(odom_to_base_current)
    ```

6. 对前 `scan_context_max_gicp_candidates` 个候选逐个运行 small_gicp。
7. 选择 `mean_error` 最低且通过跳变门限的候选，更新 `map -> odom`。

Scan Context 不会每一帧都强行改 TF。当前触发方式有三种：

- 启动后首次收到有效点云时触发一次：`scan_context_query_on_startup: true`
- 普通 GICP 连续失败达到阈值后触发：`scan_context_query_on_gicp_failure: true`
- 手动服务触发：

    ```zsh
    ros2 service call /trigger_scan_context_relocalization std_srvs/srv/Trigger {}
    ```

如果使用了 namespace：

```zsh
ros2 service call /<namespace>/trigger_scan_context_relocalization std_srvs/srv/Trigger {}
```

query 模式会发布两个调试 topic：

- `/scan_context_candidates`：`geometry_msgs/msg/PoseArray`，显示 Scan Context 候选位姿。
- `/scan_context_best_pose`：`geometry_msgs/msg/PoseStamped`，显示最终被 GICP 接受的候选位姿。

如果候选方向整体反了，可以先把：

```yaml
scan_context_yaw_delta_sign: -1.0
```

然后重新运行对比 RViz 中 `/scan_context_candidates` 的朝向和 GICP 验证结果。

## Scan Context 建库质量

Scan Context 数据库质量本质上取决于：数据库里的 keyframe 是否覆盖了机器人未来可能出现的位置和视角，
以及每个 keyframe 的 descriptor 是否稳定、可区分、和导航时的点云一致。

主要影响因素如下。

1. 先验地图或建图轨迹质量

    仿真 prior PCD 建库时，质量主要取决于 `RMUC2026.pcd` 是否干净、完整，
    `prior_pcd_transform` 的 z 高度是否和 Gazebo 场地模型一致，以及 PCD 与导航地图是否在同一个坐标约定下。

    实车 live 建库时，质量主要取决于 Point-LIO 建图是否漂移小、轨迹是否闭合、点云是否重影少。
    如果建图过程中 Point-LIO 本身漂了，Scan Context 会把错误位姿保存进 `.scdb`，
    后续查询会得到错误候选。

2. TF 和坐标系正确性

    live 建库依赖 `/registered_scan -> base_footprint` 和 `map -> base_footprint`。
    如果 `base_footprint -> left_mid360` 外参、`odom -> base_footprint` 或静态 `map -> odom`
    有问题，descriptor 可能仍能生成，但 keyframe 的 `map_to_base` 会错。

3. 采集覆盖范围

    实车建库时要覆盖所有未来需要导航和重定位的区域，尤其是容易丢定位、遮挡多、转弯多、
    起点附近和关键通道。只跑中间主路，后续在边角区域查询时容易没有好候选。

4. 视角和朝向覆盖

    Scan Context 可以通过 sector shift 估计 yaw 差，但数据库里如果只有单一方向、
    局部环境又高度对称，候选会更容易混淆。实车采集时建议在关键区域有适当转向，
    不要只用完全单向、单视角轨迹。

5. 运动质量

    建库时速度要平稳，避免急加速、急刹、快速原地旋转和明显震动。
    这些会让 LiDAR 去畸变和里程计质量变差，进而影响 descriptor 和 keyframe 位姿。

6. 动态障碍和临时物体

    建库时尽量减少人、车、临时箱子、可移动障碍物进入 LiDAR 视野。
    动态物体被保存进 descriptor 后，导航时环境变化会导致相似度下降或误匹配。

7. 参数选择

    - `scan_context_max_radius`：描述子的感知半径。太小区分度不足；太大容易包含远处噪声和动态物体。
    - `scan_context_min_height` / `scan_context_max_height`：高度过滤范围。要覆盖稳定结构，
      过滤地面噪声和过高无关点。
    - `scan_context_num_rings` / `scan_context_num_sectors`：描述子分辨率。越大越细，
      但对噪声更敏感、计算更多。
    - `scan_context_keyframe_min_translation`：live 建库的关键帧间距。太大覆盖稀疏，
      太小数据库膨胀且重复很多。
    - `scan_context_keyframe_min_yaw`：转角变化达到阈值才新增关键帧。转弯区可适当降低。
    - `scan_context_min_points`：点数门槛。太高会漏掉空旷区域，太低会收进质量差的帧。
    - `scan_context_prior_sample_resolution`：prior PCD 离线建库的采样间距。越小越密，
      数据库越大、生成越慢；默认 `1.0` m 比较保守。
    - `scan_context_prior_leaf_size`：prior PCD 离线建库前的体素滤波。太大细节丢失，
      太小数据库生成更慢。

提升数据库质量的建议：

1. 先保证 Point-LIO/先验 PCD 地图本身正确，再建 Scan Context 数据库。
2. 建图时低速、稳定、少动态物体，关键区域至少完整经过一次。
3. 实车 live 建库后检查日志中的 keyframe 数量，确认不是几乎没有 keyframe，也不是异常爆炸。
4. 对称场景、长直通道和空旷区要增加视角变化，必要时降低 `keyframe_min_translation`
   或 `keyframe_min_yaw`。
5. 仿真 prior PCD 建库后，在 RViz 中确认 PCD、PGM 和机器人出生点坐标一致。
6. 建库完成后保存 `.scdb`，并把 Point-LIO 输出的 `scans.pcd` 复制到
   `pb2025_nav_bringup/pcd/reality/<world>.pcd` 作为导航先验地图。
