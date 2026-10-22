# RMUL2027 仿真世界

来源为仓库根目录 `RMUL2027_3V3_V1.0.0.stp`，保留 CAD 原始配色。本次包含
Gazebo 世界、视觉与碰撞模型、Blender 工程、渲染图，以及同坐标系的导航点云和栅格地图。

## 运行

```bash
cd /home/pnx/nav_ws/sentry-navigation-RM27
source install/setup.bash
ros2 launch rm_27_stimulation rmul2027_simulation.launch.py
```

默认打开 Gazebo、小车及云台控制窗口，不启动导航、地图服务器或重定位。
启动器等待物理时钟就绪后再生成小车，避免复杂场地初次加载时 GUI 漏显示新模型。
小车出生位置读取 `config/spawn_poses.yaml` 的 RMUL2027 项。
最初的原点出生方案中，CAD 原点处表面高 11 mm，轮半径为 80 mm，
生成高度 `z=0.093 m` 包含 2 mm 落地间隙。手动改出生位置后，应按新位置
地面高度调整 z，并同步所用导航模式的初始位姿。

只查看场地：

```bash
ros2 launch rm_27_stimulation rmul2027_simulation.launch.py spawn_robot:=false
```

无图形界面运行：

```bash
ros2 launch rm_27_stimulation rmul2027_simulation.launch.py gui:=false gimbal_panel:=false
```

原有入口的默认场地不变。上述世界启动并生成小车后，在第二个终端启动
Point-LIO + GICP + MINCO：

```bash
source install/setup.bash
ros2 launch pb2025_nav_bringup rm_navigation_simulation_launch.py \
  world:=RMUL2027 navigation_mode:=minco use_ground_truth_odom:=false
```

`world:=RMUL2027` 自动选择对应 PCD、PGM/YAML，并把当前参数文件中的
`simulation_world_profiles.ros__parameters.RMUL2027` 合并到同一个运行参数文件。
MINCO 使用 `config/simulation/minco_params.yaml`，legacy 使用 `nav2_params.yaml`。
不再借用 RMUC2026 的 `init_pose` 和 PCD 高度变换。
如果手动移动出生点，需要同步 RMUL2027 profile 的 `init_pose`，
或用 RViz 初始位姿提供相应粗定位；MINCO 与 legacy 各有自己的 profile。

也可使用联合入口：

```bash
ros2 launch rm_27_stimulation sim_with_nav.launch.py \
  world:=RMUL2027 navigation_mode:=minco use_ground_truth_odom:=false gui:=true
```

联合入口现在也等待场地物理时钟就绪后再生成小车和启动导航，避免网格仍在加载
时就开始导航。旧版碰撞初始化约 40～70 秒；碰撞 LOD 优化后，同机无头测试
物理时钟就绪约 3.49 秒，RMUC2026 对照约 2.68 秒。加载较慢时仍每 10 秒提示。
`pause:=true` 时需先在 Gazebo 按 Play。GUI + RViz + 真值定位联合入口已复测，
约 4 秒物理就绪、18 秒导航激活（包含默认生成/导航启动延时）。
`gui_config:=auto` 会根据 `worlds.yaml` 选择
RMUL2027 的相机视角；显式传入配置路径可以覆盖自动选择。

## 导航地图

以下路径相对 `src/pb2025_nav_bringup`：

| 路径 | 内容 |
| --- | --- |
| `pcd/simulation/RMUL2027.pcd` | 47,239 个 XYZ 点，二进制，约 554 KiB，5 cm 体素降采样 |
| `map/simulation/RMUL2027.pgm` | 5 cm 栅格，260 × 180，不预先膨胀 |
| `map/simulation/RMUL2027.yaml` | 原点 `[-6.5,-4.5,0]`，灰色区域保持 unknown |
| `map/simulation/RMUL2027_metadata.json` | 生成方法、分辨率、坐标和采样统计 |
| `map/simulation/RMUL2027_preview.png` | 放大后的地图预览 |

点云从同一 CAD 表面以 2 cm 间距采样，从可通行地面上的 138 个雷达视点检查
可见性，排除地下底面和被实体遮住的内部面，再以 5 cm 体素取质心。
它直接使用 Gazebo 世界坐标，**不能再加 RMUC2026 的 1.65 m 平移或雷达安装位姿**。
GICP 的目标和源体素均为 5 cm，最大对应距离 0.5 m，保留原有匹配质量门限。
场地有对称结构，局部 GICP 不保证从任意错误朝向完成全局定位。

PGM 先做 1 cm 子栅格地面连通性分析，再合并为 5 cm 格子。原始 CAD 中没有
通向 20 cm 高台的缓坡；非水平斜面主要是 45° 小倒角，因此这些高台按不可通行
区域处理。未生成先验高程图，不把平台自动解释为小车能上的坡。

## 资源

下列路径相对 `src/rm_27_stimulation`：

| 路径 | 内容 |
| --- | --- |
| `world/RMUL2027_world/RMUL2027.world` | 世界、物理插件、光照与场地实例 |
| `meshes/RMUL2027/RMUL2027_visual/` | 9 个 CAD 配色视觉网格及材质配置 |
| `meshes/RMUL2027/RMUL2027_collision/` | 独立碰撞模型 |
| `meshes/RMUL2027/blender/RMUL2027.blend` | 可编辑 Blender 4.5 工程 |
| `meshes/RMUL2027/renders/` | 斜视、俯视渲染图 |
| `meshes/RMUL2027/import_metadata.json` | 源文件校验值、变换、面与碰撞块对应关系 |
| `meshes/RMUL2027/geometry.json` | 保留颜色的米制网格中间数据 |

原始 CAD 单位为毫米，统一使用以下变换：

```text
Gazebo X = CAD X / 1000
Gazebo Y = CAD Z / 1000
Gazebo Z = -CAD Y / 1000
```

场地外包尺寸为 12.6 × 8.6 m，主地面为 `z=0`，模型高度范围约 `-0.10～0.42 m`。
视觉和碰撞实例均采用零位姿，不沿用 RMUC2026 的平移、外墙或额外地板。

## 碰撞与材质

CAD 内核确认实体有效，1,274 个面全部为平面。碰撞 LOD 从 CAD 外壳分层生成
实体体积，合并相邻方盒。60 个完全封闭、外部不可进入的内部空腔填实，外部
开口保留；不对整场地做单一凸包。分层保留孔洞，斜面使用真实三角棱柱，避免
中间高度采样把斜面变成台阶。独立的开放曲面保留向内 1 mm 的碰撞皮层。

| 项目 | 优化前 | 当前 | RMUC2026 对照 |
| --- | ---: | ---: | ---: |
| 碰撞体总数 | 1700 | 503 | 607 |
| 方盒 | 1224 | 397 | 33 |
| 网格实例 | 476 | 106 | 574 |
| 独立碰撞 STL | 191 | 1 | 548 |
| 需运行时凸包处理的独立网格 | 191 | 1 | 4 |

所有三角棱柱复用 `unit_right_prism.stl`，通过旋转和三个方向缩放恢复尺寸。
保留 `convex_hull` 路径和 1 mm 接触余量，避免改成默认三角网格后切换到
10 mm 余量。唯一网格以 200,000 体素处理一次，其结果在同一 Gazebo 进程中
复用。不能只修改 STL 或单独修改 SDF 的 scale、pose。

`tools/rmul2027_collision.py` 中 `DETAIL_TOLERANCE=0.003` 允许合并支撑面上
最多 3 mm 的高度细节。薄顶盖、悬空薄板、厘米级结构和矩形斜面保留。
这是碰撞 LOD 的精度限制：视觉和雷达继续使用原始 CAD，碰撞表面可相差最多
3 mm，不适合作为毫米级极限净空的最终验证模型。
Bullet/ODE 场地摩擦系数仍为 0.8，轮子和驱动力参数未因本次优化调整。

转换器先在临时目录生成完整碰撞模型，成功后先发布依赖网格、再原子替换 SDF，
最后清理旧网格；转换失败时保留旧碰撞资源。替换新的 CAD 版本后必须重跑几何
和物理检查，不能只依据碰撞体数量判断模型正确。

视觉约 3,811 个三角面。已有颜色逐面保留，未赋色的 50 个边界曲面使用中性灰。
材质采用哑光参数；STEP 没有提供实测粗糙度或摩擦系数。Blender 中的影棚底板和
补光灯只用于渲染，不导出到 Gazebo。

## 重新生成

转换工具是离线制作依赖，正常运行仿真不需要 CAD Python 环境或 Blender。

```bash
python3 -m venv /home/pnx/nav_ws/.tools/rmul2027-cad-venv
/home/pnx/nav_ws/.tools/rmul2027-cad-venv/bin/python -m pip install \
  -r src/rm_27_stimulation/tools/rmul2027_requirements.txt

/home/pnx/nav_ws/.tools/rmul2027-cad-venv/bin/python \
  src/rm_27_stimulation/tools/import_rmul2027_world.py

/home/pnx/nav_ws/.tools/blender-4.5.14-linux-x64/blender \
  --background --factory-startup \
  --python src/rm_27_stimulation/tools/render_rmul2027_world.py -- \
  --assets src/rm_27_stimulation/meshes/RMUL2027

/home/pnx/nav_ws/.tools/rmul2027-cad-venv/bin/python \
  src/pb2025_nav_bringup/tools/generate_rmul2027_maps.py

source /opt/ros/jazzy/setup.bash
colcon build --base-paths src --packages-select \
  rm_27_stimulation ign_sim_pointcloud_tool pb2025_nav_bringup --symlink-install
```

网格文件名可能变化，重新生成后必须构建，使新文件进入 install 目录。
Blender 导出脚本使用 4.5 的 COLLADA 导出器。

## 验证

离线检查覆盖凹角、孔洞保留、碰撞网格闭合与凸性、DAE 单位和资源引用：

```bash
/home/pnx/nav_ws/.tools/rmul2027-cad-venv/bin/python \
  src/rm_27_stimulation/test/test_rmul2027_geometry.py
```

`tools/check_rmul2027_runtime.py` 用于隔离仿真中的接触高度、雷达与底盘移动检查。
要求 `GZ_PARTITION=rmul2027-validation`，以及同分区的世界和以下探针桥接：

```bash
ros2 run ros_gz_bridge parameter_bridge \
  '/rmul2027/probes/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry'
```

测试会临时生成测量球体并发布短距离底盘速度，结束时停止底盘并删除测量球体。

本版已通过 12 项离线检查；密集几何对照覆盖 173,376 个地面采样、22,466 条
墙面射线和 76,659 个三维点（含车身高度附近空域）。最大地面高度差 3 mm，
墙面法向偏差 1 mm，三维空域采样未发现额外填塞。
Gazebo Bullet 的 8 个落地测点包含共享棱柱网格，最大接触高度误差 0.99 mm；
小车短距离移动约 0.348 m，MID360 正常输出。

```bash
/home/pnx/nav_ws/.tools/rmul2027-cad-venv/bin/python \
  src/rm_27_stimulation/tools/check_rmul2027_collision_geometry.py
```

同机无头对照使用相同机器人、核显渲染、IMU/LiDAR 和物理步长，各统计 12 秒，
不启动导航/RViz：RMUC2026 与 RMUL2027 实时系数均约 1.0，CPU 秒/仿真秒分别
约 1.81 与 1.43。这是短时性能样本，不代表所有导航工况的长期性能保证。
报告与复查限制见 [碰撞优化复查](validation/rmul2027_collision_review_2026-10-21.md)。

导航检查工具 `tools/mapping/check_rmul2027_localization.py --gimbal` 使用真实
Point-LIO/GICP，覆盖初始定位、15 cm / 5° 初始位姿扰动、前进、侧移和转云台。
它只把 Gazebo 真值作为误差参考，不向定位器发布真值位姿。测试要求隔离环境
`ROS_DOMAIN_ID=75 GZ_PARTITION=rmul2027-map-validation`，世界和导航也需使用相同设置。
连续转动的原始 Point-LIO 对照工具为 `tools/mapping/check_sim_gimbal_imu.py`。
仿真传感器修复和实车排查依据见 [云台与 Point-LIO](gimbal_pointlio_stability.md)。
