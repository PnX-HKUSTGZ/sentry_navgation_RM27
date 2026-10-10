# 建图与地图编辑流程

本文说明 Point-LIO 录包、离线导出、ERASOR2 静态点云处理和 map_edit 地图编辑的独立用法。新增工具不改现有 bringup，也不会启动或停止导航节点。地图和区域文件都需要人工检查、另存并决定是否接入导航。

## 来源与适配

| 目录 | 固定来源 | 适配内容 |
| --- | --- | --- |
| `src/map_edit` | [map_edit](https://github.com/Tony-tpc/map_edit)，`b1c1f8d27d5f4626cefe34cdfa3bc7a90f6a1297` | ROS 2 Jazzy / RViz 插件；本地地图直接送入编辑器；在线参考地图需显式开启；编辑预览、PCD 和区域话题归在 `/map_edit` 下。 |
| `tools/mapping/erasor2/vendor` | [ERASOR2](https://github.com/Tony-tpc/ERASOR2)，`4f300b60965e51e5686af7c40350e170b8bc6580` | 保留上游源码和 `Licence`；用独立 CMake 构建，关闭可选 Rerun 下载，不构建 catkin/ROS 工作区。 |
| `tools/mapping/pointlio_export` | 本仓库新增脚本，不移植 Point-LIO 节点 | 只读 rosbag2，将 PointCloud2 和里程计导出为 ERASOR2 所需的扫描、位姿和时间文件。 |

新增布局集中在以下目录：

```text
src/map_edit/                    ROS 2 / RViz 编辑插件、launch、配置和测试
tools/mapping/                  独立工具根目录（COLCON_IGNORE 隔离 ERASOR2）
  configs/                      录包 QoS、MID360 起步参数、外参与固定对齐示例
  pointlio_export/              rosbag2 导出与合成验收工具
  erasor2/vendor/                固定版本 ERASOR2 源码和 Licence
  scripts/                      显式录制、导出、构建、运行和验收入口
  .build/                       独立构建产物（忽略）
  .venv/                        独立 Python 环境（忽略）
docs/mapping_pipeline.md         本交接文档
```

`map_edit/package.xml` 的上游许可字段仍是 `TODO`，没有随包提供可确认的许可证文本；在重新发布这部分源码前需由项目维护者确认许可。ERASOR2 的 GPL-3.0 文本保留在 `tools/mapping/erasor2/vendor/Licence`。

## 依赖和构建

目标系统为 Ubuntu 24.04、ROS 2 Jazzy。当前机器已有 Jazzy 和 C++ 构建依赖；本文只给出安装命令，不在本次交接中改动系统包。

如目标机器缺少 ROS 或 map_edit 依赖，可在仓库根目录手工安装：

```bash
sudo apt install ros-jazzy-desktop ros-dev-tools
rosdep install --from-paths src/map_edit --ignore-src -r -y
```

ERASOR2 的 C++ 依赖包括 CMake、Eigen、MPI、PCL、Boost、OpenCV 和 yaml-cpp；缺少时安装：

```bash
sudo apt install build-essential cmake libeigen3-dev libopenmpi-dev libpcl-dev \
  libboost-system-dev libboost-filesystem-dev libopencv-dev libyaml-cpp-dev
```

所有构建命令从仓库根目录运行。map_edit 只扫描 `src/map_edit`，使用独立 build/install/log 目录；默认 CMake 并发为 2、colcon worker 为 1。ERASOR2 单独构建到 `tools/mapping/.build/erasor2`，Python 依赖装进 `tools/mapping/.venv`，不会写入系统 Python：

```bash
./tools/mapping/scripts/build_map_edit.sh
./tools/mapping/scripts/setup_erasor2_env.sh   # 首次运行一次
./tools/mapping/scripts/build_erasor2.sh
```

## 采集、导出和 ERASOR2 处理

先按现有方式启动 Point-LIO。录包脚本只记录正在发布的点云和 `/aft_mapped_to_init`，不会启动、停止或改写 Point-LIO。默认点云话题是 `/cloud_registered`；也可选 `/cloud_registered_full`。输出目录必须是新路径，按 Ctrl-C 结束录制：

```bash
source /opt/ros/jazzy/setup.bash
./tools/mapping/scripts/record_pointlio.sh /data/bags/site_run_01
# 需要全量点云时：
./tools/mapping/scripts/record_pointlio.sh /data/bags/site_run_02 --cloud-topic /cloud_registered_full
```

导出脚本和下文的合成 smoke 都使用 ROS 的系统 Python，因此每个新终端都需先 `source /opt/ros/jazzy/setup.bash`；ERASOR2 的独立 Python 环境由脚本选择，无需激活到这个终端。导出到新的 sequence 目录：

```bash
source /opt/ros/jazzy/setup.bash
./tools/mapping/scripts/export_pointlio.sh \
  /data/bags/site_run_01 /data/sequences/site_run_01 \
  --cloud-topic /cloud_registered --world-frame camera_init
```

默认扫描原点是 body 虚拟原点，默认要求点云与里程计的 `header.stamp` 完全一致。若需真实 LiDAR 原点，显式加 `--scan-frame lidar --extrinsics tools/mapping/configs/pointlio_mid360_export.example.yaml`；示例外参必须按正在使用的 Point-LIO 配置和标定结果核实。只有在准备好了固定的 world 到目标地图变换后，才用 `--alignment <标定文件> --output-world-frame map`；`alignment.example.yaml` 是单位矩阵示例，不能代替实际标定。

导出目录包含：

- `velodyne/000000.bin...`：小端 float32 的 `x,y,z,intensity`，每帧均在扫描原点局部坐标系。默认 `/cloud_registered` 已由 Point-LIO 注册、去畸变和下采样；导出器不会恢复原始 LiDAR 扫描或原始点数。
- `poses_suma_optim.txt`：逐帧直接 `T_world_scan`，每行 3×4、行优先；不是 KITTI 相机坐标或相机外参格式。
- `times.txt`：点云消息头时间；`manifest.json` 记录输入哈希、坐标约定和逐帧匹配审计信息。

不应拿未经核验的 `camera_init` / `odom` 点云直接叠到 `map` 坐标地图上。精确时间戳匹配失败时，默认导出会报错；只有确认误差范围后才显式启用 `--match nearest --tolerance-ms ...`，或使用 `--match interpolate --max-gap-ms ...`。插值只在相邻位姿之间进行，不外推；录包接收时间不会代替消息头时间。

运行 ERASOR2 时，输入 sequence 必须是上述格式，输出目录需是新路径并位于原始 sequence 目录之外。默认输出在 sequence 的同级目录：

```bash
ERASOR2_OUTPUT_DIR=/data/results/site_run_01 \
  ./tools/mapping/scripts/run_sequence.sh \
  /data/sequences/site_run_01 ./tools/mapping/configs/mid360_start.yaml
```

`mid360_start.yaml` 是待实测调参的起点，尤其要按扫描原点校准 `sensor_height`、范围和地面参数。运行结果写入独立目录。`results/` 下有 ERASOR2 的静态估计 PCD（`mapping_0_frame_0_to_<末帧>_streaming_estimated.pcd`）、累计原始/体素地图和 `effective_config.yaml`；运行目录根部有 `requested_config.yaml`、`preprocessing.json`、`run_manifest.json` 和 `run.log`。`input/mapping/velodyne` 指向原始扫描目录，流程只读取这个链接；位姿副本和本次新增标签放在独立运行目录。脚本拒绝覆盖已有输出目录。若要重跑或读取 `input/mapping/velodyne`，原始 sequence 路径及其扫描文件需保持可用；已经生成的 PCD 是独立文件，复制到其他位置后可以单独使用。

Patchwork++ / HDBSCAN 预处理为完整导出扫描生成地面和实例标签；随后 ERASOR2 的 C++ dataloader 按 `x²+y² < robot_body_size²` 裁掉扫描原点附近点。输出 MOS 标签按这一步裁剪后的扫描排列，点数可能少于原始扫描，不能按原始 PointCloud2 的索引直接套用。`run_manifest.json` 的 `mos_frame_counts` 逐帧记录原始点数、裁剪后点数、被裁点数和 MOS 标签数；包装器在发布结果目录前核验标签长度。静态 PCD 的 `intensity` 字段会被算法用作地面/实例编码，不再表示原始反射强度。

## map_edit 地图编辑

先构建，再打开一个已有 OccupancyGrid YAML。以下用仓库中的仿真地图作示例；实际使用时换成目标地图路径：

```bash
./tools/mapping/scripts/build_map_edit.sh
./tools/mapping/scripts/start_map_edit.sh \
  --map-file src/pb2025_nav_bringup/map/simulation/RMUC2026.yaml
```

也可以不传 `--map-file`，启动后在面板里选本地 YAML。地图编辑器不从 PCD 自动生成二维占据栅格；要编辑 2D 地图，先加载已有 Nav2 YAML，再用橡皮擦修改占用格，最后另存为新的 YAML 和配套图像。PCD 面板可加载 ERASOR2 输出并按 Z 高度过滤显示，但它只发布 RViz 辅助点云，不会写出过滤后的 PCD。

编辑工具中，左键将笔刷区域设为占用（黑），右键设为自由（白）；Ctrl+Z 撤销，Ctrl+Y 或 Ctrl+Shift+Z 重做，按住 Shift 再点击左键或右键，会连接同一画笔模式的上一个点与当前点击点，画出占用或自由直线。区域工具按住 Shift 左键逐点添加多边形顶点，右键结束当前区域。

地图输入订阅为 `/map_edit/source`；预览、PCD、标记和区域输出分别使用 `/map_edit/preview`、`/map_edit/pcd`、`/map_edit/markers`、`/map_edit/regions`。默认不订阅现有 `/map`。需要在线地图作只读参考时，显式运行：

```bash
./tools/mapping/scripts/start_map_edit.sh --online-reference \
  --reference-topic /map --use-sim-time true
```

区域工具在当前地图的 XY 平面绘制多边形，默认 `frame_id` 为 `map`、点的 `z` 为 0；完成后另存 `regions.yaml`。文件记录区域 ID、frame、类型、备注和点坐标。它目前只供编辑和导出，没有导航端消费者，也不会自动转换成禁行区或其他导航语义。

地图另存会生成新的 YAML 和配套图像：Nav2 `trinary` / `raw` 模式写 PGM，`scale` 模式写支持透明 unknown 的 PNG。原 YAML、图像和已有输出路径受保护；会保留原地图元数据和高程栅格引用，但不会创建或编辑高程数据。map_edit 与 PCD 都按 `map` frame 显示，PCD 数值坐标本身不会被编辑器变换；导入前先让 PCD 与地图处于同一坐标系。仿真时钟选项默认关闭，仿真数据使用时显式打开。

## 输出接入现有导航

编辑器只发布 `/map_edit/preview` 等隔离话题，保存的地图不会自动替换导航地图。确认文件和坐标后，在计划中的既有实机启动命令里手工填写路径，例如：

```bash
ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py slam:=False \
  map:=/absolute/path/to/edited_map.yaml \
  prior_pcd_file:=/absolute/path/to/static_estimated.pcd
```

`map` 载入二维 YAML；`prior_pcd_file` 载入已有导航配置使用的先验点云。传入 PCD 前必须确认它在该导航配置期望的固定地图坐标系中。`regions.yaml` 不会被上述启动参数读取，接入前需要单独实现并验证消费者。

## 验收范围与限制

从仓库根目录可以重复运行合成验收。先完成上面的本地依赖环境和构建，再在 ROS 已 source 的终端执行；`--output` 必须是尚不存在的目录，省略时自动使用临时目录：

```bash
source /opt/ros/jazzy/setup.bash
./tools/mapping/scripts/smoke_mapping.sh --output /tmp/mapping-smoke-new-run
```

脚本会实际生成 sqlite3 rosbag2、导出 body 和带标定外参的 LiDAR 序列，再调用真实 Patchwork++ / HDBSCAN 和 C++ mapgen / ERASOR2。它检查局部坐标及固定对齐、输入文件哈希、MOS 长度、非空静态 PCD、已有输出拒绝和中途失败清理。结果记录在所选目录的 `acceptance.json`，各次验收使用不同的新目录。

本次最终独立 Jazzy 构建退出码为 0；CTest 两个测试目标全部通过（2/2），其中地图目标含 2 个 Nav2 读写/往返用例，插件目标覆盖 RViz 类实例化和工具销毁后的注册清理。最终 launch 参数检查通过，使用独立启动脚本读取 RMUC2026 地图数据为 583×300；此处确认的是数据加载，栅格视觉显示仍见下述环境限制。离线模式对 `/map` 的发布和订阅均为 0，在线参考模式为 0 发布、1 订阅。`use_sim_time:=true` 已在 RViz、橡皮擦、区域和面板节点上验证。Point-LIO→ERASOR2 合成 smoke 覆盖 8 帧、每帧 3300 点，包括 8 组 body 局部坐标和 8 组非零 LiDAR 外参与固定对齐比较；每帧 MOS 经车体半径裁剪为 3298 点，不对应原始 3300 点索引。静态 PCD 为 21837 点、239704 字节。时间戳错配会失败并清理暂存目录，输入 bag/sequence 哈希保持不变。最终 CTest 明细在 `tools/mapping/.build/map_edit/build/map_edit/Testing/Temporary/LastTest.log`，最终构建日志在 `tools/mapping/.build/map_edit/log/build_2026-10-10_01-55-34/`；早期运行隔离检查在 `/tmp/navi-map-edit-final-runtime-probe.log`，点云流程证据在 `/tmp/mapping-acceptance-delivery/acceptance.json`。没有真实机器人、人车场地、定位效果或导航语义消费验证；ERASOR2 输出应由使用者查看并按场地调参，不能据此宣称实际清障或导航有效。

本次还逐项核对了 1410 个原有普通文件的 SHA256，全部与任务开始时基线一致；父仓库 index、已有 tracked 状态和 6 个子模块的 HEAD/状态也保持不变。基线中的 6 条 `MISSING src/...` 是子模块目录项，已按 gitlink 单独核对，不作为普通文件哈希行处理。报告在 `/tmp/navi-final-scope-review.json`，原始基线在 `/tmp/navi_handoff_validation_baseline/`。

### 当前 WSL 图形环境的验收边界

最终独立启动在当前 WSL2、`DISPLAY=:0` 环境读取地图成功，并正常响应退出；RViz 报告 OpenGL 4.5，但首帧地图出现 `indexed_8bit_image` 的 GLSL sampler 类型冲突日志。使用的是 RViz 默认 `rviz_default_plugins/Map` 显示和系统材质；本插件没有修改该 shader。这个现象在 [RViz 上游问题 #463](https://github.com/ros2/rviz/issues/463) 已有记录，上游报告也说明首帧告警可能与正常地图显示并存；[Jazzy 上游问题 #1279](https://github.com/ros2/rviz/issues/1279) 在其他设备上也出现同样日志。因此暂不能把原因限定为 WSL 驱动，也不能单凭数据加载或这条日志判断栅格绘制是否正确。

本机另做了 `LIBGL_ALWAYS_SOFTWARE=1` 对照，仍出现同样的一次首帧日志，未将软件渲染设为默认。Qt `QScreen::grabWindow` 获取的 RViz 窗口截图整张为黑；同一抓屏路径对已知红色 Qt 控制窗口也返回全黑，而控件自身 `grab()` 正确生成红色图像，说明该抓屏路径在当前显示环境无法用于视觉验收，不能据此认定 RViz 窗口实际全黑。控制记录为 `/tmp/navi-qt-capture-control.log`，RViz 抓屏和独立进程退出记录为 `/tmp/navi-final-rviz-map.png`、`/tmp/navi-final-visual-launch.log`。

本次未完成桌面栅格显示、笔刷反馈和点云叠加的人工视觉验收。使用者需在自己的显示环境检查地图是否完整、平移缩放是否正常以及编辑预览是否与保存结果一致，再用于实际地图编辑。默认渲染日志为 `/home/zhang/.ros/log/rviz2_104627_1791568758134.log`，启动/退出记录为 `/home/zhang/.ros/log/2026-10-10-01-59-18-022375-powercenter-104531/launch.log`。这不影响已通过的地图数据读写、插件生命周期和独立点云处理测试；也不构成对实际界面渲染效果的验证。

## 回退

新增内容集中在 `src/map_edit/`、`tools/mapping/` 和本文档。删除这三个新增路径即可回退源码；`tools/mapping/.build`、`.venv` 及外部数据输出目录可按需单独删除。
