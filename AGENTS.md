# AI 开发指南

本文件适用于整个仓库。面向开发者的介绍和运行方法见 [README.md](README.md) / [README_en.md](README_en.md)。先读取本文件，再进入相关包；若后续新增更深层的 `AGENTS.md`，同时遵守其目录范围内的约定。

## 1. 开始工作

1. 阅读用户任务，运行 `git status --short`，保留已有修改。先确定任务是文档、参数、估计、规划、控制、仿真还是离线实验。
2. 使用 `rg --files`、`rg -n` 找实际入口；以 `package.xml`、CMake、当前 launch 和最终解析的 YAML 为依据。`docs/` 与各包 README 含历史设计，不能直接当作当前实现。
3. 检查 `git submodule status` 和 `colcon list --base-paths src`。驱动及旧版 Nav2 插件是子模块，不要把缺失依赖误判为当前代码故障。
4. 修改前选定定位来源、导航模式、地图、时间源及测试方法。实车/仿真配置分开维护；需要同步时说明原因。
5. 最小化改动范围，不为通过测试而删除门限、放宽断言或重写无关文件。发现任务外的问题应记录证据和影响。

默认用中文与用户沟通；源码符号和 ROS 接口保持项目已有英文命名。根文档变更同步中英文事实、命令和默认值。

## 2. 环境与构建约定

当前完整工程面向 **Ubuntu 24.04 / ROS 2 Jazzy / Gazebo Harmonic**。`Dockerfile`、CI、部分旧包文档保留 Humble；它们不能证明当前工程已在 Humble 通过。

从仓库根目录执行：

```bash
source /opt/ros/jazzy/setup.bash
# 已有构建时再加载本地依赖；迁移/重命名后先检查旧 overlay 路径。
source install/local_setup.bash
colcon list --base-paths src
CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build --base-paths src \
  --symlink-install --executor sequential --packages-up-to point_lio \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
source install/setup.bash
```

全新工作区没有 `install/` 时省略对应 source。`--packages-up-to` 会带上依赖，`--packages-select` 只适用于依赖已构建的环境。不要仅凭当前 shell 中其他工作区的安装产物声称干净构建成功；检查 `ros2 pkg prefix <package>`、CMake cache 与 overlay 顺序。

- 包名以 `package.xml` 为准：`src/pnx-localization/backend` 是 `rm27_backend`；`localization_interfaces` 是 `rm27_localization_interfaces`。
- 主要代码为 C++17、Python 3、ament/colcon；`relocalization_manager` 的 CMake 当前声明 C++14，修改时检查该目标实际编译设置。
- GTSAM 4.2、TBB 用于独立后端；Point-LIO 联合初始化使用 yaml-cpp、nanoflann、glog。不要因为前端依赖共享消息而让它依赖后端求解器。
- small_gicp 由 FetchContent 获取：全局包固定 `fa0cfc983417c044b6036f71dfa448038bb9c820`，管理器当前跟踪 `master`。离线缓存必须符合调用脚本的版本检查；不要悄悄升级算法依赖。
- `.pre-commit-config.yaml` 含会修改文件的 Ruff/clang-format hooks。只检查任务范围，避免全仓自动格式化第三方代码和参数。

## 3. 按任务定位代码

| 任务 | 首先检查 |
| --- | --- |
| 启动、模式选择、参数路由 | `src/pnx-launch/launch_bringup/launch/`，特别是 `bringup_launch.py`、`navigation_launch.py`、`rm_navigation_*_launch.py` |
| 实车/仿真参数 | `src/pnx-launch/launch_bringup/config/{reality,simulation}/{nav2_params,minco_params}.yaml` |
| LIO、初始化、IMU 数值更新 | `src/pnx-localization/point_lio/src/`、`include/`、`test/`；联合核心在 `third_party/zlio_startup/` |
| 局部先验匹配、`map → odom` | `src/pnx-localization/relocalization_manager/` |
| 全局检索与比赛确认 | `src/pnx-localization/global_relocalization/`、`global_relocalization_interfaces/action/Relocalize.action` |
| 全局初始化导航交接 | `launch_bringup/scripts/relocalize_then_navigate.py`、`global_pose_handoff.py`、`rm_27_stimulation/launch/sim_with_nav.launch.py` |
| 前端原子帧、关键帧、位姿图 | `src/pnx-localization/localization_interfaces/`、`backend/`；读后端 README 和 VALIDATION |
| 里程计、点云坐标转换 | `src/pnx-localization/{loam_interface,sensor_scan_generation}/` |
| 占据、地形、净空、动态障碍 | `src/pnx-perception/rog_map/`，尤其 `projection_layer.cpp`、先验与查询模块 |
| MINCO 轨迹、会话、碰撞门 | `src/pnx-navigation/minco_planner/` |
| MPC、输入验证、轨迹授权 | `src/pnx-navigation/minco_controller/` |
| 最终速度坐标转换、停车门 | `src/pnx-navigation/fake_vel_transform/` |
| 仿真传感器/物理/机器人 | `src/pnx-stimulation/rm_27_stimulation/{launch,config,urdf,src,tools,test}/` |
| 仿真点字段和扫描时间 | `src/pnx-stimulation/ign_sim_pointcloud_tool/src/point_cloud_converter.cpp` |
| 离线先验、回放、评估 | `tools/global_relocalization/`、`tools/mapping/`；实验 CMake 子目录有 `COLCON_IGNORE` |
| 资源/性能回归 | `tools/stimulation/benchmark_*.py`、`trace_odometry_pipeline.py` |

表中 `launch_bringup/...`、`rm_27_stimulation/...` 为上方完整包目录的简写，不是根目录下的路径。

## 4. 启动与配置合同

- 只支持 `navigation_mode:=legacy` / `minco`。`minco_shadow` 已删除。两种模式互斥、启动时决定，不实现在线自动切换。
- 单机器人 `namespace` 保持默认空值，命令行省略该参数。非空值会被 launch 拒绝；不要通过只 remap 部分传感器/TF 来声称支持多机器人。
- 普通仿真默认 `use_ground_truth_odom:=true`，会跳过 Point-LIO 与 GICP 链。测试 LIO、定位精度或后端时显式设 `false`。
- `slam:=true` 只支持 `legacy`，且与 `use_ground_truth_odom:=true` 互斥。
- `params_file` 按部署和模式选择完整 YAML。MINCO 要求仅注册 `MincoPlanner` / `MincoMpc` 的完整 profile，不能把零散覆盖文件当主配置。
- 仿真 launch 会应用 `simulation_world_profiles`，MINCO 的 Point-LIO 模式还会应用 `minco_input_profiles` 并生成临时 YAML。检查最终参数及运行时 `ros2 param`，不能只读源文件某一段。
- `enable_legacy_terrain:=auto` 在 legacy 开、MINCO 关。旧 terrain guard 和 MINCO 的 ROG freshness 门分别维护，不能在一条输入断开后默默退化为无动态障碍保护。
- `sim_with_nav.launch.py` 不转发所有内部参数，例如普通 `map` / `prior_pcd_file`；自定义地图应检查具体导航入口，使用 `--show-args` 确认。
- 全局组合流程仅支持 RMUC2027、MINCO、非 SLAM，比赛配置搜索负 X 半场。脚本默认 domain 87，独立 overlay、GZ partition 和 DDS profile；与普通工作区构建分开核对。

## 5. 坐标、时间与状态身份

这些是跨包合同；改动任何一项都要检查上下游。

- 普通 Point-LIO 定位模式的 `map → odom` 由 `relocalization_manager` 发布；GT 模式由 `rm27_ground_truth_localizer` 发布；SLAM 模式由 slam_toolbox 发布。每种模式只保留一个所有者，不能再补一条静态 TF。
- Point-LIO 输出原生世界系 `camera_init` 和 IMU 状态 `body`；`loam_interface` 建立导航 `odom` 对齐并发布 `/registered_scan`、`/lidar_odometry`。`sensor_scan_generation` 利用同刻机械 TF 发布底盘 TF 与 `/odometry`，不能仅改 frame 字符串冒充坐标变换。
- `/lidar_odometry`、`/odometry`、`/aft_mapped_to_init` 描述的参考点和速度坐标不同。杆臂、固定安装角和活动云台必须按实际 TF 处理；保留 twist 与协方差语义。
- 仿真雷达当前挂在 `gimbal_yaw`，默认 `[0,0.126,0.130] m`、roll `-30°`。实车固定模型与驱动 JSON 是另一套配置，不能复制仿真参数。驱动补偿与 TF 不能重复施加同一旋转。
- 全局确认位姿保留原始时间戳；管理器查询同刻 `odom → base_footprint`，确认接收后才激活导航。不要改为最新 TF、重新盖时间戳或把 RViz 出生点当定位结果。
- 时间判断区分传感器 stamp、ROS 仿真时间和单调墙钟。处理回退、重复、乱序、未来时间与接收超时，不能用增加 timeout 掩盖时间合同错误。
- 原子 `FrontendFrame` 的里程计和完整世界系点云必须来自同一扫描、同时间戳，保留 instance/epoch/sequence。后端拒绝重复、旧代和乱序，连续性中断创建新 segment，不能跨重置连接里程计边。
- 后端局部协方差使用 RIGHT 扰动 `[rho,phi]`，GTSAM 使用 `[phi,rho]`；前端位置与旋转误差的参考系不同，按现有适配器转换。
- 后端现在只做旁路图，默认导航不消费其估计，不广播 TF。增加地图/回环约束时须实现实际测量生产、几何验收、身份匹配和资源边界，不能把现有接口称为已完成全局优化。

## 6. 控制、地图与资源边界

- 主速度链为 `/cmd_vel_controller → velocity_smoother → /cmd_vel_nav2_result → fake_vel_transform → /cmd_vel`。保留 MINCO 会话 token、BLOCK、lease、输入新鲜度和碰撞检查；旧目标、旧轨迹或失效输入不能重新获得控制权。
- 全局初始化临时使用 `/relocalization/cmd_vel` 经协调器控制，成功交接后销毁该速度入口，失败/超时停车。不要与正常导航同时向底盘抢发运动命令。
- MINCO 不应添加绕过 MPC/ROG 门的运动 behavior；`fake_vel_transform` 的 `/cmd_spin` 兼容入口也不能绕过停车结果。改变模式时先取消任务并停止当前启动链。
- 地图参数要同时核对分辨率、车高/车宽、footprint、物理 collision、自身滤波、地面支撑、净空、unknown 策略和实测障碍优先级。不能仅调小机器人尺寸或把未知变空闲来通过窄通道。
- PGM 的 `negate`、`mode` 和占据阈值决定灰度是否未知；不要凭颜色判断可行驶。
- PCD、manifest、栅格地图、场景 collision 与外参必须匹配。RMUC2027 世界现在使用从实际 RM27 visual 网格生成的 `RMUC2027.pcd`，该 PCD 已在 map 系，RMUC2027 profile 不再叠加 1.65 m 补偿。旧官方 CAD PCD 的严格对齐失败结论仍保留在历史报告中，不能与当前仿真网格点云混淆。
- 全局定位 GT 只用于独立评分，不能流入估计、候选筛选或训练；固定 holdout 分母和失败记录，不能删失败样本或复用训练点提升成功率。
- `station_submaps` 和离线地图实验独立运行，使用新输出目录，保留原始采集、来源哈希、冻结参数及失败结果，不能覆盖已有实验或偷偷改比赛配置。
- 保留队列、图窗口、点云缓存和存盘预算。后端归档是诊断快照，不能当成完整 iSAM2 会话恢复或已具备历史回环能力。

## 7. 验证与交付

选择与改动相匹配的已有测试；修复行为时补充能捕获真实回归的用例，不为简单文档或低影响编辑编写镜像测试。

```bash
# 已构建且 BUILD_TESTING=ON 的包
colcon test --packages-select point_lio --event-handlers console_direct+
colcon test-result --verbose

# 全局核心 + 工具 + station 合同；此入口会构建全局包
bash tools/global_relocalization/check.bash

# 单独 station Python 合同；不会构建或启动仿真
bash tools/global_relocalization/station_submaps/check.bash

git diff --check
```

| 改动 | 至少关注的测试 |
| --- | --- |
| LIO/IMU/初始化 | `point_lio/test/` 的 adaptive、joint startup、IMU 更新测试；需要时使用初始化/里程计基准 |
| 前后端原子帧/图 | `backend/test/test_backend.cpp`、Point-LIO 测试、`verify_rm27_backend_contract.py`；按需运行后端基准 |
| 坐标/速度/时间 | loam、sensor_scan_generation、fake_vel_transform 包测试及全局 pose handoff / startup heading 测试 |
| 地图/净空/动态障碍 | `rog_map/test/`、`launch_bringup/test/`、仿真碰撞几何测试 |
| MINCO/MPC/授权 | planner 的 session/lease/safety/optimizer 测试，controller 的 input/session/MPC/recovery 测试 |
| launch/模式/场景 | `launch_bringup/test/`、`rm_27_stimulation/test/` 的 startup、DDS、pose handoff；渲染改动再跑 rendering 测试 |
| 全局检索/比赛流程 | 全局 CTest action/competition/core/raw 测试及 `tools/global_relocalization/test_*.py` |

性能或全栈验证必须记录提交、最终配置、地图/manifest 哈希、定位来源、ROS domain、样本数、通过标准、仿真时间和墙钟时间。合成精度、单点仿真和完整导航/实车是不同证据；不能相互替代。CPU 统计说明 100% 是否为一个逻辑核，RSS/PSS 不混用。

已知基线（2026-11-05，完整复现见 [审查记录](docs/repository_review_2026-11-05.md)）：

- 工具和 station submaps pytest：169 通过；指定启动、交接、渲染测试：91 通过、17 失败。
- 失败包括旧球/盒碰撞断言与当前 mesh 模型不匹配，也包括实际参数合同差异。先按测试名对照基线，再判断是否新增回归；不能直接更新期望值掩盖风险。
- 仿真 MINCO `projection.vehicle_height=0.040`，模型 `folded_vehicle_height=0.240`；RMUL2027 灰边阈值也有失败项。修改这些配置应验证物理模型和地图语义。
- 可选 CAD 测试需要独立依赖；系统 Python 缺 `shapely` 时无法收集。当前 checkout 的四个子模块未初始化，已有构建可能借用其他工作区依赖。
- 固定非零 LiDAR–IMU 外参的运行期坐标一致性风险已有记录；不能因联合初始化或后端测试通过就称其已修复。

## 8. 文件、依赖与操作范围

- `point_lio` 是普通目录；四个子模块见 `.gitmodules`。修改子模块或第三方代码前先确认任务涉及它们，保留来源、版本和许可说明。
- 根许可证不是所有文件的统一许可证。ZLIO 启动核心为 GPL-2.0-only，CAD 派生数据另有条款；保留其 attribution 和 manifest。
- `build/`、`install/`、`log/`、`.build/`、数据采集和大多数 PCD 是生成物，不提交；已跟踪的少数 PCD/manifest 是交付资源，不随意删除或重新生成。
- 根 `.gitignore` 忽略整个 `docs/` 目录，文档和验证摘要默认仅保留在本地，不自动强制添加；只有用户明确要求将具体文档纳入版本控制时才可添加。
- 不自动运行 `tools/reality/start_nav.bash`、`tools/stimulation/start_nav_stimulation.bash` 等遗留脚本；其中有固定机器路径和宽泛 `pkill`。优先直接 launch，清理时仅管理当前任务创建的进程。
- 文档、静态审查和常规单元测试不需要启动实车或发布 `/cmd_vel`。需要运行时检查时使用隔离仿真、独立 domain/partition 和新输出目录。
- 不为本地验证自行 push、发布或发信；用户要求时再执行。提交前查看 diff 和状态，只包含本任务文件，报告实际通过、失败、跳过及未验证项。
