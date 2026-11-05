# 仓库审查与文档核对（2026-11-05）

审查基线：`35edb8f`，分支 `replace_gicp`。本次交付为中英文 README、根 `AGENTS.md` 和本记录；未修改算法、导航参数、模型或测试期望。

## 范围与结论

检查了一级功能目录、包清单/CMake、四个子模块、主启动链、四份部署 YAML、机器人/传感器配置、局部/全局定位、前后端接口与实现组织、ROG/MINCO/MPC 安全入口、离线工具、地图资源、现有测试、Docker/CI 和历史验证文档，并复跑下列 Python 回归。

当前主线环境为 Ubuntu 24.04 / ROS 2 Jazzy / Gazebo Harmonic。旧 README 的 Humble/Classic 环境、统一雷达外参、静态建图 TF、PCD 全部外置等说法不符合当前源码，已纠正。默认仿真为真值定位 + legacy；Point-LIO、MINCO、全局初始化、旁路后端是需要分别辨明的能力。

这次检查未执行全新完整编译、重新编译原生测试、完整 Gazebo 导航或实车验证；未逐行审计全部第三方实现。已有生成物或历史报告不能作为本轮重新验收证据。

## 需要处理的发现

以下优先级用于安排后续修复：P1 为影响地图/碰撞判断的合同问题，P2 为构建可重复性、覆盖或部署维护问题。运行影响中尚未经全栈复现的部分明确标为风险推断。

### P1：仿真净空车高与机器人模型不一致

证据：

- [仿真 MINCO 配置](../src/pnx-launch/launch_bringup/config/simulation/minco_params.yaml) 中 `planner_server.MincoPlanner.rog_map.projection.vehicle_height: 0.040`、`headroom_margin: 0.0`、`body_bottom_clearance: 0.03`。
- [机器人 Xacro](../src/pnx-stimulation/rm_27_stimulation/urdf/simulation_waking_robot.xacro) 定义 `folded_vehicle_height=0.240`。
- [projection_layer.cpp](../src/pnx-perception/rog_map/src/rog_map/projection_layer.cpp) 的 `knownRatioInBodyVolume` 使用 `ground_z + vehicle_height + headroom_margin` 计算车体净空上界。
- `test_simulation_minco_gate_clearance_accounts_for_voxel_quantization` 在 `0.04 != 0.24` 处失败；另一整车净空测试先在旧 collision 结构解析处失败，未完成实际包络验证。

风险推断：当前配置不能支持“整车 24 cm 净空已验收”的结论；具体行为还受先验通行策略和实测障碍分支影响，不能只凭参数差异断言某一场景必撞。后续应从实际 mesh 包络、投影策略和坡面姿态共同确定车高与裕量，测试低顶障碍及已知/未知列，不能仅修改断言。

### P1：RMUL2027 地图将预期未知灰边解释为空闲

[RMUL2027.yaml](../src/pnx-launch/launch_bringup/map/simulation/RMUL2027.yaml) 使用 `mode: trinary`、`negate: 0`、`free_thresh: 0.35`。PGM 灰边像素为 205，对应占据概率 `(255-205)/255 ≈ 0.196`，低于空闲阈值，因此该灰边会落入空闲范围。

[test_rmul2027_maps.py](../src/pnx-launch/launch_bringup/test/test_rmul2027_maps.py) 明确要求灰边保持 UNKNOWN，当前断言失败。后续应按地图生成约定恢复未知语义，核对 map_server 实际 OccupancyGrid 与边界规划，不应只去掉该断言。本次未启动 map_server 对运行时地图再次采样。

### P2：部分测试与模型及部署参数已失配

17 项失败中，多个用例仍假定球体车轮、方盒云台/上盖，当前 Xacro 已使用 CAD mesh；高程生成器仍寻找旧 `map` 模型。另有恢复速度、地面台阶、MPC/优化器、足迹补偿和观测范围的参数断言失败。

这同时包含测试维护和实际跨参数合同问题。需要逐项辨明设计意图、生成资源版本、最终 launch 参数和物理效果，再修复测试或实现。2026-11-02 的[移植记录](global_relocalization_merge_20261102.md) 已报告目标分支有 17 项基线失败，但它不是免除本轮修复责任的依据；本次没有重新检出当时基线逐项比较。

### P2：Docker 和 CI 与当前完整工程不一致

- [Dockerfile](../Dockerfile) 基于 Humble，构建时重新 clone 固定远端 URL，不会直接构建本 checkout 的源码变更；依赖和下载也沿用历史流程。
- [构建 CI](../.github/workflows/build_and_test.yml) 使用 Jammy/Humble，目标是 `pnx_sentry_nav`。
- [集合包清单](../src/pnx-launch/pnx_sentry_nav/package.xml) 没有把 `launch_bringup`、全局定位和独立后端列入该目标依赖，不能据此认为完整启动链受测。

后续应统一支持环境、容器源码来源和 CI 构建/测试目标。本次没有运行 Docker build 或远端 CI，因此没有把静态不一致写成已观察到的 CI 失败。

### P2：small_gicp 依赖版本不统一

[全局包 CMake](../src/pnx-localization/global_relocalization/CMakeLists.txt) 固定提交 `fa0cfc983417c044b6036f71dfa448038bb9c820`；[管理器 CMake](../src/pnx-localization/relocalization_manager/CMakeLists.txt) 使用 `GIT_TAG master`。相同源码在新的网络构建中可能得到不同依赖，且两处可能使用不同版本。

后续应选定共同版本、验证数值/接口后统一固定。全局构建脚本对本地缓存有精确 revision 检查，不能用其他缓存绕过。本次未更新依赖。

### P2：实车描述、补偿和辅助脚本需要部署校准

[实车固定描述](../src/pnx-launch/sentry_robot_description/urdf/sentry_robot.urdf.xacro) 的雷达位置默认 `[0,-0.20,0.30] m`、零 roll；[驱动 JSON](../src/pnx-launch/launch_bringup/config/reality/mid360_user_config.json) 补偿 roll 为 `45.0°`；仿真雷达则挂在活动 `gimbal_yaw`，默认 `[0,0.126,0.130] m`、`-30°`。这些数值不能用一个旧安装描述统括，也不能据此声称已经实车标定。

实车 launch 的 `use_robot_state_pub` 实际默认 `true`，旧 README 曾写 `false`，容易与完整机器人系统重复发布 TF。本次在 README 示例中显式关闭并说明前提。

[实车启动脚本](../tools/reality/start_nav.bash) 与[仿真启动脚本](../tools/stimulation/start_nav_stimulation.bash) 含机器绝对路径和宽泛 `pkill`；本轮没有执行它们。后续宜改为按当前任务跟踪进程及相对工作区路径。

## 已实现能力与边界

- `relocalization_manager` 做局部 GICP 验证和 `map → odom` 管理，不做无初值全局检索。
- 全局重定位有 raw LiDAR/IMU 前端、检索、比赛状态与停车确认，接入导航需 timestamped handoff。瞬时扫描、半场搜索和活动云台假设仍需区分；既有独立演示的固定云台模型不能代替正式导航模型验证。
- `rm27_backend` 有原子消息校验、关键帧、局部云、有界图/缓存和存盘，地图/回环接口已有实现，但没有测量生产者，不改导航 TF，也不消除纯前端漂移。
- Point-LIO 有联合初始化和 IMU 更新数值保护，当前四份部署参数使用 `joint`；仿真 8 帧，实车 20 帧，其他细节依 profile 而异。不能将不同 profile 写成同一组种子样本/线程参数。
- RMUC2026/RMUL2027/RMUC2027 的部分 PCD 已在 Git 中；实车 PCD 需要用户提供。RMUC2027 官方 CAD PCD 的严格三维兼容性失败是已保存结论，不可替换成“完全对齐”。
- 根许可证、各第三方组件及官方 CAD 数据有各自的许可与来源记录，已在 README 和 AGENTS 中提示保留，未作统一重新许可。

## 本轮验证

环境：Ubuntu 24.04.5 / ROS 2 Jazzy，系统 `/usr/bin/python3`（Python 3.12）、pytest 7.4.4。四个子模块当前均未初始化；colcon 发现 22 个本地包。已有 shell/安装环境包含其他工作区依赖，因此以下结果是当前环境中的回归检查，不是全新 checkout 的完整构建结果。

### 工具与 station submaps

```bash
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 /usr/bin/python3 -m pytest -q \
  tools/global_relocalization/test_*.py \
  tools/global_relocalization/station_submaps/test_*.py --tb=short
```

结果：**169 passed in 4.60s**。仅包含上述 Python 测试，不包含全局 C++ CTest、原生 station adapter 或新一次地图采集/回放。

### 启动、交接、模型与渲染

```bash
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
OPENBLAS_NUM_THREADS=1 /usr/bin/python3 -m pytest -q \
  src/pnx-launch/launch_bringup/test \
  src/pnx-stimulation/rm_27_stimulation/test/test_simulation_startup.py \
  src/pnx-stimulation/rm_27_stimulation/test/test_simulation_pose_handoff.py \
  src/pnx-stimulation/rm_27_stimulation/test/test_dds_transport.py \
  src/pnx-stimulation/rm_27_stimulation/test/test_sentry_collision_geometry.py \
  src/pnx-stimulation/rm_27_stimulation/test/test_gimbal_state.py \
  src/pnx-stimulation/rm27_rendering/test --tb=short
```

结果：**17 failed, 91 passed in 8.35s**。其中需要节点的交接测试使用已有安装产物，本轮未重新编译它们。

失败列表如下；前 13 项位于 `test_navigation_launch.py`，第 14 项位于 `test_rmul2027_maps.py`，最后 3 项位于 `test_simulation_collision_footprint.py`：

| 测试名 | 本轮失败证据 |
| --- | --- |
| `test_rmuc2026_ground_elevation_grid_is_current_generator_output` | 生成器要求旧 `ground_plane`/`map` 模型结构 |
| `test_simulation_minco_uses_no_elevation_ground_connectivity` | `min_headroom_known_ratio` 为 `0.1`，断言要求 `0.0` |
| `test_rmuc2026_physics_supports_holonomic_ramp_contacts` | 预期车轮 sphere，但当前为 mesh |
| `test_folded_upper_body_collision_fits_the_actual_cad_top` | 旧上盖 box 节点不存在 |
| `test_simulation_collision_height_and_filter_clear_250mm_roof` | 旧 sphere 节点不存在 |
| `test_overlap_recovery_limits_match_controller[reality]` | 恢复限值 `0.15 != 0.2` |
| `test_reality_global_seed_ignores_rolling_rog_frontier` | 台阶门限 `0.06 != 0.08` |
| `test_minco_uses_one_complete_deployment_profile` | 两份 profile 的 Point-LIO 参数已分化 |
| `test_active_minco_profiles_feed_measured_rog_obstacles_to_nav2_costmaps` | 标记范围 `3.0 != 2.0` |
| `test_reality_short_ramp_speed_and_connectivity_contract` | 参考速度条件 `0.3 > 0.6` 不成立 |
| `test_simulation_minco_gate_clearance_accounts_for_voxel_quantization` | 车高 `0.04 != 0.24` |
| `test_simulation_minco_speed_limit_is_faster_but_stays_inside_mpc_envelope` | 优化器速度 `1.8 != 1.5` |
| `test_active_footprint_bootstrap_covers_one_rog_seed_step` | 足迹补偿长度 `0.82 != 0.924351` |
| `test_grid_uses_the_same_world_coordinates_without_preinflation` | 灰边占据概率小于空闲阈值 |
| `test_all_wheel_spheres_fit_with_four_mm_margin` | 旧 sphere 节点不存在 |
| `test_folded_and_upright_spinning_gimbal_fit` | 旧云台 box 节点不存在 |
| `test_folded_clearance_height_encloses_the_physical_top` | 旧云台 box 节点不存在，未完成净空包络验算 |

直接收集整个仿真测试目录还会在 `test_rmul2027_geometry.py` 因缺少 `shapely` 失败；该文件标注为需要可选 CAD 环境。本轮通过显式列出测试文件完成上述 108 项检查，未安装或修改系统依赖。

原始日志在本机 `/tmp/rm27_repository_review_tools.log`、`/tmp/rm27_repository_review_launch.log`；日志不是版本化交付物，上方保留可复现命令和失败摘要。

### 文档与静态语法检查

四份交付文档的 74 个本地链接均存在，30 个 Bash 代码块通过 `bash -n`，Markdown 代码围栏配对；`git diff --check` 通过。排除第三方、缓存及数据目录后，182 个 Python 文件通过 AST 解析，22 个 `package.xml` 通过 XML 解析。

AST 检查在 `tools/global_relocalization/station_submaps/diagnose_mapping_recovery.py:105` 报告一项 `SyntaxWarning`：`>=0for` 缺少空格。当前 Python 仍能解析；本轮未修改该实验脚本。以上静态检查不替代编译或算法验证。
