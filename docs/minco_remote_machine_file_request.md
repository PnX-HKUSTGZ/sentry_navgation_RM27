# MINCO 导航缺失文件核查请求

请在“MINCO 仿真能正常启动”的电脑上只读检查仓库和实际运行环境，找出当前电脑缺失的 ROG 动态障碍层插件文件及其正确版本。请不要先修改或覆盖文件；先把核查结果、文件路径和版本发回。

## 当前电脑已确认的情况

- 主仓库最新提交：`3196248`。
- 仿真可以启动：Gazebo 成功创建机器人，地图服务器加载 `RMUC2026` 并进入 active。
- Nav2 导航启动失败发生在 costmap 配置阶段。局部、全局 costmap 都请求加载 `pb_nav2_costmap_2d::RogDynamicObstacleLayer`，但 pluginlib 报告该类不存在。
- 仿真参数在以下文件中声明了该插件：
  - `src/pb2025_nav_bringup/config/simulation/minco_params.yaml`，局部 costmap 约第 170–172 行，全局 costmap 约第 207–209 行。
- 当前电脑安装目录中的 `pb_nav2_plugins` 插件清单只登记了 `pb_nav2_costmap_2d::IntensityVoxelLayer`，没有登记 `RogDynamicObstacleLayer`。
- 主仓库记录的 `src/pb_nav2_plugins` 子模块提交是 `60b82d12faad6974657d29a4d984f77d481df980`；当前电脑子模块实际处于 `3364270bcfe26687ef5ede0ce6d0a4b9e7abca8b`，且多个跟踪文件显示为删除，包括 `costmap_plugins.xml`、`CMakeLists.txt`、`package.xml` 和 `src/` 下的源码。
- 当前电脑日志 `/tmp/rm27-simulation-latest.log` 中的关键错误是：
  - `class pb_nav2_costmap_2d::RogDynamicObstacleLayer ... does not exist`
  - local/global costmap 的 `configure` 回调失败
  - 后续 controller 仍为 `unconfigured`，导航 lifecycle manager 报 `Failed to bring up all requested nodes. Aborting bringup.`

## 已下载目录的补充核查

当前工作区旁边的 `sentry-navigation-RM27-find/` 是从正常电脑下载来的副本。核查发现它也没有完整下载插件子模块：

- 副本主仓库是 `3196248`，gitlink 指向正确的插件子模块提交 `60b82d12faad6974657d29a4d984f77d481df980`。
- 副本的 `src/pb_nav2_plugins/costmap_plugins.xml` 已声明 `RogDynamicObstacleLayer`，`CMakeLists.txt` 也引用了其实现文件；但 `src/layers/`、`src/behaviors/` 和 `include/pb_nav2_plugins/` 目录为空。
- 子模块 `.git` 元数据不完整：虽然 HEAD 字符串显示为 `60b82d1`，`git cat-file -t HEAD` 无法读取对应 Git 对象。
- 副本中 `build/pb_nav2_plugins/liblayers.so` 只有 524288 字节，但 ELF 节表偏移为 1035480 字节，`readelf` 报节表超出文件末尾；安装目录中的同名库也只有 262144 字节。这些二进制不能作为完整文件使用。
- 副本的历史构建日志显示正常电脑曾编译过 `src/layers/rog_dynamic_obstacle_layer.cpp`，并安装过对应头文件。需要从正常电脑重新拿到源码，而不是从这个下载副本或其中的二进制恢复。

请优先从正常电脑补齐整个 `src/pb_nav2_plugins` 子模块，至少包括：

```text
src/pb_nav2_plugins/src/layers/rog_dynamic_obstacle_layer.cpp
src/pb_nav2_plugins/include/pb_nav2_plugins/layers/rog_dynamic_obstacle_layer.hpp
src/pb_nav2_plugins/src/layers/intensity_voxel_layer.cpp
src/pb_nav2_plugins/include/pb_nav2_plugins/layers/intensity_voxel_layer.hpp
src/pb_nav2_plugins/src/behaviors/back_up_free_space.cpp
src/pb_nav2_plugins/include/pb_nav2_plugins/behaviors/back_up_free_space.hpp
```

最稳妥的补齐方式是让正常电脑在仓库根目录执行 `git submodule update --init src/pb_nav2_plugins`，然后传完整子模块源码或其准确 commit。请同时核实文件来自上述固定提交，或明确说明正常电脑使用了哪些未提交改动。

## 后续恢复与启动核验

用户随后把完整插件源码放在主仓库根目录的 `pb_nav2_plugins/`。已将其中源码、头文件、XML 和构建文件复制到 `src/pb_nav2_plugins/`，并成功构建插件。构建必须使用 `--base-paths src`，避免把工作区根目录的副本和 `sentry-navigation-RM27-find/` 下载副本也扫描成重复包。

当前 overlay 的 costmap XML 已包含 `RogDynamicObstacleLayer`，`liblayers.so` 也导出了该插件。重新启动仿真后，local/global costmap 均成功初始化该图层；导航 lifecycle manager 报 `Managed nodes are active`，`/navigate_to_pose` 和 `/follow_path` action 可见。

注意：当前 `src/pb_nav2_plugins` 的 Git 子模块元数据仍与主仓库 gitlink 不一致，子模块 HEAD 仍为 `3364270`，而主仓库要求 `60b82d12...`；恢复的源码可构建运行，但在整理提交或清理工作区前，应由正常电脑 Agent 确认该源码对应的准确 commit / diff。不要把这个状态误报为子模块已完成 Git 同步。

## 请在正常电脑上核查并回报

### 1. 仓库和子模块版本

请回报主仓库和插件子模块的实际版本、工作区改动，以及子模块远端地址：

```bash
cd <仓库路径>
git rev-parse HEAD
git status --short
git ls-tree HEAD src/pb_nav2_plugins
git -C src/pb_nav2_plugins remote -v
git -C src/pb_nav2_plugins rev-parse HEAD
git -C src/pb_nav2_plugins status --short
```

如果正常电脑的主仓库不是 `3196248`，也请说明它的提交版本。若插件代码来自未提交改动或另一个仓库，请指出来源。

### 2. 找到插件实现和导出声明

请定位并回报下列文件的实际路径；如文件名不同，请找出等价文件：

- `RogDynamicObstacleLayer` 的 C++ 实现和头文件。
- 声明该类的 `costmap_plugins.xml`（或等价 pluginlib XML）。
- `CMakeLists.txt` 中编译共享库、安装 XML/头文件并导出插件的部分。
- `package.xml` 中的 pluginlib / Nav2 依赖和导出声明。
- 如实现分散在其他包中，请列出所有相关包和文件。

可用以下命令搜索：

```bash
rg -n --hidden 'RogDynamicObstacleLayer|rog_dynamic_obstacle_layer' src
find src/pb_nav2_plugins -maxdepth 5 -type f -print
```

请确认 XML 中的插件 `type`、`base_class_type`、库路径，与 C++ 类的命名空间和类名完全对应。

### 3. 核对“实际运行时”加载的安装产物

请检查正常仿真使用的 ROS 环境，而不只是源码目录，并回报：

- ROS 发行版、`ros2 pkg prefix pb_nav2_plugins` 的输出。
- 实际被加载的 `costmap_plugins.xml` 的绝对路径和内容。
- 插件共享库的绝对路径及文件名。
- 运行时插件清单是否真的包含 `pb_nav2_costmap_2d::RogDynamicObstacleLayer`。
- 启动时局部/全局 costmap 加载该插件的日志片段。

可以检查当前安装前缀下的 XML 和库，例如：

```bash
ros2 pkg prefix pb_nav2_plugins
find "$(ros2 pkg prefix pb_nav2_plugins)" -type f \
  \( -name '*plugin*.xml' -o -name 'lib*.so' \) -print
```

如果正常电脑的源码里有该类，但实际运行使用的安装前缀没有它，请同时提供实际 source 的 `setup.bash` 顺序或相关 `AMENT_PREFIX_PATH`，以便定位 overlay 不一致。

### 4. 给回当前电脑所需文件

请整理一份最小文件清单，至少包含：

1. 插件类实现和头文件；
2. 插件 XML；
3. 对应 `CMakeLists.txt`、`package.xml` 及必要依赖；
4. 这些文件所属的准确仓库和 commit SHA；
5. 若有未提交改动，提供对应 diff 或说明。

优先提供源码和版本信息，让当前电脑在本机重建。不要只提供另一台电脑的 `build/` 或 `install/` 二进制目录；如插件依赖其他未包含的源码，也请一并列出。

## 期望的回报格式

```text
主仓库 commit：
插件仓库/子模块 URL：
插件仓库 commit：
工作区是否有相关未提交改动：
RogDynamicObstacleLayer 实现文件：
插件 XML：
共享库及构建目标：
正常运行时使用的安装前缀：
实际插件清单是否包含 RogDynamicObstacleLayer：
最小需要带回的文件/补丁：
```
