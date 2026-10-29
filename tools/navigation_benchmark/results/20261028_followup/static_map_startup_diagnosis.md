# 静态地图启动失败诊断

原始证据：`../20261028/obstacle_speed/repeat_01/launch.log`。该次定位生命周期管理器开始配置 map_server 后，map_server 读入 583×300 的 PGM，但在 1793125513.139 报 `/map_server/change_state` 响应发送超时；此日志没有 map_server 激活或 global StaticLayer 接收、调整地图尺寸的记录。导航生命周期管理器随后配置 planner，1793125514.600 在静态几何检查处失败。成功样本 `../20261028/obstacle_speed/repeat_02/launch.log` 在 planner 插件创建前记录了 global StaticLayer 调整为 583×300。

这说明该失败并非只有规划器内的初始化顺序可以解释：地图服务生命周期响应已先出错，静态地图可能从未发布。旧异常未记录几何，日志也不能证明两地图当时的具体数值或确认唯一根因。不能通过跳过静态检查解决上游地图服务故障。服务响应超时的成因还需要进一步证据；也不能仅凭该警告断言客户端超时的具体原因。

为后续复现增加异常中的 base 和 static 地图尺寸、分辨率、原点；几何一致性阈值和失败行为不变。新增单元测试检查几何不一致仍拒绝构造，并输出双方数值。该日志增强不声称修复地图服务故障，也不能证明启动成功率改善。未运行构建或仿真；需在后续构建运行测试并通过受控重启记录地图服务及两侧几何。

后续处理：`bringup_launch.py` 同时包含 localization 和 navigation 两套启动，二者分别启动各自的 lifecycle manager，没有等待 map_server 激活的依赖。Nav2 Jazzy 的 PlannerServer 在调用插件 `configure` 前启动 costmap 独立 executor；其 StaticLayer 在收到、填充首张地图后，在静态层互斥锁内设置 current。MINCO 现在等待该状态，使用 steady clock 限时 3 秒，确保仿真时钟暂停也能结束；未就绪则明确失败。就绪后按 Nav2 resizeMap 的 master→layer 锁序锁住快照并再次检查就绪，几何不一致仍拒绝。新增测试覆盖延迟就绪和超时，以及几何不一致和诊断内容。这个改动处理发布延迟与读写交错，不能使未激活的 map_server 自行恢复，也未改变两个生命周期管理器的依赖。上述测试和受控重复启动需在构建后执行；3 秒期限内未收到地图的系统仍会安全失败。

验证补充：检查本机 Nav2 Jazzy 对应源码，Layer 构造 `current_ = false`；StaticLayer 首次 `incomingMap` 校验后调用 `processMap`，在静态层锁内填充完成后设置 current。`updateCosts` 仅在 `updateBounds` 已观察到 map_received 时才会将 current 设为 true。新增真实 Costmap2DROS + StaticLayer 测试，在独立 ROS domain 发布延迟到达的 OccupancyGrid：无图时 planner 的 3 秒等待安全失败且无静态查询；地图到达后成功构造并确认 lethal 障碍与相邻硬化净空。单独构建该测试目标后，整个 `test_map_query_adapters` 20/20 通过（无 Gazebo）。该测试没有验证 map_server 生命周期管理器故障恢复。
