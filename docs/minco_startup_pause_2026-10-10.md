# 18:04 起步位置不变事件复查

## 结论

原记录覆盖 8.761 秒。能够确认该目标开始与被下一目标替换时，规划器记录的起点均为 `(-5.496, -1.423)`；缺少连续里程计，不能严格证明中间每一时刻都没有移动。

期间没有持续 BLOCK、碰撞拒绝、净空拒绝或优化失败日志。启动辅助没有出现触发记录，下一目标却立即触发。启动参考与速度死区的配合是代码层面的疑点，但现有记录不足以确认根因。

补做的三次起步检查都在约 0.3 秒内产生至少 1 cm 位移，没有复现该长停。本次没有修改导航代码或参数，没有将它标为已修复。

## 原事件时间线

日期为 2026-10-10，时间为本地 UTC+8。

| 时间 | 记录 | 含义 |
| --- | --- | --- |
| 18:04:38.323 | controller `GOAL_REACHED` | 上一个目标完成 |
| 18:04:38.788 | 上一规划任务心跳租约到期 | 在上一目标完成之后，不能当成后续持续停止的证据 |
| 18:04:41.242 | 新目标 `(3.617, -2.450)` | 当时位置约 `(-5.496, -1.423)` |
| 18:04:41.244 | `BLOCK_COMMAND` | 新任务交接时撤销旧指令 |
| 18:04:41.322 | 全局搜索输入 | 起点与终点的 cost 都为 0；该行本身不等于完整车身检查结果 |
| 18:04:41.665～49.714 | 每约 0.35 秒 `Passing new path to controller` | Nav2 路径/任务刷新，不等于持续有新的有效 MINCO 速度指令 |
| 18:04:50.003 | 用户新目标替换为 `(-4.496, -4.566)` | 前一目标被替换，不能说它自己等待 8.8 秒后恢复 |
| 18:04:50.076 | 新目标全局搜索 | 起点仍记录为 `(-5.496, -1.423)` |
| 18:04:50.215 | 启动参考前探 0.30 秒，指令 `0.101 -> 0.600 m/s` | 新目标触发了启动辅助 |
| 18:04:56.093 | `Goal succeeded` | 替换后的目标完成 |

原始日志：

- `~/.ros/log/bt_navigator_3622910_1791626549307.log`，第 18～21 行。
- `~/.ros/log/planner_server_mt_3622908_1791626549308.log`，第 796～806 行。
- `~/.ros/log/controller_server_3622906_1791626549305.log`，第 415～451 行。

## 启动死区的代码疑点

当时启动日志确认以下配置：

```yaml
deadzone_speed_threshold: 0.05
reference_progress_max_lead_time: 0.05
reference_startup_max_lead_time: 0.85
reference_startup_target_speed: 0.08
reference_startup_min_command_speed: 0.60
```

`buildReferenceFromOptPath()` 在实际速度处于起步死区内时，检查从当前位置向前最多 0.85 秒的轨迹参考，寻找速度达到 0.08 m/s 的采样。

如果该窗口内参考速度一直不足，启动辅助可以不激活；后续正常 MPC 指令若小于 0.05 m/s，`shouldSuppressPlanarCommand()` 会将平移速度清零。这条分支不会调用 `failClosedCommand()`，所以不会产生对应的 fail-closed 警告。

如果局部轨迹开头加速过慢，又持续从静止位置重规划，这套条件存在持续给零速度的可能。这个机制符合“没有持续保护日志、没有启动补偿、位置不变”的表现，但原事件未记录参考速度与最终输出，不能把可能性写成已发生事实。

相关代码：

- `src/navigation/minco_controller/include/minco_controller/reference_progress.hpp`：启动参考搜索、启动指令辅助及死区判断。
- `src/navigation/minco_controller/src/minco_mpc_controller.cpp`：`buildReferenceFromOptPath()` 和 `computeVelocityCommandsImpl()`。

## 隔离仿真复测

使用 ROS domain 83 和独立 Gazebo partition，无 GUI、无 RViz、真值定位，保留当前参数；额外加载临时接触记录插件。所有测试进程结束后关闭。

| 起步情况 | 目标 | 首次指令超过 0.05 m/s | 首次位移达到 1 cm |
| --- | --- | ---: | ---: |
| 直接在 `(-5.496, -1.423)` 启动 | `(3.617, -2.450)` | 0.208 s | 0.308 s |
| 完成返回目标后，从附近 `(-5.422, -1.373)` 再出发 | 同上 | 0.183 s | 0.303 s |
| 连续执行返回目标，再从附近 `(-5.417, -1.367)` 出发 | 同上 | 0.209 s | 0.308 s |

三个离开目标均完成。直接启动的第一条参考轨迹约在 0.65 秒处达到启动门槛，因此成功触发 0.6 m/s 辅助。同点起步记录显示轮子与场地接触，没有车身与障碍的起步卡死证据。

第一轮途中另外发生过绕行，因此这次验证仅用于检查起步，不能写成整条路线表现正常。后两次完成前一目标后的位置与原事件有厘米级差别；原目标朝向、完整地图历史和 GUI 负载也未完整还原。这些都是未复现结果的限制。

复测产物：

- `/tmp/rm27_startup_stall_1010_nav.log`
- `/tmp/rm27_startup_stall_1010_direct.csv`
- `/tmp/rm27_startup_stall_1010_after_goal.csv`
- `/tmp/rm27_startup_stall_1010_sequence_departure.csv`
- `/tmp/rm27_startup_stall_1010_trajectories.jsonl` 及后续两份轨迹记录。
- `/tmp/rm27_startup_stall_contacts.log`

## 后续确诊需要的证据

原运行的 planner 和 MPC 详细 CSV 均未开启，普通节点日志无法补回已丢失的参考速度、输出速度和物理接触。

下次同类事件应同步记录 `/minco/opt_path`、`/minco/cmd_vel_mpc`、`/cmd_vel_controller`、`/cmd_vel`、`/odometry` 和 `/ground_truth/odometry`。用当前已有 `MincoMpc.performance.detailed_csv_enable` 可进一步记录参考与控制输出，但本次未修改该配置。

若参考起步速度不足且最终指令被清零，应修正启动参考/死区配合；若指令充足而实际速度为零，再依据接触和驱动力记录处理物理问题。不能在缺少这两类证据时继续盲目提高最低速度或扩大障碍容差。
