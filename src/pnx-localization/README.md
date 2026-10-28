# PnX Localization

定位相关的 ROS 2 包集中存放在此目录：

- `point_lio`：激光惯性里程计，作为主仓库的普通目录维护。
- `loam_interface`：将 Point-LIO 输出转换为导航使用的里程计与点云。
- `relocalization_manager`：先验点云匹配与 `map → odom` 变换管理。

包名、可执行文件名和 ROS 接口保持原有约定。colcon 会递归发现这三个包。

在工作区根目录构建：

```bash
colcon build --symlink-install --packages-select point_lio loam_interface relocalization_manager
```
