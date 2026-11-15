# relocalization_manager

生产导航由 `rm27_backend` 链接本包的 `rm27_relocalization` 库：两个包、一个进程、后台线程计算。
本库负责离线参考地图加载、Scan Context 查询、目标子地图缓存、点到面跟踪和全局恢复验证。
状态机、因子图写入、恢复锚和 map→odom 发布由后端负责，本库没有这些写入权限。

公共 C++ 合同位于 `include/relocalization_manager/`。为兼容既有后端代码，公共几何类型暂保留 `rm27_backend` 命名空间；
本包没有对 rm27_backend 的编译依赖，不构成循环依赖。公共库通过 ament 导出为 `relocalization_manager::rm27_relocalization`。

原 `RelocalizationManagerNode` 与原局部 GICP 代码保留作历史对照；新的导航 launch 不启动它。
如手动启动旧节点，必须停用后端 TF，不能同时发布 map→odom。

离线输入准备见 `tools/mapping/prepare_relocalization_map.py`；生产库的独立几何检查工具：

```bash
ros2 run relocalization_manager recovery_probe /path/map.pcd.rm27 /path/query_local.pcd
```

probe 查询坐标默认重力对齐，结果用于几何验收，不发布 TF、不控制机器人。
实际倾斜 IMU 的在线处理由后端提供对应 odom 姿态用于描述子整平与初值转换。
完整架构与部署参数见并列 backend 包 README；开发与实验记录见仓库 `docs/backend_recovery_2026-11-12.md`。
