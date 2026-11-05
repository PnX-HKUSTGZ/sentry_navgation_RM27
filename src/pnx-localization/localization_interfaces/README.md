# rm27_localization_interfaces

Point-LIO 和 `rm27_backend` 共用的 ROS 2 消息包；没有 GTSAM 或估计算法依赖。

`FrontendFrame` 将同次扫描的位姿、世界系完整去畸变点云、质量和身份封装在一条消息中。
时间戳及坐标系契约在 [消息定义](msg/FrontendFrame.msg) 中说明。
其中位姿协方差沿用 Point-LIO 的混合误差坐标，不能脱离本协议直接解释为标准 ROS 固定轴协方差。

实例 ID 随进程启动变化，epoch 随前端重置变化，sequence 在该代次内递增。
接收方必须检查这些字段，禁止将重置前后坐标系中的位姿直接组成里程计增量。
