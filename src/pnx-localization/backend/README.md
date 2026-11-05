# rm27_backend

RM27 的独立 Point-LIO 位姿图后端。目录名 `backend`，包名 `rm27_backend`。
本阶段接入原子扫描观测、关键帧与局部点云管理、里程计过程噪声和 GTSAM 4.2 iSAM2。
地图位姿观测与回环测量已有 C++ 接口及测试，尚未接入任何实际匹配结果。

本阶段仅有首帧规范先验和里程计链。输出用于旁路验证，不发布 TF，不替代
`relocalization_manager` 的 `map→odom`。只有里程计链时不能消除前端漂移。

## 构建与依赖

ROS 2 Jazzy / Ubuntu 24.04，C++17，Eigen、PCL、TBB、GTSAM 4.2。
正常部署可安装 `ros-jazzy-gtsam`，再运行工作区 rosdep 安装其他依赖。
没有管理员权限时，可以在工作区根目录运行：

```bash
python3 tools/stimulation/bootstrap_rm27_gtsam.py
source /opt/ros/jazzy/setup.bash
source install/setup.bash
MAKEFLAGS=-j2 colcon build --symlink-install \
  --packages-select rm27_localization_interfaces rm27_backend point_lio launch_bringup \
  --allow-overriding point_lio launch_bringup rm27_backend \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
source install/setup.bash
ros2 launch rm27_backend backend.launch.py use_sim_time:=true
```

用户目录安装脚本仅支持 Noble amd64：下载发行版 GTSAM 4.2/METIS 二进制，解包到
`~/.local/opt/rm27-gtsam`，保留许可证和下载 SHA-256。只修改本地 CMake 导出中的
METIS 绝对路径及发行包未提供的 CppUnitLite 目标，未修改 GTSAM 库二进制。
CMake 优先使用标准安装，找不到时使用上述路径。其他前缀可显式指定 `GTSAM_DIR`。
安装程序和库的 RPATH 指向构建时选定的 GTSAM；迁移机器后应重新构建，不能只复制 executable。

导航启动文件会读取部署 YAML 的 `rm27_backend.enabled`；设为 false 时不启动该节点。
真值里程计模式也不启动后端。仿真/实车的 MINCO 和 Nav2 四份参数均包含中文注释的完整参数块。

## 模块组织

- `localization_interfaces/` 是并列的小型接口包 `rm27_localization_interfaces`。前端只依赖消息，不依赖 GTSAM 或后端算法。
- `include/rm27_backend/types.hpp`：位姿、帧身份、关键帧和三类测量。
- `src/pointlio_adapter.cpp`：原子消息验收、Point-LIO 协方差坐标转换、世界云转局部云。
- `src/keyframe_manager.cpp`：身份水位、连续性、关键帧触发、过程噪声和活跃元数据。
- `src/cloud_store.cpp`：只读点云缓存、异步存盘、配额和加载。
- `src/pose_graph.cpp`：iSAM2、边缘化、因子槽压缩、地图/回环测量接口。
- `apps/backend_node.cpp`：单一 ROS 回调执行器、参数、输出和诊断。
- `test/test_backend.cpp`：几何、异常、图、边缘化和存储回归。

核心图的写入由调用方串行执行。文件写入使用一个独立工作线程，队列有界。
前端数据不会被后端修改；普通扫描只解析元数据，只有创建关键帧时才转换/采样点云。
当前没有多帧配准源累积，只保留连续性检查需要的上一帧元数据；后续接地图匹配时再增加有界近期点云窗口。

## Point-LIO 原子观测

新增 `/point_lio/backend_frame`，类型 `rm27_localization_interfaces/msg/FrontendFrame`，
QoS 为 best effort、keep-last 3。包含实例 ID、epoch、sequence、同次扫描的 Odometry、
完整世界系去畸变点云，以及 IMU 饱和、IMU 拒绝次数、有效匹配数和激光更新数。

Point-LIO 的 `publish.backend_frame` 控制发布，默认 true；无订阅者时不复制和发送原子消息。
必须使用 `odometry.publish_odometry_without_downsample: false`，确保扫描结束时刻的状态。
首次 joint 初始化只发里程计，没有完整扫描时，后端从后续第一条完整原子帧开始。

`instance_id` 在进程启动时变化；已有的传感器时间回退重置路径递增 epoch 并清零序号。
后端拒绝重复序号、同代乱序、旧 epoch、退休实例。实例切换和连续性中断创建新的独立 segment，
不构造跨段里程计边。最多记录 64 个退休实例，达到上限须重启后端。
实例 ID 基于进程号和高精度启动墙钟，服务于单机进程区分，不是分布式身份认证。

位姿表示 `camera_init ← body(IMU)`；完整点云位于 camera_init。时间戳必须精确相同。
后端以同帧位姿逆变换得到参考时刻 IMU 局部点云，不再次去畸变。点云支持小端 FLOAT32
XYZ，强度可缺省；对点数、消息字节数、字段偏移、行跨度和非有限值进行检查。

Point-LIO 的位姿协方差为 `[世界系加法位置, 机体系右扰动旋转]`。适配器用
`diag(Rᵀ,I)` 转成局部右扰动，并标记无效协方差。里程计边不直接使用该边缘协方差。

## 关键帧、点云与记录

首个有效原子帧建帧。以后先满足最小时间间隔，再由平移、三维转角、最大时间间隔任一触发。
默认 0.5 m / 10° / 最小 0.2 s / 最大 0.5 s。阈值使用原始 odom 位姿。
如果超过输入时间间隔 1 s 或相邻速度超出配置门限，则新开轨迹段，不把跳变当成运动约束。
合法高速运动也可能触发分段，因此实车应根据运动范围校准门限。

关键帧 ID 在后端进程内单调递增，不因分段复用。活跃关键帧保存原始位姿、图估计、身份、
时间和质量摘要；原始位姿保持不变。活跃元数据随图窗口裁剪。外部调用图观测接口后，
调用 `KeyframeManager::synchronizeEstimates()` 刷新关键帧估计。

关键帧点云先等间隔限采到最多 12,000 点，再用 0.15 m 体素滤波；至少需要 30 点。
局部点云只读、共享所有权；默认缓存预算 32 MiB，按插入顺序逐出，不随全部运行历史增长。
缓存与图窗口分别管理，因此有图节点不保证点云仍在缓存。

`storage.directory` 空值关闭存盘。非空时创建唯一 run 目录，保存：

```text
manifest.txt                 格式版本、坐标/协方差约定
keyframes.csv                ID、段、实例、代次、时间、原始位姿和创建时估计
odometry.csv                 相对测量及各轴过程方差
clouds/<id>.pcd              IMU 局部二进制点云
```

写盘队列默认最多 4 个等待项和 1 个正在写入项；满时丢弃该次归档请求并累计告警。
默认每个 run 的保守磁盘配额 1024 MiB，超限停止该次写入并报告错误，不删除历史文件。
缓存仍可继续服务图运行。没有启用存盘时，被缓存逐出的历史几何不能用于未来回环。
PCD 临时文件完成后 rename，再追加索引；索引与点云并非跨文件崩溃原子事务，异常终止可能
留下未索引点云或不完整尾记录。它是诊断归档，不支持恢复完整 iSAM2 会话，也不是 Z 的地图包格式。
记录中的估计是创建时估计，未来回环后的历史地图导出需另外增加一致快照保存功能。

## 里程计因子与输出协方差

同段相邻关键帧 `i,j` 的测量为 `Z_ij=O_i⁻¹ O_j`，即 `S_i ← S_j`。
使用 GTSAM BetweenFactor<Pose3>；测量始终来自原始 odom，不从优化结果反算。
首帧 PriorFactor 固定本段的坐标规范，以原始首帧位姿为中心，不意味着赛场绝对位置已知。

内部协方差为局部 RIGHT 扰动 `[rho,phi]`，进入 GTSAM 时转换为 `[phi,rho]`。
过程模型为：

```text
translation_variance = max(1e-12, q_time * dt + q_distance * net_distance)
rotation_variance    = max(1e-12, q_time * dt + q_angle * net_rotation)
```

每组方差复制到三个轴。参数是方差系数，不是标准差；当前值为工程起点，未实车标定。
区间内任一帧降级或序号缺失时，方差乘 10；降级来自饱和、IMU 更新拒绝、匹配不足或无激光更新。
它是保守启发式，不构成严格的相关噪声估计。净位移/转角也不能精确代表往返运动的累计误差。

关键帧时查询图边缘协方差。两关键帧之间，使用里程计增量逆变换的 Adjoint 传播最近锚的
局部右扰动，并添加上述过程噪声；这是模型预测近似。输出位置/旋转误差通过旋转变换到
世界固定轴表达，再填入 Odometry。速度沿用前端，不产生独立后端速度估计。

## iSAM2 资源管理与预留接口

默认 30 s 固定滞后、最多 120 个活跃位姿，两个条件中先达到者生效。过期节点先重排到
可边缘化位置，再调用 `marginalizeLeaves`，保留存活状态的联合线性信息。
为避免 GTSAM 4.2 的空因子槽长期积累，在槽数超过 `2*活动状态数+16` 时，用所有存活因子
（包括线性边缘因子）和当前状态重建求解器。测试比较完整图与裁剪/压缩图的位姿和协方差。
本实现每次过期重排整个有界窗口，因此窗口扩大后开销会增加。

预留的接口已实现并测试，但没有接入 ROS 测量生产者：

- `PoseGraph::addAbsolute(AbsolutePoseMeasurement)`：地图绝对位姿＋rank×6 平方根信息。
- `PoseGraph::addLoop(LoopMeasurement)`：两个活跃节点间的相对位姿＋rank×6 平方根信息。

两个接口都支持 1～6 维独立信息行，保持方向耦合；拒绝非法矩阵、错误 segment、过期节点和
重复测量 ID。每个活动窗口最多保留 1024 个外部测量身份，ID 长度最多 256。
回环因子使用 Cauchy 鲁棒核。接口接受的是已经完成几何/一致性验收的测量，当前没有实现
NIS、描述子召回、回环候选验证或先验匹配；鲁棒核不能替代这些验收。

固定窗口不能接受连接已边缘化历史节点的回环。未来建图模式须选择完整图或另设全局图，
而不能把磁盘上有历史点云误认为图节点仍存在。

## 对外输出

- `~/odometry`：每个有效原子帧的图预测位姿、模型协方差和前端速度。
- `~/keyframes`：有界活动关键帧 Path，保留各自时间戳。
- `~/diagnostics`：输入计数、拒绝/缺帧/分段、图节点与槽数、处理时间、缓存和归档状态。

坐标系名称为 `output.frame` 加 `_s<segment>`，例如 `rm27_graph_s0`，避免重置前后的独立轨迹
被当作同一坐标系。它与本段 Point-LIO 世界系采用相同初始规范，但不广播 TF；默认导航链不消费它。

## 测试与边界

```bash
colcon test --packages-select rm27_backend point_lio
colcon test-result --test-result-base build/rm27_backend --verbose
ROS_DOMAIN_ID=178 python3 tools/stimulation/benchmark_rm27_backend.py pointlio \
  --output /tmp/rm27_ab --repetitions 3
ROS_DOMAIN_ID=178 python3 tools/stimulation/benchmark_rm27_backend.py synthetic \
  --output /tmp/rm27_load --duration 60 --points 30000
ROS_DOMAIN_ID=178 python3 tools/stimulation/benchmark_rm27_backend.py fullstack \
  --output /tmp/rm27_full --duration 45 --repetitions 3 --navigate
```

完整性能和精度结果见 [VALIDATION.md](VALIDATION.md)。CPU 使用率按 100%=一个逻辑核，
RSS 包含共享页，比较整个工程时优先使用 PSS。合成点云精度、实际 Point-LIO 合成回放和 Gazebo
系统检查各自说明口径，不能互相当作实车证据。

已知前端风险：现有 Point-LIO 固定非零外参路径曾发现点坐标使用不一致，非零外参运动回放
的运行误差较大；本次未更改该估计算法。后端忠实复现它，不能修复前端系统性误差。
未验证实车长时间运行、真实时间同步故障和极端振动。只有里程计链，后端不提高定位精度；
主要收益是提供可靠的因子图、数据生命周期和未来全局约束接口。
