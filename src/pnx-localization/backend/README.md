# rm27_backend

RM27 的独立 Point-LIO 位姿图后端。目录名 `backend`，包名 `rm27_backend`。
本阶段接入原子扫描观测、关键帧与局部点云管理、里程计过程噪声、GTSAM 4.2 iSAM2，
以及两种互斥的全局约束：先验地图配准因子（`prior_map.*`）和 Scan Context 回环因子（`loop.*`、
`scan_context.*`），由 `constraint.mode: prior_map | loop | odometry` 选择，照搬 z-localization 的互斥规则。

启用先验地图后，本段图直接建立在 `map` 系，关键帧上的 scan-to-map 配准作为部分秩绝对位姿因子
入图，可消除前端漂移。输出仍为旁路，不发布 TF，不替代 `relocalization_manager` 的 `map→odom`；
`~/map_alignment` 给出等价的 map←camera_init 估计，供后续接管 TF 前对比。

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

## 先验地图配准因子

设计参照 z-localization 的 tracking/registration 流程，按本工程的 Point-LIO 原子帧和有界图改写。

- **地图**：后台线程加载导航 launch 传入的 `prior_pcd_file`（参数 `prior_map.file`），先施加
  `prior_map.transform`（须与定位管理器的 `prior_pcd_transform` 一致），再按 `prior_map.voxel_size`
  体素化，建立 KD 树、法向和协方差，并按特征值比标记平面点。加载失败时诊断报错并退回纯里程计图。
- **锚定（坐标基准）**：地图就绪后不再用原始 odom 建首帧先验。用 `seed_source=tf` 查询同刻
  `map←camera_init`（定位链已有 TF；精确时刻不可用时退回最新）或参数初值，再对当前帧做配准。
  只有满秩、质量合格且相对种子不超过 `acquisition_max_*` 的结果，才外推到下一帧作为首帧
  map 先验。断段后 2 s 内优先以上一段最后的 map 位姿为种子。
- **跟踪**：非首关键帧按 `correction_period` 排队，初值为图估计，源点云为最多
  `source_keyframes` 个近邻关键帧（按原始里程计相对位姿合并）。后台单线程、单任务槽，忙时跳过，
  提交时检查代次、段号和节点是否仍在图中。
- **配准与信息**：small_gicp GICP（头文件方式固定提交 `fa0cfc98`，不安装其库）求位姿；随后按
  平面对应重建点到面 Hessian，雅可比对应 IMU 局部右扰动。门限为平面对应数/比例、RMSE、
  36 扇区水平覆盖。残差方差取 `max(floor², rmse²·n/(n−rank))`，信息再除以源帧数；平衡 Hessian
  特征分解后丢弃弱于 `λmax/max_condition` 的方向，因此退化走廊只约束可观方向；最后叠加
  `translation/rotation_std_floor` 共模协方差，避免大量相关点导致过度自信。
- **入图检验**：校正量超过 `max_*_correction` 或 NIS（当前边缘协方差，卡方上尾概率低于
  `rejection_probability`）不通过时，观测不进图并计一次冲突；连续 `max_conflicts` 次后进入
  `suspect`，此后两次连续满秩配准推得的 map←camera_init 一致时才跳过 NIS 接入，回到 `tracking`。
- **已知限制**：NIS 依赖里程计噪声模型；若实车前端的旋转漂移持续超过 `odom_noise.rotation_*`
  所允许的量，地图校正会被拒并周期性经 suspect 恢复，需先标定噪声。固定窗口仍不能跨已边缘化节点
  建回环；地图因子不使用鲁棒核，依赖上述验收。

## Scan Context 回环因子（`constraint.mode: loop`）

照搬 z-localization 的 `scan_context/descriptor.cpp`、`loop.cpp`、`loop_consistency.cpp`，配准复用
上节的 GICP + 点到面信息构造（`PriorMap::fromPoints` 建目标子图）。

- **图结构**：回环需要历史节点仍在图中，回环模式关闭时间滞后，改为全图，以 `loop.max_keyframes`
  （默认 3000）为节点与回环库上限；图坐标为本段 `camera_init`（与纯里程计相同，`rm27_graph_s<段号>`）。
  回环库只覆盖当前轨迹段，前端重置或连续性中断开启新段时清空。
- **描述子**：每个关键帧的局部点云先用原始里程计 roll/pitch 整平（Point-LIO 的 `camera_init` 重力对齐），
  0.6 m 体素后编码为 30 环 × 90 扇区的最大高度图（z 加 `lidar_height`），另存 2 个横向 ±2 m 平移增广。
  检索时最新帧只用原始描述子，与候选的各增广比较：扇区键定粗列移，再逐列移取有效列余弦相似度，
  距离 = 1 − 相似度，列移取反为 source→target 偏航。
- **候选**：图估计位置在 `candidate_max_distance`（7 m）内、排除最近 15 帧、距离 < 0.8 的前
  `scan_context.candidates` 个（部署 6，z-localization 15）；精搜索列移限于粗对齐 ±`search_ratio`/2 周
  （部署 0.1）。检索限频：距上次检索原始里程计行程 ≥ `loop.query_distance`（1 m），行程不足时间隔
  ≥ `loop.query_interval`（5 s，静止时仍低频检索）；接受回环后冷却 5 帧；后台单任务，忙时跳过。
  回环模式下关键帧定时触发间隔改为 `loop.keyframe_max_interval`（2 s），静止时少建帧。
- **验证（后台，`loop.threads` 个 TBB 线程）**：源为最新帧及之前最多 3 帧（原始里程计拼接）。目标子图默认
  （`loop.target_same_pass: true`）取候选前后相邻最多 17 帧、12 m 内，按原始里程计拼接到候选系，避免多趟
  之间尚未校正的偏差把子图拼花；设为 false 时照搬 z-localization 取 12 m 内空间近邻并按图估计拼接。
  初值为图预测平移 + 描述子偏航（经两端整平旋转换回原始 IMU 系）。
  依次检验：配准质量（≥80 平面对应、比例 ≥0.5、RMSE ≤0.25 m、≥6 扇区覆盖）、0.5 m 重叠率 ≥0.45、
  测量平移 ≤3 m、原始里程计下 z 闭合差 ≤0.5 m 与 roll/pitch ≤`max_roll_pitch_deg`（部署 1°；
  Point-LIO 横滚/俯仰由重力可观，超出多为配准倾斜失败，z-localization 为 10°）、几何与描述子偏航差 ≤60°、
  PCM（历史回环经原始里程计闭合误差 ≤1 m/15°，至少一条一致或无可比历史），
  最后对同源候选求最大一致团，只提交其中 RMSE 最小者。
- **入图**：两端联合边缘协方差的 NIS（p < 0.001 拒绝）通过后，以 `candidate←latest` 方向的 rank×6
  平方根信息、Cauchy 核（`graph.loop_cauchy_size`=0.5，乘 √秩）写入。共模下限
  `loop.translation_std_floor`/`rotation_std_floor_deg` 部署为 1 cm/0.1°（仿真真值实测回环误差约
  0.5～0.8 cm、0.06°；z-localization 为 5 mm/0.02°，过紧会把厘米级配准偏差当高精度约束）。
  每条入图回环以 `loop accepted ... from_ns= to_ns= rel=` 单行写日志，
  `benchmark_rm27_prior_map.py loops` 据此按真值统计回环测量误差。
- **已知限制**：联合 NIS 依赖里程计噪声模型；漂移若系统性超过噪声模型（长回环时整圈闭合差远大于
  √N 累积的不确定度），回环会按设计被拒，须先标定 `odom_noise.*`。回环只是相对约束，误差上界取决于
  回环频度，不能像先验地图那样提供绝对基准；Point-LIO 本身几乎不漂时，回环最多与其持平。
  全图节点随时间增长，外部测量身份上限 1024 条。

## iSAM2 资源管理与预留接口

默认 30 s 固定滞后、最多 120 个活跃位姿，两个条件中先达到者生效。过期节点先重排到
可边缘化位置，再调用 `marginalizeLeaves`，保留存活状态的联合线性信息。
为避免 GTSAM 4.2 的空因子槽长期积累，在槽数超过 `2*活动状态数+16` 时，用所有存活因子
（包括线性边缘因子）和当前状态重建求解器。测试比较完整图与裁剪/压缩图的位姿和协方差。
本实现每次过期重排整个有界窗口，因此窗口扩大后开销会增加。

外部测量接口：

- `PoseGraph::addAbsolute(AbsolutePoseMeasurement)`：地图绝对位姿＋rank×6 平方根信息，
  已由上节的先验地图定位器使用；`assessAbsolute` 提供入图前的 NIS 检验。
- `PoseGraph::addLoop(LoopMeasurement)`：两个活跃节点间的相对位姿＋rank×6 平方根信息，
  由上节回环模块使用；`assessLoop` 提供基于联合边缘协方差的入图前 NIS 检验。

两个接口都支持 1～6 维独立信息行，保持方向耦合；拒绝非法矩阵、错误 segment、过期节点和
重复测量 ID。每个活动窗口最多保留 1024 个外部测量身份，ID 长度最多 256。
回环因子使用 Cauchy 鲁棒核；鲁棒核不能替代上节的几何与一致性验收。

固定窗口不能接受连接已边缘化历史节点的回环，因此回环模式使用全图；先验地图与纯里程计模式仍用
30 s / 120 帧固定窗口。不能把磁盘上有历史点云误认为图节点仍存在。

## 对外输出

- `~/odometry`：每个有效原子帧的图预测位姿、模型协方差和前端速度。
- `~/keyframes`：有界活动关键帧 Path，保留各自时间戳。
- `~/diagnostics`：输入计数、拒绝/缺帧/分段、图节点与槽数、处理时间、缓存和归档状态；
  启用地图时另有 `map_state`、各类拒绝计数、最近 RMSE/秩/NIS/校正量和配准耗时。
- `~/map_alignment`（仅锚定后）：`TransformStamped`，map←`input.world_frame`。

锚定后 `~/odometry`、`~/keyframes` 使用 `prior_map.map_frame`（默认 `map`）。未启用地图或
地图加载失败时，坐标系名称为 `output.frame` 加 `_s<segment>`，例如 `rm27_graph_s0`，避免重置前后
的独立轨迹被当作同一坐标系，它与本段 Point-LIO 世界系采用相同初始规范。均不广播 TF；
默认导航链不消费它。

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
