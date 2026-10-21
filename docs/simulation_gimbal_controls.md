# 仿真云台与立头控制

适用模型：`src/rm_27_stimulation/urdf/simulation_waking_robot.xacro`。
默认使用 `meshes/rm27_sentry_visual` 中的拆分 CAD 网格。实车不启动这些控制节点。

## 启动与操作

在仓库根目录执行：

```bash
source install/setup.bash
ros2 launch rm_27_stimulation sim_with_nav.launch.py \
  navigation_mode:=minco use_ground_truth_odom:=true \
  gui:=true use_rviz:=true gimbal_panel:=true
```

原有 `tools/stimulation/start_nav_stimulation.bash` 也会自动打开控制窗口。

| 按钮 | 行为 |
| --- | --- |
| 立头 | roll 平滑转动 90 度，保持大 yaw 朝前 |
| 开启旋转 | 立头实际到位后，大 yaw 持续旋转 |
| 停止并回正 | 大 yaw 就近回到相对底盘朝前的位置，保持立头 |
| 倒头 | 停止旋转，等待大 yaw 回正且停稳，再向前翻下 |

倒头状态禁止持续旋转。窗口显示实际关节状态，按钮请求不等于动作已完成。
关闭窗口不会停止导航或改变当前旋转模式；重新打开窗口会读取当前状态：

```bash
ros2 run rm_27_stimulation gimbal_panel.py
```

无桌面运行时传 `gimbal_panel:=false`。也可以直接调用服务：

```bash
ros2 service call /sim/gimbal/set_upright std_srvs/srv/SetBool '{data: true}'
ros2 service call /sim/gimbal/set_spin std_srvs/srv/SetBool '{data: true}'
ros2 service call /sim/gimbal/set_spin std_srvs/srv/SetBool '{data: false}'
ros2 service call /sim/gimbal/set_upright std_srvs/srv/SetBool '{data: false}'
```

## 速度参数

修改 `src/rm_27_stimulation/config/gimbal_control.yaml`，重启仿真生效。

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `spin_speed` | 0.30 rad/s | 大 yaw 持续转速，约 17.2 度/秒 |
| `home_speed` | 0.60 rad/s | 回正过程的目标速度上限 |
| `roll_speed` | 0.65 rad/s | 立头、倒头过程的目标速度上限 |
| `angular_acceleration` | 1.0 rad/s² | 关节目标速度变化上限 |
| `feedback_timeout` | 0.50 s | 关节反馈过期时暂停目标推进 |
| `feedback_future_tolerance` | 0.05 s | 容忍关节反馈略早于 `/clock` 到达 |

控制器以实际关节角和角速度判断到位。回正容差为 0.025 rad、停稳速度为
0.05 rad/s；不会在仍偏离前方或仍转动时直接翻下。关节反馈超时会暂停目标推进，
保持最近的反馈位置，恢复后继续用户选择的模式；停止旋转请求仍可接受。
仿真时钟倒退则取消旋转并恢复倒头请求。

## 模型与坐标链路

```text
base_link
└── gimbal_yaw_joint：绕 Z 轴连续旋转
    └── gimbal_yaw
        ├── left_mid360 / imu_link：固定安装，roll 为 -30 度
        └── gimbal_roll_joint：绕局部 -X 轴，0 到 pi/2
            └── gimbal_roll：小 yaw 与头部组合，本次不独立驱动小 yaw
```

大 yaw 零位相对底盘朝前，不是朝地图固定方向。roll 零位是倒头，90 度是立头。
MID360 与 IMU 只随大 yaw 运动。模拟碰撞体随关节运动，倒头最高碰撞点约 240 mm，
立头约 382 mm。质量沿用约 10.21 kg 总量并分配到活动部件；部件质量和惯量是仿真
近似值，STL 本身不提供材料密度或实测质量。

原始 `饼922.zip` 有 513 个 STL。拆分、对齐、简化的来源记录在
`meshes/rm27_sentry_visual/articulated_metadata.json`，可用以下命令重新生成网格：

```bash
python3 src/rm_27_stimulation/tools/import_articulated_sentry.py 饼922.zip
```

原包与旧 `哨922stl.zip` 的装配坐标不同。大 yaw 按原包校正，roll 轴心取其
DM4310 电机参考位置；将倒头时的枪头俯仰调整到朝前，再固定小 yaw/俯仰机构。
倒头几何缩放已烘焙进各部件网格，动作过程中不缩放网格。

Gazebo 发布真实 `/joint_states`，`robot_state_publisher` 生成活动 TF。真值定位
按点云采样时刻查动态雷达安装变换，并计入关节运动产生的雷达速度。
MINCO、MPC 和底盘速度转换使用 `/odometry` 的底盘位姿；ROG 射线仍使用实际雷达位姿。
ROG 的地面参考和自过滤通过同一采样时刻的关节变换得到，头部过滤框也随 roll 运动。

## 固定导航高度

按当前使用方式，**没有加入姿态切换时动态修改导航车高、净空阈值或通行策略的逻辑**。
现有 `vehicle_height` 等阈值保留；进洞前由操作者先倒头。

`projection.robot_origin_to_ground` 从雷达中心到地面的 0.210 m 换成底盘轴心到
地面的 0.080 m，这是参考坐标改变后的外参换算，不是把实际车高改成 80 mm。
`projection.robot_pose_frame: base_link` 使地面参考不跟着云台旋转。

## 验证

自动测试覆盖倒头旋转互锁、回正到位条件、多圈回正、关节反馈丢失，以及动态 TF
自过滤。集成检查脚本只控制仿真云台，不发布底盘速度：

```bash
python3 src/rm_27_stimulation/tools/check_gimbal_simulation.py
```

已在隔离的 Gazebo 测试场景中验证两次立头/旋转/回正/倒头序列、雷达安装位姿与
扫描时刻 TF，以及旋转期间的一次 MINCO 移动导航目标。该验证使用默认真值定位；
Point-LIO 模式的实际定位精度需另行测试。
