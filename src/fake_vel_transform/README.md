# fake_vel_transform

本功能包启动时，Fake Velocity Transform 会创建一个 `fake_robot_base_frame` 的坐标系，其 x, y, z 与 `robot_base_frame` 的坐标系一致，但 yaw 固定指向正前方，然后将生成的该坐标系发布到 tf2。同时，Fake Velocity Transform 会订阅 `input_cmd_vel_topic` 话题，将接收到的速度指令转换到 `robot_base_frame` 坐标系下，并发布到 `output_cmd_vel_topic` 话题。

主要目的是用于适配 NAV2 局部路径规划器，当速度参考坐标系 `robot_base_frame` 变化剧烈时，如云台处于自旋扫描时，NAV2 局部路径规划器会将机器人的方向视为与当前路径规划方向一致，导致机器人无法正常运动。而使用 `fake_robot_base_frame` 可以规避这个问题，实现较稳定的轨迹跟踪效果。

由于 NAV2 humble 发行版出于避免破坏原有接口的原因，依然使用 Twist 类型（不含时间戳），humble 往后的版本才使用 TwistStamped，导致无法直接实现 cmd_vel 与 odometry 的时间戳对齐。因此，本功能包暂时订阅 local_plan 话题（由局部路径规划器发布），以获取时间戳，将它的时间戳视为 cmd_vel 的时间戳，以间接实现时间戳对齐。
Related issue: [Switch from Twist to TwistStamped for cmd_vel #1594](https://github.com/ros-navigation/navigation2/issues/1594)

在 `use_latest_odom_for_cmd: true` 的 MINCO 配置下，不订阅 `local_plan`，也不创建上述同步器。
TF 和速度转换使用最新有效里程计，重复或乱序的里程计不能覆盖尚有效的新姿态；里程计过期仍停止输出运动命令。

## 真实云台与虚拟坐标系

本节点只发布 `robot_base_frame -> fake_robot_base_frame` 的零平移、负 yaw 变换，
不发布 `base_link -> gimbal_yaw`，不订阅云台编码器，也不估计底盘位置。
旋转雷达的里程计不等于底盘里程计；真实云台关节 TF 必须由带时间戳的机械角度反馈提供。
输入里程计所描述的朝向必须与 `robot_base_frame` 一致，输入速度则必须与里程计父坐标系的平面轴一致。
仅将参数命名为 `base_link` 不会把雷达 yaw 转换成底盘 yaw。

## Published Topics

* `tf` (`tf2_msgs/msg/TFMessage`) - 虚拟速度参考坐标系的变换，不是机械关节反馈
* `output_cmd_vel_topic` (`geometry_msgs/msg/Twist`) - 转换后的速度指令

## Subscribed Topics

* `input_cmd_vel_topic` (`geometry_msgs/msg/Twist`) - 机器人的速度指令
* `local_plan_topic` (`nav_msgs/msg/Path`) - 局部路径规划器的路径
* `odom_topic` (`nav_msgs/msg/Odometry`) - 里程计数据
* `cmd_spin_topic` (`example_interfaces/msg/Float32`) - 控制底盘固定旋转速度，将会叠加到 `output_cmd_vel_topic` 中

## Parameters

* `odom_topic` (`string`, default: "odom") - 里程计话题，其被跟踪坐标系的朝向应与 `robot_base_frame` 一致；`header.frame_id` 是世界参考系，不是机器人坐标系
* `use_latest_odom_for_cmd` (`bool`, default: `false`) - 为 true 时直接使用最新有效里程计，不等待或消费 `local_plan` 同步结果
* `robot_base_frame` (`string`, default: "gimbal_link") - 速度参考坐标系
* `fake_robot_base_frame` (`string`, default: "gimbal_link_fake") - 伪速度参考坐标系
* `local_plan_topic` (`string`, default: "local_plan") - 局部路径规划器的路径话题
* `cmd_spin_topic` (`string`, default: "cmd_spin") - 控制底盘固定旋转速度的话题
* `input_cmd_vel_topic` (`string`, default: "") - 输入速度指令的话题
* `output_cmd_vel_topic` (`string`, default: "") - 输出速度指令的话题。将原本基于 `fake_robot_base_frame` 的速度变换到 `robot_base_frame` 后发布
* `init_spin_speed` (`double`, default: 0.0) - 若没有接收 `cmd_spin_topic`，则使用该值作为固定旋转速度
