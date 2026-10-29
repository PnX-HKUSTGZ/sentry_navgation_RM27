# 南侧中央通道覆盖试验

保持出生点和当前安全保护，按现有批测接口执行三个不同 Gazebo 种子；每个任务为三段，先到西侧平台，再前往东侧并返回：

```bash
for seed in 42 43 44; do
  python3 tools/navigation_benchmark/batch.py --output "/tmp/rm27-south-$seed" \
    --domain 92 --repeats 1 --seed "$seed" \
    --params tools/navigation_benchmark/profiles/obstacle_speed.yaml \
    --goals '[[-2.0,-4.4],[2.0,-4.2],[-2.0,-4.4]]'
  python3 tools/navigation_benchmark/summarize.py --root "/tmp/rm27-south-$seed" \
    --scenario tools/navigation_benchmark/scenario.json
done
```

离线检查（输入为当前仿真 xacro 展开的 URDF）：

```bash
python3 tools/navigation_benchmark/coverage_south.py --robot-urdf tools/global_relocalization/.build/drive_physics_validation_20261028/robot.urdf
```

两端满足碰撞网格水平静止全向旋转包络检查，支持面分别为约 0.20 m 和 0 m。检查过程使用场景实际 SDF 碰撞模型，车身外包络半径约 0.3883 m、总高 0.24 m，另加 0.02 m 径向余量。来源哈希和结果见 `results/20261028/south_coverage_geometry.json`。中间为坡地，此检查不证明中段能通过或规划器会选择该通道。

判定覆盖时必须检查真值轨迹穿过 `scenario.json` 的南洞矩形且在各自段落中实际到达两端；仅经过矩形、目标成功、或零停车均不足以证明穿越。规划器可能绕路，记录绕路、不覆盖、失败和启动失败，不得写成通过；每次种子对应独立重启，也不代表 ROS 调度完全确定。若需额外对照，可对同三个种子使用 `safe_baseline.yaml` 和不同输出目录；成功段之外不补计零停车。

旧坡道事件未记录完整车身 yaw 和逐时刻 3D 坐标变换；点高度与 odom 高度不得直接相减。`geometry_swept.py` 接受同步完整位姿、逐样本 `world_from_pose_frame` 4×4 变换和已证实的采样间最大位移，计算定向车体包络与静态三角网格表面重叠，保守筛查采样间包络。地面接触、斜坡支撑和代理包络重叠不等于实际碰撞；未获得完整同步输入前，两例保护均维持待判定。
