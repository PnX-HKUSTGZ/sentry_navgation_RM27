# RMUC2027 官方 CAD 点云与 map 对齐检查

2026-11-05：已生成 `src/pnx-launch/launch_bringup/pcd/simulation/RMUC2027.pcd`。
它是官方 CAD 的表面先验，使用米制、右手 Z 向上坐标，应用了项目场景的
CAD→map 变换。文件大小和 PCL 读取检查通过；与现有 RMUC2026/RM27
地图的严格三维兼容性检查未通过，原因包括实际地形和局部模型差异。

| 项目 | 结果 |
| --- | --- |
| PCD 格式 | PCD v0.7，binary，float32 XYZ |
| 点数 | 385,659 |
| 文件字节数 | 4,628,142，约 4.63 MB / 4.41 MiB，小于 30,000,000 字节 |
| 体素降采样 | 0.04 m，使用体素内可见表面点的质心 |
| 采样间距 | 0.02 m，固定随机种子 2027 |
| XYZ 范围，m | `[-14.019989, -6.377222, -0.216872]` ～ `[13.980003, 9.625909, 3.490494]` |
| PCD SHA256 | `4f6b5a84d0282ab8f26824206bb971b24098cca8fea39b4dc62c8a3308c6da83` |

来源为 RoboMaster 高校赛务的
[《RMU 2027场地图纸&可移动道具开源》](https://bbs.robomaster.com/article/1970956?source=1)
中的 [RMUC2027_V1.0.0.stp](https://hz-rm-bbs-web-prod.oss-cn-hangzhou.aliyuncs.com/080ae7230f6c469c9bec9d2c892797b61791994813953/RMUC2027_V1.0.0.stp)。
原文件 653,443,279 字节，MD5 `3d3d6a411f44e9a4983794261b3f0bf7`，与帖子一致。
来源及派生点云遵循原文标注的 CC BY-NC-SA 4.0，保留作者、出处和许可信息。

## 生成方法

读取完整 STEP 装配体，保留各零件装配位姿，以 2 mm 线性挠度、0.3 rad
角度挠度三角化，获得 214,794 个面、6,058,760 个三角形。4 个面首次
三角化失败；细化后修复 2 个，剩余 2 个面的面积分别约为
`8.13e-20`、`2.71e-20 mm²`，记录为退化面。

从 CAD 表面采样，去除模型底面以下的点，并从场地支撑面上方 0.25 m、0.8 m
的共 1,530 个视点进行遮挡检查，再做 4 cm 体素降采样。保留真实斜坡、
平台、墙体和可见道具表面；不添加单独附件中的可移动能量单元。
这是静态 CAD 可见表面采样，不模拟某一型号雷达的扫描轨迹、反射强度或噪声。

**保留负高度地面。** 官方模型中存在约 1.5° 的微倾斜地面，不能以 `z>=0`
裁剪。仅排除 CAD 底面 `z≈-0.30 m` 附近的地下底板，筛选下界约为
`-0.297 m`；最终可见点最低约 `-0.217 m`。

## 坐标与检查结论

现有 `RMUC2026_world.world` 引用 `RM27_battlefield_visual`。
其模型内部位姿为 `(-0.02, -1.624344, 1.641344)`，世界实例位姿为
`(0, 1.624344, 0)`，两者旋转均为单位旋转。由此采用：

```text
map.x = CAD.X / 1000 - 0.02
map.y = CAD.Y / 1000
map.z = CAD.Z / 1000 + 1.641344
```

PCD 的坐标已包含该变换，`VIEWPOINT` 为单位位姿。PCD 本身没有标准 ROS
frame 字段，发布为 PointCloud2 时应设置 `frame_id=map`；manifest 中已注明。

检查对照了现有场景中独立导入的 1,535,746 个三角形、现有
`RMUC2026.pcd` 和 `RMUC2026.yaml/pgm`。主要 XY 墙体和障碍物轮廓基本吻合，
但不能将新 PCD 视为旧地图完全等价的三维先验。

| 同一 map XY，m | 官方 CAD 表面 Z，m | 现有场景表面 Z，m |
| --- | --- | --- |
| `(-8, 0)` | -0.042535 | 0.000000 |
| `(-8, 5)` | 0.111674 | 0.200000 |
| `(0, 6)` | 0.185420 | 0.300000 |
| `(8, -2)` | 0.105162 | 0.200000 |

上述非零差异随位置变化，不能通过一个统一 Z 偏移消除。源 CAD 在这些位置
的坡度约 1.5°，现有仿真相应表面为水平面；另外，基地、通道和外缘存在
局部形状差异。保留官方几何，没有为贴合旧场景将其压平或变形。

独立旧场景表面的六方向射线探测统计（100,000 个固定种子随机点）：

- 残差中位数 4.48 cm，P95 25.81 cm；这是六方向射线残差，不是精确最近表面距离。
- 53.919% 的探测点在 5 cm 内，86.158% 在 10 cm 内。
- 对高于 0.10 m 的两份点云进行诊断配准，建议增量约为
  `(0.04, 2.23, 3.58) cm`，roll/pitch/yaw 约
  `(0.100, -0.022, 0.043)°`。该增量仅作诊断，未写入交付点云；
  局部几何差异也会影响这个估计。

因此 `alignment.json` 中严格兼容性 `checks.passed=false`，检查器以状态码 1
退出；这不表示 PCD 损坏或文件大小不达标。它表示不能宣称新官方 CAD 与
旧简化地图完全三维对齐。本次没有运行在线 TF、完整导航或重定位成功率验证。

## 文件与使用

- [点云](../src/pnx-launch/launch_bringup/pcd/simulation/RMUC2027.pcd)
- [来源、坐标和 SHA256 manifest](../src/pnx-launch/launch_bringup/pcd/simulation/RMUC2027_manifest.json)
- [二维地图与点云叠加图](validation/rmuc2027/map_alignment.png)
- [三维点云预览](validation/rmuc2027/pointcloud.png)
- [机器可读检查报告](validation/rmuc2027/alignment.json)
- [PCL 实际读取日志](validation/rmuc2027/pcl_read.log)

选用该 PCD 时，重定位参数应为：

```yaml
prior_pcd_transform: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
transform_prior_map_with_lidar_offset: false
```

提供了 `tools/mapping/configs/rmuc2027_cad_prior.yaml` 参数覆盖文件。
在常规参数文件之后加载它，再通过 `prior_pcd_file` 选择新 PCD。
现有默认仿真配置中的 `+1.65 m` 先验偏移不可再次施加到本文件。
原默认配置和旧地图均保持原样，本次未将运行中的导航切换到新点云。

现有 `global_relocalization` 可直接加载此 PCD，并使用配套
`RMUC2027_manifest.json` 的 `map_sha256`、`frame_id`、`T_map_world` 做校验；
后者为单位矩阵，说明输出点云与项目 Gazebo world/map 使用同一坐标约定。
完整重定位效果需要使用与官方 CAD 对应的场景或实测数据评估。

仅查看点云可运行：

```bash
source /opt/ros/jazzy/setup.bash
ros2 run pcl_ros pcd_to_pointcloud --ros-args \
  -p file_name:="$PWD/src/pnx-launch/launch_bringup/pcd/simulation/RMUC2027.pcd" \
  -p tf_frame:=map
```

RViz 设置 Fixed Frame 为 `map`，添加 `/cloud_pcd` 的 PointCloud2 显示。

## 复现

CAD 源文件、缓存网格和隔离 Python 环境在 `~/.cache/rmuc2027/`，
不将 653 MB 原始 CAD 加入仓库。创建可用的 Python 3.12 虚拟环境后：

```bash
python3 -m venv ~/.cache/rmuc2027/venv
~/.cache/rmuc2027/venv/bin/python -m pip install \
  -r tools/mapping/rmuc2027_requirements.txt

curl -L --fail --retry 3 \
  'https://hz-rm-bbs-web-prod.oss-cn-hangzhou.aliyuncs.com/080ae7230f6c469c9bec9d2c892797b61791994813953/RMUC2027_V1.0.0.stp' \
  -o ~/.cache/rmuc2027/RMUC2027_V1.0.0.stp

OMP_NUM_THREADS=8 OPENBLAS_NUM_THREADS=4 \
  ~/.cache/rmuc2027/venv/bin/python tools/mapping/generate_rmuc2027_pcd.py

OMP_NUM_THREADS=8 OPENBLAS_NUM_THREADS=4 \
  ~/.cache/rmuc2027/venv/bin/python tools/mapping/check_rmuc2027_map_alignment.py
```

已有网格缓存时可用 `--stage pcd` 重做采样。生成器按完整 PCD 文件的字节数
检查 `<30,000,000`，必要时自动增大体素；manifest 记录实际体素大小。
检查器比较的旧模型存在上述差异，严格兼容性检查预期返回 1。
