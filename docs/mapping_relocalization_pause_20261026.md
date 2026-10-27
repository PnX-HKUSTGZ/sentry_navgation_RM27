# 暂停留存与续跑说明

用户要求暂停，目标状态已设为 paused。所有 Agent 已停止，原生回放收到 SIGINT 后，Python 包装器已完成关闭报告与 provenance。未继续启动后续阶段。

## 当前结果

- 最终 100% 验收仍未达标；batch06 尚未生成。
- mapping07 原始采集完整：562 路径点、34 站、10056 激光帧、100564 IMU 消息，原始 bag 已关闭。
- stock 协方差 + mapping-only 下一真实扫描恢复的单站试验通过；原失败帧仍 false，不参与融合/训练。
- 冻结管线 52 项相关契约测试通过；版本 `460740311322db95c1497d6fad0700fca1bd6e4dcdf97aedce4e5b2deb88ade4`。
- 最新全站运行因用户暂停中断：已保存 25/34 站、3922 行，其中 3921 registered accepted。原失败帧和未完成阶段保留，不计作成功。
- 用户明确要求：重定位准备只允许原地旋转，禁止平移消歧；建图路线采集不受此约束影响。

## 已留存

`checkpoint.json` 记录精确路径与暂停状态。`working_tree.tar.gz` 保存当前有改动/未跟踪的源码与文档；`source_file_manifest.json` 记录文件 SHA256。Git 差异及子模块状态另存。`critical_artifacts.json` 记录恢复关键输入、二进制和管线冻结清单的 SHA256。`data_inventory.json` 记录已有实验文件路径和大小；大型 bag/PCD 保留在原目录，不再次复制，也未删除。

## 恢复顺序

收到用户明确恢复指令后再继续。先核验关键输入和版本；不要覆盖 `mapping07_station_recovery_v1` 的中断输出。使用同一冻结输入和 v2 native，在新的输出目录重放全部 34 站，比较原 5311 个成功位姿保持一致。然后按显式 support 模式依次运行 support → seed → domains → 全 561 对 pairs → graph；保持原规则、所有失败与验证分母。

仅当地图检查通过，才导出候选地图并执行历史定位回归。之后才生成下一批全新位置，最终必须全数满足自主地图 0.15 m / 3° / 开始事件到输出 1 s；准备阶段时间另计。不以历史点回归、局部统计或插值支持冒充正式 100% 验收。

这是同一工作区内的留存与快照，不宣称已经完成异地备份。

本次留存目录：`tools/global_relocalization/data/optimization_20261026/pause_checkpoint_20261026T033334Z`。
