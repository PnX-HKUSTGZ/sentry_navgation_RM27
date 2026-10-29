# Prior pcd file

For other PCD files not included in the repository, see:

<https://flowus.cn/lihanchen/share/87f81771-fc0c-4e09-a768-db01f4c136f4?code=4PP1RS>

`RMUC2026.pcd` is the repository's Gazebo world-coordinate prior copied from
`tools/global_relocalization/assets/RMUC2026/map.pcd`; its matching capture
manifest is `RMUC2026_manifest.json`. Ground truth was used offline to prepare
this prior. The Point-LIO/GICP localization chain does not consume ground-truth
poses online. The MINCO RMUC2026 world profile therefore applies no additional
PCD transform or lidar mounting offset.
