# Dense raw LiDAR prior export

`tools/global_relocalization/export_dense_sensor_prior.py` exports a new map from a completed, frozen full-SLAM capture's original `/livox/lidar` bag messages and its unchanged Point-LIO trajectory. It avoids the online mapping feature cloud's `point_filter_num` and `filter_size_surf` sparsification. It runs offline and publishes no ROS or simulator commands.

Example:

```bash
source tools/global_relocalization/mapping_environment.bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python3 \
  tools/global_relocalization/export_dense_sensor_prior.py \
  tools/global_relocalization/data/optimization_20261026/mapping06 \
  tools/global_relocalization/data/optimization_20261026/mapping06_raw_prior \
  tools/global_relocalization/data/optimization_20261026/mapping06_dense_prior_v2
```

The output directory must not exist and must be outside the source capture. Source scans, raw bag, trajectory and existing priors remain unchanged.

## Frame and timing contract

The current supported capture uses `camera_init <- body` LIO poses, fixed identity `IMU <- LiDAR` calibration and an instantaneous whole-scan sensor model. The exporter verifies `extrinsic_est_en=false`, identity calibration, and both converter and Point-LIO instantaneous-scan settings. It refuses other calibration/time models rather than silently assuming compatibility. Original LiDAR points are projected with `T_camera_body * T_body_lidar`, then the raw prior's existing single initial `T_world_camera_init` gauge is applied once. No GT trajectory, ideal map, estimated 2D graph or query pose participates in projection.

Raw message header timestamps are associated with exact LIO stamps, allowing a 1 ns serialization tolerance. Otherwise, translation and SO(3) interpolation require a containing bracket no longer than 0.15 seconds. There is no extrapolation. Every raw frame is inventoried, including unsupported initialization/trailing frames. An internal LIO support gap aborts export. The raw frame count must match bag metadata, every original LIO/cloud stamp must have an exact raw counterpart, and the predeclared reprojection checks must all execute. Nonrigid poses, invalid ranges, missing coverage or changed inputs prevent a completion marker.

All finite raw returns within 0.5–30 m are fused, matching the raw localization frontend's operating range. No registration, GT error, source feature index or outcome selects points. The input, nonfinite, below-range, above-range and fused point counts are recorded. The final voxel size is inherited explicitly from the raw prior (mapping06 uses 0.03 m).

Twelve deterministic frames spread across the original trajectory reproduce the source converter's finite/ring filtering, Point-LIO's `point_filter_num`, and its surface voxel centroids. Their registered-cloud agreement independently checks calibration and rigid-transform direction before accepting the export. This is a consistency check, not an estimate correction.

## Provenance and results

The exporter verifies the raw prior's capture/parameters/provenance hashes, completion marker, map hash and scan linkage. It hashes every registered scan and every raw bag file before projection, rechecks all inputs afterward, and archives the exact exporter source. Outputs include:

- `map.pcd`, `manifest.json`, `dense_prior_export_complete.json`;
- `source_inputs.json`, `rules.json`, `exporter_source.py`;
- `raw_frame_inventory.json`, `registered_reprojection_checks.json`;
- unchanged copies of mapping parameters and binary provenance.

The manifest states that the LIO trajectory is unchanged; it does not copy a source map's quality verdict. Map acceptance still requires independent localization evaluation.

For mapping06, all 4604 supported raw frames match exact LIO stamps. Ten initialization and 101 trailing frames lack pose support and are listed explicitly. The exporter fuses 53,727,367 finite in-range returns into 841,652 points. Across the twelve checks, the largest registered-to-reproduced P95 is `4.93e-7 m`. The resulting map SHA256 is `f2aa5dc6fc2973cd475be1ccdfb603ee6e4196a4a1b0d8af736cfe9dee9b4027`.

The v2 integrity guards produce exactly the same PCD bytes as v1. This dense map remains an experimental diagnostic: the frozen batch05 holdout055 query still accepts a wrong 180 degree hypothesis. More points alone did not resolve that ambiguity, so successful export must not be interpreted as localization acceptance.

Tests:

```bash
python3 -m pytest -q tools/global_relocalization/test_export_dense_sensor_prior.py
```

The separate `refine_sensor_prior.py` / `evaluate_refined_prior.py` tools are experimental 3D pose-graph research. Mapping06 graph v1 did not materially improve held-out geometry and did not converge; v2 correctly refused export after exhausting its solver budget. They are not the dense export path and their maps must not be treated as approved priors.
