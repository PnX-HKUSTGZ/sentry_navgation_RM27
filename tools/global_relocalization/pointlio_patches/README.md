These patches apply to the exact Point-LIO HEAD exported by `build_mapping.bash`.
The dirty submodule worktree and index are never restored or patched. The source
preparer checks existing isolated C++ sources before changing them and records
both the upstream revision and all patch/source hashes in
`.build/mapping_source_manifest.json`.

`preprocess.instantaneous_scan` defaults to false for real spinning sensors.
Simulation sets it true to retain every point at the message header timestamp,
including all-zero time fields, without the azimuth-based missing-time fallback.
The Gazebo converter has a matching `instantaneous_scan` parameter, default true.

The ESEKF uses `M_Noise` as measurement variance R in its small-observation
innovation branch (`H P H^T + R I`). Its large-observation information branch
must therefore use `P^-1 + H^T H / R` and `K = posterior H^T / R`. Upstream
multiplied by R in both expressions, giving inconsistent estimates whenever the
number of correspondences crossed the state dimension. Instantaneous scans put
many returns at one timestamp and expose that branch on every update. The patch
corrects the algebra; it does not change residual thresholds or use ground truth.

Build and test after sourcing `tools/global_relocalization/environment.bash`:

```bash
bash tools/global_relocalization/build_mapping.bash
cmake -S tools/global_relocalization/pointlio_tests -B tools/global_relocalization/.build/pointlio_tests
cmake --build tools/global_relocalization/.build/pointlio_tests -j2
ctest --test-dir tools/global_relocalization/.build/pointlio_tests --output-on-failure
```

The actual ESEKF implementation is compared with the innovation reference over
36 covariance, observation-count, state-dimension, and rank-deficient cases.
Preprocessing tests cover instantaneous clouds, missing-time inference, and real
per-point seconds. The converter integration test runs in isolated ROS domain 83.

The scratch-vector patch constructs `pbody_list` and `crossmat_list` elements
with `resize` before indexed writes. `reserve` alone left their size zero.

The input-history patch declares `input_buffer.imu_depth=2000` and
`input_buffer.lidar_depth=100` for both DDS history and internal queue limits.
An overflow fails explicitly rather than silently discarding required IMU.
This preserves observations during synchronous registration and publishing;
it does not waive latency or mapping accuracy requirements.
Accepted-header gaps are logged. Optional `diagnostics.trace_file` writes a CSV
of scan/clock timestamps, processing time, queue sizes, callback/gap counters,
pose, and covariance diagnostics. Its default empty path disables trace I/O.

The DDS integration test deliberately holds an executor for one second during
a 100 Hz IMU stream. Default depth five retains only the final five messages;
the revised depth retains all 100. This validates the retention mechanism,
while a full sensor replay is still required to establish trajectory quality.

The steady-loop patch replaces `rclcpp::Rate(500)` (default `RCL_SYSTEM_TIME`)
with `rclcpp::WallRate(500)` (`RCL_STEADY_TIME`). Sensor propagation timestamps
are unchanged. On the affected workstation a direct 45-second observation
recorded a 1.996149-second rollback of realtime relative to monotonic time.
The buffered mapper retained all sensor messages but had 1.87-second backlog
with registration taking at most 5.1 ms; a steady scheduling clock addresses
that stall without modifying observations or relaxing latency requirements.
