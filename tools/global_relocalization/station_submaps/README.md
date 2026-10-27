# Ordered station experiment

`pipeline.py` is an independent, arbitrary-N sensor mapping diagnostic. It does
not install or change the production relocalization core, profile, mapping
controller, acceptance sampling, or existing experiment outputs. Every output
directory must be new. Graph rejection returns nonzero and never exports a map.

The source capture and raw-prior export must be complete and the bag closed
before `prepare` is run. `prepare` checks the final raw-prior manifest and bag
metadata before opening bag payloads. Bag metadata message counts must exactly
match the extracted raw IMU/LiDAR inventories; header times must be unique,
increasing, and continuous under the existing sensor gap limits.
The raw-prior manifest must identify the exact capture directory, match its
original source hashes/completion counters, and retain the supported identity
`T_map_world`. A different capture's gauge cannot be silently reused.
Observed visit IDs/indices/kinds must match both saved copies of the frozen
ordered protocol; a completed route cannot silently omit a planned visit.
Protocol XYZ/heights are not used by this check or estimator.

## Stages

Run from the repository root after sourcing
`tools/global_relocalization/mapping_environment.bash`. Keep
`OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2`; stages are sequential, without ROS
playback or simulation. These examples use new output directories, not existing
frozen experiments:

```bash
export OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2
python3 tools/global_relocalization/station_submaps/pipeline.py prepare \
  --capture tools/global_relocalization/data/optimization_20261026/mapping07 \
  --raw-prior tools/global_relocalization/data/optimization_20261026/mapping07_raw_prior \
  --output /path/to/new/inputs

python3 tools/global_relocalization/station_submaps/pipeline.py local \
  --input /path/to/new/inputs/input.json \
  --native tools/global_relocalization/.build/station_submaps/offline_station_submaps \
  --output /path/to/new/local

python3 tools/global_relocalization/station_submaps/pipeline.py seed \
  --input /path/to/new/inputs/input.json --local /path/to/new/local \
  --output /path/to/new/seed

python3 tools/global_relocalization/station_submaps/pipeline.py domains \
  --seed /path/to/new/seed/sensor_seed.json --output /path/to/new/domains

python3 tools/global_relocalization/station_submaps/pipeline.py pairs \
  --domains /path/to/new/domains --output /path/to/new/pairs

python3 tools/global_relocalization/station_submaps/pipeline.py graph \
  --pairs /path/to/new/pairs --output /path/to/new/graph
```

The existing native adapter must already be built; this pipeline never builds
or installs anything. Its wrapper checks all ordered station IDs, every raw
timestamp/accepted pose, and the trusted calibration bias/source/stamp. Native
exit0 alone is insufficient: an adapter that records a fault still causes the
wrapper to return1 after preserving all results.

### Explicit mapping recovery and pose support

The default `local`, `seed` and `domains` contracts remain strict. An isolated
mapping experiment can explicitly use the stock-covariance adapter's one-token
next-real-scan recovery, followed by a separate pose-support stage:

```bash
python3 tools/global_relocalization/station_submaps/pipeline.py local \
  --input /path/to/frozen/inputs/input.json --native /path/to/offline_station_mapping_recovery \
  --support --output /path/to/new/local
python3 tools/global_relocalization/station_submaps/pipeline.py support \
  --input /path/to/frozen/inputs/input.json --local /path/to/new/local --output /path/to/new/support
python3 tools/global_relocalization/station_submaps/pipeline.py seed \
  --input /path/to/frozen/inputs/input.json --local /path/to/new/local \
  --support /path/to/new/support --output /path/to/new/seed
python3 tools/global_relocalization/station_submaps/pipeline.py domains \
  --seed /path/to/new/seed/sensor_seed.json --support --output /path/to/new/domains
```

`local --support` writes a runtime-only copy of the unchanged prepared input,
adding only `diagnostic_mapping_next_scan_retry=true`. The original input is
never rewritten. The complete copy and its SHA are bound by support validation.
Local registration completeness remains false when a registration failed;
source-ledger completeness is recorded separately. The native cloud may retain
the historical `incomplete_prefix_cloud.pcd` name even after later scans recover:
its explicit fused ordinals/count, rather than the filename, prove its content.

Support requires an isolated pure-convergence failure between two real accepted
poses, complete raw IMU, unchanged failed state, and a single mapping-only retry
token consumed by the next real accepted scan. The new offline bridge limit is
0.25 s; the original native scan gap remains 0.35 s. The original gyro innovation
0.08 rad, base step 0.12 m and base speed 0.35 m/s gates remain in force. Bias
comes only from the actual trusted initial calibration, independently verified
against prepared sensor calibration/source/stamp. No failed candidate pose,
failed cloud, GT, or cross-station target supplies a bridge. Actual gyro rotation
receives fractional accepted-endpoint SO3 closure correction; base position is
interpolated before reapplying the sensor lever arm.

Normalized rows are new artifacts. A rejected row keeps `accepted=false` and
`registration_accepted=false`; an `imu_bridge` sets only `pose_supported=true`.
It is never fused or trained, regardless of its original turn role. If its role
is heldout, every original raw point/scan remains available to the unchanged
seed-frozen domain and fixed heldout denominator. Any unsupported failure rejects
the whole support stage. Counts distinguish raw, registration accepted, bridged,
pose supported, fused, training and heldout frames. Seed and domains require
explicit support consumption; none of the pair or graph acceptance gates change.

## Sensor and reference semantics

- Required station metadata comes from `capture.json.station_scans`:
  `station_id`, `waypoint_index`, `kind`, `label`, commanded rotation direction,
  `capture_start_ros_ns`, `settle_zero_command_ros_ns`,
  `settle_start_sensor_stamp_ns`, `settle_end_sensor_stamp_ns`,
  `rotation_command_start_ros_ns`, `stop_command_ros_ns`,
  `capture_end_ros_ns`, and `requested_settle_sec`. Command/capture events are
  integer simulated ROS clock times. Settle sensor times are LIO headers, used
  to verify the declared settle duration, not as rotation boundaries.
- Initial and closure visits have distinct IDs and time windows even if their
  physical XY coordinates coincide. No XY deduplication or requested XYZ enters
  estimation. Control `measured_yaw_rad`, `station_xy`, and start/end XYZ are
  explicitly excluded from prepared station metadata.
- The fixed earliest raw scan having at least2s actual IMU support supplies the
  initial calibration. Its native sample window is `[scan-2.0-.035,scan]`.
  Existing gyro/acceleration/gravity stationarity checks apply. Failed initial
  calibration stops preparation; another apparently better window is not sought.
  Every visit must also pass those same sensor stationarity checks over the
  fixed native window `[settle_end-2.0-.035,settle_end]`. The earlier arrival/
  braking prefix is not used. A failing visit causes whole-experiment rejection
  after saving every raw frame/ledger; it is never dropped. These selfchecks
  never supply a replacement bias or orientation; the native trusted initial
  snapshot alone is carried into every visit. Its source SHA identifies the
  actual complete raw IMU NPZ, not the control/capture JSON.
- Gyro integration uses real timestamp intervals, the calibrated three-axis
  bias, and midpoint right-SO3 exponentials. Acceleration during movement is not
  used for orientation. The existing native carried-gauge implementation is
  currently verified for identity sensor/IMU extrinsics; unsupported differing
  extrinsics are rejected. The nonzero base/sensor extrinsic is respected when
  projecting base heading.
  Raw topic frame IDs must be stable and nonempty, matching configured LiDAR
  frame and the verified simulator `imu_link` convention.
- Rotation phase is anchored at the command timestamp and obtained by unwrapping
  base heading from the integrated SO3 trajectory. Angular-speed magnitude or
  sensor Z rate alone is not a turn measure. Command direction sets the sign;
  command magnitude and controller progress do not set the measured angle.
- Actual first2π and4π crossings are linearly interpolated between IMU samples
  to integer-nanosecond boundaries. `[command_start,first2π)` is train;
  `[first2π,first4π)` is heldout. Exact boundary scans belong to the later role.
  Pre-static and post4π/overshoot frames are auxiliary. All original raw scans
  remain present. A missing crossing is explicitly incomplete, never
  filled from the requested2.05 turns. Discrete LiDAR scans must bracket each
  continuous gyro turn; endpoint scan phases need not land exactly at2π.
- The capture begins before arrival braking/settle. Raw frames at or before
  `settle_end_sensor_stamp_ns` are retained as `aux_settle_prefix`, explicitly
  excluded from stationary local mapping, fitting and future station fusion.
  The native geometry input starts with the first raw header strictly after
  that predeclared sensor-time boundary; this rule is fixed before looking at
  estimation errors. All subsequent scans, including both turns and overshoot
  auxiliaries, enter the local frontend. Crossstation training uses only the first turn; raw-scan heldout
  evaluation uses only the second. This is not an independent validation of the
  local frontend that produced those poses. Local soft base-translation prior
  remains enabled only for this controlled stationary experiment, with all
  existing LiDAR/motion gates intact.
- Seeds use original LIO solely as initialization:
  `T_map_local = T_world_camera_init * T_LIO_sensor_first * inv(T_local_sensor_first)`.
  The original common gauge is inherited once. GT/post-evaluation/query files
  are not estimator inputs, and GT scoring must run separately after estimation.

## Frozen geometric observation and graph guards

Every unordered station pair is attempted in both directions. Target geometry
uses only first-turn training data and bounded, valid normals. A fixed
GoodNormalTree returns original target point indices. Both training and raw
heldout potential domains are determined once at the LIO seed with a.30m
window to this tree, then saved with point IDs and hashes before any fit. These
domains are potential overlap, not demonstrated common visibility.

This new observation version consistently uses the good-normal tree for initial
domain creation as well as training/evaluation. Oldv3 reused oldv2 domains for a
single-variable diagnostic; those old masks and outputs are untouched. There
is no mask expansion after optimization. Actual over-window heldout points
retain.30m capped loss in the original fixed denominator. Per-frame losses are
combined with equal frame weights. Domain ratio, absolute cell counts, spatial
extent, normal distribution, support loss and original full-cloud coverage are
reported. Unique target voxel cells do not represent independent measurements.

Reviewed fitter/evaluator numerical rules remain unchanged: correspondence.30m,
domain coverage.65, at least50 train cells/matches and15 valid target cells in
every heldout scan, bounded normal quality,20 train steps, original update
bounds, bidirectional cycle.03m/1degree, and fixed heldout loss no worse than
seed+.002m. `frozen_rules.json` records the complete numeric rule dataclass.

Full graph adoption requires every station, connectivity and pure-geometry rank
`6*(N-1)` after fixing the first ordered station's original LIO seed. There is
no odometry prior or information floor to hide weak directions. Information is
reported at a fixed1m/rad scene scale, including eigenvalues, weak axes and
condition. Every accepted edge must have a different-pair alternate path with
consistent measurement closure; deleting it must retain full connectivity and
rank. Shared scans/surfaces mean edges remain statistically correlated.

Only after those structural guards pass is the complete SE3 graph optimized.
The reviewed objective, Huber2, default parameter scaling, tolerances and600
evaluation cap are retained; the small graph uses dense exact trust-region
linear solves. Success requires genuine solver convergence before the cap,
finite SE3, exact first gauge, node updates within.5m/10degrees, edge residuals
within.03m/1degree, and every fixed raw heldout loss no worse than both
seed+.002m and the corresponding independent pairfit+.002m. The actual adopted
forward transform and its inverse are evaluated; a separately optimized reverse
fit is not substituted. Full geometry/redundancy is checked again afterward.

Rejected candidates remain explicitly named/flagged and are never silently
converted into a partial adopted map. Even a passing nodepose result is a
diagnostic only: this pipeline has no map-export command. Any later authorized
export must fuse retained raw scans at the original voxel resolution; it cannot
relabel the.05m local registration cloud as a dense.03m map.

## Reproducibility and checks

Each stage saves direct input hashes and an artifact hash manifest. Downstream
stages validate the artifacts they actually consume against the upstream
completed stage. The transitive provenance chain retains the original bag
hashes without repeatedly reading unrelated multi-GB bag payloads. Cache arrays
are memory-mapped; only two station NN trees are retained at a time. Every domain
is frozen on disk before pair fitting starts.
Local provenance also records `ldd`, actual resolved registration runtime
libraries where present, the native link record and pinned small-GICP link
input hashes. The current adapter includes its experimental frontend statically;
GICP templates can be compiled into it with the shared link input removed by
`--as-needed`. An absent runtime dependency is disclosed rather than claimed as
a loaded library. Every local pose, including auxiliary geometry frames, must
be finite SE3 with the correct base/sensor composition.

`bash tools/global_relocalization/station_submaps/check.bash` runs unit/contract
checks and the existing native trusted-calibration test if its build is present.
It never launches ROS, builds, reads live bags, replays, or exports maps. Tests
cover actual gyro turns with a tilted sensor, exact boundary ownership,
incomplete turns, IMU gaps, same-position closure visits, immutable provenance,
good-normal/true-over-window semantics, missing normals, exact legacy numeric
equivalence, arbitrary graph size, weak geometry/bridges, actual inverse heldout
rejection, and a complete synthetic3-visit stage chain. Synthetic fixture
success is not evidence of real capture quality or global alias resolution.
