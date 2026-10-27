# Source-only mapping protocol: ordered route and 34 station visits

This protocol defines a **mapping source capture**, not a new localization validation batch. Preparation was completed before starting the simulator, mapper or capture. The protocol has since been executed as `mapping07`; its measured outcome is recorded below. Existing batch history and frozen maps remain unchanged. Station positions use the original planned polyline and actual collision support only; no query, GT trajectory error or failed localization position selects them.

The reviewable configuration is `tools/global_relocalization/config/survey_route_dense_20261026.json`, SHA256 `2eabec6841f4f9de53989122de8bef25888986f6e84d9b9f2ec22616353f0bfc`. It preserves every one of mapping06's **562 ordered waypoints**. The length remains **85.763230 m**; the floating summation difference is approximately `-4.3e-14 m`. The ordered controller never calls the route planner to rearrange these vertices. Every supplied segment also passed the original traversable-grid and diagonal-corner checks.

## Station rule and geometry

Starting at `[-11.7, 3.1]`, fixed arc targets occur every 2.5 m. Each target selects the nearest existing route vertex that passes actual collision support, within at most 1.25 m along the original path; ties prefer the lower route index. There are no off-route relocations. Initial and final closure observations are mandatory. This uniformly reselects the stations; it does not append old anchor stations.

Of 563 candidate vertices including the initial point, 441 pass the conservative full-turn geometry check. The selected protocol has **34 visits including initial and closure**: 21 at 0 m, eight at 0.2 m and five at 0.3 m actual support height. The narrow 0.4 m portion of the existing route has no qualifying full-footprint turn center. Elevated platforms are retained where they satisfy the same geometry rule.

The robot's maximum collision swept radius is 0.388299 m and its height above contact is 0.24 m. The added radial margin is 0.10 m: it covers the controller's 0.075 m arrival tolerance plus 0.025 m extra margin. Support sampling is 0.025 m with at most 0.005 m height variation. Stepped/sloped/unsupported footprints and swept-volume obstacles are rejected. All selected centers, heights and source hashes are included in the configuration.

The 47.5 m and 62.5 m arc targets have no qualifying center inside their declared search intervals and remain explicitly unfilled. The median station gap is 2.5 m; the maximum is **5.954163 m**. The path is not changed to conceal these gaps. The final closure is a distinct visit identity at the final existing route vertex, approximately 0.029 m from the initial point. It executes its own turn instead of being removed by XY deduplication. Legacy waypoint mode retains its original XY deduplication behavior.

## Control and sensor protocol

Each visit requests **2.05 turns** at 1 rad/s: two complete turns plus a fixed **18 degree overscan** declared before collection. The overscan and post-stop tail remain auxiliary raw data. Later train/held-out turn boundaries must come from trusted-calibration, continuous measured gyro and actual 2π/4π crossings. Controller truth yaw or elapsed command time must not stretch a short gyro turn into completion.

Before rotation, publish zero velocity and wait at least **2.5 seconds of fresh sensor time**. The first settle header must occur after the zero-command ROS event; repeated callbacks or a stalled clock cannot satisfy the duration. The legacy option default remains 0.5 seconds. This duration does not replace the later raw IMU stationarity self-check.

The existing independent simulator controller may use GT to navigate. These control observations never enter mapping or scan fusion. Runtime checks stop a station if the controlled vehicle leaves the validated 0.075 m center tolerance. Moving rotation speed is zero, translation speed is at most 0.3 m/s, terrain mode is surveyed, and the existing estimator-lag stop guard remains active.

Both `capture.json["station_scans"]` and `station_scans.json` record each `station_id`, `waypoint_index`, `kind`, label and integer nanosecond events: `capture_start_ros_ns`, `settle_zero_command_ros_ns`, `settle_start_sensor_stamp_ns`, `settle_end_sensor_stamp_ns`, `rotation_command_start_ros_ns`, `stop_command_ros_ns`, and `capture_end_ros_ns`. ROS events use the survey node's simulation clock. Sensor fields are explicitly the latest LIO header and can lag command events. Existing float fields remain for compatibility. The observed controller XYZ/yaw fields are marked as excluded from estimation. The exact protocol is archived as `survey_route_protocol.json` in the new capture.

## Executed capture command

The capture used the audited domain82 / v3 simulation and matching 64 MiB DDS environment. The command below completed successfully; its output directory is now frozen and must not be reused:

```bash
source tools/global_relocalization/mapping_environment.bash
python3 tools/global_relocalization/full_slam_mapping.py \
  --output tools/global_relocalization/data/optimization_20261026/mapping07 \
  --mapper tools/global_relocalization/.build/mapping_ros/install/point_lio/lib/point_lio/pointlio_mapping \
  --control-source simulator --survey-terrain surveyed \
  --survey-route tools/global_relocalization/config/survey_route_dense_20261026.json \
  --speed 0.3 --rotation-speed 0 --station-scan-speed 1.0 \
  --station-scan-turns 2.05 --station-settle-sec 2.5 \
  --max-estimator-lag 0.5 --max-wall-sec 1600
```

`--survey-route` is mutually exclusive with `--survey-waypoints`. Protocol loading verifies its source hashes, waypoint order, station identities and declared minimum control settings. Using fewer turns, a shorter settle, faster motion or a different controller/terrain mode is rejected.

## Time, disk and checks

Ideal translation takes 285.9 s and ideal station turn/settle/post-stop time totals 539.9 s. With a predeclared 1.35 transit factor, 1.2 s turn overhead per visit and 60 s startup/export allowance, the estimate is **1026.7 s, approximately 17.1 minutes**. This is a planning estimate, not measured capture performance.

Mapping06's recorded bag rate is 29.63 MB/s. At that rate, the expected new bag is **28.3 GiB**, with **42.5 GiB** reserved at 1.5 times the estimated duration. An optional complete raw PCD extraction adds about 7.9 GiB at the expected duration. A **60 GiB workspace budget** covers the duration allowance, raw extraction and derived maps.

Sixteen applicable unit tests pass, covering unchanged route order, changed-source rejection, unsafe initial/closure rejection, diagonal obstacle checks, sensor-time settling and stall failure, declared overscan/settle limits, measured yaw crossing behavior, and closure identity independent of legacy XY deduplication. The actual protocol also passed all 560 source hashes, all 562 original vertices and every original grid segment. Evidence is in `data/optimization_20261026/source_mapping_protocol_20261026/validation.json`.

## Measured capture outcome

`mapping07` completed all 562 waypoints and all 34 visits. The closed bag contains 10,056 raw LiDAR frames and 100,564 IMU messages; 9,983 registered clouds were saved. Mapper diagnostics report zero LiDAR/IMU callback gaps and a maximum processing time of 66.25 ms, below the 100 ms scan period. The frozen controller inputs remained unchanged. Evidence is in `mapping07/source_capture_integrity.json` and `mapping07_execution_inputs/frozen_inputs.json` under the optimization data directory.

The independent station preparation passed all 34 fixed-window IMU stationarity checks and measured two complete gyro turns at every visit. It retained 6,204 raw station-window scans: 869 predeclared settle-prefix scans remain auxiliary, and 5,335 scans enter local geometry (2,489 first-turn training, 2,492 second-turn held-out, and 354 pre/post-turn auxiliary scans). No failed station or incomplete turn was dropped.

The first local-geometry replay completed 33 of 34 visits. One registration entered a two-cycle; increasing the isolated iteration budget to 50 did not resolve it. A separate bounded-covariance experiment fixed that visit but introduced a convergence failure in another visit, so it was not adopted. The current isolated experiment retains stock covariance and permits one mapping-only retry on the next actual scan. Its rejected scan remains rejected and is not fused or trained; an explicitly marked, bounded IMU bridge may supply its held-out evaluation pose without deleting raw points from the validation denominator. The source capture is complete, but cross-station geometry and downstream localization acceptance remain unverified. These mapping results do not replace the required fresh-point, 100% localization batch.
