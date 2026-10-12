# Simulation rendering validation

Validated on WSL2 Ubuntu 24.04 with ROS 2 Jazzy, Gazebo Harmonic, and an
NVIDIA GeForce RTX 5070 Ti Laptop GPU. The live run used the RMUC2026 world
with `render_mode:=gpu`, Gazebo GUI, and RViz enabled.

The launch log selected `backend=glx` and reported
`D3D12 (NVIDIA GeForce RTX 5070 Ti Laptop GPU)` for both Gazebo and RViz.
Gazebo's Ogre log recorded OpenGL 4.6 Core Profile and the same D3D12 NVIDIA
renderer. RViz's own log recorded OpenGL 4.6 (GLSL 4.6). The Gazebo and RViz
processes inherited the WSL D3D12/NVIDIA selection and X11 backend settings;
the run did not use a software-rendering override.

The live ROS graph exposed `/clock`, `/ground_truth/odometry`, `/livox/lidar`,
`/livox/imu`, and `/registered_scan`. A nearby `NavigateToPose` goal was
accepted and completed with `error_code: 0` and status `SUCCEEDED`; action
feedback showed the remaining distance drop from about 9.46 m to 0.28 m.

All 17 new rendering helper and launch tests passed. The post-change
navigation test run reported 58 passed and the same 9 failures seen in the
baseline run: `rmuc2026_ground_elevation_grid_is_current_generator_output`,
`simulation_minco_uses_no_elevation_ground_connectivity`,
`simulation_collision_height_and_filter_clear_250mm_roof`,
`reality_global_seed_ignores_rolling_rog_frontier`,
`minco_uses_one_complete_deployment_profile`,
`reality_short_ramp_speed_and_connectivity_contract`,
`simulation_minco_gate_clearance_accounts_for_voxel_quantization`,
`simulation_minco_speed_limit_is_faster_but_stays_inside_mpc_envelope`, and
`active_footprint_bootstrap_covers_one_rog_seed_step`.

Per user direction, no GPU-versus-CPU performance comparison was run and no
RTF conclusion is reported. An initial stats capture targeted the wrong Gazebo
world topic and did not provide a valid RTF sample.
