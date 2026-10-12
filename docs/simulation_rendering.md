# Simulation rendering modes

`render_mode` controls Gazebo and RViz rendering for the RM27 simulation:

| Mode | Behavior |
| --- | --- |
| `auto` | Probe the available renderer and use a supported GPU path when available; otherwise use CPU rendering. This is the default. |
| `gpu` | Require GPU rendering. Startup fails with a diagnostic if the requested GPU path cannot be initialized. |
| `cpu` | Force Mesa llvmpipe software rendering. |

Use the startup wrapper and pass a launch argument to select a mode:

```bash
./tools/stimulation/start_nav_stimulation.bash render_mode:=auto
./tools/stimulation/start_nav_stimulation.bash render_mode:=gpu
./tools/stimulation/start_nav_stimulation.bash render_mode:=cpu
```

The wrapper defaults to `auto` and forwards launch arguments, so the same option
can be used with `ros2 launch rm_27_stimulation sim_with_nav.launch.py` directly.
The legacy `hardware_acceleration:=true|false` and
`rviz_hardware_acceleration:=true|false` arguments remain supported when
`render_mode` is omitted: `true` selects `auto`, and `false` selects `cpu`. An
explicit `render_mode` takes precedence.

On WSL2, GPU rendering uses the WSLg X11/GLX display with Mesa's D3D12 driver
and the NVIDIA adapter. It does not use CUDA, VirtualGL, or headless EGL. Both
GPU and CPU sensor rendering require WSLg's working display connection, even
when `gui:=false`; CPU mode selects llvmpipe through GLX. If the NVIDIA D3D12
adapter is unavailable but WSLg still works, use `render_mode:=cpu` or `auto`.
Without a usable WSLg display, Gazebo cannot render its simulated sensors.
On native Linux, Gazebo uses EGL for sensor rendering with `gui:=false`.

Rendering changes the graphics backend only. It does not change the simulated
sensor models, their placement, or their ROS topic names and message layout.
The CPU mode uses llvmpipe and can reduce simulation speed.

To inspect renderer selection without starting the simulation, run the helper
after building and sourcing the workspace:

```bash
python3 -m rm27_rendering --mode auto --purpose gazebo --gui true
python3 -m rm27_rendering --mode gpu --purpose gazebo --gui false
python3 -m rm27_rendering --mode cpu --purpose rviz --gui true
```

The helper prints JSON with the requested and effective modes, selected backend
and renderer, environment changes, and any warning. A GPU request exits with a
nonzero status if GPU initialization fails.

See [the live GPU validation report](simulation_rendering_validation.md) for
the WSL2 Gazebo and RViz checks.
