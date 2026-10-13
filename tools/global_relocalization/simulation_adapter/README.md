# Isolated simulation drive and motion evidence

The final simulation uses Gazebo Harmonic's `gz::sim::systems::VelocityControl`
on the model, subscribing to `/cmd_vel`. `../prepare_simulation.bash` generates
an isolated robot URDF with the original wrench controller removed. The original
robot package remains unchanged. The generated JSON records the controller,
physics backend and URDF SHA256.

This drive sets model velocity components before the physics step, as implemented
in [Gazebo's VelocityControl source](https://github.com/gazebosim/gz-sim/blob/gz-sim8/src/systems/velocity_control/VelocityControl.cc).
It constrains
zero linear velocity, including the vertical component, and commands angular
velocity. Contact response can reduce the resulting yaw rate. This is a
simulation localization experiment; it does not validate a real robot's motor,
traction or torque controller. No repeated pose setting implements the spin.
Trial placement through `set_pose` occurs only before each independent trial.

The retained `planar_preupdate.cpp` is a diagnostic alternative to the project's
original Update-stage wrench controller. Both controllers received continuous
commands but produced no ground rotation in the diagnostic positions. Its stderr
markers record subscription sequence, requested torque, command-component torque
and measured angular velocity. An airborne test did produce motion; the exact
force-control/contact failure was not resolved, so moving the callback to
PreUpdate is not claimed to fix the ground rotation.

`check_physical_rotation.py` commands fixed durations using `/clock` alone:
three simulation seconds stationary, 22 seconds at 0.5 rad/s, then four seconds
stopped. GT and raw IMU are observation-only subscriptions and never affect the
command. It retains timestamped JSONL evidence and a summary of accumulated
GT yaw, translation, GT pose-derived yaw rate and raw gyro magnitude. Success
requires a full turn in the commanded direction, less than 0.3 m translation,
and both pose-derived yaw rate and raw gyro below 0.06 rad/s after settling.

The initial builtin-only full-turn observation recorded 7.848454 rad of actual
yaw and 0.216 mm net translation. The stopped pose-derived yaw rate was zero
and the largest settled raw gyro magnitude was 1.4458e-6 rad/s. In that runtime,
Gazebo's GT odometry **twist** retained the preceding 0.3565 rad/s value after
the pose stopped. The GT pose and raw gyro agreed about stopping. The checker
therefore preserves GT twist as evidence but uses pose differences and raw IMU
for its stop check. No sensor observer or GT-fed estimator was introduced.
