cd /home/pnx/nav_ws/sentry-navigation-RM27
source install/setup.bash

pkill -SIGKILL -u "$(id -u)" -f 'ros2 launch rm_27_stimulation|gz sim|parameter_bridge|joint_state_publisher|robot_state_publisher|rm27_ground_truth_localizer|fake_vel_transform_node|map_server|lifecycle_manager|controller_server|smoother_server|planner_server|behavior_server|bt_navigator|waypoint_follower|velocity_smoother|rviz2|minco'

ros2 launch rm_27_stimulation sim_with_nav.launch.py \
  world:=RMUC2026 \
  nav_world:=auto \
  use_ground_truth_odom:=true \
  navigation_mode:=minco \
  enable_legacy_terrain:=auto \
  gui:=true \
  use_rviz:=true