cd /home/pnx/nav_ws/sentry-navigation-RM27
source install/setup.bash

NAV_PROC_RE='/home/pnx/nav_[w]s/(install|build)/|[r]os2[[:space:]]+launch|[g]zserver|[g]zclient|[g]azebo|[g]z[[:space:]]+sim|[r]viz2|[r]os_gz_bridge|[p]arameter_bridge|[c]omponent_container(_mt)?|[c]ontroller_server|[p]lanner_server|[b]t_navigator|[b]ehavior_server|[w]aypoint_follower|[v]elocity_smoother|[m]ap_server|[m]ap_saver_server|[l]ifecycle_manager|[s]ync_slam_toolbox_node|[a]sync_slam_toolbox_node|[p]ointcloud_to_laserscan_node|[r]obot_state_publisher|[s]tatic_transform_publisher|[s]pawn_entity.py'

pkill -INT  -u "$USER" -f -- "$NAV_PROC_RE" || true
sleep 3
pkill -TERM -u "$USER" -f -- "$NAV_PROC_RE" || true
sleep 2
pkill -KILL -u "$USER" -f -- "$NAV_PROC_RE" || true

ros2 daemon stop 2>/dev/null || true

ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py \
  slam:=False \
  navigation_mode:=minco \
  world:=highbay3