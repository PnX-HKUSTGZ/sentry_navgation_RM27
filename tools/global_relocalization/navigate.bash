#!/usr/bin/env bash
# RViz spawn selection -> global localization -> normal manual-goal navigation.
set -eo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
if ! /usr/bin/python3 "$root/tools/global_relocalization/runtime.py" check-build; then
  echo 'Building the current global relocalization sources...'
  bash "$root/tools/global_relocalization/build.bash"
fi
source "$root/tools/global_relocalization/environment.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-87}"
export GZ_PARTITION="${GZ_PARTITION:-rm27_navigation_${ROS_DOMAIN_ID}_$$}"
export FASTDDS_DEFAULT_PROFILES_FILE="${FASTDDS_DEFAULT_PROFILES_FILE:-$root/tools/global_relocalization/config/fastdds_large_cloud.xml}"
export FASTRTPS_DEFAULT_PROFILES_FILE="$FASTDDS_DEFAULT_PROFILES_FILE"
# Use the normal physical wheel-force actuator: gravity must remain unconstrained.
exec ros2 launch rm_27_stimulation sim_with_nav.launch.py \
  global_relocalization:=true gui:=true use_rviz:=true \
  navigation_mode:=minco use_ground_truth_odom:=false \
  relocalization_map:="$root/tools/global_relocalization/assets/RMUC2026/map.pcd" \
  relocalization_manifest:="$root/tools/global_relocalization/assets/RMUC2026/manifest.json" \
  "$@"
