#!/usr/bin/env bash
set -eo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
source "$root/tools/global_relocalization/environment.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-86}"
export GZ_PARTITION="${GZ_PARTITION:-global_relocalization_${ROS_DOMAIN_ID}}"
# The bridge is also a large-cloud writer: configuring only mapping clients
# leaves its shared-memory segment at the small middleware default.
export FASTDDS_DEFAULT_PROFILES_FILE="${FASTDDS_DEFAULT_PROFILES_FILE:-${FASTRTPS_DEFAULT_PROFILES_FILE:-$root/tools/global_relocalization/config/fastdds_large_cloud.xml}}"
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-$FASTDDS_DEFAULT_PROFILES_FILE}"
if [[ ! -f "$root/tools/global_relocalization/.build/robot_velocity.urdf" ]]; then
  bash "$root/tools/global_relocalization/prepare_simulation.bash"
fi
if [[ -f "$root/tools/global_relocalization/.build/robot_velocity.urdf" ]]; then
  set -- "robot_description_file:=$root/tools/global_relocalization/.build/robot_velocity.urdf" "$@"
fi
exec ros2 launch "$root/src/global_relocalization/launch/simulation.launch.py" "$@"
