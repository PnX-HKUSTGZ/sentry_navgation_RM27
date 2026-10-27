#!/usr/bin/env bash
# Source after build_full_slam.bash to use the complete reviewed mapping chain.
_mapping_env_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
source "$_mapping_env_root/tools/global_relocalization/environment.bash"
for _mapping_env_overlay in mapping_ros sensor_timing_ros full_slam_ros; do
  _mapping_env_setup="$_mapping_env_root/tools/global_relocalization/.build/$_mapping_env_overlay/install/local_setup.bash"
  if [[ ! -f "$_mapping_env_setup" ]]; then
    echo "Missing mapping overlay: $_mapping_env_setup. Run build_full_slam.bash first." >&2
    return 1
  fi
  source "$_mapping_env_setup"
done
unset _mapping_env_root _mapping_env_overlay _mapping_env_setup
