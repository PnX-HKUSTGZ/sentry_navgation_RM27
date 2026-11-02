#!/usr/bin/env bash
# Source this file before CLI tools; the isolated final build takes precedence.
_global_reloc_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
source "${GLOBAL_RELOCALIZATION_ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
if [[ -n "${GLOBAL_RELOCALIZATION_DEPENDENCY_INSTALL:-}" ]]; then
  source "$GLOBAL_RELOCALIZATION_DEPENDENCY_INSTALL/local_setup.bash"
fi
if [[ -f "$_global_reloc_root/install/local_setup.bash" ]]; then
  source "$_global_reloc_root/install/local_setup.bash"
fi
_global_reloc_overlay="$_global_reloc_root/tools/global_relocalization/.build/ros/install"
if [[ -f "$_global_reloc_overlay/local_setup.bash" ]]; then
  source "$_global_reloc_overlay/local_setup.bash"
  export LD_LIBRARY_PATH="$_global_reloc_overlay/global_relocalization/lib:${LD_LIBRARY_PATH:-}"
fi
# The raw lidar stream requires the same large-cloud transport for every
# participant. Respect a caller-supplied profile; otherwise use the profile
# validated by the relocalization navigation entry point.
if [[ -z "${FASTDDS_DEFAULT_PROFILES_FILE:-}" && -z "${FASTRTPS_DEFAULT_PROFILES_FILE:-}" ]]; then
  export FASTDDS_DEFAULT_PROFILES_FILE="$_global_reloc_root/tools/global_relocalization/config/fastdds_large_cloud.xml"
  export FASTRTPS_DEFAULT_PROFILES_FILE="$FASTDDS_DEFAULT_PROFILES_FILE"
fi
unset _global_reloc_root _global_reloc_overlay
