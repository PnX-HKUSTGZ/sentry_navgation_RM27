#!/usr/bin/env bash
set -euo pipefail

workspace_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$workspace_root"

# This workspace targets ROS 2 Jazzy. Clear any ROS overlay inherited from a
# different workspace before loading the base installation and this workspace.
ros_distro=jazzy
ros_setup="/opt/ros/${ros_distro}/setup.bash"
if [[ ! -r "$ros_setup" ]]; then
  printf 'ROS 2 %s was not found at %s\n' "$ros_distro" "$ros_setup" >&2
  exit 1
fi

# This checkout can be used beside a previously built workspace whose
# submodules supply dependencies absent from the current source tree. Preserve
# that overlay as an underlay, then put this checkout first.
dependency_overlay_setup="${RM27_DEPENDENCY_OVERLAY:-}"
if [[ -z "$dependency_overlay_setup" && -n "${AMENT_PREFIX_PATH:-}" ]]; then
  IFS=: read -r -a inherited_prefixes <<< "$AMENT_PREFIX_PATH"
  for prefix in "${inherited_prefixes[@]}"; do
    case "$prefix" in
      */install/*)
        candidate_setup="${prefix%%/install/*}/install/setup.bash"
        if [[ -r "$candidate_setup" && "$candidate_setup" != "$workspace_root/install/setup.bash" ]]; then
          dependency_overlay_setup="$candidate_setup"
          break
        fi
        ;;
    esac
  done
fi
if [[ ! -r install/local_setup.bash ]]; then
  printf 'Workspace is not built yet. Run rosdep install and colcon build in %s\n' "$workspace_root" >&2
  exit 1
fi

unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH ROS_PACKAGE_PATH
unset PYTHONPATH LD_LIBRARY_PATH GZ_SIM_RESOURCE_PATH GZ_CONFIG_PATH
# ROS setup scripts reference optional variables, so load them with nounset off.
set +u
source "$ros_setup"
if [[ -n "$dependency_overlay_setup" && -r "$dependency_overlay_setup" ]]; then
  printf 'Loading dependency overlay: %s\n' "$dependency_overlay_setup"
  source "$dependency_overlay_setup"
fi
source install/local_setup.bash
set -u

# Leave render_mode unset here: launch resolves the empty default to auto after
# honoring any supplied legacy acceleration flag.
exec ros2 launch rm_27_stimulation sim_with_nav.launch.py \
  world:=RMUC2026 \
  nav_world:=auto \
  use_ground_truth_odom:=true \
  navigation_mode:=minco \
  enable_legacy_terrain:=auto \
  gui:=true \
  use_rviz:=true \
  "$@"
