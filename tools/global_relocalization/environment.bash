#!/usr/bin/env bash
# Source this file. Prefer this checkout's assets, use existing Point-LIO only as an isolated dependency.
_global_reloc_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
source /opt/ros/jazzy/setup.bash
_global_reloc_external="${GLOBAL_RELOCALIZATION_DEPENDENCY_INSTALL:-/home/zhang/ros_ws_jazzy/sentry_navgation_RM27/install}"
if [[ -f "$_global_reloc_external/local_setup.bash" ]]; then
  source "$_global_reloc_external/local_setup.bash"
fi
if [[ -f "$_global_reloc_root/install/local_setup.bash" ]]; then
  source "$_global_reloc_root/install/local_setup.bash"
fi
if [[ -f "$_global_reloc_root/tools/global_relocalization/.build/ros/install/local_setup.bash" ]]; then
  source "$_global_reloc_root/tools/global_relocalization/.build/ros/install/local_setup.bash"
fi
for _global_reloc_package in global_relocalization_interfaces global_relocalization; do
  _global_reloc_setup="$_global_reloc_root/tools/global_relocalization/.build/ros/install/$_global_reloc_package/share/$_global_reloc_package/local_setup.bash"
  if [[ -f "$_global_reloc_setup" ]]; then
    source "$_global_reloc_setup"
  fi
done
unset _global_reloc_external _global_reloc_root _global_reloc_package _global_reloc_setup
