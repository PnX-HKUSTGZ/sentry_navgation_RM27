#!/usr/bin/env bash
# One entry for full UC SLAM testing, result display, and owned-session cleanup.
set -eo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
source "$root/tools/global_relocalization/environment.bash"
mapping_install="$root/tools/global_relocalization/.build/mapping_ros/install"
if [[ ! -f "$mapping_install/local_setup.bash" ]]; then
  bash "$root/tools/global_relocalization/build_mapping.bash"
fi
source "$mapping_install/local_setup.bash"
exec /usr/bin/python3 "$root/tools/global_relocalization/mapping_session.py" "$@"
