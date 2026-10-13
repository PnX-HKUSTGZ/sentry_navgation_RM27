#!/usr/bin/env bash
set -eo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
source "$root/tools/global_relocalization/environment.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-86}"
exec /usr/bin/python3 "$root/src/global_relocalization/scripts/capture_map.py" "$@"
