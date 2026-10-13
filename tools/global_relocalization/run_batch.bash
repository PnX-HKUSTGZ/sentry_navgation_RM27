#!/usr/bin/env bash
set -eo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
source "$root/tools/global_relocalization/environment.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-86}"
export GZ_PARTITION="${GZ_PARTITION:-global_relocalization_${ROS_DOMAIN_ID}}"
exec /usr/bin/python3 "$root/tools/global_relocalization/batch_run.py" "$@"
