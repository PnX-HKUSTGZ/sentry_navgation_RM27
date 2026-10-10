#!/usr/bin/env bash
# Only records existing publishers. This script never launches/stops navigation nodes.
set -euo pipefail
usage() {
  echo 'Usage: record_pointlio.sh <new_bag_directory> [--cloud-topic /cloud_registered|/cloud_registered_full]'
  echo 'Requires an already running Point-LIO and a sourced ROS2 environment. Stop recording with Ctrl-C.'
}
if [[ ${1:-} == -h || ${1:-} == --help ]]; then usage; exit 0; fi
if [[ $# -lt 1 ]]; then usage >&2; exit 2; fi
output=$1
shift
cloud=/cloud_registered
if [[ $# -gt 0 ]]; then
  if [[ $# -ne 2 || $1 != --cloud-topic ]]; then usage >&2; exit 2; fi
  cloud=$2
fi
if [[ $cloud != /cloud_registered && $cloud != /cloud_registered_full ]]; then usage >&2; exit 2; fi
if [[ -e $output || -L $output ]]; then echo "Refusing existing output: $output" >&2; exit 2; fi
if ! command -v ros2 >/dev/null; then echo 'Source /opt/ros/jazzy/setup.bash first.' >&2; exit 2; fi
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
echo "Recording existing $cloud and /aft_mapped_to_init into $output"
exec ros2 bag record --storage sqlite3 --output "$output" --qos-profile-overrides-path "$ROOT/configs/record_qos.yaml" "$cloud" /aft_mapped_to_init
