#!/usr/bin/env bash
# Build the repository's Point-LIO revision without changing its submodule worktree.
set -eo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
source "$root/tools/global_relocalization/environment.bash"
snapshot="$root/tools/global_relocalization/.build/mapping_source"
base="$root/tools/global_relocalization/.build/mapping_ros"
revision="$(git -C "$root/src/point_lio" rev-parse HEAD)"
python3 "$root/tools/global_relocalization/prepare_pointlio_source.py" --root "$root" --revision "$revision"
export CMAKE_BUILD_PARALLEL_LEVEL="${CMAKE_BUILD_PARALLEL_LEVEL:-2}"
colcon --log-base "$base/log" build --base-paths "$snapshot" \
  --build-base "$base/build" --install-base "$base/install" \
  --executor sequential --allow-overriding point_lio --cmake-args -DCMAKE_BUILD_TYPE=Release
sensor_base="$root/tools/global_relocalization/.build/sensor_timing_ros"
colcon --log-base "$sensor_base/log" build --base-paths "$root/src/ign_sim_pointcloud_tool" \
  --build-base "$sensor_base/build" --install-base "$sensor_base/install" \
  --executor sequential --allow-overriding ign_sim_pointcloud_tool \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
