#!/usr/bin/env bash
# Build the existing mapping mode and native final-graph exporter in isolation.
set -eo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
bash "$root/tools/global_relocalization/build_mapping.bash"
source "$root/tools/global_relocalization/environment.bash"
full_slam_base="$root/tools/global_relocalization/.build/full_slam_ros"
colcon --log-base "$full_slam_base/log" build --base-paths "$root/src/loam_interface" \
  --build-base "$full_slam_base/build" --install-base "$full_slam_base/install" \
  --executor sequential --allow-overriding loam_interface \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
cmake -S "$root/tools/global_relocalization/slam_export" \
  -B "$root/tools/global_relocalization/.build/slam_export" -DCMAKE_BUILD_TYPE=Release
cmake --build "$root/tools/global_relocalization/.build/slam_export" -j2
