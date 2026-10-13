#!/usr/bin/env bash
set -eo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
source "$root/tools/global_relocalization/environment.bash"
base="$root/tools/global_relocalization/.build/ros"
mkdir -p "$base"
cache="$root/build/relocalization_manager/_deps/small_gicp-src"
if [[ ! -f "$cache/CMakeLists.txt" ]]; then
  cache="/home/zhang/ros_ws_jazzy/sentry_navgation_RM27/build/relocalization_manager/_deps/small_gicp-src"
fi
export CMAKE_BUILD_PARALLEL_LEVEL=2
colcon --log-base "$base/log" build \
  --base-paths "$root/src/global_relocalization_interfaces" "$root/src/global_relocalization" \
  --build-base "$base/build" --install-base "$base/install" \
  --executor sequential --cmake-args -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Release \
  "-DFETCHCONTENT_SOURCE_DIR_SMALL_GICP=$cache"
