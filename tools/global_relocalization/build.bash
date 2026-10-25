#!/usr/bin/env bash
set -eo pipefail
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
source "$root/tools/global_relocalization/environment.bash"
base="$root/tools/global_relocalization/.build/ros"
cache="${GLOBAL_RELOCALIZATION_SMALL_GICP_SOURCE:-$root/build/relocalization_manager/_deps/small_gicp-src}"
cmake_args=(-DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Release -DFETCHCONTENT_SOURCE_DIR_SMALL_GICP=)
if [[ -f "$cache/CMakeLists.txt" ]]; then
  revision="$(git -C "$cache" rev-parse HEAD 2>/dev/null || true)"
  if [[ "$revision" != fa0cfc983417c044b6036f71dfa448038bb9c820 ]]; then
    echo "small_gicp cache must be at the pinned revision: $cache" >&2
    exit 2
  fi
  cmake_args+=("-DFETCHCONTENT_SOURCE_DIR_SMALL_GICP=$cache")
fi
export CMAKE_BUILD_PARALLEL_LEVEL="${CMAKE_BUILD_PARALLEL_LEVEL:-2}"
colcon --log-base "$base/log" build \
  --base-paths "$root/src/global_relocalization_interfaces" "$root/src/global_relocalization" \
  --build-base "$base/build" --install-base "$base/install" \
  --executor sequential --cmake-args "${cmake_args[@]}"
/usr/bin/python3 "$root/tools/global_relocalization/runtime.py" freeze-build
