#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
用法: tools/mapping/scripts/build_map_edit.sh

从仓库根目录以 ROS 2 Jazzy 构建 src/map_edit，产物只写入
tools/mapping/.build/map_edit/{build,install,log}。

可选环境变量:
  ROS_SETUP   ROS setup 文件路径，默认 /opt/ros/jazzy/setup.bash
  MAP_EDIT_JOBS  CMake 编译并发，默认 2
  COLCON_PARALLEL_WORKERS  colcon 包级并发，默认 1
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi
if (($#)); then
  usage >&2
  exit 2
fi

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd -- "$script_dir/../../.." && pwd)"
ros_setup="${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
build_jobs="${MAP_EDIT_JOBS:-2}"
parallel_workers="${COLCON_PARALLEL_WORKERS:-1}"

if [[ ! "$build_jobs" =~ ^[1-9][0-9]*$ || ! "$parallel_workers" =~ ^[1-9][0-9]*$ ]]; then
  printf 'MAP_EDIT_JOBS 和 COLCON_PARALLEL_WORKERS 必须是正整数。\n' >&2
  exit 2
fi

if [[ ! -f "$ros_setup" ]]; then
  printf 'ROS setup 文件不存在: %s\n' "$ros_setup" >&2
  exit 1
fi
if [[ ! -f "$repo_root/src/map_edit/package.xml" ]]; then
  printf '找不到 src/map_edit/package.xml；请从完整仓库运行此脚本。\n' >&2
  exit 1
fi

# Drop any active workspace overlays so colcon sees the ROS installation only.
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH ROS_PACKAGE_PATH
unset PYTHONPATH LD_LIBRARY_PATH ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION
set +u
source "$ros_setup"
set -u
export CMAKE_BUILD_PARALLEL_LEVEL="$build_jobs"
export MAKEFLAGS="-j${build_jobs} -l${build_jobs}"
if ! command -v colcon >/dev/null 2>&1; then
  printf '找不到 colcon；请先安装 ROS 2 Jazzy 的 colcon 构建工具。\n' >&2
  exit 1
fi
cd "$repo_root"

colcon \
  --log-base tools/mapping/.build/map_edit/log \
  build \
  --parallel-workers "$parallel_workers" \
  --base-paths src/map_edit \
  --build-base tools/mapping/.build/map_edit/build \
  --install-base tools/mapping/.build/map_edit/install
