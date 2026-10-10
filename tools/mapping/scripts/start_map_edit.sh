#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
用法: tools/mapping/scripts/start_map_edit.sh [选项]

只启动独立安装的 map_edit RViz 编辑器，不启动导航。
默认使用隔离话题，不订阅现有导航地图。

选项:
  --use-sim-time true|false  是否使用仿真时钟，默认 false
  --map-file PATH            启动时加载本地地图 YAML，可省略后在界面选择
  --online-reference         显式订阅参考地图话题
  --reference-topic TOPIC    参考地图话题，默认 /map
  -h, --help                 显示帮助

可选环境变量:
  ROS_SETUP   ROS setup 文件路径，默认 /opt/ros/jazzy/setup.bash
EOF
}

use_sim_time=false
online_reference=false
reference_topic=/map
map_file=

while (($#)); do
  case "$1" in
    --use-sim-time)
      if (($# < 2)); then
        printf '选项 --use-sim-time 需要 true 或 false。\n' >&2
        usage >&2
        exit 2
      fi
      use_sim_time="$2"
      shift 2
      ;;
    --use-sim-time=*)
      use_sim_time="${1#*=}"
      shift
      ;;
    --online-reference)
      online_reference=true
      shift
      ;;
    --map-file)
      if (($# < 2)); then
        printf '选项 --map-file 需要文件路径。\n' >&2
        usage >&2
        exit 2
      fi
      map_file="$2"
      shift 2
      ;;
    --map-file=*)
      map_file="${1#*=}"
      shift
      ;;
    --reference-topic)
      if (($# < 2)); then
        printf '选项 --reference-topic 需要话题名。\n' >&2
        usage >&2
        exit 2
      fi
      reference_topic="$2"
      shift 2
      ;;
    --reference-topic=*)
      reference_topic="${1#*=}"
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      printf '未知选项: %s\n' "$1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

case "$use_sim_time" in
  true|false) ;;
  *)
    printf '%s\n' '--use-sim-time 的值必须是 true 或 false。' >&2
    exit 2
    ;;
esac
if [[ -z "$reference_topic" || "$reference_topic" != /* ]]; then
  printf '%s\n' '--reference-topic 必须是非空绝对话题名，例如 /map。' >&2
  exit 2
fi
if [[ "$online_reference" != true && "$reference_topic" != /map ]]; then
  printf '%s\n' '自定义 --reference-topic 需要同时指定 --online-reference。' >&2
  exit 2
fi
if [[ -n "$map_file" ]]; then
  if [[ ! -f "$map_file" ]]; then
    printf '本地地图文件不存在: %s\n' "$map_file" >&2
    exit 2
  fi
  map_file="$(cd -- "$(dirname -- "$map_file")" && pwd)/$(basename -- "$map_file")"
fi

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd -- "$script_dir/../../.." && pwd)"
install_setup="$repo_root/tools/mapping/.build/map_edit/install/setup.bash"
ros_setup="${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"

if [[ ! -f "$ros_setup" ]]; then
  printf 'ROS setup 文件不存在: %s\n' "$ros_setup" >&2
  exit 1
fi
if [[ ! -f "$install_setup" ]]; then
  printf '找不到独立 map_edit 安装: %s\n请先运行 tools/mapping/scripts/build_map_edit.sh。\n' "$install_setup" >&2
  exit 1
fi

# Start from ROS Jazzy and the package-local install, excluding other overlays.
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH ROS_PACKAGE_PATH
unset PYTHONPATH LD_LIBRARY_PATH ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION
set +u
source "$ros_setup"
source "$install_setup"
set -u
if ! command -v ros2 >/dev/null 2>&1; then
  printf 'source ROS 和独立 map_edit install 后仍找不到 ros2。\n' >&2
  exit 1
fi
if ! ros2 pkg prefix map_edit >/dev/null 2>&1; then
  printf '当前独立安装中找不到 map_edit；请先运行 tools/mapping/scripts/build_map_edit.sh。\n' >&2
  exit 1
fi
cd "$repo_root"

exec ros2 launch map_edit map_edit.launch.py \
  "use_sim_time:=$use_sim_time" \
  "map_file:=$map_file" \
  "online_reference:=$online_reference" \
  "reference_topic:=$reference_topic"
