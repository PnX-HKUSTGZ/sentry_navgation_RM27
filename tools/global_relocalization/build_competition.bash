#!/usr/bin/env bash
# Freeze current sources once, then build only into an explicitly new directory.
set -eo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
base="${1:?Usage: build_competition.bash /absolute/new-build-directory}"
[[ "$base" = /* && ! -e "$base" ]] || { echo 'Build directory must be new and absolute' >&2; exit 2; }
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
source /opt/ros/jazzy/setup.bash
source "${GLOBAL_RELOCALIZATION_DEPENDENCY_INSTALL:-/home/zhang/ros_ws_jazzy/sentry_navgation_RM27/install}/local_setup.bash"
mkdir -p "$base/source"
cp -a "$root/src/global_relocalization" "$root/src/global_relocalization_interfaces" "$base/source/"
/usr/bin/python3 - "$base" <<'PY'
import hashlib,json,sys
from pathlib import Path
base=Path(sys.argv[1]);files=sorted(p for p in (base/'source').rglob('*') if p.is_file() and '__pycache__' not in p.parts)
(base/'source_manifest.json').write_text(json.dumps({str(p.relative_to(base)):hashlib.sha256(p.read_bytes()).hexdigest() for p in files},indent=2)+'\n')
PY
cache="$root/build/relocalization_manager/_deps/small_gicp-src"
[[ -f "$cache/CMakeLists.txt" ]] || cache=/home/zhang/ros_ws_jazzy/sentry_navgation_RM27/build/relocalization_manager/_deps/small_gicp-src
export CMAKE_BUILD_PARALLEL_LEVEL=2
colcon --log-base "$base/log" build --base-paths "$base/source" --build-base "$base/build" --install-base "$base/install" --executor sequential --cmake-args -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Release "-DFETCHCONTENT_SOURCE_DIR_SMALL_GICP=$cache"
