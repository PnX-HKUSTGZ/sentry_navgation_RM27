#!/usr/bin/env bash
# Unit/contract checks only. Never builds, replays, exports a map or starts ROS.
set -eo pipefail

station_check_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
station_check_root="$(cd "$station_check_dir/../../.." && pwd)"
source "$station_check_root/tools/global_relocalization/environment.bash"

station_check_python="${STATION_SUBMAP_PYTHON:-/usr/bin/python3}"
station_check_tests=("$station_check_dir"/test_*.py)
if [[ ! -f "${station_check_tests[0]}" ]]; then
  printf 'No station submap unit tests found in %s\n' "$station_check_dir" >&2
  exit 1
fi
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
  "$station_check_python" -m pytest -q "${station_check_tests[@]}" "$@"

# The adapter is an isolated experiment and is not built by the main build.
# Validate its existing native contract when available; never rebuild it here.
station_check_build="${STATION_SUBMAP_CTEST_DIR:-$station_check_root/tools/global_relocalization/.build/station_submaps}"
if [[ -f "$station_check_build/CTestTestfile.cmake" ]]; then
  OMP_NUM_THREADS=1 ctest --test-dir "$station_check_build" \
    --output-on-failure -R '^trusted_calibration_contract$'
else
  printf 'SKIP native trusted_calibration_contract: no configured station build at %s\n' "$station_check_build"
fi
