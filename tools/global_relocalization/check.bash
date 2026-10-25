#!/usr/bin/env bash
set -eo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
bash "$root/tools/global_relocalization/build.bash"
source "$root/tools/global_relocalization/environment.bash"
export GLOBAL_RELOCALIZATION_EXPECTED_CORE_LIBRARY="$root/tools/global_relocalization/.build/ros/install/global_relocalization/lib/libglobal_relocalization_core.so"
ctest --test-dir "$root/tools/global_relocalization/.build/ros/build/global_relocalization" --output-on-failure
/usr/bin/python3 -m pytest -q "$root/tools/global_relocalization/test_evaluation.py" \
  "$root/tools/global_relocalization/test_runtime.py" "$root/tools/global_relocalization/test_reset_pose.py"
