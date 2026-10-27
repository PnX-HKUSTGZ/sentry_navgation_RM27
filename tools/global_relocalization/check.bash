#!/usr/bin/env bash
set -eo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
bash "$root/tools/global_relocalization/build.bash"
source "$root/tools/global_relocalization/environment.bash"
export GLOBAL_RELOCALIZATION_EXPECTED_CORE_LIBRARY="$root/tools/global_relocalization/.build/ros/install/global_relocalization/lib/libglobal_relocalization_core.so"
ctest --test-dir "$root/tools/global_relocalization/.build/ros/build/global_relocalization" --output-on-failure
/usr/bin/python3 -m pytest -q "$root/tools/global_relocalization/test_evaluation.py" \
  "$root/tools/global_relocalization/test_runtime.py" "$root/tools/global_relocalization/test_reset_pose.py" \
  "$root/tools/global_relocalization/test_sensor_map_batch.py" \
  "$root/tools/global_relocalization/test_holdout_trials.py" \
  "$root/tools/global_relocalization/test_actual_collision_support.py" \
  "$root/tools/global_relocalization/test_audit_sensor_batch.py" \
  "$root/tools/global_relocalization/test_report_sensor_map.py" \
  "$root/tools/global_relocalization/test_export_raw_prior.py" \
  "$root/tools/global_relocalization/test_mapping_provenance.py" \
  "$root/tools/global_relocalization/test_prepare_pointlio_source.py" \
  "$root/tools/global_relocalization/test_sensor_bag.py" \
  "$root/tools/global_relocalization/test_full_slam_mapping.py" \
  "$root/tools/global_relocalization/test_survey_protocol.py" \
  "$root/tools/global_relocalization/test_refine_sensor_prior.py" \
  "$root/tools/global_relocalization/test_export_dense_sensor_prior.py" \
  "$root/tools/global_relocalization/test_audit_sensor_plane_identifiability.py"
bash "$root/tools/global_relocalization/station_submaps/check.bash"
