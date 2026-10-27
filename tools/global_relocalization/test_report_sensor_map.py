import unittest

from report_sensor_map import (altitude_strata, failure_reasons_for_profile,
                               holdout_status, outcome_metrics)


class ReportSensorMapTests(unittest.TestCase):
    def test_legacy_and_regression_never_pass(self):
        summary = {"planned": 1, "completed": 1, "acceptance_passed": True}
        audit = {"integrity_passed": True, "acceptance_eligible": True, "acceptance_passed": True}
        legacy = holdout_status({"trials": []}, summary, audit)
        self.assertFalse(legacy["passed"])
        self.assertIn("legacy", legacy["status"])
        frozen = {"schema_version": 2, "historical_validation": {"history_source_count": 1},
                  "trial_source": {"kind": "generated"}}
        summary["regression_mode"] = True
        regression = holdout_status(frozen, summary, audit)
        self.assertFalse(regression["passed"])
        self.assertIn("regression", regression["status"])

    def test_incomplete_never_passes_and_altitude_denominator_includes_failures(self):
        frozen = {"schema_version": 2, "historical_validation": {"history_source_count": 1},
                  "trial_source": {"kind": "generated", "seed": 9},
                  "trials": [
                      {"trial_id": "a", "requested_pose": {"z": .20}},
                      {"trial_id": "b", "requested_pose": {"z": .20}},
                      {"trial_id": "c", "requested_pose": {"z": .30}},
                  ]}
        summary = {"planned": 3, "completed": 2, "acceptance_passed": False}
        audit = {"integrity_passed": True, "acceptance_eligible": True, "acceptance_passed": False}
        self.assertFalse(holdout_status(frozen, summary, audit)["passed"])
        groups = altitude_strata(frozen, [
            {"trial_id": "a", "accepted": "True", "position_error_m": ".12",
             "orientation_error_deg": "2.5", "trigger_to_pose_sec": ".8"},
            {"trial_id": "b", "accepted": "False", "position_error_m": "",
             "orientation_error_deg": "", "trigger_to_pose_sec": ""},
        ], "sensor_map")
        self.assertEqual(groups["0.20"], {"planned": 2, "strict": 0, "profile": 1, "fast": 1, "completed": 2})
        self.assertEqual(groups["0.30"]["planned"], 1)
        self.assertEqual(groups["0.30"]["completed"], 0)

    def test_verified_complete_joint_acceptance_passes(self):
        frozen = {"schema_version": 2, "historical_validation": {"history_source_count": 1},
                  "trial_source": {"kind": "generated", "seed": 4}}
        summary = {"planned": 1, "completed": 1, "acceptance_passed": True}
        audit = {"integrity_passed": True, "acceptance_eligible": True, "acceptance_passed": True}
        self.assertTrue(holdout_status(frozen, summary, audit)["passed"])
        stopped = holdout_status(frozen, summary, audit, {"reason": "operator stop"})
        self.assertFalse(stopped["passed"])
        self.assertEqual(stopped["status"], "未完成：不得判为通过")

    def test_always_reports_strict_and_both_profile_joint_counts(self):
        metrics = outcome_metrics([
            {"accepted": "True", "position_error_m": ".12", "orientation_error_deg": "2.5",
             "trigger_to_pose_sec": ".8"},
            {"accepted": "True", "position_error_m": ".08", "orientation_error_deg": "1.5",
             "trigger_to_pose_sec": "1.2"},
        ], 3)
        self.assertEqual(metrics["strict"], 1)
        self.assertEqual(metrics["ideal"], 1)
        self.assertEqual(metrics["ideal_fast"], 0)
        self.assertEqual(metrics["sensor_map"], 2)
        self.assertEqual(metrics["sensor_map_fast"], 1)
        self.assertEqual(metrics["planned"], 3)

    def test_runtime_source_changes_are_distinct_from_frozen_freshness(self):
        frozen = {"schema_version": 2, "historical_validation": {"history_source_count": 1},
                  "trial_source": {"kind": "generated", "seed": 4}}
        summary = {"planned": 1, "completed": 1, "acceptance_passed": True}
        audit = {"integrity_passed": True, "acceptance_eligible": True, "acceptance_passed": True,
                 "frozen_input_sources": [{"path": "/src/core.cpp", "frozen_sha256": "old",
                                            "runtime_sha256": "new", "runtime_matches_frozen": False}]}
        status = holdout_status(frozen, summary, audit)
        self.assertTrue(status["passed"])
        self.assertEqual(status["runtime_sources_changed"], 1)
        self.assertEqual(status["runtime_source_hashes"][0]["frozen_sha256"], "old")

    def test_failure_summary_uses_selected_profile_not_legacy_strict_label(self):
        failures = failure_reasons_for_profile([
            {"accepted": "True", "position_error_m": ".1089", "orientation_error_deg": "1.17",
             "trigger_to_pose_sec": ".8"},
            {"accepted": "True", "position_error_m": ".1566", "orientation_error_deg": "1.96",
             "trigger_to_pose_sec": ".8"},
            {"accepted": "False", "position_error_m": "", "orientation_error_deg": "",
             "trigger_to_pose_sec": "", "reason": "IMU stationary calibration timeout"},
        ], "sensor_map")
        self.assertEqual(failures, {"accepted pose exceeds sensor_map pose limits": 1,
                                    "IMU stationary calibration timeout": 1})


if __name__ == "__main__":
    unittest.main()
