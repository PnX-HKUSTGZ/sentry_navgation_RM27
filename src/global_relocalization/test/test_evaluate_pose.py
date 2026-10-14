"""Offline tests for the evaluator's transform and scoring math."""

from __future__ import annotations

import math
import hashlib
import json
import pathlib
import sys
import tempfile
import unittest

SCRIPT_DIR = pathlib.Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPT_DIR))

import evaluate_pose as evaluator  # noqa: E402


def close(actual: float, expected: float, places: int = 7) -> None:
    if not math.isclose(actual, expected, abs_tol=10 ** (-places), rel_tol=0.0):
        raise AssertionError(f"{actual!r} != {expected!r} within {places} decimal places")


def pose(x=0.0, y=0.0, z=0.0, roll=0.0, pitch=0.0, yaw=0.0):
    return evaluator.Pose.make(
        (x, y, z), evaluator.rpy_to_quaternion(roll, pitch, yaw)
    )


class QuaternionTests(unittest.TestCase):
    def test_quaternion_and_its_negation_are_same_rotation(self):
        q = evaluator.rpy_to_quaternion(0.31, -0.22, 1.17)
        q_neg = tuple(-value for value in q)
        close(evaluator.quaternion_geodesic_angle(q, q_neg), 0.0)
        close(evaluator.quaternion_geodesic_angle(q, q), 0.0)

    def test_slerp_handles_equivalent_negative_endpoint(self):
        q = evaluator.rpy_to_quaternion(0.0, 0.0, 0.7)
        halfway = evaluator.quaternion_slerp(q, tuple(-v for v in q), 0.5)
        close(evaluator.quaternion_geodesic_angle(q, halfway), 0.0)

    def test_positive_and_negative_ninety_degree_axis_rotations(self):
        for axis_pose in (
            pose(yaw=math.pi / 2), pose(yaw=-math.pi / 2),
            pose(roll=math.pi / 2), pose(roll=-math.pi / 2),
            pose(pitch=math.pi / 2), pose(pitch=-math.pi / 2),
        ):
            close(evaluator.quaternion_geodesic_angle(axis_pose.orientation, (0, 0, 0, 1)), math.pi / 2)
        plus = pose(yaw=math.pi / 2)
        minus = pose(yaw=-math.pi / 2)
        close(evaluator.quaternion_geodesic_angle(plus.orientation, minus.orientation), math.pi)

    def test_roll_pitch_errors_are_reported_in_full_orientation_metrics(self):
        truth = pose()
        estimate = pose(roll=math.radians(1), pitch=math.radians(-1))
        error = evaluator.calculate_pose_error(estimate, truth, pose())
        close(error["roll_error_deg"], 1.0)
        close(error["pitch_error_deg"], -1.0)
        close(error["yaw_error_deg"], 0.0)
        self.assertGreater(error["orientation_geodesic_error_deg"], 1.4)

    def test_shortest_yaw_difference_across_positive_negative_179(self):
        estimate = pose(yaw=math.radians(179))
        truth = pose(yaw=math.radians(-179))
        error = evaluator.calculate_pose_error(estimate, truth, pose())
        close(error["yaw_error_deg"], -2.0)
        close(error["orientation_geodesic_error_deg"], 2.0)


class InterpolationTests(unittest.TestCase):
    def setUp(self):
        self.buffer = evaluator.GroundTruthBuffer(max_gap_sec=0.1)

    def test_interpolates_position_and_shortest_quaternion_path(self):
        self.buffer.insert(evaluator.TimedPose(1_000_000_000, pose(0, 0, 0, yaw=math.radians(179))))
        self.buffer.insert(evaluator.TimedPose(1_020_000_000, pose(2, 4, 6, yaw=math.radians(-179))))
        result = self.buffer.lookup(1_010_000_000)
        self.assertEqual(result.validity, "interpolated")
        self.assertAlmostEqual(result.alpha, 0.5)
        self.assertEqual(result.pose.position, (1.0, 2.0, 3.0))
        close(abs(evaluator.quaternion_to_rpy(result.pose.orientation)[2]), math.pi)

    def test_exact_sample_and_no_extrapolation_before_or_after(self):
        self.buffer.insert(evaluator.TimedPose(100, pose(1)))
        self.buffer.insert(evaluator.TimedPose(200, pose(2)))
        self.assertEqual(self.buffer.lookup(100).validity, "exact")
        self.assertEqual(self.buffer.lookup(99).validity, "would_extrapolate_before_first_sample")
        self.assertEqual(self.buffer.lookup(201).validity, "need_future_bracket")

    def test_future_bracket_timeout_uses_ros_stamps_not_callback_arrival(self):
        query_stamp = 5_000_000_000
        self.assertFalse(evaluator.timestamp_timeout_elapsed(query_stamp, 5_999_999_999, 1.0))
        self.assertTrue(evaluator.timestamp_timeout_elapsed(query_stamp, 6_000_000_000, 1.0))
        self.assertFalse(evaluator.timestamp_timeout_elapsed(query_stamp, 4_000_000_000, 1.0))

    def test_rejects_interpolation_across_gap_above_limit(self):
        self.buffer.insert(evaluator.TimedPose(1_000_000_000, pose()))
        self.buffer.insert(evaluator.TimedPose(1_200_000_001, pose(1)))
        result = self.buffer.lookup(1_100_000_000)
        self.assertEqual(result.validity, "interpolation_gap_too_large")
        self.assertIsNone(result.pose)

    def test_out_of_order_insertions_remain_timestamp_sorted(self):
        self.buffer.insert(evaluator.TimedPose(300, pose(3)))
        self.buffer.insert(evaluator.TimedPose(100, pose(1)))
        self.buffer.insert(evaluator.TimedPose(200, pose(2)))
        result = self.buffer.lookup(150)
        self.assertEqual(result.validity, "interpolated")
        self.assertEqual(result.pose.position, (1.5, 0.0, 0.0))


class TransformAndDecisionTests(unittest.TestCase):
    def test_known_map_from_world_translation_and_rotation(self):
        # map <- world: rotate world +90 degrees about z, then translate.
        map_from_world = pose(10.0, -2.0, 0.5, yaw=math.pi / 2)
        truth_world = pose(1.0, 0.0, 2.0, yaw=math.pi / 2)
        truth_map = evaluator.compose_pose(map_from_world, truth_world)
        close(truth_map.position[0], 10.0)
        close(truth_map.position[1], -1.0)
        close(truth_map.position[2], 2.5)
        close(abs(evaluator.quaternion_to_rpy(truth_map.orientation)[2]), math.pi)

        error = evaluator.calculate_pose_error(truth_map, truth_world, map_from_world)
        close(error["position_3d_error_m"], 0.0)
        close(error["orientation_geodesic_error_deg"], 0.0)

    def test_manifest_transform_is_authoritative_and_frame_mismatch_is_rejected(self):
        manifest = {
            "frame_id": "map",
            "pose_parent_frame": "world",
            "pose_child_frame": "base_footprint",
            "T_map_world": [
                [0.0, -1.0, 0.0, 4.0],
                [1.0, 0.0, 0.0, -3.0],
                [0.0, 0.0, 1.0, 0.2],
                [0.0, 0.0, 0.0, 1.0],
            ],
        }
        with tempfile.TemporaryDirectory() as directory:
            path = pathlib.Path(directory) / "manifest.json"
            map_data = b"frozen-pcd-bytes"
            (path.parent / "map.pcd").write_bytes(map_data)
            manifest["map_sha256"] = hashlib.sha256(map_data).hexdigest()
            path.write_text(json.dumps(manifest), encoding="utf-8")
            from_manifest, metadata = evaluator.load_alignment_manifest(
                str(path), "map", "world", "base_footprint"
            )
            expected = pose(4.0, -3.0, 0.2, yaw=math.pi / 2)
            self.assertTrue(evaluator.transforms_match(from_manifest, expected))
            self.assertEqual(metadata["map_sha256"], manifest["map_sha256"])
            with self.assertRaisesRegex(ValueError, "pose_parent_frame"):
                evaluator.load_alignment_manifest(str(path), "map", "odom", "base_footprint")

            (path.parent / "map.pcd").write_bytes(b"changed-pcd")
            with self.assertRaisesRegex(ValueError, "does not match"):
                evaluator.load_alignment_manifest(str(path), "map", "world", "base_footprint")

    def test_transform_consistency_check_rejects_different_fixed_alignment(self):
        authoritative = pose(1.0, 0.0, 0.0)
        conflicting_cli = pose(1.01, 0.0, 0.0)
        self.assertFalse(evaluator.transforms_match(authoritative, conflicting_cli))
        self.assertTrue(evaluator.transforms_match(authoritative, pose(1.0, 0.0, 0.0)))

    def test_acceptance_requires_full_three_dimensional_position_and_so3(self):
        identity = pose()
        exact_gt = evaluator.LookupResult(identity, "exact", 0.0, 100, 100)
        accepted = evaluator.make_sample_record(
            sample_index=1, pose_stamp_ns=100, pose_frame="map",
            interpolation=exact_gt, estimate_map=pose(z=0.099, yaw=math.radians(1.9)),
            map_from_world=identity, position_threshold_m=0.1, orientation_threshold_deg=2.0,
        )
        self.assertTrue(accepted["success"])
        rejected_z = evaluator.make_sample_record(
            sample_index=2, pose_stamp_ns=100, pose_frame="map",
            interpolation=exact_gt, estimate_map=pose(z=0.101),
            map_from_world=identity, position_threshold_m=0.1, orientation_threshold_deg=2.0,
        )
        self.assertFalse(rejected_z["success"])
        self.assertGreater(rejected_z["position_3d_error_m"], 0.1)
        rejected_roll = evaluator.make_sample_record(
            sample_index=3, pose_stamp_ns=100, pose_frame="map",
            interpolation=exact_gt, estimate_map=pose(roll=math.radians(2.1)),
            map_from_world=identity, position_threshold_m=0.1, orientation_threshold_deg=2.0,
        )
        self.assertFalse(rejected_roll["success"])
        self.assertGreater(rejected_roll["orientation_geodesic_error_deg"], 2.0)

    def test_unbracketed_ground_truth_is_invalid_not_success(self):
        record = evaluator.make_sample_record(
            sample_index=1, pose_stamp_ns=123, pose_frame="map",
            interpolation=evaluator.LookupResult(None, "would_extrapolate_before_first_sample"),
            estimate_map=pose(), map_from_world=pose(),
            position_threshold_m=0.1, orientation_threshold_deg=2.0,
        )
        self.assertFalse(record["evaluation_valid"])
        self.assertFalse(record["success"])
        self.assertEqual(record["outcome"], "failure")


class BatchAggregationTests(unittest.TestCase):
    def test_empty_ledger_still_emits_summary_without_loop_variable_error(self):
        with tempfile.TemporaryDirectory() as directory:
            trials_path = pathlib.Path(directory) / "trials.jsonl"
            scores_path = pathlib.Path(directory) / "pose_errors.jsonl"
            output_prefix = pathlib.Path(directory) / "aggregate"
            trials_path.write_text("", encoding="utf-8")
            scores_path.write_text("", encoding="utf-8")
            report = evaluator.aggregate_batch(
                str(trials_path), str(scores_path), str(output_prefix), expected_trials=1
            )
            batch = report["summary"]["batch"]
            self.assertFalse(batch["complete"])
            self.assertEqual(batch["observed_trials"], 0)
            self.assertIsNone(batch["success_rate"])

    def test_duplicate_non_null_action_pose_stamps_are_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            trials_path = pathlib.Path(directory) / "trials.jsonl"
            scores_path = pathlib.Path(directory) / "pose_errors.jsonl"
            trials = [
                {"trial_id": f"trial-{index}", "action": {
                    "success": True, "timeout": False, "pose_stamp_ns": 101,
                }}
                for index in (1, 2)
            ]
            trials_path.write_text(
                "\n".join(json.dumps(row) for row in trials) + "\n", encoding="utf-8"
            )
            scores_path.write_text("", encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "duplicate trial pose stamp 101"):
                evaluator.aggregate_batch(
                    str(trials_path), str(scores_path), str(pathlib.Path(directory) / "aggregate"),
                    expected_trials=2,
                )

    def test_failed_action_and_legacy_top_level_stamp_cannot_join_scores(self):
        with tempfile.TemporaryDirectory() as directory:
            trials_path = pathlib.Path(directory) / "trials.jsonl"
            scores_path = pathlib.Path(directory) / "pose_errors.jsonl"
            output_prefix = pathlib.Path(directory) / "aggregate"
            trials = [
                {
                    "trial_id": "failed-with-stale-stamp",
                    "pose_stamp_ns": 102,
                    "action": {"success": False, "timeout": False, "pose_stamp_ns": 101},
                },
                {
                    "trial_id": "success-with-legacy-only-stamp",
                    "pose_stamp_ns": 102,
                    "action": {"success": True, "timeout": False},
                },
                {
                    "trial_id": "success-with-noninteger-stamp",
                    "action": {"success": True, "timeout": False, "pose_stamp_ns": 101.8},
                },
            ]
            scores = [
                {"pose_stamp_ns": 101, "evaluation_valid": True, "success": True,
                 "position_3d_error_m": 0.0, "orientation_geodesic_error_deg": 0.0},
                {"pose_stamp_ns": 102, "evaluation_valid": True, "success": True,
                 "position_3d_error_m": 0.0, "orientation_geodesic_error_deg": 0.0},
            ]
            trials_path.write_text(
                "\n".join(json.dumps(row) for row in trials) + "\n", encoding="utf-8"
            )
            scores_path.write_text(
                "\n".join(json.dumps(row) for row in scores) + "\n", encoding="utf-8"
            )

            report = evaluator.aggregate_batch(
                str(trials_path), str(scores_path), str(output_prefix), expected_trials=3
            )

            rows = report["trials"]
            self.assertEqual(report["summary"]["batch"]["observed_trials"], 3)
            self.assertEqual(report["summary"]["batch"]["success_count"], 0)
            self.assertTrue(all(row["pose_stamp_ns"] is None for row in rows))
            self.assertTrue(all(row["evaluation_valid"] is False for row in rows))
            self.assertEqual(report["summary"]["accuracy"]["valid_pose_count"], 0)

    def test_failures_and_timeouts_remain_in_success_rate_denominator(self):
        with tempfile.TemporaryDirectory() as directory:
            trials_path = pathlib.Path(directory) / "trials.jsonl"
            scores_path = pathlib.Path(directory) / "pose_errors.jsonl"
            output_prefix = pathlib.Path(directory) / "aggregate"
            trials = [
                {
                    "trial_id": "trial-1", "trial_index": 1,
                    "requested_pose": {"x": 1.0, "y": 2.0, "z": 0.0, "yaw": 0.0},
                    "action": {"success": True, "reason": "ok", "timeout": False, "pose_stamp_ns": 101},
                    "timing_sec": {"total_ros": 2.0, "total_wall": 3.0,
                                   "static": {"ros": 0.2, "wall": 0.3},
                                   "sweep": {"ros": 1.0, "wall": 1.2},
                                   "retrieval": {"ros": None, "wall": 0.1},
                                   "gicp": {"ros": None, "wall": 0.2},
                                   "confirmation": {"ros": None, "wall": 0.05},
                                   "search_total": {"ros": None, "wall": 0.6},
                                   "stop_settle": {"ros": 0.4, "wall": 0.5}},
                    "algorithm_process": {"cpu_user_delta_sec": 0.5, "cpu_sys_delta_sec": 0.1,
                                          "wall_sec": 3.0, "cpu_percent": 20.0,
                                          "core_seconds": 0.6, "peak_rss_bytes": 1000},
                    "lio_pipeline": {"cpu_user_delta_sec": 1.0, "cpu_sys_delta_sec": 0.2,
                                     "wall_sec": 3.0, "cpu_percent": 40.0,
                                     "core_seconds": 1.2, "peak_rss_bytes": 2000},
                    "gazebo_process_load": {"cpu_user_delta_sec": 2.0, "cpu_sys_delta_sec": 0.5,
                                            "wall_sec": 3.0, "cpu_percent": 83.0,
                                            "core_seconds": 2.5, "peak_rss_bytes": 3000},
                    "rtf": 0.8,
                    "environment": {"ros_distro": "jazzy"},
                },
                {
                    "trial_id": "trial-2", "trial_index": 2,
                    "requested_pose": {"x": 1.0, "y": 2.0, "z": 0.0, "yaw": math.pi / 2},
                    "action": {"success": True, "reason": "ok", "timeout": False, "pose_stamp_ns": 202},
                },
                {"trial_id": "trial-3", "trial_index": 3,
                 "action": {"success": False, "reason": "deadline", "timeout": True}},
            ]
            score = {
                "pose_stamp_ns": 101, "evaluation_valid": True, "success": True,
                "ground_truth_interpolation_validity": "interpolated",
                "position_3d_error_m": 0.04, "position_xy_error_m": 0.03,
                "orientation_geodesic_error_deg": 0.8, "dx_m": 0.03, "dy_m": 0.0,
                "dz_m": -0.0265, "roll_error_deg": 0.0, "pitch_error_deg": 0.0,
                "yaw_error_deg": 0.8, "position_threshold_m": 0.1,
                "orientation_threshold_deg": 2.0,
                "T_map_world_translation_m": [0.0, 0.0, 0.0],
                "T_map_world_quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
                "alignment_metadata": {"map_sha256": "same-map"},
            }
            failed_accuracy_score = dict(score)
            failed_accuracy_score.update({
                "pose_stamp_ns": 202, "success": False,
                "position_3d_error_m": 0.3, "position_xy_error_m": 0.3,
                "orientation_geodesic_error_deg": 3.0, "dx_m": 0.3,
                "yaw_error_deg": 3.0,
            })
            trials_path.write_text("\n".join(json.dumps(row) for row in trials) + "\n", encoding="utf-8")
            scores_path.write_text(
                json.dumps(score) + "\n" + json.dumps(failed_accuracy_score) + "\n",
                encoding="utf-8",
            )
            report = evaluator.aggregate_batch(
                str(trials_path), str(scores_path), str(output_prefix), expected_trials=3
            )
            batch = report["summary"]["batch"]
            self.assertTrue(batch["complete"])
            self.assertEqual(batch["observed_trials"], 3)
            self.assertEqual(batch["success_count"], 1)
            self.assertEqual(batch["failure_count"], 2)
            recovery = report["summary"]["algorithm_recovery_outcomes"]
            self.assertEqual(recovery["wrong_pose_accepted_count"], 1)
            self.assertEqual(recovery["action_failure_count"], 1)
            self.assertAlmostEqual(batch["failure_rate"], 2.0 / 3.0)
            self.assertEqual(batch["timeout_count"], 1)
            self.assertAlmostEqual(batch["success_rate"], 1.0 / 3.0)
            self.assertAlmostEqual(report["summary"]["accuracy"]["overall_position_rmse_m"], math.sqrt((0.04**2 + 0.3**2) / 2.0))
            position_stats = report["summary"]["accuracy"]["position_3d_error_m"]
            self.assertAlmostEqual(position_stats["mean"], 0.17)
            self.assertAlmostEqual(position_stats["rmse"], math.sqrt((0.04**2 + 0.3**2) / 2.0))
            self.assertAlmostEqual(position_stats["p95"], 0.287)
            self.assertEqual(position_stats["max"], 0.3)
            orientation_stats = report["summary"]["accuracy"]["orientation_geodesic_error_deg"]
            self.assertAlmostEqual(orientation_stats["mean"], 1.9)
            self.assertAlmostEqual(orientation_stats["rmse"], math.sqrt((0.8**2 + 3.0**2) / 2.0))
            self.assertAlmostEqual(orientation_stats["p95"], 2.89)
            self.assertEqual(orientation_stats["max"], 3.0)
            nonsuccess = report["summary"]["accuracy_cohorts"]["valid_pose_outputs_from_non_successful_trials"]
            self.assertEqual(nonsuccess["valid_pose_count"], 1)
            self.assertAlmostEqual(nonsuccess["overall_position_rmse_m"], 0.3)
            self.assertEqual(report["summary"]["timing"]["sweep"]["ros_sec"]["p50"], 1.0)
            self.assertEqual(report["summary"]["timing"]["sweep"]["ros_sec"]["mean"], 1.0)
            self.assertEqual(report["summary"]["timing"]["search_total"]["wall_sec"]["p50"], 0.6)
            self.assertEqual(report["summary"]["timing"]["search_total"]["ros_sec"]["count"], 0)
            self.assertEqual(report["summary"]["timing"]["stop_settle"]["wall_sec"]["p50"], 0.5)
            self.assertAlmostEqual(report["summary"]["timing"]["capture_total"]["ros_sec"]["mean"], 1.6)
            self.assertAlmostEqual(report["summary"]["timing"]["capture_total"]["wall_sec"]["mean"], 2.0)
            self.assertIsNone(report["summary"]["timing"]["computation_total"]["ros_sec"]["mean"])
            self.assertAlmostEqual(report["summary"]["timing"]["computation_total"]["wall_sec"]["mean"], 0.35)
            self.assertEqual(report["summary"]["algorithm_process"]["cpu_percent"]["p50"], 20.0)
            self.assertEqual(report["summary"]["algorithm_process"]["cpu_percent"]["mean"], 20.0)
            self.assertEqual(report["summary"]["lio_pipeline"]["cpu_percent"]["p50"], 40.0)
            self.assertEqual(report["summary"]["gazebo_process_load"]["cpu_percent"]["p50"], 83.0)
            self.assertIn("lio_pipeline", report["trials"][0])
            self.assertTrue((pathlib.Path(str(output_prefix) + ".summary.json")).is_file())
            self.assertTrue((pathlib.Path(str(output_prefix) + ".trials.csv")).is_file())

    def test_raw_input_observation_preserves_null_counts_and_missing_lio_cost(self):
        with tempfile.TemporaryDirectory() as directory:
            trials_path = pathlib.Path(directory) / "trials.jsonl"
            scores_path = pathlib.Path(directory) / "pose_errors.jsonl"
            output_prefix = pathlib.Path(directory) / "aggregate"
            missing_cost = {
                "pids": [], "measurement_valid": False, "cpu_user_delta_sec": None,
                "cpu_sys_delta_sec": None, "wall_sec": None, "cpu_percent": None,
                "core_seconds": None, "peak_rss_bytes": None,
            }
            trials = [
                {
                    "trial_id": "raw-success", "trial_index": 1, "input_mode": "raw_imu",
                    "action_window_exists": True,
                    "requested_pose": {"x": 1.0, "y": 2.0, "z": 0.0, "yaw": 0.0},
                    "action": {"success": True, "timeout": False, "timeout_source": None,
                               "pose_stamp_ns": 101},
                    "raw_input_observation": {
                        "counts_since_client_start": {"lidar": 120, "imu": 40},
                        "counts_during_action": {"lidar": 100, "imu": 30},
                        "readiness_used_ground_truth": False,
                    },
                    "frontend_process": {**missing_cost, "included_in_algorithm": True},
                    "lio_pipeline": {**missing_cost, "reason": "not_applicable"},
                    "algorithm_process": {"measurement_valid": True, "cpu_percent": 75.0,
                                          "peak_rss_bytes": 4096},
                    "timing_sec": {"total_wall": 2.0,
                                   "raw_lidar_preprocess": {"ros": 0.3, "wall": 0.4}},
                    "status_events": [{
                        "stage": "SUCCEEDED",
                        "raw_frontend": {
                            "measurement_valid": True,
                            "resource_group": "algorithm_process",
                            "inputs": {
                                "lidar": {"received": 10, "used": 9, "drop": 1, "stale": 0,
                                          "rejected": 0, "rejection_reasons": {}},
                                "imu": {"received": 20, "used": 18, "drop": 1, "stale": 0,
                                        "rejected": 1, "rejection_reasons": {"nonfinite": 1}},
                            },
                            "queue_depth": 0,
                            "calibrated": True,
                            "local_fitness_m": 0.03,
                            "local_inlier_ratio": 0.92,
                            "local_registration_wall_sec": 0.2,
                            "relative_translation_m": 0.001,
                            "angular_speed_rad_sec": 0.3,
                            "linear_speed_m_sec": 0.0,
                            "failure_reason": None,
                        },
                        "stage_wall_sec": {"SETTLING": 3.0, "SPINNING": 10.0,
                                           "STOPPING": 2.0, "SEARCHING": 1.0},
                        "stage_ros_sec": {"SETTLING": 3.1, "SPINNING": 9.8,
                                          "STOPPING": 2.0, "SEARCHING": 1.0},
                        "metrics": {"raw_frontend": "active"},
                    }],
                },
                {
                    "trial_id": "raw-readiness-failure", "trial_index": 2,
                    "input_mode": "raw_imu", "action_window_exists": False,
                    "action": {"success": False, "timeout": True,
                               "timeout_source": "readiness", "pose_stamp_ns": None},
                    "raw_input_observation": {
                        "counts_since_client_start": {"lidar": None, "imu": None},
                        "counts_during_action": {"lidar": None, "imu": None},
                        "readiness_used_ground_truth": False,
                    },
                },
            ]
            score = {
                "pose_stamp_ns": 101, "evaluation_valid": True, "success": True,
                "ground_truth_interpolation_validity": "interpolated",
                "position_3d_error_m": 0.001, "position_xy_error_m": 0.001,
                "orientation_geodesic_error_deg": 0.1, "dx_m": 0.001, "dy_m": 0.0,
                "dz_m": 0.0, "roll_error_deg": 0.0, "pitch_error_deg": 0.0,
                "yaw_error_deg": 0.1, "position_threshold_m": 0.1,
                "orientation_threshold_deg": 2.0,
                "T_map_world_translation_m": [0.0, 0.0, 0.0],
                "T_map_world_quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
                "alignment_metadata": {"map_sha256": "same-map"},
            }
            trials_path.write_text("\n".join(json.dumps(row) for row in trials) + "\n", encoding="utf-8")
            scores_path.write_text(json.dumps(score) + "\n", encoding="utf-8")
            report = evaluator.aggregate_batch(
                str(trials_path), str(scores_path), str(output_prefix), expected_trials=2
            )

            self.assertEqual(report["summary"]["batch"]["success_count"], 1)
            self.assertEqual(report["summary"]["batch"]["failure_count"], 1)
            self.assertEqual(report["summary"]["input_mode_counts"], {"raw_imu": 2})
            self.assertEqual(report["summary"]["action_window_count"], 1)
            self.assertEqual(report["summary"]["action_timeout_source_counts"], {"readiness": 1})
            raw_lidar = report["summary"]["raw_input_observation"]["counts_since_client_start"]["lidar"]
            self.assertEqual(raw_lidar["count"], 1)
            self.assertEqual(raw_lidar["mean"], 120.0)
            self.assertIsNone(report["trials"][1]["lio_pipeline"])
            self.assertEqual(report["summary"]["lio_pipeline"]["cpu_percent"]["count"], 0)
            self.assertIsNone(report["summary"]["lio_pipeline"]["cpu_percent"]["mean"])
            self.assertEqual(report["summary"]["raw_frontend"]["measurement_valid_trial_count"], 1)
            self.assertEqual(report["summary"]["raw_frontend"]["resource_group"], "algorithm_process")
            self.assertEqual(report["summary"]["raw_frontend"]["inputs"]["lidar"]["drop"]["mean"], 1.0)
            self.assertEqual(report["summary"]["raw_frontend"]["inputs"]["imu"]["stale"]["mean"], 0.0)
            self.assertEqual(report["summary"]["raw_frontend"]["rejection_reasons"]["imu"]["nonfinite"]["mean"], 1.0)
            self.assertEqual(report["summary"]["status_stage_timing"]["SEARCHING"]["ros_sec"]["mean"], 1.0)
            self.assertTrue(report["trials"][0]["raw_frontend"]["measurement_valid"])
            self.assertEqual(report["trials"][0]["status_events"], trials[0]["status_events"])
            self.assertEqual(
                report["summary"]["timing"]["additional_stages"]["raw_lidar_preprocess"]["wall_sec"]["mean"],
                0.4,
            )

    def test_flat_node_raw_status_is_normalized_without_inventing_drops(self):
        with tempfile.TemporaryDirectory() as directory:
            trials_path = pathlib.Path(directory) / "trials.jsonl"
            scores_path = pathlib.Path(directory) / "pose_errors.jsonl"
            output_prefix = pathlib.Path(directory) / "aggregate"
            terminal_ok = {
                "stage": "SUCCEEDED", "map_sha256": "same-map",
                "raw_frontend": {
                    "accepted": True, "reason": "full SE(3) local scan registration accepted",
                    "lidar_received": 260, "lidar_used": 220, "lidar_rejected": 0,
                    "imu_received": 2600, "imu_accepted": 2600, "imu_rejected": 0,
                    "queue_dropped": 3, "processing_sec": 0.004,
                    "worker_wall_sec": 0.006, "max_worker_wall_sec": 0.010,
                    "generation": 1, "scan_queue_capacity": 32,
                    "max_scan_message_bytes": 4194304, "scan_queue_depth": 2,
                    "scan_result_depth": 0, "scan_outstanding_count": 1,
                    "scan_queue_bytes": 1200000, "scan_queue_high_watermark": 5,
                    "scan_queue_max_bytes": 8388608, "queue_overflow_count": 3,
                    "oversized_scan_count": 0, "stale_scan_count": 0,
                    "capture_ignored_count": 0, "canceled_scan_count": 0,
                    "lidar_max_received_stamp_gap_sec": 0.05,
                    "lidar_latest_receipt_ros_delay_sec": 0.06,
                    "lidar_max_receipt_ros_delay_sec": 0.08,
                    "processed_source_age_sec": 0.02,
                    "converged": True, "transform_finite": True,
                    "iterations": 12, "first_attempt_iterations": 12, "retry_count": 0,
                    "rejected_gates": [],
                    "registration_attempts": [{"attempt": 1, "iterations": 12,
                        "converged": True, "transform_finite": True, "rejected_gates": []}],
                },
                "stage_wall_sec": {"SPINNING": 18.0, "SEARCHING": 2.0},
            }
            terminal_candidate_failure = {
                "stage": "FAILED", "map_sha256": "same-map",
                "raw_frontend": {
                    "accepted": True, "reason": "full SE(3) local scan registration accepted",
                    "lidar_received": 340, "lidar_used": 225, "lidar_rejected": 0,
                    "imu_received": 3400, "imu_accepted": 3400, "imu_rejected": 0,
                    "queue_dropped": 0, "processing_sec": 0.005,
                    "worker_wall_sec": 0.007, "max_worker_wall_sec": 0.012,
                    "generation": 2, "scan_queue_capacity": 32,
                    "max_scan_message_bytes": 4194304, "scan_queue_depth": 4,
                    "scan_result_depth": 1, "scan_outstanding_count": 2,
                    "scan_queue_bytes": 3600000, "scan_queue_high_watermark": 8,
                    "scan_queue_max_bytes": 8388608, "queue_overflow_count": 5,
                    "oversized_scan_count": 1, "stale_scan_count": 2,
                    "capture_ignored_count": 0, "canceled_scan_count": 1,
                    "lidar_max_received_stamp_gap_sec": 0.35,
                    "lidar_latest_receipt_ros_delay_sec": 0.24,
                    "lidar_max_receipt_ros_delay_sec": 0.30,
                    "processed_source_age_sec": 0.12,
                    "converged": False, "transform_finite": True,
                    "iterations": 25, "first_attempt_iterations": 15, "retry_count": 1,
                    "rejected_gates": ["coverage_gap"], "worker_error": "queue overflow",
                    "registration_attempts": [
                        {"attempt": 1, "iterations": 15, "converged": False,
                         "transform_finite": True, "rejected_gates": ["coverage_gap"]},
                        {"attempt": 2, "iterations": 10, "converged": False,
                         "transform_finite": True, "rejected_gates": ["coverage_gap"]},
                    ],
                },
                "stage_wall_sec": {"SPINNING": 18.0, "SEARCHING": 3.0},
            }
            trials = [
                {
                    "trial_id": "raw-ok", "trial_index": 1, "input_mode": "raw_imu",
                    "action_window_exists": True,
                    "action": {"success": True, "timeout": False, "pose_stamp_ns": 501},
                    "lio_pipeline": None,
                    "algorithm_process": {"measurement_valid": True, "cpu_percent": 60.0},
                    "status_events": [terminal_ok],
                },
                {
                    "trial_id": "raw-candidate-failure", "trial_index": 2,
                    "input_mode": "raw_imu", "action_window_exists": True,
                    "action": {"success": False, "timeout": False,
                               "reason": "no retrieved candidate passed GICP quality and search-side gates",
                               "pose_stamp_ns": None},
                    "lio_pipeline": None,
                    "algorithm_process": {"measurement_valid": True, "cpu_percent": 55.0},
                    "status_events": [terminal_candidate_failure],
                },
            ]
            score = {
                "pose_stamp_ns": 501, "evaluation_valid": True, "success": True,
                "ground_truth_interpolation_validity": "interpolated",
                "position_3d_error_m": 0.001, "position_xy_error_m": 0.001,
                "orientation_geodesic_error_deg": 0.1, "dx_m": 0.001, "dy_m": 0.0,
                "dz_m": 0.0, "roll_error_deg": 0.0, "pitch_error_deg": 0.0,
                "yaw_error_deg": 0.1, "position_threshold_m": 0.1,
                "orientation_threshold_deg": 2.0,
                "T_map_world_translation_m": [0.0, 0.0, 0.0],
                "T_map_world_quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
                "alignment_metadata": {"map_sha256": "same-map"},
            }
            trials_path.write_text(
                "\n".join(json.dumps(row) for row in trials) + "\n", encoding="utf-8"
            )
            scores_path.write_text(json.dumps(score) + "\n", encoding="utf-8")

            report = evaluator.aggregate_batch(
                str(trials_path), str(scores_path), str(output_prefix), expected_trials=2
            )
            summary = report["summary"]
            raw_summary = summary["raw_frontend"]
            normalized = report["trials"][0]["raw_frontend"]

            self.assertEqual(summary["batch"]["success_count"], 1)
            self.assertEqual(summary["batch"]["failure_count"], 1)
            self.assertEqual(raw_summary["measurement_valid_trial_count"], 2)
            self.assertEqual(raw_summary["inputs"]["lidar"]["received"]["mean"], 300.0)
            self.assertEqual(raw_summary["inputs"]["lidar"]["used"]["mean"], 222.5)
            self.assertEqual(raw_summary["inputs"]["lidar"]["drop"]["count"], 0)
            self.assertEqual(raw_summary["inputs"]["imu"]["accepted"]["mean"], 3000.0)
            self.assertEqual(raw_summary["inputs"]["imu"]["stale"]["count"], 0)
            self.assertIsNone(raw_summary["rejection_reasons"]["lidar"])
            self.assertEqual(raw_summary["metrics"]["queue_dropped"]["mean"], 1.5)
            self.assertEqual(raw_summary["metrics"]["queue_overflow_count"]["mean"], 4.0)
            self.assertEqual(raw_summary["metrics"]["scan_queue_capacity"]["max"], 32.0)
            self.assertAlmostEqual(raw_summary["metrics"]["lidar_max_received_stamp_gap_sec"]["mean"], 0.2)
            self.assertAlmostEqual(raw_summary["metrics"]["lidar_latest_receipt_ros_delay_sec"]["mean"], 0.15)
            self.assertEqual(raw_summary["metrics"]["retry_count"]["mean"], 0.5)
            self.assertEqual(raw_summary["metrics"]["iterations"]["mean"], 18.5)
            self.assertEqual(raw_summary["converged"]["true_count"], 1)
            self.assertEqual(raw_summary["converged"]["false_count"], 1)
            self.assertEqual(raw_summary["rejected_gate_trial_counts"], {"coverage_gap": 1})
            self.assertEqual(raw_summary["worker_error_counts"], {"queue overflow": 1})
            self.assertEqual(raw_summary["metrics"]["max_worker_wall_sec"]["max"], 0.012)
            self.assertEqual(normalized["resource_group"], "algorithm_process")
            self.assertFalse(normalized["separate_cpu_rss_accounting"])
            self.assertEqual(normalized["retry_count"], 0)
            self.assertEqual(len(normalized["registration_attempts"]), 1)
            self.assertEqual(report["trials"][0]["terminal_status"], terminal_ok)
            self.assertEqual(report["trials"][0]["status_events"], [terminal_ok])
            self.assertIsNone(report["summary"]["lio_pipeline"]["cpu_percent"]["mean"])

    def test_reset_attempts_are_summarized_without_expanding_trial_rows(self):
        with tempfile.TemporaryDirectory() as directory:
            trials_path = pathlib.Path(directory) / "trials.jsonl"
            scores_path = pathlib.Path(directory) / "pose_errors.jsonl"
            output_prefix = pathlib.Path(directory) / "aggregate"
            first_success = {
                "attempt": 1, "success": True, "classification": "success",
                "wall_sec": 0.1, "returncode": 0,
            }
            timeout = {
                "attempt": 1, "success": False, "classification": "cli_timeout",
                "wall_sec": 10.0, "returncode": None,
            }
            retry_success = {
                "attempt": 2, "success": True, "classification": "success",
                "wall_sec": 0.1, "returncode": 0,
            }
            trials = [
                {"trial_id": "reset-first-pass", "action": {"success": False},
                 "reset_attempts": [first_success]},
                {"trial_id": "reset-recovered", "action": {"success": False},
                 "reset_attempts": [timeout, retry_success]},
                {"trial_id": "reset-not-attempted", "action": {"success": False},
                 "reset_attempts": []},
            ]
            trials_path.write_text(
                "\n".join(json.dumps(row) for row in trials) + "\n", encoding="utf-8"
            )
            scores_path.write_text("", encoding="utf-8")

            report = evaluator.aggregate_batch(
                str(trials_path), str(scores_path), str(output_prefix), expected_trials=3
            )

            reset = report["summary"]["batch"]["reset_summary"]
            self.assertEqual(len(report["trials"]), 3)
            self.assertEqual(reset["planned_trial_count"], 3)
            self.assertEqual(reset["available_trial_count"], 3)
            self.assertEqual(reset["attempted_trial_count"], 2)
            self.assertEqual(reset["first_attempt_success_count"], 1)
            self.assertEqual(reset["first_attempt_success_rate"], 0.5)
            self.assertEqual(reset["recovered_on_retry_count"], 1)
            self.assertEqual(reset["failed_after_reset_count"], 0)
            self.assertEqual(reset["total_attempt_count"], 3)
            self.assertEqual(report["trials"][1]["reset_attempts"], [timeout, retry_success])
            self.assertEqual(len(pathlib.Path(str(output_prefix) + ".trials.csv").read_text().splitlines()), 4)

    def test_three_round_identity_joins_scores_and_separates_deadline_and_recovery(self):
        with tempfile.TemporaryDirectory() as directory:
            trials_path = pathlib.Path(directory) / "trials.jsonl"
            scores_path = pathlib.Path(directory) / "pose_errors.jsonl"
            output_prefix = pathlib.Path(directory) / "aggregate"
            trials = []
            scores = []
            for repetition, stamp, elapsed, recoveries in (
                (1, 101, 12.0, 0), (2, 201, 15.001, 1), (3, 301, 15.0, 0),
            ):
                trials.append({
                    "run_id": "fix45-run", "repetition": repetition,
                    "trial_id": "p05_yaw0", "trial_index": 1, "source_trial_index": 5,
                    "action": {"success": True, "timeout": False, "pose_stamp_ns": stamp,
                               "elapsed_sec": elapsed},
                    "status_events": [{"stage": "SUCCEEDED", "elapsed_sec": elapsed,
                                       "raw_frontend": {"recovery_attempts": recoveries}}],
                })
                scores.append({
                    "run_id": "fix45-run", "repetition": repetition,
                    "pose_stamp_ns": stamp, "evaluation_valid": True, "success": True,
                    "position_3d_error_m": 0.01, "orientation_geodesic_error_deg": 0.5,
                })
            trials_path.write_text("\n".join(json.dumps(row) for row in trials) + "\n", encoding="utf-8")
            scores_path.write_text("\n".join(json.dumps(row) for row in scores) + "\n", encoding="utf-8")

            report = evaluator.aggregate_batch(
                str(trials_path), str(scores_path), str(output_prefix), expected_trials=3
            )
            summary = report["summary"]

            self.assertEqual([row["repetition"] for row in report["trials"]], [1, 2, 3])
            self.assertEqual([row["source_trial_index"] for row in report["trials"]], [5, 5, 5])
            self.assertEqual(summary["cohort_identity"]["unique_trial_identity_count"], 3)
            self.assertEqual(summary["batch"]["success_count"], 3)
            deadline = summary["correct_localization_within_15s"]
            self.assertEqual(deadline["confirmed_correct_count"], 2)
            self.assertAlmostEqual(deadline["confirmed_correct_rate_all_cohort"], 2 / 3)
            self.assertAlmostEqual(deadline["confirmed_correct_rate_among_successful_actions"], 2 / 3)
            recovery = summary["algorithm_recovery_outcomes"]
            self.assertEqual(recovery["correct_without_recovery_count"], 2)
            self.assertEqual(recovery["correct_after_one_recovery_count"], 1)
            self.assertEqual(recovery["action_failure_count"], 0)

    def test_tagged_score_from_another_repetition_is_rejected_as_orphan(self):
        with tempfile.TemporaryDirectory() as directory:
            trials_path = pathlib.Path(directory) / "trials.jsonl"
            scores_path = pathlib.Path(directory) / "pose_errors.jsonl"
            trial = {
                "run_id": "fix45-run", "repetition": 2,
                "trial_id": "p05_yaw0", "trial_index": 1, "source_trial_index": 5,
                "action": {"success": True, "timeout": False, "pose_stamp_ns": 101,
                           "elapsed_sec": 10.0},
            }
            stale_score = {
                "run_id": "fix45-run", "repetition": 1,
                "pose_stamp_ns": 101, "evaluation_valid": True, "success": True,
            }
            trials_path.write_text(json.dumps(trial) + "\n", encoding="utf-8")
            scores_path.write_text(json.dumps(stale_score) + "\n", encoding="utf-8")

            with self.assertRaisesRegex(ValueError, "do not match an action identity/stamp"):
                evaluator.aggregate_batch(
                    str(trials_path), str(scores_path), str(pathlib.Path(directory) / "aggregate"),
                    expected_trials=1,
                )

    def test_partial_round_identity_and_duplicate_action_identity_are_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            trials_path = pathlib.Path(directory) / "trials.jsonl"
            scores_path = pathlib.Path(directory) / "pose_errors.jsonl"
            output_prefix = pathlib.Path(directory) / "aggregate"
            scores_path.write_text("", encoding="utf-8")

            trials_path.write_text(json.dumps({
                "run_id": "fix45-run", "trial_id": "p05_yaw0", "action": {"success": False},
            }) + "\n", encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "positive integer repetition"):
                evaluator.aggregate_batch(
                    str(trials_path), str(scores_path), str(output_prefix), expected_trials=1,
                )

            duplicate = {
                "run_id": "fix45-run", "repetition": 1,
                "trial_id": "p05_yaw0", "source_trial_index": 5,
                "action": {"success": False},
            }
            trials_path.write_text(
                json.dumps(duplicate) + "\n" + json.dumps(duplicate) + "\n", encoding="utf-8",
            )
            with self.assertRaisesRegex(ValueError, "duplicate trial identity"):
                evaluator.aggregate_batch(
                    str(trials_path), str(scores_path), str(output_prefix), expected_trials=2,
                )

    def test_missing_poses_remain_null_in_full_45_trial_denominator(self):
        with tempfile.TemporaryDirectory() as directory:
            trials_path = pathlib.Path(directory) / "trials.jsonl"
            scores_path = pathlib.Path(directory) / "pose_errors.jsonl"
            output_prefix = pathlib.Path(directory) / "aggregate"
            trials = []
            for repetition in (1, 2, 3):
                for index in range(15):
                    is_only_scored_trial = repetition == 1 and index == 0
                    trials.append({
                        "run_id": "fix45-run", "repetition": repetition,
                        "trial_id": f"cell-{index}", "trial_index": index,
                        "source_trial_index": index,
                        "action": {
                            "success": is_only_scored_trial, "timeout": False,
                            "pose_stamp_ns": 101 if is_only_scored_trial else None,
                            "elapsed_sec": 14.0 if is_only_scored_trial else 3.0,
                        },
                        "status_events": [{"stage": "SUCCEEDED" if is_only_scored_trial else "FAILED",
                                           "raw_frontend": {"recovery_attempts": 0}}],
                    })
            score = {
                "run_id": "fix45-run", "repetition": 1, "pose_stamp_ns": 101,
                "evaluation_valid": True, "success": True,
                "position_3d_error_m": 0.01, "orientation_geodesic_error_deg": 0.5,
            }
            trials_path.write_text("\n".join(json.dumps(row) for row in trials) + "\n", encoding="utf-8")
            scores_path.write_text(json.dumps(score) + "\n", encoding="utf-8")

            report = evaluator.aggregate_batch(
                str(trials_path), str(scores_path), str(output_prefix), expected_trials=45
            )
            deadline = report["summary"]["correct_localization_within_15s"]

            self.assertEqual(report["summary"]["batch"]["observed_trials"], 45)
            self.assertEqual(report["summary"]["batch"]["failure_count"], 44)
            self.assertEqual(sum(row["pose_stamp_ns"] is None for row in report["trials"]), 44)
            self.assertEqual(deadline["expected_cohort_count"], 45)
            self.assertEqual(deadline["confirmed_correct_count"], 1)
            self.assertAlmostEqual(deadline["confirmed_correct_rate_all_cohort"], 1 / 45)

    def test_legacy_formal_raw120_without_new_fields_still_aggregates_to_58_of_120(self):
        repository_root = SCRIPT_DIR.parents[2]
        formal = repository_root / "tools/global_relocalization/data/session_20261013T160806Z_0f2b982a/formal_raw120_run01"
        trials_path = formal / "trials.jsonl"
        scores_path = formal / "pose_errors.jsonl"
        self.assertTrue(trials_path.is_file())
        self.assertTrue(scores_path.is_file())
        with tempfile.TemporaryDirectory() as directory:
            report = evaluator.aggregate_batch(
                str(trials_path), str(scores_path), str(pathlib.Path(directory) / "legacy"),
                expected_trials=120,
            )

        batch = report["summary"]["batch"]
        self.assertEqual(batch["observed_trials"], 120)
        self.assertEqual(batch["success_count"], 58)
        self.assertEqual(batch["failure_count"], 62)
        self.assertAlmostEqual(batch["success_rate"], 58 / 120)
        self.assertEqual(report["summary"]["accuracy"]["valid_pose_count"], 58)
        self.assertEqual(batch["reset_summary"]["available_trial_count"], 0)
        self.assertIsNone(batch["reset_summary"]["first_attempt_success_rate"])
        recovery = report["summary"]["algorithm_recovery_outcomes"]
        self.assertIsNone(recovery["correct_without_recovery_count"])
        self.assertEqual(recovery["correct_with_unknown_recovery_history_count"], 58)


if __name__ == "__main__":
    unittest.main()
