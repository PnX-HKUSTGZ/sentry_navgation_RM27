import math
import json
import sys
from pathlib import Path

import pytest

import sensor_map_batch
from sensor_map_batch import summarize, statistics


@pytest.mark.parametrize("marker", ["MAP_QUALITY_REJECTED.json", "DIAGNOSTIC_MAP_ONLY.json"])
def test_non_adopted_map_rejected_before_reserving_new_trials(tmp_path, monkeypatch, marker):
    map_dir = tmp_path / "map"
    map_dir.mkdir()
    (map_dir / "manifest.json").write_text(json.dumps({
        "capture_complete": True, "ground_truth_used_for_scan_fusion": False,
    }))
    (map_dir / marker).write_text(json.dumps({"formal_batch_allowed": False}))
    output = tmp_path / "batch"
    monkeypatch.setattr(sys, "argv", ["sensor_map_batch", "--output", str(output),
                                     "--map-dir", str(map_dir), "--node", "/unused/node"])
    def no_reservation(*args, **kwargs):
        raise AssertionError("a rejected map must not reserve new validation points")
    monkeypatch.setattr(sensor_map_batch, "prepare_frozen_trials", no_reservation)
    with pytest.raises(ValueError, match="use offline diagnostics"):
        sensor_map_batch.main()
    assert not (output / "trials.json").exists()


def row(name, error=None, angle=0, accepted=True):
    value = {"trial_id": name, "requested_pose": {"yaw": 0},
             "action": {"success": accepted, "reason": "test failure"},
             "terminal_status": {"competition": {"trigger_to_pose_sec": 0.5}}}
    if error is not None:
        value["score"] = {"valid": True, "strict_correct": error <= .1 and angle <= 2,
                          "translation_3d_m": error, "translation_xy_m": error,
                          "orientation_so3_deg": angle, "yaw_error_deg": -angle}
    return value


def test_statistics_rmse_is_not_mean_and_empty_is_not_zero():
    assert statistics([])["mean"] is None
    assert math.isclose(statistics([0, 2])["rmse"], math.sqrt(2))


def test_failures_stay_in_success_denominator_and_false_accepts_in_error_population():
    report = summarize([row("good", .01), row("false_accept", 1, 5),
                        row("rejected", accepted=False)], 4)
    assert report["planned"] == 4
    assert report["completed"] == 3
    assert report["accepted"] == 2
    assert report["valid_scored"] == 2
    assert report["strict_correct"] == 1
    assert report["strict_correct_rate_all_completed"] == 1 / 3
    assert report["translation_3d_m"]["max"] == 1
    assert report["orientation_so3_deg"]["mean"] == 2.5
    assert sum(report["failure_reasons"].values()) == 2


def test_missing_gt_is_never_scored_as_zero_error_or_correct():
    report = summarize([row("missing_gt")], 1)
    assert report["accepted"] == 1
    assert report["valid_scored"] == report["strict_correct"] == 0
    assert report["translation_3d_m"]["count"] == 0


def test_sensor_map_profile_retains_strict_and_requires_latency_for_joint_success():
    slow = row("slow", .14, 2.5)
    slow["terminal_status"]["competition"]["trigger_to_pose_sec"] = 1.01
    missing_time = row("missing_time", .01)
    missing_time["terminal_status"] = None
    report = summarize([row("strict", .01), row("relaxed", .14, 2.5), slow,
                        missing_time, row("rejected", accepted=False)], 5, "sensor_map")
    assert report["strict_correct"] == 2
    assert report["pose_correct"] == 4
    assert report["pose_correct_and_fast"] == 2
    assert report["pose_correct_and_fast_rate_all_completed"] == 2 / 5
    assert sum(report["failure_reasons"].values()) == 3
    assert report["failure_reasons"]["accepted pose missing valid latency or exceeds 1 s"] == 2
    assert not report["acceptance_passed"]
    assert not summarize([], 0)["acceptance_passed"]
    assert not summarize([row("good", .01)], 2)["acceptance_passed"]
    assert summarize([row("good", .01)], 1)["acceptance_passed"]


def test_explicit_null_score_is_a_failure_without_artificial_error():
    missing = row("missing", accepted=True)
    missing["score"] = None
    report = summarize([missing], 1, "sensor_map")
    assert not report["acceptance_passed"]
    assert report["translation_3d_m"]["count"] == 0
    assert report["failure_reasons"] == {"accepted output missing valid GT score": 1}


def test_frozen_input_snapshot_survives_source_edit_and_rejects_tampering(tmp_path):
    import hashlib
    source = tmp_path / "core.so"
    source.write_bytes(b"frozen algorithm")
    digest = hashlib.sha256(source.read_bytes()).hexdigest()
    out = tmp_path / "batch"
    out.mkdir()
    sensor_map_batch.archive_frozen_inputs(out, {str(source): digest})
    index = json.loads((out / "frozen_inputs/index.json").read_text())
    snapshot = out / "frozen_inputs" / index[0]["snapshot"]
    source.write_bytes(b"next algorithm")
    assert snapshot.read_bytes() == b"frozen algorithm"
    with pytest.raises(ValueError, match="input changed"):
        sensor_map_batch.archive_frozen_inputs(out, {str(source): digest})
    source.write_bytes(b"frozen algorithm")
    snapshot.write_bytes(b"tampered archive")
    with pytest.raises(ValueError, match="snapshot mismatch"):
        sensor_map_batch.archive_frozen_inputs(out, {str(source): digest})


def trial(name, x, y, yaw=0.0):
    return {"trial_id": name, "requested_pose": {"x": x, "y": y, "z": 0.089, "yaw": yaw}}


def write_trial_file(path, trials, **metadata):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps({**metadata, "trials": trials}, indent=2))
    return path


def test_explicit_trials_exclude_only_the_input_file_and_check_all_other_history(tmp_path):
    history = tmp_path / "history"
    write_trial_file(history / "trials_old.json", [trial("old", -10.0, -10.0)])
    (history / "batch01" / "p00" ).mkdir(parents=True)
    (history / "batch01" / "p00" / "result.json").write_text(json.dumps({
        "requested_pose": {"x": -5.0, "y": -4.9, "z": 0.089, "yaw": 0.0},
        "action": {"success": False},
    }))
    explicit = write_trial_file(history / "trials_holdout.json", [trial("new", -5.0, -5.0)])
    with pytest.raises(ValueError, match="historical data"):
        sensor_map_batch.prepare_frozen_trials(
            tmp_path / "rejected", trials_path=explicit, history_roots=[history])

    # Excluding the source matrix does not exclude prior executed results.
    valid = write_trial_file(history / "trials_valid.json", [trial("new0", -3.0, -3.0),
                                                                trial("new1", -3.0, -2.0)])
    frozen = sensor_map_batch.prepare_frozen_trials(
        tmp_path / "accepted", trials_path=valid, history_roots=[history])
    assert frozen["trial_source"]["kind"] == "explicit"
    assert frozen["trial_source"]["path"] == str(valid.resolve())
    assert frozen["trial_source"]["sha256"]
    assert frozen["historical_validation"]["excluded_input_file"] == str(valid.resolve())
    assert frozen["historical_validation"]["history_pose_count"] == 3
    assert frozen["historical_validation"]["history_source_count"] == 3


def test_explicit_batch_duplicate_positions_are_rejected(tmp_path):
    path = write_trial_file(tmp_path / "trials.json", [trial("a", -4.0, 1.0), trial("b", -4.1, 1.0)])
    with pytest.raises(ValueError, match="separated by at least 0.25 m"):
        sensor_map_batch.prepare_frozen_trials(tmp_path / "out", trials_path=path, history_roots=[])


def test_trial_spacing_boundary_matches_the_holdout_sampler():
    sensor_map_batch.validate_trial_separation(
        [trial("a", -2.0, 0.0), trial("b", -2.0, 0.25)], [(-2.0, -0.25)])
    with pytest.raises(ValueError, match="historical data"):
        sensor_map_batch.validate_trial_separation([trial("a", -2.0, 0.0)], [(-2.0, -0.249)])


def test_default_trials_are_generated_once_with_random_seed_and_resume_uses_frozen(tmp_path, monkeypatch):
    calls = []

    def generated(*, count, seed, map_yaml, history_roots, min_distance):
        calls.append({"count": count, "seed": seed, "map_yaml": map_yaml,
                      "history_roots": history_roots, "min_distance": min_distance})
        return {
            "seed": seed, "sampling_map": str(map_yaml), "map_sha256": "map-hash",
            "history_sources": [{"path": "prior.json", "sha256": "history-hash"}],
            "trials": [trial(f"holdout_{i:03d}", -100.0 + i * 0.5, -5.0) for i in range(count)],
        }

    monkeypatch.setattr(sensor_map_batch, "sample_trials", generated)
    out = tmp_path / "generated"
    frozen = sensor_map_batch.prepare_frozen_trials(
        out, limit=12, seed=None, history_roots=[])
    assert len(calls) == 1
    assert calls[0]["count"] == 12
    assert 0 <= frozen["trial_source"]["seed"] < 2**64
    assert frozen["generation_metadata"]["map_sha256"] == "map-hash"
    assert frozen["generation_metadata"]["history_sources"][0]["sha256"] == "history-hash"
    assert frozen["planned_count"] == 12

    default = sensor_map_batch.prepare_frozen_trials(
        tmp_path / "default_count", seed=2026102602, history_roots=[])
    assert calls[-1]["count"] == 120
    assert default["planned_count"] == 120

    def no_resample(**kwargs):
        raise AssertionError("resume must not resample")

    monkeypatch.setattr(sensor_map_batch, "sample_trials", no_resample)
    resumed = sensor_map_batch.prepare_frozen_trials(
        out, resume=True, history_roots=[])
    assert resumed == frozen
    with pytest.raises(ValueError, match="profile differs"):
        sensor_map_batch.prepare_frozen_trials(
            out, resume=True, evaluation_profile="sensor_map", history_roots=[])
    with pytest.raises(ValueError, match="limit differs"):
        sensor_map_batch.prepare_frozen_trials(
            out, resume=True, limit=13, history_roots=[])
    with pytest.raises(ValueError, match="seed differs"):
        sensor_map_batch.prepare_frozen_trials(
            out, resume=True, seed=(frozen["trial_source"]["seed"] + 1) % (2**64), history_roots=[])


def test_resume_requires_original_explicit_source_to_remain_unchanged(tmp_path):
    source = write_trial_file(tmp_path / "trials_source.json", [trial("a", -5.0, -5.0)])
    out = tmp_path / "out"
    original = sensor_map_batch.prepare_frozen_trials(out, trials_path=source, history_roots=[])
    resumed = sensor_map_batch.prepare_frozen_trials(out, resume=True, history_roots=[])
    assert resumed == original
    source.write_text(source.read_text() + " ")
    with pytest.raises(ValueError, match="source is missing or changed"):
        sensor_map_batch.prepare_frozen_trials(out, resume=True, history_roots=[])


def test_regression_bypasses_history_but_never_passes_acceptance_or_mixes_profile(tmp_path):
    history = tmp_path / "history"
    write_trial_file(history / "trials_old.json", [trial("old", -5.0, -5.0)])
    source = write_trial_file(history / "trials_regression.json", [trial("repeat", -5.0, -5.0)])
    frozen = sensor_map_batch.prepare_frozen_trials(
        tmp_path / "out", trials_path=source, regression_mode=True, history_roots=[history])
    assert frozen["regression_mode"] is True
    assert frozen["historical_validation"]["regression_bypassed_historical_exclusion"] is True
    assert not summarize([row("good", .01)], 1, regression_mode=True)["acceptance_passed"]
    with pytest.raises(ValueError, match="cannot be combined"):
        sensor_map_batch.validate_modes("ideal", True)


def test_trial_validator_rejects_nonnegative_x_and_invalid_yaw(tmp_path):
    with pytest.raises(ValueError, match="negative-x"):
        sensor_map_batch.validate_trial_separation([trial("bad", 0.0, 1.0)], [], allow_historical=True)
    with pytest.raises(ValueError, match="yaw"):
        sensor_map_batch.validate_trial_separation([trial("bad", -1.0, 1.0, math.nan)], [], allow_historical=True)
