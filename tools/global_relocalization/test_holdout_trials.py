import json
import math
from pathlib import Path

import numpy as np
from PIL import Image
import pytest
from scipy.ndimage import distance_transform_edt

from generate_holdout_trials import sample_trials, write_new_file


def make_map(root: Path, *, rows: int = 50, cols: int = 60, resolution: float = 0.1) -> Path:
    root.mkdir(parents=True, exist_ok=True)
    occupancy = np.full((rows, cols), 255, dtype=np.uint8)
    occupancy[[0, -1], :] = 0
    occupancy[:, [0, -1]] = 0
    # A small obstacle and unknown/elevation-no-data patch exercise both filters.
    occupancy[20:24, 28:33] = 0
    height = np.empty((rows, cols), dtype=np.uint16)
    height[:] = 1000 + np.arange(rows, dtype=np.uint16)[:, None] * 20
    height[8:13, 8:13] = 65535
    Image.fromarray(occupancy).save(root / "occupancy.png")
    Image.fromarray(height).save(root / "elevation.png")
    map_yaml = root / "field.yaml"
    map_yaml.write_text(
        "image: occupancy.png\n"
        "mode: trinary\n"
        f"resolution: {resolution}\n"
        "origin: [-3.0, -2.0, 0.0]\n"
        "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.35\n"
        "ground_elevation:\n"
        "  grid:\n"
        "    image: elevation.png\n"
        "    scale: 0.001\n"
        "    offset: -1.0\n"
        "    no_data: 65535\n"
    )
    return map_yaml


def write_history(path: Path, poses: list[tuple[float, float]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps({
        "trials": [
            {"requested_pose": {"x": x, "y": y, "z": 0.089, "yaw": 0.0}}
            for x, y in poses
        ]
    }))


def run_sample(tmp_path: Path, *, seed: int = 17, count: int = 20, min_distance: float = 0.25):
    map_yaml = make_map(tmp_path / "map")
    history = tmp_path / "history"
    history.mkdir(exist_ok=True)
    return sample_trials(count=count, seed=seed, map_yaml=map_yaml,
                         history_roots=[history], min_distance=min_distance)


def test_holdout_is_deterministic_and_respects_map_history_and_batch_limits(tmp_path):
    map_yaml = make_map(tmp_path / "map")
    history = tmp_path / "history"
    # Exclude a prior trial and a completed result. capture.json must stay unread.
    write_history(history / "trials_previous.json", [(-2.0, 0.0)])
    result_dir = history / "old" / "one"
    result_dir.mkdir(parents=True)
    (result_dir / "result.json").write_text(json.dumps({
        "requested_pose": {"x": -2.0, "y": 1.0, "z": 0.089, "yaw": 1.2},
        "action": {"success": False},
    }))
    (history / "capture.json").write_text("this file must not be read")

    a = sample_trials(count=30, seed=101, map_yaml=map_yaml,
                      history_roots=[history], min_distance=0.25)
    b = sample_trials(count=30, seed=101, map_yaml=map_yaml,
                      history_roots=[history], min_distance=0.25)
    assert a == b
    assert len(a["trials"]) == 30
    assert len({trial["trial_id"] for trial in a["trials"]}) == 30
    assert a["history_pose_count"] == 2
    assert a["sampling_limits"]["algorithm_outcomes_read"] is False
    assert a["map_sha256"] == a["map_source"]["map_content_sha256"]
    assert a["map_sha256"] != a["map_source"]["map_yaml_sha256"]
    assert len(a["history_sources"]) == 2

    poses = [trial["requested_pose"] for trial in a["trials"]]
    xy = np.asarray([[pose["x"], pose["y"]] for pose in poses])
    assert np.all(xy[:, 0] < 0.0)
    assert np.all((xy[:, 0] >= -3.0) & (xy[:, 0] < 0.0))
    assert np.all((xy[:, 1] >= -2.0) & (xy[:, 1] < 3.0))
    pairwise = np.linalg.norm(xy[:, None, :] - xy[None, :, :], axis=2)
    pairwise += np.eye(len(xy)) * 100.0
    assert pairwise.min() + 1e-9 >= 0.25
    historical = np.asarray([[-2.0, 0.0], [-2.0, 1.0]])
    assert np.all(np.linalg.norm(xy[:, None, :] - historical[None, :, :], axis=2) + 1e-9 >= 0.25)
    assert all(-math.pi <= pose["yaw"] < math.pi for pose in poses)
    # The local elevation map slopes by row; spawn z is sampled ground height + 80 mm.
    expected_ground = []
    for pose in poses:
        row = int((pose["y"] + 2.0) / 0.1)
        image_row = 49 - row
        expected_ground.append((1000 + image_row * 20) * 0.001 - 1.0 + 0.08)
    np.testing.assert_allclose([p["z"] for p in poses], expected_ground)


def test_unknown_elevation_and_nonnegative_x_are_excluded(tmp_path):
    map_yaml = make_map(tmp_path / "map")
    result = sample_trials(count=20, seed=23, map_yaml=map_yaml,
                           history_roots=[], min_distance=0.25)
    # The image is flipped into map coordinates, so source rows 8:13 become
    # map rows 37:42; no pose can come from that no-data patch.
    occupancy = np.asarray(Image.open(tmp_path / "map" / "occupancy.png"))[::-1]
    elevation = np.asarray(Image.open(tmp_path / "map" / "elevation.png"))[::-1]
    free_and_known = (occupancy > 250) & (elevation != 65535)
    free_and_known[[0, -1], :] = False
    free_and_known[:, [0, -1]] = False
    clearance_cells = distance_transform_edt(free_and_known)
    for trial in result["trials"]:
        pose = trial["requested_pose"]
        col = int((pose["x"] + 3.0) / 0.1)
        row = int((pose["y"] + 2.0) / 0.1)
        assert pose["x"] < 0
        assert not (37 <= row < 42 and 8 <= col < 13)
        # Unknown elevation edges count as blocked for the 0.5 m clearance.
        assert clearance_cells[row, col] * 0.1 >= 0.5


def test_collision_support_controls_height_and_is_part_of_frozen_map_hash(tmp_path, monkeypatch):
    import actual_collision_support
    from generate_holdout_trials import sha256_file
    map_yaml = make_map(tmp_path / "map")
    world = map_yaml.parent / "scene.world"
    robot = map_yaml.parent / "robot.urdf"
    world.write_text("scene collision v1")
    robot.write_text("robot geometry")
    with map_yaml.open("a") as stream:
        stream.write("collision_support:\n  world_path: scene.world\n"
                     "  robot_urdf_path: robot.urdf\n  separation_margin: 0.002\n")

    def support(points, **options):
        assert options["world_path"] == world.resolve()
        assert options["robot_urdf_path"] == robot.resolve()
        # Half of the occupancy-free cells are physically unsupported.
        mask = points[:, 1] > 0
        heights = np.full(len(points), 0.2)
        return mask, heights, {
            "robot_envelope": {"base_height_above_contact": 0.08},
            "source_hashes": {str(p.resolve()): sha256_file(p) for p in (world, robot)},
        }

    monkeypatch.setattr(actual_collision_support, "filter_supported_positions", support)
    first = sample_trials(count=5, seed=9, map_yaml=map_yaml,
                          history_roots=[], min_distance=.25)
    assert first["sampling_limits"]["actual_collision_support_checked"]
    assert all(t["requested_pose"]["y"] > 0 for t in first["trials"])
    assert all(abs(t["requested_pose"]["z"] - .282) < 1e-12 for t in first["trials"])
    world.write_text("scene collision v2")
    second = sample_trials(count=5, seed=9, map_yaml=map_yaml,
                           history_roots=[], min_distance=.25)
    assert first["map_sha256"] != second["map_sha256"]


def test_insufficient_space_fails_without_output_or_relaxing_spacing(tmp_path):
    map_yaml = make_map(tmp_path / "map")
    with pytest.raises(ValueError, match="cannot produce 120 trials|spacing was not reduced"):
        sample_trials(count=120, seed=1, map_yaml=map_yaml,
                      history_roots=[], min_distance=1.0)
    output = tmp_path / "should_not_exist.json"
    assert not output.exists()


def test_output_structure_and_no_overwrite(tmp_path):
    result = run_sample(tmp_path, count=5)
    output = tmp_path / "nested" / "trials_holdout.json"
    write_new_file(output, result)
    saved = json.loads(output.read_text())
    assert saved == result
    with pytest.raises(FileExistsError, match="refusing to overwrite"):
        write_new_file(output, result)
    assert json.loads(output.read_text()) == result
    with pytest.raises(ValueError, match="future batches discover"):
        write_new_file(tmp_path / "undiscoverable.json", result)


def test_too_few_separated_cells_fails_explicitly(tmp_path):
    map_yaml = make_map(tmp_path / "map")
    with pytest.raises(ValueError, match="spacing was not reduced"):
        sample_trials(count=20, seed=4, map_yaml=map_yaml,
                      history_roots=[], min_distance=1.0)


def test_minimum_distance_cannot_be_lowered_below_history_exclusion_floor(tmp_path):
    map_yaml = make_map(tmp_path / "map")
    with pytest.raises(ValueError, match="at least 0.25 m"):
        sample_trials(count=1, seed=1, map_yaml=map_yaml,
                      history_roots=[], min_distance=0.24)


@pytest.mark.parametrize("count,seed", [(1.5, 1), (1, 2.5), (1, -1), (1, True)])
def test_count_and_seed_are_integer_and_seed_is_unsigned_64_bit(tmp_path, count, seed):
    map_yaml = make_map(tmp_path / "map")
    with pytest.raises(ValueError, match="count must|seed must"):
        sample_trials(count=count, seed=seed, map_yaml=map_yaml,
                      history_roots=[], min_distance=0.25)
