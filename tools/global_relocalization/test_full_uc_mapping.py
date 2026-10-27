"""Checks that full-field evaluation cannot silently reduce to a left-field test."""
import json
from pathlib import Path

import numpy as np
from PIL import Image
import pytest
import yaml

from full_uc_mapping import HERE, ROOT, make_route
from report_full_uc_mapping import occupancy_metrics
from sensor_mapping import Route

MAP = ROOT / "src/pb2025_nav_bringup/map/simulation/RMUC2026.yaml"


def test_full_route_crosses_both_halves_and_stays_on_inflated_surveyed_ground():
    planner, route = make_route(MAP, HERE / "config/survey_full_uc.json", .55)
    points = np.asarray(route["waypoints"])
    assert points[:, 0].min() < -11 and points[:, 0].max() > 11
    assert len(route["stations"]) == 14
    assert np.linalg.norm(points[-1] - [-11.7, 3.1]) < .08
    assert max(route["ground_heights_m"]) > .2  # Actually crosses raised ground.
    for a, b in zip([[-11.7, 3.1]] + route["waypoints"], route["waypoints"]):
        for point in np.linspace(a, b, max(2, int(np.linalg.norm(np.subtract(a, b)) / .01))):
            assert planner.safe[planner.cell(point)], (a, b, point)
    assert route["pose_resets"] == 0


def test_legacy_route_remains_left_and_flat():
    planner = Route(MAP, [-11.7, 3.1], .55)
    positions = np.asarray([planner.xy(cell) for cell in planner.cells])
    assert positions[:, 0].max() < 0
    assert np.abs(planner.ground[planner.safe]).max() <= .03


def write_grid(path, image, origin):
    Image.fromarray(image.astype(np.uint8)).save(path / "grid.pgm")
    config = {"image": "grid.pgm", "resolution": 1., "origin": origin,
              "negate": 0, "occupied_thresh": .65, "free_thresh": .35}
    return config


def test_occupancy_uses_fixed_world_gauge_and_excludes_unknown_cells(tmp_path):
    refdir, run = tmp_path / "reference", tmp_path / "run"
    refdir.mkdir(); run.mkdir()
    image = np.full((4, 4), 254, dtype=np.uint8)
    image[1, 1] = 0
    image[0, 0] = 128
    ref = write_grid(refdir, image, [10., 0., 0.])
    cfg = write_grid(run, image, [0., 0., 0.])
    reference_yaml = refdir / "reference.yaml"
    reference_yaml.write_text(yaml.safe_dump(ref))
    (run / "occupancy.yaml").write_text(yaml.safe_dump(cfg))
    transform = np.eye(4)
    transform[0, 3] = 10.
    good = occupancy_metrics(run, reference_yaml, transform)
    assert good["comparison_cells"] == 15
    assert good["occupied_iou"] == pytest.approx(1.)
    assert good["known_reference_area_observed_fraction"] == pytest.approx(1.)
    wrong = occupancy_metrics(run, reference_yaml, np.eye(4))
    assert wrong["comparison_cells"] == 0
    assert wrong["occupied_iou"] is None  # Never fit away a coordinate error.


def test_missing_occupancy_is_explicit(tmp_path):
    result = occupancy_metrics(tmp_path, MAP, np.eye(4))
    assert result["available"] is False
    assert "not saved" in result["reason"]
