import hashlib
import json
import tempfile
import unittest
from pathlib import Path

import numpy as np
from PIL import Image
import yaml

from audit_sensor_batch import (archived_input_index, archived_map_view, audit_frozen_freshness,
                               coordinator_only_inputs, frozen_source_path,
                               verify_generated_geometry_and_seed)
from generate_holdout_trials import sample_trials


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


class AuditSensorBatchTests(unittest.TestCase):
    def test_archived_input_is_authoritative_and_runtime_hash_is_separate(self):
        with tempfile.TemporaryDirectory() as temporary:
            batch = Path(temporary) / "batch"
            snapshots = batch / "frozen_inputs"
            snapshots.mkdir(parents=True)
            original = Path(temporary) / "source.py"
            original.write_text("new runtime source")
            snapshot = snapshots / "old_source.py"
            snapshot.write_text("frozen source")
            index = [{"original": str(original), "snapshot": snapshot.name, "sha256": sha(snapshot)}]
            (snapshots / "index.json").write_text(json.dumps(index))
            archives, problems = archived_input_index(batch)
            self.assertEqual(problems, [])
            check_problems = []
            resolved = frozen_source_path(original, sha(snapshot), archives, check_problems)
            self.assertEqual(resolved, snapshot)
            self.assertEqual(check_problems, [])
            self.assertNotEqual(sha(original), sha(snapshot))

    def test_archived_map_resources_are_resolved_outside_temporary_tree(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            source_dir = root / "maps"
            source_dir.mkdir()
            (source_dir / "occ.png").write_bytes(b"occ")
            (source_dir / "elev.png").write_bytes(b"elev")
            (source_dir / "world.sdf").write_bytes(b"world")
            (source_dir / "robot.urdf").write_bytes(b"urdf")
            (source_dir / "models").mkdir()
            original = source_dir / "map.yaml"
            original.write_text("runtime config")
            snapshot = root / "frozen_inputs" / "map.yaml"
            snapshot.parent.mkdir()
            snapshot.write_text(yaml.safe_dump({
                "image": "../maps/occ.png", "resolution": .05, "origin": [-1, -1, 0],
                "ground_elevation": {"grid": {"image": "../maps/elev.png"}},
                "collision_support": {"world_path": "../maps/world.sdf",
                                      "robot_urdf_path": "../maps/robot.urdf",
                                      "model_roots": ["../maps/models"]},
            }))
            with archived_map_view(original, snapshot) as view:
                data = yaml.safe_load(view.read_text())
                paths = [data["image"], data["ground_elevation"]["grid"]["image"],
                         data["collision_support"]["world_path"],
                         data["collision_support"]["robot_urdf_path"],
                         data["collision_support"]["model_roots"][0]]
                self.assertTrue(all(Path(p).is_absolute() for p in paths))
                self.assertTrue(all(Path(p).exists() for p in paths))

    def test_legacy_is_not_acceptance_eligible(self):
        freshness, problems = audit_frozen_freshness({"trials": []})
        self.assertEqual(freshness["status"], "legacy_missing_holdout_metadata")
        self.assertFalse(freshness["acceptance_eligible"])
        self.assertEqual(problems, [])

    def test_frozen_freshness_checks_only_frozen_sources(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            history = root / "trials_old.json"
            history.write_text(json.dumps({"trials": [{"requested_pose": {"x": -1.0, "y": 0.0}}]}))
            map_yaml = root / "map.yaml"
            occupancy = root / "occ.png"
            elevation = root / "elev.png"
            map_yaml.write_bytes(b"map config")
            occupancy.write_bytes(b"occupancy")
            elevation.write_bytes(b"elevation")
            geometry = root / "geometry" / "world.sdf"
            geometry.parent.mkdir()
            geometry.write_bytes(b"collision geometry")
            explicit_trials = root / "input_trials.json"
            explicit_trials.write_text("{}")
            content = hashlib.sha256()
            for path in (map_yaml, occupancy, elevation, geometry):
                content.update(path.relative_to(root).as_posix().encode())
                content.update(b"\0")
                content.update(path.read_bytes())
            source = {"path": str(history), "sha256": sha(history), "requested_pose_count": 1}
            frozen = {
                "schema_version": 2,
                "trials": [{"trial_id": "new", "requested_pose": {"x": -2.0, "y": 0.0, "z": .2, "yaw": 0.0}}],
                "trial_source": {"kind": "explicit", "path": str(explicit_trials),
                                 "sha256": sha(explicit_trials)},
                "generation_metadata": {
                    "seed": 7, "sampling_map": str(map_yaml), "map_sha256": content.hexdigest(),
                    "map_source": {"map_yaml_sha256": sha(map_yaml), "occupancy_image": str(occupancy),
                                   "occupancy_sha256": sha(occupancy), "elevation_image": str(elevation),
                                   "elevation_sha256": sha(elevation),
                                   "collision_support": {"source_sha256": {str(geometry.resolve()): sha(geometry)}},
                                   "map_content_sha256": content.hexdigest()},
                    "sampling_limits": {"minimum_clearance_m": .5},
                },
                "historical_validation": {"history_sources": [source], "history_source_count": 1,
                                          "history_pose_count": 1, "minimum_historical_distance_m": .25,
                                          "minimum_batch_distance_m": .25},
            }
            freshness, problems = audit_frozen_freshness(frozen)
            self.assertTrue(freshness["acceptance_eligible"], problems)
            self.assertAlmostEqual(freshness["minimum_historical_distance_m"], 1.0)

    def test_coordinator_inputs_are_not_expected_from_child(self):
        source = Path("/tmp/input_trials.json")
        yaml_path = Path("/tmp/map.yaml")
        frozen = {"trial_source": {"kind": "explicit", "path": str(source)},
                  "generation_metadata": {"sampling_map": str(yaml_path),
                                          "map_source": {"occupancy_image": "/tmp/occ.png",
                                                          "collision_support": {"source_sha256": {
                                                              "/tmp/model/model.sdf": "sha"}}}}}
        provenance = {"/repo/tools/global_relocalization/sensor_map_batch.py": "a",
                      "/repo/tools/global_relocalization/actual_collision_support.py": "b"}
        paths = coordinator_only_inputs(frozen, provenance)
        self.assertIn(source.resolve(), paths)
        self.assertIn(yaml_path.resolve(), paths)
        self.assertIn(Path("/tmp/occ.png").resolve(), paths)
        self.assertIn(Path("/tmp/model/model.sdf").resolve(), paths)
        self.assertIn(Path("/repo/tools/global_relocalization/actual_collision_support.py").resolve(), paths)

    def test_seed_reproduction_and_unknown_edge_clearance(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            occupancy = np.full((80, 80), 255, dtype=np.uint8)
            elevation = np.full((80, 80), 1000, dtype=np.uint16)
            elevation[:, 39:41] = 65535
            Image.fromarray(occupancy).save(root / "occ.png")
            Image.fromarray(elevation).save(root / "elev.png")
            map_yaml = root / "map.yaml"
            map_yaml.write_text(yaml.safe_dump({
                "image": "occ.png", "resolution": .05, "origin": [-2.0, -2.0, 0.0],
                "ground_elevation": {"grid": {"image": "elev.png", "scale": .001,
                                               "offset": -1.0, "no_data": 65535}},
            }))
            history = root / "history"
            history.mkdir()
            generated = sample_trials(count=3, seed=12345, map_yaml=map_yaml,
                                      history_roots=[history], min_distance=.25)
            reproduced = verify_generated_geometry_and_seed(
                generated["trials"], generated, np.empty((0, 2)))
            self.assertEqual(reproduced, 3)

            separated = json.loads(json.dumps(generated))
            separated["sampling_limits"]["spawn_separation_margin_m"] = .002
            for trial in separated["trials"]:
                trial["requested_pose"]["z"] += .002
            self.assertEqual(verify_generated_geometry_and_seed(
                separated["trials"], separated, np.empty((0, 2))), 3)

            # Column 38 is free and known, but sits directly beside unknown
            # elevation columns 39–40, so it must fail the 0.5 m EDT clearance.
            too_close = [{"trial_id": "edge", "trial_index": 0,
                          "requested_pose": {"x": -.075, "y": .025, "z": .08, "yaw": 0.0}}]
            with self.assertRaisesRegex(ValueError, "clearance-qualified"):
                verify_generated_geometry_and_seed(too_close, generated, np.empty((0, 2)))


if __name__ == "__main__":
    unittest.main()
