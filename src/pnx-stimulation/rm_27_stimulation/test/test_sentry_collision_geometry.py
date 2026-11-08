"""覆盖完整 CAD 表面，同时拒绝整组凸包填空隙及运行时重新合包。"""

import contextlib
import importlib.util
import io
import json
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation
import xacro
import yaml

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    'sentry_collision_geometry', ROOT / 'tools/generate_sentry_collision_geometry.py')
GEOMETRY = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(GEOMETRY)


def inside(points, hulls, tolerance=1e-7):
    return np.any([np.max(points @ hull.equations[:, :3].T + hull.equations[:, 3], axis=1)
                   <= tolerance for hull in hulls], axis=0)


class SentryCollisionGeometry(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.raw, _, _ = GEOMETRY.source_groups()
        with contextlib.redirect_stdout(io.StringIO()):
            cls.report = GEOMETRY.generate(check=True)

    def test_actual_urdf_contains_every_visual_vertex(self):
        for scale in (1.0, .8, 1.2):
            robot = ET.fromstring(xacro.process_file(
                str(ROOT / 'urdf/simulation_waking_robot.xacro'),
                mappings={'cad_visual_scale': str(scale)}).toxml())
            for link, groups in {'base_link': ('chassis_body', 'chassis_armor'),
                                 'gimbal_yaw': ('yaw_body', 'yaw_sensors'),
                                 'gimbal_roll': ('roll_body',)}.items():
                for name in groups:
                    self.check_containment(robot.find(f"link[@name='{link}']"), name,
                                           self.raw[name] * scale)
            for index, (sx, sy) in enumerate(GEOMETRY.WHEELS, 1):
                name = f'wheel_{index}'
                origin = next(
                    joint.find('origin') for joint in robot.findall('joint')
                    if joint.find('child').get('link') == name)
                center = np.fromstring(origin.get('xyz'), sep=' ')
                rotation = Rotation.from_euler('xyz', np.fromstring(origin.get('rpy'), sep=' ')).as_matrix()
                raw = self.raw['wheels']
                points = raw[(raw[:, 0] * sx > 0) & (raw[:, 1] * sy > 0)]
                self.check_containment(robot.find(f"link[@name='{name}']"), name,
                                       (points * scale - center) @ rotation)
            self.assertEqual(len(robot.findall('.//link/collision')), self.report['collision_count'])

    def check_containment(self, link, group, points):
        collisions = {node.get('name'): node for node in link.findall('collision')}
        hulls = []
        for entry in self.report['parts'][group]:
            stem = Path(entry['file']).stem
            collision = collisions[stem + '_collision']
            mesh = collision.find('geometry/mesh')
            self.assertEqual(mesh.get('filename'),
                             f'package://rm_27_stimulation/meshes/rm27_sentry_collision/{entry["file"]}')
            vertices = GEOMETRY.vertices(GEOMETRY.OUTPUT / entry['file'])
            vertices *= np.fromstring(mesh.get('scale'), sep=' ')
            vertices += np.fromstring(collision.find('origin').get('xyz'), sep=' ')
            hulls.append(ConvexHull(vertices))
        # 分件并集；不能再用整个组的大凸包来验证，否则会重新掩盖过包。
        self.assertTrue(np.all(inside(points, hulls, -.00039)), group)

    def test_navigation_footprint_covers_complete_articulation(self):
        config = ROOT.parents[1] / 'pnx-launch/launch_bringup/config/simulation/minco_params.yaml'
        params = yaml.safe_load(config.read_text())
        safety = params['planner_server']['ros__parameters']['MincoPlanner']['safety']
        polygon = np.array(safety['footprint_points']).reshape(-1, 2)
        for name in ('local_costmap', 'global_costmap'):
            value = params[name][name]['ros__parameters']['footprint']
            np.testing.assert_array_equal(np.array(json.loads(value)), polygon)
        hull = ConvexHull(polygon)
        self.assertLessEqual(GEOMETRY.max_plane_violation(np.array(self.report['footprint_points']), hull), 2e-8)
        self.assertTrue(np.all(-hull.equations[:, 2] >= self.report['moving_swept_radius_m'] + .00199))
        self.assertGreaterEqual(safety['footprint_length'], np.ptp(polygon[:, 0]))
        self.assertGreaterEqual(safety['footprint_width'], np.ptp(polygon[:, 1]))
        self.assertLess(safety['footprint_length'], .636)

    def test_known_chassis_air_gap_is_not_filled(self):
        # 原始 CAD 在 z=80 mm 的此处没有车体，旧整组凸包会填满。
        point = np.array([[.15, .10, .080]])
        old_hull = ConvexHull(self.raw['chassis_body'])
        self.assertTrue(inside(point, [old_hull])[0])
        hulls = [ConvexHull(GEOMETRY.vertices(GEOMETRY.OUTPUT / entry['file']))
                 for entry in self.report['parts']['chassis_body']]
        self.assertFalse(inside(point, hulls)[0])
        self.assertLessEqual(self.report['collision_count'], GEOMETRY.MAX_COLLISIONS)

    def test_spawn_explicitly_uses_separate_exact_convex_meshes(self):
        spec = importlib.util.spec_from_file_location('spawn_robot', ROOT / 'launch/spawn_robot.launch.py')
        spawn = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(spawn)
        urdf = xacro.process_file(str(ROOT / 'urdf/simulation_waking_robot.xacro')).toxml()
        sdf = ET.fromstring(spawn._convex_collision_description(urdf))
        meshes = sdf.findall('.//collision/geometry/mesh')
        self.assertEqual(len(meshes), self.report['collision_count'])
        self.assertEqual(len({m.findtext('uri') for m in meshes}), len(meshes))
        self.assertTrue(all(mesh.get('optimization') == 'convex_hull' for mesh in meshes))
        plugins = sdf.findall('.//plugin')
        self.assertEqual(sum(p.get('name') == 'rm_27_stimulation::CadCollisionCacheSystem'
                             for p in plugins), 1)
        cache = next(p for p in plugins if p.get('name') == 'rm_27_stimulation::CadCollisionCacheSystem')
        self.assertEqual(sorted(p.text for p in cache.findall('mesh_file')),
                         sorted(Path(m.findtext('uri')).name for m in meshes))
        self.assertEqual(spawn._convex_collision_description('<robot name="other"/>'), '<robot name="other"/>')


class DecompositionTests(unittest.TestCase):
    def test_disconnected_solids_preserve_gap_and_solid(self):
        corners = GEOMETRY.CORNERS * .01
        left = corners + [-.03, 0, 0]
        right = corners + [.03, 0, 0]
        surface = np.concatenate([p[ConvexHull(p).simplices] for p in (left, right)])
        hulls = [h for h, _ in GEOMETRY.decompose(surface)]
        self.assertEqual(len(hulls), 2)
        np.testing.assert_array_equal(inside(np.array([[0, 0, 0], [-.03, 0, 0], [.03, 0, 0]]), hulls),
                                      [False, True, True])

    def test_planar_fragment_is_kept_with_bounded_thickness(self):
        points = np.array([[0, 0, 0], [.01, 0, 0], [0, .01, 0]])
        hull = GEOMETRY.padded_hull(points)
        self.assertTrue(np.all(inside(points, [hull], -.00049)))
        self.assertLessEqual(np.ptp(hull.points[:, 2]), .00100001)

    def test_cut_preserves_crossing_triangle_area(self):
        triangle = np.array([[[0, 0, -1], [1, 0, 1], [0, 1, 1]]], dtype=float)
        low = GEOMETRY.clip_z(triangle, 0, False)
        high = GEOMETRY.clip_z(triangle, 0, True)
        def area(t):
            return np.linalg.norm(np.cross(t[:, 1] - t[:, 0], t[:, 2] - t[:, 0]), axis=1).sum() / 2

        self.assertAlmostEqual(area(triangle), area(low) + area(high))
        self.assertTrue(np.all(low[:, :, 2] <= 0))
        self.assertTrue(np.all(high[:, :, 2] >= 0))


if __name__ == '__main__':
    unittest.main()
