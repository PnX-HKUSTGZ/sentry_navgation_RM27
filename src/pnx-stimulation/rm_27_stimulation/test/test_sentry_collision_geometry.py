"""校验实际 URDF 引用、所有 CAD 顶点包含关系和完整旋转足迹。"""

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


class SentryCollisionGeometry(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.raw, _, _ = GEOMETRY.source_groups()
        with contextlib.redirect_stdout(io.StringIO()):
            cls.report = GEOMETRY.generate(check=True)

    def test_actual_urdf_contains_every_visual_vertex(self):
        # 缩放时轮碰撞网格还须补偿固定关节中心偏移，不能只缩放局部网格。
        for scale in (1.0, .8, 1.2):
            robot = xacro.process_file(
                str(ROOT / 'urdf/simulation_waking_robot.xacro'),
                mappings={'cad_visual_scale': str(scale)}).documentElement
            links = {n.getAttribute('name'): n for n in robot.getElementsByTagName('link')}
            joints = {n.getElementsByTagName('child')[0].getAttribute('link'): n
                      for n in robot.getElementsByTagName('joint')}
            sources = {'base_link': ['chassis_body', 'chassis_armor'],
                       'gimbal_yaw': ['yaw_body', 'yaw_sensors'], 'gimbal_roll': ['roll_body']}
            for link, names in sources.items():
                for name in names:
                    self.check_containment(links[link], name, self.raw[name] * scale)
            for index, (sx, sy) in enumerate(GEOMETRY.WHEELS, 1):
                name = f'wheel_{index}'
                origin = joints[name].getElementsByTagName('origin')[0]
                center = np.fromstring(origin.getAttribute('xyz'), sep=' ')
                rotation = Rotation.from_euler('xyz', np.fromstring(
                    origin.getAttribute('rpy'), sep=' ')).as_matrix()
                raw = self.raw['wheels']
                points = raw[(raw[:, 0] * sx > 0) & (raw[:, 1] * sy > 0)]
                self.check_containment(links[name], name, (points * scale - center) @ rotation)
            self.assertEqual(sum(len(link.getElementsByTagName('collision'))
                                 for link in links.values()), 9)

    def check_containment(self, link, mesh_name, points):
        collisions = {n.getAttribute('name'): n for n in link.getElementsByTagName('collision')}
        collision = collisions[mesh_name + '_collision']
        mesh = collision.getElementsByTagName('mesh')[0]
        self.assertEqual(mesh.getAttribute('filename'),
                         f'package://rm_27_stimulation/meshes/rm27_sentry_collision/{mesh_name}.stl')
        vertices = GEOMETRY.vertices(GEOMETRY.OUTPUT / (mesh_name + '.stl'))
        vertices *= np.fromstring(mesh.getAttribute('scale'), sep=' ')
        origin = collision.getElementsByTagName('origin')[0]
        vertices += np.fromstring(origin.getAttribute('xyz'), sep=' ')
        # 验证全部顶点，不只验证包围盒或稀疏采样。凸包也包含其间全部三角面。
        self.assertLessEqual(GEOMETRY.max_plane_violation(points, ConvexHull(vertices)), -0.00039)

    def test_navigation_footprint_covers_complete_articulation(self):
        config = ROOT.parents[1] / 'pnx-launch/launch_bringup/config/simulation/minco_params.yaml'
        params = yaml.safe_load(config.read_text())
        safety = params['planner_server']['ros__parameters']['MincoPlanner']['safety']
        polygon = np.array(safety['footprint_points']).reshape(-1, 2)
        np.testing.assert_allclose(polygon, self.report['footprint_points'], atol=1e-9, rtol=0)
        for name in ('local_costmap', 'global_costmap'):
            value = params[name][name]['ros__parameters']['footprint']
            np.testing.assert_array_equal(np.array(json.loads(value)), polygon)
        hull = ConvexHull(polygon)
        self.assertTrue(np.all(-hull.equations[:, 2] >= self.report['moving_swept_radius_m'] + .00199))
        self.assertGreaterEqual(safety['footprint_length'], np.ptp(polygon[:, 0]))
        self.assertGreaterEqual(safety['footprint_width'], np.ptp(polygon[:, 1]))
        self.assertLess(safety['footprint_length'], .636)

    def test_spawn_explicitly_uses_exact_convex_meshes(self):
        spec = importlib.util.spec_from_file_location('spawn_robot', ROOT / 'launch/spawn_robot.launch.py')
        spawn = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(spawn)
        urdf = xacro.process_file(str(ROOT / 'urdf/simulation_waking_robot.xacro')).toxml()
        sdf = ET.fromstring(spawn._convex_collision_description(urdf))
        meshes = sdf.findall('.//collision/geometry/mesh')
        self.assertEqual(len(meshes), 9)
        self.assertTrue(all(mesh.get('optimization') == 'convex_hull' for mesh in meshes))
        plugins = sdf.findall('.//plugin')
        self.assertEqual(sum(p.get('name') == 'rm_27_stimulation::CadCollisionCacheSystem'
                             for p in plugins), 1)
        self.assertEqual(spawn._convex_collision_description('<robot name="other"/>'), '<robot name="other"/>')


if __name__ == '__main__':
    unittest.main()
