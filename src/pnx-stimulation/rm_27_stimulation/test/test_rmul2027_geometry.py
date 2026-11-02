"""CAD conversion checks; run using the documented optional CAD environment."""

import json
from pathlib import Path
import sys
from tempfile import TemporaryDirectory
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

import numpy as np
from shapely.geometry import Polygon, Point
from shapely.ops import unary_union
import trimesh

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(PACKAGE/'tools'))
from import_rmul2027_world import convex_patches
from rmul2027_collision import decompose_region, merge_boxes, publish_collision, right_triangles


class ConvexBoundaryTests(unittest.TestCase):
    def test_annular_face_keeps_its_hole(self):
        vertices = np.array([[-2,-2,0],[2,-2,0],[2,2,0],[-2,2,0],
                             [-1,-1,0],[1,-1,0],[1,1,0],[-1,1,0]],float)
        triangles = np.array([[0,1,5],[0,5,4],[1,2,6],[1,6,5],
                              [2,3,7],[2,7,6],[3,0,4],[3,4,7]])
        patches, normal = convex_patches(vertices,triangles)
        union = unary_union([Polygon(p[:,:2]) for p in patches])
        self.assertAlmostEqual(union.area,12)
        self.assertFalse(union.covers(Point(0,0)))
        self.assertTrue(union.covers(Point(1.5,1.5)))
        np.testing.assert_allclose(normal,[0,0,1])

    def test_concave_corner_is_not_closed_by_a_hull(self):
        vertices = np.array([[0,0,0],[2,0,0],[2,1,0],[1,1,0],[1,2,0],[0,2,0]],float)
        triangles = np.array([[0,1,3],[1,2,3],[0,3,5],[3,4,5]])
        patches,_ = convex_patches(vertices,triangles)
        union = unary_union([Polygon(p[:,:2]) for p in patches])
        self.assertAlmostEqual(union.area,3)
        self.assertFalse(union.covers(Point(1.5,1.5)))

    def test_reverse_face_preserves_inward_extrusion_direction(self):
        vertices = np.array([[0,0,1],[1,0,1],[1,1,1],[0,1,1]],float)
        _,normal = convex_patches(vertices,np.array([[2,1,0],[3,2,0]]))
        np.testing.assert_allclose(normal,[0,0,-1])


class GeneratedAssetTests(unittest.TestCase):
    def test_collision_lod_reuses_one_convex_mesh(self):
        root = ET.parse(PACKAGE/'meshes/RMUL2027/RMUL2027_collision/model.sdf')
        collisions = root.findall('.//collision')
        self.assertLessEqual(len(collisions),607)
        meshes = root.findall('.//collision/geometry/mesh')
        self.assertEqual(len({m.findtext('uri') for m in meshes}),1)
        self.assertTrue(all(m.get('optimization') == 'convex_hull' for m in meshes))
        # Preserve the native convex path / 1 mm contact margin; default mesh
        # collisions would switch Bullet to a different 10 mm-margin shape.
        self.assertTrue(all(m.findtext('convex_decomposition/max_convex_hulls') == '1' for m in meshes))

    def test_models_use_separate_visuals_and_collisions(self):
        assets = PACKAGE/'meshes/RMUL2027'
        for suffix in ('visual','collision'):
            root = ET.parse(assets/f'RMUL2027_{suffix}/model.sdf').getroot()
            self.assertEqual(root.findtext('model/static'),'true')
            self.assertFalse(root.findall('.//collision' if suffix == 'visual' else './/visual'))
            for uri in root.findall('.//uri'):
                self.assertTrue((assets/uri.text.removeprefix('model://')).is_file(),uri.text)

    def test_every_exported_collision_mesh_is_a_closed_convex_solid(self):
        folder = PACKAGE/'meshes/RMUL2027/RMUL2027_collision/meshes'
        paths = list(folder.glob('*.stl'))
        self.assertTrue(paths)
        for path in paths:
            mesh = trimesh.load_mesh(path)
            self.assertTrue(mesh.is_watertight,path.name)
            self.assertTrue(mesh.is_convex,path.name)
            self.assertGreater(mesh.volume,0,path.name)

    def test_dae_uses_metres_z_up_and_identity_model_transform(self):
        folder = PACKAGE/'meshes/RMUL2027/RMUL2027_visual/meshes'
        ns = {'c':'http://www.collada.org/2005/11/COLLADASchema'}
        for path in folder.glob('*.dae'):
            root = ET.parse(path).getroot()
            self.assertEqual(root.find('c:asset/c:unit',ns).get('meter'),'1')
            self.assertEqual(root.findtext('c:asset/c:up_axis',namespaces=ns),'Z_UP')
            for node in root.findall('.//c:library_visual_scenes/c:visual_scene/c:node',ns):
                matrix = np.fromstring(node.findtext('c:matrix',namespaces=ns),sep=' ')
                np.testing.assert_allclose(matrix.reshape(4,4),np.eye(4))

    def test_center_spawn_support_is_recorded(self):
        metadata = json.loads((PACKAGE/'meshes/RMUL2027/import_metadata.json').read_text())
        self.assertAlmostEqual(metadata['origin_support_height_m'],.011,places=6)
        self.assertEqual(metadata['face_count'],1274)


class SolidDecompositionTests(unittest.TestCase):
    def test_failed_generation_preserves_previous_assets(self):
        with TemporaryDirectory() as temporary:
            folder = Path(temporary)/'collision'
            (folder/'meshes').mkdir(parents=True)
            (folder/'model.sdf').write_text('previous model')
            (folder/'meshes/hull_previous.stl').write_bytes(b'previous mesh')
            with patch('rmul2027_collision.generate_collision',side_effect=RuntimeError('invalid CAD')):
                with self.assertRaises(RuntimeError):
                    publish_collision([],folder)
            self.assertEqual((folder/'model.sdf').read_text(),'previous model')
            self.assertEqual((folder/'meshes/hull_previous.stl').read_bytes(),b'previous mesh')
            self.assertFalse(list(Path(temporary).glob('.collision-build-*')))

    def test_region_hole_and_diagonal_boundaries_are_preserved(self):
        region = Polygon([(0,0),(3,0),(4,2),(3,4),(0,4)],
                         [[(1,1),(2,1),(2,2),(1,2)]])
        rectangles, triangles = decompose_region(region)
        from shapely.geometry import box
        reconstructed = unary_union([box(*r) for r in rectangles]+[Polygon(t) for t in triangles])
        self.assertLess(region.symmetric_difference(reconstructed).area,1e-9)
        self.assertFalse(reconstructed.covers(Point(1.5,1.5)))

    def test_right_triangle_split_preserves_a_skewed_triangle(self):
        original = np.array([[0.,0.],[2.,.4],[.7,1.9]])
        parts = list(right_triangles(original))
        self.assertAlmostEqual(sum(Polygon(t).area for t in parts),Polygon(original).area)
        for t in parts:
            self.assertAlmostEqual(np.dot(t[1]-t[0],t[2]-t[0]),0)

    def test_box_merge_does_not_bridge_an_opening(self):
        boxes = [[0,0,0,1,1,1],[1,0,0,2,1,1],[3,0,0,4,1,1]]
        merged = merge_boxes(boxes)
        self.assertEqual(len(merged),2)
        self.assertFalse(any(b[0] < 2.5 < b[3] for b in merged))


if __name__ == '__main__':
    unittest.main()
