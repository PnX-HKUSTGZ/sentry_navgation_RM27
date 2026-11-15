"""离线地图工具契约：字段读取、像素坐标、遮挡近似及不覆盖既有文件。"""
import importlib.util
from pathlib import Path
import numpy as np
import pytest

_spec = importlib.util.spec_from_file_location('prepare', Path(__file__).with_name('prepare_relocalization_map.py'))
module = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(module)


# 二进制 XYZI 输入可正确还原 XYZ，强度不会混入坐标。
def test_pcd_roundtrip(tmp_path):
    p = np.array([[1., 2., 3.], [-2., 0., .2]])
    module.write_pcd(tmp_path/'a.pcd', p)
    np.testing.assert_allclose(module.read_pcd(tmp_path/'a.pcd'), p, atol=1e-6)


# ASCII 任意字段顺序同样读取正确，忽略无效点。
def test_ascii_fields(tmp_path):
    path = tmp_path/'a.pcd'
    path.write_text('FIELDS intensity z x y\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\nPOINTS 2\nDATA ascii\n9 3 1 2\n2 nan 4 5\n')
    np.testing.assert_array_equal(module.read_pcd(path), [[1, 2, 3]])


# 图像行反转与旋转原点经过独立手算案例验证。
def test_grid_projection():
    rows, cols, valid = module.world_to_pixel(np.array([[9.75, 20.75], [9.25, 20.25]]), [10, 20, np.pi/2], .5, (4, 4))
    np.testing.assert_array_equal(rows, [3, 2])
    np.testing.assert_array_equal(cols, [1, 0])
    assert valid.all()


# 同一方向只保留最近回波，遮挡后点不能进入参考描述子。
def test_occlusion():
    cloud = module.visible_cloud(np.array([[2., 0., 0.], [4., 0., 0.], [0., 3., 0.]]), np.zeros(3), 10, 1, [-60, 60], 100)
    assert len(cloud) == 2
    assert not np.any(np.all(cloud == [4, 0, 0], axis=1))


# 压缩格式显式拒绝，避免将压缩字节当作浮点坐标。
def test_compressed_rejected(tmp_path):
    path = tmp_path/'compressed.pcd'
    path.write_text('FIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nPOINTS 1\nDATA binary_compressed\n')
    with pytest.raises(ValueError, match='binary_compressed'):
        module.read_pcd(path)


# 控制点用于独立验证变换，错误旋转和平移不能仅凭边界重叠通过。
def test_alignment_control_points(tmp_path):
    source = np.array([[0, 0, 0], [1, 0, 0], [0, 2, .1]])
    target = source + [2, 3, 1]
    path = tmp_path/'controls.csv'
    np.savetxt(path, np.column_stack([source, target]), delimiter=',')
    result = module.check_alignment_pairs(path, np.array([2, 3, 1, 0, 0, 0]), .01)
    assert result['verified'] and result['max_error_m'] < 1e-9
    with pytest.raises(ValueError, match='failed'):
        module.check_alignment_pairs(path, np.zeros(6), .01)


# 共线控制点不能检验整个二维坐标基准。
def test_collinear_alignment_rejected(tmp_path):
    source = np.array([[0, 0, 0], [1, 0, 0], [2, 0, 0]])
    path = tmp_path/'controls.csv'
    np.savetxt(path, np.column_stack([source, source]), delimiter=',')
    with pytest.raises(ValueError, match='collinear'):
        module.check_alignment_pairs(path, np.zeros(6), .01)
