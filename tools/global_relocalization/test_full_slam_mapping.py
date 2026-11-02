"""The final graph must change the fused cloud, without using GT per frame."""
import json
from pathlib import Path

import numpy as np
from full_slam_mapping import GraphCorrectionTimeline
import pytest
from scipy.spatial.transform import Rotation

from full_slam_mapping import (base_sensor, export_optimized, se2, StationTurnTracker,
                               station_already_scanned, surveyed_vertical_velocity)


def test_station_turn_uses_measured_yaw_across_wrap_and_requires_complete_turn():
    tracker = StationTurnTracker(np.deg2rad(170), 0, turns=1, timeout=30)
    for index, travel in enumerate(np.linspace(0, 2*np.pi+.01, 65)):
        yaw = np.arctan2(np.sin(np.deg2rad(170)+travel), np.cos(np.deg2rad(170)+travel))
        done = tracker.update(yaw, index*.1)
        assert done == (index == 64)
    assert tracker.progress >= 2*np.pi


def test_station_turn_does_not_count_oscillation_or_command_time_as_motion():
    tracker = StationTurnTracker(0, 0, turns=1, timeout=30, stall_timeout=2)
    for index in range(20):
        assert not tracker.update(.001 if index%2 else -.001, index*.1)
    with pytest.raises(TimeoutError, match="no sustained measured angular motion"):
        tracker.update(0, 2.1)
    assert abs(tracker.progress) < .002


def test_station_turn_total_timeout_even_with_slow_actual_motion():
    tracker = StationTurnTracker(0, 0, turns=1, timeout=3)
    for index in range(1, 7):
        assert not tracker.update(index*.03, index*.5)
    with pytest.raises(TimeoutError, match="timed out"):
        tracker.update(.21, 3.5)


def test_station_deduplication_and_surveyed_height_velocity_bounds():
    assert station_already_scanned([0.1, 0], [[0, 0]])
    assert not station_already_scanned([.21, 0], [[0, 0]])
    class Planner:
        ground = np.full((2, 2), .5)
        def cell(self, xy):
            return 0, 0
    pose = np.eye(4)
    pose[2, 3] = .08
    assert surveyed_vertical_velocity(Planner(), pose, [.2, 0]) == .25
    pose[2, 3] = .58
    assert abs(surveyed_vertical_velocity(Planner(), pose, [0, 0])) < 1e-12


def test_long_graph_gap_preserves_station_scans_only_with_continuous_odometry():
    corrections = [(0.0, se2([0, 0, 0])), (8.0, se2([0.08, 0, 0]))]
    odom = [(float(t), se2([0, 0, t])) for t in np.linspace(0, 8, 81)]
    timeline = GraphCorrectionTimeline(corrections, odom)
    assert np.isclose(timeline.at(4)[0, 3], 0.04)
    assert timeline.at(-0.1) is None and timeline.at(8.1) is None
    broken = GraphCorrectionTimeline(corrections, odom[:20] + odom[30:])
    assert broken.at(4) is None
    assert broken.at(8) is corrections[-1][1]


def test_gravity_aligned_initial_mount_is_not_applied_twice():
    initial_camera_sensor = base_sensor().copy()
    initial_camera_sensor[:3, 3] = 0
    odom_camera = base_sensor() @ np.linalg.inv(initial_camera_sensor)
    first_base = odom_camera @ initial_camera_sensor @ np.linalg.inv(base_sensor())
    np.testing.assert_allclose(first_base, np.eye(4), atol=1e-12)
    ground_camera = np.array([1, 2, -0.13, 1])
    assert abs((odom_camera @ ground_camera)[2]) < 1e-12


def test_export_applies_final_graph_at_each_timestamp_and_one_gt_anchor(tmp_path):
    out = tmp_path
    (out / "registered_scans").mkdir()
    nodes = [{"id": i, "stamp": float(i), "odometry_xy_yaw": [i * 1.1, 0, 0],
              "optimized_xy_yaw": [float(i), 0, 0]} for i in range(10)]
    (out / "graph_optimized.json").write_text(json.dumps({"nodes": nodes, "edges": []}))
    for name in ["mapping_params.yaml", "slam.posegraph", "slam.data"]:
        (out / name).write_text("test")
    clouds, odom, gt = [], [], []
    for i in range(10):
        np.save(out / "registered_scans" / f"{i:06d}.npy", np.array([[i * 1.1 + 1, 0, 0]]))
        clouds.append({"index": i, "stamp": float(i)})
        odom.append({"stamp": float(i), "matrix": (se2([i * 1.1, 0, 0]) @ base_sensor()).tolist()})
        gt.append({"stamp": float(i), "matrix": se2([i + 10, 20, 0]).tolist()})
    capture = {"gt": gt, "odom": odom, "clouds": clouds, "T_odom_camera_init": np.eye(4).tolist(),
               "completed_waypoints": 1, "total_waypoints": 1, "failure": None}
    summary = export_optimized(out, capture, 0.01)
    assert summary["translation_3d_m"]["max"] < 1e-10
    payload = (out / "map.pcd").read_bytes().split(b"DATA binary\n")[1]
    points = np.frombuffer(payload, dtype="<f4").reshape(-1, 3)
    np.testing.assert_allclose(points[:, 0], np.arange(10) + 11)
    # Later GT deliberately wrong: points and fusion must remain identical.
    for row in gt[1:]:
        row["matrix"][0][3] += 100
    before = (out / "map.pcd").read_bytes()
    summary = export_optimized(out, capture, 0.01)
    assert summary["translation_3d_m"]["max"] > 99
    assert before == (out / "map.pcd").read_bytes()
    manifest = json.loads((out / "manifest.json").read_text())
    assert manifest["optimized_graph_used_for_3d_fusion"] is True
    assert manifest["ground_truth_used_for_scan_fusion"] is False


def test_initial_alignment_uses_full_lio_base_pose_not_planar_graph(tmp_path):
    (tmp_path / "registered_scans").mkdir()
    base = np.eye(4)
    base[:3, :3] = Rotation.from_euler("xyz", [5, -3, 8], degrees=True).as_matrix()
    base[:3, 3] = [0.2, -0.1, 0.15]
    sensor = base @ base_sensor()
    truth = se2([10, 20, 0.3])
    nodes = [{"id": i, "stamp": float(i), "odometry_xy_yaw": [0.2, -0.1, np.deg2rad(8)],
              "optimized_xy_yaw": [0.2, -0.1, np.deg2rad(8)]} for i in range(10)]
    (tmp_path / "graph_optimized.json").write_text(json.dumps({"nodes": nodes, "edges": []}))
    for name in ["mapping_params.yaml", "slam.posegraph", "slam.data"]:
        (tmp_path / name).write_text("test")
    clouds = []
    for i in range(10):
        np.save(tmp_path / "registered_scans" / f"{i:06d}.npy", np.array([(sensor @ [1, 0, 0, 1])[:3]]))
        clouds.append({"index": i, "stamp": float(i)})
    capture = {"gt": [{"stamp": float(i), "matrix": truth.tolist()} for i in range(10)],
               "odom": [{"stamp": float(i), "matrix": sensor.tolist()} for i in range(10)],
               "clouds": clouds, "T_odom_camera_init": np.eye(4).tolist(),
               "completed_waypoints": 1, "total_waypoints": 1, "failure": None}
    summary = export_optimized(tmp_path, capture, 0.01)
    assert summary["translation_3d_m"]["max"] < 1e-10
    assert summary["orientation_so3_deg"]["max"] < 1e-10
    points = np.frombuffer((tmp_path / "map.pcd").read_bytes().split(b"DATA binary\n")[1], dtype="<f4").reshape(-1, 3)
    np.testing.assert_allclose(points[0], (truth @ base_sensor() @ [1, 0, 0, 1])[:3], atol=1e-6)


def test_station_settle_waits_for_fresh_sensor_time_and_stalled_clock_fails():
    from types import SimpleNamespace
    from full_slam_mapping import wait_station_settle
    class Node:
        def __init__(self,stalled=False):
            self.wall=0.;self.clock_ns=1_000_000_000;self.odom=[(.9,np.eye(4))];self.commands=[];self.stalled=stalled
        def get_clock(self):return SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=self.clock_ns))
        def command(self,*args):self.commands.append(args)
        def spin_for(self,seconds):
            self.wall+=seconds;self.clock_ns+=round(seconds*1e9)
            if not self.stalled:self.odom.append(((self.clock_ns-100_000_000)*1e-9,np.eye(4)))
    node=Node();record=wait_station_settle(node,lambda:np.eye(4),2.5,wall_now=lambda:node.wall)
    assert record['settle_start_sensor_stamp_ns']>record['settle_zero_command_ros_ns']
    assert record['settle_end_sensor_stamp_ns']-record['settle_start_sensor_stamp_ns']>=2_500_000_000
    assert all(args==()for args in node.commands)
    frozen=Node(stalled=True)
    with pytest.raises(TimeoutError,match='sensor clock'):wait_station_settle(frozen,lambda:np.eye(4),.5,wall_now=lambda:frozen.wall)


def test_ordered_protocol_controls_keep_fixed_overscan_and_settle_requirements():
    from types import SimpleNamespace
    from full_slam_mapping import validate_ordered_controls
    protocol={'minimum_control':{'station_scan_turns':2.05,'station_settle_sec':2.5,'station_scan_speed_rad_sec':1.,'maximum_translation_speed_m_sec':.3,'rotation_speed_rad_sec':0.,'survey_terrain':'surveyed','control_source':'simulator'}}
    args=SimpleNamespace(station_scan_turns=2.05,station_settle_sec=2.5,station_scan_speed=1.,speed=.3,rotation_speed=0.,survey_terrain='surveyed',control_source='simulator')
    validate_ordered_controls(protocol,args)
    args.station_scan_turns=2.
    with pytest.raises(ValueError,match='declared'):validate_ordered_controls(protocol,args)
    args.station_scan_turns=2.05;args.station_settle_sec=.5
    with pytest.raises(ValueError):validate_ordered_controls(protocol,args)


def test_ordered_closure_visit_is_distinct_while_legacy_xy_dedup_is_preserved():
    from full_slam_mapping import station_visit_already_scanned
    xy=[-11.7,3.1];scanned=[xy];ids={'initial_visit'}
    assert station_visit_already_scanned(xy,scanned)
    assert station_visit_already_scanned(xy,scanned,'initial_visit',ids)
    assert not station_visit_already_scanned(xy,scanned,'closure_visit',ids)
