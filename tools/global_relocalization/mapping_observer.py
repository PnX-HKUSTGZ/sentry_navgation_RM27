#!/usr/bin/env python3
"""Read-only mapping display for RViz and Gazebo; never publishes control or SLAM TF."""
import argparse
import json
from pathlib import Path
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header, String
from sensor_msgs_py import point_cloud2
from visualization_msgs.msg import Marker, MarkerArray

from evaluation import interpolate, matrix, score_pose
from sensor_mapping import base_sensor, stats
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

FRAME = "mapping_eval_world"
COLORS = {"route": (0.5, 0.5, 0.5), "truth": (0.1, 1., 0.25),
          "estimate": (0., 0.8, 1.), "error": (1., 0.2, 0.1)}


def cloud_xyz(msg):
    raw = point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
    return np.column_stack([raw[n].reshape(-1) for n in ("x", "y", "z")])


class Observer(Node):
    def __init__(self, args):
        super().__init__("full_uc_mapping_observer")
        self.display_tf = StaticTransformBroadcaster(self)
        anchor = TransformStamped()
        anchor.header.frame_id, anchor.child_frame_id = FRAME, FRAME + "_display_anchor"
        anchor.transform.rotation.w = 1.
        self.display_tf.sendTransform(anchor)
        self.args = args
        self.gt, self.pending, self.errors = [], [], []
        self.alignment, self.initial_stamp = None, None
        self.latest_cloud, self.voxels, self.display_points = None, {}, None
        self.frozen, self.last_gz, self.gz_ok, self.last_update = False, 0., False, 0.
        self.skipped, self.input_samples = 0, 0
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.markers_pub = self.create_publisher(MarkerArray, "/mapping_eval/markers", qos)
        self.cloud_pub = self.create_publisher(PointCloud2, "/mapping_eval/map", qos)
        self.scan_pub = self.create_publisher(PointCloud2, "/mapping_eval/current_scan", qos)
        self.grid_pub = self.create_publisher(OccupancyGrid, "/mapping_eval/occupancy", qos)
        self.grid_cloud_pub = self.create_publisher(PointCloud2, "/mapping_eval/occupancy_cloud", qos)
        self.status_pub = self.create_publisher(String, "/mapping_eval/status", qos)
        self.create_subscription(Odometry, "/ground_truth/odometry", self.truth, qos_profile_sensor_data)
        if not getattr(args, "skip_odom_subscription", False):
            self.create_subscription(Odometry, "/aft_mapped_to_init", self.odom, 100)
        self.create_subscription(PointCloud2, "/cloud_registered", self.cloud, qos_profile_sensor_data)
        self.create_subscription(OccupancyGrid, "/map", self.grid, qos)
        self.grid_message, self.grid_cloud, self.map_transform = None, None, None
        self.gz_node = None
        if args.gazebo:
            from gz.transport13 import Node as GzNode
            self.gz_node = GzNode()
        self.create_timer(0.5, self.update)

    @staticmethod
    def stamp(msg):
        return msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec

    def truth(self, msg):
        if not self.frozen:
            stamp = self.stamp(msg)
            if self.gt and stamp <= self.gt[-1][0]:
                return
            self.gt.append((stamp, matrix(msg.pose.pose)))

    def odom(self, msg):
        if not self.frozen:
            stamp = self.stamp(msg)
            if not self.pending or stamp - self.pending[-1][0] >= .049:
                self.pending.append((stamp, matrix(msg.pose.pose)))
                self.input_samples += 1

    def cloud(self, msg):
        if not self.frozen:
            self.latest_cloud = msg

    def grid(self, msg):
        if not self.frozen:
            self.grid_message = msg

    def header(self):
        return Header(frame_id=FRAME)

    def score(self):
        remaining = []
        for stamp, pose in self.pending:
            truth = interpolate(self.gt, stamp, .25)
            if truth is None:
                if not self.gt or stamp > self.gt[-1][0]:
                    remaining.append((stamp, pose))
                else:
                    self.skipped += 1
                continue
            if self.alignment is None:
                self.alignment = truth @ base_sensor() @ np.linalg.inv(pose)
                self.initial_stamp = stamp
                # loam_interface fixes odom->camera_init to T_base_lidar.
                # With baseline map->odom identity, map->world follows this
                # single gauge; no geometry registration is fitted here.
                self.map_transform = self.alignment @ np.linalg.inv(base_sensor())
            estimate = self.alignment @ pose @ np.linalg.inv(base_sensor())
            self.errors.append({"stamp": stamp, "estimated_matrix": estimate.tolist(),
                                "truth_matrix": truth.tolist(), **score_pose(estimate, truth)})
        self.pending = remaining

    def load_result(self):
        path = self.args.result / "mapping_errors.json"
        if self.frozen or not path.exists():
            return
        try:
            manifest = json.loads((self.args.result / "manifest.json").read_text())
            self.errors = json.loads(path.read_text())
            capture = json.loads((self.args.result / "capture.json").read_text())
            self.gt = [(r["stamp"], np.asarray(r["matrix"])) for r in capture["gt"]]
            self.alignment = np.asarray(manifest["T_world_camera_init"])
            self.map_transform = self.alignment @ np.linalg.inv(base_sensor())
            from audit_prior_geometry import read_xyz
            points = read_xyz(self.args.result / "map.pcd")
            stride = max(1, int(np.ceil(len(points) / self.args.max_points)))
            points = points[::stride]
            self.voxels = {tuple(key): point for key, point in zip(np.floor(points / .08).astype(int), points)}
            saved_grid = self.args.result / "visual_occupancy.npz"
            if saved_grid.exists():
                points = np.load(saved_grid)["points"]
                fields = [PointField(name=key, offset=i * 4, datatype=PointField.FLOAT32, count=1)
                          for i, key in enumerate(("x", "y", "z"))]
                fields.append(PointField(name="rgba", offset=12, datatype=PointField.UINT32, count=1))
                self.grid_cloud = point_cloud2.create_cloud(self.header(), fields, points)
            self.frozen = True
            self.get_logger().info("Loaded complete saved evidence; display frozen at the end of the capture")
        except (OSError, ValueError, KeyError):
            pass  # Export publishes multiple files; retry when all are ready.

    def publish_cloud(self):
        if self.latest_cloud is not None and self.alignment is not None and not self.frozen:
            points = cloud_xyz(self.latest_cloud)
            points = points @ self.alignment[:3, :3].T + self.alignment[:3, 3]
            points = points[np.isfinite(points).all(axis=1)]
            self.scan_pub.publish(point_cloud2.create_cloud_xyz32(self.header(), points.astype(np.float32)))
            for key, point in zip(np.floor(points / .08).astype(int), points):
                self.voxels[tuple(key)] = point
            # Bound display memory independently of the full-resolution export.
            if len(self.voxels) > 2 * self.args.max_points:
                keys = list(self.voxels)[::2]
                self.voxels = {key: self.voxels[key] for key in keys}
            self.latest_cloud = None
        if not self.voxels:
            return
        points = np.asarray(list(self.voxels.values()), dtype=np.float32)
        stride = max(1, int(np.ceil(len(points) / self.args.max_points)))
        points = points[::stride]
        self.display_points = points
        self.cloud_pub.publish(point_cloud2.create_cloud_xyz32(self.header(), points))

    def marker(self, name, index, points, kind=Marker.LINE_STRIP):
        msg = Marker(header=self.header(), ns="mapping_eval", id=index, type=kind, action=Marker.ADD)
        msg.pose.orientation.w = 1.
        msg.scale.x = .06
        msg.scale.y = .1
        msg.scale.z = .1
        msg.color.r, msg.color.g, msg.color.b = COLORS[name]
        msg.color.a = 1.
        msg.points = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2]) + .12) for p in points]
        return msg

    def gazebo(self, messages):
        if self.gz_node is None or time.monotonic() - self.last_gz < 1.:
            return
        self.last_gz = time.monotonic()
        from gz.msgs10.marker_pb2 import Marker as GzMarker
        from gz.msgs10.empty_pb2 import Empty
        successes = []
        for item in messages:
            # Ogre2 does not implement gz::msgs::Marker::TEXT. RViz carries
            # the metric labels; Gazebo shows the geometry and native stats.
            if item.type not in (Marker.LINE_LIST, Marker.LINE_STRIP, Marker.POINTS, Marker.ARROW):
                continue
            marker = GzMarker(ns="full_uc_mapping_eval", id=item.id + 10, action=GzMarker.ADD_MODIFY,
                              visibility=GzMarker.GUI)
            marker.type = {Marker.LINE_LIST: GzMarker.LINE_LIST, Marker.LINE_STRIP: GzMarker.LINE_STRIP,
                           Marker.POINTS: GzMarker.POINTS,
                           Marker.ARROW: GzMarker.LINE_STRIP}[item.type]
            marker.pose.orientation.w = 1.
            marker.scale.x = marker.scale.y = marker.scale.z = 1.
            for color in (marker.material.ambient, marker.material.diffuse, marker.material.emissive):
                color.r, color.g, color.b, color.a = item.color.r, item.color.g, item.color.b, 1.
            for point in item.points:
                marker.point.add(x=point.x, y=point.y, z=point.z)
            # This is a one-way service: a false response flag does not mean
            # rejection. Verify the fixed marker IDs with the list service.
            self.gz_node.request("/marker", marker, GzMarker, Empty, 5)
            successes.append(marker.id)
        from gz.msgs10.marker_v_pb2 import Marker_V
        ok, listed = self.gz_node.request("/marker/list", Empty(), Empty, Marker_V, 50)
        present = {m.id for m in listed.marker if m.ns == "full_uc_mapping_eval"}
        self.gz_ok = ok and bool(successes) and set(successes).issubset(present)

    def update(self):
        self.load_result()
        if not self.frozen:
            self.score()
        self.publish_cloud()
        messages = []
        route_path = self.args.result / "route.json"
        if route_path.exists():
            route = json.loads(route_path.read_text())
            xyz = np.column_stack((route["waypoints"], route.get("ground_heights_m", [0.] * len(route["waypoints"]))))
            messages.append(self.marker("route", 0, xyz))
        if self.errors:
            selected = self.errors[::max(1, len(self.errors) // 1800)]
            truth = np.asarray([r["truth_matrix"] for r in selected])[:, :3, 3]
            estimate = np.asarray([r["estimated_matrix"] for r in selected])[:, :3, 3]
            full_truth = np.asarray([pose[:3, 3] for _, pose in self.gt[::max(1, len(self.gt) // 3000)]])
            messages.extend([self.marker("truth", 1, full_truth), self.marker("estimate", 2, estimate)])
            pairs = np.column_stack((truth[::30], estimate[::30])).reshape(-1, 3)
            messages.append(self.marker("error", 3, pairs, Marker.LINE_LIST))
            for name, index, pose_key in (("truth", 5, "truth_matrix"), ("estimate", 6, "estimated_matrix")):
                pose = np.asarray(self.errors[-1][pose_key])
                if name == "truth" and self.gt:
                    pose = self.gt[-1][1]
                arrow = self.marker(name, index, [pose[:3, 3], pose[:3, 3] + pose[:3, 0] * .8], Marker.ARROW)
                arrow.scale.x, arrow.scale.y, arrow.scale.z = .1, .22, .22
                messages.append(arrow)
        if self.display_points is not None:
            points = self.display_points[::max(1, int(np.ceil(len(self.display_points) / 10000)))]
            cloud_marker = self.marker("estimate", 9, points, Marker.POINTS)
            cloud_marker.scale.x = cloud_marker.scale.y = .035
            # RViz already has its height-coloured PointCloud2; this marker is
            # sent only to the Gazebo GUI to keep both displays inexpensive.
        else:
            cloud_marker = None
        status = {"mode": "saved_result" if self.frozen else getattr(self, "display_mode", "live"), "samples": len(self.errors),
                  "unmatched_samples": self.skipped, "gazebo_markers_available": self.gz_ok,
                  "translation_3d_m": stats([r["translation_3d_m"] for r in self.errors]),
                  "orientation_so3_deg": stats([r["orientation_so3_deg"] for r in self.errors])}
        progress = self.args.result / "progress.json"
        if progress.exists():
            status["progress"] = json.loads(progress.read_text())
        summary = self.args.result / "mapping_summary.json"
        if summary.exists():
            final = json.loads(summary.read_text())
            status.update(capture_complete=final.get("capture_complete"), failure=final.get("failure"))
        text = Marker(header=self.header(), ns="mapping_eval", id=4, type=Marker.TEXT_VIEW_FACING, action=Marker.ADD)
        text.pose.orientation.w, text.pose.position.y, text.pose.position.z = 1., 10., 1.
        text.scale.z, text.color.r, text.color.g, text.color.b, text.color.a = .4, 1., 1., 1., 1.
        text.text = "FULL UC | green: truth | cyan: Point-LIO | red: error\n"
        if self.errors:
            text.text += (f"{status['mode']} | ATE RMSE {status['translation_3d_m']['rmse']:.3f} m | "
                          f"last scored {self.errors[-1]['translation_3d_m']:.3f} m | "
                          f"rotation {self.errors[-1]['orientation_so3_deg']:.2f} deg")
        else:
            text.text += "Waiting for timestamp-matched truth and Point-LIO"
        if "progress" in status:
            text.text += f"\nRoute {status['progress']['completed_waypoints']}/{status['progress']['total_waypoints']}"
        replay_time = getattr(self, "replay_time", None)
        if replay_time is not None:
            text.text += f"\nBag time: {replay_time:.1f} s"
            if self.errors and replay_time - self.errors[-1]["stamp"] > 2:
                text.text += f" | Last valid score: {self.errors[-1]['stamp']:.1f} s (stale)"
        messages.append(text)
        if self.frozen and status.get("failure"):
            text.text += "\nMAPPING FAILED / incomplete estimate: " + status["failure"]
        self.markers_pub.publish(MarkerArray(markers=messages))
        self.gazebo(messages + ([cloud_marker] if cloud_marker is not None else []))
        if self.grid_message is not None and self.map_transform is not None:
            from scipy.spatial.transform import Rotation
            msg = self.grid_message
            origin = self.map_transform @ matrix(msg.info.origin)
            msg.header.frame_id = FRAME
            msg.info.origin.position.x, msg.info.origin.position.y, msg.info.origin.position.z = map(float, origin[:3, 3])
            q = Rotation.from_matrix(origin[:3, :3]).as_quat()
            (msg.info.origin.orientation.x, msg.info.origin.orientation.y,
             msg.info.origin.orientation.z, msg.info.origin.orientation.w) = map(float, q)
            self.grid_pub.publish(msg)
            values = np.asarray(msg.data).reshape(msg.info.height, msg.info.width)
            rows, cols = np.where(values >= 0)
            xy = np.column_stack(((cols + .5) * msg.info.resolution, (rows + .5) * msg.info.resolution,
                                  np.zeros(len(cols))))
            xy = xy @ origin[:3, :3].T + origin[:3, 3]
            rgba = np.where(values[rows, cols] >= 65, np.uint32(0xFFEBE7DF), np.uint32(0xFF424954))
            structured = np.zeros(len(xy), dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("rgba", "<u4")])
            for i, key in enumerate(("x", "y", "z")):
                structured[key] = xy[:, i]
            structured["rgba"] = rgba
            fields = [PointField(name=key, offset=i * 4, datatype=PointField.FLOAT32, count=1)
                      for i, key in enumerate(("x", "y", "z"))]
            fields.append(PointField(name="rgba", offset=12, datatype=PointField.UINT32, count=1))
            self.grid_cloud = point_cloud2.create_cloud(self.header(), fields, structured)
            np.savez_compressed(self.args.result / "visual_occupancy.npz", points=structured)
            self.grid_message = None
        if self.grid_cloud is not None:
            self.grid_cloud_pub.publish(self.grid_cloud)
        status["gazebo_markers_available"] = self.gz_ok
        self.status_pub.publish(String(data=json.dumps(status)))
        if self.args.result.exists():
            path = self.args.result / "visualization_status.json"
            temporary = path.with_suffix(".tmp")
            temporary.write_text(json.dumps(status, indent=2) + "\n")
            temporary.replace(path)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--result", type=Path, required=True)
    parser.add_argument("--gazebo", action="store_true")
    parser.add_argument("--max-points", type=int, default=60000)
    args = parser.parse_args()
    rclpy.init(args=[])
    node = Observer(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
