#!/usr/bin/env python3
"""Use RViz's first initial-pose selection for Gazebo placement, never localization."""
import math
from pathlib import Path
import subprocess
import time

from ament_index_python.packages import get_package_prefix
import numpy as np
from PIL import Image
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Bool, String
import yaml


class PlacementMap:
    def __init__(self, filename):
        path = Path(filename)
        config = yaml.safe_load(path.read_text())
        self.resolution = float(config["resolution"])
        self.origin = config["origin"]
        with Image.open(path.parent / config["image"]) as image:
            pixels = np.asarray(image.convert("L")).copy()
        occupancy = (
            pixels / 255.0 if config.get("negate", 0) else (255.0 - pixels) / 255.0
        )
        self.free = occupancy < float(config["free_thresh"])
        self.height, self.width = self.free.shape
        grid = config["ground_elevation"]["grid"]
        with Image.open(path.parent / grid["image"]) as image:
            elevation = np.asarray(image).copy()
        if elevation.shape != self.free.shape:
            raise ValueError("Spawn elevation and occupancy grid dimensions differ")
        self.known_height = elevation != grid["no_data"]
        self.elevation = elevation * float(grid["scale"]) + float(grid["offset"])

    def spawn_pose(self, msg, clearance=0.5):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
        if msg.header.frame_id != "map" or not all(math.isfinite(v) for v in values):
            raise ValueError("Select a finite pose in RViz fixed frame map")
        if abs(math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w) - 1.0) > 0.01:
            raise ValueError("Invalid orientation quaternion")
        if p.x >= 0:
            raise ValueError(
                "The current global relocalizer searches the left half; select x < 0"
            )
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        c, s = math.cos(self.origin[2]), math.sin(self.origin[2])
        dx, dy = p.x - self.origin[0], p.y - self.origin[1]
        gx, gy = (c * dx + s * dy) / self.resolution, (
            -s * dx + c * dy
        ) / self.resolution
        col, row = math.floor(gx), self.height - 1 - math.floor(gy)
        radius = math.ceil(clearance / self.resolution + math.sqrt(0.5)) + 1
        if not (
            radius <= col < self.width - radius and radius <= row < self.height - radius
        ):
            raise ValueError("Selected footprint is outside the map")
        # Include a half-cell diagonal so every touched grid cell is checked.
        yy, xx = np.mgrid[
            row - radius : row + radius + 1, col - radius : col + radius + 1
        ]
        distance = (
            np.hypot(xx + 0.5 - gx, self.height - yy - 0.5 - gy) * self.resolution
        )
        mask = distance <= clearance + self.resolution * math.sqrt(0.5)
        if not np.all(self.free[yy[mask], xx[mask]]):
            raise ValueError(
                "Select open ground with 0.5 m clearance from obstacles/unknown cells"
            )
        if not self.known_height[row, col]:
            raise ValueError("Selected point has no known ground height")
        ground = float(self.elevation[row, col])
        if not -0.1 <= ground <= 0.6:
            raise ValueError("Selected height is outside the validated spawn range")
        return p.x, p.y, ground + 0.082, yaw


class SpawnFromRviz(Node):
    def __init__(self):
        super().__init__("spawn_from_rviz")
        self.declare_parameter("map_yaml", "")
        self.declare_parameter("world_name", "")
        self.declare_parameter("entity", "rm27_sentry")
        self.map = PlacementMap(self.get_parameter("map_yaml").value)
        self.process = None
        self.spawned = False
        self.started = 0.0
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.ready = self.create_publisher(Bool, "/simulation/spawned", qos)
        self.selection_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/simulation/spawn_pose", qos)
        self.selection = None
        self.status = self.create_publisher(String, "~/status", qos)
        self.ready.publish(Bool(data=False))
        self.create_subscription(
            PoseWithCovarianceStamped, "/initialpose", self.select, 10
        )
        self.create_timer(0.1, self.poll)
        self.report(
            "Waiting for RViz 2D Pose Estimate: select position and drag a heading on the left half"
        )

    def report(self, text):
        self.get_logger().info(text)
        self.status.publish(String(data=text))

    def select(self, msg):
        if self.spawned or self.process is not None:
            self.report(
                "Spawn already selected. Use Nav2 Goal to navigate; restart to choose a new spawn"
            )
            return
        try:
            x, y, z, yaw = self.map.spawn_pose(msg)
        except ValueError as exc:
            self.report(f"Spawn rejected: {exc}. Select another point")
            return
        executable = Path(get_package_prefix("ros_gz_sim")) / "lib/ros_gz_sim/create"
        command = [
            str(executable),
            "-world",
            self.get_parameter("world_name").value,
            "-name",
            self.get_parameter("entity").value,
            "-allow_renaming",
            "false",
            "-topic",
            "/robot_description",
            "-x",
            str(x),
            "-y",
            str(y),
            "-z",
            str(z),
            "-Y",
            str(yaw),
        ]
        self.process = subprocess.Popen(command)
        self.selection = msg
        self.started = time.monotonic()
        self.report(f"Spawning at x={x:.3f}, y={y:.3f}, z={z:.3f}, yaw={yaw:.3f}")

    def poll(self):
        if self.process is None:
            return
        code = self.process.poll()
        if code is None:
            if time.monotonic() - self.started > 60:
                self.process.terminate()
                self.report("Spawn timed out; waiting for create process to exit")
            return
        self.process = None
        if code != 0:
            self.report(f"Gazebo create failed ({code}); select the spawn again")
            return
        self.spawned = True
        self.selection_pub.publish(self.selection)
        self.ready.publish(Bool(data=True))
        self.report(
            "Robot spawned. Global relocalization will start; wait for READY before sending Nav2 Goal"
        )


def main():
    rclpy.init()
    node = SpawnFromRviz()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.process is not None and node.process.poll() is None:
            node.process.terminate()
            try:
                node.process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                node.process.kill()
                node.process.wait(timeout=3)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
