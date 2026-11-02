"""Physical Gazebo opponent and independent truth checks for navigation tests.

Ground truth is used only by this test monitor; the navigation detector receives
registered LiDAR and the fixed prior point cloud, never these actor poses.
"""
import json
import os
import math
import subprocess
import time
import threading
from pathlib import Path
import numpy as np
import yaml
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from rclpy.qos import qos_profile_sensor_data


class VehicleScenario:
    def __init__(self, node, name):
        self.node, self.name = node, name
        self.other = None
        self.detector = {}
        self.active = False
        self.started = None
        self.samples = []
        self.identified_frames = 0
        self.fresh_detection_frames = 0
        self.detection_samples = []
        self.created_stamp = None
        self.spawn_error = None
        self.unvalidated_after_spawn = 0
        self.detector_frames = 0
        self.command = (0.0, 0.0)
        self.spin_rates = []
        self.last_yaw = None
        self.bridge = None
        self.expected_spin = float(yaml.safe_load(Path('src/pnx-stimulation/rm_27_stimulation/config/gimbal_control.yaml').read_text())['sim_gimbal_control']['ros__parameters']['spin_speed'])
        self.known_at_start = False
        self.first_observed_stamp = None
        self.spawn_xy = {'parked': (-9.7, 3.2), 'crossing': (-9.7, 2.2),
                         'oncoming': (-8.6, 3.2), 'stop': (-9.7, 2.2),
                         'late_crossing': (-9.7, 2.2)}[name]
        params = yaml.safe_load(Path(os.environ.get('NAV_VEHICLE_PARAMS', 'src/pnx-launch/launch_bringup/config/simulation/minco_params.yaml')).read_text())
        self.footprint = np.array(params['planner_server']['ros__parameters']['MincoPlanner']['safety']['footprint_points']).reshape(-1, 2)
        self.other_footprint = np.array([[-.30, -.26], [.30, -.26], [.30, .26], [-.30, .26]])
        self.pub = node.create_publisher(Twist, '/validation_vehicle/cmd_vel', 1)
        self.subs = [node.create_subscription(Odometry, '/validation_vehicle/odometry', self.opponent, qos_profile_sensor_data),
                     node.create_subscription(String, '/rog_map/vehicle_obstacles', self.detection, 1),
                     node.create_subscription(String, '/sim/gimbal/state', self.gimbal, 1)]
        self.timer = node.create_timer(.05, self.tick)

    def time(self):
        return self.node.get_clock().now().nanoseconds * 1e-9

    @staticmethod
    def yaw(q):
        return math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))

    def opponent(self, msg):
        self.other = msg

    def detection(self, msg):
        self.detector = json.loads(msg.data)
        if self.active:
            self.detector_frames += 1
            if self.other:
                p = self.other.pose.pose.position
                matches = [t for t in self.detector.get('tracks', []) if math.hypot(t['x']-p.x, t['y']-p.y) < .7]
                self.identified_frames += bool(matches)
                fresh = any(t['age'] < .3 for t in matches)
                self.fresh_detection_frames += fresh
                if fresh and self.first_observed_stamp is None:
                    self.first_observed_stamp = self.detector['stamp']
            self.detection_samples.append(self.detector)

    def gimbal(self, msg):
        value = json.loads(msg.data)
        now = self.time()
        if value.get('spinning') and value.get('upright') and self.last_yaw:
            dt = now - self.last_yaw[0]
            if dt > .005:
                diff = value['yaw'] - self.last_yaw[1]
                self.spin_rates.append(math.atan2(math.sin(diff), math.cos(diff)) / dt)
        self.last_yaw = (now, value['yaw'])

    def tick(self):
        vx = vy = 0.0
        if self.active and self.other:
            p = self.other.pose.pose.position
            if self.name in ('crossing', 'late_crossing') and p.y < 3.65:
                vy = .6
            elif self.name == 'stop' and p.y < 3.2:
                vy = .6
            elif self.name == 'oncoming' and p.x > -11.8:
                vx = -.6
        command = Twist()
        command.linear.x, command.linear.y = vx, vy
        self.pub.publish(command)
        self.command = (vx, vy)

    def start_bridge(self):
        if self.bridge is None:
            self.bridge = subprocess.Popen(['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                '/validation_vehicle/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                '/validation_vehicle/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry'],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def spawn(self):
        self.start_bridge()
        if self.name != 'late_crossing':
            self.spawn_now()

    def spawn_now(self):
        try:
            self.create_actor()
        except Exception as exc:
            self.spawn_error = str(exc)

    def create_actor(self):
        x, y = self.spawn_xy
        wheels = ''.join(f'''<link name="wheel_{i}"><pose>{wx} {wy} -.08 0 0 0</pose>
<inertial><mass>.1</mass><inertia><ixx>.000256</ixx><iyy>.000256</iyy><izz>.000256</izz></inertia></inertial>
<collision name="wheel_collision"><geometry><sphere><radius>.08</radius></sphere></geometry><surface><friction><ode><mu>.02</mu><mu2>.02</mu2></ode></friction></surface></collision>
<visual name="wheel_visual"><geometry><sphere><radius>.08</radius></sphere></geometry><material><ambient>.03 .03 .03 1</ambient><diffuse>.03 .03 .03 1</diffuse></material></visual></link>
<joint name="wheel_joint_{i}" type="ball"><parent>body</parent><child>wheel_{i}</child></joint>'''
                          for i, (wx, wy) in enumerate([(-.22, -.18), (-.22, .18), (.22, -.18), (.22, .18)]))
        sdf = f'''<sdf version="1.9"><model name="validation_vehicle"><pose>{x} {y} .16 0 0 0</pose><link name="body">
<inertial><mass>5</mass><inertia><ixx>.10</ixx><iyy>.16</iyy><izz>.24</izz></inertia></inertial>
<collision name="chassis"><geometry><box><size>.60 .46 .16</size></box></geometry><surface><friction><ode><mu>.02</mu><mu2>.02</mu2></ode></friction></surface></collision>
<visual name="chassis_visual"><geometry><box><size>.60 .46 .16</size></box></geometry><material><ambient>.8 .03 .03 1</ambient><diffuse>.8 .03 .03 1</diffuse></material></visual></link>{wheels}
<plugin filename="librm27_cmd_vel_pose_control_system.so" name="rm_27_stimulation::CmdVelPoseControlSystem"><topic>/validation_vehicle/cmd_vel</topic><body_frame>false</body_frame><command_timeout>.3</command_timeout><linear_velocity_gain>25</linear_velocity_gain><linear_velocity_integral_gain>15</linear_velocity_integral_gain><angular_velocity_gain>8</angular_velocity_gain><max_planar_force>35</max_planar_force><max_planar_integral_force>15</max_planar_integral_force><max_yaw_torque>8</max_yaw_torque></plugin>
<plugin filename="librm27_ground_truth_odometry_system.so" name="rm_27_stimulation::GroundTruthOdometrySystem"><odom_topic>/validation_vehicle/odometry</odom_topic><robot_base_frame>validation_vehicle</robot_base_frame><odom_publish_frequency>50</odom_publish_frequency></plugin>
</model></sdf>'''
        made = subprocess.run(['gz', 'service', '-s', '/world/default/create', '--reqtype', 'gz.msgs.EntityFactory', '--reptype', 'gz.msgs.Boolean', '--timeout', '4000', '--req', 'sdf: '+json.dumps(sdf)], capture_output=True, text=True, timeout=6)
        if 'data: true' not in made.stdout:
            raise RuntimeError('Opponent spawn failed: '+made.stdout+made.stderr)
        self.created_stamp = self.time()

    def begin(self):
        if (self.other is None and self.name != 'late_crossing') or not self.detector.get('ready'):
            raise RuntimeError('Opponent truth or vehicle detector is not ready')
        self.samples = []
        self.identified_frames = self.detector_frames = self.fresh_detection_frames = 0
        self.detection_samples = []
        self.known_at_start = self.other is not None and any(
            math.hypot(t['x'] - self.other.pose.pose.position.x, t['y'] - self.other.pose.pose.position.y) < .7
            for t in self.detector.get('tracks', []))
        self.first_observed_stamp = None
        self.active = True
        if self.name == 'late_crossing':
            threading.Thread(target=self.spawn_now, daemon=True).start()
        self.started = self.time()

    def record(self, own):
        if not self.active:
            return
        if self.other is None:
            if self.created_stamp is not None and self.time() - self.created_stamp > .15:
                self.unvalidated_after_spawn += 1
            return
        other = self.other
        stamp = own.header.stamp.sec + own.header.stamp.nanosec*1e-9
        other_stamp = other.header.stamp.sec + other.header.stamp.nanosec*1e-9
        p, q = own.pose.pose.position, other.pose.pose.position
        a, b = self.yaw(own.pose.pose.orientation), self.yaw(other.pose.pose.orientation)
        def vertices(poly, yaw, xy):
            rot = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
            return poly @ rot.T + xy
        own_poly = vertices(self.footprint, a, [p.x, p.y])
        other_poly = vertices(self.other_footprint, b, [q.x, q.y])
        gaps = []
        for poly in (own_poly, other_poly):
            for edge in np.roll(poly, -1, axis=0) - poly:
                axis = np.array([-edge[1], edge[0]]) / np.linalg.norm(edge)
                u, v = own_poly @ axis, other_poly @ axis
                gaps.append(max(min(u)-max(v), min(v)-max(u)))
        vel = other.twist.twist.linear
        quat = other.pose.pose.orientation
        roll = math.atan2(2*(quat.w*quat.x+quat.y*quat.z),1-2*(quat.x*quat.x+quat.y*quat.y))
        pitch = math.asin(max(-1., min(1., 2*(quat.w*quat.y-quat.z*quat.x))))
        self.samples.append([stamp, p.x, p.y, q.x, q.y, max(gaps),
                             abs(stamp-other_stamp), math.hypot(vel.x, vel.y),
                             q.z, roll, pitch])

    def result(self):
        self.active = False
        self.tick()
        samples = np.asarray(self.samples)
        fresh = samples[samples[:, 6] < .15] if len(samples) else samples
        rate = float(np.median(self.spin_rates[-200:])) if self.spin_rates else None
        collision_samples = int((fresh[:, 5] < -.005).sum()) if len(fresh) else None
        closest_time = float(fresh[np.argmin(fresh[:, 5]), 0]) if len(fresh) else None
        detected_in_time = self.known_at_start or (self.first_observed_stamp is not None and
            closest_time is not None and self.first_observed_stamp < closest_time)
        travel = float(np.linalg.norm(np.diff(samples[:, 3:5], axis=0), axis=1).sum()) if len(samples) else 0.0
        max_tilt = float(np.abs(samples[:, 9:11]).max()) if len(samples) else float('inf')
        max_speed = float(samples[:, 7].max()) if len(samples) else float('inf')
        stopped_after_motion = 0.0
        moving_seen = False
        stop_start = None
        for row in samples:
            moving_seen = moving_seen or row[7] > .2
            if moving_seen and row[7] < .05:
                if stop_start is None: stop_start = row[0]
                stopped_after_motion = max(stopped_after_motion, row[0] - stop_start)
            else: stop_start = None
        moved = len(samples) and (max_speed < .05 if self.name == 'parked' else travel > .6 and max_speed > .2)
        actor_valid = bool(moved and max_speed < 1.25 and max_tilt < .15 and
                           (self.name != 'stop' or stopped_after_motion > 1.0))
        passed = bool(collision_samples == 0 and len(fresh) >= 50 and
            len(fresh) >= .95 * len(samples) and self.unvalidated_after_spawn == 0 and
            self.fresh_detection_frames > 0 and detected_in_time and actor_valid and
            rate is not None and abs(rate - self.expected_spin) < .1 * abs(self.expected_spin))
        return dict(scenario=self.name, samples=self.samples, passed=passed,
                    route_start_stamp=self.started, actor_valid=actor_valid,
                    actor_dimensions_m=[.60,.52,.24], actor_travel_m=travel,
                    actor_max_tilt_rad=max_tilt, actor_stopped_after_motion_s=stopped_after_motion,
                    known_at_start=self.known_at_start, first_observed_stamp=self.first_observed_stamp,
                    detected_before_closest_approach=detected_in_time,
                    truth_samples=len(samples), synchronized_samples=len(fresh),
                    collision_samples=int((fresh[:, 5] < -.005).sum()) if len(fresh) else None,
                    minimum_separation_m=float(fresh[:, 5].min()) if len(fresh) else None,
                    opponent_max_speed_mps=float(samples[:, 7].max()) if len(samples) else None,
                    detector_frames=self.detector_frames, identified_frames=self.identified_frames,
                    fresh_detection_frames=self.fresh_detection_frames, detection_samples=self.detection_samples,
                    unvalidated_after_spawn=self.unvalidated_after_spawn, spawn_error=self.spawn_error,
                    measured_gimbal_rate_median_rad_s=rate, expected_gimbal_rate_rad_s=self.expected_spin)

    def close(self):
        self.active = False
        self.tick()
        self.timer.cancel()
        if self.bridge is not None:
            self.bridge.terminate()
            try:
                self.bridge.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.bridge.kill()
