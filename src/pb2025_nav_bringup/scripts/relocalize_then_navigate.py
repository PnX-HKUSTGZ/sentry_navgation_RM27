#!/usr/bin/env python3
"""Sequence map, RViz placement, global pose handoff, then Nav2 activation."""
import json
import math
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from global_relocalization_interfaces.action import Relocalize
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
from nav2_msgs.srv import ManageLifecycleNodes
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import Imu, PointCloud2
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener, TransformException


def planar_yaw(q):
    return math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))


def rotation_clear(grid, x, y, radius=0.45):
    """Conservative swept disk, including every cell touched by the chassis.

    The RM27 padded polygon fits within this disk at every yaw. Unknown,
    outside-map and occupied cells cannot authorize the startup rotation.
    """
    r = grid.info.resolution
    if not math.isfinite(r) or r <= 0 or not all(math.isfinite(v) for v in (x, y)):
        return False
    a = planar_yaw(grid.info.origin.orientation)
    dx, dy = x-grid.info.origin.position.x, y-grid.info.origin.position.y
    gx, gy = (math.cos(a)*dx+math.sin(a)*dy)/r, (-math.sin(a)*dx+math.cos(a)*dy)/r
    extent = radius/r + math.sqrt(.5)
    for row in range(math.floor(gy-extent), math.ceil(gy+extent)+1):
        for col in range(math.floor(gx-extent), math.ceil(gx+extent)+1):
            if math.hypot(col+.5-gx, row+.5-gy) > extent:
                continue
            if not (0 <= col < grid.info.width and 0 <= row < grid.info.height):
                return False
            index = row*grid.info.width+col
            if index >= len(grid.data) or grid.data[index] != 0:
                return False
    return True


class Startup(Node):
    def __init__(self):
        super().__init__('relocalize_then_navigate')
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(String, '~/status', qos)
        self.command_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.command_sub = self.create_subscription(Twist, '/relocalization/cmd_vel', self.command, 10)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/relocalization/initialpose', 10)
        self.create_subscription(PoseWithCovarianceStamped, '/relocalization/initialpose_accepted', self.accepted, qos)
        self.create_subscription(Bool, '/simulation/spawned', self.spawn, qos)
        self.create_subscription(PoseWithCovarianceStamped, '/simulation/spawn_pose', self.selection, qos)
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_received, qos)
        self.odom_sub = self.create_subscription(Odometry, '/odometry', self.odometry, qos_profile_sensor_data)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.map = None
        self.odom = None
        self.odom_ready_since = None
        self.selected_yaw = None
        self.aligned_since = None
        self.sensor_subs = [
            self.create_subscription(PointCloud2, '/livox/lidar', lambda _: self.sensor('lidar'), qos_profile_sensor_data),
            self.create_subscription(Imu, '/livox/imu', lambda _: self.sensor('imu'), qos_profile_sensor_data),
        ]
        self.map_state = self.create_client(GetState, '/map_server/get_state')
        self.map_change = self.create_client(ChangeState, '/map_server/change_state')
        self.nav_client = self.create_client(ManageLifecycleNodes, '/lifecycle_manager_navigation/manage_nodes')
        self.trigger = self.create_client(Trigger, '/global_relocalization/start_match')
        self.action = ActionClient(self, Relocalize, '/global_relocalization/run')
        self.state = None
        self.started = time.monotonic()
        self.state_since = self.started
        self.sensor_times = {}
        self.spawned = False
        self.last_command = 0.0
        self.future = None
        self.map_step = 'get'
        self.goal = None
        self.armed = False
        self.triggered = False
        self.seed = None
        self.set_state('STARTING_MAP')
        self.timer = self.create_timer(0.1, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def set_state(self, state, reason=''):
        self.state = state
        self.state_since = time.monotonic()
        payload = {'state': state, 'reason': reason, 'elapsed_sec': self.state_since - self.started}
        self.status_pub.publish(String(data=json.dumps(payload)))
        self.get_logger().info(f'{state}: {reason}')

    def fail(self, reason):
        if self.state != 'FAILED':
            if self.command_pub:
                self.command_pub.publish(Twist())
            if self.goal:
                self.goal.cancel_goal_async()
            self.set_state('FAILED', reason)

    def sensor(self, name):
        self.sensor_times[name] = time.monotonic()

    def spawn(self, msg):
        self.spawned = msg.data

    def selection(self, msg):
        q = msg.pose.pose.orientation
        if (self.selected_yaw is None and msg.header.frame_id == 'map' and
                all(math.isfinite(v) for v in (q.x, q.y, q.z, q.w)) and
                abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1.) < .02):
            # Heading is an operator request, never a localization seed.
            self.selected_yaw = planar_yaw(q)

    def map_received(self, msg):
        if msg.header.frame_id == 'map':
            self.map = msg

    def odometry(self, msg):
        now = time.monotonic()
        if now - self.sensor_times.get('odom', 0) > 0.5:
            self.odom_ready_since = now
        self.sensor_times['odom'] = now
        self.odom = msg

    def command(self, msg):
        if self.state != 'RELOCALIZING' or not self.command_pub:
            return
        values = (msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y, msg.angular.z)
        if not all(math.isfinite(v) for v in values):
            self.fail('Non-finite relocalization velocity')
            return
        self.command_pub.publish(msg)
        self.last_command = time.monotonic()

    def accepted(self, msg):
        if self.state != 'HANDOFF' or self.seed is None or msg != self.seed:
            return
        self.command_pub.publish(Twist())
        if self.selected_yaw is None:
            self.fail('Missing RViz spawn heading; cannot complete heading alignment')
            return
        self.set_state('ALIGNING_HEADING',
                       f'localized_yaw={planar_yaw(msg.pose.pose.orientation):.6f} '
                       f'target_yaw={self.selected_yaw:.6f}')

    def release_control(self):
        # The global localizer only publishes to the private input. Release the
        # sole bridge before Nav2 may begin publishing chassis commands.
        self.command_pub.publish(Twist())
        self.destroy_subscription(self.command_sub)
        self.destroy_publisher(self.command_pub)
        self.command_sub = self.command_pub = None
        for subscription in self.sensor_subs + [self.map_sub, self.odom_sub]:
            self.destroy_subscription(subscription)
        self.sensor_subs.clear()
        self.future = None
        self.set_state('STARTING_NAVIGATION')

    def align_heading(self):
        # This uses the confirmed map transform and normal relative odometry;
        # it neither reads Gazebo truth nor resets the accepted localization.
        try:
            tf = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
        except TransformException:
            self.command_pub.publish(Twist())
            self.aligned_since = None
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        age = lambda stamp: now - (stamp.sec + stamp.nanosec*1e-9)
        steady = time.monotonic()
        if (not self.odom or not -.05 <= age(self.odom.header.stamp) <= .25 or
                not -.05 <= age(tf.header.stamp) <= .25 or
                any(steady-self.sensor_times.get(k, 0) > .5 for k in ('lidar', 'imu'))):
            self.command_pub.publish(Twist())
            self.aligned_since = None
            return
        p, q = tf.transform.translation, tf.transform.rotation
        start = self.seed.pose.pose.position
        if (not self.map or not rotation_clear(self.map, p.x, p.y) or
                math.hypot(p.x-start.x, p.y-start.y) > .10):
            self.fail('Heading alignment has insufficient swept clearance or position drift')
            return
        velocity = self.odom.twist.twist
        if (not all(math.isfinite(v) for v in
                    (q.x, q.y, q.z, q.w, velocity.linear.x, velocity.linear.y, velocity.angular.z)) or
                abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1.) > 1e-3):
            self.fail('Heading alignment received invalid orientation or velocity')
            return
        error = math.atan2(math.sin(self.selected_yaw-planar_yaw(q)),
                           math.cos(self.selected_yaw-planar_yaw(q)))
        command = Twist()
        if abs(error) > .01:
            command.angular.z = max(-.6, min(.6, 1.5*error))
            self.aligned_since = None
        elif (math.hypot(velocity.linear.x, velocity.linear.y) < .03 and
              abs(velocity.angular.z) < .03):
            if self.aligned_since is None:
                self.aligned_since = now
            elif now-self.aligned_since >= .3:
                self.get_logger().info(f'Heading aligned and stopped: yaw_error={error:.6f} rad')
                self.release_control()
                return
        else:
            self.aligned_since = None
        self.command_pub.publish(command)
        self.last_command = steady

    def feedback(self, msg):
        if msg.feedback.stage == 'ARMED':
            self.armed = True

    def goal_response(self, future):
        try:
            self.goal = future.result()
            if not self.goal.accepted:
                self.fail('Global relocalization rejected the request')
                return
            self.goal.get_result_async().add_done_callback(self.result)
        except Exception as exc:
            self.fail(f'Global relocalization request failed: {exc}')

    def result(self, future):
        if self.state != 'RELOCALIZING':
            return
        try:
            reply = future.result()
            result = reply.result
            if reply.status != GoalStatus.STATUS_SUCCEEDED or not result.success:
                self.fail(f'Global relocalization failed: {result.reason}')
                return
            pose = result.pose
            p, q = pose.pose.position, pose.pose.orientation
            if (pose.header.frame_id != 'map' or
                    pose.header.stamp.sec == 0 and pose.header.stamp.nanosec == 0 or
                    not all(math.isfinite(v) for v in (p.x, p.y, p.z, q.x, q.y, q.z, q.w)) or
                    abs(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w - 1.0) > 1e-3):
                self.fail('Global relocalization returned an invalid pose')
                return
            self.seed = PoseWithCovarianceStamped()
            self.seed.header = pose.header
            self.seed.pose.pose = pose.pose
            self.command_pub.publish(Twist())
            self.set_state('HANDOFF', f'fitness={result.fitness:.4f} inlier_ratio={result.inlier_ratio:.3f} '
                           f'xyz=({p.x:.6f},{p.y:.6f},{p.z:.6f}) '
                           f'quaternion=({q.x:.8f},{q.y:.8f},{q.z:.8f},{q.w:.8f}) '
                           f'stamp={pose.header.stamp.sec}.{pose.header.stamp.nanosec:09d}')
        except Exception as exc:
            self.fail(f'Global relocalization result error: {exc}')

    def lifecycle(self, client, next_state):
        if self.future is None:
            if client.service_is_ready():
                self.future = client.call_async(ManageLifecycleNodes.Request(command=ManageLifecycleNodes.Request.STARTUP))
        elif self.future.done():
            try:
                response = self.future.result()
                if not response or not response.success:
                    self.fail('Lifecycle startup failed')
                    return
                self.future = None
                self.set_state(next_state)
            except Exception as exc:
                self.fail(f'Lifecycle startup error: {exc}')

    def start_map(self):
        # Start the map server directly. Nav2's localization lifecycle manager
        # can wait indefinitely for its own service clients during discovery;
        # map_server exposes its lifecycle services independently. Re-read the
        # state after every transition, including an already-active server.
        if self.future is None:
            if self.map_state.service_is_ready():
                self.map_step = 'get'
                self.future = self.map_state.call_async(GetState.Request())
            return
        if not self.future.done():
            return
        try:
            response = self.future.result()
            self.future = None
            if self.map_step != 'get':
                if not response or not response.success:
                    self.get_logger().warning('Map transition not accepted; rechecking its state')
                return
            if not response:
                self.fail('Map server returned no lifecycle state')
                return
            state = response.current_state.id
            if state == State.PRIMARY_STATE_ACTIVE:
                self.set_state('WAITING_FOR_INITIAL_POSE')
            elif state in (State.PRIMARY_STATE_UNCONFIGURED, State.PRIMARY_STATE_INACTIVE):
                if self.map_change.service_is_ready():
                    request = ChangeState.Request()
                    request.transition.id = (Transition.TRANSITION_CONFIGURE if
                                             state == State.PRIMARY_STATE_UNCONFIGURED else
                                             Transition.TRANSITION_ACTIVATE)
                    self.map_step = 'change'
                    self.future = self.map_change.call_async(request)
            else:
                self.get_logger().warning(f'Waiting for map lifecycle state to settle: {state}')
        except Exception as exc:
            self.fail(f'Map lifecycle error: {exc}')

    def tick(self):
        now = time.monotonic()
        if self.command_pub and now - self.last_command > 0.5:
            self.command_pub.publish(Twist())
        if self.state == 'STARTING_MAP':
            self.start_map()
        elif self.state == 'WAITING_FOR_INITIAL_POSE' and self.spawned:
            self.set_state('WAITING_FOR_SENSORS')
        elif self.state == 'WAITING_FOR_SENSORS':
            if (self.selected_yaw is not None and
                    all(now - self.sensor_times.get(k, 0) < 0.5 for k in ('lidar', 'imu', 'odom')) and
                    self.odom_ready_since is not None and now - self.odom_ready_since >= 1.0 and
                    self.action.server_is_ready() and self.trigger.service_is_ready()):
                self.set_state('RELOCALIZING')
                self.action.send_goal_async(Relocalize.Goal(), feedback_callback=self.feedback).add_done_callback(self.goal_response)
        elif self.state == 'RELOCALIZING' and self.armed and not self.triggered and self.trigger.service_is_ready():
            self.triggered = True
            def finished(future):
                try:
                    response = future.result()
                    if not response or not response.success:
                        self.fail('Start-match trigger failed: ' + (response.message if response else 'no response'))
                except Exception as exc:
                    self.fail(f'Start-match trigger error: {exc}')
            self.trigger.call_async(Trigger.Request()).add_done_callback(finished)
        elif self.state == 'HANDOFF':
            self.pose_pub.publish(self.seed)
        elif self.state == 'ALIGNING_HEADING':
            self.align_heading()
        elif self.state == 'STARTING_NAVIGATION':
            self.lifecycle(self.nav_client, 'READY')
        if self.state not in ('READY', 'FAILED', 'WAITING_FOR_INITIAL_POSE') and now - self.state_since > (180 if self.state == 'RELOCALIZING' else 60):
            self.fail(f'Timeout in {self.state}')


def main():
    rclpy.init()
    node = Startup()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.command_pub:
            node.command_pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
