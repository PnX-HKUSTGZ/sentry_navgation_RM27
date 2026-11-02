#!/usr/bin/env python3
"""Two independent ROS services with a feedback-driven folding interlock."""

import json
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String
from std_srvs.srv import SetBool

from gimbal_state import GimbalState


class GimbalControl(Node):
    def __init__(self):
        super().__init__('sim_gimbal_control')
        values = [self.declare_parameter(name, default).value for name, default in
                  [('spin_speed', .30), ('home_speed', .60), ('roll_speed', .65),
                   ('angular_acceleration', 1.0)]]
        self.state = GimbalState(*values)
        self.feedback_timeout = self.declare_parameter('feedback_timeout', .5).value
        self.future_tolerance = self.declare_parameter('feedback_future_tolerance', .05).value
        if (not math.isfinite(self.feedback_timeout) or self.feedback_timeout <= 0 or
                not math.isfinite(self.future_tolerance) or self.future_tolerance < 0):
            raise ValueError('Invalid feedback timestamp tolerances')
        self.feedback = None
        self.feedback_time = None
        self.last_tick = self.get_clock().now()
        self.yaw_pub = self.create_publisher(Float64, '/sim/gimbal/yaw_position', 1)
        self.roll_pub = self.create_publisher(Float64, '/sim/gimbal/roll_position', 1)
        self.status_pub = self.create_publisher(String, '/sim/gimbal/state', 1)
        self.create_subscription(JointState, '/joint_states', self.joints, qos_profile_sensor_data)
        self.create_service(SetBool, '/sim/gimbal/set_spin', self.spin)
        self.create_service(SetBool, '/sim/gimbal/set_upright', self.upright)
        self.create_timer(.02, self.tick)

    def joints(self, msg):
        try:
            yi, ri = msg.name.index('gimbal_yaw_joint'), msg.name.index('gimbal_roll_joint')
            values = (msg.position[yi], msg.position[ri], msg.velocity[yi], msg.velocity[ri])
            if not all(math.isfinite(v) for v in values):
                return
            stamp = rclpy.time.Time.from_msg(msg.header.stamp,
                                           clock_type=self.get_clock().clock_type)
            if (self.feedback_time is not None and stamp <= self.feedback_time and
                    self.get_clock().now() >= self.feedback_time):
                return
            self.feedback, self.feedback_time = values, stamp
        except (ValueError, IndexError):
            return

    def fresh(self):
        return self.feedback_time is not None and -self.future_tolerance <= (
            self.get_clock().now()-self.feedback_time).nanoseconds/1e9 < self.feedback_timeout

    def spin(self, req, response):
        if req.data and not self.fresh():
            response.success, response.message = False, 'No fresh joint feedback'
        else:
            response.success, response.message = self.state.set_spin(req.data)
        return response

    def upright(self, req, response):
        if not self.fresh():
            response.success, response.message = False, 'No fresh joint feedback'
        else:
            self.state.set_upright(req.data)
            response.success, response.message = True, 'Raising' if req.data else 'Homing then folding'
        return response

    def tick(self):
        now = self.get_clock().now()
        dt = (now-self.last_tick).nanoseconds/1e9
        self.last_tick = now
        fresh = self.fresh()
        if self.feedback is not None:
            yaw, roll, yv, rv = self.feedback
            if not fresh or dt < 0:
                self.state.hold(yaw, roll, reset=dt < 0)
            else:
                self.state.update(yaw, roll, yv, rv, dt)
            self.yaw_pub.publish(Float64(data=self.state.yaw_ref))
            self.roll_pub.publish(Float64(data=self.state.roll_ref))
        payload = dict(status=self.state.status, connected=fresh, spinning=self.state.spinning,
                       upright=self.state.upright, want_upright=self.state.want_upright,
                       yaw=self.feedback[0] if self.feedback else 0.0,
                       roll=self.feedback[1] if self.feedback else 0.0)
        self.status_pub.publish(String(data=json.dumps(payload)))


def main():
    rclpy.init()
    node = GimbalControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
