"""Feedback-driven yaw/roll sequencing, independent of ROS and the GUI."""

import math


def angle_error(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class GimbalState:
    def __init__(self, spin_speed=.30, home_speed=.60, roll_speed=.65, acceleration=1.0):
        values = (spin_speed, home_speed, roll_speed, acceleration)
        if not all(math.isfinite(v) and v > 0 for v in values):
            raise ValueError('gimbal speeds and acceleration must be finite and positive')
        self.spin_speed, self.home_speed, self.roll_speed, self.acceleration = values
        self.want_upright = False
        self.spinning = False
        self.initialized = False
        self.yaw_ref = self.roll_ref = self.yaw_rate = self.roll_rate = 0.0
        self.upright = False
        self.status = 'Waiting for joint feedback'

    def set_spin(self, enabled):
        if enabled and (not self.upright or not self.want_upright):
            return False, 'Raise the head and wait until it is upright'
        self.spinning = enabled
        return True, 'Rotation enabled' if enabled else 'Returning yaw to forward'

    def set_upright(self, enabled):
        self.want_upright = enabled
        if not enabled:
            self.spinning = False

    def hold(self, yaw, roll, reset=False):
        if reset:
            self.spinning = False
            self.want_upright = False
        self.yaw_ref, self.roll_ref = yaw, roll
        self.yaw_rate = self.roll_rate = 0.0
        self.upright = False
        self.status = 'Waiting for fresh joint feedback'

    def _step(self, position, rate, target, speed, dt):
        error = target - position
        wanted = math.copysign(min(speed, math.sqrt(2*self.acceleration*abs(error))), error)
        rate += max(-self.acceleration*dt, min(self.acceleration*dt, wanted-rate))
        step = rate*dt
        if step*error >= 0 and abs(step) >= abs(error):
            return target, 0.0
        return position+step, rate

    def update(self, yaw, roll, yaw_velocity, roll_velocity, dt):
        if not all(math.isfinite(v) for v in (yaw, roll, yaw_velocity, roll_velocity, dt)):
            raise ValueError('non-finite joint feedback')
        if not self.initialized:
            self.yaw_ref, self.roll_ref = yaw, roll
            self.initialized = True
        dt = max(0.0, min(dt, .05))
        home = abs(angle_error(yaw)) < .025 and abs(yaw_velocity) < .05
        self.upright = abs(roll-math.pi/2) < .025 and abs(roll_velocity) < .05
        folded = abs(roll) < .025 and abs(roll_velocity) < .05

        if self.spinning and self.want_upright and self.upright:
            self.yaw_rate = min(self.spin_speed, self.yaw_rate+self.acceleration*dt)
            self.yaw_ref = min(self.yaw_ref+self.yaw_rate*dt, yaw+.20)
            self.status = 'Rotating'
        else:
            # Use the nearest equivalent zero, even after many revolutions.
            target = yaw-angle_error(yaw)
            self.yaw_ref, self.yaw_rate = self._step(
                self.yaw_ref, self.yaw_rate, target, self.home_speed, dt)
            self.status = 'Returning to forward' if not home else 'Upright' if self.upright else 'Folded'

        target_roll = math.pi/2 if self.want_upright else 0.0
        if home or (self.want_upright and self.upright):
            self.roll_ref, self.roll_rate = self._step(
                self.roll_ref, self.roll_rate, target_roll, self.roll_speed, dt)
            if not self.want_upright and not folded:
                self.status = 'Folding'
            elif self.want_upright and not self.upright:
                self.status = 'Raising'
        else:
            # Do not start folding while yaw is still moving or off-centre.
            self.roll_rate = 0.0
        self.roll_ref = max(0.0, min(math.pi/2, self.roll_ref))
        return self.yaw_ref, self.roll_ref
