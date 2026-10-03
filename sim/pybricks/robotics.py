import math

from world import world


class DriveBase:
    def __init__(self, left_motor, right_motor, wheel_diameter, axle_track):
        self.left, self.right = left_motor, right_motor
        world.wheel_diameter = wheel_diameter
        world.axle_track = axle_track
        world.drive_ports = (left_motor.port, right_motor.port)
        self._settings = dict(straight_speed=200, turn_rate=100)
        self.reset()

    def settings(self, straight_speed=None, straight_acceleration=None,
                 turn_rate=None, turn_acceleration=None):
        if straight_speed:
            self._settings["straight_speed"] = straight_speed
        if turn_rate:
            self._settings["turn_rate"] = turn_rate

    def _deg_per_mm(self):
        return 360.0 / (math.pi * world.wheel_diameter)

    def reset(self):
        self._l0 = self.left._angle()
        self._r0 = self.right._angle()

    def distance(self):
        dl = self.left._angle() - self._l0
        dr = self.right._angle() - self._r0
        return int((dl + dr) / 2.0 / self._deg_per_mm())

    def drive(self, speed, turn_rate):
        # turn_rate > 0 means clockwise (right) -- left wheel goes faster.
        diff_mm = math.radians(turn_rate) * world.axle_track / 2.0
        self.left.run((speed + diff_mm) * self._deg_per_mm())
        self.right.run((speed - diff_mm) * self._deg_per_mm())

    def stop(self):
        self.left.stop()
        self.right.stop()

    # straight() and turn() are closed-loop moves on the real hub: they slow
    # down near the target and stop on it. Mimic that with a simple
    # proportional ramp so the simulated robot does not overshoot.
    def straight(self, distance):
        start = self.distance()
        while True:
            remaining = abs(distance) - abs(self.distance() - start)
            if remaining <= 1:
                break
            speed = min(self._settings["straight_speed"], max(40.0, 4.0 * remaining))
            self.drive(math.copysign(speed, distance), 0)
            world.advance(10)
        self.stop()
        self.left.brake(); self.right.brake()
        world.advance(100)

    def turn(self, angle):
        start = world.heading_cw
        while True:
            remaining = abs(angle) - abs(world.heading_cw - start)
            if remaining <= 0.5:
                break
            rate = min(self._settings["turn_rate"], max(20.0, 4.0 * remaining))
            self.drive(0, math.copysign(rate, angle))
            world.advance(10)
        self.stop()
        self.left.brake(); self.right.brake()
        world.advance(100)
