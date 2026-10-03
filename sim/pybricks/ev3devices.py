import math

from world import world
from pybricks.parameters import Color


class Motor:
    def __init__(self, port, positive_direction=1, gears=None):
        self.port = port
        self.sign = positive_direction
        self.target_speed = 0.0   # physical deg/s
        self.phys_speed = 0.0   # actual physical deg/s
        self.phys_angle = 0.0   # physical shaft angle, deg
        self._offset = 0.0
        self.tau_ms = world.MOTOR_TAU_MS
        world.motors[port] = self

    # Pybricks-facing API (all values in the motor's own positive direction)
    def run(self, speed):
        speed = max(-world.MAX_MOTOR_DPS, min(world.MAX_MOTOR_DPS, speed))
        self.target_speed = self.sign * speed
        self.tau_ms = world.MOTOR_TAU_MS

    def stop(self, then=None):
        self.target_speed = 0.0
        self.tau_ms = world.BRAKE_TAU_MS if then in ("brake", "hold") else world.MOTOR_TAU_MS

    def brake(self):
        self.stop("brake")

    def hold(self):
        self.brake()

    def _angle(self):
        return self.sign * (self.phys_angle - self._offset)

    def angle(self):
        return int(self._angle())

    def reset_angle(self, a=0):
        self._offset = self.phys_angle - self.sign * a

    def speed(self):
        return int(self.sign * self.phys_speed)

    def run_angle(self, speed, rotation_angle, then=None, wait=True):
        self.run_target(speed, self._angle() + rotation_angle, then=then, wait=wait)

    def run_target(self, speed, target_angle, then=None, wait=True):
        delta = target_angle - self._angle()
        duration = abs(delta) / max(1.0, abs(speed)) * 1000.0
        world.event("motor_target", (self.port, target_angle))
        if duration <= 0:
            return
        self.target_speed = self.sign * math.copysign(abs(speed), delta)
        self.phys_speed = self.target_speed  # position moves are servo-controlled
        world.advance(duration)
        self.target_speed = 0.0
        self.phys_speed = 0.0
        self.phys_angle = self._offset + self.sign * target_angle


class GyroSensor:
    def __init__(self, port, positive_direction=1):
        self.port = port
        self._offset = 0.0

    def _raw(self):
        return world.heading_cw + world.gyro_bias

    def angle(self):
        return int(round(self._raw() - self._offset))

    def reset_angle(self, a=0):
        self._offset = self._raw() - a
        world.event("gyro_reset", a)

    def speed(self):
        return 0


class ColorSensor:
    def __init__(self, port):
        self.port = port

    def color(self):
        sx, sy = world.sensor_xy()
        on_tape = world.tape is not None and world.tape.contains(sx, sy)
        if on_tape and not world.on_tape:
            world.event("tape", (sx, sy))
        world.on_tape = on_tape
        return Color.BLUE if on_tape else Color.WHITE

    def reflection(self):
        return 10 if self.color() == Color.BLUE else 60


class UltrasonicSensor:
    def __init__(self, port):
        self.port = port

    def distance(self):
        return 2550
