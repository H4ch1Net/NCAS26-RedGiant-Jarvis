"""Virtual world for the Jarvis simulator.

Holds the simulated clock, the robot pose and the motor states. The mock
``pybricks`` package in this folder reads and writes this module, so the
real competition programs can run unchanged on a normal computer.

Conventions (chosen to match Pybricks on the EV3):
  * World frame: +x = east, +y = north, millimetres.
  * ``heading_cw`` is the robot heading in degrees measured clockwise from
    +x, which is what the EV3 gyro reports (clockwise = positive).
  * Motor speeds are in degrees/second of wheel rotation.
"""
import math


class SimTimeout(Exception):
    """Raised by wait() when the simulated run hits its time limit."""


class Tape:
    """Blue boundary tape: a band of ``width`` mm running around a rectangle."""

    def __init__(self, x0, y0, x1, y1, width=50):
        self.x0, self.y0, self.x1, self.y1, self.width = x0, y0, x1, y1, width

    def contains(self, x, y):
        w = self.width
        inside_outer = self.x0 - w <= x <= self.x1 + w and self.y0 - w <= y <= self.y1 + w
        inside_inner = self.x0 < x < self.x1 and self.y0 < y < self.y1
        return inside_outer and not inside_inner


class World:
    DT_MS = 5            # integration step
    MOTOR_TAU_MS = 60    # first-order lag between commanded and actual speed
    BRAKE_TAU_MS = 30    # braking stops the wheels faster than coasting
    MAX_MOTOR_DPS = 1000 # EV3 large motor top speed, roughly

    def __init__(self):
        self.reset()

    def reset(self, x=0.0, y=0.0, heading_cw=0.0, wheel_diameter=57.0,
                axle_track=134.0, tape=None, sensor_offset_mm=70.0,
                right_wheel_scale=1.0, gyro_drift_dps=0.0, time_limit_ms=None,
                forward_sign=None, drive_ports=None):
        self.t_ms = 0
        self.x, self.y, self.heading_cw = x, y, heading_cw
        self.wheel_diameter = wheel_diameter
        self.axle_track = axle_track
        self.tape = tape
        self.sensor_offset_mm = sensor_offset_mm
        # <1.0 makes the right wheel slightly weaker, which pulls the robot
        # off course -- the kind of drift the gyro loop has to fight.
        self.right_wheel_scale = right_wheel_scale
        self.gyro_drift_dps = gyro_drift_dps
        self.gyro_bias = 0.0
        self.time_limit_ms = time_limit_ms
        # Per-port multiplier mapping a motor's *configured* positive direction
        # to "forward". The competition chassis needs none; the dance demos
        # declare the left motor COUNTERCLOCKWISE, so they pass {"D": -1}.
        self.forward_sign = forward_sign or {}
        self.on_tape = False
        self.motor_log = []       # (t, l_target, l_speed, r_target, r_speed, l_angle, r_angle)
        self.globals = {}
        self.motors = {}          # port -> SimMotor state
        # (left_port, right_port). A DriveBase sets this itself; programs that
        # drive the wheel motors directly (the dance demos) pass it in.
        self.drive_ports = drive_ports
        self.trace = []           # (t, x, y, heading_cw)
        self.events = []          # (t, kind, payload)
        self.label = ""

    # ---- helpers -------------------------------------------------------
    def mm_per_deg(self):
        return math.pi * self.wheel_diameter / 360.0

    def sensor_xy(self):
        h = math.radians(-self.heading_cw)
        return (self.x + self.sensor_offset_mm * math.cos(h),
                self.y + self.sensor_offset_mm * math.sin(h))

    def event(self, kind, payload=None):
        self.events.append((self.t_ms, kind, payload))

    # ---- physics -------------------------------------------------------
    def advance(self, ms):
        steps = max(1, int(round(ms / self.DT_MS)))
        dt = ms / steps
        for _ in range(steps):
            self._step(dt)
        if self.time_limit_ms is not None and self.t_ms >= self.time_limit_ms:
            raise SimTimeout()

    def _step(self, dt_ms):
        for m in self.motors.values():
            alpha = 1.0 - math.exp(-dt_ms / m.tau_ms)
            m.phys_speed += (m.target_speed - m.phys_speed) * alpha
            m.phys_angle += m.phys_speed * dt_ms / 1000.0

        if self.drive_ports:
            lp, rp = self.drive_ports
            ml, mr = self.motors[lp], self.motors[rp]
            vl = ml.phys_speed * self.forward_sign.get(lp, 1) * self.mm_per_deg()
            vr = mr.phys_speed * self.forward_sign.get(rp, 1) * self.mm_per_deg() * self.right_wheel_scale
            v = (vl + vr) / 2.0
            omega_cw = math.degrees((vl - vr) / self.axle_track)  # deg/s
            dt = dt_ms / 1000.0
            h = math.radians(-(self.heading_cw + omega_cw * dt / 2))
            self.x += v * math.cos(h) * dt
            self.y += v * math.sin(h) * dt
            self.heading_cw += omega_cw * dt
            self.gyro_bias += self.gyro_drift_dps * dt
            self.motor_log.append((self.t_ms + dt_ms, ml.target_speed, ml.phys_speed,
                                   mr.target_speed, mr.phys_speed, ml.phys_angle, mr.phys_angle))

        self.t_ms += dt_ms
        self.trace.append((self.t_ms, self.x, self.y, self.heading_cw))


world = World()
