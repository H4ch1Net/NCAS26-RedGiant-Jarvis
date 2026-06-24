#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, GyroSensor, ColorSensor
from pybricks.parameters import Port, Stop, Color
from pybricks.robotics import DriveBase
from pybricks.tools import wait

# ---- Field dimensions (mm) ----
TILE_MM = 600
GRID_TILES = 6
SIDE_MM = TILE_MM * GRID_TILES   # 3600

SHIFT_MM = 300                   # lane spacing

BUFFER_MM = 150                  # keep-away from outer boundary tape
DIAG_BUFFER_MM = 100             # keep-away from diagonal boundary

# Triangle sweep region: x >= y (diagonal from bottom-left to top-right)

# ---- Robot geometry (calibrated) ----
WHEEL_DIAMETER_MM = 57.0
AXLE_TRACK_MM = 134

# ---- Drive tuning ----
DRIVE_SPEED = 250
DRIVE_MIN_SPEED = 120
SLOW_ZONE = 600

KP_TURNRATE = 8.0
MAX_TURNRATE = 120
HEADING_DEADBAND = 0.8

# ---- Gyro turn tuning ----
TURN_FAST = 600
TURN_SLOW = 150
TURN_TOL = 2
TURN_SLOW_ZONE = 20
KP_TURN = 8
MIN_TURN_SPEED = 80

# ---- Color edge detection ----
EDGE_COLORS = (Color.BLUE,)
COLOR_DEBOUNCE = 2
EDGE_BACKOFF_MM = 150

# ---- Claw ----
CLAW_SPEED = 400
CLAW_OPEN_ANGLE = 200

# ---- Hardware setup ----
ev3 = EV3Brick()

left_motor = Motor(Port.D)
right_motor = Motor(Port.A)
claw_motor = Motor(Port.C)

gyro = GyroSensor(Port.S3)
color = ColorSensor(Port.S4)

robot = DriveBase(
    left_motor, right_motor,
    wheel_diameter=WHEEL_DIAMETER_MM,
    axle_track=AXLE_TRACK_MM,
)


def clamp(v, lo, hi):
    if v < lo:
        return lo
    if v > hi:
        return hi
    return v


def edge_color_seen(state):
    c = color.color()
    if c in EDGE_COLORS:
        state["edge_count"] += 1
    else:
        state["edge_count"] = 0
    return state["edge_count"] >= COLOR_DEBOUNCE


def turn_relative(deg):
    robot.stop()
    gyro.reset_angle(0)
    wait(200)

    target = deg

    while True:
        angle = gyro.angle()
        error = target - angle
        if abs(error) <= TURN_TOL:
            break

        speed = int(KP_TURN * abs(error))
        speed = max(MIN_TURN_SPEED, min(TURN_FAST, speed))
        if abs(error) <= TURN_SLOW_ZONE:
            speed = min(speed, TURN_SLOW)

        if error > 0:
            left_motor.run(speed)
            right_motor.run(-speed)
        else:
            left_motor.run(-speed)
            right_motor.run(speed)

        wait(20)

    left_motor.stop(Stop.BRAKE)
    right_motor.stop(Stop.BRAKE)
    wait(200)

    # Fine-tune correction pass
    while abs(gyro.angle() - target) > TURN_TOL:
        angle = gyro.angle()
        error = target - angle

        speed = int(KP_TURN * abs(error))
        speed = max(MIN_TURN_SPEED, min(TURN_SLOW, speed))

        if error > 0:
            left_motor.run(speed)
            right_motor.run(-speed)
        else:
            left_motor.run(-speed)
            right_motor.run(speed)

        wait(20)

    left_motor.stop(Stop.BRAKE)
    right_motor.stop(Stop.BRAKE)
    wait(200)


def turn_left_90():
    turn_relative(-90)


def turn_right_90():
    turn_relative(90)


def turn_180():
    turn_relative(180)


def drive_segment(distance_mm, state, path=None, record=False, check_edge=True):
    robot.stop()
    robot.reset()

    direction = 1 if distance_mm >= 0 else -1
    target = abs(distance_mm)
    target_heading = gyro.angle()
    hit_edge = False

    while abs(robot.distance()) < target:
        traveled = abs(robot.distance())
        remaining = target - traveled

        speed = DRIVE_SPEED
        if remaining < SLOW_ZONE:
            speed = max(DRIVE_MIN_SPEED, int(DRIVE_SPEED * remaining / SLOW_ZONE))

        err = gyro.angle() - target_heading
        if abs(err) < HEADING_DEADBAND:
            err = 0

        turn_rate = clamp(-KP_TURNRATE * err, -MAX_TURNRATE, MAX_TURNRATE)

        robot.drive(direction * speed, turn_rate)

        if check_edge and edge_color_seen(state):
            hit_edge = True
            break

        wait(20)

    robot.stop()
    left_motor.stop(Stop.BRAKE)
    right_motor.stop(Stop.BRAKE)
    wait(150)

    traveled_signed = int(robot.distance())

    if record and path is not None and abs(traveled_signed) > 5:
        path.append(("drive", traveled_signed))

    if hit_edge:
        state["edge_count"] = 0
        drive_segment(-EDGE_BACKOFF_MM, state, path=path, record=record, check_edge=False)

    return traveled_signed, hit_edge


def do_turn(deg, path=None, record=False):
    turn_relative(deg)
    if record and path is not None:
        path.append(("turn", deg))


def invert_move(move):
    kind, val = move
    if kind == "drive":
        return ("drive", -val)
    if kind == "turn":
        return ("turn", -val)
    return move


def return_home_by_retrace(path, state):
    for move in reversed(path):
        kind, val = invert_move(move)
        if kind == "drive":
            drive_segment(val, state, path=None, record=False, check_edge=False)
        elif kind == "turn":
            turn_relative(val)


# ---- Triangle geometry ----

def x_min_for_y(y_mm):
    return max(BUFFER_MM, y_mm + DIAG_BUFFER_MM)


def x_max():
    return SIDE_MM - BUFFER_MM


def can_have_lane(y_mm):
    return (x_max() - x_min_for_y(y_mm)) > 0


# ---- Main ----

state = {"edge_count": 0}

claw_motor.run_target(CLAW_SPEED, CLAW_OPEN_ANGLE, then=Stop.HOLD)
wait(200)

gyro.reset_angle(0)
wait(2000)

path = []

# Robot starts near bottom-left corner facing +X.
start_y = BUFFER_MM
start_x = x_min_for_y(start_y)

drive_segment(start_x, state, path=path, record=True, check_edge=True)

do_turn(-90, path=path, record=True)
drive_segment(start_y, state, path=path, record=True, check_edge=True)
do_turn(90, path=path, record=True)

lane_idx = 0
y = start_y

while can_have_lane(y):
    lane_len = x_max() - x_min_for_y(y)
    if lane_len <= 0:
        break

    if lane_idx % 2 == 0:
        drive_segment(lane_len, state, path=path, record=True, check_edge=True)

        y_next = y + SHIFT_MM
        if not can_have_lane(y_next):
            break

        do_turn(-90, path=path, record=True)
        drive_segment(SHIFT_MM, state, path=path, record=True, check_edge=True)
        do_turn(-90, path=path, record=True)
    else:
        drive_segment(-lane_len, state, path=path, record=True, check_edge=True)

        y_next = y + SHIFT_MM
        if not can_have_lane(y_next):
            break

        # Step along the diagonal to the next lane's start
        do_turn(180, path=path, record=True)
        drive_segment(SHIFT_MM, state, path=path, record=True, check_edge=True)
        do_turn(-90, path=path, record=True)
        drive_segment(SHIFT_MM, state, path=path, record=True, check_edge=True)
        do_turn(90, path=path, record=True)

    lane_idx += 1
    y += SHIFT_MM

return_home_by_retrace(path, state)

claw_motor.run_target(CLAW_SPEED, CLAW_OPEN_ANGLE, then=Stop.HOLD)
wait(300)

ev3.speaker.beep()
