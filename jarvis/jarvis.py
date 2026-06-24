#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, GyroSensor, ColorSensor
from pybricks.parameters import Port, Stop, Color
from pybricks.robotics import DriveBase
from pybricks.tools import wait

# ---- Field dimensions (mm) ----
LONG_RUN_MM = 3600
SHIFT_MM = 300
AREA_X_MM = 1800

# Obstacle zone: top-half of tiles 3 and 4 (x = 1200..2400) in lane 5 (y = 1500mm)
AVOID_LANE_INDEX = 5
AVOID_X_START_MM = 1200
AVOID_X_LEN_MM = 1200

# ---- Robot geometry (calibrated) ----
WHEEL_DIAMETER_MM = 57.0
AXLE_TRACK_MM = 134

# ---- Drive tuning ----
DRIVE_SPEED = 300
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
COLOR_DEBOUNCE = 2
EDGE_COLORS = (Color.BLUE,)
EDGE_BACKOFF_MM = 80

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


def turn_gyro(deg):
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

    left_motor.brake()
    right_motor.brake()
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

    left_motor.brake()
    right_motor.brake()
    wait(200)


def turn_left_90():
    turn_gyro(-90)


def turn_right_90():
    turn_gyro(90)


def drive_straight_gyro(distance_mm, state, path=None, record=False, check_edge=True):
    robot.stop()
    robot.reset()

    direction = 1 if distance_mm >= 0 else -1
    target_distance = abs(distance_mm)
    target_heading = gyro.angle()

    while abs(robot.distance()) < target_distance:
        traveled = abs(robot.distance())
        remaining = target_distance - traveled

        speed = DRIVE_SPEED
        if remaining < SLOW_ZONE:
            speed = max(DRIVE_MIN_SPEED, int(DRIVE_SPEED * remaining / SLOW_ZONE))

        err = gyro.angle() - target_heading
        if abs(err) < HEADING_DEADBAND:
            err = 0

        turn_rate = clamp(-KP_TURNRATE * err, -MAX_TURNRATE, MAX_TURNRATE)

        robot.drive(direction * speed, turn_rate)

        if check_edge and edge_color_seen(state):
            break

        wait(20)

    robot.stop()
    left_motor.brake()
    right_motor.brake()
    wait(150)

    actual = int(abs(robot.distance()))
    if record and path is not None and actual > 5:
        path.append(("drive", direction * actual))

    if check_edge and state["edge_count"] >= COLOR_DEBOUNCE:
        state["edge_count"] = 0
        drive_straight_gyro(-EDGE_BACKOFF_MM, state, path=path, record=record, check_edge=False)
        if record and path is not None:
            path.append(("turnL", 0))
        turn_left_90()


def invert_move(move):
    kind, val = move
    if kind == "drive":
        return ("drive", -val)
    if kind == "turnL":
        return ("turnR", 0)
    if kind == "turnR":
        return ("turnL", 0)
    return move


def return_home_by_retrace(path, state):
    state["edge_count"] = 0
    for move in reversed(path):
        inv = invert_move(move)
        kind, val = inv
        if kind == "drive":
            drive_straight_gyro(val, state, path=None, record=False, check_edge=False)
        elif kind == "turnL":
            turn_left_90()
        elif kind == "turnR":
            turn_right_90()


def detour_down(travel_dir, state, path):
    if travel_dir == 1:
        path.append(("turnR", 0)); turn_right_90()
        drive_straight_gyro(SHIFT_MM, state, path=path, record=True, check_edge=True)
        path.append(("turnL", 0)); turn_left_90()
    else:
        path.append(("turnL", 0)); turn_left_90()
        drive_straight_gyro(SHIFT_MM, state, path=path, record=True, check_edge=True)
        path.append(("turnR", 0)); turn_right_90()


def detour_up(travel_dir, state, path):
    if travel_dir == 1:
        path.append(("turnL", 0)); turn_left_90()
        drive_straight_gyro(SHIFT_MM, state, path=path, record=True, check_edge=True)
        path.append(("turnR", 0)); turn_right_90()
    else:
        path.append(("turnR", 0)); turn_right_90()
        drive_straight_gyro(SHIFT_MM, state, path=path, record=True, check_edge=True)
        path.append(("turnL", 0)); turn_left_90()


def drive_long_with_avoid(travel_dir, state, path):
    drive_straight_gyro(AVOID_X_START_MM, state, path=path, record=True, check_edge=True)
    detour_down(travel_dir, state, path)
    drive_straight_gyro(AVOID_X_LEN_MM, state, path=path, record=True, check_edge=True)
    detour_up(travel_dir, state, path)
    remaining = LONG_RUN_MM - (AVOID_X_START_MM + AVOID_X_LEN_MM)
    drive_straight_gyro(remaining, state, path=path, record=True, check_edge=True)


# ---- Main ----

state = {"edge_count": 0}

claw_motor.run_target(CLAW_SPEED, CLAW_OPEN_ANGLE, then=Stop.HOLD)
wait(200)

gyro.reset_angle(0)
wait(2000)

lanes = int(AREA_X_MM // SHIFT_MM)  # 6 lanes (0..5)
path = []
travel_dir = 1  # +1 = east, -1 = west

for lane_idx in range(lanes):
    if lane_idx == AVOID_LANE_INDEX:
        drive_long_with_avoid(travel_dir, state, path)
    else:
        drive_straight_gyro(LONG_RUN_MM, state, path=path, record=True, check_edge=True)

    if lane_idx == lanes - 1:
        break

    if travel_dir == 1:
        path.append(("turnL", 0)); turn_left_90()
        drive_straight_gyro(SHIFT_MM, state, path=path, record=True, check_edge=True)
        path.append(("turnL", 0)); turn_left_90()
        travel_dir = -1
    else:
        path.append(("turnR", 0)); turn_right_90()
        drive_straight_gyro(SHIFT_MM, state, path=path, record=True, check_edge=True)
        path.append(("turnR", 0)); turn_right_90()
        travel_dir = 1

return_home_by_retrace(path, state)

claw_motor.run_target(CLAW_SPEED, CLAW_OPEN_ANGLE, then=Stop.HOLD)
wait(300)

ev3.speaker.beep()
