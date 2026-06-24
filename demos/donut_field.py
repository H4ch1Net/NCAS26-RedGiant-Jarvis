#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor
from pybricks.parameters import Port, Stop, Direction, Button
from pybricks.robotics import DriveBase
from pybricks.tools import wait

ev3 = EV3Brick()

left = Motor(Port.D, Direction.CLOCKWISE)
right = Motor(Port.A, Direction.CLOCKWISE)

WHEEL_DIAMETER = 56   # mm
AXLE_TRACK = 120      # mm

robot = DriveBase(left, right, WHEEL_DIAMETER, AXLE_TRACK)

robot.settings(
    straight_speed=2800,
    straight_acceleration=2500,
    turn_rate=1400,
    turn_acceleration=2500,
)

# ---- Operating area (mm) ----
AREA_X = 2500
AREA_Y = 1400
MARGIN = 200

STEP_X = 500
STEP_Y = 350

DONUT_SPEED = 800
DONUT_TURN = 1200
DONUT_MS = 1600
DONUT_PAIRS_PER_SPOT = 2


def stop_requested():
    return Button.DOWN in ev3.buttons.pressed()


def safe_wait(ms):
    t = 0
    while t < ms and not stop_requested():
        t += 10
        wait(10)


def donut(direction=1, ms=DONUT_MS):
    robot.drive(DONUT_SPEED, direction * DONUT_TURN)
    safe_wait(ms)
    robot.stop()


def go_straight(mm):
    if stop_requested():
        return False
    robot.straight(mm)
    return not stop_requested()


def turn(angle_deg):
    if stop_requested():
        return False
    robot.turn(angle_deg)
    return not stop_requested()


def run_donut_field():
    usable_x = max(0, AREA_X - 2 * MARGIN)
    usable_y = max(0, AREA_Y - 2 * MARGIN)
    cols = max(1, int(usable_x // STEP_X) + 1)
    rows = max(1, int(usable_y // STEP_Y) + 1)

    for r in range(rows):
        for c in range(cols):
            for _ in range(DONUT_PAIRS_PER_SPOT):
                donut(+1)
                donut(-1)
                if stop_requested():
                    return

            if c != cols - 1:
                if not go_straight(STEP_X):
                    return

        if r != rows - 1:
            if r % 2 == 0:
                if not turn(90): return
                if not go_straight(STEP_Y): return
                if not turn(90): return
            else:
                if not turn(-90): return
                if not go_straight(STEP_Y): return
                if not turn(-90): return


ev3.speaker.beep()
run_donut_field()

left.stop(Stop.COAST)
right.stop(Stop.COAST)
