#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor
from pybricks.parameters import Port, Direction, Color
from pybricks.tools import wait, StopWatch
import os

WAV_PATH = "/home/robot/daisy.wav"

CYCLE_MS = 5000
TOTAL_MS = 40000  # 8 cycles x 5 seconds

left_motor = Motor(Port.D, positive_direction=Direction.COUNTERCLOCKWISE)
right_motor = Motor(Port.A)

ev3 = EV3Brick()


def file_exists(path):
    try:
        os.stat(path)
        return True
    except OSError:
        return False


def play_music():
    try:
        ev3.speaker.play_file(WAV_PATH)
    except Exception:
        ev3.speaker.beep()


if file_exists(WAV_PATH):
    try:
        from _thread import start_new_thread
        start_new_thread(play_music, ())
    except Exception:
        play_music()
else:
    ev3.screen.clear()
    ev3.screen.print("Missing WAV:")
    ev3.screen.print(WAV_PATH)
    ev3.speaker.beep()

# One 5-second choreography block, repeated 8 times to fill 40 seconds.
# Each entry: (left_speed, right_speed, duration_ms)
BASE_BLOCK = [
    ( 400,  400,  500),  # forward
    (-400, -400,  500),  # back
    (-500,  500,  500),  # spin left
    ( 500, -500,  500),  # spin right
    ( 650, -650,  250),  # shimmy
    (-650,  650,  250),
    ( 650, -650,  250),
    (-650,  650,  250),
    ( 250,  650, 1000),  # arc left
    ( 650,  250, 1000),  # arc right
]

SCALES = [0.9, 1.0, 1.1, 1.2, 1.2, 1.1, 1.0, 0.9]
COLORS = [
    Color.GREEN, Color.YELLOW, Color.ORANGE, Color.RED,
    Color.RED, Color.ORANGE, Color.YELLOW, Color.GREEN,
]


def wait_until(sw, target_ms):
    while sw.time() < target_ms:
        wait(5)


sw = StopWatch()
sw.reset()
t = 0

for cycle_index, scale in enumerate(SCALES):
    ev3.light.on(COLORS[cycle_index])

    for ls, rs, dur in BASE_BLOCK:
        left_motor.run(int(ls * scale))
        right_motor.run(int(rs * scale))
        t += dur
        wait_until(sw, t)

left_motor.stop()
right_motor.stop()
ev3.light.off()

if t < TOTAL_MS:
    wait(TOTAL_MS - t)
