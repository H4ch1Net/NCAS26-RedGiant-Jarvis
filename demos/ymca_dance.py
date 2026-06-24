#!/usr/bin/env pybricks-micropython

from _thread import start_new_thread
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor
from pybricks.parameters import Port, Direction, Stop
from pybricks.tools import wait

ev3 = EV3Brick()
ev3.speaker.set_volume(100)

left_motor = Motor(Port.D, Direction.COUNTERCLOCKWISE)
right_motor = Motor(Port.A)

SPEED = 300
STEP = 45  # degrees per motor pulse


def play_ymca_forever():
    path = "/home/robot/NCAS/ymca.wav"
    while True:
        try:
            ev3.speaker.play_file(path)
        except OSError:
            ev3.screen.clear()
            ev3.screen.print("Missing:", path)
            wait(2000)


start_new_thread(play_ymca_forever, ())

while True:
    left_motor.run_angle(SPEED, -STEP, then=Stop.HOLD, wait=True)
    left_motor.run_angle(SPEED, STEP, then=Stop.HOLD, wait=True)

    right_motor.run_angle(SPEED, STEP, then=Stop.HOLD, wait=True)
    right_motor.run_angle(SPEED, -STEP, then=Stop.HOLD, wait=True)
