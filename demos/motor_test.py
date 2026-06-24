#!/usr/bin/env python3
# Basic motor test using ev3dev2 (different from pybricks used in the main programs).

from ev3dev2.motor import MoveTank, OUTPUT_B, OUTPUT_C

tank = MoveTank(OUTPUT_B, OUTPUT_C)

tank.on(left_speed=50, right_speed=50)
input("Press Enter to stop...")
tank.off()
