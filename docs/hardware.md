# Hardware and setup

[← Docs index](README.md)

![EV3 port map](images/hardware_ports.png)

## The robot

Jarvis is a LEGO MINDSTORMS EV3 rover with two drive wheels and a worm-gear claw. Both competition programs (`jarvis.py` and `jarvis_ii.py`) use this wiring:

| Port | Device | Pybricks object | Used for |
|---|---|---|---|
| **A** | Large motor | `right_motor = Motor(Port.A)` | Right drive wheel |
| **D** | Large motor | `left_motor = Motor(Port.D)` | Left drive wheel |
| **C** | Motor | `claw_motor = Motor(Port.C)` | Worm-gear claw |
| **S3** | Gyro sensor | `gyro = GyroSensor(Port.S3)` | Heading for straight driving and turns |
| **S4** | Color sensor | `color = ColorSensor(Port.S4)` | Spotting the blue boundary tape |

The two drive motors are combined into a Pybricks `DriveBase`:

```python
robot = DriveBase(
    left_motor, right_motor,
    wheel_diameter=WHEEL_DIAMETER_MM,   # 57.0 mm
    axle_track=AXLE_TRACK_MM,           # 134 mm
)
```

`DriveBase` uses those two numbers to turn wheel rotation into millimetres travelled (`robot.distance()`) and to turn `robot.drive(speed, turn_rate)` into separate speeds for each wheel. If either number is wrong, every distance and turn in the program is wrong by the same ratio, so these two are the first things to calibrate.

> **About the ultrasonic sensor.** The project README lists an ultrasonic sensor for obstacle detection and claw activation. Neither competition program in this repository creates an `UltrasonicSensor`. The obstacle is avoided by a detour hard-coded at a known position (see [jarvis.md](jarvis.md#the-obstacle-detour)), and the claw only moves to its open position. If the sensor was on the robot, the code that read it isn't in this repo.

### Sign conventions

These conventions come from Pybricks, and the turn code depends on them:

- **Gyro:** `gyro.angle()` gets **larger when the robot turns clockwise** (to the right).
- **`robot.drive(speed, turn_rate)`:** a positive `turn_rate` also turns clockwise.
- So in the code, **`+90` means a right turn and `-90` means a left turn.** That's why `turn_left_90()` calls `turn_gyro(-90)`.

The docs and simulator use a map where **+x is east (the direction the robot first drives)** and **+y is north (to the robot's left at the start)**.

## Software environment

- **Runtime:** Pybricks MicroPython on ev3dev. Each program starts with `#!/usr/bin/env pybricks-micropython`.
- **Editor:** VS Code with an EV3 extension. You can also copy files over SSH.
- `demos/motor_test.py` is the exception. It uses the separate **ev3dev2** Python library and runs under regular `python3` on the brick. See [demos.md](demos.md#motor_testpy).

## Getting a program onto the brick

1. Flash a microSD card with an EV3 image that includes Pybricks MicroPython ([ev3dev getting started](https://www.ev3dev.org/docs/getting-started/)). Then boot the EV3 from it.
2. Connect from VS Code (EV3 extension) or over SSH (`ssh robot@ev3dev.local`).
3. Copy the `jarvis/` folder to `/home/robot/` on the brick.
4. Make the scripts executable so they show up in the brick's file browser:
   ```sh
   chmod +x /home/robot/jarvis/*.py
   ```
5. Run a program from the brick's file browser, from the VS Code extension, or over SSH:
   ```sh
   pybricks-micropython /home/robot/jarvis/jarvis.py
   ```

## Before every run

1. **Put the robot at the start point,** facing along the first lane (+x). Every position in the program is measured from where the robot starts. See [how-it-works.md](how-it-works.md#coordinates).
2. **Don't touch it for the first ~2.7 s.** The program opens the claw (~0.5 s), resets the gyro, then waits 2 s so the gyro can settle. Bumping the robot during that wait adds a heading error to the whole run.
3. **Check the battery.** Low voltage weakens the motors. The gyro loop corrects heading but not distance, so worn or slipping wheels shorten every lane.

## Calibration

The full list of tuning constants is in [tuning.md](tuning.md). In short:

| Constant | Default | How to measure it |
|---|---|---|
| `WHEEL_DIAMETER_MM` | 57.0 | Drive `robot.straight(1000)` and measure. If the robot went *d* mm, the new value is `57.0 * d / 1000`. |
| `AXLE_TRACK_MM` | 134 | Only used by `DriveBase.drive()` to split `turn_rate` between the wheels. The turns themselves are measured by the gyro, so this mostly changes how strongly heading corrections act. |
| `SHIFT_MM` | 300 | Lane spacing. Make it a little narrower than whatever the robot sweeps, so lanes overlap. |
| `LONG_RUN_MM` | 3600 | Lane length = six 600 mm tiles. |
