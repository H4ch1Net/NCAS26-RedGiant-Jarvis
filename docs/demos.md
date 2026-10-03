# Demo programs

[← Docs index](README.md)

The `demos/` folder holds short programs used for testing and showing off the robot. They aren't part of the competition run.

| File | Library | What it does | Stops by |
|---|---|---|---|
| [`ymca_dance.py`](#ymca_dancepy) | Pybricks | Plays YMCA and rocks each wheel to the beat | never (stop it from the brick) |
| [`daisy_dance.py`](#daisy_dancepy) | Pybricks | 40-second dance with status-light colours | finishes after 40 s |
| [`donut_field.py`](#donut_fieldpy) | Pybricks | Spins "donuts" on a 5 × 3 grid | end of grid, or **DOWN** button |
| [`motor_test.py`](#motor_testpy) | ev3dev2 | Runs two motors until you press Enter | Enter key |

The WAV files the dance demos play (`ymca.wav`, `daisy.wav`) aren't in the repo, and `.gitignore` excludes `*.wav`. Copy your own onto the brick at the paths below.

> **Motor direction.** Both dance demos create the left motor with `Direction.COUNTERCLOCKWISE`, while `jarvis.py` uses the default. The dance demos were probably written for a chassis with the left motor mounted the other way round. On the competition robot, a "forward" step in these demos may spin the robot instead.

---

## `ymca_dance.py`

![YMCA motor angles](images/ymca_dance.png)

```python
start_new_thread(play_ymca_forever, ())     # music in the background

while True:
    left_motor.run_angle(SPEED, -STEP, then=Stop.HOLD, wait=True)
    left_motor.run_angle(SPEED,  STEP, then=Stop.HOLD, wait=True)
    right_motor.run_angle(SPEED,  STEP, then=Stop.HOLD, wait=True)
    right_motor.run_angle(SPEED, -STEP, then=Stop.HOLD, wait=True)
```

- **Two threads.** `_thread.start_new_thread` runs the music loop at the same time as the motor loop. `play_ymca_forever()` replays `/home/robot/NCAS/ymca.wav` forever. If the file is missing it prints the path on the brick's screen every 2 s instead of crashing.
- **Wheel rocking:** each wheel turns 45° out and back at 300 °/s, one wheel at a time. `wait=True` makes each move finish before the next starts, so the timing comes from the motors themselves. One full cycle takes about 0.6 s.
- The loop never ends. Stop it with the brick's back button or from VS Code.

---

## `daisy_dance.py`

![Daisy Bell dance](images/daisy_dance.png)

The dance is a table. Each row of `BASE_BLOCK` is `(left_speed, right_speed, duration_ms)`:

```python
BASE_BLOCK = [
    ( 400,  400,  500),  # forward
    (-400, -400,  500),  # back
    (-500,  500,  500),  # spin left
    ( 500, -500,  500),  # spin right
    ( 650, -650,  250),  # shimmy ×4
    ...
    ( 250,  650, 1000),  # arc left
    ( 650,  250, 1000),  # arc right
]
SCALES = [0.9, 1.0, 1.1, 1.2, 1.2, 1.1, 1.0, 0.9]
```

The 5-second block plays 8 times. Each time, every speed is multiplied by that cycle's `SCALES` value, so the dance speeds up toward the middle and slows down again. The status light changes colour to match (green → yellow → orange → red → back).

**Timing that doesn't drift:**

```python
sw = StopWatch(); t = 0
for ...:
    left_motor.run(...); right_motor.run(...)
    t += dur
    wait_until(sw, t)          # wait until the absolute time t
```

Each step waits until an *absolute* time on a stopwatch, not for a fixed `wait(dur)`. Any delay in one step is made up in the next, so after 40 s the dance is still in time with the music.

The music runs on a background thread when `_thread` is available. If it isn't, the program plays the music first and then dances. If the WAV is missing, the program shows "Missing WAV" on screen, beeps and dances anyway. In the right-hand panel above, the robot ends up about 3 m from where it started, because each arc-left/arc-right pair leaves it slightly shifted.

---

## `donut_field.py`

![Donut field layout](images/donut_field.png)

The robot drives a lawn-mower pattern across a 2.5 m × 1.4 m area and spins donuts at each spot:

```python
usable_x = AREA_X - 2 * MARGIN       # 2100
usable_y = AREA_Y - 2 * MARGIN       # 1000
cols = usable_x // STEP_X + 1        # 2100 // 500 + 1 = 5
rows = usable_y // STEP_Y + 1        # 1000 // 350 + 1 = 3
```

- At each spot it does `DONUT_PAIRS_PER_SPOT = 2` pairs: a clockwise donut then a counter-clockwise one, 1.6 s each, using `robot.drive(800, ±1200)`.
- At the end of a row it turns 90° (`robot.turn`) toward the next row and back. The first turn is to the right (`turn(90)` is clockwise), so **start the robot in the top-left corner** of the area.
- **Emergency stop:** every helper checks `Button.DOWN`, and `safe_wait()` checks it every 10 ms, so holding DOWN stops the robot within a few milliseconds.
- `robot.settings(straight_speed=2800, turn_rate=1400, ...)` asks for more speed than an EV3 motor can deliver, so in practice the robot just runs at full speed.
- This demo uses its own `DriveBase` numbers (56 mm wheels, 120 mm axle track). It was likely tuned on a different chassis from Jarvis.

The figure shows the **planned** layout, worked out from the constants. The simulator's motor model is too rough to reproduce full-speed spins reliably.

---

## `motor_test.py`

```python
from ev3dev2.motor import MoveTank, OUTPUT_B, OUTPUT_C

tank = MoveTank(OUTPUT_B, OUTPUT_C)
tank.on(left_speed=50, right_speed=50)
input("Press Enter to stop...")
tank.off()
```

A quick check that two motors work. Things to note:

- It uses **ev3dev2**, not Pybricks. Its shebang is `#!/usr/bin/env python3`, so it runs under regular Python on the brick and **won't** run under `pybricks-micropython`.
- It drives ports **B and C**. The Jarvis drive motors are on A and D, so move the cables or change `OUTPUT_B, OUTPUT_C` to `OUTPUT_A, OUTPUT_D` before testing them.
- The speeds are percentages (50 = half power), not °/s as in Pybricks.
- `input()` needs a terminal, so run it over SSH, not from the brick menu.
