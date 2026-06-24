# Jarvis & Jarvis II

Autonomous rover software built for the NASA Community College Aerospace Scholars (NCAS) 2026 program. Written in Python/MicroPython for LEGO EV3 hardware via [ev3dev](https://www.ev3dev.org/) and [Pybricks](https://pybricks.com/), developed in VS Code.

Team Red Giant X outperformed all other competing teams and earned Team MVP.

---

## Background

NCAS is a NASA-sponsored program where community college students form simulated aerospace companies and compete in lunar rover missions tied to the Artemis program. The mission required a rover to navigate an arena, avoid obstacles, identify mineral samples by color, and retrieve them with a claw.

While other teams used simple straight-line programs requiring manual repositioning between runs, Jarvis navigated the full arena autonomously. Once deployed, the rover handled navigation, obstacle avoidance, and sample interaction entirely on its own with no manual intervention between runs.

---

## Hardware

- **Platform:** LEGO MINDSTORMS EV3
- **Runtime:** ev3dev + Pybricks MicroPython
- **Sensors:**
  - Gyroscope (S3) -- heading correction and turn accuracy
  - Color sensor (S4) -- boundary detection and mineral identification
  - Ultrasonic sensor -- obstacle detection and claw activation
- **Actuators:**
  - Left drive motor (Port D)
  - Right drive motor (Port A)
  - Worm gear claw motor (Port C)

---

## Programs

### `jarvis/jarvis.py` -- Main competition program

Full-arena rectangular sweep using a predefined coordinate system in millimeters. Key features:

- **Gyroscope-corrected straight driving** -- active heading correction in the control loop eliminates chassis steering drift
- **Gyroscope-based turns** -- two-pass proportional control for accurate 90-degree turns
- **Obstacle zone avoidance** -- automatic detour around the designated obstacle zone at lane 5 (tiles 3-4)
- **Color edge detection** -- debounced blue-tape detection with automatic backoff and recovery
- **Path recording and retrace** -- all moves are logged so the rover can autonomously return to the start

### `jarvis/jarvis_ii.py` -- Triangle sweep variant

A sweep variant for triangular arena regions (x >= y diagonal). Computes lane lengths dynamically based on the diagonal boundary and navigates each lane accordingly, then retraces its path home.

---

## Demos

These are standalone programs used for testing and exhibition. They are not part of the competition run.

| File | Description |
|---|---|
| `demos/ymca_dance.py` | Plays YMCA while the motors pulse to the beat |
| `demos/daisy_dance.py` | 40-second Daisy Bell choreography with light effects |
| `demos/donut_field.py` | Drives a lawn-mower grid of donuts across the arena |
| `demos/motor_test.py` | Basic ev3dev2 motor sanity check |

---

## Setup

1. Flash your EV3 with [ev3dev](https://www.ev3dev.org/docs/getting-started/) and install Pybricks MicroPython.
2. Connect to the EV3 via SSH or VS Code with the [EV3 extension](https://marketplace.visualstudio.com/items?itemName=ev3dev.ev3dev-browser).
3. Copy the `jarvis/` folder to `/home/robot/` on the brick.
4. Run `jarvis.py` from the EV3 menu or via SSH.

### Calibration

Before a run, adjust these constants at the top of `jarvis.py` to match your build:

| Constant | Default | Description |
|---|---|---|
| `WHEEL_DIAMETER_MM` | 57.0 | Measured wheel diameter |
| `AXLE_TRACK_MM` | 134 | Distance between wheel centers |
| `DRIVE_SPEED` | 300 | Forward drive speed (mm/s) |
| `SHIFT_MM` | 300 | Lane width (mm) |
| `LONG_RUN_MM` | 3600 | Arena length (mm) |

---

## Architecture

```
startup
  |
  +-- open claw
  +-- gyro settle (2s)
  |
  for each lane (0..5):
    |
    +-- [lane 5] drive with obstacle detour
    +-- [other]  drive straight with gyro hold
    |
    +-- color edge failsafe (blue tape -> backoff + turn)
    +-- U-turn + lane shift
  |
  +-- retrace path home (reversed move log)
  +-- open claw at home
  +-- beep
```

---

## Notes

- `motor_test.py` uses the **ev3dev2** Python library, not Pybricks. It will not run in the Pybricks environment.
- WAV files referenced in the dance demos (`ymca.wav`, `daisy.wav`) are not included in this repository.
- The gyro sensor on EV3 is sensitive to vibration. Keep the robot still during the 2-second settle period at startup.

---

Built by **h4ch1net** -- Team Red Giant X, NASA NCAS 2026
