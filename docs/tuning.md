# Tuning reference and known issues

[← Docs index](README.md)

All tuning constants sit at the top of each program. Distances are in millimetres. Speeds are mm/s for `robot.drive()` and °/s of wheel rotation for `Motor.run()`.

## Field and route

| Constant | `jarvis.py` | `jarvis_ii.py` | Meaning |
|---|---|---|---|
| `LONG_RUN_MM` | 3600 | — | Lane length |
| `SHIFT_MM` | 300 | 300 | Lane spacing |
| `AREA_X_MM` | 1800 | — | Width to cover. Lanes = `AREA_X_MM // SHIFT_MM` |
| `AVOID_LANE_INDEX` | 5 | — | Which lane has the obstacle |
| `AVOID_X_START_MM` | 1200 | — | Distance along that lane where the detour begins |
| `AVOID_X_LEN_MM` | 1200 | — | Length of the detour |
| `TILE_MM`, `GRID_TILES` | — | 600, 6 | Field is 6 × 600 = 3600 mm square |
| `BUFFER_MM` | — | 150 | Distance to keep from the outer tape |
| `DIAG_BUFFER_MM` | — | 100 | Distance to keep from the diagonal |

## Robot geometry

| Constant | Value | Notes |
|---|---|---|
| `WHEEL_DIAMETER_MM` | 57.0 | Measured. Affects every distance. |
| `AXLE_TRACK_MM` | 134 | Distance between the wheels' contact points. |

## Straight driving (`drive_straight_gyro` / `drive_segment`)

| Constant | Value | Effect of raising it |
|---|---|---|
| `DRIVE_SPEED` | 300 (`jarvis_ii`: 250) | Faster runs, more overshoot and more wheel slip |
| `DRIVE_MIN_SPEED` | 120 | Slowest speed in the slow zone. Too low and the robot may stall short of the target. |
| `SLOW_ZONE` | 600 | Starts slowing down earlier |
| `KP_TURNRATE` | 8.0 | Stronger heading correction. Too high makes the robot snake from side to side. |
| `MAX_TURNRATE` | 120 | Caps how hard a single correction can steer (°/s) |
| `HEADING_DEADBAND` | 0.8 | Ignores more gyro noise but lets more drift through |

## Turns (`turn_gyro` / `turn_relative`)

| Constant | Value | Effect |
|---|---|---|
| `TURN_FAST` | 600 | Top motor speed in pass 1 |
| `TURN_SLOW` | 150 | Speed cap within `TURN_SLOW_ZONE` of the target, and in pass 2 |
| `TURN_SLOW_ZONE` | 20 | How many degrees before the target to slow down |
| `KP_TURN` | 8 | Motor °/s per degree of error |
| `MIN_TURN_SPEED` | 80 | Keeps the motors from stalling near the target |
| `TURN_TOL` | 2 | A turn counts as done when within ±2° |

## Edge detection and claw

| Constant | Value | Notes |
|---|---|---|
| `EDGE_COLORS` | `(Color.BLUE,)` | Add colours here to stop on other markings too |
| `COLOR_DEBOUNCE` | 2 | Readings in a row (×20 ms) needed to count as an edge |
| `EDGE_BACKOFF_MM` | 80 (`jarvis_ii`: 150) | How far to back away from the tape |
| `CLAW_SPEED` | 400 | °/s |
| `CLAW_OPEN_ANGLE` | 200 | Motor angle for "open", measured from where the motor was when the program started |

The claw's angle is measured from where the motor was at power-on. **Close the claw fully before starting a run**, otherwise "open" ends up somewhere else.

---

## Known issues

1. **`jarvis_ii.py` reverses on odd lanes.** `drive_segment(-lane_len, ...)` at line 277 makes a robot that already faces west drive backwards, out of the field. Use `lane_len`. Details and a simulator comparison are in [jarvis_ii.md](jarvis_ii.md#simulated-result-and-a-sign-bug-on-odd-lanes).
2. **Turn errors build up.** Each turn zeroes the gyro and only aims for ±2°, so small errors add up across a run (see the lane tilt in [jarvis.md](jarvis.md#the-whole-run)). The retrace undoes them on the way home, but the far lanes may not be exactly where they're meant to be.
3. **The edge failsafe in `jarvis.py` doesn't tell the main loop.** After a tape hit it turns left, and the main loop carries on as if it hadn't (see [jarvis.md](jarvis.md#5-the-edge-failsafe-in-practice)).
4. **The edge backoff in `jarvis_ii.py` doesn't check direction.** It always calls `drive_segment(-EDGE_BACKOFF_MM)`. That's correct after a forward drive, but after a reverse drive it pushes the robot further toward the tape.
5. **No tape check on the way home.** The retrace uses `check_edge=False`.
6. **The claw only opens.** Both programs move the claw to `CLAW_OPEN_ANGLE` at the start and end. There's no code here that closes it or reads a sample's colour.

## Ideas for improvement

- **Track an absolute heading.** Zero the gyro once at startup and keep a `target_heading` variable that each turn adds ±90 to. Then turn until `gyro.angle()` reaches the target, and have `drive_straight_gyro()` hold `target_heading` instead of "whatever we're pointing at now". Turn errors would then be corrected instead of added up.
- **Tell the main loop about a tape hit.** Have `drive_straight_gyro()` return `hit_edge` (as `drive_segment()` already does) and let the main loop decide what to do, such as skipping the lane shift or ending the sweep early.
- **Make the backoff go the opposite way to the drive:** `drive_segment(-direction * EDGE_BACKOFF_MM, ...)`.
- **Combine the two programs.** The building blocks are almost identical. Moving them into a shared `jarvis_lib.py` would mean a fix only needs to be made once.

Any of these can be tried in the [simulator](simulator.md) first with `patches=`.
