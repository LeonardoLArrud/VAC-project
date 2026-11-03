# Basic FSM (v1) of the ST5 robot

| File | What it does |
|---|---|
| `fsm_basic.py` | The FSM (IDLE, FOLLOW, WAIT, SEARCH, TURN). Touches no device |
| `line_follower.py` | The follower from `pure_pursuit.py`, packaged (same law; with no target it returns LOST and v = w = 0) |
| `hardware.py` | The hooks: camera, rangefinder, encoders, commands, wheels, servo |
| `webots_hardware.py` | Webots hooks. **Done:** camera, 4 motors, keyboard. **TODO (sensors):** `obstacle_distance()` (URM37), `heading()` (PS1 to PS4), `set_rangefinder_angle()` (servo) |
| `fsm.py` | Main loop: `hw.read()` -> `fsm.step()` -> `hw.wheels()`, writes `fsm_log.csv` |
| `offline/` | Simulated hooks (`sim_hardware.py`), 11 scenarios (`scenarios.py`), timeline plot (`timeline.py`) |

## Run in Webots

- Set the robot's `controller` to `fsm` (or, in the VM: `./run_extern.sh fsm` with `huit_extern.wbt`).
- Starts by itself 1 s after launch (`AUTO_START` in `fsm.py`). Keys, with the 3D view clicked:
  `G` start, `S` or space stop, `L` blind/unblind the camera.
- Until the sensor hooks are filled in, two placeholders run, each announced once in the console:
  `obstacle_distance()` returns "nothing ahead" (**obstacles are ignored: the robot drives into them**) and
  `heading()` integrates the commanded turn rate (SEARCH works, without slip).
- Every transition is printed: `[FSM] t=  8.64s FOLLOW -> WAIT (obstacle at 0.25 m)`.

## Real robot

Write `RobotHardware(Hardware)` (Pi camera, ultrasound, encoders, UART to the Arduino) and use it in `fsm.py`.
The FSM does not change.

## Offline test (no Webots, no robot)

```
cd offline
python scenarios.py                    # 11 scenarios, ~45 s (numpy, opencv)
python scenarios.py static_obstacle    # only one
python timeline.py ../fsm_log.csv      # timeline of a run (matplotlib)
```

`offline/sim_hardware.py` is a test double of the hooks on a 2D world; it is not robot code.
