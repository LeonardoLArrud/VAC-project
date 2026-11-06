# FSM of the ST5 robot in Webots

| File | What it does |
|---|---|
| `fsm_basic.py` | The FSM (IDLE, FOLLOW, NODE, WAIT, SEARCH, TURN). Touches no device |
| `line_follower.py` | The pure pursuit line follower, packaged (with no target it returns LOST and v = w = 0) |
| `node_detector.py` | Sees the bar of an intersection 0.40-0.65 m ahead and its exits |
| `navigators.py` | Where to go at a node: `Explorer` (the 8), `Mission` (Dijkstra on a grid, from `Robo_FSM/graph/`), `OrderList` (tests) |
| `hardware.py` | The hooks: camera, rangefinder, encoders, commands, wheels, servo |
| `webots_hardware.py` | Webots hooks: camera, 4 motors, encoders PS1 to PS4 (`heading()`), `obstacle_distance()` (URM37), `set_rangefinder_angle()` (servo), keyboard |
| `fsm.py` | Main loop: `hw.read()` -> `fsm.step()` -> `hw.wheels()`, writes `fsm_log.csv` |
| `live_view.py` | The camera and the mask in the browser while the simulation runs |
| `cli_webots.py` | `Robo_FSM/pi/cli.py` (deliveries typed in a terminal) driving the simulated robot |
| `offline/` | Simulated hooks (`sim_hardware.py`), 22 scenarios (`scenarios.py`), timeline plot (`timeline.py`) |

The real robot runs its own copy of the FSM, follower, detector and navigators in `Robo_FSM/pi/`
(with `RobotHardware` in `robot_hardware.py`). Both load the graph code from `Robo_FSM/graph/`.

## Worlds (`../../worlds/`)

| World | Robot controller |
|---|---|
| `huit.wbt` | the 8 track; `basique` by default, set the robot's `controller` to `fsm` to run this FSM |
| `grid_5x5.wbt` | 5x5 grid, 0.50 m between nodes; `fsm` + the `grid_pose` supervisor (true pose) |
| `grid_5x5_box.wbt` | the same with a box on edge (2, 3)-(3, 3) |
| `grid_5x5_reflexos.wbt` | the same with floor reflections near the lines |
| `*_extern.wbt` | the same worlds with both controllers `<extern>`, for `cli_webots.py` |

`make_grid_5x5.py` rebuilds the grid textures and worlds.

## Run in Webots

- Open a grid world (controller `fsm` already set) or set the robot's `controller` to `fsm` in `huit.wbt`.
- Starts by itself 1 s after launch (`AUTO_START` in `fsm.py`). Keys, with the 3D view clicked:
  `G` start, `S` or space stop, `M` send `MISSION`, `L` blind/unblind the camera.
- `MISSION` in `fsm.py`: `None` (Explorer on the 8) or e.g. `("MISSION", (0, 0), "E", (4, 3))` on the 5x5 grid.
- `heading()` comes from the wheel encoders (no gyro). If a 180 deg U-turn in place comes out
  clearly more or less than half a turn, calibrate `K_WHEELS` (`hardware.py`) and `K` (`line_follower.py`).
- Every transition is printed: `[FSM] t=  8.64s FOLLOW -> WAIT (obstacle at 0.25 m)`.
- Deliveries from a terminal: open `grid_5x5_extern.wbt` and run `cli_webots.py` (instructions at its top).

## Reflection filter and live view

- `line_follower.py` filters the image before looking for the line (blur, `THRESHOLD`, opening
  `OPEN_W` x `OPEN_H`), as on the real robot, so small floor reflections are not taken as the line.
  `FILTER = False` turns it off. numpy only, no OpenCV.
- While the simulation runs: `http://localhost:8000/` (camera, bands, line points: green = target,
  magenta = out of range) and `http://localhost:8000/mask` (what the follower sees). `LIVE_VIEW` in `fsm.py`.

## Offline test (no Webots, no robot)

```
cd offline
python scenarios.py                    # 22 scenarios (numpy, opencv)
python scenarios.py static_obstacle    # only one
python timeline.py ../fsm_log.csv      # timeline of a run (matplotlib)
```

`offline/sim_hardware.py` is a test double of the hooks on a 2D world; it is not robot code.
Results go to `offline/results/` (not versioned).
