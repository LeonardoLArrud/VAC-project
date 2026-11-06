# Robo_FSM: FSM line follower on the real robot

The Pi runs the FSM (IDLE, FOLLOW, NODE, ALIGN, WAIT, SEARCH, TURN, CHECK) and the pure pursuit follower.
At each intersection (seen by `node_detector.py`) the navigator decides: `Explorer` (START, e.g. the 8:
straight on) or `Mission` (`MISSION` in `params.ini`: goal on a grid, Dijkstra from `graph/` at every node).
The Arduino only applies the PWM it receives and reports the ultrasound distance.

| Folder | What it has |
|---|---|
| `pi/` | `fsm.py` (FSM and main loop), `line_follower.py`, `node_detector.py` (bars) and `corner_detector.py` (corners), `navigators.py`, `robot_hardware.py` (Pi Camera + serial), `camera_stream.py` (live view), `cli.py` (deliveries), `diag.py` (checks and calibration), `replay_filters.py` (replays a recording offline), `params.py` + `params.ini` / `params_entrega.ini` |
| `graph/` | the graph logic: `path.py` (Dijkstra) and `track.py` (grid map) used by `Mission`, plus `navigation.py`, `maneuver.py`, `simulation.py`. The only copy in the repo: the Webots `navigators.py`, `tests/` and `visual_test.py` load it from here |
| `tests/` | unit tests of `graph/` and `cli.py`: from `Robo_FSM/`, `python -m unittest discover tests` (`test_cli.py` needs `rich`) |
| `visual_test.py` | interactive view of the graph logic (tkinter): `python visual_test.py` from `Robo_FSM/` |
| `arduino/robo_fsm/` | `robo_fsm.ino` and the course files it uses (`Motor`, `MyServo`, `Parameters.h`) |

## Serial protocol (115200)

- Pi to Arduino: `S<left>,<right>` wheel speed in mm/s (per-wheel PI on the encoders), `M<left>,<right>` raw PWM -255..255, `X` stop. No command for 300 ms: motors stop.
- Arduino to Pi: `D,<cm>,<nFR>,<nFL>,<nRR>,<nRL>` every 50 ms (cm: -1 = no echo, -2 = no signal; n = encoder counts).
- Pi to Arduino: `A<deg>` sonar servo at 90 + deg (0..180; the Pi limits it with `SONAR_MAX_deg`).
- The navigation (grid map, Dijkstra) runs only on the Pi (`navigators.py`).

## Performance note: a core for the Arduino link?

The serial link to the Arduino runs in the main loop, in the same process as the camera, the line follower and the
bar node detector (the corner detector already has its own process). If that loop gets slow, the Pi reads the
Arduino late and sends the wheel commands late; past 300 ms without a command the Arduino stops the motors (jerks).
`python3 diag.py loop` and every recorded drive print the loop period (each cycle is that late) and warn about
cycles over 300 ms; `[SERIAL] ... D lines/s` (20 expected) checks the link itself (lines lost, Arduino rebooting).
If the period shows trouble, give the serial link a core of its own: a separate process pinned with
`os.sched_setaffinity`, or move the heavy detectors off the main core.

## Run

1. Upload `arduino/robo_fsm/robo_fsm.ino` (Bluno Mega 2560).
2. Copy `Robo_FSM/` to the Pi (`navigators.py` finds `Robo_FSM/graph/` going up from `pi/`).
   **Switch the motor battery on before starting any program.** Started without it, the wheel PI drives full PWM
   into still wheels, and when the battery comes on the surge drops it. The FSM stops after `T_STALL` (1 s) of
   commanded motion without wheel motion (`STALL` in the terminal): restart the program then.
   Needs `python3-picamera2`, `python3-serial`, `python3-opencv`, `numpy`.
3. In `pi/params.ini`, `[fsm]`: `MISSION = None` (test of the 8: it follows the line and stops for obstacles,
   **no node and no corner detection**) or e.g. `("MISSION", (0, 0), "E", (4, 3), 5, 0.50)`
   (start node, heading, goal, nodes per side, m between nodes: measure the track). `pi/params_entrega.ini` is a
   copy with a delivery mission: `ROBO_PARAMS=params_entrega.ini python3 robot_hardware.py`.
4. On the Pi: `python3 robot_hardware.py` (or `python3 fsm.py`). Starts `AUTO_START` s after launch (0: at once), Ctrl+C stops.
   Writes `fsm_log.csv`; every intersection is printed as `[NODE] ... exits=[...]`.
5. Deliveries chosen while it runs: `python3 cli.py` (needs `rich`: `pip install rich` or `sudo apt install python3-rich`).
   Type a destination `x y` (x right, y up, (0, 0) bottom-left) and Enter: it drives there; at the goal it asks for the
   next one, from where it stopped. `q` + Enter stops (also while driving). At launch it asks `V0`, the start node and
   the heading for **this run only** (Enter keeps the value from `params.ini`; the file is not changed: the run uses a
   temporary copy through `ROBO_PARAMS`). Grid size and spacing come from `MISSION` (its goal is ignored), or 5 and 0.975 m.
   rich only prints at events (new destination, arrival), not in the loop: the 64 ms loop is not slowed.
   `--lp` (live preview, also with `--fake` and in Webots): while it drives, one panel is redrawn in place with the
   Dijkstra path, the edges blocked by obstacles (✗) and the car; after an obstacle the new path shows at once.
   While driving only `q` works, alone (no Enter); the prompt comes back at the goal. In `--fake`,
   `block x1 y1 x2 y2` puts a hidden obstacle on an edge, to see the replanning.
   Before the Pi, on a PC (a Python with `rich`, plus `numpy` and `opencv` for Webots):
   - `python3 cli.py --fake`: a fake robot (no camera, no serial) moves along the real `Mission` path, 1 s per edge.
   - Webots, real FSM on the 5x5 grid: open `worlds/grid_5x5_extern.wbt`, then from `controllers/fsm/` run
     `WEBOTS_HOME=/Applications/Webots.app $WEBOTS_HOME/Contents/MacOS/webots-controller --robot-name=Cherokey_4W_v3 cli_webots.py`
     (see `cli_webots.py`; the `python3` first in PATH is the one used). It also starts the world's `grid_pose`
     supervisor: Webots runs the simulation only once every extern robot is connected. Reset (⟲) and ▶ before each run.

## Before the first test

All the parameters are in `pi/params.ini`, and only there (the .py files declare the names). Values are
Python literals, angles with `_deg` are in degrees; a missing or misspelled name stops the program.

- `[robot_hardware]`: `PORT`, `CAM_FOV_deg` (62.2 for camera v2, 53.5 for v1.3), `PWM_MIN`, `V_MAX`;
  `USE_ENCODERS`: True = per-wheel PI and heading from the encoders, False = open-loop PWM.
- `[line_follower]`: camera geometry `H_CAM`, `X_CAM`, `TILT_deg`; `K` (`python3 diag.py turn`); `V0` (start low,
  e.g. 0.08) and `GAIN`; `THRESHOLD` for the lighting; `OPEN_W`, `OPEN_H` (filter against floor reflections,
  `FILTER = False` turns it off). Check it on `/mask` of the live view.
- `[fsm]`: `MISSION` (None = the 8: line and obstacles only, nodes and corners are not searched), obstacle
  distances and times, turns. `NODES` (with a MISSION only): who announces intersections: `"bars"`
  (`node_detector.py`), `"corners"` (`corner_detector.py`) or `"both"`.
- `[corner_detector]`: the corner detection v4 of the embedded dev TP, `corner_detection_v4.py` (a "+" crossing =
  3 or 4 concave corners, exits S, L, R; T and L nodes are left to the bars). Its px sizes are the v4 ones at 1280 px,
  scaled to `CAM_WIDTH`. `PARALLEL = "process"` runs it on another core of the Pi 3 so the 64 ms loop is not slowed;
  at exit it prints `[CORNERS] ... images/s, ms each`: it needs about 5 images/s or more at `V0 = 0.25`.
- Checks: `python3 diag.py encoders` (counts go up when each wheel turns forward), then `python3 diag.py pi` (robot suspended, each wheel near 0.10 m/s).
  PI gains `KP`, `KI` and feedforward `U0`, `KFF` at the top of `robo_fsm.ino`; `ENC_SIGN` flips an encoder that counts backwards.
- Distance to a node (= odometry `s` + camera `d`), in this order:
  1. `python3 diag.py odometry`: follows a straight line for 1 m by the encoders, you measure it with a tape: `WHEEL_R`.
  2. `python3 diag.py nodecam`: robot still, a crossing at 2-3 distances you measure (robot centre to tape middle): `TILT_deg`.
  3. `python3 diag.py nodestop`: follows the line and stops on a node centre with the FSM, you measure the error (cm).
     It also prints the loop period: well above `TIME_STEP`, the robot sees fewer images per node.
  4. `python3 diag.py nodegap`: follows a straight grid line through several nodes and places each one with the
     odometry: the spacing between nodes (the last number of `MISSION`). A camera bias cancels in the differences,
     so only `WHEEL_R` (step 1) matters.
