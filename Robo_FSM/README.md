# Robo_FSM: FSM line follower on the real robot

The Pi runs the FSM (IDLE, FOLLOW, NODE, ALIGN, WAIT, SEARCH, TURN, CHECK) and the pure pursuit follower.
At each intersection (seen by `node_detector.py`) the navigator decides: `Explorer` (START, e.g. the 8:
straight on) or `Mission` (`MISSION` in `params.ini`: goal on a grid, Dijkstra from `graph/` at every node).
The Arduino only applies the PWM it receives and reports the ultrasound distance.

| Folder | What it has |
|---|---|
| `pi/` | `fsm.py` (FSM and main loop), `line_follower.py`, `robot_hardware.py` (Pi Camera + serial), `navigators.py` and `node_detector.py` (same files as the Webots controller) |
| `graph/` | the graph logic: `path.py` (Dijkstra) and `track.py` (grid map) used by `Mission`, plus `navigation.py`, `maneuver.py`, `simulation.py`. The only copy in the repo: the Webots `navigators.py`, `tests/` and `visual_test.py` load it from here |
| `arduino/robo_fsm/` | `robo_fsm.ino` and the course files it uses (`Motor`, `MyServo`, `Parameters.h`) |

## Serial protocol (115200)

- Pi to Arduino: `S<left>,<right>` wheel speed in mm/s (per-wheel PI on the encoders), `M<left>,<right>` raw PWM -255..255, `X` stop. No command for 300 ms: motors stop.
- Arduino to Pi: `D,<cm>,<nFR>,<nFL>,<nRR>,<nRL>` every 50 ms (cm: -1 = no echo, -2 = no signal; n = encoder counts).
- Grid mission on the Arduino (`params.ini` `NAV_ON_ARDUINO = True`): `navigators.Mission` ported to C++ in
  `arduino/robo_fsm/navigator.cpp`, asked by `pi/arduino_navigator.py`. Pi to Arduino: `N<i>,<j>,<heading 0-3 = ENWS>,<goal i>,<goal j>,<n>,<g mm>`
  (new mission), `C<exits>,<s mm>` (on_node; exits -1 = None, else bits S=1 L=2 R=4), `B<s mm>,<d mm>` (on_blocked),
  `F<0 node missed | 1 turn failed>` (on_fault). Arduino to Pi: `R,<order>,<prior 0/1>,<s_next mm>,<exits>,<reason>`.

## Run

1. Upload `arduino/robo_fsm/robo_fsm.ino` (Bluno Mega 2560).
2. Copy `Robo_FSM/` to the Pi (`navigators.py` finds `Robo_FSM/graph/` going up from `pi/`).
   Needs `python3-picamera2`, `python3-serial`, `python3-opencv`, `numpy`.
3. In `pi/params.ini`, `[fsm]`: `MISSION = None` (test of the 8: it follows the line and stops for obstacles,
   **no node and no corner detection**) or e.g. `("MISSION", (0, 0), "E", (4, 3), 5, 0.50)`
   (start node, heading, goal, nodes per side, m between nodes: measure the track). `pi/params_entrega.ini` is a
   copy with a delivery mission: `ROBO_PARAMS=params_entrega.ini python3 robot_hardware.py`.
4. On the Pi: `python3 robot_hardware.py` (or `python3 fsm.py`). Starts `AUTO_START` s after launch (0: at once), Ctrl+C stops.
   Writes `fsm_log.csv`; every intersection is printed as `[NODE] ... exits=[...]`.

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
- `[corner_detector]`: the corner detection v4 of `TP-dev-emb/TP_Dev_Emb/corner_detection_v4.py` (a "+" crossing =
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
