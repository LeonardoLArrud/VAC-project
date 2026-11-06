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
- Arduino to Pi: `D,<cm>,<nFR>,<nFL>,<nRR>,<nRL>,<pFR>,<pFL>,<pRR>,<pRL>` every 20 ms (`PERIOD_D_MS`, `D_RATE` on
  the Pi; cm: -1 = no echo, -2 = no signal; n = encoder counts; p = PWM applied now). The Pi also reads the older
  line without the PWM.
- Pi to Arduino: `T<pwm>` PWM ceiling of each wheel while spinning in place (`PWM_TURN_MAX`, sent at start; 255 at
  boot). The wheel PI also ramps the PWM up by at most 20 per 20 ms: the 4 motors never jump to full PWM together.
- Pi to Arduino: `A<deg>` sonar servo at 90 + deg (0..180; the Pi limits it with `SONAR_MAX_deg`). Only
  `diag.py sonar` turns it: the FSM keeps the sonar straight ahead.
- The navigation (grid map, Dijkstra) runs only on the Pi (`navigators.py`).

## Performance note: a core for the Arduino link?

The serial link to the Arduino runs in the main loop, in the same process as the camera and the line follower. The
node detector (~36 ms per image on the Pi 3) runs in a process of its own (`NodeProcess` in `node_detector.py`): the
loop copies each grey image to it through shared memory and gets the result one or two cycles later, placed with the
odometry `s` of its image. At exit it prints `[NODES] ... images/s, ms each`: at `TIME_STEP` 40 ms it should do
about 25 images/s; much fewer and the robot sees fewer images per node. If that loop gets slow, the Pi reads the
Arduino late and sends the wheel commands late; past 300 ms without a command the Arduino stops the motors (jerks).
`python3 diag.py loop` and every recorded drive print the loop period (each cycle is that late) and warn about
cycles over 300 ms; `[SERIAL] ... D lines/s` (50 expected) checks the link itself (lines lost, Arduino rebooting).
If the period shows trouble, give the serial link a core of its own: a separate process pinned with
`os.sched_setaffinity`, or move the heavy detectors off the main core.

## Run

1. Upload `arduino/robo_fsm/robo_fsm.ino` (Bluno Mega 2560).
2. Copy `Robo_FSM/` to the Pi (`navigators.py` finds `Robo_FSM/graph/` going up from `pi/`).
   **Switch the motor battery on before starting any program.** Started without it, the wheel PI drives full PWM
   into still wheels, and when the battery comes on the surge drops it. The FSM stops after `T_STALL` (1 s) of
   commanded motion without wheel motion (`STALL` in the terminal): restart the program then.
   Needs `python3-picamera2`, `python3-serial`, `python3-opencv`, `numpy`.
3. `pi/params.ini` is the one params file: the final values. `[fsm]` `MISSION` is the delivery,
   `("MISSION", (0, 0), "N", (1, 1), 5, 0.975)` (start node, heading, goal, nodes per side, m between nodes: measure
   the track); `MISSION = None` is the test of the 8 (it follows the line and stops for obstacles, **no node and no
   corner detection**). Another file for one run: `ROBO_PARAMS=<file> python3 robot_hardware.py`.
4. On the Pi: `python3 robot_hardware.py` (or `python3 fsm.py`). Starts `AUTO_START` s after launch (0: at once), Ctrl+C stops.
   Writes `fsm_log.csv`; every intersection is printed as `[NODE] ... exits=[...]`.
5. Deliveries chosen while it runs: `python3 cli.py` (needs `rich`: `pip install rich` or `sudo apt install python3-rich`).
   Type a destination `x y` (x right, y up, (0, 0) bottom-left) and Enter: it drives there; at the goal it asks for the
   next one, from where it stopped. `q` + Enter stops (also while driving). At launch it asks `V0`, the start node and
   the heading for **this run only** (Enter keeps the value from `params.ini`; the file is not changed: the run uses a
   temporary copy through `ROBO_PARAMS`). Grid size and spacing come from `MISSION` (its goal is ignored), or 5 and 0.975 m.
   rich only prints at events (new destination, arrival), not in the loop: the 40 ms loop is not slowed.
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
- `[corner_detector]`: the corner detection v4 of `TP-dev-emb/TP_Dev_Emb/corner_detection_v4.py` (a "+" crossing =
  3 or 4 concave corners, exits S, L, R; T and L nodes are left to the bars). Its px sizes are the v4 ones at 1280 px,
  scaled to `CAM_WIDTH`. `PARALLEL = "process"` runs it on another core of the Pi 3 so the FSM loop is not slowed;
  at exit it prints `[CORNERS] ... images/s, ms each`: it needs about 5 images/s or more at `V0 = 0.25`.
- Motor battery cutting out at the node turns: `python3 diag.py turnrate [rate ...] [cap=<pwm>]` spins once in place
  per rate (default 1.0 1.5 2.0 2.5 rad/s), shows the PWM of the wheels and whether they stopped (cut out), and proposes
  `TURN_RATE`; `cap=` tries another `PWM_TURN_MAX`.
- Checks: `python3 diag.py encoders` (counts go up when each wheel turns forward), then `python3 diag.py pi` (robot suspended, each wheel near 0.10 m/s).
  PI gains `KP`, `KI` and feedforward `U0`, `KFF` at the top of `robo_fsm.ino`; `ENC_SIGN` flips an encoder that counts backwards.
- Distance to a node (= odometry `s` + camera `d`), in this order:
  1. `python3 diag.py odometry`: follows a straight line for 1 m by the encoders, you measure it with a tape: `WHEEL_R`.
  2. `python3 diag.py nodecam`: robot still, a crossing at 2-3 distances you measure (robot centre to tape middle): `TILT_deg`.
  3. `python3 diag.py nodestop`: follows the line and stops on a node centre with the FSM, you measure the error (cm);
     3 stops, then it proposes `X_CAM` (it moves where the camera places the node).
     It also prints the loop period: well above `TIME_STEP`, the robot sees fewer images per node.
  4. `python3 diag.py nodegap`: follows a straight grid line through several nodes and places each one with the
     odometry: the spacing between nodes (the last number of `MISSION`). A camera bias cancels in the differences,
     so only `WHEEL_R` (step 1) matters.
