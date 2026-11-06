# Webots simulation

The robot follows the line, detects the intersections, stops for obstacles and navigates the 5x5 grid with Dijkstra.
The navigation (`navigators.py` and `Robo_FSM/graph/`) is the same as on the real robot.

`SIM` = `Webots/Simulation_Webots/Simulation_Webots`.

## Install

- **Webots R2025a**: <https://cyberbotics.com>.
- **Python 3** (tested with 3.11), with `numpy` and `rich` (plus `opencv-python` and `matplotlib` for the offline test):

  | OS | Command |
  |---|---|
  | Windows | `python -m pip install numpy rich opencv-python matplotlib` |
  | macOS | `python3 -m pip install numpy rich opencv-python matplotlib` |
  | Ubuntu/Debian | `sudo apt install python3-numpy python3-rich python3-opencv python3-matplotlib` |

- Webots uses the `python` (Windows) or `python3` (macOS and Linux) found in PATH. To use another one, such as a
  `venv`'s, set **Tools → Preferences → General → Python command**.

## 1. Webots only

1. *(Optional)* For a mission, edit `SIM/controllers/fsm/fsm.py`:
   `MISSION = ("MISSION", (0, 0), "E", (4, 3))` (start node, heading, goal).
   x grows to the right, y upwards, and (0, 0) is the bottom-left corner.
2. **File → Open World →** `SIM/worlds/grid_5x5.wbt` and **▶**. The robot starts by itself after 1 s.

- **Keys** (with the 3D view clicked): `G` starts, `S` stops, `M` sends the mission again, `L` blinds the camera.
- **Camera:** <http://localhost:8000/> shows the camera, and <http://localhost:8000/mask> what the follower sees.
- **Other worlds:**
  - `grid_5x5_box.wbt`: a box between (2, 3) and (3, 3); with the mission above, the robot turns back and replans;
  - `grid_5x5_reflexos.wbt`: reflections on the floor;
  - `huit.wbt`: the 8; set the robot's `controller` to `fsm`, otherwise the old controller runs.

## 2. Deliveries typed in a terminal

1. Open `SIM/worlds/grid_5x5_extern.wbt` (or `grid_5x5_box_extern.wbt`) and press **▶**.
2. In a terminal:

   **Windows (PowerShell)**
   ```powershell
   $env:WEBOTS_HOME = "C:\Program Files\Webots"    # or "$env:LOCALAPPDATA\Programs\Webots"
   cd <repo>\Webots\Simulation_Webots\Simulation_Webots\controllers\fsm
   & "$env:WEBOTS_HOME\msys64\mingw64\bin\webots-controller.exe" --robot-name=Cherokey_4W_v3 cli_webots.py --lp
   ```

   **macOS**
   ```bash
   export WEBOTS_HOME=/Applications/Webots.app
   cd <repo>/Webots/Simulation_Webots/Simulation_Webots/controllers/fsm
   "$WEBOTS_HOME/Contents/MacOS/webots-controller" --robot-name=Cherokey_4W_v3 cli_webots.py --lp
   ```

   **Linux**
   ```bash
   export WEBOTS_HOME=/usr/local/webots
   cd <repo>/Webots/Simulation_Webots/Simulation_Webots/controllers/fsm
   "$WEBOTS_HOME/webots-controller" --robot-name=Cherokey_4W_v3 cli_webots.py --lp
   ```
3. Type the start node and heading (Enter: `0 0` and `E`), or add `--start 2 1 --heading N` to the command.
4. Type a destination `x y` (0 to 4) and Enter. While it drives, `q` (no Enter) stops the robot. At the goal,
   type the next destination, or `q` + Enter to quit.

**Laps on the 8:** open `huit_extern.wbt` and replace `--lp` with `--eight`. Type a number of laps (`0` = until `q`).
In Webots the crossing of the 8 is not detected, so the laps are not counted: stop with `q`.

Before each run: **⟲** and **▶**.

## 3. Offline (no Webots)

```bash
cd <repo>/Webots/Simulation_Webots/Simulation_Webots/controllers/fsm/offline
python scenarios.py                   # the 22 scenarios, PASS or FAIL (takes a few minutes)
python scenarios.py mission_blocked   # only one
```

## Common problems

| Message | Fix |
|---|---|
| `Set the path to your webots installation folder in WEBOTS_HOME` | Set `WEBOTS_HOME` (section 2, step 2) |
| `webots-controller not found: start grid_pose yourself` | `WEBOTS_HOME` points to the wrong folder |
| PowerShell: `The '--' operator works only on variables or on properties` | The `&` before the quoted path is missing |
| Stuck at `connecting to Webots` | Open a `*_extern.wbt` world and press ⟲ and ▶ |
| `No module named 'numpy'` (or `'rich'`) | Install the package in the Python that Webots uses |
| `pose lost at (x, y)` | The robot was not where the mission assumed: ⟲, ▶ and check the node and heading you typed |
| `Texture image size ... is not a power of two` | A Webots warning, it can be ignored |
