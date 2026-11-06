"""Robo_FSM/pi/cli.py in Webots: test the deliveries on the PC before the Pi, with the real FSM driving the
simulated robot on the 5x5 grid (line following, nodes, turns, obstacles).

Step by step on Windows, macOS and Linux: Webots/README.md.
1. Open worlds/grid_5x5_extern.wbt in Webots (0.50 m between nodes) and press play.
2. In a terminal, from this folder, with WEBOTS_HOME set (the python found first in PATH needs numpy and rich):
       <webots-controller> --robot-name=Cherokey_4W_v3 cli_webots.py
   <webots-controller>: $WEBOTS_HOME/Contents/MacOS/webots-controller (macOS), $WEBOTS_HOME/webots-controller
   (Linux), %WEBOTS_HOME%\\msys64\\mingw64\\bin\\webots-controller.exe (Windows).
3. It asks the start node and heading (Enter: (0, 0) facing east, as in the world); grid_pose puts the robot
   there. Then type destinations in that terminal; q + Enter stops. Add --lp after cli_webots.py for the live preview
   (path, obstacles and car redrawn while it drives; then q alone stops).
The world has a second extern robot, the grid_pose supervisor, and Webots does not run the simulation until
every extern robot is connected: this script starts it too (if it is already running, the copy just exits).
Before each run: reset the simulation in Webots (⟲) and press ▶.
"""
import importlib.util
import os
import subprocess
import sys

import fsm
from webots_hardware import WebotsHardware

# cli.py from Robo_FSM/pi, loaded by its path. Robo_FSM/pi is NOT put on sys.path: its params.py, fsm.py,
# navigators.py... would replace the Webots ones (node_detector.py uses params only when it can import it).
HERE = os.path.dirname(os.path.abspath(__file__))
_spec = importlib.util.spec_from_file_location("cli", os.path.join(HERE, *[".."] * 5, "Robo_FSM", "pi", "cli.py"))
cli_module = sys.modules["cli"] = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(cli_module)
cli_hardware, console, start_cli = cli_module.cli_hardware, cli_module.console, cli_module.start_cli

START, HEADING, N, G = (0, 0), "E", 5, 0.50     # grid_5x5*.wbt (worlds/make_grid_5x5.py)


def start_grid_pose(start, heading):
    """The world's other extern robot (grid_pose, a supervisor): without it Webots does not step the simulation
    and this controller waits forever in robot.step(). Started with the same launcher and the same python3.
    GRID_START tells it to put the robot on the start node first."""
    env = dict(os.environ, GRID_START=f"{start[0]} {start[1]} {heading}")
    home = os.environ.get("WEBOTS_HOME", "/Applications/Webots.app")
    launcher = next((p for p in (os.path.join(home, "Contents", "MacOS", "webots-controller"),   # macOS
                                 os.path.join(home, "webots-controller"),                          # Linux
                                 os.path.join(home, "msys64", "mingw64", "bin", "webots-controller.exe"))  # Windows
                     if os.path.isfile(p)), None)
    if launcher is None:
        console.print(f"[yellow]webots-controller not found: start grid_pose yourself, with "
                      f"GRID_START=\"{env['GRID_START']}\"[/]")
        return None
    script = os.path.join(HERE, "..", "grid_pose", "grid_pose.py")
    return subprocess.Popen([launcher, "--robot-name=grid_pose", script], env=env,
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def main():
    fsm.AUTO_START = None                       # no START at 1 s: the CLI gives the missions
    live = "--lp" in sys.argv[1:]              # live preview: cli_webots.py --lp
    start, heading = cli_module.ask_start(N, START, HEADING)
    cli = cli_module.DeliveryCli(start, heading, N, G, live=live)
    grid_pose = start_grid_pose(start, heading)
    # Connect to Webots first: the robot is created here, and this waits until Webots offers it.
    with console.status("connecting to Webots (robot Cherokey_4W_v3, world grid_5x5_extern.wbt). Stuck here? "
                        "In Webots: reset the simulation (⟲), then press ▶"):
        hw = cli_hardware(WebotsHardware, cli)(fsm.TIME_STEP)
    console.print("[green]connected to Webots[/]")
    start_cli(start, heading, N, G, cli, live=live)
    try:
        fsm.main(hw, live_view=False)
    except KeyboardInterrupt:
        console.print("\nstopped (Ctrl+C)")
    cli.close()
    console.print(cli.summary())
    if grid_pose is not None:
        grid_pose.terminate()


if __name__ == "__main__":
    main()
