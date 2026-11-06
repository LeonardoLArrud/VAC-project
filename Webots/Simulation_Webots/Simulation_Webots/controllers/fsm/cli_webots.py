"""Robo_FSM/pi/cli.py in Webots: test the deliveries on the PC before the Pi, with the real FSM driving the
simulated robot on the 5x5 grid (line following, nodes, turns, obstacles).

1. Open worlds/grid_5x5_extern.wbt in Webots: robot on node (0, 0) facing east, 0.50 m between nodes.
2. In a terminal, from this folder (the python3 found first in PATH needs numpy, opencv and rich):
       export WEBOTS_HOME=/Applications/Webots.app
       PATH=<the .venv>/bin:$PATH $WEBOTS_HOME/Contents/MacOS/webots-controller \\
           --robot-name=Cherokey_4W_v3 cli_webots.py
3. Type destinations in that terminal; q + Enter stops. Add --lp after cli_webots.py for the live preview
   (path, obstacles and car redrawn while it drives; then q alone stops).
The world has a second extern robot, the grid_pose supervisor, and Webots does not run the simulation until
every extern robot is connected: this script starts it too (if it is already running, the copy just exits).
Before each run: reset the simulation in Webots (⟲) and press ▶.

The 8 (laps): open worlds/huit_extern.wbt instead and add --eight after cli_webots.py. The settings (V0, ...)
are not asked here: the Webots controller keeps its own values (it has no params.ini).
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


def load_cli_eight():
    """cli_eight.py, loaded by its path like cli.py (it imports cli, already in sys.modules)."""
    spec = importlib.util.spec_from_file_location("cli_eight", os.path.join(HERE, *[".."] * 5, "Robo_FSM", "pi",
                                                                            "cli_eight.py"))
    module = sys.modules["cli_eight"] = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module

START, HEADING, N, G = (0, 0), "E", 5, 0.50     # grid_5x5*.wbt (worlds/make_grid_5x5.py)


def start_grid_pose():
    """The world's other extern robot (grid_pose, a supervisor): without it Webots does not step the simulation
    and this controller waits forever in robot.step(). Started with the same launcher and the same python3."""
    home = os.environ.get("WEBOTS_HOME", "/Applications/Webots.app")
    launcher = next((p for p in (os.path.join(home, "Contents", "MacOS", "webots-controller"),   # macOS
                                 os.path.join(home, "webots-controller")) if os.path.exists(p)), None)   # Linux
    if launcher is None:
        console.print("[yellow]webots-controller not found: start grid_pose yourself[/]")
        return None
    script = os.path.join(HERE, "..", "grid_pose", "grid_pose.py")
    return subprocess.Popen([launcher, "--robot-name=grid_pose", script],
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def connect(cli, world):
    """The robot is created here: this waits until Webots offers it."""
    with console.status(f"connecting to Webots (robot Cherokey_4W_v3, world {world}). Stuck here? "
                        "In Webots: reset the simulation (⟲), then press ▶"):
        hw = cli_hardware(WebotsHardware, cli)(fsm.TIME_STEP)
    console.print("[green]connected to Webots[/]")
    return hw


def main_eight():
    """cli_webots.py --eight, with worlds/huit_extern.wbt: laps on the 8 (the robot's controller detects nodes
    in Webots anyway, so the crossing is counted)."""
    eight = load_cli_eight()
    cli = eight.EightCli()
    hw = connect(cli, "huit_extern.wbt")
    eight.start(cli)
    try:
        fsm.main(hw, live_view=False)
    except KeyboardInterrupt:
        console.print("\nstopped (Ctrl+C)")
    cli.close()
    console.print(cli.summary())


def main():
    fsm.AUTO_START = None                       # no START at 1 s: the CLI gives the missions
    if "--eight" in sys.argv[1:]:
        return main_eight()
    live = "--lp" in sys.argv[1:]              # live preview: cli_webots.py --lp
    cli = cli_module.DeliveryCli(START, HEADING, N, G, live=live)
    grid_pose = start_grid_pose()
    hw = connect(cli, "grid_5x5_extern.wbt")   # connect first: the CLI shows up once the robot is there
    start_cli(START, HEADING, N, G, cli, live=live)
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
