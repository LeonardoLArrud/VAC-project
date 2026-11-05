"""Robo_FSM/pi/cli.py in Webots: test the deliveries on the PC before the Pi, with the real FSM driving the
simulated robot on the 5x5 grid (line following, nodes, turns, obstacles).

1. Open worlds/grid_5x5_extern.wbt in Webots: robot on node (0, 0) facing east, 0.50 m between nodes.
2. In a terminal, from this folder (the python3 found first in PATH needs numpy, opencv and rich):
       export WEBOTS_HOME=/Applications/Webots.app
       PATH=<the .venv>/bin:$PATH $WEBOTS_HOME/Contents/MacOS/webots-controller \\
           --robot-name=Cherokey_4W_v3 cli_webots.py
3. Type destinations in that terminal; q + Enter stops.
"""
import importlib.util
import os
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


def main():
    fsm.AUTO_START = None                       # no START at 1 s: the CLI gives the missions
    cli = cli_module.DeliveryCli(START, HEADING, N, G)
    # Connect to Webots first: the robot is created here, and this waits until Webots offers it.
    with console.status("connecting to Webots (robot Cherokey_4W_v3, world grid_5x5_extern.wbt). Stuck here? "
                        "In Webots: reset the simulation (⟲), then press ▶"):
        hw = cli_hardware(WebotsHardware, cli)(fsm.TIME_STEP)
    console.print("[green]connected to Webots[/]")
    start_cli(START, HEADING, N, G, cli)
    try:
        fsm.main(hw, live_view=False)
    except KeyboardInterrupt:
        console.print("\nstopped (Ctrl+C)")
    console.print(cli.summary())


if __name__ == "__main__":
    main()
