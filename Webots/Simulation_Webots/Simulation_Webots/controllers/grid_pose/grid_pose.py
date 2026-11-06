"""Supervisor of grid_5x5.wbt: the true pose of the robot (DEF ST5) on the 5x5 grid.

Checks the FSM's counted pose and node centres against the truth (step 0 of "FSM única e grid 5x5").
Prints the pose every second and, for each pass within NEAR of a node centre, the closest distance to
that centre (when the robot leaves it, or when the simulation ends). Writes grid_pose.csv
(t, x, y, heading, i, j) in this folder.
Node (i, j), i east and j north, is at ((i - 2) G, (j - 2) G), as in worlds/make_grid_5x5.py.
GRID_START="i j H" (set by controllers/fsm/cli_webots.py): first puts the robot on node (i, j) facing H (E, N, W, S).
"""
import csv
import math
import os

from controller import Supervisor

N, G, NEAR = 5, 0.50, 0.03   # nodes per side, m between nodes, m "at the node"
PRINT_EVERY = 1.0            # s

sup = Supervisor()
dt = int(sup.getBasicTimeStep())
robot = sup.getFromDef("ST5")
if os.environ.get("GRID_START"):
    i0, j0, h0 = os.environ["GRID_START"].split()
    z = robot.getField("translation").getSFVec3f()[2]
    robot.getField("translation").setSFVec3f([(int(i0) - (N - 1) / 2) * G, (int(j0) - (N - 1) / 2) * G, z])
    robot.getField("rotation").setSFRotation([0, 0, 1, "ENWS".index(h0.upper()) * math.pi / 2])
    robot.resetPhysics()
    print(f"[POSE] start: robot put on node ({i0}, {j0}) facing {h0.upper()}")
path =os.path.join(os.path.dirname(os.path.abspath(__file__)), "grid_pose.csv")
with open(path, "w", newline="", buffering=1) as f:
    log = csv.writer(f)
    log.writerow(["t", "x", "y", "heading_deg", "i", "j"])
    t_print, at, best = -PRINT_EVERY, None, None    # node being passed, (closest distance, t, heading)

    def report():
        print(f"[POSE] t={best[1]:6.2f}s node {at}: closest {100 * best[0]:.1f} cm from its centre, "
              f"heading {best[2]:+.0f} deg ({'ENWS'[round(best[2] / 90) % 4]})")

    while sup.step(dt) != -1:
        t, (x, y, _), r = sup.getTime(), robot.getPosition(), robot.getOrientation()
        h = math.degrees(math.atan2(r[3], r[0]))     # robot forward axis in the world, 0 = east, + = left
        i, j = x / G + (N - 1) / 2, y / G + (N - 1) / 2
        node = (round(i), round(j))
        off = G * math.hypot(i - node[0], j - node[1])
        near = off < NEAR and all(0 <= k < N for k in node)
        log.writerow([f"{t:.3f}", f"{x:.4f}", f"{y:.4f}", f"{h:.1f}", f"{i:.3f}", f"{j:.3f}"])
        if at is not None and (not near or node != at):    # left the node: report its closest pass
            report()
            at = None
        if near and (at is None or off < best[0]):
            at, best = node, (off, t, h)
        if t - t_print >= PRINT_EVERY:
            print(f"[POSE] t={t:6.2f}s x={x:+.3f} y={y:+.3f} heading {h:+.0f} deg, grid ({i:.2f}, {j:.2f})")
            t_print = t
if at is not None:
    report()
