"""Supervisor: shows the mission on the grid with floating spheres. Blue = start, red = goal,
green = planned path (redrawn at every node), orange = edges the robot found blocked or missing.

Reads controllers/fsm/mission.json, written by fsm.py at every decision of the Mission navigator.
The spheres float above anything the camera sees, have no boundingObject (the sonar ignores them)
and cast no shadow, so they never change what the robot perceives.
Node (i, j), i east and j north, is at ((i - (n - 1) / 2) g, (j - (n - 1) / 2) g): the grid worlds are centred.
"""
import json
import os

from controller import Supervisor

VIEW = os.environ.get("FSM_VIEW") or os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "fsm", "mission.json")
Z = 0.35                       # m above the floor: the camera only looks down


def at(ij, v):                 # node (i, j) -> world (x, y)
    c = (v["n"] - 1) / 2
    return (ij[0] - c) * v["g"], (ij[1] - c) * v["g"]


def ball(x, y, r, color):
    return (f"Pose {{ translation {x:.3f} {y:.3f} {Z} children [ Shape {{ castShadows FALSE appearance "
            f"PBRAppearance {{ baseColor {color} emissiveColor {color} roughness 1 metalness 0 }} "
            f"geometry Sphere {{ radius {r:.3f} }} }} ] }}")


sup = Supervisor()
dt = int(sup.getBasicTimeStep())
sup.getRoot().getField("children").importMFNodeFromString(-1, "DEF MISSION_VIEW Group { }")
markers, stamp = sup.getFromDef("MISSION_VIEW").getField("children"), None
while sup.step(dt) != -1:
    try:
        if os.path.getmtime(VIEW) == stamp:
            continue
        stamp = os.path.getmtime(VIEW)
        with open(VIEW) as f:
            v = json.load(f)
    except (OSError, ValueError):            # no mission yet
        continue
    while markers.getCount():
        markers.removeMF(0)
    balls = [(at(ij, v), 0.05, "0.1 0.8 0.1") for ij in v["path"][1:-1]]
    balls += [(at(((a[0] + b[0]) / 2, (a[1] + b[1]) / 2), v), 0.08, "1 0.5 0") for a, b in v["closed"]]
    balls += [(at(v["start"], v), 0.08, "0.1 0.3 1"), (at(v["goal"], v), 0.12, "1 0.1 0.1")]
    for (x, y), r, color in balls:
        markers.importMFNodeFromString(-1, ball(x, y, r * v["g"], color))   # sizes for g = 1 m, scaled
    print(f"[MISSION] goal {tuple(v['goal'])}, path {[tuple(p) for p in v['path']]}, closed {len(v['closed'])}")
