"""Mission (Robo_FSM/pi/navigators.py): relocation when the exits seen at a node do not fit the map."""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "Robo_FSM", "pi"))
from navigators import Mission

G = 0.975


def started(start, heading, goal=(4, 3)):
    m = Mission(start, heading, goal, 5, G)
    m.on_node(None, 0.0)            # the FSM does this at MISSION: the start node, exits from the map
    return m


def test_exits_that_fit_keep_the_pose():
    m = started((0, 0), "N")
    order, why = m.on_node({"S", "R"}, G)
    assert m.node == (0, 1) and "relocated" not in why and order in ("STRAIGHT", "RIGHT")


def test_wrong_start_heading_is_relocated():
    m = started((0, 0), "N")         # MISSION says N, the robot faces E: it reaches (1, 0), a bottom border node
    order, why = m.on_node({"S", "L"}, G)
    assert m.node == (1, 0) and m.h_in == 0 and why.startswith("relocated to (1, 0) facing E")
    assert order in ("STRAIGHT", "LEFT")


def test_no_neighbour_fits_stops():
    m = started((0, 0), "N")
    order, why = m.on_node({"L", "R"}, G)     # (0, 1) has no L, (1, 0) facing E has no R
    assert order == "STOP" and "pose lost" in why


def test_several_neighbours_fit_stops():
    m = started((2, 4), "E")         # top border: (3, 4) facing E has S and R
    order, why = m.on_node({"S", "L"}, G)     # fits (1, 4) facing W and (2, 3) facing S: no guess
    assert order == "STOP" and "pose lost" in why
