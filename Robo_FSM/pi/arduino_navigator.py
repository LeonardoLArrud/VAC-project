"""Mission of navigators.py running on the Arduino (robo_fsm.ino + navigator.cpp, the same code in C++).

Same methods as navigators.Mission: each one sends one command to the Arduino and returns its
answer (order, reason). prior() returns what the last answer said, without asking again.
Used by fsm.py when params.ini [fsm] NAV_ON_ARDUINO = True.
"""
from navigators import Navigator

BITS = {"S": 1, "L": 2, "R": 4}
FAULTS = {"node missed": 0, "turn failed": 1}


class ArduinoMission(Navigator):
    def __init__(self, hw, start, heading, goal, n=5, g=0.50):
        self.hw, self._prior = hw, None
        order, why = self._ask("N%d,%d,%d,%d,%d,%d,%d" % (start[0], start[1], "ENWS".index(heading),
                                                          goal[0], goal[1], n, round(1000 * g)))
        if order == "STOP":
            raise ValueError(f"the Arduino refused the mission: {why}")

    def _ask(self, command):
        _, order, known, s_next, exits, why = self.hw.ask(command).split(",", 5)
        self._prior = None if known == "0" else (int(s_next) / 1000, {e for e, b in BITS.items() if int(exits) & b})
        return order, why

    def on_node(self, exits, s_node=0.0):
        mask = -1 if exits is None else sum(BITS[e] for e in exits)
        return self._ask("C%d,%d" % (mask, round(1000 * s_node)))

    def on_blocked(self, s, d):
        return self._ask("B%d,%d" % (round(1000 * s), round(1000 * d)))

    def on_fault(self, kind):
        if kind not in FAULTS:
            return "GO_ON", kind
        return self._ask("F%d" % FAULTS[kind])

    def prior(self):
        return self._prior
