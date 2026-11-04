"""FSM of the ST5 robot: IDLE, FOLLOW, NODE, WAIT, SEARCH, TURN (v1 + NODE).

Imports nothing from Webots: takes one Reading per cycle and returns (v, w).
The same file runs in Webots, on the real robot and in the offline test.

The FSM knows how to move; the navigator (navigators.py, chosen by the config)
knows where to go: on_node(exits) at a node, on_blocked() when WAIT is still blocked
after T_STATIC (Explorer: U-turn once, then IDLE, as in v1), on_fault(kind).
Without node_seen events (no node detector yet) the FSM is v1.

How it grows without a rewrite: v2..v5 only add lines to _priorities(),
one method per new state (EMERGENCY, NODE, AVOID), an entry in
FORWARD_STATES, and new uses of TURN (e.g. +90/-90 deg at nodes).
"""
from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass

from line_follower import LOST, OK, V0
from navigators import Explorer, Mission

IDLE, FOLLOW, NODE, WAIT, SEARCH, TURN = "IDLE", "FOLLOW", "NODE", "WAIT", "SEARCH", "TURN"
FORWARD_STATES = {FOLLOW, NODE}  # v5: + AVOID
TURNS = {"LEFT": math.pi / 2, "RIGHT": -math.pi / 2, "U_TURN": math.pi}   # order -> TURN(theta)


@dataclass
class Params:
    D_STOP: float = 0.25        # m, stop in front of an obstacle
    D_CLEAR: float = 0.35       # m, hysteresis: clear only above this
    D_SLOW: float = 0.60        # m, start slowing down
    SLOW_FRACTION: float = 0.25  # speed ceiling at D_STOP, fraction of V0
    T_CLEAR: float = 1.0        # s, continuously clear before moving again
    T_STATIC: float = 5.0       # s, still blocked after this: ask the navigator
    T_MIN: float = 0.3          # s, minimum time before a navigation transition
    N_OK: int = 3               # consecutive frames to accept the line again
    SEARCH_ANGLE: float = math.radians(60)  # search sweep, each side
    SEARCH_RATE: float = 1.0    # rad/s, search turn rate
    TURN_RATE: float = 1.0      # rad/s, turn in place
    TURN_WINDOW: float = math.radians(20)  # accept the line only within |theta| +- this (a bright reflection
                                           # half way through a turn is not the line)
    TURN_TOL: float = math.radians(10)     # line centred: target this close to the axis
    N_CENTERED: int = 2         # consecutive frames with the line centred
    V_NODE: float = 0.08        # m/s, NODE: straight to the node centre
    K_HOLD: float = 2.0         # 1/s, NODE: heading hold gain
    D_STRAIGHT: float = 0.20    # m, STRAIGHT: back to FOLLOW this far before the centre
    PRIOR_TOL: float = 0.18     # m, a node seen farther than this from prior() is ignored
    D_SEE: float = 0.40         # m, a node closer than this cannot be seen any more (node_seen is sent at 0.40 m)


@dataclass
class Reading:
    t: float         # s
    image: object    # BGRA camera image
    dist: float      # m, raw rangefinder distance (the FSM filters it)
    heading: float   # rad, accumulated angle from the encoders
    cmd: str | tuple | None  # "START", "STOP", ("MISSION", start, heading, goal) or None (key, button or central)
    s: float = 0.0   # m, distance rolled by the wheels (distance() hook)
    node: tuple | None = None  # node_seen: (d m ahead, exits {"L", "S", "R"}) or None


class FSM:
    def __init__(self, follower, p=None, on_change=None, nav=Explorer):
        self.follower = follower
        self.p = p or Params()
        self.on_change = on_change or (lambda t, old, new, reason: None)
        self.state, self.reason = IDLE, "start"
        self.t = self.t_enter = 0.0
        self.v = self.w = 0.0          # last command actually applied
        self.d = float("inf")          # filtered distance (median of 3)
        self._dists = deque(maxlen=3)
        self.heading = self.s = 0.0
        self.make_nav, self.nav = nav, nav()   # nav: navigator factory from the config
        self.s_node = self.psi_ref = 0.0       # NODE: centre of the node seen, heading to hold
        self.order, self.why = "STOP", ""      # NODE: navigator's answer
        self.line = LOST               # last follower status
        self.side = 1.0
        self.saved = None              # state interrupted by WAIT
        self.history = []              # (t, old, new, reason)
        self.actions = {IDLE: self._idle, FOLLOW: self._follow, NODE: self._node, WAIT: self._wait,
                        SEARCH: self._search, TURN: self._turn}

    # ------------------------------------------------------------ cycle
    def step(self, r: Reading):
        self.t, self.heading, self.s = r.t, r.heading, r.s
        self._dists.append(r.dist)
        self.d = sorted(self._dists)[len(self._dists) // 2]
        self._priorities(r)
        self.v, self.w = self.actions[self.state](r)
        return self.v, self.w

    def _priorities(self, r):
        # 1. v2: ESTOP or Arduino fault -> EMERGENCY (latched)
        # 2. STOP command, from any state
        if r.cmd == "STOP" and self.state != IDLE:
            self._go(IDLE, "STOP command")
            return
        # 3. v2: no message from the central for T_COMM -> IDLE
        # 4. obstacle near, only in states that move forward
        if self.state in FORWARD_STATES and self.d < self.p.D_STOP:
            self.saved = self.state
            self._go(WAIT, f"obstacle at {self.d:.2f} m")
        # 5. navigation: inside each state (FOLLOW: node_seen before line_lost)

    def _go(self, new, reason):
        self.history.append((self.t, self.state, new, reason))
        self.on_change(self.t, self.state, new, reason)
        self.state, self.reason, self.t_enter = new, reason, self.t
        if new == WAIT:
            self.t_clear = None
        elif new == SEARCH:
            a = self.p.SEARCH_ANGLE * self.side
            self.goals = [a, -a, 0.0]          # relative to the heading at entry
            self.heading0, self.n_ok = self.heading, 0
            self._next_leg()
        elif new == TURN:
            self.heading0, self.n_ok = self.heading, 0

    def _start_turn(self, theta, reason):
        self.turn_goal = theta                 # rad, + = left (counter-clockwise)
        self._go(TURN, reason)

    def _time_in_state(self):
        return self.t - self.t_enter

    def _v_max(self):
        p = self.p
        f = (self.d - p.D_STOP) / (p.D_SLOW - p.D_STOP)
        return V0 * min(1.0, max(p.SLOW_FRACTION, f))

    # ------------------------------------------------------------ states
    def _idle(self, r):
        if r.cmd == "START":
            self.nav = self.make_nav()         # START without a mission: navigator from the config
            self._go(FOLLOW, "START command")
        elif isinstance(r.cmd, tuple) and r.cmd[0] == "MISSION":   # MISSION(start, heading, goal)
            self.nav = Mission(*r.cmd[1:])
            self.s_node, self.psi_ref = self.s, self.heading
            self.order, self.why = self.nav.on_node(None, self.s)  # decide at once on the start node
            self._go(NODE, f"MISSION {r.cmd[1:]}: {self.order} ({self.why})")
        return 0.0, 0.0

    def _follow(self, r):
        out = self.follower.step(r.image, self._v_max(), self.v, self.w)
        self.line, self.side = out.status, out.side
        prior = self.nav.prior()               # None or (s_next, exits)
        if r.node:                             # node_seen, before line_lost
            d, exits = r.node
            if prior and abs(self.s + d - prior[0]) > self.p.PRIOR_TOL:
                self.on_change(self.t, FOLLOW, FOLLOW, f"false node at {d:.2f} m ignored")
            else:
                self.s_node = self.s + d                  # hold the heading that points at the line ahead
                self.psi_ref = self.heading + (math.atan2(out.target[1], out.target[0]) if out.target else 0.0)
                self.order, self.why = self.nav.on_node(exits, self.s_node)
                self._go(NODE, f"node at {d:.2f} m, exits {''.join(sorted(exits))}: {self.order} ({self.why})")
                return out.v, out.w
        elif prior and self.s > prior[0] - self.p.D_SEE + self.p.PRIOR_TOL:   # should have been seen by now
            order, why = self.nav.on_fault("node missed")
            self.on_change(self.t, FOLLOW, FOLLOW, why)
            if order == "STOP":
                self._go(IDLE, why)
                return 0.0, 0.0
        if out.status == LOST and self._time_in_state() >= self.p.T_MIN:
            self._go(SEARCH, "line lost")
            return 0.0, 0.0
        return out.v, out.w

    def _node(self, r):
        p = self.p
        if self.order == "STRAIGHT" and self.s >= self.s_node - p.D_STRAIGHT:
            self._go(FOLLOW, "bar passed")
        elif self.order != "STRAIGHT" and self.s >= self.s_node:
            if self.order in TURNS:
                self._start_turn(TURNS[self.order], f"at the node centre: {self.order}")
            else:
                self._go(IDLE, f"{self.order} at the node: {self.why}")
        else:
            return min(p.V_NODE, self._v_max()), p.K_HOLD * (self.psi_ref - self.heading)
        return 0.0, 0.0

    def _wait(self, r):
        p = self.p
        if self.d > p.D_CLEAR:
            self.t_clear = self.t if self.t_clear is None else self.t_clear
            if self.t - self.t_clear >= p.T_CLEAR:
                self.nav.on_clear()
                self._go(self.saved or FOLLOW, "obstacle cleared")
        else:
            self.t_clear = None
            if self._time_in_state() >= p.T_STATIC:   # v4: send BLOCKED to the central
                order, why = self.nav.on_blocked(self.s, self.d)
                if order == "U_TURN":
                    self._start_turn(math.pi, why)
                elif order == "WAIT":
                    self.t_enter = self.t          # wait longer: the timer restarts
                else:
                    self._go(IDLE, why)
        return 0.0, 0.0

    def _next_leg(self):
        self.direction = 1.0 if self.goals[0] >= self.heading - self.heading0 else -1.0

    def _search(self, r):
        p = self.p
        out = self.follower.step(r.image, 0.0, 0.0, self.w)   # perception only: v = 0
        self.line = out.status
        if out.status == OK:               # line seen: stop turning and confirm
            self.n_ok += 1
            if self.n_ok >= p.N_OK and self._time_in_state() >= p.T_MIN:
                self._go(FOLLOW, "line found")
            return 0.0, 0.0
        self.n_ok = 0
        if (self.goals[0] - (self.heading - self.heading0)) * self.direction <= 0:
            self.goals.pop(0)              # reached this end of the sweep
            if not self.goals:
                self._go(IDLE, "line lost (search exhausted)")
                return 0.0, 0.0
            self._next_leg()
        return 0.0, self.direction * p.SEARCH_RATE

    def _turn(self, r):
        p = self.p
        out = self.follower.step(r.image, 0.0, 0.0, self.w)   # perception only: v = 0
        self.line = out.status
        turned, goal = abs(self.heading - self.heading0), abs(self.turn_goal)
        centred = (out.status == OK and abs(math.atan2(out.target[1], out.target[0])) < p.TURN_TOL)
        if turned >= goal - p.TURN_WINDOW and centred:   # encoders open the window, camera closes it
            self.n_ok += 1                 # stop turning and confirm
            prior = self.nav.prior()
            if self.n_ok >= p.N_CENTERED and prior and prior[0] - self.s < p.D_SEE:   # e.g. after a U-turn
                self.s_node, self.psi_ref = prior[0], self.heading
                self.order, self.why = self.nav.on_node(None, prior[0])            # too close to see: map exits
                self._go(NODE, f"turn done, node {prior[0] - self.s:.2f} m ahead: {self.order} ({self.why})")
            elif self.n_ok >= p.N_CENTERED:
                self._go(FOLLOW, "turn done, line centred")
            return 0.0, 0.0
        self.n_ok = 0
        if turned >= goal + p.TURN_WINDOW:
            order, why = self.nav.on_fault("turn failed")
            if order == "STOP":
                self._go(IDLE, why)
            else:
                self._go(SEARCH, "turned past the window without the line")
            return 0.0, 0.0
        return 0.0, math.copysign(p.TURN_RATE, self.turn_goal)
