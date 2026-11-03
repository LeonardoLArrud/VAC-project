"""Basic FSM of the ST5 robot (v1): IDLE, FOLLOW, WAIT, SEARCH, TURN.

Imports nothing from Webots: takes one Reading per cycle and returns (v, w).
The same file runs in Webots, on the real robot and in the offline test.

WAIT -> TURN(180 deg): if the obstacle is still there after T_STATIC, the robot
turns in place until the line it came from is centred, then follows it back.
If it is blocked again right after a U-turn, it gives up (IDLE).

How it grows without a rewrite: v2..v5 only add lines to _priorities(),
one method per new state (EMERGENCY, NODE, AVOID), an entry in
FORWARD_STATES, and new uses of TURN (e.g. +90/-90 deg at nodes).
"""
from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass

from line_follower import LOST, OK, V0

IDLE, FOLLOW, WAIT, SEARCH, TURN = "IDLE", "FOLLOW", "WAIT", "SEARCH", "TURN"
FORWARD_STATES = {FOLLOW}  # v3: + NODE; v5: + AVOID


@dataclass
class Params:
    D_STOP: float = 0.25        # m, stop in front of an obstacle
    D_CLEAR: float = 0.35       # m, hysteresis: clear only above this
    D_SLOW: float = 0.60        # m, start slowing down
    SLOW_FRACTION: float = 0.25  # speed ceiling at D_STOP, fraction of V0
    T_CLEAR: float = 1.0        # s, continuously clear before moving again
    T_STATIC: float = 5.0       # s, still blocked after this: U-turn
    MAX_U_TURNS: int = 1        # U-turns in a row before giving up (blocked both ways)
    T_MIN: float = 0.3          # s, minimum time before a navigation transition
    N_OK: int = 3               # consecutive frames to accept the line again
    SEARCH_ANGLE: float = math.radians(60)  # search sweep, each side
    SEARCH_RATE: float = 1.0    # rad/s, search turn rate
    TURN_RATE: float = 1.0      # rad/s, turn in place
    TURN_WINDOW: float = math.radians(45)  # accept the line only after |theta| - this
    TURN_TOL: float = math.radians(10)     # line centred: target this close to the axis
    N_CENTERED: int = 2         # consecutive frames with the line centred


@dataclass
class Reading:
    t: float         # s
    image: object    # BGRA camera image
    dist: float      # m, raw rangefinder distance (the FSM filters it)
    heading: float   # rad, accumulated angle from the encoders
    cmd: str | None  # "START", "STOP" or None (command() hook: key, button or central)


class FSM:
    def __init__(self, follower, p=None, on_change=None):
        self.follower = follower
        self.p = p or Params()
        self.on_change = on_change or (lambda t, old, new, reason: None)
        self.state, self.reason = IDLE, "start"
        self.t = self.t_enter = 0.0
        self.v = self.w = 0.0          # last command actually applied
        self.d = float("inf")          # filtered distance (median of 3)
        self._dists = deque(maxlen=3)
        self.heading = 0.0
        self.line = LOST               # last follower status
        self.side = 1.0
        self.saved = None              # state interrupted by WAIT
        self.u_turns = 0               # U-turns in a row without the obstacle clearing
        self.history = []              # (t, old, new, reason)
        self.actions = {IDLE: self._idle, FOLLOW: self._follow, WAIT: self._wait,
                        SEARCH: self._search, TURN: self._turn}

    # ------------------------------------------------------------ cycle
    def step(self, r: Reading):
        self.t, self.heading = r.t, r.heading
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
        # 5. navigation: inside each state (v3: node_seen before line_lost)

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
            self.u_turns = 0
            self._go(FOLLOW, "START command")
        return 0.0, 0.0

    def _follow(self, r):
        out = self.follower.step(r.image, self._v_max(), self.v, self.w)
        self.line, self.side = out.status, out.side
        # v3: if node_seen: self._go(NODE, ...)   (before line_lost)
        if out.status == LOST and self._time_in_state() >= self.p.T_MIN:
            self._go(SEARCH, "line lost")
            return 0.0, 0.0
        return out.v, out.w

    def _wait(self, r):
        p = self.p
        if self.d > p.D_CLEAR:
            self.t_clear = self.t if self.t_clear is None else self.t_clear
            if self.t - self.t_clear >= p.T_CLEAR:
                self.u_turns = 0
                self._go(self.saved or FOLLOW, "obstacle cleared")
        else:
            self.t_clear = None
            if self._time_in_state() >= p.T_STATIC:   # v4: send BLOCKED, wait for the central
                if self.u_turns >= p.MAX_U_TURNS:
                    self._go(IDLE, "blocked both ways")
                else:
                    self.u_turns += 1
                    self._start_turn(math.pi, "still blocked after 5 s: U-turn")
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
            if self.n_ok >= p.N_CENTERED:
                self._go(FOLLOW, "turn done, line centred")
            return 0.0, 0.0
        self.n_ok = 0
        if turned >= goal + p.TURN_WINDOW:
            self._go(SEARCH, "turned past the window without the line")
            return 0.0, 0.0
        return 0.0, math.copysign(p.TURN_RATE, self.turn_goal)
