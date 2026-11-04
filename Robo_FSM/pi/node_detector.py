"""Node detector: "+", T and L intersections 0.40-0.65 m ahead. step(image, s) -> (d, exits) once per node.

Works on the floor, in metres, along the line itself (robust to the robot coming in at an angle):
1. its own mask of the rows that see the floor: blur, then Otsu (adapts to the light), never below
   the floor median + CONTRAST (no tape in view: no white)
2. the line: centres of the narrow white runs, fitted with a straight line y = a + b x
3. every STEP along the line, three strips across it: left (SIDE), right (-SIDE) and centre (+-AXIS)
4. a bar = FULL of the left and/or right strip white, over a thickness within THICK: exits L and/or R;
   exit S = the line goes on beyond the bar (BEYOND)
A curve or a glare does not fill a straight strip 4-12 cm beside the line over 1-6 cm: not a bar.
"""
import math

import cv2
import numpy as np

from line_follower import H_CAM, TILT, WIDTH, X_CAM, to_grey

ZONE = (0.40, 0.65)          # m ahead: where a bar is looked for; announced at ZONE[0]
SIDE = (0.04, 0.12)          # m beside the line centre: strip where a bar is looked for
FULL = 0.8                   # white fraction of that strip for a bar
THICK = (0.012, 0.06)        # m, bar thickness accepted (tape 0.025 + blur)
LINE_W = 0.045               # m, widest white run still taken as the line itself
AXIS = 0.02                  # m, exit S: centre strip half width
BEYOND = (0.05, 0.15)        # m beyond the bar where the line is looked for (exit S)
MAX_ANGLE = math.radians(20) # the line must be this close to the robot axis: none while spinning or askew
CONTRAST = 30                # grey levels above the floor median for "white"
N_FRAMES, S_TOL, SAME = 3, 0.03, 0.10
MISS = 1                     # frames without the bar tolerated while confirming
try:  # Robo_FSM: params.ini [node_detector]; the Webots controller has no params.py
    from params import load
    load("node_detector", globals())
except ImportError:
    pass

STEP = 0.004                 # m between two cuts across the line
NEAR = ZONE[0] - 0.12        # m, nearest floor used for the line fit
FAR = ZONE[1] + BEYOND[1]    # m, farthest floor used


class NodeDetector:
    def __init__(self, width, height, fov, grey=None):
        """grey: LineFollower.grey, so the image is turned grey once per frame for both."""
        self.p = max(1, width // WIDTH)
        self.nu, nv = width // self.p, height // self.p
        self.grey = grey or (lambda img: to_grey(img, self.p, nv, self.nu))
        self.f = (self.nu / 2) / math.tan(fov / 2)
        self.cu, self.cv = (self.nu - 1) / 2, (nv - 1) / 2
        self.r0, self.r1 = (min(max(round(self._row(x)), 0), nv - 1) for x in (FAR, NEAR))
        q = (np.arange(self.r0, self.r1 + 1) - self.cv) / self.f
        self.x = X_CAM + H_CAM / np.tan(np.arctan(q) + TILT)     # m ahead, per ROI row
        self.m = H_CAM / (self.f * (math.sin(TILT) + q * math.cos(TILT)))   # m per px across, per ROI row
        self.xs = np.arange(ZONE[0], FAR + 1e-9, STEP)         # cuts across the line
        self.o_side = np.linspace(SIDE[0], SIDE[1], 25)
        self.o_axis = np.linspace(-AXIS, AXIS, 9)
        self.cand, self.last = None, -1.0         # [sum of s + d, frames, exits, misses]; last s_node
        self.why = ""                            # find(): why the last image gave no bar, or what it found

    def _row(self, x):
        """m ahead -> image row (follower pixels)."""
        return self.cv + self.f * math.tan(math.atan2(H_CAM, x - X_CAM) - TILT)

    def step(self, image, s, announce=True):
        """announce=False: keeps tracking the bar but announces nothing (the FSM is not following the line:
        a node announced then would be lost, and taken as already seen afterwards)."""
        bar, c = self.find(image), self.cand
        if bar and abs(s + bar[0] - self.last) > SAME:
            if c and abs(s + bar[0] - c[0] / c[1]) <= S_TOL:
                c[0], c[1], c[2], c[3] = c[0] + s + bar[0], c[1] + 1, c[2] | bar[1], 0
            elif not c or c[1] < N_FRAMES:
                self.cand = [s + bar[0], 1, bar[1], 0]
        elif c and c[1] < N_FRAMES:               # bar lost while confirming: tolerate MISS frames
            c[3] += 1
            if c[3] > MISS:
                self.cand = None
        elif c and c[0] / c[1] - s > ZONE[0] + 0.05:   # confirmed but not seen where it should be
            self.cand = None
        c = self.cand
        if announce and c and c[1] >= N_FRAMES and c[0] / c[1] - s <= ZONE[0]:
            self.cand, self.last = None, c[0] / c[1]
            return self.last - s, c[2]
        return None

    def forget(self):
        """Drop the node being confirmed (the robot spun in place: its position no longer holds)."""
        self.cand = None

    def white(self, image):
        """Mask of the ROI rows (bool): blur + Otsu, at least CONTRAST above the floor median."""
        g = cv2.blur(self.grey(image)[self.r0 : self.r1 + 1], (3, 3))
        otsu = cv2.threshold(g, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)[0]
        self.threshold = max(otsu, float(np.median(g)) + CONTRAST)
        return g > self.threshold

    def _line(self, w):
        """(a, b, x of its points) of the line y = a + b x (m), from the narrow white runs; None if not seen."""
        pad = np.zeros((len(w), w.shape[1] + 2), np.int8)
        pad[:, 1:-1] = w
        rows, cols = np.nonzero(np.diff(pad, axis=1))     # run edges, row by row: start, end, start, end...
        r, start, end = rows[::2], cols[::2], cols[1::2]
        width = (end - start) * self.m[r]
        ok = (width >= 0.01) & (width <= LINE_W)          # narrow runs: the line
        r, y = r[ok], -self.m[r[ok]] * ((start[ok] + end[ok] - 1) / 2 - self.cu)
        if len(r) < 4:
            self.why = f"no line: {len(r)} white runs 1-{100 * LINE_W:.1f} cm wide (needs 4)"
            return None
        order = np.lexsort((np.abs(y), r))                # per row, the run nearest the robot axis first
        first = order[np.r_[True, np.diff(r[order]) != 0]]
        if len(first) < 4:
            self.why = f"no line: narrow runs on {len(first)} rows (needs 4)"
            return None
        x, y = self.x[r[first]], y[first]
        b, a = np.polyfit(x, y, 1)
        keep = np.abs(y - (a + b * x)) <= 0.02      # drop runs that are not the line
        if keep.sum() < 4:
            self.why = f"no line: only {keep.sum()} runs on a straight line (needs 4)"
            return None
        b, a = np.polyfit(x[keep], y[keep], 1)
        return a, b, x[keep]

    def _strip(self, w, a, b, offsets):
        """White fraction of a strip across the line at each cut (offsets: m along the normal, + = left)."""
        n = math.hypot(1.0, b)
        x = self.xs[:, None] - b * offsets[None, :] / n
        y = (a + b * self.xs)[:, None] + offsets[None, :] / n
        q = np.tan(np.arctan2(H_CAM, x - X_CAM) - TILT)
        v = np.round(self.cv + self.f * q).astype(int) - self.r0
        u = np.round(self.cu - y * self.f * (math.sin(TILT) + q * math.cos(TILT)) / H_CAM).astype(int)
        ok = (v >= 0) & (v < len(w)) & (u >= 0) & (u < self.nu)
        hit = np.zeros(ok.shape, bool)
        hit[ok] = w[v[ok], u[ok]]
        n_ok = ok.sum(axis=1)
        return np.where(n_ok >= 0.4 * len(offsets), hit.sum(axis=1) / np.maximum(n_ok, 1), 0.0)

    def find(self, image):
        """(d m ahead, exits) of the nearest bar in ZONE in this image, or None."""
        w = self.white(image)
        line = self._line(w)
        if line is None:
            return None
        a, b, xl = line
        if abs(math.atan(b)) > MAX_ANGLE:          # spinning (s does not move: anything would confirm) or askew
            self.why = f"line at {math.degrees(math.atan(b)):+.0f} deg: robot not along the line (max {math.degrees(MAX_ANGLE):.0f})"
            return None
        left, right = self._strip(w, a, b, self.o_side), self._strip(w, a, b, -self.o_side)
        side = (left >= FULL) | (right >= FULL)
        k = np.flatnonzero(side & (self.xs <= ZONE[1]))
        line_txt = f"line at {100 * a:+.1f} cm, {math.degrees(math.atan(b)):+.0f} deg"
        if not len(k):
            zone = self.xs <= ZONE[1]
            self.why = (f"no bar: {line_txt}; whitest strip beside it in the zone: left {left[zone].max():.0%}, "
                        f"right {right[zone].max():.0%} (needs {FULL:.0%})")
            return None
        k = k[: np.argmax(np.r_[np.diff(k) > 2, True]) + 1]      # nearest bar: cuts until the first gap
        thick = (k[-1] - k[0] + 1) * STEP
        d = float(self.xs[k].mean())
        if not THICK[0] <= thick <= THICK[1]:     # not a tape
            self.why = f"bar at {100 * d:.0f} cm rejected: {100 * thick:.1f} cm thick (accepted {100 * THICK[0]:.1f}-{100 * THICK[1]:.1f})"
            return None
        if (xl < d - 0.03).sum() < 3:             # no line coming to it
            self.why = f"bar at {100 * d:.0f} cm rejected: no line before it"
            return None
        exits = {e for e, f in (("L", left), ("R", right)) if (f[k] >= FULL).mean() >= 0.5}
        beyond = (self.xs >= d + BEYOND[0]) & (self.xs <= d + BEYOND[1]) & ~side
        if ((self._strip(w, a, b, self.o_axis) >= 0.5) & beyond).sum() >= 3:
            exits.add("S")
        self.why = f"bar at {100 * d:.1f} cm, {100 * thick:.1f} cm thick, exits {''.join(sorted(exits))}; {line_txt}"
        return d, exits
