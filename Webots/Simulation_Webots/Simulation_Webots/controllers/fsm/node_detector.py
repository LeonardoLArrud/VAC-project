"""Node detector: the bar of an intersection 0.40-0.65 m ahead. step(image, s) -> (d, exits) once per node."""
import math

import numpy as np

from line_follower import H_CAM, THRESHOLD, TILT, WIDTH, X_CAM

ZONE = (0.40, 0.65)          # m ahead
REACH, AXIS, LINE_W = 0.06, 0.04, 0.04   # m
THICK = (0.01, 0.03)         # m, bar thickness
BEYOND = (0.05, 0.15)        # m, where S is looked for
N_FRAMES, S_TOL, SAME = 3, 0.03, 0.10
MISS = 1                     # frames without the bar tolerated while confirming
PERP_TOL = math.radians(15)
try:  # Robo_FSM: params.ini [node_detector]; the Webots controller has no params.py
    from params import load
    load("node_detector", globals())
except ImportError:
    pass


class NodeDetector:
    def __init__(self, width, height, fov, mask=None):
        self.p = max(1, width // WIDTH)
        self.mask = mask or (lambda img: img[:: self.p, :: self.p, :3].mean(axis=2) > THRESHOLD)
        nu, nv = width // self.p, height // self.p
        f = (nu / 2) / math.tan(fov / 2)
        q = (np.arange(nv) - (nv - 1) / 2) / f
        a = np.arctan(q) + TILT
        x = np.where(a > 0, X_CAM + H_CAM / np.tan(np.maximum(a, 1e-6)), np.inf)
        self.rows = np.flatnonzero((x >= ZONE[0]) & (x <= ZONE[1]))[::-1]
        self.x = x[self.rows]
        self.m = (H_CAM / (f * (math.sin(TILT) + q * math.cos(TILT))))[self.rows]   # m per px
        self.cu = (nu - 1) / 2
        self.cand, self.last = None, -1.0         # [sum of s + d, frames, exits, misses]; last s_node

    def step(self, image, s):
        bar, c = self._bar(self.mask(image)[self.rows] > 0), self.cand
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
        if c and c[1] >= N_FRAMES and c[0] / c[1] - s <= ZONE[0]:
            self.cand, self.last = None, c[0] / c[1]
            return self.last - s, c[2]
        return None

    def _bar(self, g):
        yl, line, bar, n_s = None, [], [], 0
        for i, row in enumerate(g):
            e = np.flatnonzero(np.diff(np.r_[0, row.astype(np.int8), 0]))
            ya, yb = -self.m[i] * (e[::2] - self.cu), -self.m[i] * (e[1::2] - 1 - self.cu)
            if yl is None:
                k = np.flatnonzero(ya - yb <= LINE_W)
                k = k[np.argmin(np.abs(ya[k] + yb[k]))] if len(k) else None
            else:                                 # run connected to the line
                k = np.flatnonzero((ya >= yl - 0.01) & (yb <= yl + 0.01))
                k = k[0] if len(k) else None
            if k is None:
                continue
            narrow, centre = ya[k] - yb[k] <= LINE_W, (ya[k] + yb[k]) / 2
            left, right = yl is not None and ya[k] - yl > REACH, yl is not None and yl - yb[k] > REACH
            if (left or right) and (not bar or bar[-1][0] == i - 1):
                bar.append((i, left, right))
            elif bar:                             # beyond the bar: S?
                d = (self.x[bar[0][0]] + self.x[bar[-1][0]]) / 2
                n_s += narrow and abs(centre - yl) < AXIS and BEYOND[0] <= self.x[i] - d <= BEYOND[1]
            elif narrow:
                yl = centre
                line.append((self.x[i], yl))
        if not bar:
            return None
        i0, i1 = bar[0][0], bar[-1][0]
        x0, x1 = self.x[i0], self.x[i1]
        xs, ys = np.array([p for p in line if p[0] > x0 - 0.10] or [(0.0, 0.0)]).T
        if len(xs) < 3:
            return None
        blk, xb = g[max(i0 - 2, 0) : i1 + 3], self.x[max(i0 - 2, 0) : i1 + 3]
        y = -self.m[(i0 + i1) // 2] * (np.arange(g.shape[1]) - self.cu) - yl
        cols = blk.any(axis=0) & (np.abs(y) >= 0.03) & (np.abs(y) <= 0.10)
        if cols.sum() < 5:
            return None
        xc = (blk[:, cols] * xb[:, None]).sum(axis=0) / blk[:, cols].sum(axis=0)
        top, low = np.where(blk[:, cols], xb[:, None], -np.inf).max(axis=0), np.where(blk[:, cols], xb[:, None], np.inf).min(axis=0)
        if not THICK[0] <= np.median(top - low) + (self.x[min(i0 + 1, len(self.x) - 1)] - x0) <= THICK[1]:
            return None                           # not a tape
        if abs(math.atan(np.polyfit(xs, ys, 1)[0]) + math.atan(np.polyfit(y[cols], xc, 1)[0])) > PERP_TOL:
            return None                           # not perpendicular: a curve
        exits = {"L"} if any(b[1] for b in bar) else set()
        exits |= ({"R"} if any(b[2] for b in bar) else set()) | ({"S"} if n_s >= 2 else set())
        return (x0 + x1) / 2, exits
