"""Line follower (ready-made block), packaged so the FSM can call it.

Same logic as the project's first pure pursuit controller:
- 4 bands on the floor (0.20 to 0.36 m); target (L, Y) = farthest valid band;
- pure pursuit: c = 2Y/(L^2 + Y^2), w = v*c;
- v limited by V0, delay, slip and outer wheel, with an acceleration limit;
- with no valid band, the last target is propagated by odometry for up to L.

Two differences, both to keep the follower separate from the FSM:
1. With no target it returns status LOST and v = w = 0. Searching for the
   line is the FSM's job (the first controller spun in place here).
2. Propagation and the acceleration limit use the motion the FSM actually
   applied (v_prev, w_prev), not what the follower proposed.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from numpy.lib.stride_tricks import sliding_window_view

H_CAM = 0.132                 # m, camera height
X_CAM = 0.104                 # m, camera ahead of the chassis centre
TILT = math.radians(30)       # camera tilt below horizontal
BANDS = [0.20, 0.26, 0.31, 0.36]  # m, distance of each band from the centre
K = 0.135                     # m, effective half track
V0 = 0.13                     # m/s, top speed
TAU_EFF = 0.10                # s, effective delay
A_LAT = 1.0                   # m/s^2, lateral acceleration limit
A_UP = 0.5                    # m/s^2, acceleration limit
A_DOWN = 1.5                  # m/s^2, deceleration limit
THRESHOLD = 150               # grey level: brighter = line
WIDTH = 480                   # px, processing width
# Image filter against floor reflections (same as Robo_FSM/pi/line_follower.py, numpy only, no OpenCV):
# blur, threshold, then an opening that removes white blobs narrower than OPEN_W or shorter than OPEN_H.
# Sizes in px of the 480x270 image: the huit.jpg line (~12 mm) is 16 px wide on the far band and
# 23 px on the near one; a crossing line on the far band is ~7 px tall. Odd sizes only.
FILTER = True
BLUR = 3
OPEN_W, OPEN_H = 9, 3

OK, MEMORY, LOST = "OK", "MEMORY", "LOST"


@dataclass
class FollowerOutput:
    status: str          # OK, MEMORY or LOST
    target: tuple | None  # (L, Y) in m, robot frame (x forward, y left)
    v: float             # m/s proposed
    w: float             # rad/s proposed
    side: float          # +1 if the line was last seen on the left, -1 on the right


class LineFollower:
    def __init__(self, cam_width, cam_height, fov, dt, wheel_v_max):
        self.dt = dt
        self.wheel_v_max = wheel_v_max
        self.px_step = max(1, cam_width // WIDTH)
        self.nu = WIDTH
        self.nv = round(cam_height * WIDTH / cam_width)
        f = (self.nu / 2) / math.tan(fov / 2)
        self.cu = (self.nu - 1) / 2
        cv = (self.nv - 1) / 2
        self.bands = []   # (distance, image row, m per px, half width)
        for dj in BANDS:
            q = math.tan(math.atan2(H_CAM, dj - X_CAM) - TILT)
            row = round(cv + f * q)
            if 2 <= row < self.nv - 2:   # with a 16:9 image the 0.20 m band is outside
                den = math.sin(TILT) + q * math.cos(TILT)
                self.bands.append((dj, row, H_CAM / (f * den), H_CAM * math.tan(fov / 2) / den))
        self.u_prev = [self.cu] * len(self.bands)
        self.memory = None
        self.L_lost = BANDS[-1]
        self.travelled = 0.0
        self.side = 1.0
        self.points = []  # (row, u, used) of the last image, in follower pixels: for the live view

    def _grey(self, img):
        p = self.px_step
        return img[::p, ::p, :3].mean(axis=2)[: self.nv, : self.nu]

    def mask(self, img):
        """1 where the follower sees the line (nv x nu, uint8): blur, threshold, then an opening."""
        g = self._grey(img).astype(np.uint8)
        if not FILTER:
            return (g > THRESHOLD).astype(np.uint8)
        m = (_blur(g, BLUR) > THRESHOLD).astype(np.uint8)
        return _morph(_morph(m, OPEN_H, OPEN_W, np.min, 1), OPEN_H, OPEN_W, np.max, 0)

    @staticmethod
    def _find_line(m, row, u_ref):
        """m: mask from mask(). Centre of the white run on this row closest to u_ref, or None."""
        white = (m[row - 2 : row + 3].mean(axis=0) > 0.5).astype(np.int8)
        edge = np.diff(np.concatenate(([0], white, [0])))
        start, end = np.flatnonzero(edge == 1), np.flatnonzero(edge == -1)
        ok = end - start >= 2
        if not ok.any():
            return None
        centres = (start[ok] + end[ok] - 1) / 2
        return float(centres[np.argmin(np.abs(centres - u_ref))])

    def step(self, img, v_max, v_prev, w_prev):
        """img: (height, width, 4) BGRA array. v_max: ceiling from the FSM, m/s."""
        g = self.mask(img)
        target, points = None, []
        for j in reversed(range(len(self.bands))):
            dj, row, mj, wj = self.bands[j]
            u = self._find_line(g, row, self.u_prev[j])
            if u is None:
                continue
            self.u_prev[j] = u
            y = -mj * (u - self.cu)
            used = abs(y) <= 0.7 * wj
            points.append((row, u, used))
            if used:
                target = (dj, y)
                break
        self.points = points

        status = OK
        if target is not None:
            self.memory = target
            self.L_lost = target[0]
            self.travelled = 0.0
        elif self.memory is not None:
            dt = self.dt
            dpsi = w_prev * dt
            dx = v_prev * dt * math.cos(dpsi / 2)
            dy = v_prev * dt * math.sin(dpsi / 2)
            x0, y0 = self.memory[0] - dx, self.memory[1] - dy
            self.memory = (math.cos(dpsi) * x0 + math.sin(dpsi) * y0,
                           -math.sin(dpsi) * x0 + math.cos(dpsi) * y0)
            self.travelled += abs(v_prev) * dt
            if self.memory[0] <= 0.05 or self.travelled > self.L_lost:
                self.memory = None
            target = self.memory
            status = MEMORY

        if target is None:
            return FollowerOutput(LOST, None, 0.0, 0.0, self.side)

        L, Y = target
        self.side = 1.0 if Y >= 0 else -1.0
        c = 2 * Y / (L * L + Y * Y)
        v_target = min(V0, v_max, 0.163 * L / TAU_EFF,
                       math.sqrt(A_LAT / max(abs(c), 1e-6)),
                       self.wheel_v_max / (1 + K * abs(c)))
        v = min(max(v_target, v_prev - A_DOWN * self.dt), v_prev + A_UP * self.dt)
        v = max(v, 0.0)
        return FollowerOutput(status, target, v, v * c, self.side)


def _blur(g, k):
    """Box blur k x k (as cv2.blur: borders reflected)."""
    p = np.pad(g.astype(np.float32), k // 2, mode="reflect")
    return sliding_window_view(sliding_window_view(p, k, axis=1).mean(axis=-1), k, axis=0).mean(axis=-1)


def _morph(m, kh, kw, fn, pad):
    """Erosion (fn=np.min, pad=1) or dilation (fn=np.max, pad=0) with a kh x kw rectangle."""
    p = np.pad(m, ((kh // 2, kh // 2), (kw // 2, kw // 2)), constant_values=pad)
    return fn(sliding_window_view(fn(sliding_window_view(p, kw, axis=1), axis=-1), kh, axis=0), axis=-1)
