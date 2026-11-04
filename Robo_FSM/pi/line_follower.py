"""Pure pursuit line follower (same law as pure_pursuit.py). With no target: LOST, v = w = 0."""
from __future__ import annotations

import math
from dataclasses import dataclass

import cv2
import numpy as np

H_CAM = 0.132
X_CAM = 0.104
TILT = math.radians(30)
BANDS = [0.20, 0.26, 0.31, 0.36]
K = 0.17
V0 = 0.27
TAU_EFF = 0.10
A_LAT = 1.0
A_UP = 0.5
A_DOWN = 1.5
THRESHOLD = 150
WIDTH = 480
# Image filter (as in corner_detection_v4.py), against floor reflections that pass THRESHOLD.
# The opening removes white blobs narrower than OPEN_W or shorter than OPEN_H (px at 640x480).
# Keep OPEN_W well below the tape width (19 mm tape: 35 px on the far band, 68 px on the near one)
# and OPEN_H below the height of a crossing tape on the far band (about 16 px).
FILTER = True
BLUR = 5
OPEN_W, OPEN_H = 15, 5
USE_OTSU = False  # True: threshold = max(Otsu, THRESHOLD); a large glare can then hide the tape

OK, MEMORY, LOST = "OK", "MEMORY", "LOST"


@dataclass
class FollowerOutput:
    status: str
    target: tuple | None
    v: float
    w: float
    side: float


class LineFollower:
    def __init__(self, cam_width, cam_height, fov, dt, wheel_v_max):
        self.dt = dt
        self.wheel_v_max = wheel_v_max
        self.px_step = max(1, cam_width // WIDTH)
        self.nu = cam_width // self.px_step
        self.nv = cam_height // self.px_step
        f = (self.nu / 2) / math.tan(fov / 2)
        self.cu = (self.nu - 1) / 2
        cv = (self.nv - 1) / 2
        self.bands = []
        for dj in BANDS:
            q = math.tan(math.atan2(H_CAM, dj - X_CAM) - TILT)
            row = round(cv + f * q)
            if 2 <= row < self.nv - 2:
                den = math.sin(TILT) + q * math.cos(TILT)
                self.bands.append((dj, row, H_CAM / (f * den), H_CAM * math.tan(fov / 2) / den))
        self.u_prev = [self.cu] * len(self.bands)
        self.memory = None
        self.L_lost = BANDS[-1]
        self.travelled = 0.0
        self.side = 1.0
        self.points = []  # (row, u, used) of the last image, in follower pixels: for the live view
        self.kernel = np.ones((OPEN_H, OPEN_W), np.uint8)

    def _grey(self, img):
        p = self.px_step
        return img[::p, ::p, :3].mean(axis=2)[: self.nv, : self.nu]

    def mask(self, img):
        """1 where the follower sees the line (nv x nu, uint8): blur, threshold, then an opening."""
        g = self._grey(img).astype(np.uint8)
        if not FILTER:
            return (g > THRESHOLD).astype(np.uint8)
        g = cv2.blur(g, (BLUR, BLUR))
        t = THRESHOLD
        if USE_OTSU:
            t = max(t, cv2.threshold(g, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)[0])
        return cv2.morphologyEx((g > t).astype(np.uint8), cv2.MORPH_OPEN, self.kernel)

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

    def find_target(self, img):
        """Farthest band first, searched from the image centre, without touching u_prev. (L, y) or None."""
        g = self.mask(img)
        points, target = [], None
        for dj, row, mj, wj in reversed(self.bands):
            u = self._find_line(g, row, self.cu)
            if u is None:
                continue
            y = -mj * (u - self.cu)
            used = abs(y) <= 0.7 * wj
            points.append((row, u, used))
            if used:
                target = (dj, y)
                break
        self.points = points
        return target

    def reset(self):
        """Forget the line position and the memory (after a U-turn the old ones are wrong)."""
        self.u_prev = [self.cu] * len(self.bands)
        self.memory, self.travelled = None, 0.0

    def _propagate(self, v, w):
        dt = self.dt
        dpsi = w * dt
        x0 = self.memory[0] - v * dt * math.cos(dpsi / 2)
        y0 = self.memory[1] - v * dt * math.sin(dpsi / 2)
        self.memory = (math.cos(dpsi) * x0 + math.sin(dpsi) * y0, -math.sin(dpsi) * x0 + math.cos(dpsi) * y0)
        self.travelled += abs(v) * dt
        if self.memory[0] <= 0.05 or self.travelled > self.L_lost:
            self.memory = None

    def step(self, img, v_max, v_prev, w_prev):
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
            self.memory, self.L_lost, self.travelled = target, target[0], 0.0
        elif self.memory is not None:
            self._propagate(v_prev, w_prev)
            target, status = self.memory, MEMORY

        if target is None:
            return FollowerOutput(LOST, None, 0.0, 0.0, self.side)

        L, Y = target
        self.side = 1.0 if Y >= 0 else -1.0
        c = 2 * Y / (L * L + Y * Y)
        v_target = min(V0, v_max, 0.163 * L / TAU_EFF, math.sqrt(A_LAT / max(abs(c), 1e-6)),
                       self.wheel_v_max / (1 + K * abs(c)))
        v = max(0.0, min(max(v_target, v_prev - A_DOWN * self.dt), v_prev + A_UP * self.dt))
        return FollowerOutput(status, target, v, v * c, self.side)
