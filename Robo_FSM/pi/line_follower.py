"""Pure pursuit line follower (same law as the Webots controllers/fsm/line_follower.py). With no target: LOST, v = w = 0."""
from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass

import cv2
import numpy as np

from params import load

# values: params.ini [line_follower]
H_CAM = X_CAM = TILT = BANDS = K = V0 = GAIN = TAU_EFF = A_LAT = A_UP = A_DOWN = None
THRESHOLD = FILTER = BLUR = OPEN_W = OPEN_H = USE_OTSU = MIN_CONTRAST = None
load("line_follower", globals())
WIDTH = 480

OK, MEMORY, LOST = "OK", "MEMORY", "LOST"


def to_grey(img, p, nv, nu):
    """Mean of the 3 channels of every p-th pixel (nv x nu, uint8). cv2.transform: the numpy mean, much faster."""
    roi = np.ascontiguousarray(img[: nv * p : p, : nu * p : p, :3])
    return cv2.transform(roi, np.full((1, 3), 1 / 3, np.float32))


@dataclass
class FollowerOutput:
    status: str
    target: tuple | None
    v: float
    w: float
    side: float
    c: float = 0.0   # curvature to the target, median of 3 frames (1/m, + = left): for the speed and the sonar


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
        self.side_px = {row: max(3, round(0.025 / mj)) for dj, row, mj, wj in self.bands}   # a tape width, px
        self.u_prev = [self.cu] * len(self.bands)
        self.c_recent = deque(maxlen=3)   # |c| of the last frames: the speed uses their median
        self.memory = None
        self.L_lost = BANDS[-1]
        self.travelled = 0.0
        self.side = 1.0
        self.points = []  # (row, u, used) of the last image, in follower pixels: for the live view
        self.kernel = np.ones((OPEN_H, OPEN_W), np.uint8)
        self._mask_img = self._mask = None   # last image and its mask (step() and find_target() share it)
        self._grey_img = self._grey = None   # last image and its grey (shared with the node and corner detectors)

    def grey(self, img):
        """to_grey() of img, computed once per image object."""
        if img is not self._grey_img:
            self._grey_img, self._grey = img, to_grey(img, self.px_step, self.nv, self.nu)
        return self._grey

    def mask(self, img):
        """1 where the follower sees the line (nv x nu, uint8), whole image. Computed once per image object."""
        if img is not self._mask_img:
            self._mask_img, self._mask = img, self._compute_mask(img)
        return self._mask

    def cached_mask(self, img):
        """mask() of img if it was already computed for it, else None: no extra work (live view)."""
        return self._mask if img is self._mask_img else None

    def _compute_mask(self, img):
        return self.mask_stages(img)[-1][1]

    def mask_stages(self, img):
        """[(name, image)] of every step from the grey image to the mask: blur, threshold, then an opening."""
        g = self.grey(img)
        if not FILTER:
            return [("grey", g), (f"grey > {THRESHOLD}", (g > THRESHOLD).astype(np.uint8))]
        b = cv2.blur(g, (BLUR, BLUR))
        t = THRESHOLD
        if USE_OTSU:
            t = max(t, cv2.threshold(b, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)[0])
        w = (b > t).astype(np.uint8)
        return [("grey", g), (f"blur {BLUR}", b), (f"> {t:.0f}", w),
                (f"opening {OPEN_W}x{OPEN_H}", cv2.morphologyEx(w, cv2.MORPH_OPEN, self.kernel))]

    def tape_runs(self, img, row):
        """White runs of the mask on this row (5 rows averaged): start, end (px) and their contrast with the floor
        beside them, one tape width away on each side seen (grey levels; nan: no floor seen). The tape has sharp
        edges: contrast >= MIN_CONTRAST; a floor glare fades into the floor around it."""
        m, g = self.mask(img), self.grey(img)
        white = (m[row - 2 : row + 3].mean(axis=0) > 0.5).astype(np.int8)
        edge = np.diff(np.concatenate(([0], white, [0])))
        start, end = np.flatnonzero(edge == 1), np.flatnonzero(edge == -1)
        ok = end - start >= 2
        start, end = start[ok], end[ok]
        csum = np.concatenate(([0.0], np.cumsum(g[row - 2 : row + 3].mean(axis=0))))   # window means in O(1)
        n = len(csum) - 1

        def mean(a, b):
            a, b = np.clip(a, 0, n), np.clip(b, 0, n)
            return np.where(b > a, (csum[b] - csum[a]) / np.maximum(b - a, 1), np.nan)

        k = np.clip(end - start, 3, self.side_px[row])
        floor = np.fmax(mean(start - 2 * k, start - k), mean(end + k, end + 2 * k))   # brighter side
        return start, end, mean(start, end) - floor

    def _find_line(self, img, row, u_ref):
        """Centre of the tape run (tape_runs) on this row closest to u_ref, or None."""
        start, end, contrast = self.tape_runs(img, row)
        tape = contrast >= MIN_CONTRAST                  # nan >= x: False
        if not tape.any():
            return None
        centres = (start[tape] + end[tape] - 1) / 2
        return float(centres[np.argmin(np.abs(centres - u_ref))])

    def find_target(self, img):
        """Farthest band first, searched from the image centre, without touching u_prev. (L, y) or None."""
        points, target = [], None
        for dj, row, mj, wj in reversed(self.bands):
            u = self._find_line(img, row, self.cu)
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
        self.c_recent.clear()

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
        target, points = None, []
        for j in reversed(range(len(self.bands))):
            dj, row, mj, wj = self.bands[j]
            u = self._find_line(img, row, self.u_prev[j])
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
        c = GAIN * 2 * Y / (L * L + Y * Y)
        self.c_recent.append(abs(c))
        c_speed = sorted(self.c_recent)[len(self.c_recent) // 2]   # median of 3 frames: one noisy frame does not brake
        v_target = min(V0, v_max, 0.163 * L / TAU_EFF, math.sqrt(A_LAT / max(c_speed, 1e-6)),
                       self.wheel_v_max / (1 + K * abs(c)))
        v = max(0.0, min(max(v_target, v_prev - A_DOWN * self.dt), v_prev + A_UP * self.dt))
        return FollowerOutput(status, target, v, v * c, self.side, math.copysign(c_speed, c))
