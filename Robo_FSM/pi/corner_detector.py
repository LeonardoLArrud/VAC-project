"""Intersection detector by corners: the corner detection v4 of TP-dev-emb/TP_Dev_Emb/corner_detection_v4.py,
run on the FSM camera image.

A "+" crossing has 4 concave corners; with at least EXPECTED of them it is a node with exits S, L and R.
step(image, s) -> (d, {"S", "L", "R"}) once per node, like NodeDetector.step(); d in m ahead of the robot centre.
T and L nodes have only 2 and 1 concave corners: node_detector.py (the bars) sees those.

The v4 sizes (px) were tuned on 1280-wide images (REF_WIDTH); here they are scaled to the camera width.
On the Pi 3 a detection is slower than the FSM loop, so it runs in parallel (PARALLEL): the loop hands
over the newest image and keeps going; each result comes back one or more cycles later and is moved to
the current position with the odometry s of its image. The time per detection is printed at exit.
"""
import atexit
import math
import multiprocessing as mp
import threading
import time

import cv2
import numpy as np

from line_follower import H_CAM, TILT, X_CAM
from node_detector import MISS, N_FRAMES, S_TOL, SAME, ZONE

# values: params.ini [corner_detector]; these defaults are the ones of corner_detection_v4.py
PARALLEL = "process"            # "process" (another CPU core), "thread", or None (inside the FSM loop)
REF_WIDTH = 1280                # image width the px sizes below were tuned for
BLUR, OPEN, MIN_DIST, WIN = 8, 17, 15, 25     # px at REF_WIDTH: blur, opening, between corners, window
MAX_CORNERS, QUALITY = 40, 0.05                # goodFeaturesToTrack
CONCAVE = (0.60, 0.85)          # white fraction around a concave corner
EXPECTED = 3                    # a "+" has 4 concave corners; 3 tolerates losing one
try:  # Robo_FSM: params.ini; without params.py the defaults above are used
    from params import load
    load("corner_detector", globals())
except ImportError:
    pass

EXITS = frozenset("SLR")


def detect_crossing_corners(img, scale=1.0):
    """detect_crossing_corners() of corner_detection_v4.py, with its px sizes multiplied by scale.
    img: BGR (as Picamera2 'RGB888' and imread give it) or already grey. Returns (concave corners N x 2, Otsu)."""
    def px(n):
        return max(1, round(n * scale))

    # 1) Binarization: Otsu picks the threshold
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if img.ndim == 3 else img
    blur = cv2.blur(gray, (px(BLUR), px(BLUR)))
    ret, thresh1 = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

    # 2) Opening: removes what is smaller than OPEN x OPEN (reflections, noise); keep it below the tape
    kernel = np.ones((px(OPEN), px(OPEN)), np.uint8)
    eroded_mask = cv2.erode(thresh1, kernel, iterations=1)
    dilated_mask = cv2.dilate(eroded_mask, kernel, iterations=1)

    # 3) Candidates: many, because goodFeaturesToTrack also marks steps of slanted edges and tape ends
    corners = cv2.goodFeaturesToTrack(np.float32(dilated_mask), MAX_CORNERS, QUALITY, px(MIN_DIST))
    if corners is None:
        return np.zeros((0, 2)), ret
    corners = corners.reshape(-1, 2)

    # 4) White fraction in a WIN x WIN window around each corner:
    #    ~0.75 concave corner (the 4 inner corners of a crossing), ~0.50 straight edge, ~0.25 tape end
    win = px(WIN)
    frac = cv2.blur(dilated_mask.astype(np.float32) / 255, (win, win))
    h, w = gray.shape
    m = win // 2 + 1
    corners = corners[(corners[:, 0] >= m) & (corners[:, 0] < w - m) &
                      (corners[:, 1] >= m) & (corners[:, 1] < h - m)]
    f = frac[corners[:, 1].round().astype(int), corners[:, 0].round().astype(int)]
    return corners[(f > CONCAVE[0]) & (f < CONCAVE[1])], ret


def _detect(img, scale):
    """One image -> (crossing centre (u, v) in px or None, number of concave corners, ms).
    Centre of the box around the corners (v4 prints their mean): with 3 of the 4 it is still the centre."""
    t0 = time.perf_counter()
    corners, _ = detect_crossing_corners(img, scale)
    centre = None
    if len(corners) >= EXPECTED:
        centre = tuple(float(c) for c in (corners.min(axis=0) + corners.max(axis=0)) / 2)
    return centre, len(corners), (time.perf_counter() - t0) * 1000


def _process_main(conn, scale):
    """The other process: receives (s, grey image), answers (s, centre, n, ms)."""
    cv2.setNumThreads(1)        # this process keeps to one core; the FSM loop has the others
    while True:
        job = conn.recv()
        if job is None:
            return
        s, grey = job
        conn.send((s,) + _detect(grey, scale))


class CornerDetector:
    def __init__(self, width, height, fov, parallel=PARALLEL):
        self.scale = width / REF_WIDTH
        self.f = (width / 2) / math.tan(fov / 2)
        self.cv = (height - 1) / 2
        self.cand, self.last = None, -1.0        # as in NodeDetector: [sum of s + d, frames, misses]; last s_node
        self.n = 0
        self.ms = 0.0
        self.t0 = time.monotonic()
        self.parallel, self.busy = parallel, False
        if parallel == "process":
            ctx = mp.get_context("spawn")       # a fresh interpreter: nothing of the camera is copied
            self.conn, child = ctx.Pipe()
            self.proc = ctx.Process(target=_process_main, args=(child, self.scale), daemon=True)
            self.proc.start()
        elif parallel == "thread":
            self.cond = threading.Condition()
            self.job = self.result = None
            threading.Thread(target=self._thread_main, daemon=True).start()
        elif parallel is not None:
            raise ValueError(f'PARALLEL must be "process", "thread" or None, not {parallel!r}')
        atexit.register(self.report)

    def _thread_main(self):
        while True:
            with self.cond:
                while self.job is None:
                    self.cond.wait()
                (s, img), self.job = self.job, None
            result = (s,) + _detect(img, self.scale)
            with self.cond:
                self.result = result

    def step(self, image, s):
        """Hand the image to the detector, use the newest result. (d, exits) once per node, else None."""
        if self.parallel == "process":
            try:
                if self.busy and self.conn.poll():
                    self._use(*self.conn.recv())
                    self.busy = False
                if not self.busy:
                    self.conn.send((s, cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)))   # grey: 3x less to copy
                    self.busy = True
            except (EOFError, OSError) as e:
                print(f"[CORNERS] detection process stopped ({e}): going on inside the FSM loop")
                self.parallel = None
        elif self.parallel == "thread":
            with self.cond:
                result, self.result = self.result, None
                if result is not None:
                    self.busy = False
                if not self.busy:
                    self.job, self.busy = (s, image), True
                    self.cond.notify()
            if result is not None:
                self._use(*result)
        else:
            self._use(s, *_detect(image, self.scale))
        return self._announce(s)

    def _ahead(self, centre):
        """Image row of the crossing centre -> m ahead of the robot centre (as NodeDetector.x)."""
        a = math.atan((centre[1] - self.cv) / self.f) + TILT
        return X_CAM + H_CAM / math.tan(a) if a > 0 else None

    def _use(self, s, centre, n, ms):
        self.n += 1
        self.ms += ms
        d = None if centre is None else self._ahead(centre)
        c = self.cand
        if d is not None and abs(s + d - self.last) > SAME:
            if c and abs(s + d - c[0] / c[1]) <= S_TOL:
                c[0], c[1], c[2] = c[0] + s + d, c[1] + 1, 0
            elif not c or c[1] < N_FRAMES:
                self.cand = [s + d, 1, 0]
        elif c and c[1] < N_FRAMES:               # crossing lost while confirming: tolerate MISS results
            c[2] += 1
            if c[2] > MISS:
                self.cand = None
        elif c and c[0] / c[1] - s > ZONE[0] + 0.05:   # confirmed but not seen where it should be
            self.cand = None

    def _announce(self, s):
        c = self.cand
        if c and c[1] >= N_FRAMES and c[0] / c[1] - s <= ZONE[0]:
            self.cand, self.last = None, c[0] / c[1]
            print(f"[CORNERS] crossing {self.last - s:.2f} m ahead ({c[1]} images)")
            return self.last - s, set(EXITS)
        return None

    def report(self):
        if self.n:
            dt = time.monotonic() - self.t0
            print(f"[CORNERS] {self.n} images in {dt:.0f} s ({self.n / dt:.1f}/s), "
                  f"{self.ms / self.n:.0f} ms each ({self.parallel or 'inside the FSM loop'})")


class NodeFusion:
    """Several node detectors (bars, corners): each node is announced once, by the first one that sees it."""

    def __init__(self, *detectors):
        self.detectors, self.last = detectors, -1.0

    def step(self, image, s):
        node = None
        for det in self.detectors:
            seen = det.step(image, s)            # every detector keeps tracking
            if seen and node is None and abs(s + seen[0] - self.last) > SAME:
                node = seen
        if node:
            self.last = s + node[0]
        return node
