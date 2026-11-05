"""Node detector: "+", T and L intersections ahead of the robot. step(image, s) -> (d, exits) once per node.

Traces the tape the camera sees on a top view of the floor (the image remapped onto the floor, RES m per pixel:
the tape is 2.5 cm wide everywhere and its corners are square):
1. tape = at least CONTRAST brighter than the floor around it (mean over BG): a glare or a bright floor patch is
   wide, as bright as its surroundings, so it is not tape
2. grid angle from the tape edges (gradient angles averaged with a 4-fold symmetry: square corners agree). A
   coherence under MIN_COHERENCE = no square tape pattern (a shoe, a tile edge). The view is turned upright
3. the line: the tape columns at least MIN_LEN long and TAPE_W wide nearest the robot axis; the bar: the tape rows
   beside the line, at least MIN_BAR long and TAPE_W thick, nearest the robot
4. the arms from the crossing centre: B (back: the line the robot is on), F (front: exit S), L, R: tape (True),
   no tape (False), or not in view yet (None)
step() adds up the arms over the images of one node: an exit is an arm seen as tape in 2 images or more, and more
often than seen without. The node is announced at ZONE[0] ahead, its position averaged over its images.
NodeProcess runs it in a process of its own (another core of the Pi 3), off the FSM loop.
"""
import atexit
import math
import multiprocessing as mp
import signal
import time

import cv2
import numpy as np

from line_follower import H_CAM, TILT, WIDTH, X_CAM, to_grey

ZONE = (0.24, 0.38)          # m ahead: farthest crossing taken; announced at ZONE[0]
CONTRAST = 30                # grey levels above the floor around it for tape
TAPE_W = (0.016, 0.038)      # m, tape width accepted (0.025 + blur)
MIN_COHERENCE = 0.4          # square tape pattern: 1 = all edges square, 0 = random
MAX_ANGLE = math.radians(20) # the line must be this close to the robot axis: none while spinning or askew
N_FRAMES, S_TOL, SAME = 3, 0.03, 0.10
MISS = 1                     # images without the crossing tolerated while confirming
try:  # Robo_FSM: params.ini [node_detector]; the Webots controller has no params.py
    from params import load
    load("node_detector", globals())
except ImportError:
    pass

RES = 0.002                  # m per top view pixel
BG = 0.08                    # m, floor around a pixel for its local mean
MIN_LEN, MIN_BAR = 0.04, 0.03   # m of tape in view: shortest line, shortest bar beside the line
ARM = (0.0175, 0.07)         # m from the crossing centre where an arm is looked at
ARM_HALF = 0.006             # m, half width of the strip of an arm
FAR_MAX = 0.60               # m, top view depth limit (with a small TILT the image sees far)
ARMS = {"B": (1, 0), "F": (-1, 0), "L": (0, -1), "R": (0, 1)}   # (row, column) step in the upright top view
EXIT = {"F": "S", "L": "L", "R": "R"}


def _runs(a):
    """(starts, ends) of the True runs of a 1-D array."""
    e = np.diff(np.r_[0, a.astype(np.int8), 0])
    return np.flatnonzero(e == 1), np.flatnonzero(e == -1)


class NodeDetector:
    def __init__(self, width, height, fov, grey=None):
        """grey: LineFollower.grey, so the image is turned grey once per frame for both."""
        p = max(1, width // WIDTH)
        nu, nv = width // p, height // p
        self.grey = grey or (lambda img: to_grey(img, p, nv, nu))
        f = (nu / 2) / math.tan(fov / 2)
        cu, cv = (nu - 1) / 2, (nv - 1) / 2

        def ahead(v):                            # m ahead of the floor seen on image row v
            a = math.atan((v - cv) / f) + TILT
            return X_CAM + H_CAM / math.tan(a) if a > 0 else math.inf

        near, far = ahead(nv - 1), min(ahead(0), FAR_MAX)
        q_far = math.tan(math.atan2(H_CAM, far - X_CAM) - TILT)
        half = cu * H_CAM / (f * (math.sin(TILT) + q_far * math.cos(TILT)))   # m beside the axis, far edge
        self.X = far - RES * np.arange(int((far - near) / RES) + 1)    # m ahead of each top view row: far on top
        self.Y = half - RES * np.arange(int(2 * half / RES) + 1)       # m left of each column: left first
        q = np.tan(np.arctan2(H_CAM, self.X[:, None] - X_CAM) - TILT)
        self.map_u = (cu - self.Y[None, :] * f * (math.sin(TILT) + q * math.cos(TILT)) / H_CAM).astype(np.float32)
        self.map_v = np.broadcast_to(cv + f * q, self.map_u.shape).astype(np.float32)
        self.seen = cv2.remap(np.full((nv, nu), 255, np.uint8), self.map_u, self.map_v, cv2.INTER_NEAREST,
                              borderMode=cv2.BORDER_CONSTANT, borderValue=0) > 0
        self.inner = cv2.erode(self.seen.astype(np.uint8), np.ones((7, 7), np.uint8)) > 0   # not the view border
        self.bg = 2 * round(BG / RES / 2) + 1
        self.seen_n = cv2.boxFilter(self.seen.astype(np.float32), -1, (self.bg, self.bg), normalize=False)
        self.tape_w = tuple(round(w / RES) for w in TAPE_W)
        self.min_len, self.min_bar = round(MIN_LEN / RES), round(MIN_BAR / RES)
        self.arm_k = np.arange(round(ARM[0] / RES), round(ARM[1] / RES))[:, None]
        self.arm_o = np.arange(-round(ARM_HALF / RES), round(ARM_HALF / RES) + 1)[None, :]
        self.axis = (half / RES, len(self.X) - 1)          # robot axis at the near edge (column, row)
        self.cand, self.last = None, -1.0   # [sum of s + d, images, arm seen as tape, arm seen without, misses]
        self.why = ""          # find(): why the last image gave no node, or what it found
        self.shape = ""        # find(): the traced shape, e.g. "T: B F R" (tape arms)
        self.trace = None      # find(): what it traced, for drawing (replay_filters.py)

    def step(self, image, s, announce=True):
        """announce=False: keeps tracking the crossing but announces nothing (the FSM is not following the line:
        a node announced then would be lost, and taken as already seen afterwards)."""
        found, c = self.find(image), self.cand
        if found and abs(s + found[0] - self.last) > SAME:
            d, arms = found
            if c and abs(s + d - c[0] / c[1]) <= S_TOL:
                c[0], c[1], c[4] = c[0] + s + d, c[1] + 1, 0
                self._add(c, arms)
            else:      # elsewhere: start again, even if confirmed (e.g. read askew just after a turn); two
                       # crossings are never in ZONE together (0.14 m of it, 0.975 m between nodes)
                self.cand = [s + d, 1, dict.fromkeys(ARMS, 0), dict.fromkeys(ARMS, 0), 0]
                self._add(self.cand, arms)
        elif c and c[1] < N_FRAMES:               # crossing lost while confirming: tolerate MISS images
            c[4] += 1
            if c[4] > MISS:
                self.cand = None
        elif c and c[0] / c[1] - s > ZONE[0] + 0.05:   # confirmed but not seen where it should be
            self.cand = None
        c = self.cand
        if announce and c and c[1] >= N_FRAMES and c[0] / c[1] - s <= ZONE[0]:
            self.cand, self.last = None, c[0] / c[1]
            tape, bare = c[2], c[3]
            if bare["B"] > tape["B"]:
                self.why = "crossing not announced: no line behind it (the robot is not on a line)"
                return None
            exits = {EXIT[a] for a in EXIT if tape[a] >= 2 and tape[a] > bare[a]}
            if not exits & {"L", "R"}:
                self.why = f"crossing not announced: no side arm seen twice (tape {tape}, none {bare})"
                return None
            return self.last - s, exits
        return None

    @staticmethod
    def _add(c, arms):
        for a, seen in arms.items():
            if seen is not None:
                c[2 if seen else 3][a] += 1

    def forget(self):
        """Drop the node being confirmed (the robot spun in place: its position no longer holds)."""
        self.cand = None

    def white(self, image):
        """Top view of the floor (grey) and its tape (bool)."""
        top = cv2.remap(self.grey(image), self.map_u, self.map_v, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT,
                        borderValue=0)
        g = top.astype(np.float32)
        mean = cv2.boxFilter(g, -1, (self.bg, self.bg), normalize=False) / np.maximum(self.seen_n, 1)
        tape = ((g - mean > CONTRAST) & self.seen).astype(np.uint8)
        return top, cv2.morphologyEx(tape, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8)) > 0

    def _angle(self, tape):
        """(grid angle, coherence) from the tape edges: gradient angles averaged with a 4-fold symmetry."""
        t = tape.astype(np.float32)
        gx, gy = cv2.Sobel(t, cv2.CV_32F, 1, 0), cv2.Sobel(t, cv2.CV_32F, 0, 1)
        m = np.hypot(gx, gy) * self.inner
        phi = 4 * np.arctan2(gy, gx)
        sin, cos = float((m * np.sin(phi)).sum()), float((m * np.cos(phi)).sum())
        return math.atan2(sin, cos) / 4, math.hypot(sin, cos) / max(float(m.sum()), 1e-9)

    def _bands(self, count, min_len):
        """(first, end) of the runs of count >= min_len that are a tape width wide."""
        s, e = _runs(count >= min_len)
        ok = (e - s >= self.tape_w[0]) & (e - s <= self.tape_w[1])
        return list(zip(s[ok], e[ok]))

    def _arm(self, up, seen, rc, cc, dr, dc):
        """True: tape along this arm; False: none where in view; None: not enough of it in view."""
        r = np.round(rc + dr * self.arm_k + dc * self.arm_o).astype(int)
        c = np.round(cc + dc * self.arm_k + dr * self.arm_o).astype(int)
        ok = (r >= 0) & (r < up.shape[0]) & (c >= 0) & (c < up.shape[1])
        ok[ok] = seen[r[ok], c[ok]]
        length = ok.sum() / self.arm_o.size * RES          # m of the arm in view
        frac = up[r[ok], c[ok]].mean() if ok.any() else 0.0
        if frac >= 0.6 and length >= 0.02:
            return True
        if frac < 0.25 and length >= 0.025:
            return False
        return None

    def find(self, image):
        """(d m ahead of the robot centre, arms) of the nearest crossing in this image, or None (see self.why)."""
        self.trace, self.shape = None, ""
        top, tape = self.white(image)
        angle, coherence = self._angle(tape)
        if coherence < MIN_COHERENCE:
            self.why = f"no square tape pattern (coherence {coherence:.2f} < {MIN_COHERENCE})"
            return None
        if abs(angle) > MAX_ANGLE:      # spinning (s does not move: anything would confirm) or askew
            self.why = f"tape at {math.degrees(angle):+.0f} deg: robot not along the line (max {math.degrees(MAX_ANGLE):.0f})"
            return None
        h, w = tape.shape
        M = cv2.getRotationMatrix2D((w / 2, h / 2), math.degrees(angle), 1.0)    # upright: tape along the axes
        up = cv2.warpAffine(tape.astype(np.uint8), M, (w, h), flags=cv2.INTER_NEAREST) > 0
        seen = cv2.warpAffine(self.seen.astype(np.uint8), M, (w, h), flags=cv2.INTER_NEAREST) > 0
        self.trace = dict(top=top, tape=tape, angle=angle, M=M, up=up, seen=seen, line=None, bar=None, arms=None)
        lines = self._bands(up.sum(axis=0), self.min_len)
        if not lines:
            self.why = f"no line: no tape {100 * TAPE_W[0]:.1f}-{100 * TAPE_W[1]:.1f} cm wide over {100 * MIN_LEN:.0f} cm"
            return None
        axis = M @ [*self.axis, 1.0]
        c0, c1 = min(lines, key=lambda b: abs((b[0] + b[1] - 1) / 2 - axis[0]))
        beside = up.copy()
        beside[:, max(0, c0 - 2) : c1 + 2] = False
        bars = self._bands(beside.sum(axis=1), self.min_bar)
        self.trace["line"] = (c0, c1)
        line_txt = f"line at {math.degrees(angle):+.0f} deg"
        if not bars:
            self.shape, self.why = "I", f"I: {line_txt}, no bar beside it"
            return None
        r0, r1 = max(bars)                         # nearest the robot: lowest in the view
        rc, cc = (r0 + r1 - 1) / 2, (c0 + c1 - 1) / 2
        arms = {a: self._arm(up, seen, rc, cc, dr, dc) for a, (dr, dc) in ARMS.items()}
        col, row = cv2.invertAffineTransform(M) @ [cc, rc, 1.0]
        d = float(np.interp(row, np.arange(len(self.X)), self.X))
        self.trace.update(bar=(r0, r1), centre=(rc, cc), arms=arms)
        tape_arms = " ".join(a for a in ARMS if arms[a])
        unseen = " ".join(a for a in ARMS if arms[a] is None)
        names = {"B F": "I", "B L": "L", "B R": "L", "B L R": "T", "B F L": "T", "B F R": "T", "B F L R": "+"}
        self.shape = f"{names.get(tape_arms, '?')}: {tape_arms}" + (f" (not in view: {unseen})" if unseen else "")
        if d > ZONE[1]:
            self.why = f"crossing at {100 * d:.0f} cm, beyond {100 * ZONE[1]:.0f}: {self.shape}"
            return None
        if not (arms["L"] or arms["R"]):
            self.why = f"bar at {100 * d:.0f} cm without a side arm: {self.shape}"
            return None
        self.why = f"crossing at {100 * d:.1f} cm, {self.shape}; {line_txt}"
        return d, arms


def _serve(conn, buf, width, height, fov):
    """The detector process: NodeDetector.step() on the grey image in buf for each (s, announce) received."""
    signal.signal(signal.SIGINT, signal.SIG_IGN)   # Ctrl+C stops the FSM; this process ends with it
    cv2.setNumThreads(1)                           # one core: the FSM loop keeps the others
    grey = np.frombuffer(buf, np.uint8)
    det = NodeDetector(width, height, fov, grey=lambda img: img)
    conn.send(grey.size)                           # ready: imports done, maps built
    while (job := conn.recv()) is not None:
        if job == "forget":
            det.forget()
            continue
        s, announce, shape = job
        t0 = time.perf_counter()
        node = det.step(grey.reshape(shape), s, announce)
        conn.send((s, node, det.shape, det.why, 1000 * (time.perf_counter() - t0)))


class NodeProcess:
    """NodeDetector in a process of its own: on the Pi 3 it takes ~36 ms per image, more than half the FSM loop.
    step() copies the grey image into shared memory for the process (when it is free) and returns the newest result:
    one or two cycles late, so d is moved from the odometry s of its image to the s of this call. A node announced
    while the FSM does not take one (announce=False) is kept until it does, as NodeDetector.step() would keep it.
    shape and why: those of the last image done (diag.py logs them)."""

    def __init__(self, width, height, fov, grey):
        self.grey = grey
        p = max(1, width // WIDTH)                 # the grey image size, as in NodeDetector
        ctx = mp.get_context("spawn")              # a fresh interpreter: nothing of the camera is copied
        self.buf = ctx.RawArray("B", (height // p) * (width // p))
        self.conn, child = ctx.Pipe()
        self.proc = ctx.Process(target=_serve, args=(child, self.buf, width, height, fov), daemon=True)
        t0 = time.monotonic()
        self.proc.start()
        if not self.conn.poll(60.0):               # Pi 3: imports and maps take a few seconds
            raise RuntimeError("node detector process not ready after 60 s")
        self.conn.recv()
        print(f"[NODES] detector process ready after {time.monotonic() - t0:.1f} s")
        self.busy, self.forgotten, self.stale, self.pending = False, False, False, None
        self.shape = self.why = ""
        self.n, self.ms, self.t0 = 0, 0.0, time.monotonic()
        atexit.register(self.close)

    def step(self, image, s, announce=True):
        try:
            if self.busy and self.conn.poll():
                s_img, node, self.shape, self.why, ms = self.conn.recv()
                self.busy, self.n, self.ms = False, self.n + 1, self.ms + ms
                if node and not self.stale:        # stale: its image is from before forget()
                    self.pending = (s_img + node[0], node[1])
                self.stale = False
            if not self.busy:
                grey = self.grey(image)
                np.copyto(np.frombuffer(self.buf, np.uint8, grey.size).reshape(grey.shape), grey)
                self.conn.send((s, announce, grey.shape))
                self.busy, self.forgotten = True, False
        except (EOFError, OSError) as e:
            raise RuntimeError(f"node detector process stopped ({e!r}): its error, if any, is printed above") from e
        if announce and self.pending:
            (s_node, exits), self.pending = self.pending, None
            return s_node - s, exits
        return None

    def forget(self):
        """NodeDetector.forget() in the process, before its next image; drops a node kept for the FSM or found on
        the image the process is still working on."""
        self.pending, self.stale = None, self.busy
        if not self.forgotten:
            self.conn.send("forget")
            self.forgotten = True

    def close(self):
        """Stop the process and print its speed. diag.py full makes several detectors: each would keep a process."""
        if not self.proc.is_alive():
            return
        try:
            self.conn.send(None)
        except OSError:
            pass
        self.proc.join(timeout=2.0)
        if self.proc.is_alive():
            self.proc.terminate()
        if self.n:
            dt = time.monotonic() - self.t0
            print(f"[NODES] {self.n} images in {dt:.0f} s ({self.n / dt:.1f}/s), {self.ms / self.n:.0f} ms each")
