"""2D world for the offline test: track on the floor, differential robot, obstacles.

Only meant to test the FSM LOGIC without Webots. The camera is rendered with the
same pinhole model as controllers/commun/geometrie_camera.py; the rangefinder uses
the URM37.proto lookupTable; the encoders integrate the real wheel speed.
"""
import math
import os
import sys

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "commun"))
from geometrie_camera import CameraModel  # noqa: E402

RES = 0.004            # m per floor pixel
LINE_WIDTH = 0.019     # m (exp_d1_ligne.wbt)
WHEEL_R = 0.0325
K_REAL = 0.135         # "real" effective half track of the simulated robot
TAU_WHEEL = 0.05       # s, first-order wheel lag
PHI_MAX = 10.0         # rad/s, Webots RotationalMotor default
X_US = 0.071           # m, rangefinder ahead of the centre
US_D = [0.0, 0.004, 0.1, 0.2, 0.25, 0.5, 0.75, 1.0]      # URM37.proto
US_V = [200, 200, 500, 500, 1250, 2500, 3750, 5000]


def straight(p0, p1, step=0.005):
    n = max(2, int(math.dist(p0, p1) / step) + 1)
    return np.linspace(p0, p1, n)


def arc(centre, radius, a0, a1, step=0.005):
    n = max(2, int(abs(a1 - a0) * radius / step) + 1)
    a = np.linspace(a0, a1, n)
    return np.c_[centre[0] + radius * np.cos(a), centre[1] + radius * np.sin(a)]


def grid_track(n=5, g=0.50):
    """Segments of an n x n grid with g between nodes; node (i, j) is at (i g, j g)."""
    return ([straight((0, j * g), ((n - 1) * g, j * g)) for j in range(n)]
            + [straight((i * g, 0), (i * g, (n - 1) * g)) for i in range(n)])


def eight_track(r=0.8):
    """A figure 8: two circles of radius r joined by two diagonals that cross at 90 deg at the origin."""
    c, t = r * math.sqrt(2), r / math.sqrt(2)
    return [straight((-t, -t), (t, t)), straight((-t, t), (t, -t)),
            arc((c, 0), r, -3 * math.pi / 4, 3 * math.pi / 4), arc((-c, 0), r, math.pi / 4, 7 * math.pi / 4)]


class Obstacle:
    """Circle. Appears at t_on and disappears at t_off. If 'ahead' is given, it is
    placed at that distance in front of the robot (along its heading) at t_on."""

    def __init__(self, radius=0.05, centre=None, t_on=0.0, t_off=math.inf, ahead=None):
        self.radius, self.centre, self.t_on, self.t_off, self.ahead = radius, centre, t_on, t_off, ahead

    def active(self, t):
        return self.t_on <= t < self.t_off


class World:
    def __init__(self, name, segments, obstacles=(), keys=(), t_end=25.0,
                 pose0=(0.0, 0.0, 0.0), seed=0, reference=None):
        self.name, self.t_end, self.keys = name, t_end, sorted(keys)
        self.obstacles = list(obstacles)
        self.rng = np.random.default_rng(seed)
        self.segments = [np.asarray(p, float) for p in segments]
        # path used to measure the lateral error (without gaps)
        self.reference = [np.asarray(p, float) for p in (reference or segments)]
        pts = np.vstack(self.segments)
        self.x0, self.y0 = pts.min(axis=0) - 1.0
        x1, y1 = pts.max(axis=0) + 1.0
        self.floor = np.full((int((y1 - self.y0) / RES) + 1, int((x1 - self.x0) / RES) + 1), 30, np.uint8)
        thick = max(1, round(LINE_WIDTH / RES))
        for p in self.segments:
            ij = np.round((p - [self.x0, self.y0]) / RES).astype(np.int32)
            cv2.polylines(self.floor, [ij.reshape(-1, 1, 2)], False, 230, thick)
        # robot state
        self.x, self.y, self.psi = pose0
        self.phi_cmd = np.zeros(4)   # Motor1..4 (front right, front left, rear right, rear left)
        self.phi_dot = np.zeros(4)
        self.phi = np.zeros(4)
        self.t = 0.0
        self.log = []                # ground truth at every step
        self._cam = None

    # ---------------------------------------------------------- physics
    def advance(self, dt):
        sub = 4
        h = dt / sub
        for _ in range(sub):
            self.phi_dot += (self.phi_cmd - self.phi_dot) * (h / TAU_WHEEL)
            self.phi += self.phi_dot * h
            left = (self.phi_dot[1] + self.phi_dot[3]) / 2
            right = (self.phi_dot[0] + self.phi_dot[2]) / 2
            v = WHEEL_R * (left + right) / 2
            w = WHEEL_R * (right - left) / (2 * K_REAL)
            self.x += v * h * math.cos(self.psi + w * h / 2)
            self.y += v * h * math.sin(self.psi + w * h / 2)
            self.psi += w * h
        self.t += dt
        for o in self.obstacles:
            if o.ahead is not None and o.centre is None and self.t >= o.t_on:
                o.centre = (self.x + (o.ahead + o.radius) * math.cos(self.psi),
                            self.y + (o.ahead + o.radius) * math.sin(self.psi))
        self.log.append(dict(t=self.t, x=self.x, y=self.y, psi=self.psi, v=v, w=w,
                             error=self.lateral_error(), d_true=self.true_distance()))

    # ---------------------------------------------------------- sensors
    def image(self, width, height, fov):
        if self._cam is None:
            m = CameraModel(0.132, 0.104, math.radians(30), fov, width, height)
            u, v = np.meshgrid(np.arange(width), np.arange(height))
            self._cam = m.pixel_para_chao(u, v)
        xr, yr = self._cam
        c, s = math.cos(self.psi), math.sin(self.psi)
        xw = self.x + c * xr - s * yr
        yw = self.y + s * xr + c * yr
        j = np.round((xw - self.x0) / RES)
        i = np.round((yw - self.y0) / RES)
        inside = np.isfinite(j) & (j >= 0) & (j < self.floor.shape[1]) & (i >= 0) & (i < self.floor.shape[0])
        g = np.full(xr.shape, 60.0)
        g[inside] = self.floor[i[inside].astype(int), j[inside].astype(int)]
        g = np.clip(g + self.rng.normal(0, 4, g.shape), 0, 255).astype(np.uint8)
        return np.dstack([g, g, g, np.full_like(g, 255)])

    def true_distance(self):
        """Shortest distance seen by the 10 sonar rays (aperture 0.52 rad)."""
        ox = self.x + X_US * math.cos(self.psi)
        oy = self.y + X_US * math.sin(self.psi)
        best = math.inf
        for o in self.obstacles:
            if not o.active(self.t) or o.centre is None:
                continue
            for a in np.linspace(-0.26, 0.26, 10):
                dx, dy = math.cos(self.psi + a), math.sin(self.psi + a)
                fx, fy = o.centre[0] - ox, o.centre[1] - oy
                proj = fx * dx + fy * dy
                perp2 = fx * fx + fy * fy - proj * proj
                if proj > 0 and perp2 <= o.radius ** 2:
                    best = min(best, proj - math.sqrt(o.radius ** 2 - perp2))
        return best

    def us_value(self):
        d = min(self.true_distance(), 1.0)
        val = float(np.interp(d, US_D, US_V))
        return val * (1 + self.rng.normal(0, 0.01))

    def lateral_error(self):
        p = np.array([self.x, self.y])
        return min(float(np.min(np.hypot(*(t - p).T))) for t in self.reference)
