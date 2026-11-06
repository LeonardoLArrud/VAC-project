"""Camera geometry model: pixel <-> point on the floor (robot frame). Used by the offline test world
(fsm/offline/world.py) to render what the camera sees.

Robot frame: x forward, y left, z up, origin at the chassis centre projected on the floor.
Image: u right, v down, origin at the top-left corner.
Pinhole model without distortion (exact in Webots).
"""
from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class CameraModel:
    height_m: float        # height of the optical centre above the floor
    x_cam_m: float         # camera position ahead of the chassis centre
    tilt_rad: float        # positive = looking down
    hfov_rad: float        # horizontal field of view
    width_px: int
    height_px: int

    @property
    def f_px(self) -> float:
        return (self.width_px / 2.0) / math.tan(self.hfov_rad / 2.0)

    @property
    def cx(self) -> float:
        return (self.width_px - 1) / 2.0

    @property
    def cy(self) -> float:
        return (self.height_px - 1) / 2.0

    def scaled(self, factor: float) -> "CameraModel":
        """The same model for a smaller image (e.g. processed at 1/4)."""
        return CameraModel(self.height_m, self.x_cam_m, self.tilt_rad,
                           self.hfov_rad, round(self.width_px * factor),
                           round(self.height_px * factor))

    def pixel_to_floor(self, u, v):
        """(x, y) on the floor. NaN for pixels above the horizon."""
        u = np.asarray(u, dtype=float)
        v = np.asarray(v, dtype=float)
        # ray direction: forward, left, up (before the tilt)
        forward = np.ones_like(u)
        left = -(u - self.cx) / self.f_px
        up = -(v - self.cy) / self.f_px
        c, s = math.cos(self.tilt_rad), math.sin(self.tilt_rad)
        forward_r = forward * c + up * s
        up_r = -forward * s + up * c
        with np.errstate(divide="ignore", invalid="ignore"):
            t = np.where(up_r < 0, self.height_m / -up_r, np.nan)
        return self.x_cam_m + t * forward_r, t * left

    def floor_to_pixel(self, x, y):
        """Inverse of pixel_to_floor (point on the floor -> pixel)."""
        x = np.asarray(x, dtype=float) - self.x_cam_m
        y = np.asarray(y, dtype=float)
        z = -self.height_m
        c, s = math.cos(self.tilt_rad), math.sin(self.tilt_rad)
        forward = x * c - z * s
        up = x * s + z * c
        with np.errstate(divide="ignore", invalid="ignore"):
            u = self.cx - self.f_px * y / forward
            v = self.cy - self.f_px * up / forward
        return u, v
