"""Simulated hooks for the offline test: implements hardware.Hardware on world.World.

This is a test double, not robot code. It shows what a filled-in Hardware looks like
and lets the FSM run without Webots or the robot.
Keys scheduled in the world: G = START, S = STOP, L = blind/unblind the camera.
"""

import numpy as np

from hardware import K_WHEELS, Hardware
from world import PHI_MAX, WHEEL_R

WIDTH, HEIGHT, FOV = 480, 270, 1.0856       # PiCamera.proto at 1/4 (16:9)
US_VALUE = [500, 1250, 2500, 3750, 5000]    # URM37.proto lookupTable, value -> metres
US_DIST = [0.20, 0.25, 0.50, 0.75, 1.00]


class SimHardware(Hardware):
    def __init__(self, world, time_step_ms=64):
        super().__init__(time_step_ms)
        self.w = world
        self.keys = list(world.keys)
        self.blind = False

    def step(self):
        if self.w.t >= self.w.t_end - 1e-9:
            return False
        self.w.advance(self.time_step_ms / 1000.0)
        return True

    def time(self):
        return self.w.t

    def camera_info(self):
        return WIDTH, HEIGHT, FOV

    def camera_image(self):
        img = self.w.image(WIDTH, HEIGHT, FOV)
        return np.zeros_like(img) if self.blind else img

    def obstacle_distance(self):
        return float(np.interp(self.w.us_value(), US_VALUE, US_DIST))

    def heading(self):
        phi = self.w.phi                          # Motor1..4: right, left, right, left
        return WHEEL_R * ((phi[0] + phi[2]) / 2 - (phi[1] + phi[3]) / 2) / (2 * K_WHEELS)

    def distance(self):
        return WHEEL_R * float(np.mean(self.w.phi))

    def command(self):
        cmd = None
        while self.keys and self.keys[0][0] <= self.w.t + 1e-9:
            key = self.keys.pop(0)[1]
            if key == "G":
                cmd = "START"
            elif key == "S":
                cmd = "STOP"
            elif key == "L":
                self.blind = not self.blind
            elif isinstance(key, tuple):       # e.g. ("MISSION", start, heading, goal)
                cmd = key
        return cmd

    def wheel_v_max(self):
        return PHI_MAX * WHEEL_R

    def set_wheels(self, v_left, v_right):
        left = max(-PHI_MAX, min(PHI_MAX, v_left / WHEEL_R))
        right = max(-PHI_MAX, min(PHI_MAX, v_right / WHEEL_R))
        self.w.phi_cmd[:] = [right, left, right, left]

    def aim_sonar(self, angle):
        if abs(angle) > 1e-9:
            raise NotImplementedError("the simulated rangefinder only looks straight ahead")
