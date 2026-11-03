"""Webots implementation of the hardware hooks (Cherokey_4W_v3 robot).

Filled in: timing, camera (for the line follower), the 4 wheel motors and the
keyboard (START/STOP for the tests).

Still to fill in (sensors), marked TODO below:
- obstacle_distance(): the URM37 rangefinder;
- heading(): the PS1..PS4 wheel encoders;
- set_rangefinder_angle(): the servo (only needed from v5).
Until then obstacle_distance() and heading() return PLACEHOLDERS so the robot can
already follow the line in Webots: no obstacle is ever seen, and the heading is
the integral of the commanded turn rate (no sensor). A warning is printed once.

Keys (click the 3D view first): G = START, S or space = STOP, L = blind/unblind the
camera (test "line disappears and comes back").
"""
from __future__ import annotations

import numpy as np
from controller import Robot

from hardware import K_WHEELS, Hardware

WHEEL_R = 0.0325                     # m, wheel radius (Cherokey_4W_v3.proto)
LEFT_MOTORS = ("Motor2", "Motor4")   # front left, rear left
RIGHT_MOTORS = ("Motor1", "Motor3")  # front right, rear right


class WebotsHardware(Hardware):
    def __init__(self, time_step_ms=64):
        super().__init__(time_step_ms)
        self.robot = Robot()
        self.cam = self.robot.getDevice("PiCam")
        self.cam.enable(time_step_ms)
        self.left = [self.robot.getDevice(n) for n in LEFT_MOTORS]
        self.right = [self.robot.getDevice(n) for n in RIGHT_MOTORS]
        for m in self.left + self.right:
            m.setPosition(float("inf"))      # velocity control
            m.setVelocity(0.0)
        self.phi_max = self.left[0].getMaxVelocity()
        self.kb = self.robot.getKeyboard()
        self.kb.enable(time_step_ms)
        self.blind = False
        self._w_cmd = 0.0                    # last commanded turn rate (placeholder heading)
        self._heading = 0.0
        self._warned = set()

    # ------------------------------------------------------------ timing
    def step(self):
        return self.robot.step(self.time_step_ms) != -1

    def time(self):
        return self.robot.getTime()

    # ------------------------------------------------------------ camera
    def camera_info(self):
        return self.cam.getWidth(), self.cam.getHeight(), self.cam.getFov()

    def camera_image(self):
        img = np.frombuffer(self.cam.getImage(), np.uint8).reshape(
            (self.cam.getHeight(), self.cam.getWidth(), 4))
        return np.zeros_like(img) if self.blind else img

    # ------------------------------------------------------------ keyboard
    def command(self):
        cmd = None
        k = self.kb.getKey()
        while k != -1:
            c = chr(k & 0xFFFF).upper()
            if c == "G":
                cmd = "START"
            elif c in ("S", " "):
                cmd = "STOP"
            elif c == "L":
                self.blind = not self.blind
                print(f"[HW] t={self.time():6.2f}s camera {'BLIND' if self.blind else 'clear'}")
            k = self.kb.getKey()
        return cmd

    # ------------------------------------------------------------ motors
    def wheel_v_max(self):
        return self.phi_max * WHEEL_R

    def set_wheels(self, v_left, v_right):
        lim = self.phi_max
        for m in self.left:
            m.setVelocity(float(np.clip(v_left / WHEEL_R, -lim, lim)))
        for m in self.right:
            m.setVelocity(float(np.clip(v_right / WHEEL_R, -lim, lim)))
        self._w_cmd = (v_right - v_left) / (2 * K_WHEELS)

    # ------------------------------------------------------------ sensors: TODO
    def obstacle_distance(self):
        """TODO(sensors): read the "URM37" DistanceSensor and return metres.
        PLACEHOLDER until then: nothing ahead, so WAIT and TURN never trigger."""
        self._warn("obstacle_distance", "rangefinder hook empty: obstacles are ignored")
        return float("inf")

    def heading(self):
        """TODO(sensors): read the PS1..PS4 PositionSensors (rad) and return
        WHEEL_R * (mean(right) - mean(left)) / (2 * K_WHEELS); right = PS1, PS3, left = PS2, PS4.
        PLACEHOLDER until then: integral of the commanded turn rate (no sensor, no slip)."""
        self._warn("heading", "encoder hook empty: heading = integral of the commanded turn rate")
        self._heading += self._w_cmd * self.time_step_ms / 1000.0
        return self._heading

    def _warn(self, hook, text):
        if hook not in self._warned:
            self._warned.add(hook)
            print(f"[HW] TODO {hook}(): {text}")
