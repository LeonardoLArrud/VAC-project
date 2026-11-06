"""Webots implementation of the hardware hooks (Cherokey_4W_v3 robot).

All hooks filled in: timing, camera (for the line follower), the 4 wheel motors,
the wheel encoders PS1..PS4 (heading), the URM37 rangefinder and its servo, and
the keyboard (START/STOP for the tests).

Keys (click the 3D view first): G = START, S or space = STOP, L = blind/unblind the
camera (test "line disappears and comes back"), M = the MISSION set in fsm.py (from IDLE).
"""
from __future__ import annotations

import numpy as np
from controller import Robot

from hardware import K_WHEELS, Hardware

WHEEL_R = 0.0325                     # m, wheel radius (Cherokey_4W_v3.proto)
LEFT_MOTORS = ("Motor2", "Motor4")   # front left, rear left
RIGHT_MOTORS = ("Motor1", "Motor3")  # front right, rear right
LEFT_ENCODERS = ("PS2", "PS4")       # same wheels as LEFT_MOTORS
RIGHT_ENCODERS = ("PS1", "PS3")      # same wheels as RIGHT_MOTORS
US_VALUE = [500, 1250, 2500, 3750, 5000]   # URM37.proto lookupTable, value -> metres
US_DIST = [0.20, 0.25, 0.50, 0.75, 1.00]   # below 500 everything is <= 0.20 m; 5000 = 1 m or more
SERVO_LIMIT = 1.58                         # rad, servomoteur min/maxPosition


class WebotsHardware(Hardware):
    def __init__(self, time_step_ms=64, mission=None):
        super().__init__(time_step_ms)
        self.mission = mission               # ("MISSION", start, heading, goal) sent by key M, or None
        self.robot = Robot()
        self.cam = self.robot.getDevice("PiCam")
        self.cam.enable(time_step_ms)
        self.left = [self.robot.getDevice(n) for n in LEFT_MOTORS]
        self.right = [self.robot.getDevice(n) for n in RIGHT_MOTORS]
        for m in self.left + self.right:
            m.setPosition(float("inf"))      # velocity control
            m.setVelocity(0.0)
        self.phi_max = self.left[0].getMaxVelocity()
        self.enc_left = [self.robot.getDevice(n) for n in LEFT_ENCODERS]
        self.enc_right = [self.robot.getDevice(n) for n in RIGHT_ENCODERS]
        for s in self.enc_left + self.enc_right:
            s.enable(time_step_ms)
        self._heading0 = self._s0 = None     # encoder heading and distance at the first reading
        self.us = self.robot.getDevice("URM37")
        self.us.enable(time_step_ms)
        self.servo = self.robot.getDevice("servomoteur")
        self.aim_sonar(0.0)      # look straight ahead
        self.kb = self.robot.getKeyboard()
        self.kb.enable(time_step_ms)
        self.blind = False

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
            elif c == "M" and self.mission:
                cmd = self.mission
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

    # ------------------------------------------------------------ encoders
    def heading(self):
        """Turn since start from the wheel encoders, rad, + = left (counter-clockwise).
        Wheel angle phi (rad) times WHEEL_R = distance rolled by that wheel; the
        right side minus the left side, over the track 2 * K_WHEELS, is the turn."""
        left = sum(s.getValue() for s in self.enc_left) / len(self.enc_left)
        right = sum(s.getValue() for s in self.enc_right) / len(self.enc_right)
        h = WHEEL_R * (right - left) / (2 * K_WHEELS)
        if np.isnan(h):                      # no value before the first robot.step()
            return 0.0
        if self._heading0 is None:
            self._heading0 = h
        return float(h - self._heading0)

    def distance(self):
        s = WHEEL_R * sum(e.getValue() for e in self.enc_left + self.enc_right) / 4
        if np.isnan(s):                      # no value before the first robot.step()
            return 0.0
        if self._s0 is None:
            self._s0 = s
        return float(s - self._s0)

    # ------------------------------------------------------------ rangefinder
    def obstacle_distance(self):
        """URM37 raw value converted to metres with the proto lookupTable (inverse).
        Raw value, not filtered: the FSM takes the median of the last 3 readings."""
        raw = self.us.getValue()
        if np.isnan(raw):                    # NaN before the first measurement
            return float("inf")
        return float(np.interp(raw, US_VALUE, US_DIST))

    def aim_sonar(self, angle):
        self.servo.setPosition(float(np.clip(angle, -SERVO_LIMIT, SERVO_LIMIT)))
