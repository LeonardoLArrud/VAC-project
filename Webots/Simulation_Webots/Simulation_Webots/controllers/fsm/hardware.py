"""Hardware hooks: everything the FSM needs from the robot, still EMPTY.

The FSM (fsm_basic.py) and the line follower (line_follower.py) never touch a
device. They only see one Reading per cycle and return (v, w). This file is the
only place where sensors and actuators are read or driven.

How to fill it in: write one subclass per platform and implement every method
marked HOOK (Webots: Robot.getDevice(...); real robot: Pi camera, UART to the
Arduino, ...). Until a hook is filled in, calling it raises NotImplementedError
with its name, so nothing runs silently with a missing sensor.

    class WebotsHardware(Hardware): ...   # webots_hardware.py: camera, motors, keyboard done;
                                          # rangefinder and encoders still TODO
    class RobotHardware(Hardware): ...    # real robot: all hooks to fill in

Units and conventions (the FSM relies on them):
- distances in metres, angles in radians, time in seconds;
- robot frame: x forward, y left, positive angle = counter-clockwise (left turn).
"""
from __future__ import annotations

from fsm_basic import Reading

K_WHEELS = 0.135   # m, effective half track (same K as the line follower)


class Hardware:
    def __init__(self, time_step_ms=64):
        self.time_step_ms = time_step_ms

    # ------------------------------------------------------------ timing
    def step(self) -> bool:
        """HOOK. Wait for the next cycle (every time_step_ms) and refresh the sensors.
        Return False to end the program (Webots: robot.step(...) == -1)."""
        raise NotImplementedError("Hardware.step")

    def time(self) -> float:
        """HOOK. Seconds since start."""
        raise NotImplementedError("Hardware.time")

    # ------------------------------------------------------------ sensors
    def camera_info(self) -> tuple[int, int, float]:
        """HOOK. (width px, height px, horizontal field of view rad), read once at start
        to build the line follower."""
        raise NotImplementedError("Hardware.camera_info")

    def camera_image(self):
        """HOOK. Latest camera frame as a uint8 array (height, width, 4), BGRA."""
        raise NotImplementedError("Hardware.camera_image")

    def obstacle_distance(self) -> float:
        """HOOK. Distance to the nearest obstacle ahead, in metres, from the
        rangefinder (ultrasound or infrared). Nothing in range: return the sensor's
        maximum range (or float('inf')). Return the raw value: the FSM already takes
        the median of the last 3 readings."""
        raise NotImplementedError("Hardware.obstacle_distance")

    def heading(self) -> float:
        """HOOK. Heading accumulated from the wheel encoders since start, in rad,
        positive = counter-clockwise. With wheel angles phi (rad) and wheel radius r:
        heading = r * (mean(phi_right) - mean(phi_left)) / (2 * K_WHEELS)."""
        raise NotImplementedError("Hardware.heading")

    def command(self) -> str | None:
        """HOOK. "START", "STOP" or None, from a key, a button or the central (WiFi)."""
        raise NotImplementedError("Hardware.command")

    # ------------------------------------------------------------ actuators
    def wheel_v_max(self) -> float:
        """HOOK. Top linear speed of one wheel, m/s (the follower caps v with it)."""
        raise NotImplementedError("Hardware.wheel_v_max")

    def set_wheels(self, v_left: float, v_right: float) -> None:
        """HOOK. Linear speed set-point of the left and right wheels, m/s (both wheels
        of a side get the same value). Real robot: send it to the Arduino over UART;
        the Arduino runs the per-wheel PI and stops after 200 ms without a command."""
        raise NotImplementedError("Hardware.set_wheels")

    def set_rangefinder_angle(self, angle: float) -> None:
        """HOOK, not used in v1 (servo stays at 0). Servo angle of the rangefinder, rad,
        0 = straight ahead, positive = left. Needed later by AVOID (v5)."""
        raise NotImplementedError("Hardware.set_rangefinder_angle")

    # ------------------------------------------------------------ glue (do not change)
    def read(self) -> Reading:
        return Reading(self.time(), self.camera_image(), self.obstacle_distance(), self.heading(), self.command())

    def wheels(self, v: float, w: float) -> None:
        self.set_wheels(v - K_WHEELS * w, v + K_WHEELS * w)