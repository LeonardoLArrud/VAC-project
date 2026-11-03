"""Hardware layer for the real robot: Pi Camera + Arduino with robo_fsm.ino. Run: python robot_hardware.py"""
import atexit
import math
import time

import serial

from line_follower import K

PORT = "/dev/ttyACM0"
CAM_WIDTH, CAM_HEIGHT = 640, 480
CAM_FOV = math.radians(62.2)
CAM_ROTATED_180 = True
PWM_MIN = 45
V_MAX = 0.60
USE_ENCODERS = True
WHEEL_R = 0.0325
COUNTS_PER_REV = 11 * 21.3
LIVE_VIEW = True


class RobotHardware:
    def __init__(self, time_step_ms=64):
        self.time_step_ms = time_step_ms
        self.ser = serial.Serial(PORT, 115200, timeout=0)
        time.sleep(2)
        self.ser.reset_input_buffer()
        self.buf = b""
        self.dist = float("inf")
        self.counts = self.counts0 = None
        self.cam = self._open_camera()
        self.stream = None
        if LIVE_VIEW:
            try:
                from camera_stream import CameraStream

                self.stream = CameraStream(CAM_WIDTH, CAM_HEIGHT, CAM_FOV)
            except Exception as e:  # noqa: BLE001
                print(f"live view off: {e}")
        self.w_cmd = 0.0
        self.turned = 0.0
        self.t0 = self.next = time.monotonic()
        atexit.register(self.stop)

    @staticmethod
    def _open_camera():
        from libcamera import Transform
        from picamera2 import Picamera2

        cam = Picamera2()
        main = {"size": (CAM_WIDTH, CAM_HEIGHT), "format": "RGB888"}
        flip = Transform(hflip=CAM_ROTATED_180, vflip=CAM_ROTATED_180)
        opts = {"buffer_count": 3, "queue": False, "transform": flip}
        try:
            cfg = cam.create_preview_configuration(main=main, raw={"size": (1640, 1232)}, **opts)
        except Exception:  # noqa: BLE001
            cfg = cam.create_preview_configuration(main=main, **opts)
        cam.configure(cfg)
        cam.start()
        time.sleep(1.0)
        return cam

    def step(self):
        dt = self.time_step_ms / 1000.0
        self.next += dt
        wait = self.next - time.monotonic()
        if wait > 0:
            time.sleep(wait)
        else:
            self.next = time.monotonic()
        self.turned += self.w_cmd * dt
        self._read_serial()
        return True

    def _read_serial(self):
        n = self.ser.in_waiting
        if n:
            self.buf += self.ser.read(n)
        while b"\n" in self.buf:
            line, self.buf = self.buf.split(b"\n", 1)
            line = line.decode(errors="ignore").strip()
            if line.startswith("D,"):
                try:
                    values = [int(x) for x in line[2:].split(",")]
                except ValueError:
                    continue
                cm = values[0]
                self.dist = cm / 100.0 if cm >= 0 else (float("inf") if cm == -1 else 0.0)
                if len(values) == 5:
                    self.counts = values[1:]
                    self.counts0 = self.counts0 or self.counts

    def time(self):
        return time.monotonic() - self.t0

    def camera_info(self):
        return CAM_WIDTH, CAM_HEIGHT, CAM_FOV

    def camera_image(self):
        img = self.cam.capture_array()
        if self.stream is not None:
            try:
                self.stream.update(img)
            except Exception as e:  # noqa: BLE001
                print(f"live view off: {e}")
                self.stream = None
        return img

    def obstacle_distance(self):
        return self.dist

    def heading(self):
        if not USE_ENCODERS or self.counts is None:
            return self.turned
        phi = [2 * math.pi * (n - n0) / COUNTS_PER_REV for n, n0 in zip(self.counts, self.counts0)]
        return WHEEL_R * ((phi[0] + phi[2]) / 2 - (phi[1] + phi[3]) / 2) / (2 * K)

    def command(self):
        return None

    def wheel_v_max(self):
        return V_MAX

    @staticmethod
    def _pwm(v):
        if abs(v) < 1e-3:
            return 0
        return int(math.copysign(PWM_MIN + (255 - PWM_MIN) * min(abs(v) / V_MAX, 1.0), v))

    def set_wheels(self, v_left, v_right):
        if USE_ENCODERS:
            self.ser.write(f"S{round(1000 * v_left)},{round(1000 * v_right)}\n".encode())
        else:
            self.ser.write(f"M{self._pwm(v_left)},{self._pwm(v_right)}\n".encode())
        self.w_cmd = (v_right - v_left) / (2 * K)

    def stop(self):
        try:
            self.ser.write(b"X\n")
            self.ser.flush()
        finally:
            self.cam.stop()


if __name__ == "__main__":
    from fsm import TIME_STEP, main

    main(RobotHardware(TIME_STEP))
