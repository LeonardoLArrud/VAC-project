"""Hardware layer for the real robot: Pi Camera + Arduino with robo_fsm.ino. Run: python robot_hardware.py"""
import atexit
import math
import time

import serial

from line_follower import K
from params import load

# values: params.ini [robot_hardware]
PORT = CAM_WIDTH = CAM_HEIGHT = CAM_FOV = CAM_ROTATED_180 = USE_ENCODERS = None
V_MAX = PWM_MIN = PWM_TURN_MAX = WHEEL_R = COUNTS_PER_REV = LIVE_VIEW = SONAR_SIGN = SONAR_MAX = None
load("robot_hardware", globals())
D_RATE = 50         # D, lines per second the Arduino sends: robo_fsm.ino PERIOD_D_MS = 20


def sonar_degrees(angle):
    """Servo offset from straight ahead (deg, integer) for a sonar angle (rad, + = left), clipped to SONAR_MAX."""
    return round(SONAR_SIGN * math.degrees(max(-SONAR_MAX, min(SONAR_MAX, angle))))


class RobotHardware:
    def __init__(self, time_step_ms=40):
        self.time_step_ms = time_step_ms
        self.ser = serial.Serial(PORT, 115200, timeout=0)   # opening it resets the Arduino: it boots while the camera starts
        self.buf = b""
        self.dist = float("inf")
        self.sonar_deg = 0                 # servo offset last sent (robo_fsm.ino starts it straight ahead)
        self.counts = self.counts0 = None
        self.pwm = None                    # PWM the Arduino applies to each wheel (FR, FL, RR, RL), from the D lines
        self.alive = False                 # a D, line arrived: the Arduino is up
        self.n_d = 0                       # D, lines read so far (D_RATE per second when the serial keeps up)
        self.cam = self._open_camera()
        self.stream = None
        if LIVE_VIEW:
            try:
                from camera_stream import CameraStream

                self.stream = CameraStream(CAM_WIDTH, CAM_HEIGHT, CAM_FOV)
            except Exception as e:  # noqa: BLE001
                print(f"live view off: {e}")
        self._wait_arduino()
        self.set_turn_pwm(PWM_TURN_MAX)
        self.frame = None
        self.w_cmd = self.v_cmd = 0.0
        self.turned = self.travelled = 0.0
        self.t0 = self.next = time.monotonic()
        atexit.register(self.stop)

    def _wait_arduino(self, timeout=3.0):
        """Wait for the first D, line instead of a fixed sleep: the Arduino rebooted and is sending."""
        t_end = time.monotonic() + timeout
        while not self.alive:
            if time.monotonic() > t_end:
                raise TimeoutError(f"no D, line from the Arduino on {PORT} after {timeout:.0f} s: is robo_fsm.ino loaded?")
            time.sleep(0.01)
            self._read_serial()

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
        self.travelled += self.v_cmd * dt
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
                cm, self.alive = values[0], True
                self.n_d += 1
                self.dist = cm / 100.0 if cm >= 0 else (float("inf") if cm == -1 else 0.0)
                if len(values) >= 5:
                    self.counts = values[1:5]
                    self.counts0 = self.counts0 or self.counts
                if len(values) == 9:       # + the PWM of each wheel
                    self.pwm = values[5:]

    def set_turn_pwm(self, pwm):
        """PWM ceiling of each wheel while spinning in place (robo_fsm.ino T command; an older one ignores it)."""
        self.ser.write(f"T{int(pwm)}\n".encode())

    def time(self):
        return time.monotonic() - self.t0

    def camera_info(self):
        return CAM_WIDTH, CAM_HEIGHT, CAM_FOV

    def camera_image(self):
        self.frame = self.cam.capture_array()
        return self.frame

    def show(self, points, mask=None):
        """Send the last image, the line points the follower found on it and its mask (None: the live view
        computes it) to the live view."""
        if self.stream is None or self.frame is None:
            return
        try:
            self.stream.update(self.frame, points, mask)
        except Exception as e:  # noqa: BLE001
            print(f"live view off: {e}")
            self.stream = None

    def obstacle_distance(self):
        return self.dist

    def aim_sonar(self, angle):
        """Sonar towards angle (rad, + = left). Sent only on a change of 3 deg or more: the servo does not jitter."""
        deg = sonar_degrees(angle)
        if abs(deg - self.sonar_deg) >= 3 or (deg == 0 and self.sonar_deg != 0):
            self.ser.write(f"A{deg}\n".encode())
            self.sonar_deg = deg

    def heading(self):
        if not USE_ENCODERS or self.counts is None:
            return self.turned
        phi = [2 * math.pi * (n - n0) / COUNTS_PER_REV for n, n0 in zip(self.counts, self.counts0)]
        return WHEEL_R * ((phi[0] + phi[2]) / 2 - (phi[1] + phi[3]) / 2) / (2 * K)

    def distance(self):
        if not USE_ENCODERS or self.counts is None:
            return self.travelled
        return WHEEL_R * sum(2 * math.pi * (n - n0) / COUNTS_PER_REV for n, n0 in zip(self.counts, self.counts0)) / 4

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
        self.w_cmd, self.v_cmd = (v_right - v_left) / (2 * K), (v_left + v_right) / 2

    def stop(self):
        try:
            self.ser.write(b"X\n")
            self.ser.flush()
        finally:
            self.cam.stop()


if __name__ == "__main__":
    from fsm import TIME_STEP, main

    try:
        main(RobotHardware(TIME_STEP))
    except KeyboardInterrupt:           # Ctrl+C: the normal way to stop (the motors stop on exit)
        print("\nstopped (Ctrl+C)")
