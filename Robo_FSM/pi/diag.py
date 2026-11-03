"""Checks on the real robot (Arduino with robo_fsm.ino).
python3 diag.py [all] | camera | live | wheels | encoders | pi | speed | turn
"""
import math
import sys
import time

import cv2
import numpy as np
import serial

from line_follower import THRESHOLD, LineFollower
from robot_hardware import (
    CAM_FOV,
    CAM_HEIGHT,
    CAM_WIDTH,
    COUNTS_PER_REV,
    PORT,
    USE_ENCODERS,
    V_MAX,
    WHEEL_R,
    RobotHardware,
)


def draw(f, img):
    g = f._grey(img)
    out = cv2.cvtColor(g.astype(np.uint8), cv2.COLOR_GRAY2BGR)
    for dj, row, mj, wj in f.bands:
        band = g[row - 2 : row + 3].mean(axis=0)
        cv2.line(out, (0, row), (f.nu - 1, row), (0, 255, 0), 1)
        out[row, band > THRESHOLD] = (0, 0, 255)
        u = f._find_line(g, row, f.cu, mj)
        if u is not None:
            cv2.circle(out, (int(u), row), 6, (255, 0, 0), 2)
    return g, out


def camera():
    cam = RobotHardware._open_camera()
    img = cam.capture_array()
    cam.stop()
    f = LineFollower(CAM_WIDTH, CAM_HEIGHT, CAM_FOV, 0.064, V_MAX)
    g, out = draw(f, img)
    print(f"image {img.shape}, grey min {g.min():.0f} median {np.median(g):.0f} max {g.max():.0f}, THRESHOLD {THRESHOLD}")
    for dj, row, mj, wj in f.bands:
        band = g[row - 2 : row + 3].mean(axis=0)
        u = f._find_line(g, row, f.cu, mj)
        text = "no line"
        if u is not None:
            y = -mj * (u - f.cu)
            text = f"line at u={u:.0f} px, Y={100 * y:+.1f} cm ({'left' if y > 0 else 'right'}), valid {abs(y) <= 0.7 * wj}"
        print(f"band {dj:.2f} m, row {row}: grey min {band.min():.0f} max {band.max():.0f}, "
              f"{(band > THRESHOLD).sum()} px above THRESHOLD, {text}")
    cv2.imwrite("diag.png", out)
    print("saved diag.png: green = bands, red = pixels above THRESHOLD, blue = line chosen")


def live():
    cam = RobotHardware._open_camera()
    f = LineFollower(CAM_WIDTH, CAM_HEIGHT, CAM_FOV, 0.064, V_MAX)
    try:
        while True:
            cv2.imshow("diag", draw(f, cam.capture_array())[1])
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    finally:
        cam.stop()
        cv2.destroyAllWindows()


def open_serial():
    ser = serial.Serial(PORT, 115200, timeout=0)
    time.sleep(2)
    return ser


def drive(ser, left, right, seconds, cmd="M"):
    t0 = time.monotonic()
    while time.monotonic() - t0 < seconds:
        ser.write(f"{cmd}{left},{right}\n".encode())
        time.sleep(0.05)
    ser.write(b"X\n")


def status(ser):
    lines = ser.read(ser.in_waiting).decode(errors="ignore").splitlines()
    for line in reversed(lines):
        parts = line.split(",")
        if parts[0] == "D" and len(parts) in (2, 6):
            try:
                values = [int(x) for x in parts[1:]]
            except ValueError:
                continue
            return values[0], (values[1:] or None)
    return None


def counts(ser):
    st = status(ser)
    return st[1] if st else None


def wheels():
    ser = open_serial()
    for name, left, right in [("left side forward", 120, 0), ("right side forward", 0, 120),
                              ("both forward", 120, 120), ("turn left in place", -120, 120)]:
        input(f"Enter -> {name}")
        drive(ser, left, right, 1.0)


def encoders():
    ser = open_serial()
    print("Turn each wheel forward by hand: its count must go up. Ctrl+C to stop.")
    while True:
        time.sleep(0.3)
        c = counts(ser)
        if c:
            print("   ".join(f"{n} {x:7d}" for n, x in zip(("FR", "FL", "RR", "RL"), c)))


def pi():
    ser = open_serial()
    input("Robot suspended. Enter -> all wheels at 0.10 m/s for 3 s")
    t0, c1 = time.monotonic(), None
    while time.monotonic() - t0 < 3.0:
        ser.write(b"S100,100\n")
        time.sleep(0.05)
        if c1 is None and time.monotonic() - t0 >= 1.0:
            c1, t1 = counts(ser), time.monotonic()
    c2, t2 = counts(ser), time.monotonic()
    ser.write(b"X\n")
    if c1 is None or c2 is None:
        print("no encoder data from the Arduino: is robo_fsm.ino uploaded?")
        return
    for name, a, b in zip(("FR", "FL", "RR", "RL"), c1, c2):
        v = 2 * math.pi * (b - a) / COUNTS_PER_REV * WHEEL_R / (t2 - t1)
        print(f"{name}: {v:+.3f} m/s (target +0.100)")


def speed():
    ser = open_serial()
    pwms, speeds = [], []
    for p in (80, 120, 160, 200):
        input(f"Place the robot on a straight line with room ahead. Enter -> PWM {p} for 2 s")
        drive(ser, p, p, 2.0)
        pwms.append(p)
        speeds.append(float(input("distance travelled (cm): ")) / 100 / 2.0)
    a, b = np.polyfit(pwms, speeds, 1)
    if a <= 0:
        print("speed does not grow with PWM: redo the test")
        return
    print(f"v = {a:.5f} * PWM {b:+.4f}")
    print(f"robot_hardware.py: PWM_MIN = {max(0, round(-b / a))}   V_MAX = {a * 255 + b:.2f}")


def turn():
    ser = open_serial()
    v_left, v_right, seconds = 0.06, 0.14, 3.0
    input(f"Mark the robot's heading on the floor. Enter -> arc to the left for {seconds:.0f} s")
    if USE_ENCODERS:
        drive(ser, round(1000 * v_left), round(1000 * v_right), seconds, "S")
    else:
        drive(ser, RobotHardware._pwm(v_left), RobotHardware._pwm(v_right), seconds)
    deg = float(input("angle the robot turned (degrees): "))
    print(f"line_follower.py: K = {(v_right - v_left) * seconds / (2 * math.radians(deg)):.3f}")


NAMES = ("FR", "FL", "RR", "RL")


def run(ser, cmd, left, right, seconds):
    time.sleep(0.15)
    c0 = counts(ser)
    t0 = time.monotonic()
    while time.monotonic() - t0 < seconds:
        ser.write(f"{cmd}{left},{right}\n".encode())
        time.sleep(0.05)
    c1, t1 = counts(ser), time.monotonic()
    ser.write(b"X\n")
    return c0, c1, t1 - t0


def everything():
    results = []

    def check(name, ok, detail):
        results.append((name, ok, detail))
        print(f"  {'OK   ' if ok else 'FAIL '} {name}: {detail}")

    input("1/4 Robot on the floor, line 5-10 cm to the LEFT of the robot's centre. Enter")
    ser = open_serial()
    time.sleep(0.3)
    st = status(ser)
    check("Arduino", st is not None, "D lines arriving" if st else "nothing on the serial port: robo_fsm.ino uploaded? PORT?")
    if st:
        cm, c = st
        check("Ultrasound", cm != -2, "no signal: check the URM37 wiring" if cm == -2 else
              ("no echo (nothing ahead)" if cm == -1 else f"{cm} cm"))
        check("Encoder data", c is not None, "counts arriving" if c else "old firmware: upload the new robo_fsm.ino")

    print("2/4 Camera")
    cam = RobotHardware._open_camera()
    img = cam.capture_array()
    cam.stop()
    f = LineFollower(CAM_WIDTH, CAM_HEIGHT, CAM_FOV, 0.064, V_MAX)
    g, out = draw(f, img)
    cv2.imwrite("diag.png", out)
    found = []
    for dj, row, mj, wj in f.bands:
        u = f._find_line(g, row, f.cu, mj)
        if u is not None and abs(-mj * (u - f.cu)) <= 0.7 * wj:
            found.append((dj, -mj * (u - f.cu)))
    check("Line seen", bool(found), f"{len(found)}/{len(f.bands)} bands (diag.png saved)" if found else
          f"no band: grey {g.min():.0f}..{g.max():.0f}, adjust THRESHOLD ({THRESHOLD}) and look at diag.png")
    if found:
        y = found[0][1]
        if abs(y) < 0.03:
            check("Image orientation", False, f"line at Y = {100 * y:+.1f} cm, too close to the centre to tell: "
                  "move it 5-10 cm to the LEFT and rerun")
        else:
            check("Image orientation", y > 0, f"line at Y = {100 * y:+.1f} cm ({'left' if y > 0 else 'RIGHT'})"
                  + ("" if y > 0 else ": image mirrored, check CAM_ROTATED_180 in robot_hardware.py"))

    if not st:
        return summary(results)
    input("3/4 Lift the robot so the wheels turn freely. Enter")
    for side, left, right, mine in (("left", 120, 0, (1, 3)), ("right", 0, 120, (0, 2))):
        c0, c1, _ = run(ser, "M", left, right, 1.0)
        ans = input(f"  Did only the {side} wheels turn, forward? [y/n] ").strip().lower()
        check(f"{side} motors", ans.startswith("y"), "ok" if ans.startswith("y") else
              "fix the wiring or the last Motor() parameter in robo_fsm.ino")
        if c0 and c1:
            for i in mine:
                n = c1[i] - c0[i]
                check(f"{NAMES[i]} encoder", n > 20, f"{n:+d} counts" + ("" if n > 20 else
                      (": no pulses, encoder not connected" if abs(n) <= 20 else f": counts backwards, flip ENC_SIGN[{i}]")))

    print("4/4 Wheel PI at 0.10 m/s for 3 s")
    t0, c1 = time.monotonic(), None
    while time.monotonic() - t0 < 3.0:
        ser.write(b"S100,100\n")
        time.sleep(0.05)
        if c1 is None and time.monotonic() - t0 >= 1.0:
            c1, t1 = counts(ser), time.monotonic()
    c2, t2 = counts(ser), time.monotonic()
    ser.write(b"X\n")
    if c1 and c2:
        for i in range(4):
            v = 2 * math.pi * (c2[i] - c1[i]) / COUNTS_PER_REV * WHEEL_R / (t2 - t1)
            check(f"{NAMES[i]} PI", abs(v - 0.10) < 0.02, f"{v:+.3f} m/s (target +0.100)")
    ser.close()
    summary(results)


def summary(results):
    bad = [r for r in results if not r[1]]
    print(f"\n{len(results) - len(bad)}/{len(results)} OK")
    for name, _, detail in bad:
        print(f"  FAIL {name}: {detail}")


if __name__ == "__main__":
    modes = {"all": everything, "camera": camera, "live": live, "wheels": wheels, "encoders": encoders, "pi": pi,
             "speed": speed, "turn": turn}
    modes[sys.argv[1] if len(sys.argv) > 1 else "all"]()
