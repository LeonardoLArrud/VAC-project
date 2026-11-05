"""Checks on the real robot (Arduino with robo_fsm.ino).
python3 diag.py [all] | camera | live | wheels | encoders | pi | speed | turn | sonar | odometry | camcal | nodecam | nodestop | nodegap | nodewhy | gridrecord [seconds] | loop [seconds]
python3 diag.py full [first step]: EVERY check below in one guided run, one zip at the end (see full())
Distance to a node = odometry s + camera d. Calibrate them in this order:
  odometry  follows a straight line for 1 m by the encoders, you measure it: WHEEL_R
  camcal    robot still, a crossing at 3 distances you measure: H_CAM, TILT_deg, X_CAM (tape edges + widths in the image)
  nodecam   robot still, a crossing at a distance you measure: what the node detector reads (TILT_deg only)
  nodestop  the whole chain: follows the line and stops on a node centre (as before a turn), you measure the error
  nodegap   follows a grid line through several nodes: the spacing between nodes (last number of MISSION)
nodewhy: robot still before a node: what the bars and corners detectors see, and why they give no node
gridrecord: free walk on the grid (random free exit at each node) recording every camera image + log (recordings/),
  to tune the detectors offline
sonar: sweeps the sonar servo right to left, distance at each angle: SONAR_SIGN
loop: robot still, motors off: ms of each part of the FSM loop and the load of each CPU core (does it fit TIME_STEP?)
"""
import ast
import builtins
import configparser
import csv
import json
import math
import os
import queue
import random
import re
import shutil
import subprocess
import sys
import threading
import time
import traceback
from collections import Counter
from itertools import pairwise

import cv2
import numpy as np
import serial

from fsm import D_SCAN, D_STOP, FSM, IDLE, MISSION, NODES, STAGES, TIME_STEP, TURN_TOL_Y, LoopTimer, look_for_node
from line_follower import BANDS, GAIN, H_CAM, K, LOST, THRESHOLD, TILT, V0, X_CAM, LineFollower
from navigators import SIDE, Explorer, Navigator, OrderList
from node_detector import ZONE, NodeDetector, NodeProcess
from params import PATH as PARAMS_PATH
from robot_hardware import (
    CAM_FOV,
    CAM_HEIGHT,
    CAM_WIDTH,
    COUNTS_PER_REV,
    PORT,
    USE_ENCODERS,
    V_MAX,
    WHEEL_R,
    D_RATE,
    SONAR_MAX,
    SONAR_SIGN,
    RobotHardware,
    sonar_degrees,
)


def draw(f, img):
    g, m = f.grey(img), f.mask(img)
    out = cv2.cvtColor(g.astype(np.uint8), cv2.COLOR_GRAY2BGR)
    for dj, row, mj, wj in f.bands:
        cv2.line(out, (0, row), (f.nu - 1, row), (0, 255, 0), 1)
        out[row, m[row] > 0] = (0, 0, 255)
        u = f._find_line(img, row, f.cu)
        if u is not None:
            cv2.circle(out, (int(u), row), 6, (255, 0, 0), 2)
    return g, m, out


def camera():
    cam = RobotHardware._open_camera()
    img = cam.capture_array()
    cam.stop()
    f = LineFollower(CAM_WIDTH, CAM_HEIGHT, CAM_FOV, 0.064, V_MAX)
    g, m, out = draw(f, img)
    print(f"image {img.shape}, grey min {g.min():.0f} median {np.median(g):.0f} max {g.max():.0f}, THRESHOLD {THRESHOLD}")
    for dj, row, mj, wj in f.bands:
        band = g[row - 2 : row + 3].mean(axis=0)
        u = f._find_line(img, row, f.cu)
        text = "no line"
        if u is not None:
            y = -mj * (u - f.cu)
            text = f"line at u={u:.0f} px, Y={100 * y:+.1f} cm ({'left' if y > 0 else 'right'}), valid {abs(y) <= 0.7 * wj}"
        print(f"band {dj:.2f} m, row {row}: grey min {band.min():.0f} max {band.max():.0f}, "
              f"{(band > THRESHOLD).sum()} px above THRESHOLD, {int(m[row].sum())} after the filter, {text}")
    cv2.imwrite("diag.png", out)
    print("saved diag.png: green = bands, red = line pixels after the filter, blue = line chosen")


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


def sonar():
    """Sonar on its servo, robot still: distance every 10 deg from right to left. A + angle must point to the LEFT
    of the robot (else SONAR_SIGN = -1). In FOLLOW the servo turns the sonar into the curves.
    Returns [(deg, cm or None)] (None: no echo or no data)."""
    ser = open_serial()
    top = round(math.degrees(SONAR_MAX))
    sweep = []
    for deg in range(-top, top + 1, 10):
        ser.write(f"A{sonar_degrees(math.radians(deg))}\n".encode())
        time.sleep(0.6)                 # servo move + a few echoes
        st = status(ser)
        side = "left" if deg > 0 else "right" if deg < 0 else "ahead"
        print(f"{deg:+4d} deg ({side}): " + ("no data from the Arduino" if st is None else
              {-1: "no echo", -2: "no signal"}.get(st[0], f"{st[0]} cm")))
        sweep.append((deg, st[0] if st is not None and st[0] >= 0 else None))
    ser.write(b"A0\n")
    ser.close()
    return sweep


def loop_report(periods):
    """Loop period of the drives below: if it is much above TIME_STEP, the robot sees fewer images per metre."""
    if len(periods) > 2:
        p = 1000 * np.array(periods[1:])
        print(f"  loop: mean {p.mean():.0f} ms, max {p.max():.0f} ms (target {TIME_STEP} ms)")


def settle(hw, seconds=1.0):
    """Wheels at 0 until the robot is surely still (it coasts a little)."""
    t_end = time.monotonic() + seconds
    while hw.step() and time.monotonic() < t_end:
        hw.set_wheels(0.0, 0.0)


def drive_line(hw, f, metres, v=0.15):
    """Follows the line for `metres` of odometry, stops; returns the odometry travelled once still."""
    s0, vc, wc, periods, t = hw.distance(), 0.0, 0.0, [], time.monotonic()
    while hw.step() and hw.distance() - s0 < metres:
        periods.append(time.monotonic() - t)
        t = time.monotonic()
        out = f.step(hw.camera_image(), v, vc, wc)
        if out.status == LOST:
            print("  line lost: put the robot back on a straight line")
            break
        vc, wc = out.v, out.w
        hw.set_wheels(vc - K * wc, vc + K * wc)
    settle(hw)
    loop_report(periods)
    return hw.distance() - s0


def ask_number(text):
    while True:
        try:
            return float(input(text).strip().replace(",", "."))
        except ValueError:
            print("  type a number")


def odometry(hw=None, metres=1.0):
    """Odometry scale: s (and so where the robot thinks a node is) must be true metres. Corrects WHEEL_R."""
    if not USE_ENCODERS:
        print("USE_ENCODERS = False: the distance comes from the commanded speed; calibrate V_MAX with diag.py speed")
        return None
    hw = hw or RobotHardware(TIME_STEP)
    f = LineFollower(CAM_WIDTH, CAM_HEIGHT, CAM_FOV, TIME_STEP / 1000.0, V_MAX)
    ratios = []
    while True:
        input(f"Robot on a straight line with {metres + 0.3:.1f} m free ahead. Mark on the floor a fixed point of the "
              f"robot (e.g. its front edge). Enter -> follows the line for {100 * metres:.0f} cm by the encoders")
        s = drive_line(hw, f, metres)
        if s < 0.5 * metres:
            print(f"  only {100 * s:.0f} cm by the encoders: try again")
            continue
        real = ask_number(f"  encoders: {100 * s:.1f} cm. How far the mark moved, measured with a tape (cm): ") / 100
        ratios.append(real / s)
        print(f"  real / encoders = {ratios[-1]:.4f}")
        if not input("Again? 2 or 3 runs are better [y/N] ").strip().lower().startswith("y"):
            break
    r = float(np.mean(ratios))
    print(f"params.ini [robot_hardware]: WHEEL_R = {WHEEL_R * r:.4f}   (now {WHEEL_R}, x{r:.4f} from {len(ratios)} run(s))")
    print("  so that the wheel speeds match too, put the same value in robo_fsm.ino (const float WHEEL_R) and upload it")
    return {"WHEEL_R": WHEEL_R * r, "ratios": ratios}


def node_distances(cam, f, det, corners, frames=20):
    """Median distance (m) to the crossing ahead read by the bars and by the corners, over `frames` images."""
    import corner_detector

    bars, cross = [], []
    for _ in range(frames):
        img = cam.capture_array()
        bar = det.find(img)
        if bar:
            bars.append(bar[0])
        centre, _, _ = corner_detector._detect(img, corners.scale)
        d = corners._ahead(centre) if centre else None
        if d:
            cross.append(d)
    return [(name, float(np.median(xs)) if xs else None, len(xs)) for name, xs in (("bars", bars), ("corners", cross))]


def tilt_for(true_d, read_d):
    """TILT with which the same image row would give the true distance (the row is angle = ray - TILT)."""
    return TILT + math.atan2(H_CAM, true_d - X_CAM) - math.atan2(H_CAM, read_d - X_CAM)


def floor_start():
    """Distance (m, from the robot centre) of the floor seen at the bottom of the image."""
    f = (CAM_WIDTH / 2) / math.tan(CAM_FOV / 2)
    return X_CAM + H_CAM / math.tan(math.atan(((CAM_HEIGHT - 1) / 2) / f) + TILT)


# Camera geometry from a crossing at distances measured with a tape (camcal): fits H_CAM, TILT, X_CAM.
#
# The robot stands still on a straight line facing a crossing (+, T or L); d = robot centre (midway between the
# axles) to the middle of the crossing tape. In the image, with no detector in between:
# - the rows of the near and far edges of the crossing tape: the floor at d - CAL_TAPE/2 and d + CAL_TAPE/2
# - the width in pixels of the line, row by row below the crossing: CAL_TAPE / (m per pixel of that row)
# The camera model (line_follower.py) gives both: row = cv + f tan(atan2(H_CAM, x - X_CAM) - TILT), and
# m per pixel = H_CAM / (f (sin TILT + q cos TILT)), q = (row - cv) / f. f comes from CAM_FOV_deg (the lens).
# Least squares over 2-4 distances: the rows fix TILT and X_CAM, the widths fix H_CAM.
CAL_TAPE = 0.025          # m, tape width


def cal_row_of(x, h, tilt, xc, f, cv):
    """Image row where the floor x m ahead of the robot centre is seen."""
    return cv + f * np.tan(np.arctan2(h, np.asarray(x) - xc) - tilt)


def cal_ahead_of(v, h, tilt, xc, f, cv):
    """m ahead of the robot centre of the floor seen on image row v."""
    return xc + h / np.tan(np.arctan((np.asarray(v) - cv) / f) + tilt)


def cal_width_px(v, h, tilt, f, cv):
    """Width in pixels of the tape (CAL_TAPE) across the robot axis, on image row v."""
    q = (np.asarray(v) - cv) / f
    return CAL_TAPE * f * (math.sin(tilt) + q * math.cos(tilt)) / h


def cal_runs(a):
    e = np.diff(np.r_[0, a.astype(np.int8), 0])
    return np.flatnonzero(e == 1), np.flatnonzero(e == -1)


def cal_measure(grey, f):
    """Tape edges and line widths in one grey image (follower pixels). Returns dict(edges: [(px right of the image
    centre, near edge row, far edge row)] per side the crossing tape goes, rows, widths: of the line, across: px of
    the line centre right of the image centre) or raises ValueError saying what is wrong with the scene."""
    g = cv2.GaussianBlur(grey, (5, 5), 0)
    t = cv2.threshold(g, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)[0]
    white = g > t
    nv, nu = white.shape
    rows, centres, widths, u_ref = [], [], [], nu / 2
    bar = None
    for v in range(nv - 1, -1, -1):                 # the line, from the bottom of the image up to the crossing
        s, e = cal_runs(white[v])
        if not len(s):
            if rows:
                raise ValueError(f"the line stops at row {v} before any crossing: is a crossing ahead?")
            continue
        k = int(np.argmin(np.abs((s + e - 1) / 2 - u_ref)))
        w = e[k] - s[k]
        if len(widths) >= 10 and w > 2.5 * np.median(widths[-10:]):
            bar = v                                 # the line widens: the crossing tape
            break
        rows.append(v)
        centres.append((s[k] + e[k] - 1) / 2)
        widths.append(w)
        u_ref = centres[-1]
    if len(rows) < 20:
        raise ValueError(f"only {len(rows)} rows of line under the crossing: robot too close, or no line")
    if bar is None:
        raise ValueError("no crossing tape seen above the line: too far, or out of the image")
    w_line = float(np.median(widths[-20:]))
    edges = []                                      # per side the crossing tape goes: (column, near row, far row)
    for sign in (1, -1):                            # columns 2-4 line widths beside the line
        side = []
        for c in np.arange(centres[-1] + sign * 2 * w_line, centres[-1] + sign * 4 * w_line, sign * 2):
            c = int(round(c))
            if not 0 <= c < nu:
                continue
            s, e = cal_runs(white[:, c])
            if not len(s):
                continue
            k = int(np.argmin(np.abs(e - 1 - bar)))
            if abs(e[k] - 1 - bar) <= 15 and e[k] < nv and s[k] > 0:   # a band ending near the crossing row
                side.append((c - (nu - 1) / 2, e[k] - 0.5, s[k] - 0.5))   # edges between tape and floor rows
        if len(side) >= 3:
            edges.append(tuple(np.median(np.array(side), axis=0)))
    if not edges:
        raise ValueError("the crossing tape is not seen beside the line (it must go at least 4 cm to one side)")
    return dict(edges=edges, rows=np.array(rows[5:-5], float), widths=np.array(widths[5:-5], float),
                across=np.array(centres[5:-5]) - (nu - 1) / 2)


def cal_angle_deg(m, h, tilt, xc, f, cv):
    """Angle (deg, + = line turning left) between the robot axis and the line, from the line on the floor. In the
    image a turned robot sees the line shifted rather than slanted (perspective): it must be measured in metres."""
    x = cal_ahead_of(m["rows"], h, tilt, xc, f, cv)
    y = -m["across"] * CAL_TAPE / cal_width_px(m["rows"], h, tilt, f, cv)
    return math.degrees(math.atan(np.polyfit(x, y, 1)[0]))


def cal_along(v, col, a, h, tilt, xc, f, cv):
    """(m along the line, m per row) of the floor seen at image row v, col px right of the image centre."""
    x = cal_ahead_of([v - 0.5, v, v + 0.5], h, tilt, xc, f, cv)
    y = -col * CAL_TAPE / cal_width_px(v, h, tilt, f, cv)
    return x[1] * math.cos(a) + y * math.sin(a), abs(x[0] - x[2])


def cal_crossing_at(m, h, tilt, xc, f, cv):
    """m from the robot centre to the middle of the crossing tape, along the line, read with these parameters."""
    a = math.radians(cal_angle_deg(m, h, tilt, xc, f, cv))
    return float(np.mean([cal_along(v, col, a, h, tilt, xc, f, cv)[0] for col, near, far in m["edges"] for v in (near, far)]))


def cal_tape_cm(m, h, tilt, f, cv):
    """Width of the line (cm) read with these parameters: 2.5 when H_CAM and TILT are right."""
    return float(np.median(100 * CAL_TAPE * m["widths"] / cal_width_px(m["rows"], h, tilt, f, cv)))


def cal_residuals(p, data, f, cv, h_tape=None):
    """Errors (px) of the model with p = (H_CAM, TILT, X_CAM) on every measurement; h_tape: H_CAM measured with a
    ruler (m), kept within ~2 mm. A robot not square to the line (angle a, from the line itself): the crossing tape
    edges are where the floor is d -+ CAL_TAPE/2 along the line, and the line looks 1/cos a wider."""
    h, tilt, xc = p
    r = []
    for d, m in data:
        a = math.radians(cal_angle_deg(m, h, tilt, xc, f, cv))
        for col, near, far in m["edges"]:
            for v, target in ((near, d - CAL_TAPE / 2), (far, d + CAL_TAPE / 2)):
                along, per_row = cal_along(v, col, a, h, tilt, xc, f, cv)
                r.append((along - target) / per_row)     # m -> rows
        k = np.linspace(0, len(m["rows"]) - 1, min(20, len(m["rows"]))).astype(int)
        r += list(cal_width_px(m["rows"][k], h, tilt, f, cv) / math.cos(a) - m["widths"][k])
    if h_tape is not None:
        r.append((h - h_tape) / 0.002)
    return np.array(r, float)


def cal_fit(data, f, cv, start, h_tape=None, iterations=50):
    """Levenberg-Marquardt from start = (H_CAM, TILT, X_CAM). Returns (p, rms px)."""
    p, lam = np.array(start, float), 1e-3
    steps = np.array([1e-5, 1e-5, 1e-5])
    cost = np.sum(cal_residuals(p, data, f, cv, h_tape) ** 2)
    for _ in range(iterations):
        r = cal_residuals(p, data, f, cv, h_tape)
        J = np.stack([(cal_residuals(p + np.eye(3)[i] * steps[i], data, f, cv, h_tape) - r) / steps[i] for i in range(3)], 1)
        A, g = J.T @ J, J.T @ r
        while lam < 1e9:
            q = p - np.linalg.solve(A + lam * np.diag(np.diag(A)), g)
            c = np.sum(cal_residuals(q, data, f, cv, h_tape) ** 2)
            if c < cost:
                p, cost, lam = q, c, lam / 3
                break
            lam *= 4
        else:
            break
    return p, math.sqrt(cost / len(cal_residuals(p, data, f, cv, h_tape)))


def camcal(cam=None, frames=10, save=False):
    """Camera geometry: robot still facing a crossing at 2-4 distances you measure; fits H_CAM, TILT_deg and X_CAM
    to the tape edges and widths in the image, no detector in between (cal_* above).
    save: one image per distance, camcal_<cm>cm.jpg in the current folder (to fit again offline)."""
    own = cam is None
    cam = cam or RobotHardware._open_camera()
    f = LineFollower(CAM_WIDTH, CAM_HEIGHT, CAM_FOV, TIME_STEP / 1000.0, V_MAX)
    fx, cv = (f.nu / 2) / math.tan(CAM_FOV / 2), (f.nv - 1) / 2
    now = (H_CAM, TILT, X_CAM)
    near, far = floor_start(), float(cal_ahead_of(0, *now, fx, cv))
    tries = ", ".join(f"{100 * x:.0f}" for x in (near + 0.08, (near + far) / 2 + 0.02, far - 0.05))
    print(f"Robot still on a straight line, square to it, a crossing (+, T or L) ahead. The image sees the floor "
          f"{100 * near:.0f}-{100 * far:.0f} cm ahead of the robot centre (midway between the axles). For each "
          f"distance, measure with a tape from the robot centre to the MIDDLE of the crossing tape: e.g. {tries} cm.")
    text = input("Camera lens height above the floor, with a ruler (cm; Enter to skip): ").strip().replace(",", ".")
    h_tape = float(text) / 100 if text else None
    data = []
    try:
        while True:
            text = input(f"Distance {len(data) + 1} (cm; Enter = done): ").strip().replace(",", ".")
            if not text:
                break
            try:
                d = float(text) / 100
            except ValueError:
                print("  type a number")
                continue
            got, errors = [], []
            for k in range(frames):
                img = cam.capture_array()
                if save and k == 0:
                    cv2.imwrite(f"camcal_{100 * d:.1f}cm.jpg", img)
                try:
                    got.append(cal_measure(f.grey(img), fx))
                except ValueError as e:
                    errors.append(str(e))
            if len(got) < frames // 2:
                print(f"  not measured in {frames - len(got)}/{frames} images: {errors[-1]}")
                continue
            m = sorted(got, key=lambda g: g["edges"][0][1])[len(got) // 2]     # the median image
            a = cal_angle_deg(m, *now, fx, cv)
            read = cal_crossing_at(m, *now, fx, cv)
            print(f"  read now {100 * read:.1f} cm (error {100 * (read - d):+.1f}), line {cal_tape_cm(m, now[0], now[1], fx, cv):.2f}"
                  f" cm wide (tape 2.50), robot {a:+.1f} deg from the line")
            if abs(a) > 5:
                print("  robot not square to the line: turn it and measure this distance again")
                continue
            data.append((d, m))
    finally:
        if own:
            cam.stop()
    if len(data) < 2:
        print("at least 2 distances are needed (3 are better)")
        return None
    p, rms = cal_fit(data, fx, cv, now, h_tape)
    print("distance   read now   read with the fit   line width now -> fit")
    for d, m in data:
        print(f"  {100 * d:5.1f} cm   {100 * cal_crossing_at(m, *now, fx, cv):5.1f} cm   {100 * cal_crossing_at(m, *p, fx, cv):5.1f} cm"
              f"           {cal_tape_cm(m, now[0], now[1], fx, cv):.2f} -> {cal_tape_cm(m, p[0], p[1], fx, cv):.2f} cm")
    print(f"params.ini [line_follower]: H_CAM = {p[0]:.3f}   TILT_deg = {math.degrees(p[1]):.1f}   X_CAM = {p[2]:.3f}")
    print(f"  (now H_CAM = {H_CAM:.3f}   TILT_deg = {math.degrees(TILT):.1f}   X_CAM = {X_CAM:.3f}); fit residual {rms:.2f} px")
    if rms > 2:
        print("  residual above 2 px: a distance mistyped or measured from another point? Measure again")
    rows = [cal_row_of(b, *now, fx, cv) for b in BANDS]
    bands = [round(float(cal_ahead_of(r, *p, fx, cv)), 3) for r in rows]
    gain = GAIN * cal_curvature(now, rows[-1], fx, cv) / cal_curvature(p, rows[-1], fx, cv)
    print(f"  BANDS that keep the follower on the same image rows: {bands} (now {BANDS})")
    print(f"  GAIN that keeps the same steering with the new geometry: {gain:.2f} (now {GAIN})")
    return {"H_CAM": float(p[0]), "TILT_deg": math.degrees(p[1]), "X_CAM": float(p[2]), "BANDS": bands,
            "GAIN": gain, "rms_px": float(rms), "distances": [d for d, _ in data]}


def cal_curvature(prm, row, f, cv, du=40.0):
    """Pure pursuit curvature (1/m) the follower computes for a line du px beside the image centre on this row,
    with the camera geometry prm = (H_CAM, TILT, X_CAM) (line_follower.py: same rows, other metres)."""
    h, tilt, _ = prm
    q = (row - cv) / f
    ahead = float(cal_ahead_of(row, *prm, f, cv))
    side = h / (f * (math.sin(tilt) + q * math.cos(tilt))) * du
    return 2 * side / (ahead * ahead + side * side)


MAX_ERR = 0.10      # nodecam: a reading this far from the measured distance is another crossing, not this one


def nodecam(cam=None, frames=20):
    """Camera distance to a node: robot still, a crossing at distances you measure. Corrects TILT_deg."""
    from corner_detector import CornerDetector

    own = cam is None
    cam = cam or RobotHardware._open_camera()
    f = LineFollower(CAM_WIDTH, CAM_HEIGHT, CAM_FOV, TIME_STEP / 1000.0, V_MAX)
    det = NodeDetector(CAM_WIDTH, CAM_HEIGHT, CAM_FOV, f.grey)
    corners = CornerDetector(CAM_WIDTH, CAM_HEIGHT, CAM_FOV, parallel=None)
    a, b = ZONE
    tries = ", ".join(f"{100 * x:.0f}" for x in (a + 0.02, (a + b) / 2, b - 0.02))
    print(f"Robot still on the line, a crossing ahead. Measure from the robot centre (midway between the front and rear "
          f"axles) to the middle of the crossing tape. The image starts {100 * floor_start():.0f} cm ahead; the bars "
          f"read {100 * a:.0f}-{100 * b:.0f} cm, the corners a '+' only, fully in the image. "
          f"Try 2 or 3 distances, e.g. {tries} cm.")
    tilts = []
    try:
        while True:
            text = input("  measured distance in cm (Enter = done): ").strip().replace(",", ".")
            if not text:
                break
            true_d = float(text) / 100
            if true_d < floor_start() + 0.03:
                print(f"    too close: the image starts {100 * floor_start():.0f} cm from the robot centre, "
                      f"so a crossing at {100 * true_d:.0f} cm is not in it (use {100 * ZONE[0]:.0f} cm or more)")
                continue
            for name, d, n in node_distances(cam, f, det, corners, frames):
                if d is None:
                    hint = " (they only see a '+', not a T or an L)" if name == "corners" else ""
                    print(f"    {name}: not seen in {frames} images{hint}")
                    continue
                text = f"    {name}: {100 * d:.1f} cm ({n}/{frames} images), error {100 * (d - true_d):+.1f} cm"
                if abs(d - true_d) > MAX_ERR:          # e.g. the next node, when this one is out of the zone
                    print(f"{text}: too far from what you measured, another crossing (the next node?): ignored")
                    continue
                tilts.append(tilt_for(true_d, d))
                print(f"{text} -> TILT {math.degrees(tilts[-1]):.2f} deg")
            if not tilts:
                print("    to see why a detector sees no node: python3 diag.py nodewhy")
    finally:
        if own:
            cam.stop()
    if tilts:
        t = float(np.median(tilts))
        print(f"params.ini [line_follower]: TILT_deg = {math.degrees(t):.1f}   (now {math.degrees(TILT):.1f}, "
              f"from {len(tilts)} reading(s))")
        if math.degrees(max(tilts) - min(tilts)) > 1.0:
            print("  the readings disagree by more than 1 deg: measure H_CAM and X_CAM again")


def nodewhy(period=0.5):
    """Robot still (or pushed by hand) before a node: every period, what each detector sees in the image and
    why it gives no node. Saves the last image as nodewhy.jpg and the top view of the floor with its tape as nodewhy_mask.png. Ctrl+C stops."""
    import corner_detector
    from corner_detector import CornerDetector

    cam = RobotHardware._open_camera()
    f = LineFollower(CAM_WIDTH, CAM_HEIGHT, CAM_FOV, TIME_STEP / 1000.0, V_MAX)
    det = NodeDetector(CAM_WIDTH, CAM_HEIGHT, CAM_FOV, f.grey)
    corners = CornerDetector(CAM_WIDTH, CAM_HEIGHT, CAM_FOV, parallel=None)
    print(f"Robot on the line, a node {100 * ZONE[0]:.0f}-{100 * ZONE[1]:.0f} cm ahead (robot centre to the crossing tape).")
    try:
        while True:
            img = cam.capture_array()
            det.find(img)
            centre, n, _ = corner_detector._detect(f.grey(img), corners.scale)
            d = corners._ahead(centre) if centre else None
            seen = f"'+' at {100 * d:.0f} cm" if d else "no '+'"
            print(f"bars: {det.why}\n  corners: {n} concave corners, {seen}")
            cv2.imwrite("nodewhy.jpg", img)
            top, tape = det.white(img)
            cv2.imwrite("nodewhy_mask.png", np.hstack([top, tape.astype(np.uint8) * 255]))
            time.sleep(period)
    except KeyboardInterrupt:
        print("saved nodewhy.jpg (camera) and nodewhy_mask.png (top view of the floor | what the node detector takes as tape)")
    finally:
        cam.stop()


class Wanderer(Navigator):
    """Free walk on the grid (gridrecord): at a node one exit drawn at random among the ones the camera saw.
    After a turn the robot stops and the sonar reads the exit (FSM state CHECK): blocked, that exit is dropped and
    another one is drawn among the rest; none left, back where it came from (U-turn). Blocked on a line: U-turn.
    The exit is checked after the turn, when the sonar faces it straight."""

    def __init__(self):
        self.options, self.backing = [], False

    def on_node(self, exits, s_node=0.0):
        self.backing = False
        if not exits:
            self.options = []
            return "STRAIGHT", "wander: exits unknown"
        self.options = [SIDE[e] for e in sorted(exits)]
        return self._draw(f"exits {''.join(sorted(exits))}")

    def _draw(self, why):
        order = random.choice(self.options)
        self.options.remove(order)
        return order, f"wander: {why}, drew {order}"

    def on_exit_blocked(self, exits):
        if self.options:
            return self._draw(f"exit blocked, {len(self.options)} left")
        if not self.backing:
            self.backing = True
            return "U_TURN", "wander: every exit blocked, back where it came from"
        return "STOP", "wander: blocked on every side, the way back too"

    def on_blocked(self, s, d):
        return "U_TURN", "wander: blocked"


class Recorder:
    """Writes camera images as JPEG in a thread (the FSM loop does not wait). image() -> file name, or "" if the
    writer is behind and the image was dropped."""

    def __init__(self, folder):
        self.folder, self.n, self.dropped = folder, 0, 0
        os.makedirs(os.path.join(folder, "frames"))
        self.queue = queue.Queue(maxsize=60)
        self.thread = threading.Thread(target=self._write, daemon=True)
        self.thread.start()

    def _write(self):
        while (job := self.queue.get()) is not None:
            name, img = job
            cv2.imwrite(os.path.join(self.folder, "frames", name), img, [cv2.IMWRITE_JPEG_QUALITY, 92])

    def image(self, img):
        name = f"{self.n:06d}.jpg"
        try:
            self.queue.put_nowait((name, img))
        except queue.Full:
            self.dropped += 1
            return ""
        self.n += 1
        return name

    def close(self):
        self.queue.put(None)
        self.thread.join()


def gridrecord(seconds=None):
    """Free walk on the grid for `seconds` (python3 diag.py gridrecord 120), recording every camera image and a log
    (recordings/<date>/): the node detectors can then be replayed and tuned offline on real images."""
    seconds = seconds or (float(sys.argv[2]) if len(sys.argv) > 2 else 120.0)
    folder = os.path.join(os.path.dirname(os.path.abspath(__file__)), "recordings", time.strftime("%Y%m%d-%H%M%S"))
    hw = RobotHardware(TIME_STEP)
    f = LineFollower(CAM_WIDTH, CAM_HEIGHT, CAM_FOV, TIME_STEP / 1000.0, hw.wheel_v_max())
    print(f"recording to {folder} for {seconds:.0f} s (Ctrl+C stops). Robot on a grid line: it starts now.")
    record_drive(hw, folder, seconds, FSM(f, nav=Wanderer), "START", node_detector(f))
    close_detectors()


def record_drive(hw, folder, seconds, fsm, cmd, det, restart=True):
    """Drives with the FSM for `seconds` (Ctrl+C stops), recording every camera image (frames/) and one log row per
    cycle (log.csv, read by replay_filters.py), plus meta.json. cmd: the first command ("START" or a MISSION tuple).
    det: node detector, None = no node searched (the 8). restart: back to START when the FSM stops (IDLE), else
    the drive ends there (a mission: arrived or gave up). Returns the LoopTimer of the drive."""
    f, bars = fsm.follower, bars_of(det)
    rec, timer = Recorder(folder), LoopTimer()
    with open(PARAMS_PATH) as src, open(os.path.join(folder, "meta.json"), "w") as out:
        json.dump({"camera": [CAM_WIDTH, CAM_HEIGHT, CAM_FOV], "time_step_ms": TIME_STEP, "seconds": seconds,
                   "command": cmd, "params_ini": src.read()}, out, indent=1)
    reason, t0 = "", hw.time()
    n_d0, t_d0 = getattr(hw, "n_d", None), time.monotonic()
    with open(os.path.join(folder, "log.csv"), "w", newline="", buffering=1) as fl:
        log = csv.writer(fl)
        log.writerow(["t", "image", "state", "v", "w", "s", "heading", "dist", "line", "node", "bars_shape",
                      "bars_why", "fsm_reason", "period_ms", "busy_ms"] + [f"{k}_ms" for k in STAGES])
        try:
            while hw.step() and hw.time() - t0 < seconds:
                timer.start()
                t, image, s = hw.time() - t0, hw.camera_image(), hw.distance()
                timer.lap("capture")
                node = look_for_node(det, fsm, image, s) if det is not None else None
                if node:
                    print(f"[NODE] t={t:6.2f}s s={s:.3f} d={node[0]:.3f} exits={sorted(node[1])}")
                timer.lap("nodes")
                f.points = []
                v, w = fsm.step(t, image, hw.obstacle_distance(), hw.heading(), cmd, s, node)
                timer.lap("fsm")
                if fsm.state == IDLE and fsm.history and not restart:
                    break
                cmd = "START" if fsm.state == IDLE else None      # never stays stopped: back to following
                hw.set_wheels(v - K * w, v + K * w)
                hw.aim_sonar(fsm.aim)            # as fsm.main(): into the curves, ahead in CHECK
                timer.lap("io")
                hw.show(f.points, f.cached_mask(image))
                name = rec.image(image)
                timer.lap("view")
                log.writerow([f"{t:.3f}", name, fsm.state, f"{v:.4f}", f"{w:.4f}", f"{s:.4f}",
                              f"{hw.heading():.4f}", f"{fsm.d:.3f}", fsm.line,
                              f"{node[0]:.3f} {''.join(sorted(node[1]))}" if node else "",
                              bars.shape if bars else "", bars.why if bars else "",
                              fsm.reason if fsm.reason != reason else ""] + timer.row())
                reason = fsm.reason
        except KeyboardInterrupt:
            print("  stopped (Ctrl+C)")
        finally:
            hw.set_wheels(0.0, 0.0)
            rec.close()
    timer.report()
    serial_rate(hw, n_d0, t_d0)
    print(f"{rec.n} images saved, {rec.dropped} dropped (writer behind) in {folder}\n"
          f"copy it to the Mac: tar czf rec.tgz -C {os.path.dirname(folder)} {os.path.basename(folder)}")
    return timer


def nodestop(hw=None):
    """The whole chain, with the FSM: follows the line, a node is announced, it stops on the node centre."""
    hw = hw or RobotHardware(TIME_STEP)
    f = LineFollower(CAM_WIDTH, CAM_HEIGHT, CAM_FOV, TIME_STEP / 1000.0, V_MAX)
    det = node_detector(f)
    fsm = FSM(f, nav=lambda: OrderList(["STOP"]))      # stops at the first node, as it does before a turn
    input("Robot on a line, 0.8 to 1.5 m before a node. Enter -> follows the line and stops on the node centre")
    cmd, periods, t0 = "START", [], time.monotonic()
    while hw.step():
        periods.append(time.monotonic() - t0)
        t0 = time.monotonic()
        image, s = hw.camera_image(), hw.distance()
        node = look_for_node(det, fsm, image, s)
        if node:
            print(f"  node announced {100 * node[0]:.1f} cm ahead, exits {''.join(sorted(node[1]))}")
        v, w = fsm.step(hw.time(), image, hw.obstacle_distance(), hw.heading(), cmd, s, node)
        cmd = None
        hw.set_wheels(v - K * w, v + K * w)
        if fsm.state == IDLE and len(fsm.history) > 1:
            break
    settle(hw)
    loop_report(periods)
    if not fsm.reason.startswith("STOP at the node"):
        print(f"  did not stop at a node: {fsm.reason}")
        return None
    est = hw.distance() - fsm.s_node
    real = ask_number(f"  odometry: robot centre {100 * est:+.1f} cm from the node centre. Measured with a tape "
                      f"(cm, + = past the centre, - = before it): ") / 100
    print(f"  node position error: {100 * (est - real):+.1f} cm (> 0: the robot thinks it is further than it is)")
    print("  with WHEEL_R calibrated (odometry), this is the camera distance: correct TILT_deg with nodecam")
    return est - real


def node_detector(f):
    """The node detector of the FSM (params.ini NODES): bars, corners or both. f: the LineFollower (its grey)."""
    det = NodeProcess(CAM_WIDTH, CAM_HEIGHT, CAM_FOV, f.grey)
    if NODES != "bars":
        from corner_detector import CornerDetector, NodeFusion

        corners = CornerDetector(CAM_WIDTH, CAM_HEIGHT, CAM_FOV, grey=f.grey if f.px_step == 1 else None)
        det = corners if NODES == "corners" else NodeFusion(det, corners)
    _detectors.append(det)
    return det


_detectors = []     # made by node_detector() and not closed yet


def close_detectors():
    """Stop the detection processes of every detector node_detector() made (diag.py full, after each step:
    otherwise each step would leave one process running)."""
    while _detectors:
        det = _detectors.pop()
        if hasattr(det, "close"):
            det.close()


def bars_of(det):
    """The NodeProcess (bars) inside det, for its shape and why, or None (NODES = "corners")."""
    if isinstance(det, NodeProcess):
        return det
    return next((d for d in getattr(det, "detectors", ()) if isinstance(d, NodeProcess)), None)


def serial_rate(hw, n0, t0):
    """D lines per second read from the Arduino since (n0, t0); it sends D_RATE. The Pi reads every line waiting at each
    cycle, so a slow loop does not lower it (its lines are only older: see the loop period). Fewer: lines lost or
    garbled on the link, or the Arduino rebooting (power)."""
    if n0 is None or not hasattr(hw, "n_d"):
        return None
    rate = (hw.n_d - n0) / max(time.monotonic() - t0, 1e-6)
    print(f"[SERIAL] {rate:.1f} D lines/s from the Arduino (it sends {D_RATE})"
          + ("" if rate >= 0.9 * D_RATE else ": lines lost or garbled, or the Arduino rebooted (cable, power?)"))
    return rate


def cpu_times():
    """(busy, total) jiffies of each CPU core since boot, from /proc/stat (idle = idle + iowait)."""
    with open("/proc/stat") as fh:
        rows = [line.split() for line in fh if line.startswith("cpu") and line[3].isdigit()]
    return [(sum(map(int, r[1:])) - int(r[4]) - int(r[5]), sum(map(int, r[1:]))) for r in rows]


def loop(seconds=None, hw=None, ask=True):
    """Cost of each part of the FSM loop on the Pi (python3 diag.py loop 20): robot still on a line before a
    crossing, motors off. The same stages as fsm.main(): capture, node detectors (params.ini NODES, even with
    MISSION = None), line follower ("fsm"), serial ("io"), live view (LIVE_VIEW). Then the load of each CPU core:
    the node detector runs in its own process, so its cost shows there (and in [NODES]), not in the stages."""
    seconds = seconds or (float(sys.argv[2]) if len(sys.argv) > 2 else 20.0)
    hw = hw or RobotHardware(TIME_STEP)
    f = LineFollower(CAM_WIDTH, CAM_HEIGHT, CAM_FOV, TIME_STEP / 1000.0, V_MAX)
    det = node_detector(f)
    timer = LoopTimer()
    if ask:
        input(f"Robot still on a line, a crossing 35-45 cm ahead; the motors stay off. Enter -> measures for "
              f"{seconds:.0f} s")
    cpu0, t_end = cpu_times(), time.monotonic() + seconds
    n_d0, t_d0 = getattr(hw, "n_d", None), time.monotonic()
    try:
        while hw.step() and time.monotonic() < t_end:
            timer.start()
            image, s = hw.camera_image(), hw.distance()
            timer.lap("capture")
            det.step(image, s, False)          # tracks the crossing (all the work), announces nothing
            timer.lap("nodes")
            f.points = []
            f.step(image, V0, 0.0, 0.0)
            timer.lap("fsm")
            hw.set_wheels(0.0, 0.0)
            timer.lap("io")
            hw.show(f.points, f.cached_mask(image))
            timer.lap("view")
            timer.row()
    except KeyboardInterrupt:
        print("  stopped")
    timer.report()
    serial_rate(hw, n_d0, t_d0)
    load = [100 * (b1 - b0) / max(t1 - t0, 1) for (b0, t0), (b1, t1) in zip(cpu0, cpu_times())]
    print("[CPU]  " + ", ".join(f"core {i} {p:.0f}%" for i, p in enumerate(load)))
    if timer.cycles:
        busy = np.mean([c[1] for c in timer.cycles])
        stage = dict(zip(STAGES, np.mean([c[2] for c in timer.cycles], axis=0)))
        top = max(stage, key=stage.get)
        print(f"  the loop works {busy:.0f} of {TIME_STEP} ms per cycle; heaviest stage: {top} ({stage[top]:.0f} ms)."
              + (" Full: move it to another core or lighten it." if busy > 0.8 * TIME_STEP else
                 f" Room left: TIME_STEP could go down to about {max(1.25 * busy, 33):.0f} ms"
                 " (not below 33: the camera gives about 30 images/s)."))


def nodegap(hw=None, nodes=4, v=0.15, max_metres=6.0):
    """Spacing between nodes: follows a straight grid line through several nodes, each one is placed with the
    odometry (s + d when it is announced). A camera bias is the same for every node and cancels in the
    differences: only the odometry scale counts, so run odometry first."""
    hw = hw or RobotHardware(TIME_STEP)
    f = LineFollower(CAM_WIDTH, CAM_HEIGHT, CAM_FOV, TIME_STEP / 1000.0, V_MAX)
    det = node_detector(f)
    input(f"Robot on a straight line of the grid, just before a node, with {nodes} nodes ahead and nothing on the "
          f"line. Enter -> follows the line at {100 * v:.0f} cm/s and places the nodes (Ctrl+C to stop early)")
    s0, vc, wc, found, periods, t = hw.distance(), 0.0, 0.0, [], [], time.monotonic()
    try:
        while hw.step() and len(found) < nodes and hw.distance() - s0 < max_metres:
            periods.append(time.monotonic() - t)
            t = time.monotonic()
            image, s = hw.camera_image(), hw.distance()
            node = det.step(image, s, True)
            if node:
                found.append(s + node[0])
                gap = f", {100 * (found[-1] - found[-2]):.1f} cm after the previous one" if len(found) > 1 else ""
                print(f"  node {len(found)} at {100 * (found[-1] - s0):.1f} cm (exits {''.join(sorted(node[1]))}){gap}")
            if hw.obstacle_distance() < D_STOP:
                print("  obstacle ahead: stopping")
                break
            out = f.step(image, v, vc, wc)
            if out.status == LOST:
                print("  line lost: stopping")
                break
            vc, wc = out.v, out.w
            hw.set_wheels(vc - K * wc, vc + K * wc)
    except KeyboardInterrupt:
        print("  stopped")
    settle(hw)
    loop_report(periods)
    gaps = np.diff(found)
    if len(gaps) == 0:
        print(f"  {len(found)} node(s) seen: at least 2 are needed (check the mask on the live view, /mask)")
        return None
    g = float(np.median(gaps))
    print(f"spacing: median {100 * g:.1f} cm, from {100 * gaps.min():.1f} to {100 * gaps.max():.1f} cm "
          f"({len(gaps)} gap(s))")
    print(f"MISSION (params.ini / params_entrega.ini): the last number = {g:.2f}   (m between nodes)")
    if gaps.max() - gaps.min() > 0.10:
        print("  the gaps differ by more than 10 cm: a node was missed or seen twice; look at the list and run again")
    return {"spacing": g, "gaps": [float(x) for x in gaps], "consistent": bool(gaps.max() - gaps.min() <= 0.10)}


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

    input("1/4 Robot on the floor, line a few cm to the LEFT of the robot's centre. Enter")
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
    cam.close()                         # diag.py full opens the camera again in the next steps
    f = LineFollower(CAM_WIDTH, CAM_HEIGHT, CAM_FOV, 0.064, V_MAX)
    g, m, out = draw(f, img)
    cv2.imwrite("diag.png", out)
    found = []
    for dj, row, mj, wj in f.bands:
        u = f._find_line(img, row, f.cu)
        if u is not None and abs(-mj * (u - f.cu)) <= 0.7 * wj:
            found.append((dj, -mj * (u - f.cu)))
    check("Line seen", bool(found), f"{len(found)}/{len(f.bands)} bands (diag.png saved)" if found else
          f"no band: grey {g.min():.0f}..{g.max():.0f}, adjust THRESHOLD ({THRESHOLD}) and look at diag.png")
    if found:
        y = found[0][1]
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
    return summary(results)


def summary(results):
    """Prints the checks that failed; returns results: [(name, ok, detail)]."""
    bad = [r for r in results if not r[1]]
    print(f"\n{len(results) - len(bad)}/{len(results)} OK")
    for name, _, detail in bad:
        print(f"  FAIL {name}: {detail}")
    return results


# ---------------------------------------------------------------- full: every check in one guided run
# python3 diag.py full [first step]: guides the steps below in order, records everything in ~/diag_sessions/<date>/
# (console.txt, images, drive logs, a copy of the code and params, summary.json), writes the params the measures
# give (params_proposed.ini, params_entrega_proposed.ini, params_changes.txt) and ends with one zip on the Desktop.
# The guide for the person at the robot: local/testes_robo_pi.html on the Mac.

SESSIONS = os.path.expanduser("~/diag_sessions")     # outside Robo_FSM/: replacing that folder keeps the sessions
CODE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))   # Robo_FSM/
DEFAULT_MISSION = ("MISSION", (0, 0), "E", (4, 3), 5, 0.975)
ENTREGA = os.path.join(os.path.dirname(os.path.abspath(__file__)), "params_entrega.ini")
HEADINGS = {"E": "east", "N": "north", "W": "west", "S": "south"}
THROTTLED = {0: "UNDER-VOLTAGE NOW", 1: "CPU frequency capped now", 2: "CPU throttled now", 3: "temperature limit now",
             16: "under-voltage happened since boot", 17: "frequency capping happened since boot",
             18: "throttling happened since boot", 19: "temperature limit happened since boot"}
SHAPES = [("plus", "a + crossing"),
          ("t_stem", "a T arriving on its stem (the bar goes left and right, nothing straight on)"),
          ("t_side", "a T arriving on one of its arms (the line goes on straight, one exit to a side)"),
          ("l", "an L corner (the line turns to one side, nothing straight on)")]


class ConsoleCopy:
    """Copies everything written to the terminal (fd 1 and 2: prints, libcamera and OpenCV messages, the corner
    detection process) into a file as well, so nothing has to be copied from the terminal by hand."""

    def __init__(self, path):
        self.file = open(path, "ab", buffering=0)  # noqa: SIM115  open for the whole session, closed by close()
        sys.stdout.flush()
        sys.stderr.flush()
        self.saved = (os.dup(1), os.dup(2))
        self.read_end, write_end = os.pipe()
        os.dup2(write_end, 1)
        os.dup2(write_end, 2)
        os.close(write_end)
        self.thread = threading.Thread(target=self._copy, daemon=True)
        self.thread.start()

    def _copy(self):
        while chunk := os.read(self.read_end, 4096):
            os.write(self.saved[0], chunk)
            self.file.write(chunk)

    def note(self, text):
        """Text the terminal shows but does not send through fd 1 (what the person types)."""
        self.file.write(text.encode())

    def close(self):
        """Back to the terminal only; the copy thread ends when no process writes to the pipe any more."""
        sys.stdout.flush()
        sys.stderr.flush()
        os.dup2(self.saved[0], 1)
        os.dup2(self.saved[1], 2)
        self.thread.join(timeout=2.0)
        self.file.close()


def logged_input(console):
    """input() that also writes the answer to the console file (the terminal echoes it, the file would not get it)."""

    def ask(prompt=""):
        sys.stdout.write(str(prompt))
        sys.stdout.flush()
        line = sys.stdin.readline()
        if not line:
            raise EOFError("no input: run diag.py full in a terminal")
        console.note(line if line.endswith("\n") else line + "\n")
        return line.rstrip("\n")

    return ask


def shell(cmd, timeout=15):
    """Output of a shell command, or why it failed."""
    try:
        r = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=timeout, check=False)
        return (r.stdout + r.stderr).strip() or f"(no output, exit {r.returncode})"
    except Exception as e:  # noqa: BLE001  any failure is only reported: the diagnosis goes on
        return f"(failed: {e})"


def version_of(package):
    try:
        from importlib.metadata import version

        return version(package)
    except Exception:  # noqa: BLE001
        return "not found"


def pi_state():
    """Power, temperature and memory of the Pi now, with warnings in plain words (start and end of a session)."""
    state = {"throttled": shell("vcgencmd get_throttled"), "temperature": shell("vcgencmd measure_temp"),
             "memory": shell("free -m")}
    warnings = []
    m = re.search(r"0x([0-9a-fA-F]+)", state["throttled"])
    if m:
        bits = int(m.group(1), 16)
        warnings += [text for bit, text in THROTTLED.items() if bits >> bit & 1]
    if any("under-voltage" in w.lower() for w in warnings):
        warnings.append("under-voltage: weak battery or power cable; motors, camera and Pi can misbehave. Charge it")
    state["warnings"] = warnings
    return state


def ini_value(path, section, key):
    """One value of an ini file written like params.ini (a Python literal), read without loading the file."""
    ini = configparser.ConfigParser(inline_comment_prefixes=("#",), interpolation=None)
    ini.optionxform = str
    if not ini.read(path):
        raise FileNotFoundError(f"parameters file not found: {path}")
    return ast.literal_eval(ini.get(section, key))


def spin_crossings(hw, f, direction, rate=0.5, turn=2 * math.pi):
    """Spins in place (direction +1 = left) by `turn` plus a margin; returns the encoder headings (rad from the start)
    where the line crossed the image centre. On a long straight line it crosses every half turn."""
    h0, y_prev, found = hw.heading(), None, []
    t_end = time.monotonic() + 2 * (turn + 0.5) / rate + 5.0      # wheels stalled: give up
    while hw.step() and time.monotonic() < t_end:
        turned = abs(hw.heading() - h0)
        if turned > turn + 0.5:
            break
        target = f.find_target(hw.camera_image())
        y = target[1] if target else None
        near = y is not None and abs(y) <= 2 * TURN_TOL_Y
        if near and y_prev is not None and y * y_prev < 0 and turned > 0.5:
            found.append(turned)
        y_prev = y if near else None
        hw.set_wheels(-K * direction * rate, K * direction * rate)
    settle(hw)
    return found


class FullDiag:
    """One guided session: the steps, the shared RobotHardware, the results of each step."""

    def __init__(self, folder):
        self.folder, self.name = folder, os.path.basename(folder)
        self.hw, self.hw_error = None, None
        self.results, self.steps = {}, []

    def hardware(self):
        """The RobotHardware shared by the steps (camera + Arduino), made once."""
        if self.hw is None:
            if self.hw_error:
                raise RuntimeError(f"the robot hardware did not start before: {self.hw_error}")
            print("starting the camera and the Arduino...")
            try:
                self.hw = RobotHardware(TIME_STEP)
            except Exception as e:
                self.hw_error = str(e)
                raise
        return self.hw

    def follower(self):
        return LineFollower(CAM_WIDTH, CAM_HEIGHT, CAM_FOV, TIME_STEP / 1000.0, self.hardware().wheel_v_max())

    # ---- the steps: each returns what goes into summary.json (and feeds params_proposed.ini)

    def system(self):
        """Pi state, the camera seen by libcamera, versions, and a copy of the code and params that ran."""
        info = {
            "model": shell("tr -d '\\0' < /proc/device-tree/model"),
            "os": shell("grep PRETTY_NAME /etc/os-release"),
            "kernel": shell("uname -a"),
            **pi_state(),
            "disk": shell(f"df -h {SESSIONS}"),
            "serial ports": shell("ls -l /dev/ttyACM* /dev/ttyUSB* 2>&1"),
            "cameras": shell("rpicam-hello --list-cameras 2>&1 || libcamera-hello --list-cameras 2>&1", 30),
            "python": sys.version.split()[0],
            "opencv": cv2.__version__,
            "numpy": np.__version__,
            "picamera2": version_of("picamera2"),
            "pyserial": version_of("pyserial"),
            "params file": PARAMS_PATH,
        }
        free = shutil.disk_usage(SESSIONS).free / 1e9
        if free < 1.0:
            info["warnings"].append(f"only {free:.1f} GB free on the SD card: the drives record ~300 MB of images")
        for key, value in info.items():
            if key != "warnings":
                print(f"{key}: {value}")
        for w in info["warnings"]:
            print(f"WARNING: {w}")
        return info

    def check(self):
        results = everything()
        return {"checks": results, "failed": [r[0] for r in results if not r[1]]}

    def sonar_sweep(self):
        sweep = sonar()
        near = [(cm, deg) for deg, cm in sweep if cm is not None and deg != 0]
        if not near:
            print("no echo at any side angle: was the box there?")
            return {"sweep": sweep, "box_at_deg": None}
        cm, deg = min(near)
        print(f"nearest echo {cm} cm at {deg:+d} deg: " + ("left, SONAR_SIGN is right" if deg > 0 else
              "RIGHT: SONAR_SIGN must flip (params_proposed.ini)"))
        return {"sweep": sweep, "box_at_deg": deg, "box_cm": cm}

    def sonar_distance(self, readings=40):
        """Sonar straight ahead: nothing in front (false echoes?), then a box at distances you measure. The FSM stops
        at D_STOP and checks exits at D_SCAN on these raw readings."""
        hw = self.hardware()
        hw.aim_sonar(0.0)
        out = []
        for true_cm in (None, 25, 50):
            if true_cm is None:
                input("Nothing in front of the sonar for 1.5 m. Enter -> 40 readings")
            else:
                while True:
                    answer = input(f"Box straight ahead, about {true_cm} cm from the FRONT OF THE SONAR. Measured "
                                   f"distance (cm; Enter = skip): ").strip().replace(",", ".")
                    try:
                        true_cm = float(answer) if answer else None
                        break
                    except ValueError:
                        print("  type a number")
                if true_cm is None:
                    continue
            got = []
            for _ in range(readings):
                hw.step()
                got.append(hw.obstacle_distance())
            valid = [100 * d for d in got if 0 < d < math.inf]
            res = {"true_cm": true_cm, "valid": len(valid), "of": readings, "no_echo": sum(d == math.inf for d in got)}
            if valid:
                res.update(median_cm=float(np.median(valid)), min_cm=min(valid), max_cm=max(valid))
            if true_cm is None:
                res["false_echoes"] = sum(v < 100 for v in valid)
                print(f"  nothing ahead: {res['false_echoes']} reading(s) under 1 m out of {readings} (should be 0)")
            elif valid:
                print(f"  median {res['median_cm']:.1f} cm (error {res['median_cm'] - true_cm:+.1f}), from "
                      f"{res['min_cm']:.0f} to {res['max_cm']:.0f}, {len(valid)}/{readings} valid")
            else:
                print(f"  no valid reading in {readings}: is the box in the sonar beam?")
            out.append(res)
        return out

    def odometry_scale(self):
        return odometry(self.hardware())

    def turn_scale(self, rate=0.5):
        """K (effective half track: it turns the wheel counts into an angle) from spins in place on a long straight
        line. The camera sees the line cross its centre every half turn; between two crossings the robot turned
        exactly 180 deg, whatever the camera delay, and the encoders say how much they counted for it."""
        if not USE_ENCODERS:
            print("USE_ENCODERS = False: the heading comes from the commanded speed, not the wheels; K not measured")
            return None
        hw, f = self.hardware(), self.follower()
        ratios = []
        for direction, name in ((1, "left"), (-1, "right")):
            input(f"Robot centred and square on the line again. Enter -> spins about 1.1 turns to the {name}")
            seen = spin_crossings(hw, f, direction, rate)
            gaps = [b - a for a, b in pairwise(seen) if abs(b - a - math.pi) < 0.6]
            print(f"  line crossings at {[round(math.degrees(c)) for c in seen]} deg by the encoders"
                  + (f": {[round(math.degrees(g)) for g in gaps]} deg counted for each true 180" if gaps else ""))
            ratios += [g / math.pi for g in gaps]
        if not ratios:
            print("  no two crossings half a turn apart: was there line ahead AND behind the robot?")
            return None
        r = float(np.median(ratios))
        print(f"params.ini [line_follower]: K = {K * r:.3f}   (now {K}, x{r:.3f} from {len(ratios)} half turn(s))")
        return {"K": K * r, "ratios": ratios, "WHEEL_R_used": WHEEL_R}

    def camera_geometry(self):
        return camcal(self.hardware().cam, save=True)

    def shapes(self, frames=15):
        """Each node shape at a measured distance, robot still: what the bars and the corners read over `frames`
        images. shape_<name>.jpg = the camera, shape_<name>_topview.png = top view | what the bars take as tape."""
        import corner_detector
        from corner_detector import CornerDetector

        hw = self.hardware()
        f = LineFollower(CAM_WIDTH, CAM_HEIGHT, CAM_FOV, TIME_STEP / 1000.0, V_MAX)
        det = NodeDetector(CAM_WIDTH, CAM_HEIGHT, CAM_FOV, f.grey)
        corners = CornerDetector(CAM_WIDTH, CAM_HEIGHT, CAM_FOV, parallel=None)
        out, rows = {}, []
        for key, text in SHAPES:
            while True:
                answer = input(f"\nRobot still and square on the line, {text}, 35-45 cm ahead.\n"
                               f"  Distance robot centre -> middle of the crossing tape (cm; Enter = skip this shape): ")
                answer = answer.strip().replace(",", ".")
                try:
                    true_d = float(answer) / 100 if answer else None
                    break
                except ValueError:
                    print("  type a number")
            if true_d is None:
                continue
            bars_d, names, corner_d = [], [], []
            for k in range(frames):
                img = hw.camera_image()
                found = det.find(img)
                centre, n, _ = corner_detector._detect(f.grey(img), corners.scale)
                cd = corners._ahead(centre) if centre else None
                if k == 0:
                    cv2.imwrite(f"shape_{key}.jpg", img)
                    top, tape = det.white(img)
                    cv2.imwrite(f"shape_{key}_topview.png", np.hstack([top, tape.astype(np.uint8) * 255]))
                if found:
                    bars_d.append(found[0])
                names.append(det.shape or "-")
                if cd:
                    corner_d.append(cd)
                rows.append([key, f"{true_d:.3f}", k, f"{found[0]:.3f}" if found else "", det.shape, det.why, n,
                             f"{cd:.3f}" if cd else ""])
            shape, count = Counter(names).most_common(1)[0]
            res = {"measured": true_d, "bars_seen": len(bars_d), "shape": shape, "shape_count": count,
                   "corners_seen": len(corner_d)}
            line = f"  bars: '{shape}' in {count}/{frames} images"
            if bars_d:
                res["bars_d"] = float(np.median(bars_d))
                line += f", crossing at {100 * res['bars_d']:.1f} cm (error {100 * (res['bars_d'] - true_d):+.1f})"
            else:
                line += f", no crossing: {det.why}"
            print(line)
            line = f"  corners: '+' in {len(corner_d)}/{frames} images"
            if corner_d:
                res["corners_d"] = float(np.median(corner_d))
                line += f", at {100 * res['corners_d']:.1f} cm (error {100 * (res['corners_d'] - true_d):+.1f})"
            print(line + ("" if key == "plus" else " (corners only see a +: none here is right)"))
            out[key] = res
        with open("shapes.csv", "w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(["shape", "measured_m", "frame", "bars_d_m", "bars_shape", "bars_why", "corners_n", "corners_d_m"])
            w.writerows(rows)
        return out

    def stops(self):
        errors = []
        while True:
            err = nodestop(self.hardware())
            close_detectors()
            if err is not None:
                errors.append(err)
            print(f"  errors so far (cm): {[round(100 * e, 1) for e in errors]}")
            if len(errors) >= 3 or input("Again from another start? [Y/n] ").strip().lower() not in ("", "y", "yes"):
                return {"errors_m": errors}

    def gap(self):
        return nodegap(self.hardware())

    def loop_cost(self):
        hw = self.hardware()
        print("1/2 with the live view: open http://<pi-ip>:8000/mask on the Mac while it measures")
        loop(20, hw)
        close_detectors()
        stream, hw.stream = hw.stream, None
        try:
            print("\n2/2 without the live view (same position)")
            loop(20, hw, ask=False)
        finally:
            hw.stream = stream
        return {"done": True}

    def drive(self, folder, seconds, nav, cmd, nodes, restart=True):
        hw, f = self.hardware(), self.follower()
        print("it starts now (Ctrl+C stops it early)")
        fsm = FSM(f, nav=nav) if nav else FSM(f)
        timer = record_drive(hw, folder, seconds, fsm, cmd, node_detector(f) if nodes else None, restart)
        states = Counter(new for _, _, new, _ in fsm.history)
        note = input("\nWhat did you see? A short note: it left the track, hit something, stopped off a node...\n"
                     "  (Enter = all as expected): ").strip()
        with open(os.path.join(folder, "notes.txt"), "w") as fh:
            fh.write((note or "all as expected") + "\n")
        return {"cycles": len(timer.cycles), "transitions": len(fsm.history), "entered": dict(states),
                "last_reason": fsm.reason, "note": note or "all as expected"}

    def eight(self):
        return self.drive("drive_8", 60, Explorer, "START", nodes=False)

    def uturn(self):
        return self.drive("drive_uturn", 40, Explorer, "START", nodes=False)

    def wander(self):
        return self.drive("drive_grid", 120, Wanderer, "START", nodes=True)

    def mission(self):
        return self.drive("drive_mission", 180, None, MISSION or DEFAULT_MISSION, nodes=True, restart=False)

    def delivery(self):
        return self.drive("drive_delivery", 180, None, ini_value(ENTREGA, "fsm", "MISSION"), nodes=True,
                          restart=False)

    # ---- the session

    def save(self):
        with open("summary.json", "w") as fh:
            json.dump({"session": self.name, "params": PARAMS_PATH, "steps": self.steps, "results": self.results},
                      fh, indent=1, default=str)

    def run(self, n, step):
        """One step: instructions, Enter / s / q, run it, record how it went. False: the person chose to finish."""
        key, title, text, method = step
        print(f"\n{'=' * 78}\nSTEP {n}/{len(FULL_STEPS)}: {title}\n{'=' * 78}\n{text}")
        if key != "system":
            while (answer := input("Enter = start   s = skip   q = finish now (makes the zip): ").strip().lower()) \
                    not in ("", "s", "q"):
                pass
            if answer == "q":
                return False
            if answer == "s":
                self.steps.append({"n": n, "step": key, "status": "skipped"})
                self.save()
                return True
        t0, status = time.monotonic(), "done"
        try:
            self.results[key] = getattr(self, method)()
        except KeyboardInterrupt:
            status = "stopped (Ctrl+C)"
            print("\nstep stopped (Ctrl+C)")
        except Exception as e:  # noqa: BLE001  one failed step must not end the session: it is logged and skipped
            status = f"error: {e}"
            traceback.print_exc()
        finally:
            close_detectors()
            if self.hw is not None:
                try:
                    self.hw.set_wheels(0.0, 0.0)
                except Exception as e:  # noqa: BLE001  serial gone: the Arduino stops by itself after 300 ms
                    print(f"could not send the stop to the wheels: {e}")
        self.steps.append({"n": n, "step": key, "status": status, "seconds": round(time.monotonic() - t0)})
        self.save()
        print(f"--- step {n} {status} ({time.monotonic() - t0:.0f} s)")
        return True


FULL_STEPS = [   # (key, title, what to do, FullDiag method)
    ("system", "Pi, camera, power, code",
     ("Nothing to do: reads the Pi state (power, temperature, memory, disk), the camera and the versions."),
     "system"),
    ("check", "Hardware check",
     ("Robot on the floor, the line a few cm to the LEFT of the robot centre. Halfway it asks you to LIFT the robot "
      "(wheels free) and to say which wheels turned."), "check"),
    ("sonar", "Sonar servo direction",
     ("Robot still. A box about 30 cm away on the LEFT (between the front-left diagonal and right beside the "
      "robot); nothing ahead or to the right for 1 m. The sonar sweeps from right to left, up to 90 deg each side: "
      "watch that its mount does not hit the robot."), "sonar_sweep"),
    ("sonardist", "Sonar distance",
     ("Sonar straight ahead. First nothing in front for 1.5 m; then a box straight ahead at about 25 cm and at about "
      "50 cm from the FRONT OF THE SONAR, each distance measured with the tape."), "sonar_distance"),
    ("odometry", "Odometry scale (WHEEL_R)",
     ("Robot on a straight line with 1.3 m free ahead; tape measure ready. Mark a fixed point of the robot on the "
      "floor; it drives 1 m by the encoders and you measure how far the mark moved. 2-3 runs."), "odometry_scale"),
    ("turn", "Turn scale (K)",
     ("Robot centred and square in the middle of a long straight line: at least 50 cm of line AHEAD and 50 cm "
      "BEHIND it, nothing around (it spins in place). It spins about 1.1 turns to the left, then to the right. "
      "Nothing to measure: the camera does it."), "turn_scale"),
    ("camcal", "Camera geometry (H_CAM, TILT, X_CAM)",
     ("Robot still and square on a straight line, a crossing ahead. Ruler for the lens height; tape for 3 distances "
      "(23-40 cm) from the robot centre (midway between the axles) to the middle of the crossing tape."),
     "camera_geometry"),
    ("shapes", "Node shapes, robot still",
     ("In turn: a +, a T from its stem, a T from one arm, an L. Each 35-45 cm ahead, distance measured with the "
      "tape. 15 images each, about 3 s."), "shapes"),
    ("nodestop", "Stop on the node centre",
     ("Robot on a line 0.8-1.5 m before a node. It drives, sees the node and stops on its centre; you measure where "
      "it stopped (+ = past the centre). 3 runs from different starts."), "stops"),
    ("nodegap", "Spacing between nodes",
     "Robot on a straight grid line just before a node, with 4 nodes and nothing else ahead (about 4 m).", "gap"),
    ("loop", "Loop cost and CPU load",
     "Robot still on a line, a crossing 35-45 cm ahead. Motors off. 2 x 20 s: with and without the live view.",
     "loop_cost"),
    ("eight", "Drive the 8 (60 s, recorded)",
     ("Robot on the 8 track. It follows the line for 60 s and records every image. Once, put a box in its way on a "
      "straight, and take it away after it stops. At the end, type what you saw."), "eight"),
    ("uturn", "Obstacle that stays: U-turn (40 s, recorded)",
     ("Robot on a straight of the 8 (or a grid line). Put a box on the line 60-80 cm ahead and LEAVE it there: it "
      "must slow down, stop at about 25 cm, wait 2 s, turn back (U-turn) and follow the line away."), "uturn"),
    ("wander", "Random walk on the grid (120 s, recorded)",
     ("Robot on a grid line. At each node it draws one of the exits it sees; after a turn the sonar checks it and a "
      "blocked one is replaced. Move boxes onto exits and lines during the walk."), "wander"),
    ("mission", "Grid mission (up to 180 s, recorded)",
     ("Robot on the centre of node {start} facing {heading}; goal {goal} ({n}x{n} grid, {g} m between nodes). "
      "Once, put a box on its path a little after a node."), "mission"),
    ("delivery", "Delivery mission of params_entrega.ini (up to 180 s, recorded)",
     ("Robot on the centre of node {d_start} facing {d_heading}; goal {d_goal}. Nothing on the way: this is the run "
      "of the delivery."), "delivery"),
]


def set_value(text, section, key, value):
    """ini text with `key = ...` of [section] set to value, its comment kept. Returns (text, old value or None)."""
    out, current, old = [], None, None
    for line in text.splitlines(keepends=True):
        m = re.match(r"\s*\[(\w+)\]", line)
        if m:
            current = m.group(1)
        elif current == section and old is None:
            m = re.match(rf"({re.escape(key)}\s*=\s*)(.*?)(\s+#.*)?(\r?\n)?$", line)
            if m:
                old = m.group(2)
                line = m.group(1) + value + (m.group(3) or "") + (m.group(4) or "")
        out.append(line)
    return "".join(out), old


def set_spacing(text, g):
    """Every MISSION tuple of the text (also the commented example) with g as its last number (m between nodes)."""
    return re.sub(r'(\("MISSION".*,\s*)([0-9.]+)(\s*\))', lambda m: f"{m.group(1)}{g:.3f}{m.group(3)}", text)


def proposed_changes(results):
    """[(section, key, new value text, why)] and notes, from the measures of the session."""
    changes, notes = [], []
    odo = results.get("odometry")
    if odo:
        changes.append(("robot_hardware", "WHEEL_R", f"{odo['WHEEL_R']:.4f}",
                        (f"odometry: tape / encoders over {len(odo['ratios'])} run(s), "
                         f"{[round(r, 4) for r in odo['ratios']]}")))
        notes.append(f"WHEEL_R also goes in robo_fsm.ino (const float WHEEL_R = {odo['WHEEL_R']:.4f};), then upload")
    cam = results.get("camcal")
    if cam and cam["rms_px"] <= 2.0:
        why = f"camcal: fit on {len(cam['distances'])} distances, residual {cam['rms_px']:.2f} px"
        changes += [("line_follower", "H_CAM", f"{cam['H_CAM']:.3f}", why),
                    ("line_follower", "TILT_deg", f"{cam['TILT_deg']:.1f}", why),
                    ("line_follower", "X_CAM", f"{cam['X_CAM']:.3f}", why),
                    ("line_follower", "BANDS", str(cam["BANDS"]), "camcal: the follower looks at the same image rows"),
                    ("line_follower", "GAIN", f"{cam['GAIN']:.2f}", "camcal: the same steering with the new geometry")]
    elif cam:
        notes.append(f"camcal residual {cam['rms_px']:.2f} px (> 2): a distance was mistyped or measured from "
                     "another point; camera values NOT proposed")
    turn = results.get("turn")
    if turn:
        k, why = turn["K"], f"turn: encoders vs camera over {len(turn['ratios'])} half turn(s)"
        if odo:                         # the heading is WHEEL_R / K: a new WHEEL_R moves K with it
            k *= odo["WHEEL_R"] / turn["WHEEL_R_used"]
            why += ", scaled to the new WHEEL_R"
        changes.append(("line_follower", "K", f"{k:.3f}", why))
    for res in results.get("sonardist") or []:
        if res["true_cm"] is None and res.get("false_echoes"):
            notes.append(f"sonar: {res['false_echoes']} echo(es) under 1 m with nothing ahead: false stops possible")
        elif res["true_cm"] is not None and "median_cm" in res and abs(res["median_cm"] - res["true_cm"]) > 3:
            notes.append(f"sonar reads {res['median_cm']:.0f} cm for {res['true_cm']:.0f} cm: D_STOP "
                         f"({100 * D_STOP:.0f} cm) and D_SCAN ({100 * D_SCAN:.0f} cm) are compared with this reading")
    shapes = results.get("shapes") or {}
    tilts = [tilt_for(r["measured"], r["bars_d"]) for r in shapes.values()
             if "bars_d" in r and abs(r["bars_d"] - r["measured"]) <= MAX_ERR]
    if tilts and not (cam and cam["rms_px"] <= 2.0):
        changes.append(("line_follower", "TILT_deg", f"{math.degrees(float(np.median(tilts))):.1f}",
                        f"node shapes: camera distance vs tape on {len(tilts)} shape(s) (camcal not available)"))
    son = results.get("sonar")
    if son and son.get("box_at_deg") is not None and son["box_at_deg"] < 0:
        changes.append(("robot_hardware", "SONAR_SIGN", str(-SONAR_SIGN),
                        f"sonar: the box on the left was seen at {son['box_at_deg']:+d} deg"))
    gap = results.get("nodegap")
    if gap and gap["consistent"]:
        changes.append(("fsm", "MISSION spacing", f"{gap['spacing']:.3f}",
                        f"nodegap: median of {[round(x, 3) for x in gap['gaps']]}"))
    elif gap:
        notes.append(f"nodegap gaps {[round(x, 3) for x in gap['gaps']]} differ by more than 10 cm: spacing NOT "
                     "proposed (a node missed or seen twice)")
    return changes, notes


def write_proposals(results, name):
    """params_proposed.ini and params_entrega_proposed.ini (the params with the measured values) and
    params_changes.txt (what changed, from what, why). Nothing in Robo_FSM/ is touched."""
    changes, notes = proposed_changes(results)
    report = [f"Params proposed by diag.py full, session {name}", ""]
    sources = [("params_proposed.ini", PARAMS_PATH),
               ("params_entrega_proposed.ini", os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                                            "params_entrega.ini"))]
    for out_name, src in sources:
        if not os.path.exists(src) or (out_name != "params_proposed.ini" and os.path.samefile(src, PARAMS_PATH)):
            continue
        with open(src) as fh:
            text = fh.read()
        report.append(f"{out_name} (from {os.path.basename(src)}):")
        for section, key, value, why in changes:
            if key == "MISSION spacing":
                text = set_spacing(text, float(value))
                report.append(f"  [{section}] MISSION last number -> {value}   ({why})")
                continue
            text, old = set_value(text, section, key, value)
            report.append(f"  [{section}] {key}: {old} -> {value}   ({why})" if old is not None else
                          f"  [{section}] {key}: not in the file, NOT set")
        if not changes:
            report.append("  no change: nothing was measured, or every measure agreed with the params")
        header = (f"# Proposed by diag.py full (session {name}): see params_changes.txt. Check it, then copy it over "
                  f"pi/{os.path.basename(src)}.\n")
        with open(out_name, "w") as fh:
            fh.write(header + text)
        report.append("")
    report += ["Notes:"] + [f"  - {n}" for n in notes] if notes else []
    with open("params_changes.txt", "w") as fh:
        fh.write("\n".join(report) + "\n")
    print("\n".join(report))


def full():
    """Every check of this file in one guided run; one zip at the end (see the section comment above)."""
    first = int(sys.argv[2]) if len(sys.argv) > 2 else 1
    name = "diag_" + time.strftime("%Y%m%d-%H%M%S")
    folder = os.path.join(SESSIONS, name)
    os.makedirs(folder)
    os.chdir(folder)                   # every file a step writes (diag.png, camcal_*.jpg...) lands in the session
    console = ConsoleCopy("console.txt")
    builtins.input = logged_input(console)
    start, heading, goal, n, g = (MISSION or DEFAULT_MISSION)[1:]
    try:
        d_start, d_heading, d_goal = ini_value(ENTREGA, "fsm", "MISSION")[1:4]
    except Exception as e:  # noqa: BLE001  the delivery step will report it; the others go on
        print(f"WARNING: no MISSION in {ENTREGA}: {e}")
        d_start, d_heading, d_goal = "?", "E", "?"
    steps = [(k, t, x.format(start=start, heading=HEADINGS[heading], goal=goal, n=n, g=g, d_start=d_start,
                             d_heading=HEADINGS.get(d_heading, d_heading), d_goal=d_goal), m)
             for k, t, x, m in FULL_STEPS]
    session = FullDiag(folder)
    shutil.copytree(CODE_DIR, "code", ignore=shutil.ignore_patterns(     # whatever the first step: the zip always
        "recordings", "__pycache__", "*.jpg", "*.png", "*.csv", "*.zip", "*.tgz", "*.mp4"))   # says what ran
    shutil.copy(PARAMS_PATH, "params_used.ini")
    print(f"FULL DIAGNOSIS: {len(steps)} steps, about 75 min. Everything is saved in {folder}\n"
          f"Follow the guide (testes_robo_pi.html). At each step: Enter = start, s = skip, q = finish now.\n"
          f"Ctrl+C during a step stops that step; at the Enter/s/q question it finishes the session.\n"
          f"The zip is made at the end, whatever happened.")
    try:
        for k, step in enumerate(steps, 1):
            if k >= first and not session.run(k, step):
                break
    except (KeyboardInterrupt, EOFError):
        print("\nfinishing now")
    finally:
        print("\nPi state at the end of the session:")
        session.results["system_end"] = end = pi_state()
        for key, value in end.items():
            print(f"{key}: {value}")
        try:
            write_proposals(session.results, name)
        except Exception:  # noqa: BLE001  the zip must still be made
            traceback.print_exc()
        if session.hw is not None:
            session.hw.stop()
        session.save()
        desktop = os.path.expanduser("~/Desktop")
        dest = os.path.join(desktop if os.path.isdir(desktop) else SESSIONS, name)
        print(f"\nmaking the zip ({len(session.steps)} step(s) recorded)...")
        console.close()                # console.txt complete before it goes into the zip
        os.chdir(SESSIONS)
        path = shutil.make_archive(dest, "zip", root_dir=SESSIONS, base_dir=name)
        print(f"\nDONE. The zip: {path} ({os.path.getsize(path) / 1e6:.0f} MB)\n"
              f"Everything is in it (the terminal too, in console.txt): take it to the Mac with the VNC Viewer "
              f"file transfer and send it.")


if __name__ == "__main__":
    modes = {"all": everything, "camera": camera, "live": live, "wheels": wheels, "encoders": encoders, "pi": pi,
             "speed": speed, "turn": turn, "sonar": sonar, "odometry": odometry, "camcal": camcal, "nodecam": nodecam,
             "nodestop": nodestop, "nodegap": nodegap, "nodewhy": nodewhy, "gridrecord": gridrecord, "loop": loop,
             "full": full}
    modes[sys.argv[1] if len(sys.argv) > 1 else "all"]()
