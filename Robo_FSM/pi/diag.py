"""Checks on the real robot (Arduino with robo_fsm.ino).
python3 diag.py [all] | camera | live | wheels | encoders | pi | speed | turn | odometry | nodecam | nodestop | nodegap | nodewhy | gridrecord [seconds]
Distance to a node = odometry s + camera d. Calibrate them in this order:
  odometry  follows a straight line for 1 m by the encoders, you measure it: WHEEL_R
  nodecam   robot still, a crossing at a distance you measure: TILT_deg
  nodestop  the whole chain: follows the line and stops on a node centre (as before a turn), you measure the error
  nodegap   follows a grid line through several nodes: the spacing between nodes (last number of MISSION)
nodewhy: robot still before a node: what the bars and corners detectors see, and why they give no node
gridrecord: free walk on the grid recording every camera image + log (recordings/), to tune the detectors offline
"""
import csv
import json
import math
import os
import queue
import random
import sys
import threading
import time

import cv2
import numpy as np
import serial

from fsm import D_STOP, FSM, IDLE, NODES, TIME_STEP, look_for_node
from line_follower import H_CAM, K, LOST, THRESHOLD, TILT, X_CAM, LineFollower
from navigators import SIDE, Navigator, OrderList
from node_detector import ZONE, NodeDetector
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
    RobotHardware,
)


def draw(f, img):
    g, m = f.grey(img), f.mask(img)
    out = cv2.cvtColor(g.astype(np.uint8), cv2.COLOR_GRAY2BGR)
    for dj, row, mj, wj in f.bands:
        cv2.line(out, (0, row), (f.nu - 1, row), (0, 255, 0), 1)
        out[row, m[row] > 0] = (0, 0, 255)
        u = f._find_line(m, row, f.cu)
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
        u = f._find_line(m, row, f.cu)
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
        return
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


def nodecam(cam=None, frames=20):
    """Camera distance to a node: robot still, a crossing at distances you measure. Corrects TILT_deg."""
    from corner_detector import CornerDetector

    own = cam is None
    cam = cam or RobotHardware._open_camera()
    f = LineFollower(CAM_WIDTH, CAM_HEIGHT, CAM_FOV, TIME_STEP / 1000.0, V_MAX)
    det = NodeDetector(CAM_WIDTH, CAM_HEIGHT, CAM_FOV, f.grey)
    corners = CornerDetector(CAM_WIDTH, CAM_HEIGHT, CAM_FOV, parallel=None)
    print(f"Robot still on the line, a crossing ahead. Measure from the robot centre (midway between the front and rear "
          f"axles) to the middle of the crossing tape. Bars read {100 * ZONE[0]:.0f}-{100 * ZONE[1]:.0f} cm, corners "
          f"~30-55 cm (a '+' only): try 2 or 3 distances, e.g. 45, 52 and 60 cm.")
    tilts = []
    try:
        while True:
            text = input("  measured distance in cm (Enter = done): ").strip().replace(",", ".")
            if not text:
                break
            true_d = float(text) / 100
            for name, d, n in node_distances(cam, f, det, corners, frames):
                if d is None:
                    print(f"    {name}: not seen in {frames} images")
                    continue
                tilts.append(tilt_for(true_d, d))
                print(f"    {name}: {100 * d:.1f} cm ({n}/{frames} images), error {100 * (d - true_d):+.1f} cm"
                      f" -> TILT {math.degrees(tilts[-1]):.2f} deg")
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
    why it gives no node. Saves the last image as nodewhy.jpg and the bars mask as nodewhy_mask.png. Ctrl+C stops."""
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
            print(f"bars (threshold {det.threshold:.0f}): {det.why}\n  corners: {n} concave corners, {seen}")
            cv2.imwrite("nodewhy.jpg", img)
            cv2.imwrite("nodewhy_mask.png", det.white(img).astype(np.uint8) * 255)
            time.sleep(period)
    except KeyboardInterrupt:
        print("saved nodewhy.jpg (camera) and nodewhy_mask.png (what the bars detector sees as white)")
    finally:
        cam.stop()


class Wanderer(Navigator):
    """Free walk on the grid (gridrecord): at a node a random exit (straight counts twice); blocked: U-turn."""

    def on_node(self, exits, s_node=0.0):
        if not exits:
            return "STRAIGHT", "wander: exits unknown"
        options = [SIDE[e] for e in sorted(exits)] + (["STRAIGHT"] if "S" in exits else [])
        return random.choice(options), f"wander: exits {''.join(sorted(exits))}"

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
    det = NodeDetector(CAM_WIDTH, CAM_HEIGHT, CAM_FOV, f.grey)
    fsm = FSM(f, nav=Wanderer)
    rec = Recorder(folder)
    with open(PARAMS_PATH) as src, open(os.path.join(folder, "meta.json"), "w") as out:
        json.dump({"camera": [CAM_WIDTH, CAM_HEIGHT, CAM_FOV], "time_step_ms": TIME_STEP, "seconds": seconds,
                   "params_ini": src.read()}, out, indent=1)
    print(f"recording to {folder} for {seconds:.0f} s (Ctrl+C stops). Robot on a grid line: it starts now.")
    cmd, reason = "START", ""
    with open(os.path.join(folder, "log.csv"), "w", newline="", buffering=1) as fl:
        log = csv.writer(fl)
        log.writerow(["t", "image", "state", "v", "w", "s", "heading", "dist", "line", "node", "bars_threshold",
                      "bars_why", "fsm_reason"])
        try:
            while hw.step() and hw.time() < seconds:
                t, image, s = hw.time(), hw.camera_image(), hw.distance()
                node = look_for_node(det, fsm, image, s)
                if node:
                    print(f"[NODE] t={t:6.2f}s s={s:.3f} d={node[0]:.3f} exits={sorted(node[1])}")
                f.points = []
                v, w = fsm.step(t, image, hw.obstacle_distance(), hw.heading(), cmd, s, node)
                cmd = "START" if fsm.state == IDLE else None      # never stays stopped: back to following
                hw.set_wheels(v - K * w, v + K * w)
                hw.show(f.points, f.cached_mask(image))
                log.writerow([f"{t:.3f}", rec.image(image), fsm.state, f"{v:.4f}", f"{w:.4f}", f"{s:.4f}",
                              f"{hw.heading():.4f}", f"{fsm.d:.3f}", fsm.line,
                              f"{node[0]:.3f} {''.join(sorted(node[1]))}" if node else "",
                              f"{det.threshold:.0f}", det.why, fsm.reason if fsm.reason != reason else ""])
                reason = fsm.reason
        except KeyboardInterrupt:
            pass
        finally:
            hw.set_wheels(0.0, 0.0)
            rec.close()
    print(f"{rec.n} images saved, {rec.dropped} dropped (writer behind) in {folder}\n"
          f"copy it to the Mac: tar czf rec.tgz -C {os.path.dirname(folder)} {os.path.basename(folder)}")


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
        return
    est = hw.distance() - fsm.s_node
    real = ask_number(f"  odometry: robot centre {100 * est:+.1f} cm from the node centre. Measured with a tape "
                      f"(cm, + = past the centre, - = before it): ") / 100
    print(f"  node position error: {100 * (est - real):+.1f} cm (> 0: the robot thinks it is further than it is)")
    print("  with WHEEL_R calibrated (odometry), this is the camera distance: correct TILT_deg with nodecam")


def node_detector(f):
    """The node detector of the FSM (params.ini NODES): bars, corners or both. f: the LineFollower (its grey)."""
    det = NodeDetector(CAM_WIDTH, CAM_HEIGHT, CAM_FOV, f.grey)
    if NODES != "bars":
        from corner_detector import CornerDetector, NodeFusion

        corners = CornerDetector(CAM_WIDTH, CAM_HEIGHT, CAM_FOV, grey=f.grey if f.px_step == 1 else None)
        det = corners if NODES == "corners" else NodeFusion(det, corners)
    return det


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
        return
    g = float(np.median(gaps))
    print(f"spacing: median {100 * g:.1f} cm, from {100 * gaps.min():.1f} to {100 * gaps.max():.1f} cm "
          f"({len(gaps)} gap(s))")
    print(f"MISSION (params.ini / params_entrega.ini): the last number = {g:.2f}   (m between nodes)")
    if gaps.max() - gaps.min() > 0.10:
        print("  the gaps differ by more than 10 cm: a node was missed or seen twice; look at the list and run again")


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
    f = LineFollower(CAM_WIDTH, CAM_HEIGHT, CAM_FOV, 0.064, V_MAX)
    g, m, out = draw(f, img)
    cv2.imwrite("diag.png", out)
    found = []
    for dj, row, mj, wj in f.bands:
        u = f._find_line(m, row, f.cu)
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
    summary(results)


def summary(results):
    bad = [r for r in results if not r[1]]
    print(f"\n{len(results) - len(bad)}/{len(results)} OK")
    for name, _, detail in bad:
        print(f"  FAIL {name}: {detail}")


if __name__ == "__main__":
    modes = {"all": everything, "camera": camera, "live": live, "wheels": wheels, "encoders": encoders, "pi": pi,
             "speed": speed, "turn": turn, "odometry": odometry, "nodecam": nodecam, "nodestop": nodestop,
             "nodegap": nodegap, "nodewhy": nodewhy, "gridrecord": gridrecord}
    modes[sys.argv[1] if len(sys.argv) > 1 else "all"]()
