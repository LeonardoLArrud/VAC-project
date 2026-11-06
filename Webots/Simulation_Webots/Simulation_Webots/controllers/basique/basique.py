import math
import time
import numpy as np
from controller import Robot
from collections import deque


# sonar
THRESHOLD = 2000          # sonar reading below this = obstacle
CHECK_TIME = 2.0          # s (simulated time)

TIME_STEP = 64
H_CAM = 0.132
X_CAM = 0.104
TILT = math.radians(30)
D = [0.20, 0.26, 0.31, 0.36]
K = 0.135
WHEEL_R = 0.0325
V0 = 0.40
TAU_EFF = 0.10
A_LAT = 1.0
A_UP = 0.5
A_DOWN = 1.5
W_SEARCH = 1.5
GREY_THRESHOLD = 150
WIDTH = 480

robot = Robot()
dt = TIME_STEP / 1000.0
cam = robot.getDevice("PiCam")
cam.enable(TIME_STEP)
sensor = robot.getDevice("URM37")
sensor.enable(TIME_STEP)
right_motors = [robot.getDevice(n) for n in ("Motor1", "Motor3")]
left_motors = [robot.getDevice(n) for n in ("Motor2", "Motor4")]
for m in right_motors + left_motors:
    m.setPosition(float("inf"))
    m.setVelocity(0.0)
# sensor
history = deque(maxlen=3)
def read_sonar():
    history.append(sensor.getValue())
    return sorted(history)[len(history) // 2]

def stop_motors():
    for m in right_motors + left_motors:
        m.setVelocity(0.0)
def check_obstacle():
    """Call when d <= THRESHOLD. Stops and watches for 2 s (simulated time).
    If the obstacle is still there, turns 180 deg. Returns True if it turned."""
    stop_motors()
    t0 = robot.getTime()
    while robot.getTime() - t0 < CHECK_TIME:
        if robot.step(TIME_STEP) == -1:
            return False
        if read_sonar() > THRESHOLD:
            return False          # it moved away: not a static obstacle
    spin_to_line()                    # spins until the camera sees the line again (instead of spinning for a time)
    history.clear()                   # old readings (of the obstacle) no longer count
    u_prev[:] = [CU] * len(bands)     # forgets where the line was before the turn
    return True


PHI_MAX = right_motors[0].getMaxVelocity()
V_WHEEL_MAX = PHI_MAX * WHEEL_R

NU = WIDTH
NV = round(cam.getHeight() * WIDTH / cam.getWidth())
F = (NU / 2) / math.tan(cam.getFov() / 2)
CU, CV = (NU - 1) / 2, (NV - 1) / 2

bands = []
for dj in D:
    q = math.tan(math.atan2(H_CAM, dj - X_CAM) - TILT)
    row = round(CV + F * q)
    if 2 <= row < NV - 2:
        den = math.sin(TILT) + q * math.cos(TILT)
        bands.append((dj, row, H_CAM / (F * den), H_CAM * math.tan(cam.getFov() / 2) / den))
u_prev = [CU] * len(bands)


def grey():
    img = np.frombuffer(cam.getImage(), np.uint8).reshape((cam.getHeight(), cam.getWidth(), 4))
    step = cam.getWidth() // WIDTH
    return img[::step, ::step, :3].mean(axis=2)[:NV, :NU]


def find_line(g, row, u_ref):
    white = (g[row - 2 : row + 3].mean(axis=0) > GREY_THRESHOLD).astype(np.int8)
    edge = np.diff(np.concatenate(([0], white, [0])))
    start, end = np.flatnonzero(edge == 1), np.flatnonzero(edge == -1)
    ok = end - start >= 2
    if not ok.any():
        return None
    centres = (start[ok] + end[ok] - 1) / 2
    return float(centres[np.argmin(np.abs(centres - u_ref))])


def wheels(v_l, v_r):
    for m in left_motors:
        m.setVelocity(float(np.clip(v_l / WHEEL_R, -PHI_MAX, PHI_MAX)))
    for m in right_motors:
        m.setVelocity(float(np.clip(v_r / WHEEL_R, -PHI_MAX, PHI_MAX)))
def spin_in_place(rate=W_SEARCH):
    """Spins in place. rate in rad/s of the robot: positive = left, negative = right.
    Call it at every loop step to keep spinning."""
    wheels(-K * rate, K * rate)


def turn_angle(degrees, rate=W_SEARCH):
    """Turns by an angle and stops. Positive = left, negative = right.
    Blocks until done (the camera is not processed during the turn)."""
    direction = 1.0 if degrees >= 0 else -1.0
    target = math.radians(abs(degrees))
    turned = 0.0
    while turned < target:
        spin_in_place(direction * abs(rate))
        if robot.step(TIME_STEP) == -1:
            break
        turned += abs(rate) * dt
    stop_motors()

def look_for_target(g):
    """The same search as the main loop, without changing u_prev. Returns (dj, y) or None."""
    for j in reversed(range(len(bands))):
        dj, row, mj, wj = bands[j]
        u = find_line(g, row, CU)
        if u is None:
            continue
        y = -mj * (u - CU)
        if abs(y) <= 0.7 * wj:
            return (dj, y)
    return None


def spin_to_line(min_deg=100, max_deg=420, rate=1.0, tol_y=0.04):
    """Spins in place (to the left) until the camera sees the line again, centred ahead.
    Turns at least min_deg, so as not to stop on the line it came from, and at most
    max_deg (more than 360 on purpose: the wheels slip and turn less than computed).
    Returns True if it found the line again."""
    lo = math.radians(min_deg)
    hi = math.radians(max_deg)
    turned = 0.0
    found = False
    while turned < hi:
        spin_in_place(abs(rate))
        if robot.step(TIME_STEP) == -1:
            break
        turned += abs(rate) * dt
        if turned >= lo:
            target = look_for_target(grey())
            if target is not None and abs(target[1]) <= tol_y:
                found = True
                break
    stop_motors()
    return found


v = 0.0
w = 0.0
memory = None
travelled = 0.0
side = 1.0
last_print = 0.0

while robot.step(TIME_STEP) != -1:
    g = grey()
    target = None
    d = read_sonar()
    if d <= THRESHOLD:
        turned_back = check_obstacle()
        v = w = 0.0                   # the robot stood still, so the internal state is reset too
        if turned_back:
            memory = None
            travelled = 0.0
            continue
    for j in reversed(range(len(bands))):
        dj, row, mj, wj = bands[j]
        u = find_line(g, row, u_prev[j])
        if u is None:
            continue
        u_prev[j] = u
        y = -mj * (u - CU)
        if abs(y) <= 0.7 * wj:
            target = (dj, y)
            break

    if target is not None:
        memory = target
        travelled = 0.0
    elif memory is not None:
        dpsi = w * dt
        dx = v * dt * math.cos(dpsi / 2)
        dy = v * dt * math.sin(dpsi / 2)
        x0, y0 = memory[0] - dx, memory[1] - dy
        memory = (math.cos(dpsi) * x0 + math.sin(dpsi) * y0, -math.sin(dpsi) * x0 + math.cos(dpsi) * y0)
        travelled += v * dt
        if memory[0] <= 0.05 or travelled > D[1]:
            memory = None
        target = memory

    if target is None:
        v = 0.0
        w = W_SEARCH * side
        wheels(-K * w, K * w)
        status = "LOST"
    else:
        L, Y = target
        side = 1.0 if Y >= 0 else -1.0
        c = 2 * Y / (L * L + Y * Y)
        v_target = min(V0, 0.163 * L / TAU_EFF, math.sqrt(A_LAT / max(abs(c), 1e-6)), V_WHEEL_MAX / (1 + K * abs(c)))
        v = min(max(v_target, v - A_DOWN * dt), v + A_UP * dt)
        w = v * c
        wheels(v - K * w, v + K * w)
        status = f"L={L:.2f} Y={Y:+.3f} c={c:+.2f} v={v:.2f}"

    t = robot.getTime()
    if t - last_print >= 1.0:
        print(f"[PP] t={t:6.2f}s {status}")
        last_print = t
