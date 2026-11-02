import cv2
import numpy as np
from controller import Robot

BASE_SPEED = 7.0
KP = 4.0
KD = 0.3
MAX_SPEED = 10.0
SEARCH_TURN = 2.5

WORK_WIDTH = 320
ROI_TOP = 0.55
ROI_BOTTOM = 0.95
LINE_THRESHOLD = 150
MIN_LINE_FRACTION = 0.005

TIME_STEP = 64

robot = Robot()
dt = TIME_STEP / 1000.0

camera = robot.getDevice("PiCam")
camera.enable(TIME_STEP)

left_motors = [robot.getDevice(n) for n in ("Motor2", "Motor4")]
right_motors = [robot.getDevice(n) for n in ("Motor1", "Motor3")]
for m in left_motors + right_motors:
    m.setPosition(float("inf"))
    m.setVelocity(0.0)


def line_error(bgra: np.ndarray) -> tuple[float | None, np.ndarray]:
    h, w = bgra.shape[:2]
    small = cv2.resize(bgra, (WORK_WIDTH, int(h * WORK_WIDTH / w)), interpolation=cv2.INTER_AREA)
    gray = cv2.cvtColor(small, cv2.COLOR_BGRA2GRAY)
    sh, sw = gray.shape
    top, bottom = int(ROI_TOP * sh), int(ROI_BOTTOM * sh)
    roi = gray[top:bottom, :]
    _, mask = cv2.threshold(roi, LINE_THRESHOLD, 255, cv2.THRESH_BINARY)

    debug = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    cv2.rectangle(debug, (0, top), (sw - 1, bottom - 1), (255, 0, 0), 1)
    cv2.line(debug, (sw // 2, top), (sw // 2, bottom), (0, 255, 0), 1)

    m = cv2.moments(mask, binaryImage=True)
    if m["m00"] < MIN_LINE_FRACTION * mask.size:
        return None, debug
    cx = m["m10"] / m["m00"]
    cy = m["m01"] / m["m00"] + top
    cv2.circle(debug, (int(cx), int(cy)), 4, (0, 0, 255), -1)
    return (cx - sw / 2) / (sw / 2), debug


def set_speeds(left: float, right: float) -> None:
    left = float(np.clip(left, -MAX_SPEED, MAX_SPEED))
    right = float(np.clip(right, -MAX_SPEED, MAX_SPEED))
    print(f"left speeds: {left} | right speeds: {right}")
    for m in left_motors:
        m.setVelocity(left)
    for m in right_motors:
        m.setVelocity(right)


prev_error = 0.0
last_print = 0.0
while robot.step(TIME_STEP) != -1:
    image = np.frombuffer(camera.getImage(), np.uint8).reshape((camera.getHeight(), camera.getWidth(), 4))
    error, debug = line_error(image)

    if error is None:
        turn = SEARCH_TURN if prev_error >= 0 else -SEARCH_TURN
        set_speeds(turn, -turn)
        status = "lost"
    else:
        derivative = (error - prev_error) / dt
        turn = KP * error + KD * derivative
        set_speeds(BASE_SPEED + turn, BASE_SPEED - turn)
        prev_error = error
        status = f"e={error:+.2f} turn={turn:+.2f}"

    t = robot.getTime()
    if t - last_print >= 1.0:
        print(f"[PD] t={t:6.2f}s {status}")
        last_print = t

    cv2.imshow("seguidor_pd", debug)
    cv2.waitKey(1)
