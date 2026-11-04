"""Every step of the line follower and of the node detector on a recording (diag.py gridrecord), image by image,
with today's params.ini.

python3 replay_filters.py recordings/<date> [first last]
Writes <recording>-filters/ (the recording is not touched): one panel per image and filters.mp4.
Panel: camera | grey | blur / threshold | opening | decision, and the decision at full size on the right:
the mask (white), the bands (red lines, as on /mask of the live view), the point aimed at (green) or seen but out
of range (magenta), and every white run on a band with its contrast to the floor beside it: tape (>= MIN_CONTRAST,
green) or not (orange). The follower is called as the FSM calls it in the logged state.
Far right, the node detector (node_detector.py) on its top view of the floor: the tape, the line, the bar and the
arms it traced (see nodes()), and NODE when the FSM would get one.
"""
import csv
import json
import os
import sys
from types import SimpleNamespace

import cv2
import numpy as np

from camera_stream import CameraStream
from fsm import SPINNING, look_for_node
from line_follower import MIN_CONTRAST, LineFollower
from node_detector import ARM, ARMS, RES, ZONE, NodeDetector

SMALL = (320, 240)
TAPE, NOT_TAPE = (0, 200, 0), (0, 140, 255)      # BGR


def label(img, text):
    (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
    cv2.rectangle(img, (0, 0), (tw + 10, th + 12), (0, 0, 0), -1)
    cv2.putText(img, text, (5, th + 6), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    return img


def small(img, text):
    if img.ndim == 2:
        img = cv2.cvtColor(img * 255 if img.max() <= 1 else img, cv2.COLOR_GRAY2BGR)
    return label(cv2.resize(img, SMALL, interpolation=cv2.INTER_AREA), text)


def decision(f, view, img, points, mask):
    """The live view /mask image (CameraStream._render), plus the runs of every band and their contrast."""
    out = cv2.imdecode(np.frombuffer(CameraStream._render(view, img, points, True, mask), np.uint8), cv2.IMREAD_COLOR)
    for dj, row, mj, wj in f.bands:
        below = row + 30 < out.shape[0]            # the nearest band is at the bottom edge: draw above it
        bar, text = (row + 5, row + 24) if below else (row - 9, row - 14)
        for a, b, c in zip(*f.tape_runs(img, row)):
            colour = TAPE if c >= MIN_CONTRAST else NOT_TAPE
            cv2.rectangle(out, (int(a), bar), (int(b) - 1, bar + 4), colour, -1)
            cv2.putText(out, "-" if np.isnan(c) else f"{c:.0f}", (int(a), text), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        colour, 2)
        cv2.putText(out, f"{100 * dj:.1f} cm", (f.nu - 75, row - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1)
    return out


def nodes(det, img, node, size=(640, 480)):
    """The node detector on this image (det.trace): the top view of the floor turned upright with the tape (blue),
    the line (cyan), the bar (yellow), the crossing centre (red) and its arms: tape (green), none (red), not in
    view (grey); ZONE (white lines). Below: the shape, why there is no node, or NODE when the FSM gets one."""
    t = det.trace
    if t is None:                                     # stopped before turning upright: plain top view
        top, tape = det.white(img)
        out = cv2.cvtColor(top, cv2.COLOR_GRAY2BGR)
        out[tape] = (255, 160, 0)
        M = np.float32([[1, 0, 0], [0, 1, 0]])
    else:
        h, w = t["up"].shape
        M = t["M"]
        out = cv2.cvtColor(cv2.warpAffine(t["top"], M, (w, h)), cv2.COLOR_GRAY2BGR)
        out[t["up"]] = (255, 160, 0)
    k = min(size[0] / out.shape[1], (size[1] - 60) / out.shape[0])
    out = cv2.resize(out, None, fx=k, fy=k, interpolation=cv2.INTER_NEAREST)
    P = lambda c, r: tuple(int(round(v * k)) for v in (c, r))       # upright top view px -> panel px
    for x in ZONE:                                    # ZONE rows of the top view, turned like it
        r = (det.X[0] - x) / RES
        ends = [M @ [c, r, 1.0] for c in (0, len(det.Y) - 1)]
        cv2.line(out, P(*ends[0]), P(*ends[1]), (255, 255, 255), 1)
        cv2.putText(out, f"{100 * x:.0f} cm", P(*ends[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
    if t and t["line"]:
        c = (t["line"][0] + t["line"][1] - 1) / 2
        cv2.line(out, P(c, 0), P(c, t["up"].shape[0] - 1), (255, 255, 0), 1)
    if t and t["bar"]:
        rc, cc = t["centre"]
        cv2.line(out, P(0, rc), P(t["up"].shape[1] - 1, rc), (0, 255, 255), 1)
        for a, (dr, dc) in ARMS.items():
            seen = t["arms"][a]
            colour = (0, 220, 0) if seen else (128, 128, 128) if seen is None else (0, 0, 255)
            p0 = P(cc + dc * ARM[0] / RES, rc + dr * ARM[0] / RES)
            p1 = P(cc + dc * ARM[1] / RES, rc + dr * ARM[1] / RES)
            cv2.line(out, p0, p1, colour, 6 if seen else 2)
            cv2.putText(out, a, p1, cv2.FONT_HERSHEY_SIMPLEX, 0.6, colour, 2)
        cv2.circle(out, P(cc, rc), 6, (0, 0, 255), -1)
    page = np.zeros((size[1], size[0], 3), np.uint8)
    page[: out.shape[0], : out.shape[1]] = out
    text = f"NODE {100 * node[0]:.0f} cm, exits {''.join(sorted(node[1]))}" if node else det.why
    for j, part in enumerate([text[i : i + 75] for i in range(0, len(text), 75)][:3]):
        cv2.putText(page, part, (5, size[1] - 42 + 18 * j), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (0, 255, 0) if node else (255, 255, 255), 1)
    return page


def main(folder, first=0, last=None):
    folder = folder.rstrip("/")
    out_dir = folder + "-filters"
    os.makedirs(out_dir, exist_ok=True)
    with open(os.path.join(folder, "log.csv")) as fl:
        rows = [r for r in csv.DictReader(fl) if r["image"]][first:last]
    if not rows:
        raise SystemExit(f"no images in {folder}/log.csv between {first} and {last}")
    with open(os.path.join(folder, "meta.json")) as fm:
        w, h, fov = json.load(fm)["camera"]          # as recorded (fov in radians)
    f = LineFollower(w, h, fov, 0.064, 1.0)
    view = SimpleNamespace(follower=f, rows=[b[1] for b in f.bands], px_step=f.px_step)
    det = NodeDetector(w, h, fov, f.grey)
    video, prev = None, None
    for i, r in enumerate(rows):
        img = cv2.imread(os.path.join(folder, "frames", r["image"]))
        if img is None:
            print(f"{r['image']}: cannot read, skipped")
            continue
        state = r["state"]
        if prev == "TURN" and state != "TURN":
            f.reset()                          # as FSM.end_turn()
        if state in ("FOLLOW", "SEARCH"):      # FSM.follow() and FSM.look(): step(), tracking u_prev
            status = f.step(img, 1.0, 0.0, 0.0).status
            used = ""
        else:                                  # TURN: find_target(); other states do not use the follower
            status = "OK" if f.find_target(img) is not None else "LOST"
            used = "" if state == "TURN" else " (not used in this state)"
        prev = state
        node = look_for_node(det, SimpleNamespace(state=state), img, float(r["s"]))   # as fsm.main()
        if state in SPINNING:                  # the FSM looks for no node while spinning: shown, not used
            det.find(img)
            det.why = "(spinning: not used) " + det.why
        stages = f.mask_stages(img)
        panels = [small(img, f"{r['image']} t={float(r['t']):.2f}s {state}")]
        panels += [small(s, name) for name, s in stages]
        full = decision(f, view, img, f.points, stages[-1][1])
        label(full, f"now: {status}{used}   recorded: {r['line']}   tape: contrast >= {MIN_CONTRAST}")
        panels.append(small(full, "decision"))
        grid = np.vstack([np.hstack(panels[:3]), np.hstack(panels[3:6])])
        big = (2 * SMALL[0], 2 * SMALL[1])
        page = np.hstack([grid, cv2.resize(full, big), label(nodes(det, img, node, big), "nodes (top view)")])
        cv2.imwrite(os.path.join(out_dir, r["image"]), page, [cv2.IMWRITE_JPEG_QUALITY, 85])
        if video is None:
            video = cv2.VideoWriter(os.path.join(out_dir, "filters.mp4"), cv2.VideoWriter_fourcc(*"mp4v"), 15,
                                    (page.shape[1], page.shape[0]))
        video.write(page)
        if i % 200 == 0:
            print(f"{i}/{len(rows)}")
    video.release()
    print(f"{len(rows)} panels and filters.mp4 in {out_dir}")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        raise SystemExit(__doc__)
    args = [int(a) for a in sys.argv[2:4]]
    main(sys.argv[1], *args)
