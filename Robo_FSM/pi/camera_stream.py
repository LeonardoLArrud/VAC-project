"""Live camera view in the browser (MJPEG over HTTP).

With the FSM:   python3 robot_hardware.py      -> the stream starts on its own
Camera only:    python3 camera_stream.py       -> no Arduino needed, motors stay off (points from a
                                                 search from the image centre, as in the U-turn)

Open on the PC (same network):
    http://<pi-ip>:8000/        camera image + band rows used by the follower
    http://<pi-ip>:8000/mask    what the follower sees as white (grey > THRESHOLD)

With the FSM, both views also mark the line centre found on each band:
green = point used as the target, magenta = line found but too far to the side.
"""
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np

from line_follower import THRESHOLD, LineFollower

PORT = 8000
FPS = 10           # stream rate; lower it if the FSM loop gets slow
QUALITY = 70       # JPEG quality 0..100
MARK = 5           # half size of the point markers, in output pixels
USED, REJECTED = (0, 255, 0), (255, 0, 255)  # BGR

try:
    import simplejpeg  # installed with picamera2

    def _jpeg(img):
        return simplejpeg.encode_jpeg(np.ascontiguousarray(img), quality=QUALITY, colorspace="BGR")
except ImportError:
    import cv2

    def _jpeg(img):
        return cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, QUALITY])[1].tobytes()

PAGE = b"""<!doctype html><meta name=viewport content="width=device-width">
<title>Robot camera</title>
<body style="margin:0;background:#111;color:#ddd;font-family:sans-serif;text-align:center">
<p><a style="color:#8cf" href="/">camera</a> | <a style="color:#8cf" href="/mask">mask</a>
 &nbsp; <span style="color:#0f0">&#9632; target</span>
 <span style="color:#f0f">&#9632; line, out of range</span></p>
<img id=v style="max-width:100%">
<script>v.src=(location.pathname=='/mask'?'/stream_mask':'/stream')</script>
"""


class CameraStream:
    """Keeps the latest frame and serves it. Call update(frame, points) from the main loop."""

    def __init__(self, width, height, fov, port=PORT):
        self.frame, self.points = None, ()
        self.lock = threading.Condition()
        # rows where the follower looks for the line (same maths as LineFollower)
        self.rows = [b[1] for b in LineFollower(width, height, fov, 0.064, 1.0).bands]
        self.px_step = max(1, width // 480)
        stream = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *args):
                pass

            def do_GET(self):
                if self.path in ("/", "/mask"):
                    self.send_response(200)
                    self.send_header("Content-Type", "text/html")
                    self.end_headers()
                    self.wfile.write(PAGE)
                elif self.path in ("/stream", "/stream_mask"):
                    stream._serve(self, mask=self.path == "/stream_mask")
                else:
                    self.send_error(404)

        self.server = ThreadingHTTPServer(("0.0.0.0", port), Handler)
        self.server.daemon_threads = True
        threading.Thread(target=self.server.serve_forever, daemon=True).start()
        print(f"[camera] live view on http://<pi-ip>:{port}/")

    def update(self, frame, points=()):
        """points: (row, u, used) in follower pixels, as LineFollower.points."""
        with self.lock:
            self.frame, self.points = frame, list(points)
            self.lock.notify_all()

    def _render(self, frame, points, mask):
        if mask:
            p = self.px_step
            grey = frame[::p, ::p, :3].mean(axis=2)
            img = np.repeat(((grey > THRESHOLD) * 255).astype(np.uint8)[:, :, None], 3, axis=2)
            scale = 1
        else:
            img = frame[:, :, :3].copy()
            scale = self.px_step
        for r in self.rows:
            r *= scale
            img[max(r - 2, 0) : r + 3 : 4, :] = (0, 0, 255)  # red lines (BGR) at rows r-2 and r+2
        for row, u, used in points:
            r, c = round(row * scale), round(u * scale)
            img[max(r - MARK, 0) : r + MARK + 1, max(c - MARK, 0) : c + MARK + 1] = USED if used else REJECTED
        return _jpeg(img)

    def _serve(self, h, mask):
        h.send_response(200)
        h.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        h.send_header("Cache-Control", "no-cache")
        h.end_headers()
        try:
            while True:
                with self.lock:
                    self.lock.wait(timeout=1.0)
                    frame, points = self.frame, self.points
                if frame is None:
                    continue
                data = self._render(frame, points, mask)
                h.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                              + str(len(data)).encode() + b"\r\n\r\n" + data + b"\r\n")
                time.sleep(1.0 / FPS)
        except (BrokenPipeError, ConnectionResetError):
            pass  # browser closed


if __name__ == "__main__":
    from robot_hardware import CAM_FOV, CAM_HEIGHT, CAM_WIDTH, RobotHardware

    cam = RobotHardware._open_camera()
    stream = CameraStream(CAM_WIDTH, CAM_HEIGHT, CAM_FOV)
    follower = LineFollower(CAM_WIDTH, CAM_HEIGHT, CAM_FOV, 1.0 / FPS, 1.0)
    try:
        while True:
            frame = cam.capture_array()
            follower.find_target(frame)
            stream.update(frame, follower.points)
            time.sleep(1.0 / FPS)
    except KeyboardInterrupt:
        pass
    finally:
        cam.stop()
