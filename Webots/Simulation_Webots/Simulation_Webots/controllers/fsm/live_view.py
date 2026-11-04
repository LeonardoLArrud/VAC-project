"""Live view of what the follower sees, in the browser (same idea as Robo_FSM/pi/camera_stream.py).

Started by fsm.py when LIVE_VIEW is True. Open while the simulation runs:
    http://localhost:8000/        camera image (follower resolution) + band rows + line points
    http://localhost:8000/mask    what the follower sees as white, after the filter
Green = point used as the target, magenta = line found but too far to the side.
numpy + zlib only (PNG frames polled by the page), so it needs nothing beyond what Webots has.
"""
import struct
import threading
import zlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np

PORT = 8000
FPS = 10
MARK = 3                                       # half size of the point markers, px
BAND, USED, REJECTED = (255, 0, 0), (0, 255, 0), (255, 0, 255)  # RGB

PAGE = b"""<!doctype html><meta name=viewport content="width=device-width">
<title>Webots camera</title>
<body style="margin:0;background:#111;color:#ddd;font-family:sans-serif;text-align:center">
<p><a style="color:#8cf" href="/">camera</a> | <a style="color:#8cf" href="/mask">mask</a>
 &nbsp; <span style="color:#0f0">&#9632; target</span>
 <span style="color:#f0f">&#9632; line, out of range</span></p>
<img id=v style="width:100%;max-width:960px;image-rendering:pixelated">
<script>
const src = location.pathname == '/mask' ? '/mask.png' : '/camera.png';
function next() { const i = new Image(); i.onload = () => { v.src = i.src; setTimeout(next, 1000 / FPS); };
  i.onerror = () => setTimeout(next, 1000); i.src = src + '?' + Date.now(); }
next();
</script>
""".replace(b"FPS", str(FPS).encode())


def _png(rgb):
    h, w, _ = rgb.shape
    raw = np.concatenate([np.zeros((h, 1), np.uint8), rgb.reshape(h, w * 3)], axis=1).tobytes()

    def chunk(kind, data):
        return struct.pack(">I", len(data)) + kind + data + struct.pack(">I", zlib.crc32(kind + data))

    return (b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
            + chunk(b"IDAT", zlib.compress(raw, 1)) + chunk(b"IEND", b""))


class LiveView:
    """Keeps the latest frame and line points of the follower and serves them as PNG."""

    def __init__(self, follower, port=PORT):
        self.follower = follower
        self.frame, self.points = None, []
        self.lock = threading.Lock()
        view = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *args):
                pass

            def do_GET(self):
                path = self.path.split("?")[0]
                if path in ("/", "/mask"):
                    body, kind = PAGE, "text/html"
                elif path in ("/camera.png", "/mask.png"):
                    body, kind = view._render(mask=path == "/mask.png"), "image/png"
                    if body is None:
                        self.send_error(503, "no frame yet")
                        return
                else:
                    self.send_error(404)
                    return
                self.send_response(200)
                self.send_header("Content-Type", kind)
                self.send_header("Cache-Control", "no-cache")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                try:
                    self.wfile.write(body)
                except (BrokenPipeError, ConnectionResetError):
                    pass

        try:
            self.server = ThreadingHTTPServer(("0.0.0.0", port), Handler)
        except OSError as e:
            print(f"[live view] off: port {port} busy ({e})")
            self.server = None
            return
        self.server.daemon_threads = True
        threading.Thread(target=self.server.serve_forever, daemon=True).start()
        print(f"[live view] http://localhost:{port}/")

    def update(self, image, points):
        with self.lock:
            self.frame, self.points = image, list(points)

    def _render(self, mask):
        with self.lock:
            frame, points = self.frame, self.points
        if frame is None:
            return None
        f = self.follower
        if mask:
            img = np.repeat((f.mask(frame) * 255)[:, :, None], 3, axis=2)
        else:
            p = f.px_step
            img = np.ascontiguousarray(frame[::p, ::p, 2::-1][: f.nv, : f.nu])  # BGRA -> RGB
        for _, row, _, _ in f.bands:
            img[[row - 2, row + 2], :] = BAND
        for row, u, used in points:
            r, c = round(row), round(u)
            img[max(r - MARK, 0) : r + MARK + 1, max(c - MARK, 0) : c + MARK + 1] = USED if used else REJECTED
        return _png(img)
