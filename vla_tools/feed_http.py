#!/usr/bin/env python3
# feed_http.py -- serve a ROS compressed-image topic to a web browser as MJPEG.
# 21 Sep 2026. HARDWARE tool. Standard library + rclpy only (no Flask needed).
#
#   python3 ~/vla_tools/feed_http.py                       # annotated feed (YOLO boxes)
#   python3 ~/vla_tools/feed_http.py raw                   # raw camera
#   python3 ~/vla_tools/feed_http.py /some/other/compressed 8082
#
# Then open   http://127.0.0.1:8081/          (page)      in the PC's browser
#             http://127.0.0.1:8081/stream    (bare MJPEG)
#             http://127.0.0.1:8081/snapshot.jpg
#
# The frames already arrive at the PC over ROS; this only re-serves them
# locally, so it adds NO Wi-Fi load -- as long as you open it on the PC.
# It listens on 127.0.0.1 only (22 Sep): another device on the hotspot watching it
# would take airtime from the robot. VLA_FEED_BIND=0.0.0.0 overrides, knowingly.
import os, sys, threading, time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage

TOPICS = {"annotated": "/vla/annotated/compressed",
          "raw": "/robot1/oakd/rgb/preview/image_raw/compressed"}
arg = sys.argv[1] if len(sys.argv) > 1 else "annotated"
TOPIC = TOPICS.get(arg, arg)
PORT = int(sys.argv[2]) if len(sys.argv) > 2 else 8081

latest = {"jpg": None, "n": 0}
lock = threading.Lock()

class Feed(Node):
    def __init__(self):
        super().__init__("vla_feed_http")
        # best-effort reader matches both the reliable annotated writer and the
        # best-effort camera writer
        self.create_subscription(CompressedImage, TOPIC, self.cb, qos_profile_sensor_data)
    def cb(self, m):
        with lock:
            latest["jpg"] = bytes(m.data); latest["n"] += 1

class H(BaseHTTPRequestHandler):
    def log_message(self, *a): pass          # keep the console quiet
    def do_GET(self):
        if self.path.startswith("/stream"):
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-cache"); self.end_headers()
            last = -1
            try:
                while True:
                    with lock: jpg, n = latest["jpg"], latest["n"]
                    if jpg is not None and n != last:
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n" % len(jpg))
                        self.wfile.write(jpg); self.wfile.write(b"\r\n"); last = n
                    time.sleep(0.03)
            except (BrokenPipeError, ConnectionResetError):
                return
        elif self.path.startswith("/snapshot"):
            with lock: jpg = latest["jpg"]
            if jpg is None: self.send_response(503); self.end_headers(); return
            self.send_response(200); self.send_header("Content-Type", "image/jpeg")
            self.send_header("Content-Length", str(len(jpg))); self.end_headers(); self.wfile.write(jpg)
        else:
            body = ("<html><body style='margin:0;background:#111;color:#ddd;font-family:sans-serif'>"
                    "<div style='padding:6px 10px;font-size:14px'>VLA robot feed &mdash; " + TOPIC +
                    "</div><img src='/stream' style='width:100%;max-width:1024px;display:block'>"
                    "</body></html>").encode()
            self.send_response(200); self.send_header("Content-Type", "text/html")
            self.send_header("Content-Length", str(len(body))); self.end_headers(); self.wfile.write(body)

def main():
    rclpy.init(); node = Feed()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    srv = ThreadingHTTPServer((os.environ.get("VLA_FEED_BIND", "127.0.0.1"), PORT), H)   # PC only (22 Sep): a laptop watching it would load the robot's channel
    print(f"[feed_http] serving {TOPIC} at http://127.0.0.1:{PORT}/  (Ctrl+C to stop)", flush=True)
    try: srv.serve_forever()
    except KeyboardInterrupt: pass
    finally:
        srv.server_close(); node.destroy_node(); rclpy.shutdown()

if __name__ == "__main__":
    main()
