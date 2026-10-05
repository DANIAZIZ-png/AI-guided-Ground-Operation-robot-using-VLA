# Measure end-to-end camera latency: frame timestamp (Pi clock) vs arrival (PC clock).
# Both machines are NTP-synced, so the difference is real pipeline+link latency.
import time, statistics
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, LaserScan

rclpy.init()
n = Node('vla_latency_probe')
lat_rgb, lat_scan = [], []

def mk(store):
    def cb(m):
        st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        now = n.get_clock().now().nanoseconds * 1e-9
        store.append((now - st) * 1000.0)
    return cb

n.create_subscription(CompressedImage, '/robot1/oakd/rgb/preview/image_raw/compressed',
                      mk(lat_rgb), qos_profile_sensor_data)
n.create_subscription(LaserScan, '/robot1/scan', mk(lat_scan), qos_profile_sensor_data)
t0 = time.time()
while time.time() - t0 < 20:
    rclpy.spin_once(n, timeout_sec=0.2)

for name, v in (("RGB frame", lat_rgb), ("laser scan", lat_scan)):
    if not v:
        print("%s: no samples" % name); continue
    v.sort()
    print("%-12s n=%3d   median %6.1f ms   p90 %6.1f ms   max %6.1f ms"
          % (name, len(v), statistics.median(v), v[int(len(v)*0.9)], v[-1]))

if lat_rgb:
    med = statistics.median(lat_rgb) / 1000.0
    print()
    print("  bearing error while rotating, at this latency:")
    for w in (1.0, 0.6, 0.4):
        import math
        err_deg = math.degrees(w * med)
        print("    %.1f rad/s (%4.0f deg/s) -> %5.1f deg stale  = %.2f m sideways at 2.3 m"
              % (w, math.degrees(w), err_deg, 2.3 * math.tan(math.radians(err_deg))))
