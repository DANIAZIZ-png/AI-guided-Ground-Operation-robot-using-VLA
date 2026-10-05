# Is the camera's timestamp offset CONSTANT (clock epoch problem) or GROWING
# (frames queueing up = real buffering latency)? Sample for 60 s in buckets.
import time, statistics
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, LaserScan

rclpy.init()
n = Node('vla_drift_probe')
rgb, scan = [], []
def mk(store):
    def cb(m):
        st = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        now = n.get_clock().now().nanoseconds * 1e-9
        store.append((now, (now - st) * 1000.0))
    return cb
n.create_subscription(CompressedImage, '/robot1/oakd/rgb/preview/image_raw/compressed',
                      mk(rgb), qos_profile_sensor_data)
n.create_subscription(LaserScan, '/robot1/scan', mk(scan), qos_profile_sensor_data)

t0 = time.time()
while time.time() - t0 < 60:
    rclpy.spin_once(n, timeout_sec=0.2)

def buckets(store, label):
    if not store:
        print(label, "no samples"); return
    base = store[0][0]
    print("\n%s  (%d samples over 60 s)" % (label, len(store)))
    print("   window      n    median lag")
    for lo in range(0, 60, 10):
        w = [v for t, v in store if lo <= t - base < lo + 10]
        if w:
            print("   %2d-%2ds   %4d   %8.1f ms" % (lo, lo + 10, len(w), statistics.median(w)))

buckets(scan, "LASER")
buckets(rgb, "RGB")
if rgb:
    first = statistics.median([v for t, v in rgb[:20]])
    last = statistics.median([v for t, v in rgb[-20:]])
    print("\n   RGB first-20 median %.0f ms  ->  last-20 median %.0f ms   (drift %+.0f ms)"
          % (first, last, last - first))
    print("   VERDICT: %s" % ("GROWING = real buffering" if last - first > 500
                              else "STABLE = fixed offset, not accumulating"))
