# Read one LaserScan and report clearance by bearing, so we can pick a safe move.
import math, sys, time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

rclpy.init()
n = Node('vla_scan_look')
got = {}
def cb(m):
    if 'm' not in got: got['m'] = m
n.create_subscription(LaserScan, '/robot1/scan', cb, qos_profile_sensor_data)
t0 = time.time()
while 'm' not in got and time.time() - t0 < 40:
    rclpy.spin_once(n, timeout_sec=0.2)
if 'm' not in got:
    print("NO SCAN"); sys.exit(1)
m = got['m']
print("scan: %d beams, angle %.1f..%.1f deg, range %.2f..%.2f m" % (
    len(m.ranges), math.degrees(m.angle_min), math.degrees(m.angle_max),
    m.range_min, m.range_max))

def stat(centre_deg, half=10.0):
    vals = []
    for i, r in enumerate(m.ranges):
        a = math.degrees(m.angle_min + i * m.angle_increment)
        a = (a + 180) % 360 - 180
        d = abs(((a - centre_deg) + 180) % 360 - 180)
        if d <= half and m.range_min < r < m.range_max and not math.isinf(r) and not math.isnan(r):
            vals.append(r)
    if not vals: return None
    vals.sort()
    return (min(vals), vals[len(vals)//2], len(vals))

print("\n bearing        min    median  beams   (0=forward, +90=left, 180=behind)")
for b in (0, 30, 60, 90, 120, 150, 180, -150, -120, -90, -60, -30):
    s = stat(b)
    if s is None:
        print("  %+5d deg     no returns" % b)
    else:
        print("  %+5d deg  %6.2f  %6.2f    %3d" % (b, s[0], s[1], s[2]))
