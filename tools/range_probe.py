# Ground-truth check: what distance does the LASER give for each YOLO box?
# Mirrors the agent's own scan_range_at() (nearest coherent cluster, #61)
# so the number printed here is what range_for_box() would return.
import base64, math, sys, time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, LaserScan, CameraInfo
import requests

def log(m): print(time.strftime("%H:%M:%S"), m, flush=True)

# agent constants
GAP, CMIN = 0.30, 2
MIN_HALF, MAX_HALF = math.radians(1.0), math.radians(12.0)

rclpy.init()
n = Node('vla_range_probe')
G = {}
n.create_subscription(CompressedImage, '/robot1/oakd/rgb/preview/image_raw/compressed',
                      lambda m: G.setdefault('rgb', bytes(m.data)), qos_profile_sensor_data)
n.create_subscription(LaserScan, '/robot1/scan',
                      lambda m: G.__setitem__('scan', m), qos_profile_sensor_data)
n.create_subscription(CameraInfo, '/robot1/oakd/rgb/preview/camera_info',
                      lambda m: G.setdefault('K', list(m.k)), qos_profile_sensor_data)
t0 = time.time()
while not all(k in G for k in ('rgb', 'scan', 'K')) and time.time() - t0 < 60:
    rclpy.spin_once(n, timeout_sec=0.2)
for k in ('rgb', 'scan', 'K'):
    if k not in G: log("missing " + k); sys.exit(1)

K = G['K']; fx, cx = K[0], K[2]
scan = G['scan']
log("fx=%.1f cx=%.1f   scan %d beams" % (fx, cx, len(scan.ranges)))

def scan_range_at(bearing, half):
    """nearest coherent cluster within +/-half of bearing (agent #61)"""
    vals = []
    for i, r in enumerate(scan.ranges):
        if not (scan.range_min < r < scan.range_max) or math.isinf(r) or math.isnan(r):
            continue
        a = scan.angle_min + i * scan.angle_increment
        d = abs(((a - bearing) + math.pi) % (2 * math.pi) - math.pi)
        if d <= half:
            vals.append(r)
    if len(vals) < CMIN:
        return None, len(vals)
    vals.sort()
    cluster = [vals[0]]
    for r in vals[1:]:
        if r - cluster[-1] <= GAP:
            cluster.append(r)
        else:
            if len(cluster) >= CMIN:
                break
            cluster = [r]
    if len(cluster) < CMIN:
        return None, len(vals)
    return sum(cluster) / len(cluster), len(vals)

b64 = base64.b64encode(G['rgb']).decode('ascii')
LOW = {c: 0.03 for c in ["person","box","cardboard box","shelf","door",
                         "chair","pillar","docking station","charging dock"]}
r = requests.post('http://127.0.0.1:5001/detect',
                  json={'image': b64, 'conf': 0.01, 'min_area': 0,
                        'per_class_conf': LOW}, timeout=60)
dets = r.json().get('detections', [])
log("%d detections" % len(dets))
print("\n  class            conf   u_centre  bearing   half   LASER RANGE   beams")
print("  " + "-" * 68)
for d in sorted(dets, key=lambda z: -z['confidence']):
    x1, y1, x2, y2 = d['box']
    uc = (x1 + x2) / 2.0
    bearing = -math.atan((uc - cx) / fx)      # +ve = left of centre (REP-103)
    half = min(MAX_HALF, max(MIN_HALF, abs(x2 - x1) / (2.0 * fx)))
    rng, nb = scan_range_at(bearing, half)
    print("  %-14s %5.3f  %7.1f  %+6.1f deg  %4.1f deg   %s   %3d" % (
        d['name'], d['confidence'], uc, math.degrees(bearing),
        math.degrees(half),
        ("%6.2f m" % rng) if rng else "  none ", nb))

print("\n  forward laser profile (for context):")
for b in (-30, -20, -10, 0, 10, 20, 30):
    rng, nb = scan_range_at(math.radians(b), math.radians(5))
    print("    %+4d deg : %s" % (b, ("%.2f m" % rng) if rng else "none"))
