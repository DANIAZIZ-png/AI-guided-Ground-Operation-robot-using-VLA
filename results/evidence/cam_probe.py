# one participant: colour, depth, laser -- rate and stamp-vs-arrival latency over N s
import sys, time, statistics
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, LaserScan
DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 30
rclpy.init(); n = Node('vla_cam_probe')
store = {'rgb': [], 'depth': [], 'laser': []}
def mk(k):
    def cb(m):
        st = m.header.stamp.sec + m.header.stamp.nanosec*1e-9
        store[k].append((n.get_clock().now().nanoseconds*1e-9 - st)*1000.0)
    return cb
n.create_subscription(CompressedImage, '/robot1/oakd/rgb/preview/image_raw/compressed', mk('rgb'), qos_profile_sensor_data)
n.create_subscription(CompressedImage, '/robot1/oakd/stereo/image_raw/compressedDepth', mk('depth'), qos_profile_sensor_data)
n.create_subscription(LaserScan, '/robot1/scan', mk('laser'), qos_profile_sensor_data)
t0 = time.time()
while time.time()-t0 < DUR: rclpy.spin_once(n, timeout_sec=0.2)
print(time.strftime('%H:%M:%S'), 'window %.0fs' % DUR)
for k, v in store.items():
    if not v: print('  %-6s  NO SAMPLES' % k); continue
    print('  %-6s  n=%4d  rate %5.1f Hz  latency median %7.0f ms  p90 %7.0f  max %7.0f' % (k, len(v), len(v)/DUR, statistics.median(v), sorted(v)[int(0.9*len(v))-1], max(v)))
n.destroy_node(); rclpy.shutdown()
