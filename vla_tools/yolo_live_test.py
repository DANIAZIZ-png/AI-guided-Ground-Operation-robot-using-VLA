# Grab ONE live compressed frame from the OAK-D and POST it to yolo_server.
# The compressed topic is already JPEG, so it goes straight to base64.
# Sensor QoS is BEST_EFFORT (agent fix #39) -- a RELIABLE subscriber gets nothing.
import base64, json, sys, time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
import requests

def log(m): print(time.strftime("%H:%M:%S"), m, flush=True)

TOPIC = '/robot1/oakd/rgb/preview/image_raw/compressed'
OUT = '/home/danyalaziz/vla_logs/yolo_live_frame.jpg'

rclpy.init()
n = Node('vla_yolo_live_test')
got = {}

def cb(m):
    if 'data' not in got:
        got['data'] = bytes(m.data)
        got['fmt'] = m.format

n.create_subscription(CompressedImage, TOPIC, cb, qos_profile_sensor_data)
t0 = time.time()
while 'data' not in got and time.time() - t0 < 60:
    rclpy.spin_once(n, timeout_sec=0.2)
if 'data' not in got:
    log("NO FRAME received on " + TOPIC)
    sys.exit(1)
log("got a live frame after %.1f s: %d bytes, format='%s'" % (time.time()-t0, len(got['data']), got['fmt']))
open(OUT, 'wb').write(got['data'])
log("frame saved to " + OUT)

b64 = base64.b64encode(got['data']).decode('ascii')
t1 = time.time()
try:
    r = requests.post('http://127.0.0.1:5001/detect', json={'image': b64}, timeout=30)
except Exception as e:
    log("POST failed: %s" % e)
    sys.exit(1)
log("YOLO replied HTTP %d in %.0f ms" % (r.status_code, (time.time()-t1)*1000))
try:
    d = r.json()
except Exception:
    log("non-JSON reply: " + r.text[:300]); sys.exit(1)
dets = d.get('detections', [])
log("detections: %d" % len(dets))
for x in dets:
    print("   ", json.dumps(x))
if not dets:
    log("(empty list = server worked, nothing above the confidence floor in view)")
