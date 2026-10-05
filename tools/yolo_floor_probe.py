# Grab one live frame, ask YOLO twice: default floors, then very low floors,
# so we can see what score each class ACTUALLY gets from this viewpoint.
import base64, json, sys, time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
import requests

def log(m): print(time.strftime("%H:%M:%S"), m, flush=True)

TOPIC = '/robot1/oakd/rgb/preview/image_raw/compressed'
OUT = '/tmp/claude-1001/-home-danyalaziz/acf25f36-94d4-4fc8-a156-f7db24ccce37/scratchpad/probe_frame.jpg'

rclpy.init()
n = Node('vla_yolo_floor_probe')
got = {}
def cb(m):
    if 'data' not in got: got['data'] = bytes(m.data)
n.create_subscription(CompressedImage, TOPIC, cb, qos_profile_sensor_data)
t0 = time.time()
while 'data' not in got and time.time() - t0 < 60:
    rclpy.spin_once(n, timeout_sec=0.2)
if 'data' not in got:
    log("NO FRAME"); sys.exit(1)
open(OUT, 'wb').write(got['data'])
b64 = base64.b64encode(got['data']).decode('ascii')
log("frame: %d bytes, saved to %s" % (len(got['data']), OUT))

LOW = {c: 0.03 for c in ["person","box","cardboard box","shelf","door",
                          "chair","pillar","docking station","charging dock"]}

for label, body in (("DEFAULT floors", {'image': b64}),
                    ("LOW floors (0.03)", {'image': b64, 'conf': 0.01,
                                           'min_area': 0, 'per_class_conf': LOW})):
    r = requests.post('http://127.0.0.1:5001/detect', json=body, timeout=60)
    d = r.json()
    dets = d.get('detections', [])
    log("%s -> HTTP %d, %d detections" % (label, r.status_code, len(dets)))
    for x in sorted(dets, key=lambda z: -z.get('confidence', 0)):
        print("    %-16s conf=%.3f area=%7.0f box=%s" % (
            x.get('name'), x.get('confidence', 0), x.get('area', 0),
            [round(v) for v in x.get('box', [])]))
    if 'error' in d: print("    error:", d['error'])
