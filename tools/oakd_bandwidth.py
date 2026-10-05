# oakd_bandwidth.py [FPS] [JPEG_QUALITY] [PREVIEW_PX] -- make the colour stream light enough for
# the far end of the room (17 Sep 2026). Default 10 fps, JPEG quality 75 (40 blurred YOLO input: person 0.17): ~200 kB/s
# instead of ~720 kB/s at 30 fps / 80. The agent thinks at 1 Hz, so 10 fps is plenty.
# rgb.i_fps is an "initial" parameter: it takes effect after stop_camera/start_camera
# (~40 s). jpeg_quality is live. Runtime only -- a Pi reboot restores the stock values.
import sys, time
import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from std_srvs.srv import Trigger
FPS = int(sys.argv[1]) if len(sys.argv) > 1 else 30
Q = int(sys.argv[2]) if len(sys.argv) > 2 else 75
# 17 Sep: preview 250 -> 416 px square. YOLO-World gets 2.8x the pixels per object
# (a chair at 4 m was 25 px tall and scored 0.2; it needs ~60 px). ~20 kB/frame at
# JPEG 75 = ~200 kB/s at 10 fps, still a third of the original 30 fps stream.
# The agent must be restarted afterwards: it caches the camera intrinsics (K).
PX = int(sys.argv[3]) if len(sys.argv) > 3 else 512
def log(m): print(time.strftime("%H:%M:%S"), m, flush=True)
rclpy.init(); n = Node('vla_oakd_bandwidth')
sp = n.create_client(SetParameters, '/robot1/oakd/set_parameters')
stop = n.create_client(Trigger, '/robot1/oakd/stop_camera')
start = n.create_client(Trigger, '/robot1/oakd/start_camera')
t0 = time.time()
while not (sp.service_is_ready() and stop.service_is_ready() and start.service_is_ready()):
    rclpy.spin_once(n, timeout_sec=0.5)
    if time.time() - t0 > 150: log("services not ready"); sys.exit(1)
log("services seen after %.1f s; settling 30 s" % (time.time() - t0))
tw = time.time()
while time.time() - tw < 30: rclpy.spin_once(n, timeout_sec=0.2)
def call(cli, req, name, limit=30):
    f = cli.call_async(req); rclpy.spin_until_future_complete(n, f, timeout_sec=limit)
    r = f.result()
    if r is None: log(name + ": NO REPLY")
    return r
req = SetParameters.Request()
req.parameters = [
    Parameter(name='rgb.i_fps', value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(FPS))),
    Parameter(name='.oakd.rgb.preview.image_raw.jpeg_quality', value=ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=Q)),
    Parameter(name='rgb.i_preview_size', value=ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=PX)),
    Parameter(name='rgb.i_preview_width', value=ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=PX)),   # depthai-ros 2.11 reads these two
    Parameter(name='rgb.i_preview_height', value=ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=PX)),
]
r = call(sp, req, "set_parameters")
if r is None: sys.exit(1)
for p, res in zip(req.parameters, r.results):
    log("set %s -> successful=%s %s" % (p.name, res.successful, res.reason))
log("cycling camera so the fps takes effect: stop_camera")
r = call(stop, Trigger.Request(), "stop_camera")
tw = time.time()
while time.time() - tw < 5: rclpy.spin_once(n, timeout_sec=0.2)
r = call(start, Trigger.Request(), "start_camera", 60)
if r: log("start_camera success=%s" % r.success)
n.destroy_node(); rclpy.shutdown()
