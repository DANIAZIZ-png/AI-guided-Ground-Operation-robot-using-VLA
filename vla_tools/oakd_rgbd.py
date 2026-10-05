# Try to switch the OAK-D pipeline to RGBD at RUNTIME, then cycle the camera.
# Non-destructive: if the parameter is read-only we get a clean refusal.
import sys, time
import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from std_srvs.srv import Trigger

def log(m): print(time.strftime("%H:%M:%S"), m, flush=True)

rclpy.init()
n = Node('vla_oakd_rgbd')
sp = n.create_client(SetParameters, '/robot1/oakd/set_parameters')
stop = n.create_client(Trigger, '/robot1/oakd/stop_camera')
start = n.create_client(Trigger, '/robot1/oakd/start_camera')
t0 = time.time()
while not (sp.service_is_ready() and stop.service_is_ready() and start.service_is_ready()):
    rclpy.spin_once(n, timeout_sec=0.5)
    if time.time() - t0 > 60:
        log("services not ready")
        sys.exit(1)
log("services seen after %.1f s; settling 30 s" % (time.time() - t0))
tw = time.time()
while time.time() - tw < 30:
    rclpy.spin_once(n, timeout_sec=0.2)

def call(cli, req, name, limit=30):
    f = cli.call_async(req)
    rclpy.spin_until_future_complete(n, f, timeout_sec=limit)
    r = f.result()
    if r is None:
        log(name + ": NO REPLY")
        return None
    return r

p = Parameter()
p.name = 'camera.i_pipeline_type'
p.value = ParameterValue(type=ParameterType.PARAMETER_STRING, string_value='RGBD')
req = SetParameters.Request()
req.parameters = [p]
r = call(sp, req, "set_parameters")
if r is None:
    sys.exit(1)
res = r.results[0]
log("set i_pipeline_type=RGBD -> successful=%s reason='%s'" % (res.successful, res.reason))
if not res.successful:
    log("parameter refused; runtime switch not possible")
    sys.exit(2)

log("cycling camera: stop_camera")
r = call(stop, Trigger.Request(), "stop_camera")
if r:
    log("  stop_camera success=%s msg='%s'" % (r.success, r.message))
tw = time.time()
while time.time() - tw < 5:
    rclpy.spin_once(n, timeout_sec=0.2)
log("start_camera")
r = call(start, Trigger.Request(), "start_camera", 60)
if r:
    log("  start_camera success=%s msg='%s'" % (r.success, r.message))
log("done -- now check whether stereo topics exist")
