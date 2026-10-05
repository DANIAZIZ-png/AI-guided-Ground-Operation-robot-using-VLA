# Long-lived param client for /robot1/oakd (fresh CLI clients get replies dropped).
import sys, time
import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import ListParameters, GetParameters

def log(m): print(time.strftime("%H:%M:%S"), m, flush=True)

rclpy.init()
n = Node('vla_oakd_params')
lp = n.create_client(ListParameters, '/robot1/oakd/list_parameters')
gp = n.create_client(GetParameters, '/robot1/oakd/get_parameters')
t0 = time.time()
while not (lp.service_is_ready() and gp.service_is_ready()):
    rclpy.spin_once(n, timeout_sec=0.5)
    if time.time() - t0 > 60: log("param services not ready"); sys.exit(1)
log("param services seen after %.1f s; settling 30 s" % (time.time() - t0))
tw = time.time()
while time.time() - tw < 30: rclpy.spin_once(n, timeout_sec=0.2)

req = ListParameters.Request(); req.depth = 0
fut = lp.call_async(req)
rclpy.spin_until_future_complete(n, fut, timeout_sec=30)
r = fut.result()
if r is None: log("list_parameters: NO REPLY"); sys.exit(1)
names = sorted(r.result.names)
log("total params: %d" % len(names))
hits = [x for x in names if any(k in x.lower() for k in
        ('time','stamp','sync','clock','tf','calib','fps','q_size','low_band'))]
for h in hits: print("   ", h)

want = [x for x in names if 'pipeline_type' in x]
if want:
    g = GetParameters.Request(); g.names = want
    fut2 = gp.call_async(g)
    rclpy.spin_until_future_complete(n, fut2, timeout_sec=30)
    r2 = fut2.result()
    if r2:
        for nm, v in zip(want, r2.values):
            log("%s = %s" % (nm, v.string_value or v.integer_value))
