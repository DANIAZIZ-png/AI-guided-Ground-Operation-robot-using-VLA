# undock_hw.py -- undock the real robot from a LONG-LIVED action client.
# Same pattern as ~/redock.py: create the client, spin 30 s so the reply
# channel is matched, then call. A fresh CLI client gets its reply dropped.
# Run inside ubuntu22-gpu after: source ~/robot_env.sh.  THE ROBOT MOVES.
import sys, time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from irobot_create_msgs.action import Undock

def log(m): print(time.strftime("%H:%M:%S"), m, flush=True)

rclpy.init()
n = Node('vla_undock')
und = ActionClient(n, Undock, '/robot1/undock')
t0 = time.time()
while not und.server_is_ready():
    rclpy.spin_once(n, timeout_sec=0.5)
    if time.time() - t0 > 120: log("undock server not ready after 120 s"); sys.exit(1)
log("undock server seen after %.1f s; settling 30 s" % (time.time() - t0))
tw = time.time()
while time.time() - tw < 30: rclpy.spin_once(n, timeout_sec=0.2)

log("sending Undock action -- ROBOT WILL MOVE")
fut = und.send_goal_async(Undock.Goal())
rclpy.spin_until_future_complete(n, fut, timeout_sec=60)
gh = fut.result()
if gh is None: log("undock: no goal response in 60 s"); sys.exit(1)
if not gh.accepted: log("undock: goal REJECTED"); sys.exit(1)
log("undock: goal accepted")
rf = gh.get_result_async()
rclpy.spin_until_future_complete(n, rf, timeout_sec=120)
r = rf.result()
if r is None: log("undock: no result in 120 s"); sys.exit(1)
log("undock result status=%d (4=SUCCEEDED) is_docked=%s" % (r.status, r.result.is_docked))
