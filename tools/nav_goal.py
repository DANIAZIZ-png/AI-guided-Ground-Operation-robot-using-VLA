# nav_goal.py X Y YAW_DEG -- send one Nav2 goal from a LONG-LIVED client.
# Same settle pattern as scripts/redock.py (fresh clients get their replies dropped).
# THE ROBOT MOVES.
import sys, math, time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose

def log(m): print(time.strftime("%H:%M:%S"), m, flush=True)

if len(sys.argv) < 4:
    print("usage: nav_goal.py X Y YAW_DEG [settle_s]"); sys.exit(2)
X, Y, YAW = float(sys.argv[1]), float(sys.argv[2]), math.radians(float(sys.argv[3]))
SETTLE = float(sys.argv[4]) if len(sys.argv) > 4 else 30.0

rclpy.init()
n = Node('vla_nav_goal')
nav = ActionClient(n, NavigateToPose, '/robot1/navigate_to_pose')
t0 = time.time()
while not nav.server_is_ready():
    rclpy.spin_once(n, timeout_sec=0.5)
    if time.time() - t0 > 120: log("nav server not ready in 120 s"); sys.exit(1)
log("nav server seen after %.1f s; settling %.0f s" % (time.time() - t0, SETTLE))
tw = time.time()
while time.time() - tw < SETTLE: rclpy.spin_once(n, timeout_sec=0.2)

g = NavigateToPose.Goal()
g.pose.header.frame_id = 'map'
g.pose.pose.position.x = X
g.pose.pose.position.y = Y
g.pose.pose.orientation.z = math.sin(YAW / 2.0)
g.pose.pose.orientation.w = math.cos(YAW / 2.0)
log("sending goal (%.2f, %.2f) heading %.0f deg -- ROBOT WILL MOVE" %
    (X, Y, math.degrees(YAW)))

last = [0.0]
def fb(msg):
    d = msg.feedback.distance_remaining
    if abs(d - last[0]) > 0.25:
        log("  distance remaining %.2f m" % d); last[0] = d

fut = nav.send_goal_async(g, feedback_callback=fb)
rclpy.spin_until_future_complete(n, fut, timeout_sec=60)
gh = fut.result()
if gh is None: log("no goal response in 60 s"); sys.exit(1)
if not gh.accepted: log("goal REJECTED"); sys.exit(1)
log("goal accepted")
rf = gh.get_result_async()
rclpy.spin_until_future_complete(n, rf, timeout_sec=180)
r = rf.result()
if r is None: log("no result in 180 s"); sys.exit(1)
log("result status=%d (4=SUCCEEDED, 5=CANCELED, 6=ABORTED)" % r.status)
sys.exit(0 if r.status == 4 else 1)
