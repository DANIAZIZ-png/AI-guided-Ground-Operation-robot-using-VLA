# redock.py -- dock the real robot from a LONG-LIVED action client (8 Sep 2026).
# Run inside ubuntu22-gpu after: source config/robot.env
#   python3 scripts/redock.py
# Drives with Nav2 to a staging pose 0.6 m in front of the dock, then runs the
# Dock action. Waits 30 s after creating its clients before the first call,
# because a freshly started client gets its replies dropped for up to ~20 s on
# this system (CHANGELOG_2026-09-08.md 9.4). Requires SLAM + Nav2 running and
# the dock at the map position it had on 8 Sep (robot docked at ~(0.04,-0.05)
# facing +x); edit the staging pose if the dock moves. THE ROBOT MOVES.
import sys, time, math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from irobot_create_msgs.action import Dock

def log(m): print(time.strftime("%H:%M:%S"), m, flush=True)

rclpy.init()
n = Node('vla_redock')
nav = ActionClient(n, NavigateToPose, '/robot1/navigate_to_pose')
dock = ActionClient(n, Dock, '/robot1/dock')
t0 = time.time()
while not (nav.server_is_ready() and dock.server_is_ready()):
    rclpy.spin_once(n, timeout_sec=0.5)
    if time.time() - t0 > 120: log("servers not ready after 120 s"); sys.exit(1)
log("both action servers seen after %.1f s; settling 30 s" % (time.time() - t0))
tw = time.time()
while time.time() - tw < 30: rclpy.spin_once(n, timeout_sec=0.2)

def run(client, goal, name, limit, fb=None):
    fut = client.send_goal_async(goal, feedback_callback=fb)
    rclpy.spin_until_future_complete(n, fut, timeout_sec=60)
    gh = fut.result()
    if gh is None: log(name + ": no goal response in 60 s"); sys.exit(1)
    if not gh.accepted: log(name + ": goal REJECTED"); sys.exit(1)
    log(name + ": goal accepted")
    rf = gh.get_result_async()
    rclpy.spin_until_future_complete(n, rf, timeout_sec=limit)
    r = rf.result()
    if r is None: log(name + ": no result in %d s" % limit); sys.exit(1)
    return r

# 1. staging pose: 0.6 m in front of the dock, facing the dock (+x)
g = NavigateToPose.Goal()
g.pose.header.frame_id = 'map'
g.pose.pose.position.x = -0.56; g.pose.pose.position.y = -0.05
g.pose.pose.orientation.z = 0.0; g.pose.pose.orientation.w = 1.0
log("sending Nav2 goal to staging pose (-0.56, -0.05), heading 0 deg")
r = run(nav, g, "nav2", 180)
log("nav2 result status=%d (4=SUCCEEDED)" % r.status)
if r.status != 4: sys.exit(1)

# 2. dock
last = [None]
def fb(msg):
    s = msg.feedback.sees_dock
    if s != last[0]: log("dock feedback: sees_dock=%s" % s); last[0] = s
log("sending Dock action")
r = run(dock, Dock.Goal(), "dock", 180, fb)
log("dock result status=%d is_docked=%s" % (r.status, r.result.is_docked))
