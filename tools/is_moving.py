# Read-only: is the robot actually moving, and is any Nav2 goal still active?
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from action_msgs.msg import GoalStatusArray

rclpy.init()
n = Node('vla_is_moving')
sp = []
cv = []
st = []
n.create_subscription(Odometry, '/robot1/odom', lambda m: sp.append(
    (m.twist.twist.linear.x, m.twist.twist.angular.z)), qos_profile_sensor_data)
n.create_subscription(Twist, '/robot1/cmd_vel', lambda m: cv.append(
    (m.linear.x, m.angular.z)), 10)
n.create_subscription(GoalStatusArray, '/robot1/navigate_to_pose/_action/status',
                      lambda m: st.append([g.status for g in m.status_list]), 10)
t0 = time.time()
while time.time() - t0 < 8:
    rclpy.spin_once(n, timeout_sec=0.2)

if sp:
    mx = max(abs(a) for a, b in sp); mr = max(abs(b) for a, b in sp)
    print("odom samples: %d   max |linear| = %.4f m/s   max |angular| = %.4f rad/s"
          % (len(sp), mx, mr))
    print("VERDICT: %s" % ("STATIONARY" if mx < 0.01 and mr < 0.02 else "*** MOVING ***"))
else:
    print("no odom samples")
print("cmd_vel messages seen in 8 s: %d %s" % (len(cv), cv[-3:] if cv else ""))
if st:
    last = st[-1]
    ACTIVE = [1, 2]   # 1=ACCEPTED, 2=EXECUTING
    print("nav goal statuses: %s   active=%s"
          % (last, any(s in ACTIVE for s in last)))
else:
    print("no goal-status message seen")
