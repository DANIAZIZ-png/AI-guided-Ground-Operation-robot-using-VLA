# say.py "command text" [listen_seconds]
# Publish ONE command on /vla/command (the same string the GUI text box or
# voice_command.py would send) from a long-lived publisher, then print every
# /vla/reply and /vla/status change for listen_seconds. Shuts rclpy down
# cleanly so it leaves no ghost participant behind (11 Sep).
import sys, time, json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
text = sys.argv[1]; listen = float(sys.argv[2]) if len(sys.argv) > 2 else 20.0
rclpy.init(); n = Node('vla_say'); last = {}
pub = n.create_publisher(String, '/vla/command', 10)
def on_reply(m): print(time.strftime('%H:%M:%S'), 'REPLY :', m.data.strip()[:300], flush=True)
def on_status(m):
    try: d = json.loads(m.data)
    except Exception: return
    keys = ('mode', 'target', 'state', 'target_dist', 'busy', 'camera')
    cur = {k: d.get(k) for k in keys}
    if cur != last.get('s'):
        last['s'] = cur; print(time.strftime('%H:%M:%S'), 'STATUS:', cur, flush=True)
n.create_subscription(String, '/vla/reply', on_reply, 10)
n.create_subscription(String, '/vla/status', on_status, 10)
t0 = time.time()
while pub.get_subscription_count() == 0 and time.time() - t0 < 40: rclpy.spin_once(n, timeout_sec=0.2)
if pub.get_subscription_count() == 0: print('no subscriber on /vla/command (agent not up?)'); sys.exit(1)
tw = time.time()
while time.time() - tw < 2.0: rclpy.spin_once(n, timeout_sec=0.2)
pub.publish(String(data=text)); print(time.strftime('%H:%M:%S'), 'SENT  :', repr(text), flush=True)
t1 = time.time()
while time.time() - t1 < listen: rclpy.spin_once(n, timeout_sec=0.2)
n.destroy_node(); rclpy.shutdown()
