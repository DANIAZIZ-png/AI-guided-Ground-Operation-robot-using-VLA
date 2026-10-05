"""Minimal fake ROS 2 stack so vla_agent_v9.py can be imported and its PURE
LOGIC exercised on a machine with no ROS installed.

This stubs only what the module touches at import time plus the few classes the
tests construct. Nothing here is used on the robot — it exists so that the
navigation maths, the staleness guard and the command router can be replayed
against the EXACT code that will run, rather than against a paraphrase of it.
"""
import sys
import types


def _mod(name):
    m = types.ModuleType(name)
    sys.modules[name] = m
    return m


class _Msg:
    def __init__(self, **kw):
        for k, v in kw.items():
            setattr(self, k, v)


class Header:
    def __init__(self):
        self.stamp = _Msg(sec=0, nanosec=0)
        self.frame_id = "camera"


class Image:
    def __init__(self):
        self.header = Header()


class CompressedImage:
    def __init__(self):
        self.header = Header()
        self.format = ""
        self.data = b""


class CameraInfo:
    def __init__(self):
        self.header = Header()
        self.k = [1.0] * 9


class LaserScan:
    def __init__(self):
        self.ranges = []


class String:
    def __init__(self):
        self.data = ""


class Twist:
    def __init__(self):
        self.linear = _Msg(x=0.0, y=0.0, z=0.0)
        self.angular = _Msg(x=0.0, y=0.0, z=0.0)


class PointStamped:
    def __init__(self):
        self.header = Header()
        self.point = _Msg(x=0.0, y=0.0, z=0.0)


class MapMetaData:
    def __init__(self):
        self.resolution = 0.05
        self.width = 0
        self.height = 0
        self.origin = _Msg(position=_Msg(x=0.0, y=0.0, z=0.0),
                           orientation=_Msg(x=0.0, y=0.0, z=0.0, w=1.0))


class OccupancyGrid:
    def __init__(self):
        self.info = MapMetaData()
        self.data = []


class Odometry:
    def __init__(self):
        self.pose = _Msg(pose=_Msg(position=_Msg(x=0.0, y=0.0, z=0.0),
                                   orientation=_Msg(x=0.0, y=0.0, z=0.0, w=1.0)))


class _Pub:
    def __init__(self):
        self.sent = []

    def publish(self, m):
        self.sent.append(m)


class _Timer:
    def __init__(self, period, cb):
        self.period = period
        self.cb = cb


class Node:
    def __init__(self, name):
        self._name = name
        self.timers = []

    def create_publisher(self, *a, **k):
        return _Pub()

    def create_subscription(self, *a, **k):
        return object()

    def create_timer(self, period, cb, callback_group=None):
        t = _Timer(period, cb)
        self.timers.append(t)
        return t

    def get_clock(self):
        return _Msg(now=lambda: _Msg(to_msg=lambda: _Msg(sec=0, nanosec=0)))

    def destroy_node(self):
        pass


def install():
    # rclpy
    rclpy = _mod("rclpy")
    rclpy.init = lambda *a, **k: None
    rclpy.shutdown = lambda *a, **k: None
    rclpy.ok = lambda: True
    rclpy.spin = lambda *a, **k: None

    class _Time:
        def __init__(self, *a, **k):
            pass
    rclpy.time = types.SimpleNamespace(Time=_Time)
    sys.modules["rclpy.time"] = rclpy.time

    m = _mod("rclpy.node");           m.Node = Node
    m = _mod("rclpy.action");         m.ActionClient = lambda *a, **k: _Msg(
        server_is_ready=lambda: False, send_goal_async=lambda g: None)
    m = _mod("rclpy.duration");       m.Duration = lambda *a, **k: None
    m = _mod("rclpy.qos")
    m.qos_profile_sensor_data = object()
    m.QoSProfile = lambda **k: object()
    m.QoSHistoryPolicy = _Msg(KEEP_LAST=1)
    m.QoSReliabilityPolicy = _Msg(RELIABLE=1, BEST_EFFORT=2)
    m = _mod("rclpy.callback_groups"); m.MutuallyExclusiveCallbackGroup = lambda: object()
    m = _mod("rclpy.executors");       m.MultiThreadedExecutor = lambda **k: _Msg(
        add_node=lambda n: None, spin=lambda: None, shutdown=lambda: None)

    m = _mod("sensor_msgs");           m = _mod("sensor_msgs.msg")
    m.Image, m.CameraInfo, m.LaserScan, m.CompressedImage = (
        Image, CameraInfo, LaserScan, CompressedImage)
    m = _mod("std_msgs");              m = _mod("std_msgs.msg"); m.String = String
    m = _mod("geometry_msgs");         m = _mod("geometry_msgs.msg")
    m.PointStamped, m.Twist = PointStamped, Twist
    m = _mod("nav_msgs");              m = _mod("nav_msgs.msg")
    m.Odometry, m.OccupancyGrid = Odometry, OccupancyGrid
    m = _mod("nav2_msgs");             m = _mod("nav2_msgs.action")

    class _NavGoal:
        def __init__(self):
            self.pose = _Msg(header=Header(),
                             pose=_Msg(position=_Msg(x=0.0, y=0.0, z=0.0),
                                       orientation=_Msg(x=0.0, y=0.0, z=0.0, w=1.0)))
    m.NavigateToPose = _Msg(Goal=_NavGoal)

    m = _mod("action_msgs");           m = _mod("action_msgs.msg")
    m.GoalStatus = _Msg(STATUS_SUCCEEDED=4, STATUS_ABORTED=6)
    m = _mod("cv_bridge")

    class _Bridge:
        def imgmsg_to_cv2(self, msg, desired_encoding=None):
            raise RuntimeError("no image in the stub")

        def cv2_to_imgmsg(self, img, encoding=None):
            return Image()
    m.CvBridge = _Bridge

    m = _mod("tf2_ros")
    m.Buffer = lambda: _Msg(lookup_transform=lambda *a, **k: (_ for _ in ()).throw(
        RuntimeError("no tf")))
    m.TransformListener = lambda *a, **k: None
    m = _mod("tf2_geometry_msgs");     m.do_transform_point = lambda *a: None

    m = _mod("message_filters")
    m.Subscriber = lambda *a, **k: object()

    class _Sync:
        def __init__(self, *a, **k):
            pass

        def registerCallback(self, cb):
            self.cb = cb
    m.ApproximateTimeSynchronizer = _Sync

    # llm_brain is imported by name; stub decide() so no Ollama is needed
    m = _mod("llm_brain")
    m.decide = lambda cmd, visible, context=None: {"action": "navigate",
                                                   "target": "chair",
                                                   "speech": "stub"}
