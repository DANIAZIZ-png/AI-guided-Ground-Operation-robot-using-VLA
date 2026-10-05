"""VLAAgent -- the ROS node itself.

After the Phase 2 split this holds only __init__, which builds every
subscription, publisher, timer and action client. All 111 other methods live in
the five mixins and arrive through the MRO, in this order:

    VLAAgent -> StateMachineMixin -> NavigationMixin -> PerceptionMixin
             -> RosIoMixin -> GuardsMixin -> rclpy Node

Nothing was rewritten during the split, only moved, so every comment below is
the original and still carries the reasoning for its line.
"""
from .core import *                 # noqa: F401,F403 - constants and ROS imports
from .core import print             # noqa: A001 - headless-safe console, fix #63
from .core import _real_print, _off_front, _DecodedFilter   # noqa: F401
from .guards import GuardsMixin
from .navigation import NavigationMixin
from .perception_client import PerceptionMixin
from .ros_io import RosIoMixin
from .state_machine import StateMachineMixin


class VLAAgent(StateMachineMixin, NavigationMixin, PerceptionMixin, RosIoMixin, GuardsMixin, Node):
    def __init__(self):
        super().__init__("vla_agent")
        self.bridge = CvBridge()
        # #32b: the live feed runs on its own thread now — give it its own
        # bridge rather than sharing one object across threads.
        self.feed_bridge = CvBridge()
        self.K = self.cam_frame = None
        self.undock_fail_count = 0
        # #44: consecutive failed undock attempts, reset on any success
        self.stale_depth_drops = 0
        self.scan_msg = None
        # #54: most recent full LaserScan, used to range detected objects
        self.lidar_fixes = 0
        self._lidar_reasons = set()
        # #55: distinct reasons already reported, so each is said once
        # #54: how many object positions came from the LiDAR rather than depth
        self._size_reported = False
        # #52: guards the one-time depth-size report
        # #48: how many detections were dropped for having too-old depth
        self._server_seen = {}
        # #43: remembers which action servers have already been discovered
        self.K_depth = list(DEPTH_K_DEFAULT)
        # #53: seeded with this camera's known depth intrinsics so projection
        # is correct immediately; on_depth_info replaces them when the real
        # camera_info finally arrives
        self.depth_wh = DEPTH_WH_DEFAULT
        # #53: the geometry those intrinsics were calibrated for
        self.depth_info_live = False
        # #53: True once a real camera_info has been received

        # #12: the primary camera state is a timestamp-matched (rgb, depth)
        # PAIR. The raw single-topic copies below are only a fallback.
        self.pair = None                 # (rgb_msg, depth_msg), same instant
        self.pair_t = 0.0                # #32a: when that pair ARRIVED (monotonic)
        self.rgb_raw = None
        self.rgb_raw_t = 0.0             # #32e: last raw RGB arrival (camera watchdog)
        self.depth_raw = None
        self.sync_seen = False           # ever received a matched pair?
        self.warned_no_sync = False
        self.warned_stale_pair = False   # #32a: only nag about drift once
        self.warned_cam_stall = False    # #32e
        self.start_t = time.monotonic()

        self.have_odom = False
        self.x = self.y = self.yaw = 0.0
        self.front_min = float("inf")

        self.mode = None
        self.target = None
        self.target_qualifier = None     # #3: nearest / farthest / leftmost / rightmost
        self.task_id = 0
        self.goal_handle = None
        self.cmd = Twist()
        self.queue = []                  # remaining steps in a multi-step command

        self.search_state = None
        self.turn_accum = 0.0
        self.last_yaw = None
        # #56: step-and-stare bookkeeping. scan_dwell_until is the monotonic
        # time the current stationary "look" ends (None = not dwelling now).
        # scan_step_yaw is how far this individual step has turned so far.
        self.scan_dwell_until = None
        self.scan_step_yaw = 0.0
        # #59: after the guard fires, mute it briefly so it does not re-trigger
        # every 100 ms while the robot is still standing next to the obstacle.
        self.guard_mute_until = 0.0
        # #60: True while the operator is driving by hand.
        self.manual_mode = False
        # Set by enqueue_command() on the ROS/worker side; ACTED ON by the
        # input loop. Teleop must run on the thread that owns the terminal,
        # so the request and the action are deliberately separated.
        self.manual_requested = False
        self.advance_start = None
        self.navigate_start = None       # #6: when the current "go to X" started

        # navigation arrival / tracking state
        self.last_obj_xy = None          # last seen map position of the current target
        self.target_announced = False    # have we already said "found it"?
        self.nav_goal_xy = None          # the goal we last sent for this target
        self.sight_count = 0             # #13: consecutive-ish sightings before committing
        self.seen_live = False           # #17: seen with the CAMERA during THIS task (not just memory)
        self.goal_fail_count = 0         # #18: consecutive unreachable/rejected goals
        self.best_dist = None            # #19a: closest we've ever been to the target
        self.last_progress_t = None      # #19a: when best_dist last improved
        self.goal_tried = []             # #19c: approach goals Nav2 refused this task
        self.jump_pending = None         # #21b: suspicious sighting awaiting confirmation
        self.jump_pending_t = 0.0        # #30b: when it was raised (for expiry)
        self.goal_obj_xy = None          # #30a: object position when goal was picked

        # #31: incremental ("hop") approach into unmapped space
        self.step_active = False         # is the CURRENT nav goal a hop?
        self.step_bearing = None         # robot->object bearing the hop was aimed on
        self.step_start_t = 0.0          # when this hop was issued
        self.step_len = STEP_GOAL_DIST   # current hop length (shrinks on refusal)
        self.step_count = 0              # hops used in this task (for the log/GUI)
        self.step_announced = False      # said "beyond the mapped area" once

        # spatial instance memory (for counting / "find another")
        self.seen_instances = {}         # class name -> list of distinct (x, y) world positions
        self.find_baseline = 0
        self.find_start = 0.0
        self.locate_start = None         # #7
        self.dock_xy = None              # #22: dock position in the map frame, once known
        self.last_located = None         # #23: (target, (x, y)) of the most recent locate hit

        # #11: what we last acted on, for pronoun resolution in the brain
        self.ctx_last_action = None
        self.ctx_last_target = None
        self.ctx_time = 0.0              # #27: when that context was set

        # #9 relative move
        self.move_spec = None
        self.move_ref_yaw = None
        self.move_turned = 0.0
        self.move_ref_pos = None
        self.move_moved = 0.0
        # #29: backward-ratchet state
        self.move_axis = None            # unit heading vector when the translate started
        self.backup_best = 0.0           # best net reverse distance achieved so far
        self.backup_stall_t = None       # when reverse progress last improved
        self.backup_nudges = 0           # how many forward shuffles we've used
        self.nudge_from = None           # position where the current shuffle began

        # pending yes/no + whether this command already has a feed step
        self.pending = None
        self.has_feed_step = False

        self.map = None
        self.frontier_navigating = False
        self.frontier_sent_goal = False
        self.cur_goal = (0.0, 0.0)
        self.blacklist = []
        self.visited_goals = []          # #4: areas we've already covered while patrolling
        self.explore_start_t = None
        self.warned_no_map = False
        self.warned_no_frontier = False

        # stuck detection
        self.last_pos = None
        self.last_move_t = None
        self.consecutive_stucks = 0

        # #10 dock state
        self.is_docked = False
        self.dock_known = False
        self.dock_goal_handle = None

        self.feed_proc = None
        self.feed_warned_dead = False    # #32e: viewer crashed -> say it once

        # #35: stop-confirmation state
        self.odom_lin = 0.0              # latest reported forward speed (m/s)
        self.odom_ang = 0.0              # latest reported turn rate (rad/s)
        self.stop_until = 0.0            # hold /cmd_vel at zero until this time
        self.stop_started = None         # when the current stop began
        self.stop_still_ticks = 0        # consecutive readings that looked stopped
        self.stop_reported = True        # has the outcome been announced yet?
        self.shutdown = False

        # #33: one intake queue for EVERY command source (keyboard, /vla/command
        # from the GUI, /vla/command from the voice node). A worker thread
        # drains it, so an LLM call that takes three seconds blocks nothing but
        # itself — not the ROS executor, not the velocity publisher, not the feed.
        self.cmd_queue = queue.Queue()
        self.cmd_worker = None
        self.last_source = "keyboard"    # where the command being served came from

        # #14 mission log + #15 pooled HTTP session for YOLO
        self.mlog = MissionLog()
        self.http = requests.Session()
        # #32b: think() and the command worker can both reach for YOLO now
        # that they run on different threads. requests.Session is not
        # documented as thread-safe, and there is one GPU anyway — serialise.
        self.http_lock = threading.Lock()
        self.last_yolo_err_t = 0.0
        # #20: cache of the most recent YOLO result, keyed by the frame's
        # timestamp, so the annotated feed can REUSE the think-loop's
        # inference instead of paying for a second pass on the same image.
        self.last_det = None             # (stamp_key, detections)
        self.last_det_t = 0.0            # #32c: when last_det was produced

        # ── #39 SENSOR QoS ──────────────────────────────────────────
        # The real OAK-D, the RPLIDAR and the Create 3 all publish
        # BEST_EFFORT. A default rclpy subscription is RELIABLE, and a
        # RELIABLE subscriber does NOT match a BEST_EFFORT publisher — it
        # receives absolutely nothing, with no warning and no error. In
        # simulation the Gazebo plugins published RELIABLE, so v12 worked;
        # on hardware every camera callback would simply never fire.
        sensor_qos = qos_profile_sensor_data
        # best-effort, keep-last depth 5 — the standard QoS for sensor streams

        # ── #50 COMPRESSED RGB TRANSPORT ────────────────────────────
        # The raw colour stream is 250x250x3 = 187 kB per frame, and the
        # OAK-D publishes it at ~13 Hz: about 2.2 MB/s. Raw depth is 1.84 MB
        # per frame, so for depth to reach even 2 Hz the link would have to
        # carry ~6 MB/s. A 2.4 GHz Wi-Fi link does not, so the two streams
        # compete and the small, frequent colour frames win. Measured result:
        # depth fell to one frame every twenty seconds, and because
        # robust_box_depth then read a twenty-second-old frame captured from
        # a different pose, a chair one metre in front was reported at 9.6 m
        # and the robot navigated to empty floor.
        #   The driver already publishes a JPEG version of the same stream at
        # roughly 1/18th the size. Subscribing to that instead frees the link
        # for depth. The frame is decoded here and handed on as an ordinary
        # Image message, so nothing downstream changes.
        if USE_COMPRESSED_RGB:
            rgb_sub = _DecodedFilter()
            # a manually-driven filter standing in for the raw subscription
            self.create_subscription(CompressedImage, RGB_COMPRESSED_TOPIC,
                                     self.on_rgb_compressed, sensor_qos)
            # the real DDS subscription, carrying ~10 kB frames instead of 187 kB
            self._rgb_filter = rgb_sub
            # kept on self so the decode callback can push frames into it
        else:
            rgb_sub = message_filters.Subscriber(self, Image, RGB_TOPIC,
                                                 qos_profile=sensor_qos)
            # original raw path, still available via VLA_RAW_RGB=1
            self._rgb_filter = None
        if USE_COMPRESSED_DEPTH:
            depth_sub = _DecodedFilter()
            # #51: another hand-driven filter, this time fed by the decoded
            # compressedDepth stream
            self.create_subscription(CompressedImage, DEPTH_COMPRESSED_TOPIC,
                                     self.on_depth_compressed, sensor_qos)
            # the real subscription, on the topic the camera actually feeds
            self._depth_filter = depth_sub
            # kept on self so the decode callback can push frames into it
        else:
            depth_sub = message_filters.Subscriber(self, Image, DEPTH_TOPIC,
                                                   qos_profile=sensor_qos)
            # original raw path, still available via VLA_RAW_DEPTH=1
            self._depth_filter = None
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [rgb_sub, depth_sub], queue_size=SYNC_QUEUE, slop=SYNC_SLOP_S)
        # #46: queue_size cut from 10 to 3. Holding ten 640x360 depth frames
        # served no purpose once the slop window is wider than one depth
        # period, and the buffer added latency to every pair.
        self.sync.registerCallback(self.on_rgb_depth)
        # call on_rgb_depth once a matched (colour, depth) pair is found

        # ── #46 SINGLE SUBSCRIPTION PER STREAM ──────────────────────
        # v14 subscribed to RGB and depth TWICE each: once through the
        # synchronizer above, and once more with create_subscription() to
        # feed the raw fallback. That is four image streams pulled across
        # the Wi-Fi link when two would do, and DDS does not deduplicate
        # them — each subscription gets its own copy of every frame.
        # Measured effect: depth ran at 5.8 Hz with the agent stopped but
        # collapsed to 0.28 Hz with it running, a 20x drop that was NOT
        # caused by the link's capacity.
        #   message_filters.Subscriber inherits registerCallback from
        # SimpleFilter, so the raw handlers can hang off the SAME
        # subscription the synchronizer already uses. Same behaviour,
        # half the traffic.
        rgb_sub.registerCallback(self.on_rgb)
        # raw colour frames for the fallback path and the camera watchdog
        depth_sub.registerCallback(self.on_depth)
        # raw depth frames for the fallback path

        self.create_subscription(CameraInfo, INFO_TOPIC, self.on_info, sensor_qos)
        # colour intrinsics K — needed to turn a pixel into a 3D direction
        self.create_subscription(CameraInfo, DEPTH_INFO_TOPIC,
                                 self.on_depth_info, sensor_qos)
        # #40: depth intrinsics K — needed to map a colour pixel onto the depth image
        self.create_subscription(LaserScan, SCAN_TOPIC, self.on_scan, sensor_qos)
        # laser scan for obstacle stopping
        self.create_subscription(Odometry, ODOM_TOPIC, self.on_odom, sensor_qos)
        # odometry for relative moves and stop confirmation
        self.create_subscription(OccupancyGrid, MAP_TOPIC, self.on_map,
                                 QoSProfile(history=QoSHistoryPolicy.KEEP_LAST,
                                            depth=1,
                                            reliability=QoSReliabilityPolicy.RELIABLE,
                                            durability=DurabilityPolicy.TRANSIENT_LOCAL))
        # the SLAM map is published RELIABLE + TRANSIENT_LOCAL (latched) so a late
        # subscriber still receives the last map; a volatile subscription would wait
        # for the next map update, which on a static scene may never come
        self.cmd_pub = self.create_publisher(Twist, CMD_TOPIC, 10)
        # velocity publisher — this is the topic the #35 stop barrage floods with zeros

        # ── #42 EXPLICIT NAMESPACED TF ──────────────────────────────
        # tf2_ros.TransformListener hard-subscribes to a bare "/tf", which
        # does not exist on the namespaced robot, so the buffer stayed empty
        # and every projection to the map frame failed. We feed the buffer
        # ourselves from the correct topics instead.
        self.create_subscription(TFMessage, TF_TOPIC, self.on_tf, 100)
        # dynamic transforms (odom->base_link etc.), high queue depth as they arrive fast
        self.create_subscription(TFMessage, TF_STATIC_TOPIC, self.on_tf_static,
                                 QoSProfile(history=QoSHistoryPolicy.KEEP_LAST,
                                            depth=100,
                                            reliability=QoSReliabilityPolicy.RELIABLE,
                                            durability=DurabilityPolicy.TRANSIENT_LOCAL))
        # static transforms are latched — TRANSIENT_LOCAL is required or we miss
        # the one-and-only publication that happened before we started

        # #20/#32d: annotated stream for the live window. The publishers exist
        # from startup so rqt_image_view can always find the topics; frames are
        # only produced while the feed is actually open.
        #   QoS: RELIABLE so ANY subscriber can connect (a best-effort
        #   subscriber matches a reliable publisher; the reverse does not
        #   match at all, which would have made the feed invisible to
        #   rqt_image_view). History KEEP_LAST depth 1 so a slow viewer causes
        #   FRAME DROPS instead of a growing backlog — a backlog of 1.7 MB
        #   images is exactly how the old feed wedged itself permanently.
        img_qos = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=1,
                             reliability=QoSReliabilityPolicy.RELIABLE)
        self.annot_pub = self.create_publisher(Image, ANNOT_TOPIC, img_qos)
        self.annot_jpg_pub = self.create_publisher(CompressedImage,
                                                   ANNOT_COMPRESSED_TOPIC, img_qos)

        # #33: command bridge. /vla/command is deliberately a plain String —
        # the agent cannot tell (and must not care) whether a command was
        # typed in the terminal, typed in the GUI or spoken into a microphone.
        self.reply_pub  = self.create_publisher(String, REPLY_TOPIC, 20)
        self.status_pub = self.create_publisher(String, STATUS_TOPIC, 5)
        self.create_subscription(String, CMD_IN_TOPIC, self.on_remote_command, 10)

        self.tf_buffer = tf2_ros.Buffer()
        # the transform buffer is still tf2's — only the way it is FED changes (#42)
        self.nav_client = ActionClient(self, NavigateToPose, NAV_ACTION)
        # #37: Nav2's action server lives at /robot1/navigate_to_pose on the real robot
        # #35: a direct client on the action server's cancel service. Cancelling
        # our stored goal handle only works if we HAVE one - and there is a
        # window between sending a goal and the acceptance callback arriving
        # where we do not. Cancel-all closes that window.
        self.nav_cancel_cli = self.create_client(CancelGoal, NAV_CANCEL_SRV)
        # #37: the cancel-all service belongs to the namespaced action server too;
        # a wrong name here means "stop" cannot cancel the Nav2 goal at all

        # #10 dock/undock clients + dock status (all optional)
        if HAVE_CREATE:
            self.dock_client = ActionClient(self, Dock, DOCK_ACTION)
            # #37: /robot1/dock on the real robot
            self.undock_client = ActionClient(self, Undock, UNDOCK_ACTION)
            # #37: /robot1/undock on the real robot
            # #16: the Create 3 publishes dock_status BEST-EFFORT. A default
            # (reliable) subscription silently gets NOTHING -> is_docked would
            # never update on the real robot. Sensor-data QoS matches it.
            self.create_subscription(DockStatus, DOCK_STATUS_TOPIC,
                                     self.on_dock_status, qos_profile_sensor_data)
            # #37 + #16: namespaced name AND best-effort QoS — both are required
            # or is_docked never updates and the agent refuses to undock
        else:
            self.dock_client = self.undock_client = None

        # #32b: HEAD-OF-LINE BLOCKING. rclpy puts every timer in the node's
        # default MutuallyExclusiveCallbackGroup, so with a single-threaded
        # spin only ONE callback runs at a time. think() can block for seconds
        # inside the YOLO HTTP call — and while it did, publish_cmd() (the
        # 10 Hz velocity heartbeat) and annotate_think() (the live feed) did
        # not run at all. That is why the feed "ran for a while and got stuck":
        # it was being starved by the perception loop. A private group per
        # timer plus a MultiThreadedExecutor in main() lets them run in
        # parallel. Shared state is only ever read/assigned as whole objects,
        # which is atomic under the GIL, so no locking is required.
        self.cbg_think  = MutuallyExclusiveCallbackGroup()
        self.cbg_pub    = MutuallyExclusiveCallbackGroup()
        self.cbg_annot  = MutuallyExclusiveCallbackGroup()
        self.cbg_status = MutuallyExclusiveCallbackGroup()
        self.create_timer(THINK_PERIOD, self.think,          callback_group=self.cbg_think)
        self.create_timer(PUB_PERIOD,   self.publish_cmd,    callback_group=self.cbg_pub)
        # #59: the collision guard gets its own fast timer, in the same
        # callback group as publishing so it can never be starved by a
        # slow think-cycle (a YOLO call can take hundreds of ms).
        self.create_timer(GUARD_PERIOD, self.collision_guard, callback_group=self.cbg_pub)
        self.create_timer(STOP_BARRAGE_PERIOD, self.stop_barrage, callback_group=self.cbg_pub)
        # #35: runs at 50 Hz but returns instantly unless a stop is in progress,
        # so the idle cost is negligible
        self.create_timer(ANNOT_PERIOD, self.annotate_think, callback_group=self.cbg_annot)
        self.create_timer(STATUS_PERIOD, self.publish_status, callback_group=self.cbg_status)  # #33
