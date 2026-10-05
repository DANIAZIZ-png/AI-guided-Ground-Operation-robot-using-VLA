"""ROS plumbing: subscription callbacks, publishers, replies and status.

Methods moved verbatim out of VLAAgent in Phase 2 of the reorg. No logic
changed; the comments are the originals. Mixed into VLAAgent in
vla_agent/agent.py, ahead of rclpy's Node, so method resolution is unchanged.
"""
from .core import *            # noqa: F401,F403 - constants and ROS imports
from .core import print        # noqa: A001 - headless-safe console, fix #63
# `import *` deliberately skips underscore-prefixed names, so the ones the
# moved code needs are imported explicitly. Omitting them imports cleanly
# and then fails at runtime.
from .core import _off_front   # noqa: F401


class RosIoMixin:

    # ── #33: outbound speech + state ──
    def emit_reply(self, text):
        """Publish one line of agent output on /vla/reply. Called by the
        stdout mirror, so it must never print anything itself (that would
        recurse) and must never raise (that would break print())."""
        try:
            m = String(); m.data = str(text)[:1000]
            self.reply_pub.publish(m)
        except Exception:
            pass

    def publish_status(self):
        """A small JSON state packet for the GUI: enough to drive a status
        bar and the operator's confidence, cheap enough to send at 2 Hz."""
        try:
            pose = self.robot_pose_map()
            dist = None
            if pose is not None and self.last_obj_xy is not None:
                dist = round(math.hypot(pose[0] - self.last_obj_xy[0],
                                        pose[1] - self.last_obj_xy[1]), 2)
            st = {
                "version": AGENT_VERSION,
                "mode": self.mode or "idle",
                "target": self.target,
                "state": self.search_state,
                "docked": bool(self.is_docked) if self.dock_known else None,
                "map": self.map is not None,
                "camera": self.current_pair() is not None,
                "target_dist": dist,
                "hop_mode": bool(self.step_active),      # #31
                "hops": self.step_count,                 # #31
                "feed": bool(self.feed_proc is not None or self.feed_watchers() > 0),
                # #34: the status bar says "feed on" when the GUI is watching too
                "busy": bool(self.mode) or not self.cmd_queue.empty(),
                "queued": self.cmd_queue.qsize(),
                "source": self.last_source,
            }
            m = String(); m.data = json.dumps(st)
            self.status_pub.publish(m)
        except Exception:
            pass

    def on_remote_command(self, msg):
        """#33: a command from the GUI or the voice node. It goes into the
        SAME queue the keyboard uses, so there is exactly one command path in
        this program and no chance of two sources racing each other."""
        text = (msg.data or "").strip()
        if not text:
            return
        print(f"\n[operator/remote] {text}")
        self.enqueue_command(text, source="remote")

    # ── sensor callbacks ──
    def on_rgb_depth(self, rgb_msg, depth_msg):        # #12: matched pair
        self.pair = (rgb_msg, depth_msg)
        self.pair_t = time.monotonic()                 # #32a: age it, don't trust it forever
        self.sync_seen = True
    def on_rgb(self, m):
        now = time.monotonic()
        # #32e INSTRUMENTATION: the stall warning fires as soon as the gap passes
        # CAM_STALL_WARN, so it can only ever report "over 5s". The number that
        # actually diagnoses a dropout is the TOTAL outage, and it is only
        # knowable HERE, on the first frame back. Logged once per outage, gated
        # on warned_cam_stall so the 30 Hz normal path stays silent.
        if self.warned_cam_stall and self.rgb_raw_t > 0.0:
            gap = now - self.rgb_raw_t
            self.mlog.log("WARN", f"#32e camera stream RESUMED after {gap:.1f}s")
            self.notify(f"Camera frames are back — the gap was {gap:.1f}s.")
        self.rgb_raw = m
        self.rgb_raw_t = now                           # #32e: camera watchdog
        self.warned_cam_stall = False
    def on_depth(self, m): self.depth_raw = m
    def on_rgb_compressed(self, m):
        # #50: decode a JPEG colour frame and push it into the pipeline as an
        # ordinary Image message, so the synchronizer and every downstream
        # consumer are unaffected by the change of transport.
        try:
            buf = np.frombuffer(m.data, dtype=np.uint8)
            # the JPEG bytes as a flat numpy array, which is what OpenCV wants
            img = cv2.imdecode(buf, cv2.IMREAD_COLOR)
            # decode to a BGR image; returns None if the buffer is corrupt
            if img is None:
                return
                # a dropped or partial frame is skipped, never crashes the callback
            out = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
            # wrap it back into a sensor_msgs/Image
            out.header = m.header
            # KEEP THE ORIGINAL HEADER: #47 depends on this stamp being the
            # true capture time, not the time we happened to decode it
            self._rgb_filter.signalMessage(out)
            # hand it to the synchronizer and the raw fallback, exactly as a
            # real message_filters.Subscriber would have done
        except Exception:
            pass
            # never let a bad frame kill the subscription

    def report_stream_sizes(self, w, h):
        # #52: print the real depth frame size ONCE, next to what camera_info
        # claims, so a resolution mismatch is visible instead of silent.
        if self._size_reported:
            return
            # only ever printed once per run
        self._size_reported = True
        info = ("%dx%d" % tuple(self.depth_wh)) if self.depth_wh else "unknown"
        # the geometry the intrinsics in use were calibrated for
        src = "live camera_info" if self.depth_info_live else "seeded defaults (#53)"
        # says plainly whether real calibration arrived or the seed is in use
        self.mlog.log("INFO", "depth frames %dx%d, intrinsics %s from %s"
                      % (w, h, info, src))
        print("[depth] receiving %dx%d frames; intrinsics %s from %s%s"
              % (w, h, info, src,
                 "  -> rescaled (#52)"
                 if (self.depth_wh and (w, h) != tuple(self.depth_wh)) else ""))

    def on_depth_compressed(self, m):
        # #51: decode a compressedDepth frame into an ordinary 16UC1 Image.
        # The wire format is compressed_depth_image_transport's: a 12-byte
        # ConfigHeader (format enum + two float quantisation constants),
        # followed by a PNG-encoded image. For 16UC1 the two constants are
        # unused - the PNG already holds millimetres - so the header is
        # simply skipped.
        try:
            raw = bytes(m.data)
            # the full payload: config header plus PNG
            if len(raw) <= DEPTH_CFG_HEADER_BYTES:
                return
                # truncated frame, nothing decodable behind the header
            png = np.frombuffer(raw[DEPTH_CFG_HEADER_BYTES:], dtype=np.uint8)
            # everything after the 12-byte header is the PNG payload
            img = cv2.imdecode(png, cv2.IMREAD_UNCHANGED)
            # IMREAD_UNCHANGED preserves 16-bit depth; IMREAD_COLOR would
            # silently crush it to 8-bit and destroy every distance reading
            if img is None or img.dtype != np.uint16:
                return
                # only the 16-bit millimetre form is trusted; anything else is skipped
            out = self.bridge.cv2_to_imgmsg(img, encoding="16UC1")
            # declare the encoding explicitly so #41 converts millimetres correctly
            out.header = m.header
            # keep the true capture time, which #47 and #48 both depend on
            self.report_stream_sizes(img.shape[1], img.shape[0])
            # #52: surface the real frame size once so a mismatch is obvious
            self._depth_filter.signalMessage(out)
            # hand it to the synchronizer and the #46 raw fallback
        except Exception:
            pass
            # a corrupt frame is skipped, never crashes the callback

    def on_info(self, m):  self.K = m.k; self.cam_frame = m.header.frame_id
    # store the COLOUR camera's 3x3 intrinsics and its optical frame name

    def on_depth_info(self, m):
        # #40: store the DEPTH camera's intrinsics so colour pixels can be
        # mapped onto the depth image. Colour is 250x250, depth is 1280x720.
        self.K_depth = m.k
        # K_depth = [fx,0,cx, 0,fy,cy, 0,0,1] for the depth image geometry
        self.depth_wh = (m.width, m.height)
        # remember the depth image size for bounds checking
        if not self.depth_info_live:
            self.depth_info_live = True
            # #53: note the switch from seeded defaults to live calibration
            self.mlog.log("INFO", "live depth camera_info received %dx%d"
                          % (m.width, m.height))

    def on_tf(self, msg):
        # #42: feed dynamic transforms from the namespaced /robot1/tf into
        # the tf2 buffer by hand, because TransformListener listens on "/tf".
        for t in msg.transforms:
            # each message carries a list of transforms, not just one
            try:
                self.tf_buffer.set_transform(t, "vla_agent")
                # insert it into the buffer, tagged with who supplied it
            except Exception:
                pass
                # a malformed transform must never crash the callback

    def on_tf_static(self, msg):
        # #42: same for latched static transforms (camera mount, wheel offsets)
        for t in msg.transforms:
            # static transforms also arrive as a list
            try:
                self.tf_buffer.set_transform_static(t, "vla_agent")
                # set_transform_static marks them as valid for all time
            except Exception:
                pass
                # never let a bad static transform kill the subscription
    def on_map(self, m):   self.map = m
    def on_odom(self, m):
        self.x = m.pose.pose.position.x
        self.y = m.pose.pose.position.y
        self.yaw = yaw_from_quat(m.pose.pose.orientation)
        self.have_odom = True
        # #35: the robot's OWN reported speed. This is the only honest way to
        # know whether a stop worked - the agent's intent proves nothing when
        # another node is also publishing to /cmd_vel.
        self.odom_lin = abs(m.twist.twist.linear.x)
        # forward/backward speed in m/s
        self.odom_ang = abs(m.twist.twist.angular.z)
        # turning rate in rad/s
    def on_scan(self, m):
        if len(m.ranges) == 0: return
        self.scan_msg = m
        # #54: keep the WHOLE scan, not just the front minimum. Ranging a
        # detected object needs the beam at that object's bearing, which
        # means the full array.
        cone = math.radians(15); best = float("inf")
        for i, r in enumerate(m.ranges):
            ang = m.angle_min + i * m.angle_increment
            if _off_front(ang) <= cone and math.isfinite(r) and r > 0.0:
                best = min(best, r)          # #65: cone centred on the FRONT
        self.front_min = best
    def on_dock_status(self, m):
        self.is_docked = bool(m.is_docked)
        self.dock_known = True
        # #22: the first time we learn we're ON the dock, the robot's own map
        # pose IS the dock's position. (TF may not be up yet at boot — then
        # robot_xy() is None and we simply try again on the next status msg.)
        if self.is_docked and self.dock_xy is None:
            xy = self.robot_xy()
            if xy is not None:
                self.set_dock_position(*xy)

    def notify(self, msg):
        self.mlog.log("ROBOT", msg)                    # #14
        print(f"\n[robot] {msg}\nCommand> ", end="", flush=True)

    @staticmethod
    def stamp_to_sec(stamp):
        """ROS time message -> float seconds."""
        return stamp.sec + stamp.nanosec * 1e-9

    # ── #20: ANNOTATED LIVE FEED ──
    def _stamp_key(self, msg):
        s = msg.header.stamp
        return (s.sec, s.nanosec)

    # ── hard stop — cancel the Nav2 goal AND publish zero velocity ──
    def stop_base(self):
        if self.goal_handle is not None:
            try: self.goal_handle.cancel_goal_async()
            except Exception: pass
            self.goal_handle = None
        self.cmd = Twist()
        for _ in range(3):
            try: self.cmd_pub.publish(Twist())
            except Exception: pass

    def publish_cmd(self):
        if time.monotonic() < self.stop_until:
            return
            # #35: a stop is in progress. Staying silent here lets the barrage
            # own /cmd_vel outright, instead of two of our own timers taking
            # turns to publish different things.
        if self.mode == "move":
            self.cmd_pub.publish(self.cmd)
        elif self.mode == "locate":
            self.cmd_pub.publish(self.cmd)
        elif self.mode in ("navigate", "find_more") and self.search_state in ("ROTATE", "ADVANCE"):
            self.cmd_pub.publish(self.cmd)
