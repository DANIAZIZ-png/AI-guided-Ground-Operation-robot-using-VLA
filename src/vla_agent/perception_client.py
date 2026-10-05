"""Perception: camera pairing, the YOLO client, ranging, sightings and the annotated feed.

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


class PerceptionMixin:

    # ── #12: one place decides which frames the whole pipeline uses ──
    def current_pair(self):
        """Return a matched (rgb_msg, depth_msg) pair. Prefers the
        timestamp-synchronized pair; falls back to latest-of-each with a
        one-time warning if pairing has never worked (e.g. a driver that
        stamps the two streams from different clocks).

        #32a THE STALE-PAIR BUG. This used to be `if self.pair is not None:
        return self.pair` — with no notion of age. self.pair is only written
        when the ApproximateTimeSynchronizer MATCHES a colour frame to a depth
        frame within SYNC_SLOP_S. In simulation both streams tick off one
        clock so that always happens. On the real OAK-D they drift, and
        matching stops for seconds at a time. The result was a single frozen
        pair returned forever:
          * the annotated feed republished one still image (the 'feed gets
            stuck' symptom), and
          * far more seriously, YOLO, the depth projection and every
            navigation goal were computed from a photograph of where the
            robot USED to be.
        A pair older than PAIR_STALE_S is now dropped and the raw
        latest-of-each fallback takes over — slightly less accurate, but
        current, and it is announced so the drift is visible rather than
        silent."""
        now = time.monotonic()
        if self.pair is not None:
            if now - self.pair_t <= PAIR_STALE_S:
                return self.pair
            # stale: fall through to the raw streams, and say so once
            if not self.warned_stale_pair:
                self.warned_stale_pair = True
                age = now - self.pair_t
                self.notify(f"RGB and depth stopped lining up ({age:.1f}s since the "
                            f"last matched pair) — switching to the newest frame of "
                            f"each. Detections stay live; depth accuracy drops a "
                            f"little while they are out of step.")
                self.mlog.log("WARN", f"#32a stale sync pair dropped after {age:.1f}s")
            self.pair = None
        if self.rgb_raw is not None and self.depth_raw is not None:
            if (not self.warned_no_sync and not self.sync_seen
                    and time.monotonic() - self.start_t > SYNC_WARN_S):
                self.warned_no_sync = True
                self.notify("RGB and depth timestamps never match (slop "
                            f"{SYNC_SLOP_S}s) — using unsynchronized frames. "
                            "Check the camera driver's stamps.")
                self.mlog.log("WARN", "RGB-depth sync failed; raw fallback in use")
            return (self.rgb_raw, self.depth_raw)
        if RGB_ONLY and self.rgb_raw is not None:      # #64: colour-only gate
            return (self.rgb_raw, None)
        return None

    def camera_ready(self):
        return self.current_pair() is not None and self.K is not None

    # ── YOLO ──
    def yolo_detect(self, rgb_msg=None):
        """Run detection on rgb_msg (or the current pair's RGB). #15: pooled
        session + throttled error message + logging."""
        if rgb_msg is None:
            pair = self.current_pair()
            if pair is None: return None
            rgb_msg = pair[0]
        cv_rgb = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
        ok, buf = cv2.imencode(".jpg", cv_rgb)
        img_b64 = base64.b64encode(buf.tobytes()).decode("utf-8")
        try:
            # #32b: was timeout=15. Nothing useful can come back 15 seconds
            # into a 1 Hz control loop — by then the frame describes a world
            # the robot has already driven through. Worse, the wait used to
            # freeze the whole node. (connect, read) seconds: fail fast, log
            # it, and let the next think-cycle try again.
            with self.http_lock:                        # #32b
                resp = self.http.post(YOLO_URL, json={"image": img_b64},
                                      timeout=(2.0, 5.0))
            dets = resp.json()["detections"]
            # #20: remember WHICH frame these belong to so the annotated feed
            # can reuse them instead of running YOLO twice on one image.
            self.last_det = (self._stamp_key(rgb_msg), dets)
            self.last_det_t = time.monotonic()          # #32c: for the feed's TTL
            return dets
        except Exception as e:
            now = time.monotonic()
            self.mlog.log("WARN", f"YOLO request failed: {e}")
            if now - self.last_yolo_err_t > YOLO_ERR_PERIOD:
                self.last_yolo_err_t = now
                print(f"\n[robot] YOLO server error (is yolo_server.py running?): {e}")
            return None

    # ── project one pixel (u, v) to a map (x, y) using depth + TF ──
    #    #8/#12: the depth frame is the one PAIRED with the RGB YOLO saw, the
    #    TF lookup uses its CAPTURE time (with a 'latest' fallback), and depth
    #    is sampled as a median patch to shrug off noise/holes.
    # ── #20: median depth (METRES) around a pixel. Factored out of
    #    project_pixel so the annotated feed's distance labels and the
    #    navigation goals come from the SAME number — a label that disagreed
    #    with the goal would be worthless as evidence.
    # ── #41: convert raw depth values to METRES using the message's own
    #    declared encoding, instead of guessing from the magnitude. v12 used
    #    "if d > 100: d /= 1000", which silently mislabels anything closer
    #    than 10 cm as 50-plus metres. The real OAK-D publishes 16UC1
    #    (unsigned 16-bit millimetres); simulation published 32FC1 (metres).
    @staticmethod
    def depth_units_to_m(d, depth_msg):
        """Raw depth sample -> metres, decided by the image encoding."""
        enc = (depth_msg.encoding or "").lower()
        # read the encoding string the publisher itself declared
        if enc in ("16uc1", "mono16"):
            return d / 1000.0
            # 16-bit integer depth is millimetres by ROS convention
        if enc in ("32fc1",):
            return d
            # 32-bit float depth is already in metres
        return d / 1000.0 if d > 100 else d
        # unknown encoding: fall back to v12's magnitude heuristic rather than
        # returning a number we know is wrong

    # ── #40: map a COLOUR pixel onto the DEPTH image ────────────────
    #    The OAK-D's colour preview is 250x250 while its depth image is
    #    1280x720 — different size AND different aspect ratio, so there is
    #    no single scale factor. v12 indexed depth_img[v, u] with colour
    #    coordinates and then CLAMPED them into range, which meant every
    #    detection silently read the depth of a completely different part
    #    of the scene. Because depth is published in the RGB optical frame
    #    (aligned), both images share one optical centre, so the correct
    #    mapping is: un-project through the colour K, re-project through
    #    the depth K.
    def rgb_px_to_depth_px(self, u, v, depth_img_shape):
        """Colour pixel (u, v) -> depth pixel (u_d, v_d), or None if that
        point lies outside the depth camera's field of view."""
        h, w = depth_img_shape[:2]
        # actual depth image dimensions, read from the array itself
        # #52 RESOLUTION MISMATCH. K_depth comes from stereo/camera_info, which
        # describes the camera's NATIVE depth size (1280x720). The frames we
        # actually receive may be smaller - the compressedDepth stream is
        # produced by the low-bandwidth pipeline and can be downscaled. v19
        # mapped colour pixels into 1280x720 coordinates and then indexed an
        # image of a different size: some samples fell outside the array and
        # returned "distance unclear", while the rest landed on the wrong part
        # of the scene entirely. That is why the same few wrong distances
        # (9.6 m, 6.4 m, 4.3 m) kept reappearing for completely different
        # objects. The intrinsics are therefore rescaled to whatever size the
        # frame in hand actually is.
        Kd = self.K_depth
        if Kd is not None and self.depth_wh is not None:
            w0, h0 = self.depth_wh
            # the size camera_info was calibrated for
            if w0 and h0 and (w0 != w or h0 != h):
                sx, sy = w / float(w0), h / float(h0)
                # how much smaller (or larger) the received frame is
                Kd = [Kd[0]*sx, Kd[1], Kd[2]*sx,
                      Kd[3], Kd[4]*sy, Kd[5]*sy,
                      Kd[6], Kd[7], Kd[8]]
                # focal length and optical centre both scale with the image
        if Kd is None or self.K is None:
            # intrinsics not received yet: fall back to proportional scaling
            # so the agent degrades instead of failing outright
            ud = u * (w / 250.0) if w else u
            vd = v * (h / 250.0) if h else v
            # crude, assumes both images cover the same field of view
        else:
            fx_r, fy_r = self.K[0], self.K[4]
            # colour focal lengths in pixels
            cx_r, cy_r = self.K[2], self.K[5]
            # colour optical centre
            fx_d, fy_d = Kd[0], Kd[4]
            # depth focal lengths, rescaled to the frame we actually received (#52)
            cx_d, cy_d = Kd[2], Kd[5]
            # depth optical centre
            x = (u - cx_r) / fx_r
            # horizontal direction of the ray through this colour pixel
            y = (v - cy_r) / fy_r
            # vertical direction of the same ray
            ud = x * fx_d + cx_d
            # re-project that ray onto the depth image's horizontal axis
            vd = y * fy_d + cy_d
            # and onto its vertical axis
        ud, vd = int(round(ud)), int(round(vd))
        # depth arrays are indexed by whole pixels
        if ud < 0 or vd < 0 or ud >= w or vd >= h:
            return None
            # OUTSIDE the depth field of view. This must return None, not clamp:
            # the colour image is taller in FOV than the depth image, so roughly
            # the top and bottom fifth of every colour frame genuinely has no
            # depth. Clamping would invent a distance from an unrelated pixel.
        return ud, vd

    def depth_at(self, u, v, depth_msg):
        """Return (depth_m, u, v), or None if the patch has too few valid
        pixels (OAK-D depth has holes) or falls outside the depth FOV."""
        try:
            depth_img = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        except Exception:
            return None
        mapped = self.rgb_px_to_depth_px(u, v, depth_img.shape)
        # #40: translate the colour pixel into depth-image coordinates first
        if mapped is None:
            return None
            # the point is outside the depth camera's view — report honestly
        ud, vd = mapped
        # depth-image coordinates to sample around
        patch = depth_img[max(0, vd-4):vd+5, max(0, ud-4):ud+5].astype(float)
        # a 9x9 neighbourhood, because a single pixel is easily a hole or speckle
        patch = patch[np.isfinite(patch) & (patch > 0)]
        # discard NaNs and zeros — on the OAK-D zero means "no return"
        if patch.size < 5:                       # too few valid pixels -> unreliable
            return None
        d = float(np.median(patch))
        # median, not mean: one bad reading cannot drag the result
        if not math.isfinite(d) or d <= 0:
            return None
        d = self.depth_units_to_m(d, depth_msg)
        # #41: convert to metres using the declared encoding
        return d, int(u), int(v)
        # NOTE: the returned u, v are the COLOUR pixel, because the caller
        # projects them through the COLOUR intrinsics in _project_uvd

    def project_pixel(self, u, v, depth_msg=None):
        if depth_msg is None:
            pair = self.current_pair()
            if pair is None: return None
            depth_msg = pair[1]
        if self.K is None or self.cam_frame is None:
            return None
        got = self.depth_at(u, v, depth_msg)
        if got is None:
            return None
        d, u, v = got
        return self._project_uvd(u, v, d, depth_msg.header.stamp)

    # ── #21a: depth for a whole BOUNDING BOX, occlusion-robust ──
    def robust_box_depth(self, box, depth_msg):
        """Median depth over a grid of samples across the central region of
        the box. The old single centre-patch read could land on a shelf/rack
        IN FRONT of a partially occluded target and report the occluder's
        distance — which is how 'Arrived at the person' fired metres away
        from the person. With a grid, the occluder must cover most of the
        box to steal the reading. Returns depth in metres, or None."""
        try:
            depth_img = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        except Exception:
            return None
        h, w = depth_img.shape[:2]
        x1, y1, x2, y2 = box
        # central 70% of the box: skip the outer rim, which is mostly
        # background pixels bleeding into the detection
        mx, my = 0.15 * (x2 - x1), 0.15 * (y2 - y1)
        xa, xb = x1 + mx, x2 - mx
        ya, yb = y1 + my, y2 - my
        n = max(2, BOX_DEPTH_GRID)
        # grid density, 5 means 25 samples spread across the box
        vals = []
        # collected valid depth readings, in raw units
        for i in range(n):
            for j in range(n):
                u = xa + (xb - xa) * (i + 0.5) / n
                # colour-image x of this grid sample (kept as float for accuracy)
                v = ya + (yb - ya) * (j + 0.5) / n
                # colour-image y of this grid sample
                mapped = self.rgb_px_to_depth_px(u, v, depth_img.shape)
                # #40: convert the colour pixel into depth-image coordinates.
                # Without this the grid sampled 25 points from the wrong part
                # of the depth image entirely, since the box coordinates come
                # from YOLO on the 250x250 colour frame.
                if mapped is None:
                    continue
                    # sample lies outside the depth FOV — skip it, do not clamp
                ud, vd = mapped
                # depth-image coordinates of this sample
                if 0 <= ud < w and 0 <= vd < h:
                    d = float(depth_img[vd, ud])
                    # read the raw depth value at the mapped location
                    if math.isfinite(d) and d > 0:
                        vals.append(d)
                        # keep only real returns; zero means "no measurement"
        if len(vals) < 5:                        # box mostly holes -> unreliable
            return None
        d = float(np.median(vals))
        # median across the grid: an occluder must cover most of the box to win
        if not math.isfinite(d) or d <= 0:
            return None
        return self.depth_units_to_m(d, depth_msg)
        # #41: convert to metres using the message's declared encoding

    # ── #54 LIDAR RANGING ───────────────────────────────────────────
    #    The OAK-D depth path never produced trustworthy distances on this
    #    robot: a chair 1.5 m away was reported as 6.4, 7.7 and 9.6 m, and the
    #    SAME few values kept reappearing for different objects in different
    #    poses, which is the signature of sampling a background surface rather
    #    than the target. Meanwhile the RPLIDAR, measured against the same
    #    chair, returned 1.514 m - correct to the centimetre - at 10 Hz, in
    #    metres, already in the robot's own frame, with no stereo alignment,
    #    no intrinsics, no compressed transport and no timestamp skew.
    #    A robot driving on a flat floor needs a BEARING and a RANGE. YOLO
    #    supplies the bearing from the colour pixel, which is the one axis the
    #    depth mapping always got right; the LiDAR supplies the range.
    #    Limitation, stated plainly: the LiDAR sees one horizontal plane, so
    #    it ranges whatever part of the object crosses that plane (a chair's
    #    legs, a person's shins). For navigation that is exactly right. It
    #    cannot range something wholly above or below the plane, and in that
    #    case this returns None and the depth path is tried instead.
    def lidar_why(self, reason):
        """#55: report ONCE per distinct reason why LiDAR ranging was not used.
        Every previous attempt to fix ranging was a guess about which step
        failed; this makes the agent say which step failed instead."""
        if reason in self._lidar_reasons:
            return
            # each distinct reason is announced only once, never spammed
        self._lidar_reasons.add(reason)
        self.mlog.log("WARN", "lidar ranging unavailable: " + reason)
        print("\n[lidar] not used — %s\nCommand> " % reason, end="", flush=True)

    def cam_bearing_in_laser(self, u, dist_guess, stamp):
        """Colour pixel column u -> bearing in the LiDAR's own frame."""
        if self.K is None:
            self.lidar_why("no colour camera_info yet (self.K is None)")
            return None
        if self.cam_frame is None:
            self.lidar_why("colour camera frame id unknown")
            return None
        if self.scan_msg is None:
            self.lidar_why("no LaserScan received on " + SCAN_TOPIC)
            return None
        fx, cx = self.K[0], self.K[2]
        # only the horizontal intrinsics matter: bearing is a horizontal angle
        pt = PointStamped()
        pt.header.frame_id = self.cam_frame
        pt.header.stamp = stamp
        # #47: the COLOUR frame's capture time, so the pose matches the bearing
        pt.point.x = (u - cx) * dist_guess / fx
        # sideways offset of the ray at the guessed distance
        pt.point.y = 0.0
        # height is irrelevant - the LiDAR only measures in its own plane
        pt.point.z = dist_guess
        # forward distance along the optical axis
        laser_frame = self.scan_msg.header.frame_id
        try:
            tf = self.tf_buffer.lookup_transform(
                laser_frame, self.cam_frame,
                rclpy.time.Time.from_msg(stamp), timeout=Duration(seconds=0.1))
            q = do_transform_point(pt, tf)
        except Exception:
            try:
                pt.header.stamp = rclpy.time.Time().to_msg()
                tf = self.tf_buffer.lookup_transform(
                    laser_frame, self.cam_frame, rclpy.time.Time())
                q = do_transform_point(pt, tf)
            except Exception as e:
                self.lidar_why("no TF from %s to %s (%s)"
                               % (self.cam_frame, laser_frame, type(e).__name__))
                return None
        # TF is used rather than assuming the camera and LiDAR point the same
        # way - on the TurtleBot 4 they do not share a yaw
        return math.atan2(q.point.y, q.point.x)
        # bearing of the ray as the LiDAR itself would express it

    def scan_range_at(self, bearing, half_span):
        """Range to the NEAREST SOLID SURFACE in [bearing±half_span].

        #61: this used to return the median of every beam in the span, which
        reported the wall behind a chair instead of the chair (see the note
        beside LIDAR_CLUSTER_GAP). It now finds the nearest coherent group of
        beams -- the closest real surface -- and returns its median."""
        m = self.scan_msg
        if m is None or not m.ranges:
            return None
        vals = []
        for i, r in enumerate(m.ranges):
            if not (math.isfinite(r) and m.range_min <= r <= m.range_max):
                continue
                # skip no-returns and out-of-spec readings
            ang = m.angle_min + i * m.angle_increment
            d = math.atan2(math.sin(ang - bearing), math.cos(ang - bearing))
            # signed angular difference, wrapped to +/-pi so 359 deg vs 1 deg
            # is treated as 2 deg apart rather than 358
            if abs(d) <= half_span:
                vals.append(r)
        if len(vals) < 3:
            self.lidar_why("only %d valid beams within %.1f deg of bearing "
                           "%.1f deg (scan frame %s, %d beams total)"
                           % (len(vals), math.degrees(half_span),
                              math.degrees(bearing), m.header.frame_id,
                              len(m.ranges)))
            return None
            # too few beams on the target to trust the reading

        vals.sort()
        # Walk the sorted ranges from nearest to furthest and cut the list
        # wherever consecutive values jump by more than LIDAR_CLUSTER_GAP.
        # Each resulting group is one physical surface at one distance.
        cluster = [vals[0]]
        for r in vals[1:]:
            if r - cluster[-1] <= LIDAR_CLUSTER_GAP:
                cluster.append(r)          # same surface, keep collecting
            else:
                if len(cluster) >= LIDAR_CLUSTER_MIN:
                    break                  # nearest real surface: done
                cluster = [r]              # too few beams -- treat as noise
                                           # and start the next group
        if len(cluster) < LIDAR_CLUSTER_MIN:
            # Nothing formed a credible surface. Fall back to the old median
            # rather than returning nothing, so behaviour degrades instead of
            # failing outright.
            self.lidar_why("no coherent surface in %d beams near bearing "
                           "%.1f deg; falling back to the median"
                           % (len(vals), math.degrees(bearing)))
            return vals[len(vals) // 2]
        return cluster[len(cluster) // 2]
        # median WITHIN the nearest surface: still immune to a single stray
        # beam, but no longer averaged together with the wall behind

    def lidar_project_box(self, box, rgb_msg):
        """Map (x, y) of a detection, ranged by LiDAR. None if unavailable."""
        if rgb_msg is None:
            self.lidar_why("no colour frame passed to the projector")
            return None
        if self.scan_msg is None:
            self.lidar_why("no LaserScan received on " + SCAN_TOPIC)
            return None
        if self.K is None:
            self.lidar_why("no colour camera_info yet (self.K is None)")
            return None
        x1, y1, x2, y2 = box
        uc = (x1 + x2) / 2.0
        # horizontal centre of the detection
        stamp = rgb_msg.header.stamp
        # first pass: guess a distance to get an approximate bearing
        b = self.cam_bearing_in_laser(uc, LIDAR_BEARING_GUESS_M, stamp)
        if b is None:
            return None
        # the box's angular half-width, so the median samples the object only
        fx = self.K[0]
        half = min(LIDAR_MAX_HALF_SPAN,
                   max(LIDAR_MIN_HALF_SPAN, abs(x2 - x1) / (2.0 * fx)))
        r = self.scan_range_at(b, half)
        if r is None:
            return None
        # second pass: redo the bearing at the measured range. The camera and
        # LiDAR sit a few centimetres apart, so the bearing depends slightly
        # on distance; one refinement removes that error.
        b2 = self.cam_bearing_in_laser(uc, r, stamp)
        if b2 is not None:
            r2 = self.scan_range_at(b2, half)
            if r2 is not None:
                b, r = b2, r2
        laser_frame = self.scan_msg.header.frame_id
        pt = PointStamped()
        pt.header.frame_id = laser_frame
        pt.header.stamp = stamp
        pt.point.x = r * math.cos(b)
        # the measured point, in the LiDAR's own frame
        pt.point.y = r * math.sin(b)
        pt.point.z = 0.0
        try:
            tf = self.tf_buffer.lookup_transform(
                MAP_FRAME, laser_frame,
                rclpy.time.Time.from_msg(stamp), timeout=Duration(seconds=0.1))
            obj = do_transform_point(pt, tf)
        except Exception:
            try:
                pt.header.stamp = rclpy.time.Time().to_msg()
                tf = self.tf_buffer.lookup_transform(
                    MAP_FRAME, laser_frame, rclpy.time.Time())
                obj = do_transform_point(pt, tf)
            except Exception as e:
                self.lidar_why("no TF from %s to %s (%s)"
                               % (laser_frame, MAP_FRAME, type(e).__name__))
                return None
        self.lidar_fixes += 1
        # counted so it is visible how often ranging came from the LiDAR
        return obj.point.x, obj.point.y

    def range_for_box(self, box, depth_msg, rgb_msg=None):
        """Distance in metres to a detection: LiDAR first, depth as fallback."""
        if USE_LIDAR_RANGE and rgb_msg is not None and self.scan_msg is not None:
            x1, y1, x2, y2 = box
            uc = (x1 + x2) / 2.0
            b = self.cam_bearing_in_laser(uc, LIDAR_BEARING_GUESS_M,
                                          rgb_msg.header.stamp)
            if b is not None and self.K is not None:
                half = min(LIDAR_MAX_HALF_SPAN,
                           max(LIDAR_MIN_HALF_SPAN,
                               abs(x2 - x1) / (2.0 * self.K[0])))
                r = self.scan_range_at(b, half)
                if r is not None:
                    b2 = self.cam_bearing_in_laser(uc, r, rgb_msg.header.stamp)
                    if b2 is not None:
                        r2 = self.scan_range_at(b2, half)
                        if r2 is not None:
                            return r2
                    return r
        if depth_msg is None:
            return None
        return self.robust_box_depth(box, depth_msg)
        # depth is still there for anything the LiDAR plane cannot see

    def project_box(self, box, depth_msg, rgb_msg=None):
        """Map (x, y) of a detection: bearing from the box centre, depth from
        the occlusion-robust grid over the whole box (#21a).

        #47 WRONG-DIRECTION GOALS. v15 looked up TF at the DEPTH frame's
        timestamp. That is correct only when RGB and depth are synchronized.
        On the real robot depth arrives at well under 1 Hz, so current_pair()
        falls back to latest-of-each and the two frames can be SECONDS apart.
        The bearing to the target is measured from the COLOUR pixel, but the
        camera pose was being read at the depth frame's older time. While the
        robot rotated during a search, that stale yaw rotated the whole ray:
        an object dead ahead was projected off to one side, and sometimes
        behind the robot. Nav2 then drove faithfully to a goal that was in
        the wrong place — the "it backs up and turns away from the chair
        that is right in front of it" symptom.
        The bearing and the pose must come from the SAME instant, so TF is
        now looked up at the COLOUR frame's stamp."""
        # #54: LiDAR first - it is the sensor that measures this robot's world
        # correctly. Only if it cannot see the target does the depth path run.
        if USE_LIDAR_RANGE:
            p = self.lidar_project_box(box, rgb_msg)
            if p is not None:
                return p
        if self.K is None or self.cam_frame is None or depth_msg is None:
            return None
        # #48: refuse to project from depth that is too old to trust. A range
        # reading from four seconds ago describes where the object was, not
        # where it is. Saying "distance unclear" is honest; inventing a map
        # position from it is what sent the robot to empty floor.
        if rgb_msg is not None:
            skew = abs(self.stamp_to_sec(rgb_msg.header.stamp)
                       - self.stamp_to_sec(depth_msg.header.stamp))
            # how far apart the colour and depth captures actually were
            if skew > MAX_PAIR_SKEW_S:
                self.stale_depth_drops += 1
                # counted so the operator can see how often this happens
                return None
        d = self.robust_box_depth(box, depth_msg)
        if d is None:
            # fall back to the old centre read rather than dropping the target
            x1, y1, x2, y2 = box
            got = self.depth_at((x1 + x2) / 2, (y1 + y2) / 2, depth_msg)
            if got is None:
                return None
            d = got[0]
        x1, y1, x2, y2 = box
        stamp = (rgb_msg.header.stamp if rgb_msg is not None
                 else depth_msg.header.stamp)
        # #47: the colour frame's stamp is when the BEARING was observed,
        # which is the pose the ray must be rotated by
        return self._project_uvd((x1 + x2) / 2, (y1 + y2) / 2, d, stamp)
        # nanosec is an integer field; 1e-9 converts it to fractional seconds

    def _project_uvd(self, u, v, d, stamp):
        """Pixel (u, v) at depth d (m) -> map (x, y) via camera ray + TF at
        the frame's capture time (#8)."""
        fx, fy = self.K[0], self.K[4]; cx, cy = self.K[2], self.K[5]
        pt = PointStamped()
        pt.header.frame_id = self.cam_frame
        pt.header.stamp = stamp                  # the time the frame was actually taken
        pt.point.x = (u - cx) * d / fx
        pt.point.y = (v - cy) * d / fy
        pt.point.z = d
        try:
            tf = self.tf_buffer.lookup_transform(
                MAP_FRAME, self.cam_frame, rclpy.time.Time.from_msg(stamp),
                timeout=Duration(seconds=0.1))
            obj = do_transform_point(pt, tf)
        except Exception:
            try:                                  # fallback: latest TF (old behaviour)
                pt.header.stamp = rclpy.time.Time().to_msg()
                tf = self.tf_buffer.lookup_transform(MAP_FRAME, self.cam_frame, rclpy.time.Time())
                obj = do_transform_point(pt, tf)
            except Exception:
                return None
        return obj.point.x, obj.point.y

    # ── #21b: TELEPORT GATE for committed targets ──
    def accept_sighting(self, ox, oy, meas_dist=None):
        """Objects don't teleport; depth noise and occluder-stolen depth do.
        Once committed to a target, a new sighting that moves it by more than
        TELEPORT_JUMP metres is held as 'pending' and only believed when a
        SECOND look agrees with it. One bad frame can no longer hijack the
        goal or fake an arrival.

        #30b: 'agrees' now SCALES WITH RANGE. Stereo depth error grows with
        distance, so two honest looks at a target 10 m away can legitimately
        disagree by more than a metre. The old fixed 1.0 m radius made a
        genuine RE-TARGET (the closest chair coming into view after the robot
        had committed to a far one) impossible to confirm — the robot kept
        driving to the wrong chair. A stale pending also expires, so a single
        odd frame can't block updates indefinitely."""
        if self.last_obj_xy is None:
            self.jump_pending = None
            return True
        now = time.monotonic()
        jump = math.hypot(ox - self.last_obj_xy[0], oy - self.last_obj_xy[1])
        if jump <= TELEPORT_JUMP:
            self.jump_pending = None
            return True
        agree = max(JUMP_AGREE,
                    JUMP_AGREE_FRAC * (meas_dist if meas_dist else jump))
        if (self.jump_pending is not None
                and now - self.jump_pending_t <= JUMP_PENDING_TTL
                and math.hypot(ox - self.jump_pending[0],
                               oy - self.jump_pending[1]) <= agree):
            self.mlog.log("SIGHT", f"large {self.target} jump ({jump:.1f} m) "
                                   f"confirmed by a second look — accepting")
            self.jump_pending = None
            return True
        self.mlog.log("WARN", f"suspicious {self.target} jump of {jump:.1f} m "
                              f"(occlusion/depth noise?) — waiting for a confirming "
                              f"look (agree radius {agree:.1f} m)")
        self.jump_pending = (ox, oy)
        self.jump_pending_t = now
        return False

    # ── #28: EVERY visible instance of the target, with distances ──
    def locate_candidates(self):
        """Return (cands, rx, ry) where cands is a list of
        (ox, oy, dist_m, pixel_u) for every instance of self.target the camera
        can see AND range right now, nearest first. Returns ([], None, None)
        if we can't see or can't range any.

        Factored out of locate_target so that reporting ("how far are the
        chairs") and acting ("go to the chair") use the SAME numbers — a
        report that disagreed with the goal would be worthless as evidence."""
        pair = self.current_pair()
        if pair is None: return [], None, None
        rgb_msg, depth_msg = pair
        detections = self.yolo_detect(rgb_msg)
        if detections is None: return [], None, None
        matches = [d for d in detections if self.target and self.target in d["name"].lower()]
        if not matches: return [], None, None
        pose = self.robot_pose_map()
        if pose is None: return [], None, None
        rx, ry, _ = pose
        cands = []                               # (ox, oy, dist, pixel_u)
        for d in matches:
            x1, y1, x2, y2 = d["box"]
            proj = self.project_box(d["box"], depth_msg, rgb_msg)  # #21a + #47
            if proj is None: continue
            ox, oy = proj
            # #22: anything sitting ON the dock IS the dock (YOLO-World kept
            # labelling it "chair") — skip it unless the dock was the target.
            if "dock" not in self.target and self.near_dock(ox, oy):
                continue
            # #28: two boxes on the SAME physical object (YOLO-World stacks
            # them on plain shapes) must not be reported as two objects.
            if any(math.hypot(ox - cx, oy - cy) < REPORT_MERGE_RADIUS
                   for cx, cy, _, _ in cands):
                continue
            cands.append((ox, oy, math.hypot(ox - rx, oy - ry), (x1 + x2) / 2))
        cands.sort(key=lambda c: c[2])           # nearest first
        return cands, rx, ry

    # ── locate the target; #3: choose nearest (or qualifier) among instances ──
    def locate_target(self):
        cands, rx, ry = self.locate_candidates()
        if not cands: return None
        q = self.target_qualifier
        if q == "farthest":
            ox, oy, dist, _ = max(cands, key=lambda c: c[2])
        elif q == "leftmost":
            ox, oy, dist, _ = min(cands, key=lambda c: c[3])   # smaller u = left in image
        elif q == "rightmost":
            ox, oy, dist, _ = max(cands, key=lambda c: c[3])
        else:                                                  # nearest (default)
            ox, oy, dist, _ = min(cands, key=lambda c: c[2])
        return ox, oy, rx, ry, dist

    # ── #28: turn a candidate list into one natural sentence ──
    def describe_distances(self, cands):
        """'about 6.3 m away' / 'one 6.3 m away and another 12.4 m away' /
        '2.1 m, 5.4 m and 9.8 m away'. Nearest first."""
        ds = [c[2] for c in cands]
        if len(ds) == 1:
            return f"about {ds[0]:.1f} m away"
        if len(ds) == 2:
            return f"one about {ds[0]:.1f} m away and another about {ds[1]:.1f} m away"
        head = ", ".join(f"{d:.1f} m" for d in ds[:-1])
        return f"at about {head} and {ds[-1]:.1f} m away"

    # ── project EVERY detection of a class to map (x, y) ──
    def locate_all(self, target, detections=None, depth_msg=None, rgb_msg=None):
        # #47: rgb_msg is carried through so the TF lookup can use the COLOUR
        # frame's stamp, which is the instant the bearing was actually observed
        if detections is None:
            pair = self.current_pair()
            if pair is None: return []
            detections = self.yolo_detect(pair[0]) or []
            rgb_msg = pair[0]
            depth_msg = pair[1]
        out = []
        for d in detections:
            if target and target in d["name"].lower():
                proj = self.project_box(d["box"], depth_msg, rgb_msg)  # #21a + #47
                if proj is not None:
                    if "dock" not in target and self.near_dock(proj[0], proj[1]):
                        continue                                   # #22
                    out.append(proj)
        return out

    # ── remember distinct object positions; return how many were NEW ──
    def register_instances(self, target, positions):
        known = self.seen_instances.setdefault(target, [])
        added = 0
        for (x, y) in positions:
            if "dock" not in target and self.near_dock(x, y):
                continue                         # #22: never memorise the dock as an object
            if all(math.hypot(x - kx, y - ky) > NEW_INSTANCE_RADIUS for (kx, ky) in known):
                known.append((x, y)); added += 1
        return added

    # ── #17: pick a remembered instance of a class (qualifier-aware) ──
    def recall_instance(self, target):
        """Return the remembered map (x, y) of a previously seen instance of
        'target', honouring nearest/farthest. leftmost/rightmost are camera-
        relative so they don't apply to memory -> fall back to nearest."""
        # #23: "find a chair" ... "go to it" must return the chair we JUST
        # located — not whichever remembered chair happens to be nearest.
        # An explicit qualifier (nearest/farthest/...) still wins.
        if (self.last_located is not None and self.last_located[0] == target
                and self.target_qualifier is None):
            return self.last_located[1]
        known = self.seen_instances.get(target, [])
        if not known:
            return None
        pose = self.robot_pose_map()
        if pose is None:
            return known[-1]                     # best effort: most recent
        rx, ry, _ = pose
        if self.target_qualifier == "farthest":
            return max(known, key=lambda p: math.hypot(p[0]-rx, p[1]-ry))
        return min(known, key=lambda p: math.hypot(p[0]-rx, p[1]-ry))

    def _class_color(self, name):
        """A stable BGR colour per class (md5, not hash(), so the colours are
        the same every run — screenshots stay consistent across slides)."""
        h = hashlib.md5(name.encode("utf-8")).digest()
        return (int(70 + h[0] % 186), int(70 + h[1] % 186), int(70 + h[2] % 186))

    def feed_watchers(self):
        """#34: how many nodes are subscribed to the annotated feed right now."""
        try:
            return (self.annot_pub.get_subscription_count()
            # counts subscribers on the raw Image topic (e.g. rqt_image_view)
                    + self.annot_jpg_pub.get_subscription_count())
            # plus subscribers on the compressed JPEG topic (that is the GUI)
        except Exception:
            return 0
            # if rclpy ever refuses the query, fall back to "nobody watching" -
            # a blank panel is a far smaller failure than a crashed agent

    def feed_watchdog(self):
        """#32e: notice when the viewer has died or the camera has stopped,
        instead of silently publishing into the void."""
        if self.feed_proc is not None and self.feed_proc.poll() is not None:
            if not self.feed_warned_dead:
                self.feed_warned_dead = True
                self.notify("The live-feed window closed. Say 'show me the feed' "
                            "to reopen it — the robot is otherwise unaffected.")
                self.mlog.log("WARN", "#32e rqt_image_view exited")
            self.feed_proc = None
        if (self.rgb_raw_t > 0.0 and not self.warned_cam_stall
                and time.monotonic() - self.rgb_raw_t > CAM_STALL_WARN):
            self.warned_cam_stall = True
            self.notify(f"I've stopped receiving camera frames (none for over "
                        f"{CAM_STALL_WARN:.0f}s). Check the OAK-D driver — I can "
                        f"still move, but I'm blind until it's back.")
            self.mlog.log("WARN", f"#32e camera stream stalled (>{CAM_STALL_WARN:.0f}s)")
            # This fires the instant the gap passes the threshold, so it can only
            # ever say "over 5s". It CANNOT tell you how long the outage actually
            # lasted -- that number is only knowable on the first frame back, and
            # is recorded there, in on_rgb().

    def annotate_think(self):
        """Republish the camera image with YOLO boxes drawn on it, so the live
        window is screenshot-ready evidence. Only runs while the feed is open.

        #32c: this function no longer runs YOLO. It used to, whenever the
        current frame's timestamp differed from the one the think-loop had
        cached — which is most of the time, because the two timers tick at
        different rates. That meant a second GPU inference per frame AND a
        second blocking HTTP call on the feed's own thread, so a busy GPU
        stalled the video. Now the feed only ever DRAWS: it reuses the
        think-loop's detections while they are fresher than ANNOT_DET_TTL and
        states their age in the caption bar. If they go stale it shows clean
        video rather than rectangles that no longer match the picture."""
        self.feed_watchdog()
        if not ANNOTATED_FEED:
            return
        # the feature is switched off entirely - nothing to do
        if self.feed_proc is None and self.feed_watchers() == 0:
            return
        # #34: publish whenever ANYONE is watching, not only when the agent itself
        # spawned an rqt_image_view window. The GUI subscribes to
        # /vla/annotated/compressed directly, so before this fix its camera panel
        # stayed black unless you ALSO said "show me the feed" and got a second,
        # redundant window. With zero subscribers we still return immediately, so
        # the CPU cost when nobody is looking is exactly as before.
        pair = self.current_pair()
        if pair is None:
            return
        rgb_msg, depth_msg = pair
        try:
            img = self.feed_bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
        except Exception:
            return

        # #32c: DRAW ONLY — never infer here.
        key = self._stamp_key(rgb_msg)
        dets, det_age = [], None
        cached = self.last_det
        if cached is not None:
            age = time.monotonic() - self.last_det_t
            if cached[0] == key:
                dets, det_age = cached[1], 0.0        # exactly this frame
            elif age <= ANNOT_DET_TTL:
                dets, det_age = cached[1], age        # recent enough to be meaningful

        # the sim's OAK-D preview is only 250x250; upscale so the labels are
        # readable when this gets pasted into a slide.
        # #32d: capped — the old ceil() could triple the width and put ~1.7 MB
        # on the wire per frame, which is what jammed the transport.
        h0, w0 = img.shape[:2]
        scale = max(1, min(ANNOT_MAX_SCALE,
                           int(math.ceil(ANNOT_MIN_WIDTH / float(max(w0, 1))))))
        img = (cv2.resize(img, (w0 * scale, h0 * scale), interpolation=cv2.INTER_LINEAR)
               if scale > 1 else img.copy())
        H, W = img.shape[:2]
        font = cv2.FONT_HERSHEY_SIMPLEX
        fs = max(0.40, W / 1500.0)
        lw = max(2, int(round(W / 400.0)))

        for d in dets:
            try:
                x1, y1, x2, y2 = [float(b) for b in d["box"]]
                name = str(d.get("name", "?"))
                conf = float(d.get("confidence", 0.0))
            except Exception:
                continue
            # highlight the object we're actually acting on
            is_target = bool(self.target and self.target in name.lower())
            color = (0, 255, 0) if is_target else self._class_color(name)
            X1, Y1 = int(round(x1 * scale)), int(round(y1 * scale))
            X2, Y2 = int(round(x2 * scale)), int(round(y2 * scale))
            cv2.rectangle(img, (X1, Y1), (X2, Y2), color, lw + (1 if is_target else 0))

            label = f"{name} {conf * 100:.0f}%"
            bd = (self.range_for_box(d["box"], depth_msg, rgb_msg) if RGB_ONLY   # #64: LiDAR range
                  else self.robust_box_depth(d["box"], depth_msg))  # #21a: same number nav uses
            if bd is not None:                   # YOLO + OAK-D depth, in one frame
                label += f"  {bd:.2f}m"
            if is_target:
                label += "  <= TARGET"
            (tw, tht), _ = cv2.getTextSize(label, font, fs, 1)
            ly = max(Y1, tht + 6)
            cv2.rectangle(img, (X1, ly - tht - 6), (X1 + tw + 6, ly), color, -1)
            cv2.putText(img, label, (X1 + 3, ly - 4), font, fs, (0, 0, 0), 1, cv2.LINE_AA)

        # caption bar ABOVE the image (never covers a detection).
        # #32c: DET age is stated, so a screenshot can never overclaim that the
        # boxes were computed on that exact frame.
        # #31: HOP is shown while the robot is crawling into unmapped space —
        # one glance tells you which navigation regime is running.
        det_txt = "DET: none" if det_age is None else (
            "DET: live" if det_age <= 0.001 else f"DET: {det_age:.1f}s ago")
        hop_txt = f"   HOP {self.step_count}" if self.step_active else ""
        cap = (f"MODE: {self.mode or 'idle'}{hop_txt}   TARGET: {self.target or '-'}   "
               f"OBJECTS: {len(dets)}   {det_txt}   "
               f"{datetime.datetime.now().strftime('%H:%M:%S')}")
        (tw, tht), _ = cv2.getTextSize(cap, font, fs, 1)
        bar = np.zeros((tht + 14, W, 3), dtype=np.uint8)
        cv2.putText(bar, cap, (6, tht + 5), font, fs, (255, 255, 255), 1, cv2.LINE_AA)
        img = np.vstack([bar, img])

        # #32d: publish BOTH. The raw Image keeps rqt_image_view and any
        # existing tooling working; the JPEG is ~30x smaller and is what the
        # GUI and anything over Wi-Fi should subscribe to. Both use KEEP_LAST
        # depth 1, so a slow consumer drops frames instead of building the
        # backlog that used to wedge the stream permanently.
        #
        # #49 PER-TOPIC GATING. v16 built and published BOTH every cycle,
        # regardless of who was listening. The GUI only ever subscribes to the
        # compressed topic, so with the GUI open the agent was still doing a
        # cv2_to_imgmsg() and a full serialization of an upscaled frame, five
        # times a second, for a topic with zero subscribers. That work happens
        # on the agent's own executor threads, so it starved the camera
        # callbacks — which is why the feed felt laggy AND why the agent
        # announced "I've stopped receiving camera frames" while the camera
        # was in fact publishing normally. Each topic is now built only if
        # somebody is actually subscribed to that topic.
        try:
            if self.annot_pub.get_subscription_count() > 0:
                # only rqt_image_view and similar raw consumers pay this cost
                out = self.feed_bridge.cv2_to_imgmsg(img, encoding="bgr8")
                out.header = rgb_msg.header
                self.annot_pub.publish(out)
        except Exception:
            pass
        try:
            if self.annot_jpg_pub.get_subscription_count() > 0:
                # the JPEG encode is much cheaper, but still skipped if nobody looks
                ok, buf = cv2.imencode(".jpg", img,
                                       [int(cv2.IMWRITE_JPEG_QUALITY), ANNOT_JPEG_Q])
                if ok:
                    cm = CompressedImage()
                    cm.header = rgb_msg.header
                    cm.format = "jpeg"
                    cm.data = buf.tobytes()
                    self.annot_jpg_pub.publish(cm)
        except Exception:
            pass

    def start_feed(self):
        self.stop_feed()
        self.feed_warned_dead = False                # #32e
        topic = ANNOT_TOPIC if ANNOTATED_FEED else RGB_TOPIC
        try:
            self.feed_proc = subprocess.Popen(
                ["ros2", "run", "rqt_image_view", "rqt_image_view", topic],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            if ANNOTATED_FEED:
                print(f"Live feed opened on {topic} — YOLO boxes, confidence and "
                      f"depth distance are drawn on it. (type 'cancel' to close it)")
                print(f"      (over Wi-Fi or in the GUI use {ANNOT_COMPRESSED_TOPIC} "
                      f"instead — same picture, ~30x less data)")
            else:
                print("Live camera feed opened in a new window. (type 'cancel' to close it)")
        except FileNotFoundError:
            print("Live feed needs rqt_image_view:  sudo apt install ros-humble-rqt-image-view")

    def stop_feed(self):
        if self.feed_proc is not None:
            try: self.feed_proc.terminate()
            except Exception: pass
            self.feed_proc = None
