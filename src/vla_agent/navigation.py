"""Navigation: the occupancy grid, goal selection, Nav2 and Dock action clients, frontiers.

Methods moved verbatim out of VLAAgent in Phase 2 of the reorg. No logic
changed; the comments are the originals. Mixed into VLAAgent in
vla_agent/agent.py, ahead of rclpy's Node, so method resolution is unchanged.
"""
from .core import *            # noqa: F401,F403 - constants and ROS imports
from .core import print        # noqa: A001 - headless-safe console, fix #63


class NavigationMixin:

    # ── #22: dock no-detect zone ──
    def set_dock_position(self, x, y):
        """Remember where the dock is, and purge any 'objects' the memory is
        holding at that spot — they were the dock wearing a costume."""
        self.dock_xy = (x, y)
        self.mlog.log("DOCK", f"dock position recorded at ({x:.2f}, {y:.2f}); "
                              f"detections within {DOCK_EXCLUDE_RADIUS} m are ignored")
        for cls in list(self.seen_instances.keys()):
            if "dock" in cls:
                continue                         # remembering the dock AS a dock is fine
            kept = [p for p in self.seen_instances[cls]
                    if not self.near_dock(p[0], p[1])]
            dropped = len(self.seen_instances[cls]) - len(kept)
            if dropped:
                self.seen_instances[cls] = kept
                self.mlog.log("MEMORY", f"purged {dropped} remembered {cls} "
                                        f"position(s) that sat on the dock")
        if (self.last_located is not None
                and self.near_dock(self.last_located[1][0], self.last_located[1][1])):
            self.last_located = None

    def near_dock(self, x, y):
        """True if (x, y) falls inside the dock's exclusion zone."""
        return (self.dock_xy is not None and
                math.hypot(x - self.dock_xy[0], y - self.dock_xy[1]) < DOCK_EXCLUDE_RADIUS)

    def robot_xy(self):
        pose = self.robot_pose_map()
        return None if pose is None else (pose[0], pose[1])

    def robot_pose_map(self):
        """Robot (x, y, yaw) in the MAP frame from TF (correct for goals)."""
        try:
            tf = self.tf_buffer.lookup_transform(MAP_FRAME, ROBOT_FRAME, rclpy.time.Time())
            return (tf.transform.translation.x,
                    tf.transform.translation.y,
                    yaw_from_quat(tf.transform.rotation))
        except Exception:
            return None

    # ── occupancy-grid helpers (#5: keep goals out of walls) ──
    def grid_value(self, x, y):
        m = self.map
        if m is None: return None
        res = m.info.resolution
        ox = m.info.origin.position.x; oy = m.info.origin.position.y
        cx = int((x - ox) / res); cy = int((y - oy) / res)
        if cx < 0 or cy < 0 or cx >= m.info.width or cy >= m.info.height:
            return None
        return m.data[cy * m.info.width + cx]

    def cell_free(self, x, y):
        v = self.grid_value(x, y)
        return v is not None and 0 <= v < 50     # known + not (near-)occupied

    # #19b: a single free CELL is not enough — the robot has a body. Check a
    # disc of ROBOT_CLEARANCE around the point so our "free" agrees with
    # Nav2's inflation layer. Without this we happily sent goals 5 cm from a
    # wall and Nav2 (correctly) refused them, which looked like "it can't
    # avoid obstacles".
    def is_goal_free(self, x, y, clearance=ROBOT_CLEARANCE):
        m = self.map
        if m is None:
            return False
        if not self.cell_free(x, y):
            return False
        res = m.info.resolution
        r = max(1, int(math.ceil(clearance / res)))
        r2 = r * r
        for iy in range(-r, r + 1):
            for ix in range(-r, r + 1):
                if ix * ix + iy * iy > r2:
                    continue
                if not self.cell_free(x + ix * res, y + iy * res):
                    return False
        return True

    # ── #31: three-way cell state, so "unknown" stops meaning "forbidden" ──
    def cell_state(self, x, y):
        """'free' | 'occupied' | 'unknown'.

        cell_free() above collapses UNKNOWN and OCCUPIED into one answer:
        False. For a full Nav2 goal that is correct and conservative. For a
        SHORT hop it is the bug — it is what made the whole unmapped half of
        the world unreachable. Outside the current grid also counts as
        unknown, not as a wall: a SLAM map grows, and the robot's own motion
        is what grows it."""
        v = self.grid_value(x, y)
        if v is None:
            return "unknown"                 # beyond the current grid extent
        if v < 0:
            return "unknown"                 # -1: never observed
        if v >= 50:
            return "occupied"
        return "free"

    def is_step_goal_ok(self, x, y, clearance=STEP_CLEARANCE):
        """#31: may a HOP goal end here? Relaxed twin of is_goal_free():
        UNKNOWN is acceptable, OCCUPIED never is.

        The safety argument, in order of strength:
          1. The hop is at most STEP_GOAL_DIST (1.5 m). The LiDAR local
             costmap covers that range continuously, so the robot is never
             navigating blind — only PLANNING through cells SLAM has not
             written yet.
          2. Nav2's global planner already accepts unknown cells; it simply
             will not choose a goal for us. This function chooses one.
          3. If reality disagrees, Nav2 aborts the goal, we shorten the hop
             and sidestep. The cost of being wrong is one 1.5 m goal."""
        m = self.map
        if m is None:
            return True                      # no map at all: LiDAR + Nav2 still guard us
        if self.cell_state(x, y) == "occupied":
            return False
        res = m.info.resolution
        r = max(1, int(math.ceil(clearance / res)))
        r2 = r * r
        for iy in range(-r, r + 1):
            for ix in range(-r, r + 1):
                if ix * ix + iy * iy > r2:
                    continue
                if self.cell_state(x + ix * res, y + iy * res) == "occupied":
                    return False
        return True

    def object_is_mapped(self, ox, oy, margin=0.8):
        """#31: is the ground AROUND the object already surveyed? Used only
        for logging and for the one-off message to the operator — the actual
        regime switch is driven by whether approach_candidates() can produce
        a legal full goal, which is the thing that really matters."""
        m = self.map
        if m is None:
            return False
        res = m.info.resolution
        r = max(1, int(math.ceil(margin / res)))
        step = max(1, r // 3)
        total = unknown = 0
        for iy in range(-r, r + 1, step):
            for ix in range(-r, r + 1, step):
                if ix * ix + iy * iy > r * r:
                    continue
                total += 1
                if self.cell_state(ox + ix * res, oy + iy * res) == "unknown":
                    unknown += 1
        return total > 0 and (unknown / float(total)) <= 0.25

    def step_goal_toward(self, ox, oy, rx, ry):
        """#31: the next HOP. Returns (gx, gy, bearing) or None.

        Aim at the object, walk out self.step_len metres, and accept the
        first end point that is not inside an obstacle. If the straight line
        is blocked, fan out sideways (STEP_FAN_DEG) — Nav2 still plans the
        actual path, we are only moving the END POINT off the wall so that a
        legal goal exists. If the whole fan is blocked at this length, try
        shorter lengths before admitting defeat."""
        dist_obj = math.hypot(ox - rx, oy - ry)
        if dist_obj < 1e-3:
            return None
        base = math.atan2(oy - ry, ox - rx)
        # never hop further than the object itself, and never past it
        lengths = []
        L = min(self.step_len, max(STEP_GOAL_MIN, dist_obj - STOP_DISTANCE))
        while L >= STEP_GOAL_MIN:
            lengths.append(L)
            L *= 0.6
        if not lengths:
            return None
        for L in lengths:
            for off_deg in STEP_FAN_DEG:
                a = base + math.radians(off_deg)
                gx = rx + L * math.cos(a)
                gy = ry + L * math.sin(a)
                if not self.is_step_goal_ok(gx, gy):
                    continue
                if self.is_goal_tried(gx, gy):
                    continue                 # Nav2 already refused this exact spot
                return gx, gy, base
        return None

    def reset_step_state(self):
        """#31: back to 'no hop in progress' — called whenever a navigation
        task starts, ends or is cancelled. step_len is restored to nominal so
        a hop that had to be shortened around one obstacle does not
        permanently cripple the next task."""
        self.step_active = False
        self.step_bearing = None
        self.step_start_t = 0.0
        self.step_len = STEP_GOAL_DIST
        self.step_count = 0
        self.step_announced = False

    # ── #19c: candidate approach goals in a RING around the object ──
    def is_goal_tried(self, x, y):
        return any(math.hypot(x - tx, y - ty) < GOAL_TRIED_RADIUS
                   for tx, ty in self.goal_tried)

    def approach_candidates(self, ox, oy, rx, ry):
        """Return reachable-looking stand-off goals AROUND the object at
        (ox, oy), best first.

        The old code used exactly ONE goal: the point on the straight line
        between the robot and the object. If a box sat on that line the task
        died — even though the object's left/right/far side was wide open.
        Here we sweep a ring of directions and stand-off distances, drop any
        that don't fit the robot's footprint or that Nav2 already refused,
        and rank the rest by travel distance (plus a mild penalty for walking
        around to the far side). Nav2 still plans the actual path — this only
        picks WHERE to end up."""
        if self.map is None:
            return []
        base = math.atan2(ry - oy, rx - ox)      # object -> robot: our own side
        out = []
        for standoff in APPROACH_STANDOFFS:
            for off_deg in APPROACH_RING:
                a = base + math.radians(off_deg)
                gx = ox + standoff * math.cos(a)
                gy = oy + standoff * math.sin(a)
                if not self.is_goal_free(gx, gy):
                    continue
                if self.is_goal_tried(gx, gy):
                    continue
                travel = math.hypot(gx - rx, gy - ry)
                cost = travel + 0.30 * abs(math.radians(off_deg))
                out.append((cost, gx, gy))
        out.sort(key=lambda c: c[0])
        return [(gx, gy) for _, gx, gy in out]

    def nudge_to_free(self, x, y, rx, ry):
        """If (x,y) is in a wall/unknown, pull it back toward the robot to the
        nearest free point on that line. Returns None if nothing works."""
        if self.is_goal_free(x, y):
            return x, y
        for frac in (0.85, 0.7, 0.55, 0.4, 0.25):
            nx = rx + (x - rx) * frac; ny = ry + (y - ry) * frac
            if self.is_goal_free(nx, ny):
                return nx, ny
        return None

    # ── #18: approach the EDGE of the known map toward a far target ──
    def farthest_free_along(self, rx, ry, x, y, min_d=0.5):
        """Walk the robot->target line in map-resolution steps and return the
        FARTHEST free point that is at least min_d from the robot. When the
        target was projected into unmapped space (e.g. a chair seen 14 m away,
        beyond where SLAM has mapped), this gives Nav2 a reachable goal at the
        frontier of the known map; as the robot gets there SLAM extends the
        map and the next think-cycle pushes the goal further. Returns None
        only if there's no free point beyond min_d (then we should rescan)."""
        if self.map is None:
            return None
        dx, dy = x - rx, y - ry
        dist = math.hypot(dx, dy)
        if dist < min_d:
            return None
        step = max(self.map.info.resolution, 0.10)
        best = None
        n = int(dist / step)
        for i in range(1, n + 1):
            d = i * step
            px = rx + dx / dist * d
            py = ry + dy / dist * d
            if d >= min_d and self.is_goal_free(px, py):
                best = (px, py)     # keep the farthest; walls in between are
                                    # fine — Nav2 plans AROUND them (#5)
        return best

    def start_next_step(self):
        if not self.queue:
            return
        step = self.queue.pop(0)
        action = (step.get("action") or "").lower()
        target = step.get("target")
        speech = step.get("speech", "")
        if speech:
            print(f"[{action}] {speech}")
        self.mlog.log("STEP", f"{action} target={target!r}")              # #14

        # #11: remember the last thing we acted on for pronoun resolution
        if target:
            self.ctx_last_action = action
            self.ctx_last_target = str(target).lower()
            self.ctx_time = time.monotonic()                              # #27

        # #10: if the robot is docked and the next step needs motion, undock first
        # then resume this step. (This is why Nav2 "stopped working" after docking.)
        if action in MOVE_ACTIONS and HAVE_CREATE and self.is_docked:
            self.queue.insert(0, step)
            self.notify("I'm docked — undocking first, then I'll continue.")
            self.start_dock_action("undock")
            return

        if action == "navigate":
            if not target:
                self.start_next_step(); return
            self.target = target.lower()
            self.target_qualifier = (step.get("qualifier") or "").lower() or None   # #3
            self.search_state = "ROTATE"
            self.turn_accum = 0.0; self.last_yaw = None; self.advance_start = None
            self.last_obj_xy = None
            self.target_announced = False
            self.nav_goal_xy = None
            self.sight_count = 0                                                    # #13
            self.seen_live = False                                                  # #17
            self.goal_fail_count = 0                                                # #18
            self.reset_step_state()                                                 # #31
            self.navigate_start = time.monotonic()                                  # #6
            self.mode = "navigate"
            extra = f" ({self.target_qualifier})" if self.target_qualifier else ""
            # #23 LIVE-FIRST: if the target class is visible RIGHT NOW, search
            # fresh (it locks on within a couple of think-cycles) and skip the
            # memory seed — no "I remember..." monologue for something in
            # plain sight, and no driving to a stale spot while the real
            # object sits in front of the camera. Memory's job (#17) is only
            # returning to things that AREN'T currently visible.
            live_now = False
            pair = self.current_pair()
            if pair is not None:
                dets = self.yolo_detect(pair[0]) or []
                live_now = any(self.target in d["name"].lower() for d in dets)
            seed = None
            if MEMORY_SEED and not live_now:
                seed = self.recall_instance(self.target)
            if seed is not None:
                self.last_obj_xy = seed
                self.target_announced = True            # suppress duplicate "Found it"
                pose = self.robot_pose_map()
                if pose is not None:
                    d0 = math.hypot(seed[0] - pose[0], seed[1] - pose[1])
                    print(f"I remember seeing a {self.target} about {d0:.1f} m away — "
                          f"heading to that spot and I'll confirm it on the way. "
                          f"(type 'cancel' to stop)")
                else:
                    print(f"I remember where a {self.target} was — heading there. "
                          f"(type 'cancel' to stop)")
                self.mlog.log("MEMORY", f"seeded {self.target} from remembered "
                                        f"instance at ({seed[0]:.2f}, {seed[1]:.2f})")
            else:
                print(f"Searching for the {self.target}{extra}. (type 'cancel' to stop)")

        elif action == "locate":                                                    # #7
            if not target:
                self.start_next_step(); return
            self.target = target.lower()
            self.target_qualifier = (step.get("qualifier") or "").lower() or None
            self.locate_start = time.monotonic()
            self.last_yaw = None; self.turn_accum = 0.0
            self.mode = "locate"
            print(f"Looking for a {self.target} (reporting only, not driving). (type 'cancel' to stop)")

        elif action == "move":                                                      # #9
            rot_deg = float(step.get("rotation_deg") or 0.0)
            dist_m  = float(step.get("distance_m") or 0.0)
            if abs(rot_deg) < 0.5 and abs(dist_m) < 0.01:
                print("That move had no distance or angle."); self.start_next_step(); return
            self.move_spec = {
                "phase":   "rotate" if abs(rot_deg) >= 0.5 else "translate",
                "rot_rad": abs(math.radians(rot_deg)),
                "rot_sign": 1 if rot_deg >= 0 else -1,        # + = left / CCW
                "dist_m":  dist_m,                             # + = forward, - = back
            }
            self.move_ref_yaw = None; self.move_ref_pos = None
            self.mode = "move"
            parts = []
            if abs(rot_deg) >= 0.5:
                parts.append(f"turn {abs(rot_deg):.0f} deg {'left' if rot_deg >= 0 else 'right'}")
            if abs(dist_m) >= 0.01:
                parts.append(f"move {'forward' if dist_m > 0 else 'back'} {abs(dist_m):.2f} m")
            print(f"Relative move: {', '.join(parts)}. (type 'cancel' to stop)")

        elif action == "find_another":
            if not target:
                self.start_next_step(); return
            self.target = target.lower()
            self.find_baseline = len(self.seen_instances.get(self.target, []))
            self.find_start = time.monotonic()
            self.search_state = "ROTATE"
            self.turn_accum = 0.0; self.last_yaw = None; self.advance_start = None
            self.mode = "find_more"
            print(f"Looking for another {self.target}. (type 'cancel' to stop)")

        elif action in ("explore", "patrol"):
            # #36: the "do you want a live camera feed?" question is gone.
            # It existed because the ONLY way to see the camera used to be an
            # rqt_image_view window the agent spawned on request, so asking was
            # the polite thing to do. Since #34 the operator console subscribes
            # to /vla/annotated/compressed directly and the feed is already on
            # screen before the command is even typed - so the question now
            # interrupts a patrol to offer something the operator can already
            # see. "patrol the area" is an unambiguous instruction; it should
            # start a patrol. Saying "patrol with the feed" still works: that
            # parses as a feed step plus a patrol step, and start_feed() opens
            # the separate window exactly as before.
            self.start_explore(label=action)

        elif action in ("dock", "undock"):                                          # #10
            self.start_dock_action(action)
            return                              # wait for the action to finish

        elif action == "count":
            t = (target or "").lower()
            saved_target = self.target
            self.target = t                      # locate_candidates works on self.target
            cands, _, _ = self.locate_candidates() if t else ([], None, None)
            self.target = saved_target
            n = len(cands)
            if t and cands:
                self.register_instances(t, [(c[0], c[1]) for c in cands])
                plural = t if n == 1 else t + "s"
                # #28: a count without distances made the follow-up question
                # ("at what distance are they?") necessary in the first place.
                print(f"I can see {n} {plural} right now — {self.describe_distances(cands)}.")
                self.mlog.log("COUNT", f"{n} {t}(s): "
                                       + ", ".join(f"{c[2]:.2f}m" for c in cands))
            else:
                # nothing ranged: fall back to a plain box count so the answer
                # is still honest when depth is missing (holes / out of range)
                pair = self.current_pair()
                det = (self.yolo_detect(pair[0]) or []) if pair is not None else []
                nb = sum(1 for d in det if t and t in d["name"].lower())
                if nb:
                    print(f"I can see {nb} '{target}' right now, but I can't get a "
                          f"depth reading on {'it' if nb == 1 else 'them'} from here.")
                else:
                    print(f"I can't see any '{target}' right now.")
            self.start_next_step()

        elif action == "forget":                                            # #24
            t = (target or "").lower().strip()
            if t:
                had = len(self.seen_instances.pop(t, []))
                if self.last_located is not None and self.last_located[0] == t:
                    self.last_located = None
                self.mlog.log("MEMORY", f"forgot {had} remembered {t} position(s)")
                print(f"Done — I've forgotten every {t} position I had remembered "
                      f"({had} of them)." if had
                      else f"I had no remembered {t} positions anyway.")
            else:
                n = sum(len(v) for v in self.seen_instances.values())
                self.seen_instances.clear()
                self.last_located = None
                self.mlog.log("MEMORY", f"cleared ALL remembered positions ({n})")
                print(f"Done — I've cleared my whole spatial memory ({n} position(s)).")
            self.start_next_step()

        elif action == "describe":
            # #28: "what do you see" now answers with distances too, so the
            # operator doesn't have to ask a second question for every object.
            pair = self.current_pair()
            det = (self.yolo_detect(pair[0]) or []) if pair is not None else []
            if not det:
                print("I see: nothing")
            else:
                parts = []
                for d in det:
                    nm = d["name"]
                    bd = self.range_for_box(d["box"], pair[1], pair[0])  # #54, same range nav uses
                    parts.append(f"{nm} ({bd:.1f} m)" if bd is not None
                                 else f"{nm} (distance unclear)")
                print("I see: " + ", ".join(parts))
            self.start_next_step()

        elif action == "feed":
            self.start_feed()
            self.start_next_step()

        else:                                   # answer / reject / clarify -> speech already printed
            self.start_next_step()

    # ── cancel + feed ──
    def cancel_navigation(self):
        self.task_id += 1
        self.mode = None
        self.search_state = None
        self.target = None
        self.target_qualifier = None
        self.queue = []
        self.pending = None
        self.last_obj_xy = None
        self.target_announced = False
        self.nav_goal_xy = None
        self.sight_count = 0                    # #13
        self.seen_live = False                  # #17
        self.goal_fail_count = 0                # #18
        self.best_dist = None                   # #19a
        self.last_progress_t = None             # #19a
        self.goal_tried = []                    # #19c
        self.jump_pending = None                # #21b
        self.goal_obj_xy = None                 # #30a
        self.reset_step_state()                 # #31
        self.navigate_start = None
        self.locate_start = None
        self.move_spec = None
        self.nudge_from = None                  # #29
        self.backup_nudges = 0                  # #29
        self.frontier_navigating = False
        self.frontier_sent_goal = False
        self.last_pos = None; self.last_move_t = None
        if self.goal_handle is not None:
            try: self.goal_handle.cancel_goal_async()
            except Exception: pass
        self.goal_handle = None
        if self.dock_goal_handle is not None:
            try: self.dock_goal_handle.cancel_goal_async()
            except Exception: pass
            self.dock_goal_handle = None
        self.cmd = Twist()
        self.begin_hard_stop()
        # #35: replaces the old three-zero burst. See begin_hard_stop().

    # ── #35: A STOP THAT IS ACTUALLY A STOP ──
    def begin_hard_stop(self):
        """Cancel every Nav2 goal and hold the wheels at zero until odometry
        agrees the robot has stopped."""
        self.cancel_all_nav_goals()
        # (a) tell Nav2 to abandon everything, so it stops publishing at all
        self.stop_until = time.monotonic() + STOP_BARRAGE_S
        # (b) how long to keep overwriting /cmd_vel with zeros
        self.stop_started = time.monotonic()
        # when this stop began, for the timeout
        self.stop_still_ticks = 0
        # reset the "has it actually stopped" counter
        self.stop_reported = False
        # (c) the outcome has not been announced yet - do NOT claim success now
        self.mlog.log("STOP", "hard stop requested (cancel-all + zero barrage)")

    def cancel_all_nav_goals(self):
        """Ask the navigate_to_pose server to cancel EVERY goal it holds.

        A default CancelGoal request carries a zero goal-id and a zero
        timestamp, which the ROS 2 action spec defines as 'cancel all goals'.
        This is stronger than cancelling self.goal_handle, because that handle
        is None during the window between sending a goal and the acceptance
        callback arriving - and a stop pressed inside that window used to do
        nothing at all."""
        try:
            if not self.nav_cancel_cli.service_is_ready():
                # Nav2 not up (or still starting) - nothing to cancel, and the
                # zero barrage alone is then sufficient
                return
            self.nav_cancel_cli.call_async(CancelGoal.Request())
            # fire it asynchronously: we must not block the ROS executor here,
            # and the barrage covers us until it takes effect
        except Exception as e:
            self.mlog.log("WARN", f"#35 cancel-all failed: {e}")

    def stop_barrage(self):
        """50 Hz, in two phases.

        PHASE 1 (while the barrage runs): hold /cmd_vel at zero.
        PHASE 2 (after it ends): STOP publishing and watch. If Nav2 ignored
        the cancel it regains the topic here and the robot drives off again.

        The phases must not overlap. Judging the stop DURING the barrage is
        circular - the robot is only still because we are forcing it still, so
        it would always look like a success even when the cancel failed."""
        if self.stop_started is None:
            return
            # no stop in progress - this is the idle path, and it is free
        now = time.monotonic()
        if now < self.stop_until:
            try:
                self.cmd_pub.publish(Twist())
                # an all-zero Twist. Publishing faster than Nav2 means our zero
                # is the last word the base hears, instead of being overwritten
            except Exception:
                pass
            self.stop_still_ticks = 0
            # deliberately do NOT judge yet: see the docstring
            return
        if self.stop_reported:
            self.stop_started = None
            # outcome announced and the barrage is over - go fully idle
            return
        moving = (self.odom_lin > STOP_LIN_EPS or self.odom_ang > STOP_ANG_EPS)
        # ask the ROBOT whether it is moving, not our own intentions
        if moving:
            self.stop_still_ticks = 0
            # one moving reading resets the count - we want sustained stillness
        else:
            self.stop_still_ticks += 1
        if self.stop_still_ticks >= STOP_CONFIRM_TICKS:
            self.stop_reported = True
            self.notify("Stopped. The robot is idle.")
            # only NOW is it honest to say this: the barrage is off, nothing is
            # forcing the wheels, and the robot is still not moving
            self.mlog.log("STOP", "confirmed stopped by odometry after barrage")
        elif now - self.stop_started > STOP_CONFIRM_TIMEOUT:
            self.stop_reported = True
            self.notify(f"I sent the stop but the robot is STILL MOVING "
                        f"({self.odom_lin:.2f} m/s, {self.odom_ang:.2f} rad/s). "
                        f"Take manual override now.")
            # a stop that failed must SAY it failed - silence here would be the
            # same class of failure as the bug this fix replaces
            self.mlog.log("ERROR", f"#35 robot still moving {STOP_CONFIRM_TIMEOUT}s "
                                   f"after stop: lin={self.odom_lin:.2f} "
                                   f"ang={self.odom_ang:.2f}")

    # ── #43: robust action-server wait ──────────────────────────────
    #    On hardware the Create 3's action servers are discovered over
    #    Wi-Fi through the FastDDS discovery server, and that can take far
    #    longer than the 3 s v13 allowed. The result was an INTERMITTENT
    #    "the undock action server isn't up" — it worked twice, then failed
    #    four times in a row, with the server present in `ros2 action list`
    #    the whole time. Two changes: wait much longer, and remember that a
    #    server was found so later calls do not pay the wait again.
    def wait_action_server(self, client, key, timeout_sec=15.0):
        """True if the action server is available. Caches success by key."""
        if client is None:
            return False
            # no client was ever constructed (irobot_create_msgs missing)
        if self._server_seen.get(key):
            # already discovered earlier in this session
            if client.server_is_ready():
                return True
                # still there — skip the expensive wait entirely
            # it vanished: fall through and wait again rather than trusting the cache
        ok = client.wait_for_server(timeout_sec=timeout_sec)
        # block up to timeout_sec for DDS discovery to complete
        if ok:
            self._server_seen[key] = True
            # remember it so the next dock/undock is instant
        return ok

    # ── #10 dock / undock as a managed action ──
    def start_dock_action(self, which):
        if not HAVE_CREATE or self.dock_client is None:
            self.notify("Docking isn't available here (irobot_create_msgs not found). "
                        "On the real Create 3 / TurtleBot 4 this will work.")
            self.start_next_step(); return
        client = self.dock_client if which == "dock" else self.undock_client
        if not self.wait_action_server(client, which):
            # #43: 15 s of discovery time, not 3
            self.undock_fail_count += 1
            # #44: count consecutive failures so we can break the retry loop
            self.notify(f"The '{which}' action server isn't up (is the Create 3 running?). "
                        f"Check: ros2 action list | grep -i dock")
            if which == "undock" and self.undock_fail_count >= UNDOCK_FAIL_LIMIT:
                # #44 RETRY LOOP: the auto-undock branch re-inserts the pending
                # step into the queue before calling this, so on failure
                # start_next_step() pops that same step, sees is_docked still
                # true, re-inserts it and undocks again — forever. v13 printed
                # the same two lines over and over until Ctrl+C. After a few
                # honest attempts we abandon the task instead.
                self.queue.clear()
                # drop the pending step so it cannot be popped again
                self.undock_fail_count = 0
                # reset for the next command the user types
                self.notify("I could not undock after several tries, so I've "
                            "cancelled the task. Undock manually, or check that "
                            "the Create 3 is awake, then ask me again.")
                self.mode = "idle"
                # return to idle rather than leaving a half-started task
                return
            self.start_next_step(); return
        self.undock_fail_count = 0
        # #44: a successful server lookup clears the failure ratchet
        self.mode = "dock"
        self.notify("Docking…" if which == "dock" else "Undocking…")
        goal = Dock.Goal() if which == "dock" else Undock.Goal()
        fut = client.send_goal_async(goal)
        fut.add_done_callback(lambda f: self._dock_accepted(f, which))

    def _dock_accepted(self, future, which):
        try:
            gh = future.result()
        except Exception as e:
            self.notify(f"Could not send the {which} request: {e}"); self._after_dock(); return
        if not gh.accepted:
            self.notify(f"The robot rejected the {which} request."); self._after_dock(); return
        self.dock_goal_handle = gh
        gh.get_result_async().add_done_callback(lambda f: self._dock_done(f, which))

    def _dock_done(self, future, which):
        self.dock_goal_handle = None
        # #16: check the REAL outcome — the old code assumed success, so a
        # failed dock left is_docked wrong and later auto-undock logic confused.
        ok = True
        try:
            ok = (future.result().status == GoalStatus.STATUS_SUCCEEDED)
        except Exception:
            ok = False
        if ok:
            self.is_docked = (which == "dock")
            # #22: a successful DOCK puts us exactly ON the dock — the best
            # possible fix of its position (always refresh). After an UNDOCK
            # we're ~0.5 m off it — good enough only if we know nothing yet.
            xy = self.robot_xy()
            if xy is not None and (which == "dock" or self.dock_xy is None):
                self.set_dock_position(*xy)
            self.notify("Docked successfully." if which == "dock" else "Undocked — ready to move.")
        else:
            self.notify(f"The {which} action did not complete — please check the robot. "
                        "(dock_status will keep me updated.)")
        self.mlog.log("DOCK", f"{which} {'ok' if ok else 'FAILED'}")
        self._after_dock()

    def _after_dock(self):
        self.mode = None
        self.start_next_step()                  # continue the queue (e.g. the nav we deferred)

    def is_visited(self, x, y):
        return any(math.hypot(x - vx, y - vy) < VISITED_RADIUS for vx, vy in self.visited_goals)

    def find_frontiers(self):
        m = self.map
        w, h = m.info.width, m.info.height
        res = m.info.resolution
        ox = m.info.origin.position.x; oy = m.info.origin.position.y
        grid = np.array(m.data, dtype=np.int16).reshape((h, w))
        free = (grid == 0); unknown = (grid == -1)
        adj = np.zeros_like(unknown)
        adj[1:, :]  |= unknown[:-1, :]
        adj[:-1, :] |= unknown[1:, :]
        adj[:, 1:]  |= unknown[:, :-1]
        adj[:, :-1] |= unknown[:, 1:]
        frontier = free & adj
        ys, xs = np.where(frontier)
        if len(xs) == 0: return []
        bins = {}
        for cy, cx in zip(ys.tolist(), xs.tolist()):
            wx = ox + (cx + 0.5) * res; wy = oy + (cy + 0.5) * res
            key = (round(wx / BIN_SIZE), round(wy / BIN_SIZE))
            bins.setdefault(key, []).append((wx, wy))
        clusters = []
        for pts in bins.values():
            if len(pts) >= MIN_FRONTIER:
                cx = sum(p[0] for p in pts) / len(pts)
                cy = sum(p[1] for p in pts) / len(pts)
                clusters.append((cx, cy, len(pts)))
        return clusters

    def is_blacklisted(self, x, y):
        return any(math.hypot(x - bx, y - by) < BLACKLIST_RADIUS
                   for bx, by in self.blacklist)

    # ── NAV2 plumbing ──
    def send_goal(self, x, y, yaw):
        if not self.nav_client.server_is_ready():
            return
        tid = self.task_id
        self.mlog.log("GOAL", f"({x:.2f}, {y:.2f}) yaw={math.degrees(yaw):.0f}deg "
                              f"mode={self.mode}")                        # #14
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = MAP_FRAME
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
        fut = self.nav_client.send_goal_async(goal)
        fut.add_done_callback(lambda f: self.on_goal_response(f, tid))

    def on_goal_response(self, future, tid):
        gh = future.result()
        if tid != self.task_id:
            try: gh.cancel_goal_async()
            except Exception: pass
            return
        if not gh.accepted:
            if self.mode == "explore":
                self.blacklist.append(self.cur_goal); self.frontier_navigating = False
            elif self.mode in ("navigate", "find_more"):
                # #19c: remember WHICH goal was refused so the next think-cycle
                # picks a different spot on the ring instead of re-sending the
                # same illegal goal until the task dies.
                if self.nav_goal_xy is not None:
                    self.goal_tried.append(self.nav_goal_xy)
                # #31: a REFUSED hop is not a failed task — it means this
                # particular 1.5 m step ended somewhere Nav2 dislikes. Halve
                # it and let the next think-cycle re-aim; step_len is restored
                # by reset_step_state() when the task ends.
                if self.step_active:
                    self.step_len = max(STEP_GOAL_MIN, self.step_len * 0.5)
                    self.mlog.log("STEP", f"#31 hop refused — shortening to "
                                          f"{self.step_len:.2f} m")
                self.search_state = "ROTATE"; self.last_yaw = None; self.turn_accum = 0.0
                self.nav_goal_xy = None
                self.goal_fail_count += 1                      # #18
                limit = STEP_FAIL_LIMIT if self.step_active else GOAL_FAIL_LIMIT   # #31
                if self.mode == "navigate" and self.goal_fail_count >= limit:
                    self.mlog.log("WARN", "nav goal rejected repeatedly — "
                                          "dropping sighting, rescanning")
                    self.last_obj_xy = None
                    self.target_announced = False
                    self.sight_count = 0
                    self.goal_fail_count = 0
                    self.goal_tried = []
                    self.reset_step_state()                    # #31
            return
        self.goal_handle = gh
        gh.get_result_async().add_done_callback(lambda f: self.on_result(f, tid))

    def on_result(self, future, tid):
        if tid != self.task_id:
            return
        status = future.result().status
        self.mlog.log("RESULT", f"nav goal status={status} "
                                f"({'ok' if status == GoalStatus.STATUS_SUCCEEDED else 'not-succeeded'})")
        if self.mode == "explore":
            if status == GoalStatus.STATUS_SUCCEEDED:      # was the magic number 4
                self.consecutive_stucks = 0
                self.visited_goals.append(self.cur_goal)   # #4: remember we covered it
            else:
                self.blacklist.append(self.cur_goal)
            self.frontier_navigating = False
        elif self.mode in ("navigate", "find_more"):
            self.search_state = "ROTATE"; self.last_yaw = None; self.turn_accum = 0.0
            if status != GoalStatus.STATUS_SUCCEEDED:          # #18
                # #19c: Nav2 tried and gave up on THIS spot (no valid path, or
                # its recoveries failed). Rule that spot out and let the next
                # cycle approach the object from another side.
                if self.nav_goal_xy is not None:
                    self.goal_tried.append(self.nav_goal_xy)
                    self.mlog.log("WARN", f"nav goal ({self.nav_goal_xy[0]:.2f}, "
                                          f"{self.nav_goal_xy[1]:.2f}) failed (status={status}); "
                                          f"trying another approach angle")
                # #31: an ABORTED hop usually means the unknown space it aimed
                # into turned out to contain something. Shorten and re-aim —
                # that is normal, expected progress, not a failure of the task.
                if self.step_active:
                    self.step_len = max(STEP_GOAL_MIN, self.step_len * 0.5)
                    self.mlog.log("STEP", f"#31 hop aborted — shortening to "
                                          f"{self.step_len:.2f} m and re-aiming")
                self.goal_fail_count += 1
                limit = STEP_FAIL_LIMIT if self.step_active else GOAL_FAIL_LIMIT   # #31
                if self.mode == "navigate" and self.goal_fail_count >= limit:
                    self.mlog.log("WARN", "every approach to the target failed — "
                                          "dropping sighting, rescanning")
                    self.last_obj_xy = None
                    self.target_announced = False
                    self.sight_count = 0
                    self.goal_fail_count = 0
                    self.goal_tried = []
                    self.reset_step_state()                    # #31
            else:
                self.goal_fail_count = 0
                # #31: a hop COMPLETED. The map is now bigger than it was when
                # this hop was planned, so restore the full step length and let
                # the next think-cycle re-range the target and decide again
                # (it may well find a full approach goal this time).
                if self.step_active:
                    self.step_len = STEP_GOAL_DIST
                    self.mlog.log("STEP", f"#31 hop {self.step_count} complete — "
                                          f"re-ranging {self.target}")
            self.nav_goal_xy = None
        self.goal_handle = None
