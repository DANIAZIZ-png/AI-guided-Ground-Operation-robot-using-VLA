"""The state machine: command intake, dispatch, the per-task think steps and their endings.

Methods moved verbatim out of VLAAgent in Phase 2 of the reorg. No logic
changed; the comments are the originals. Mixed into VLAAgent in
vla_agent/agent.py, ahead of rclpy's Node, so method resolution is unchanged.
"""
from .core import *            # noqa: F401,F403 - constants and ROS imports
from .core import print        # noqa: A001 - headless-safe console, fix #63
# `import *` deliberately skips underscore-prefixed names, so the ones the
# moved code needs are imported explicitly. Omitting them imports cleanly
# and then fails at runtime.
from .core import _real_print   # noqa: F401


class StateMachineMixin:

    # ── #33: COMMAND INTAKE (any source) ──
    def enqueue_command(self, text, source="keyboard"):
        """The ONE door into the agent. The keyboard loop, /vla/command from
        the GUI and /vla/command from the voice node all come through here, so
        there is exactly one command path in the program and no way for two
        sources to race each other into inconsistent state.

        Returns True if the agent should shut down.

        cancel/stop is executed HERE, on the calling thread, instead of being
        queued — a stop that waits in line behind a three-second LLM call is
        not a stop. It also DRAINS the queue, because anything the operator
        asked for before hitting cancel is, by definition, no longer wanted."""
        cmd = (text or "").strip()
        if not cmd:
            return False
        stripped = strip_politeness(cmd.lower())

        # 'quit' is keyboard-only on purpose. Shutting the agent down is
        # unrecoverable, and a microphone in a noisy hangar must never be able
        # to trigger it. (Same reasoning as #25 keeping quit out of fuzzy
        # matching.)
        if stripped.startswith(QUIT_WORDS):
            if source == "keyboard":
                return True
            print(f"(ignoring 'quit' from {source} — the agent is shut down from "
                  f"its own terminal.)")
            return False

        # #60: manual override is a SYSTEM function, not a task to reason
        # about. It is matched here, ahead of the LLM, for the same reason
        # 'stop' is: sending it to the brain got back "not supported", because
        # a language model has no way to know the agent can yield control.
        # Keyboard only — a voice command or a GUI click cannot put the robot
        # into a mode that needs a keyboard to get out of.
        if any(stripped.startswith(w) for w in MANUAL_WORDS):
            if source != "keyboard":
                print(f"(ignoring manual override from {source} — it has to be "
                      f"taken from the agent's own terminal.)")
                return False
            self.mlog.log("CMD", f"(manual override via {source}) {cmd}")
            self.manual_requested = True
            return False

        # #1 + #25: cancel/stop preempts EVERYTHING, even with politeness
        # wrappers — and now even with a typo. ("cancle" used to reach the
        # LLM and come back as a reject.)
        hit = None
        if stripped.startswith(STOP_WORDS):
            hit = next(w for w in STOP_WORDS if stripped.startswith(w))
        else:
            hit = fuzzy_word(first_word(stripped), STOP_WORDS)
        if hit is not None:
            typed = first_word(stripped)
            if typed != hit:
                print(f"(reading '{typed}' as '{hit}')")
                self.mlog.log("FUZZY", f"'{typed}' -> '{hit}'")
            self.mlog.log("CMD", f"(cancel via {source}) {cmd}")
            dropped = 0
            while True:
                try:
                    self.cmd_queue.get_nowait(); dropped += 1
                except queue.Empty:
                    break
            self.cancel_task()
            extra = f" ({dropped} queued command(s) dropped.)" if dropped else ""
            print(f"Stopping...{extra}")
            # #35: was "Task cancelled. Robot is idle." - which was printed the
            # instant the REQUEST was sent, while the robot was still driving at
            # 0.26 m/s. The confirmation now comes from stop_barrage(), and only
            # once odometry agrees.
            return False

        self.cmd_queue.put((cmd, source))
        return False

    def command_worker(self):
        """#33: drains the intake queue on its own thread.

        Everything slow lives here — the YOLO look-around in handle_command()
        and the Ollama call inside decide(). Before this, those ran on
        whichever thread happened to read the keyboard; now they are off the
        ROS executor entirely, so a thinking robot is still a robot that
        publishes velocity, streams video and answers /vla/status."""
        while not self.shutdown:
            try:
                cmd, source = self.cmd_queue.get(timeout=0.25)
            except queue.Empty:
                continue
            try:
                self.last_source = source
                self.process_command(cmd, source)
            except Exception as e:
                self.mlog.log("ERROR", f"command worker failed on {cmd!r}: {e}")
                print(f"Something went wrong handling that command: {e}")

    def process_command(self, cmd, source="keyboard"):
        """The old body of the input loop, minus the parts #33 moved to
        enqueue_command(). Unchanged in behaviour."""
        stripped = strip_politeness(cmd.lower())

        # a pending yes/no question (e.g. patrol feed prompt)
        if self.pending is not None:
            self.resolve_pending(cmd); return

        # #26: a bare "yes"/"no" with NO question open means nothing. It
        # used to go to the brain, which had nothing to attach it to and
        # invented a task from stale context ("yes" -> "checking if the
        # shelf is still there"). Answer honestly instead.
        bare = first_word(stripped)
        if (len(stripped.split()) == 1
                and (bare in YES_WORDS or bare in NO_WORDS
                     or fuzzy_word(bare, YES_WORDS + NO_WORDS) is not None)):
            print("I'm not waiting on a yes/no right now — tell me what "
                  "you'd like me to do (e.g. 'go to the chair', 'patrol', 'dock').")
            self.mlog.log("CMD", f"(bare confirmation ignored) {cmd}")
            return

        self.mlog.log("CMD", f"[{source}] {cmd}")                     # #14/#33

        # #10: handle obvious dock/undock deterministically (no camera/LLM needed)
        di = self.dock_intent(stripped)
        if di is not None:
            self.cancel_navigation()
            self.queue = [{"action": di, "target": None,
                           "speech": "Undocking." if di == "undock" else "Returning to the dock."}]
            self.has_feed_step = False
            self.start_next_step()
            return

        self.cancel_navigation()
        self.handle_command(cmd)

    # ── INPUT LOOP (main thread) ──
    # ── #60: hand the wheel to the operator ────────────────────────
    def manual_override(self):
        """Raw-keyboard teleoperation inside the agent's own terminal.

        Runs on the input thread, so the ROS executor keeps spinning
        underneath: odometry, TF, the camera feed and the status topic all
        stay live while the operator drives. Returns when the operator
        presses 'q' (or Ctrl+C), handing control back to the autonomy."""
        import termios, tty, select        # POSIX-only; imported here so the
                                           # agent still starts on a system
                                           # without them (headless/GUI mode)
        if not sys.stdin or not sys.stdin.isatty():
            self.notify("Manual override needs a real terminal — "
                        "run the agent from a console, not the GUI.")
            return

        self.cancel_task()                 # autonomy lets go before we take over
        self.manual_mode = True            # #59: mutes the collision guard
        self.mlog.log("MANUAL", "operator took manual control")
        print("\n" + "=" * 62)
        print(" MANUAL OVERRIDE — you are driving.")
        print("     u  i  o      u/o = forward + turn      i = forward")
        print("     j  k  l      j/l = turn on the spot    k = stop")
        print("     m  ,  .      m/. = reverse + turn      , = reverse")
        print("     +/-  faster / slower        q  = hand back to the robot")
        print(f" Speed {MANUAL_LIN_MAX:.2f} m/s max. Release the key and the "
              f"robot stops after {MANUAL_HOLD_S:.2f}s.")
        print("=" * 62)

        lin = min(0.15, MANUAL_LIN_MAX)    # start gently, not at full speed
        ang = min(0.50, MANUAL_ANG_MAX)
        last_key_t = 0.0
        fd = sys.stdin.fileno()
        old_term = termios.tcgetattr(fd)   # remember the terminal's settings
        try:
            tty.setcbreak(fd)              # read single keys without ENTER
            while not self.shutdown:
                # select() with a timeout is what makes this a dead-man switch:
                # if no key arrives we fall through and publish a stop.
                ready, _, _ = select.select([sys.stdin], [], [], 0.05)
                if ready:
                    ch = sys.stdin.read(1)
                    if ch == "q" or ch == "\x03":       # q or Ctrl+C
                        break
                    if ch in ("+", "="):
                        lin = min(MANUAL_LIN_MAX, lin + MANUAL_LIN_STEP)
                        ang = min(MANUAL_ANG_MAX, ang + MANUAL_ANG_STEP)
                        print(f"\r speed {lin:.2f} m/s  {ang:.2f} rad/s   ", end="", flush=True)
                        continue
                    if ch in ("-", "_"):
                        lin = max(0.05, lin - MANUAL_LIN_STEP)
                        ang = max(0.10, ang - MANUAL_ANG_STEP)
                        print(f"\r speed {lin:.2f} m/s  {ang:.2f} rad/s   ", end="", flush=True)
                        continue
                    if ch in MANUAL_KEYS:
                        fwd, turn = MANUAL_KEYS[ch]
                        t = Twist()
                        t.linear.x  = fwd  * lin
                        t.angular.z = turn * ang
                        self.cmd_pub.publish(t)
                        last_key_t = time.monotonic()
                        continue
                # no key, or a key we don't recognise: has the hold expired?
                if last_key_t and time.monotonic() - last_key_t > MANUAL_HOLD_S:
                    self.cmd_pub.publish(Twist())      # dead-man stop
                    last_key_t = 0.0
        except Exception as e:
            self.mlog.log("ERROR", f"manual override: {e}")
        finally:
            # Whatever happened, put the terminal back the way we found it and
            # make certain the wheels are stopped before autonomy resumes.
            termios.tcsetattr(fd, termios.TCSADRAIN, old_term)
            for _ in range(5):
                try: self.cmd_pub.publish(Twist())
                except Exception: pass
                time.sleep(0.02)
            self.manual_mode = False
            self.mlog.log("MANUAL", "control handed back to the robot")
            print("\n[robot] Manual override ended — I have control again.")

    def run_input_loop(self):
        print(f"[{AGENT_VERSION}]")
        print("VLA agent ready. 'cancel'/'stop' stops a task, 'quit' exits.")
        print("'manual override' hands you the wheel (drive with u i o j k l m , . "
              "— press q to give control back).")
        print("Try: go to the nearest person | find a chair | move forward 2 m | "
              "turn right 30 | patrol the area | dock | undock | "
              "go to the person then to the chair")
        print(f"Also listening on {CMD_IN_TOPIC} (GUI / voice). Everything I say "
              f"goes out on {REPLY_TOPIC}, state on {STATUS_TOPIC}.\n")
        self.cmd_worker = threading.Thread(target=self.command_worker, daemon=True)
        self.cmd_worker.start()

        # #33 HEADLESS MODE. When the GUI (or a launch file, or systemd) starts
        # the agent there is no terminal attached, so input() raises EOFError
        # on the very first call. The old loop treated that as "operator typed
        # quit" and shut the agent down a fraction of a second after it
        # started — the robot would appear to launch and instantly die. With
        # no keyboard we simply do not read one: the agent stays up and is
        # driven entirely through /vla/command, which is exactly what the GUI
        # and the voice node use anyway.
        if not sys.stdin or not sys.stdin.isatty():
            print("No terminal attached — running HEADLESS. "
                  f"Send commands on {CMD_IN_TOPIC}; Ctrl-C or a SIGTERM to stop.")
            self.mlog.log("RUN", "headless mode (no tty)")
            try:
                while not self.shutdown:
                    time.sleep(0.25)
            except KeyboardInterrupt:
                pass
            self.shutdown = True
            return

        while not self.shutdown:
            try:
                cmd = input("Command> ").strip()
            except (EOFError, KeyboardInterrupt):
                break
            if not cmd:
                continue
            if self.enqueue_command(cmd, source="keyboard"):
                break
            # #60: enqueue_command only RAISES the flag; the actual driving
            # happens here, on the thread that owns the terminal. Doing it in
            # the worker thread would have two threads reading one keyboard.
            if self.manual_requested:
                self.manual_requested = False
                self.manual_override()
        self.shutdown = True

    def dock_intent(self, low):
        """Return 'dock'/'undock' for clear one-word docking commands, else None."""
        if low in ("undock", "un dock", "leave the dock", "leave dock",
                   "move off the dock", "get off the dock", "off the dock"):
            return "undock"
        if low in ("dock", "go dock", "dock now", "return to dock", "return to the dock",
                   "go to the dock", "go to dock", "return to base", "go home",
                   "go to base", "charge", "go charge", "go to the charging station"):
            return "dock"
        return None

    def handle_command(self, cmd):
        # Camera is only needed for vision tasks; the brain still runs without it
        # so move/dock/explore work before the camera is up.
        visible = []
        pair = self.current_pair()
        if pair is not None:
            detections = self.yolo_detect(pair[0])
            if detections:
                visible = sorted({d["name"] for d in detections})
        print(f"(I see: {visible if visible else 'nothing'})  thinking...")
        try:
            # #11: pass the previous action/target so "go to IT" resolves.
            # #27: ...but only while it's FRESH. Stale context is how a target
            # from many commands ago gets resurrected into a nonsense task.
            if self.ctx_last_target and time.monotonic() - self.ctx_time > CONTEXT_TTL:
                self.mlog.log("CTX", f"dropping stale context "
                                     f"({self.ctx_last_action} -> {self.ctx_last_target})")
                self.ctx_last_action = None
                self.ctx_last_target = None
            context = {"last_action": self.ctx_last_action,
                       "last_target": self.ctx_last_target}
            decision = decide(cmd, visible, context=context)
        except Exception as e:
            self.mlog.log("ERROR", f"brain failed: {e}")
            print(f"Brain error (is Ollama running?): {e}"); return
        self.mlog.log("BRAIN", f"{decision}")                             # #14
        steps = decision.get("steps")
        if not steps:
            steps = [decision] if decision.get("action") else []
        if not steps:
            print("I didn't catch a clear command."); return
        self.queue = list(steps)
        self.has_feed_step = any((s.get("action") or "").lower() == "feed" for s in steps)
        self.start_next_step()

    # ── start frontier explore/patrol ──
    def start_explore(self, label="explore"):
        self.blacklist = []
        self.visited_goals = []                 # #4: fresh coverage memory each patrol
        self.frontier_navigating = False
        self.frontier_sent_goal = False
        self.explore_start_t = time.monotonic()
        self.warned_no_map = self.warned_no_frontier = False
        self.consecutive_stucks = 0
        self.mode = "explore"
        print(f"Starting to {label} and map the area. (type 'cancel' to stop)")

    # ── handle the answer to a pending yes/no question ──
    def resolve_pending(self, cmd):
        p = self.pending
        self.pending = None
        if p["type"] == "feed_confirm":
            if is_yes(cmd):                      # #25: tolerant of 'yse', 'yeh'
                self.start_feed()
            else:
                print("OK, patrolling without the live feed.")
            self.start_explore(label="patrol")

    def cancel_task(self):
        self.cancel_navigation()
        self.stop_feed()

    # ── slow brain ──
    def think(self):
        self.check_stuck()
        if self.mode == "navigate":
            self.navigate_think()
        elif self.mode == "locate":             # #7
            self.locate_think()
        elif self.mode == "move":               # #9
            self.move_think()
        elif self.mode == "find_more":
            self.find_more_think()
        elif self.mode == "explore":
            self.explore_think()

    # ── NAVIGATE: drive to the target, pathing around obstacles ──
    def navigate_think(self):
        if not self.camera_ready() or not self.have_odom:
            return

        now = time.monotonic()
        pose = self.robot_pose_map()

        # #19a: two DIFFERENT clocks, because "I can't find it" and "I can't
        # reach it" are different failures.
        #   - not committed yet  -> plain SEARCH_TIMEOUT.
        #   - committed to a target -> no deadline at all while we keep
        #     closing the gap; only quit after NO_PROGRESS_LIMIT seconds of
        #     getting no nearer. The old flat 60 s killed Nav2 mid-drive on
        #     anything far away, which is what looked like "it can't avoid
        #     the obstacle".
        if self.last_obj_xy is None:
            if self.navigate_start and now - self.navigate_start > SEARCH_TIMEOUT:
                self.stop_base()
                if self.seen_live:
                    self.notify(f"I saw the {self.target} but lost track of it and "
                                f"couldn't find it again. Try asking me once more.")
                else:
                    self.notify(f"Sorry, I couldn't find any {self.target} in this area.")
                self.end_navigate(); return
        elif pose is not None:
            d_now = math.hypot(pose[0] - self.last_obj_xy[0], pose[1] - self.last_obj_xy[1])
            if self.best_dist is None or d_now < self.best_dist - PROGRESS_EPS:
                self.best_dist = d_now
                self.last_progress_t = now              # we're getting closer: keep going
            elif self.last_progress_t is None:
                self.last_progress_t = now
            elif now - self.last_progress_t > NO_PROGRESS_LIMIT:
                self.stop_base()
                self.notify(f"I know where the {self.target} is but I can't get any "
                            f"closer — the way looks blocked. I've remembered its spot, "
                            f"so try again from somewhere else, or clear the path.")
                self.mlog.log("WARN", f"no progress toward {self.target} for "
                                      f"{NO_PROGRESS_LIMIT:.0f}s; best={self.best_dist:.2f}m")
                self.end_navigate(); return

        found = self.locate_target()

        # #13: require SIGHT_CONFIRM sightings before committing to the target,
        # so one hallucinated frame doesn't send the robot chasing a ghost.
        # NEW: HOLD STILL between confirmation looks (the old code kept the
        # scan spinning, so look #2 was often past the object and confirmation
        # needed a whole extra revolution). A lost candidate resets the count.
        if found is not None:
            ox, oy, rx, ry, dist = found
            if self.last_obj_xy is None and self.sight_count + 1 < SIGHT_CONFIRM:
                self.sight_count += 1
                self.cmd = Twist()               # freeze; re-check next think-cycle
                return
            announced_before = self.target_announced
            first_live = not self.seen_live
            # #21b: a big unconfirmed jump must not touch our state at all —
            # keep driving to the CURRENT belief and wait for a second look.
            if not self.accept_sighting(ox, oy, dist):        # #30b
                pass
            else:
                self.seen_live = True
                self.goal_fail_count = 0         # #18: a fresh sighting resets failures
                # #19a: if the target itself has MOVED, the old "closest we ever
                # got" is meaningless — restart the progress clock so a walking
                # person (or a re-projected chair) can't trigger a false give-up.
                if (self.last_obj_xy is not None
                        and math.hypot(ox - self.last_obj_xy[0], oy - self.last_obj_xy[1]) > 2 * PROGRESS_EPS):
                    self.best_dist = None
                    self.last_progress_t = time.monotonic()
                    self.goal_tried = []         # a moved target deserves fresh angles
                self.last_obj_xy = (ox, oy)
                self.register_instances(self.target, [(ox, oy)])   # #17: keep memory fresh
                if ARRIVE_IF_SEEN_M > 0 and dist <= ARRIVE_IF_SEEN_M:   # #64e: seen and close = arrived
                    self.stop_base()
                    self.notify(f"Arrived at the {self.target}.")
                    self.mlog.log("ARRIVE", f"#64e {self.target} in frame at {dist:.2f} m "
                                            f"(<= {ARRIVE_IF_SEEN_M} m) -- stopping here")
                    self.end_navigate(); return
                if not self.target_announced:
                    self.notify(f"Found the {self.target} — navigating to it.")
                    self.mlog.log("SIGHT", f"{self.target} at ({ox:.2f}, {oy:.2f}), "
                                           f"{dist:.2f} m away")
                    self.target_announced = True
                elif announced_before and first_live:
                    # we started from MEMORY and the camera has now re-confirmed it
                    self.notify(f"I can see the {self.target} now — it's where I remembered.")
                    self.mlog.log("SIGHT", f"memory-seeded {self.target} re-confirmed at "
                                           f"({ox:.2f}, {oy:.2f}), {dist:.2f} m away")
        elif self.last_obj_xy is None and self.sight_count > 0:
            self.sight_count = 0                 # candidate vanished mid-confirmation

        # arrive by pose (stop even if depth/YOLO drops at close range).
        # #17: be honest — if we only ever knew it from memory and never
        # re-saw it, say so instead of claiming a confirmed arrival.
        if self.last_obj_xy is not None and pose is not None:
            rx, ry, _ = pose
            gap = math.hypot(rx-self.last_obj_xy[0], ry-self.last_obj_xy[1])
            # #59: the LiDAR safety net that used to live here has moved to
            # collision_guard(), on its own 10 Hz timer. Gating it on the
            # believed gap (as v24 did) meant it inherited the very error it
            # was supposed to protect against. What remains here is the plain
            # pose-based arrival test, which is all it should ever have been.
            if gap <= STOP_DISTANCE + ARRIVE_TOL:
                self.stop_base()
                if self.seen_live:
                    self.notify(f"Arrived at the {self.target}.")
                else:
                    self.notify(f"I'm at the spot where I last saw the {self.target}, "
                                f"but I can't see it from here right now — it may have "
                                f"moved. Say 'go to the {self.target}' to search fresh.")
                    # stale memory: drop this instance so a fresh search isn't
                    # poisoned by it again
                    known = self.seen_instances.get(self.target, [])
                    self.seen_instances[self.target] = [
                        p for p in known
                        if math.hypot(p[0]-self.last_obj_xy[0], p[1]-self.last_obj_xy[1]) > 0.5]
                    # #23: don't let "go to it" resurrect the same dead point
                    if (self.last_located is not None
                            and self.last_located[0] == self.target
                            and math.hypot(self.last_located[1][0]-self.last_obj_xy[0],
                                           self.last_located[1][1]-self.last_obj_xy[1]) <= 0.5):
                        self.last_located = None
                self.end_navigate(); return

        # #5/#19c: if we know where it is (seen now OR remembered while
        # occluded), pick a stand-off goal and let Nav2 plan the full path
        # AROUND obstacles. We no longer insist on the single point directly
        # between us and the object — we take the cheapest reachable spot on
        # a ring around it.
        if self.last_obj_xy is not None and pose is not None:
            rx, ry, _ = pose
            ox, oy = self.last_obj_xy
            dx, dy = ox - rx, oy - ry
            dist = math.hypot(dx, dy)
            if dist < 1e-3:
                return

            # ── #31: WHICH REGIME ARE WE IN? ──────────────────────────
            # This single call is the regime test. approach_candidates() only
            # returns points that pass is_goal_free(), i.e. surrounded by
            # KNOWN-free cells. If it returns anything, the object's
            # surroundings are mapped and we can plan the real approach in
            # one shot (unchanged behaviour). If it returns nothing, either
            # the object is beyond the map or it is walled in — and in both
            # cases the answer is the same: get closer, one short hop at a
            # time, and re-decide with better information.
            cands = self.approach_candidates(ox, oy, rx, ry)

            # #31: the moment the destination becomes mapped, stop crawling.
            if self.step_active and cands:
                self.mlog.log("STEP", f"{self.target} area now mapped after "
                                      f"{self.step_count} hop(s) — switching to a "
                                      f"direct approach")
                self.notify(f"The area around the {self.target} is mapped now — "
                            f"going straight to it.")
                self.step_active = False
                self.nav_goal_xy = None          # force a fresh full goal below

            # ── #31: HOP STICKINESS ──
            # A hop is re-planned only when it is finished or invalid. Without
            # this we would re-aim every think-cycle at a point 1.5 m ahead of
            # wherever the robot happens to be — a carrot on a stick that
            # preempts Nav2 once a second and never arrives anywhere. Same
            # disease #30a cured for full goals, different goal type.
            if (self.step_active and self.nav_goal_xy is not None
                    and self.search_state == "NAVIGATING"):
                g0x, g0y = self.nav_goal_xy
                d_goal = math.hypot(rx - g0x, ry - g0y)
                bearing_now = math.atan2(oy - ry, ox - rx)
                off = abs(ang_norm(bearing_now - (self.step_bearing
                                                  if self.step_bearing is not None
                                                  else bearing_now)))
                if (d_goal > STEP_ARRIVE_TOL                       # not there yet
                        and off < math.radians(STEP_REPLAN_BEARING)  # still aimed right
                        and now - self.step_start_t < STEP_TIMEOUT   # not wedged
                        and self.is_step_goal_ok(g0x, g0y)):         # still legal
                    return                       # keep crawling — leave Nav2 alone

            # #30a: STICKY GOAL (FULL goals only). If the goal we are already
            # driving to is still a sane approach to the (jittering) object
            # position, KEEP it and let Nav2 finish. Re-ranking the ring every
            # cycle preempted Nav2 about once a second — that is the dancing
            # goal marker and the "found it, navigating... but stuck" behaviour.
            if (not self.step_active and self.nav_goal_xy is not None
                    and self.search_state == "NAVIGATING"):
                g0x, g0y = self.nav_goal_xy
                d_obj = math.hypot(g0x - ox, g0y - oy)
                drift = (math.hypot(ox - self.goal_obj_xy[0], oy - self.goal_obj_xy[1])
                         if self.goal_obj_xy is not None else 0.0)
                if (GOAL_STICK_MIN <= d_obj <= GOAL_STICK_MAX
                        and drift <= OBJ_DRIFT_REISSUE
                        and not self.is_goal_tried(g0x, g0y)
                        and self.is_goal_free(g0x, g0y)):
                    return                       # goal still good — don't touch it

            safe = None
            hop = None
            if cands:
                safe = cands[0]
            elif self.goal_tried:
                # every ring spot has been refused already — forget the
                # refusals and let Nav2 have another go from where we are now
                self.mlog.log("WARN", f"all approach goals for {self.target} refused; "
                                      f"clearing the tried-list and retrying")
                self.goal_tried = []
                cands = self.approach_candidates(ox, oy, rx, ry)
                safe = cands[0] if cands else None
            if safe is None and self.map is None:
                # no map at all (e.g. Nav2/SLAM not up) — fall back to the
                # old straight-line stand-off rather than refusing to move
                standoff = max(0.0, dist - STOP_DISTANCE)
                safe = (rx + (dx / dist) * standoff, ry + (dy / dist) * standoff)

            # ── #31: NO LEGAL FULL GOAL -> HOP ────────────────────────
            # This is the fix for "the robot won't go to an off-map target".
            # Previously the only remaining option was farthest_free_along(),
            # which by construction stops at the LAST KNOWN-FREE cell — the
            # near side of the frontier. The robot dutifully drove to the edge
            # of its own map, found the same situation there, and after
            # GOAL_FAIL_LIMIT cycles dropped the sighting. A hop is allowed to
            # END in unknown space, which is the whole difference.
            if safe is None and dist <= STEP_MAX_RANGE:
                hop = self.step_goal_toward(ox, oy, rx, ry)
            elif safe is None:
                self.mlog.log("WARN", f"ignoring {self.target} sighting at {dist:.1f} m "
                                      f"(beyond STEP_MAX_RANGE {STEP_MAX_RANGE} m — "
                                      f"depth at that range is not trustworthy)")

            if safe is None and hop is None:
                # #18: last resort — the frontier point on the straight line.
                # Kept because when the hop fan is blocked by REAL obstacles,
                # this can still find a known-free detour end point.
                safe = self.farthest_free_along(rx, ry, ox, oy)

            if safe is None and hop is None:
                # truly nowhere to go from here toward it
                self.cmd = Twist()               # kill any stale scan spin (#18)
                self.goal_fail_count += 1
                limit = STEP_FAIL_LIMIT if self.step_active else GOAL_FAIL_LIMIT
                if self.goal_fail_count >= limit:
                    self.notify(f"I can't find a reachable path toward the "
                                f"{self.target} from here — searching again.")
                    self.mlog.log("WARN", f"dropping unreachable sighting of "
                                          f"{self.target} at ({ox:.2f}, {oy:.2f}) "
                                          f"after {self.goal_fail_count} attempts "
                                          f"(hop_mode={self.step_active})")
                    self.last_obj_xy = None
                    self.nav_goal_xy = None
                    self.goal_fail_count = 0
                    self.target_announced = False
                    self.sight_count = 0
                    self.reset_step_state()      # #31
                    self.search_state = "ROTATE"
                    self.last_yaw = None; self.turn_accum = 0.0
                return

            # ── #31: issue a HOP ──────────────────────────────────────
            if hop is not None:
                gx, gy, bearing = hop
                if not self.step_announced:
                    self.step_announced = True
                    self.notify(f"The {self.target} is about {dist:.1f} m away, past the "
                                f"edge of what I've mapped. Moving up in short steps and "
                                f"mapping as I go.")
                    self.mlog.log("STEP", f"#31 hop mode engaged for {self.target}: "
                                          f"object ({ox:.2f}, {oy:.2f}) at {dist:.1f} m, "
                                          f"area_mapped={self.object_is_mapped(ox, oy)}")
                self.cmd = Twist()
                self.search_state = "NAVIGATING"
                self.step_active = True
                self.step_bearing = bearing
                self.step_start_t = now
                self.step_count += 1
                self.nav_goal_xy = (gx, gy)
                self.goal_obj_xy = (ox, oy)
                hop_len = math.hypot(gx - rx, gy - ry)
                self.mlog.log("GOAL", f"#31 hop {self.step_count}: {hop_len:.2f} m toward "
                                      f"{self.target} (still {dist:.1f} m away)")
                # face along the hop so the camera keeps the target in frame and
                # we can re-range it on arrival
                self.send_goal(gx, gy, bearing)
                return

            # ── full approach goal (mapped destination) ───────────────
            self.step_active = False
            gx, gy = safe
            # #19c: face the OBJECT from the goal, not just our travel
            # direction — matters now that the goal can be on its far side.
            fdx, fdy = ox - gx, oy - gy
            yaw = math.atan2(fdy, fdx) if math.hypot(fdx, fdy) > 1e-3 \
                else math.atan2(dy, dx)
            if (self.nav_goal_xy is None or
                    math.hypot(gx - self.nav_goal_xy[0], gy - self.nav_goal_xy[1]) > GOAL_REISSUE):
                self.cmd = Twist()
                self.search_state = "NAVIGATING"
                self.nav_goal_xy = (gx, gy)
                self.goal_obj_xy = (ox, oy)      # #30a: for drift tracking
                self.send_goal(gx, gy, yaw)
            return

        # never seen it (enough) yet — scan around
        if self.search_state == "NAVIGATING":
            return
        if self.search_state == "ROTATE":
            self.do_rotate_scan()
        elif self.search_state == "ADVANCE":
            self.do_advance()

    # ── LOCATE: look and REPORT only, never drive to it (#7) ──
    def locate_think(self):
        if not self.camera_ready():
            return
        if self.locate_start and time.monotonic() - self.locate_start > LOCATE_TIMEOUT:
            self.cmd = Twist()
            self.notify(f"I don't see any {self.target} in this area.")
            self.end_locate(); return
        found = self.locate_target()
        if found is not None:
            ox, oy, rx, ry, dist = found
            self.cmd = Twist()
            # #28: report EVERY instance in frame, not just the selected one.
            # "I can see 2 chairs" followed by a single distance was the gap.
            cands, _, _ = self.locate_candidates()
            if not cands:                        # vanished between the two looks
                cands = [(ox, oy, dist, 0.0)]
            self.register_instances(self.target, [(c[0], c[1]) for c in cands])
            self.last_located = (self.target, (ox, oy))            # #23: for "go to it"
            self.mlog.log("MEMORY", f"remembered {len(cands)} {self.target}(s) at "
                                    + ", ".join(f"({c[0]:.2f}, {c[1]:.2f}) d={c[2]:.2f}m"
                                                for c in cands))
            n = len(cands)
            plural = self.target if n == 1 else self.target + "s"
            article = f"a {self.target}" if n == 1 else f"{n} {plural}"
            self.notify(f"Yes — I can see {article} {self.describe_distances(cands)}. "
                        "I'll remember where they are. "
                        "(Locating only — not driving to them.)"
                        if n > 1 else
                        f"Yes — I can see {article} {self.describe_distances(cands)}. "
                        "I'll remember where it is. (Locating only — not driving to it.)")
            self.end_locate(); return
        # not visible yet -> turn in place to look around (no driving toward it)
        t = Twist(); t.angular.z = SEARCH_TURN_SPEED
        self.cmd = t

    # ── MOVE: precise relative turn then translate (#9) ──
    def move_think(self):
        if not self.have_odom or self.move_spec is None:
            return
        spec = self.move_spec

        if spec["phase"] == "rotate":
            if self.move_ref_yaw is None:
                self.move_ref_yaw = self.yaw; self.move_turned = 0.0
            dyaw = abs(math.atan2(math.sin(self.yaw - self.move_ref_yaw),
                                  math.cos(self.yaw - self.move_ref_yaw)))
            self.move_turned += dyaw; self.move_ref_yaw = self.yaw
            if spec["rot_rad"] - self.move_turned <= math.radians(2.0):
                self.cmd = Twist()
                spec["phase"] = "translate"; self.move_ref_pos = None
            else:
                t = Twist(); t.angular.z = ROTATE_SPEED * spec["rot_sign"]
                self.cmd = t
            return

        if spec["phase"] == "translate":
            if abs(spec["dist_m"]) < 1e-3:
                self.finish_move(); return
            if self.move_ref_pos is None:
                self.move_ref_pos = (self.x, self.y)
                # #29: lock the heading NOW. Progress is measured ALONG this
                # axis (signed), not as raw displacement — otherwise a forward
                # shuffle would look like backward progress being undone twice.
                self.move_axis = (math.cos(self.yaw), math.sin(self.yaw))
                self.move_moved = 0.0
                self.backup_best = 0.0
                self.backup_stall_t = None
                self.backup_nudges = 0
                self.nudge_from = None

            going_fwd = spec["dist_m"] > 0
            ax, ay = self.move_axis
            signed = ((self.x - self.move_ref_pos[0]) * ax +
                      (self.y - self.move_ref_pos[1]) * ay)
            # progress TOWARD the goal, always positive when going the right way
            self.move_moved = signed if going_fwd else -signed

            if going_fwd and self.front_min < OBSTACLE_STOP:
                self.cmd = Twist()
                self.notify("There's an obstacle ahead — stopping the move early.")
                self.finish_move(); return
            if self.move_moved >= abs(spec["dist_m"]):
                self.cmd = Twist(); self.finish_move(); return

            # ── #29: reverse ratchet ──
            if not going_fwd:
                # (a) currently shuffling forward to clear the firmware limit?
                if self.nudge_from is not None:
                    crept = math.hypot(self.x - self.nudge_from[0],
                                       self.y - self.nudge_from[1])
                    if crept >= BACKUP_NUDGE_DIST:
                        self.nudge_from = None
                        self.backup_stall_t = None
                        self.backup_best = self.move_moved   # restart the stall watch
                    elif self.front_min < OBSTACLE_STOP:
                        # boxed in: can't reverse (firmware) and can't creep
                        # forward (obstacle). Say so instead of grinding.
                        self.cmd = Twist()
                        self.notify(f"I can't reverse any further and there's something "
                                    f"in front of me, so I can't shuffle forward to reset "
                                    f"the limit. I managed {self.move_moved:.2f} m of the "
                                    f"{abs(spec['dist_m']):.2f} m.")
                        self.mlog.log("MOVE", "backup ratchet blocked front and rear")
                        self.finish_move(); return
                    else:
                        t = Twist(); t.linear.x = MOVE_SPEED
                        self.cmd = t
                        return

                # (b) reversing normally — watch for the firmware cutting us off
                now = time.monotonic()
                if self.move_moved > self.backup_best + BACKUP_STALL_EPS:
                    self.backup_best = self.move_moved
                    self.backup_stall_t = now
                elif self.backup_stall_t is None:
                    self.backup_stall_t = now
                elif now - self.backup_stall_t > BACKUP_STALL_TIME:
                    if self.backup_nudges >= MAX_BACKUP_NUDGES:
                        self.cmd = Twist()
                        self.notify(f"I've hit the reverse limit {self.backup_nudges} times "
                                    f"and only made {self.move_moved:.2f} m of the "
                                    f"{abs(spec['dist_m']):.2f} m. Something may be behind "
                                    f"me — please check, or turn me around and drive forward.")
                        self.mlog.log("MOVE", f"backup ratchet gave up after "
                                              f"{self.backup_nudges} nudges at "
                                              f"{self.move_moved:.2f} m")
                        self.finish_move(); return
                    self.backup_nudges += 1
                    self.nudge_from = (self.x, self.y)
                    print(f"(reverse limit reached at {self.move_moved:.2f} m — "
                          f"shuffling {BACKUP_NUDGE_DIST:.2f} m forward to clear it, "
                          f"then continuing back)")
                    self.mlog.log("MOVE", f"backup limit at {self.move_moved:.2f} m; "
                                          f"nudge #{self.backup_nudges} forward "
                                          f"{BACKUP_NUDGE_DIST} m")
                    t = Twist(); t.linear.x = MOVE_SPEED
                    self.cmd = t
                    return

            t = Twist(); t.linear.x = MOVE_SPEED * (1.0 if going_fwd else -1.0)
            self.cmd = t

    def finish_move(self):
        self.cmd = Twist()
        try: self.cmd_pub.publish(Twist())
        except Exception: pass
        # #29: if the ratchet was used, say so — it's real evidence of the
        # platform constraint being handled, not hidden.
        if self.backup_nudges:
            self.notify(f"Done with that move — I made {self.move_moved:.2f} m in "
                        f"reverse using {self.backup_nudges} forward shuffle"
                        f"{'s' if self.backup_nudges > 1 else ''} to clear the "
                        f"Create 3 backup limit.")
        else:
            self.notify("Done with that move.")
        self.backup_nudges = 0
        self.nudge_from = None
        self.task_id += 1
        self.mode = None
        self.move_spec = None
        self.start_next_step()

    # ── FIND ANOTHER: scan around until a NEW instance appears ──
    def find_more_think(self):
        if not self.camera_ready() or not self.have_odom:
            return
        if time.monotonic() - self.find_start > FIND_TIMEOUT:
            self.cmd = Twist()
            total = len(self.seen_instances.get(self.target, []))
            self.notify(f"I couldn't find another {self.target}. I still count {total}. "
                        "Waiting for further instructions.")
            self.end_find_more(); return

        pair = self.current_pair()
        if pair is not None:
            detections = self.yolo_detect(pair[0])
            if detections is not None:
                positions = self.locate_all(self.target, detections, pair[1], pair[0])
                added = self.register_instances(self.target, positions)
                total = len(self.seen_instances.get(self.target, []))
                if added > 0 and total > self.find_baseline:
                    self.cmd = Twist()
                    plural = self.target if total == 1 else self.target + "s"
                    self.notify(f"I found another {self.target}. I now count {total} {plural}. "
                                "Waiting for further instructions.")
                    self.end_find_more(); return

        if self.search_state == "ROTATE":
            self.do_rotate_scan()
        elif self.search_state == "ADVANCE":
            self.do_advance()

    def do_rotate_scan(self):
        # ── #56 STEP-AND-STARE ──────────────────────────────────────
        # A continuous spin blurs every frame and, at one think-cycle per
        # second, skips 28.6 deg of a 64.6 deg field of view between looks.
        # So instead: turn ~25 deg, STOP DEAD, hold still for 1.6 s while the
        # camera and YOLO get a clean look, then turn again. Detection itself
        # is not done here -- it happens in the caller every think-cycle --
        # this method only decides whether the wheels should be moving.
        now = time.monotonic()

        # PHASE 1: are we currently holding still for a look?
        if self.scan_dwell_until is not None:
            if now < self.scan_dwell_until:
                # Publish an all-zero Twist so the base is genuinely stopped,
                # not merely un-commanded (an un-commanded base coasts).
                self.cmd = Twist()
                return
            # dwell finished -- begin the next turning step from zero
            self.scan_dwell_until = None
            self.scan_step_yaw = 0.0
            self.last_yaw = None

        # PHASE 2: turning. Measure how far we actually rotated since the
        # last look, using the odometry yaw rather than assuming the commanded
        # speed was achieved (carpet, low battery and wheel slip all make the
        # real turn rate differ from the commanded one).
        if self.last_yaw is None:
            self.last_yaw = self.yaw
        dyaw = abs(math.atan2(math.sin(self.yaw - self.last_yaw),
                              math.cos(self.yaw - self.last_yaw)))
        self.turn_accum  += dyaw      # total turned this revolution
        self.scan_step_yaw += dyaw    # turned during THIS step
        self.last_yaw = self.yaw

        # PHASE 3: step complete? then stop and start a dwell.
        if self.scan_step_yaw >= SCAN_STEP_RAD:
            self.cmd = Twist()                              # stop the wheels
            self.scan_dwell_until = now + SCAN_DWELL_S      # look from here
        else:
            t = Twist(); t.angular.z = SEARCH_TURN_SPEED    # keep turning
            self.cmd = t

        # PHASE 4: a full revolution done and still nothing found.
        if self.turn_accum >= 2 * math.pi:
            self.turn_accum = 0.0; self.last_yaw = None
            self.scan_dwell_until = None; self.scan_step_yaw = 0.0
            if ENABLE_ADVANCE:
                self.advance_start = (self.x, self.y)
                self.search_state = "ADVANCE"
                self.notify(f"Can't see the {self.target} here — moving to look elsewhere.")

    def do_advance(self):
        if self.front_min < OBSTACLE_STOP:
            self.search_state = "ROTATE"; self.last_yaw = None
            self.turn_accum = math.pi; return
        if self.advance_start is not None:
            moved = math.hypot(self.x - self.advance_start[0], self.y - self.advance_start[1])
            if moved >= ADVANCE_DISTANCE:
                self.search_state = "ROTATE"; self.last_yaw = None; self.turn_accum = 0.0
                return
        t = Twist(); t.linear.x = ADVANCE_SPEED
        self.cmd = t

    def end_navigate(self):
        self.task_id += 1
        self.mode = None
        self.search_state = None
        self.target = None
        self.target_qualifier = None
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
        self.cmd = Twist()
        self.start_next_step()

    def end_locate(self):
        self.task_id += 1
        self.mode = None
        self.target = None
        self.target_qualifier = None
        self.locate_start = None
        self.cmd = Twist()
        self.start_next_step()

    def end_find_more(self):
        self.task_id += 1
        self.mode = None
        self.search_state = None
        self.target = None
        self.cmd = Twist()
        self.start_next_step()

    # ── EXPLORE / PATROL: frontier mapping with coverage memory (#4) ──
    def explore_think(self):
        if self.frontier_navigating: return
        now = time.monotonic()
        if self.map is None:
            if self.explore_start_t and now - self.explore_start_t > 6 and not self.warned_no_map:
                self.warned_no_map = True
                self.notify("I'm not receiving a /map. Is SLAM running? Launch with slam:=true.")
            return
        pose = self.robot_pose_map()
        if pose is None: return
        rx, ry, ryaw = pose
        clusters = [c for c in self.find_frontiers()
                    if not self.is_blacklisted(c[0], c[1]) and not self.is_visited(c[0], c[1])]
        if not clusters:
            if self.frontier_sent_goal:
                self.finish_explore()
            elif (self.explore_start_t and now - self.explore_start_t > 10
                  and not self.warned_no_frontier):
                self.warned_no_frontier = True
                self.notify("No unmapped area found. If you launched in localization mode the map "
                            "is already complete — relaunch with slam:=true to map fresh.")
            return

        # #4: prefer frontiers AHEAD (penalise turning around) so it stops
        # retreating down the path it just came from.
        def score(c):
            d = math.hypot(c[0] - rx, c[1] - ry)
            ang = math.atan2(c[1] - ry, c[0] - rx)
            turn = abs(math.atan2(math.sin(ang - ryaw), math.cos(ang - ryaw)))
            return d + TURN_WEIGHT * turn
        clusters.sort(key=score)
        gx, gy, _ = clusters[0]
        yaw = math.atan2(gy - ry, gx - rx)
        self.frontier_navigating = True
        self.frontier_sent_goal = True
        self.cur_goal = (gx, gy)
        self.send_goal(gx, gy, yaw)

    def finish_explore(self):
        self.notify("Area fully mapped — no frontiers left. Saving the map.")
        try:
            subprocess.run(["ros2", "run", "nav2_map_server", "map_saver_cli",
                            "-f", SAVE_MAP_PATH], timeout=30, check=False)
            self.notify(f"Map saved to {SAVE_MAP_PATH}.yaml")
        except Exception as e:
            self.notify(f"Auto-save failed ({e}); save manually with map_saver_cli.")
        self.task_id += 1
        self.mode = None
        self.frontier_navigating = False
        self.frontier_sent_goal = False
        self.start_next_step()
