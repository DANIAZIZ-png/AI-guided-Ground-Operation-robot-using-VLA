"""Safety guards: front clearance, collision braking and stuck detection.

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


class GuardsMixin:

    # ── fast hands ──
    # ── #59: how much clear space is REALLY in front of the bumper ──
    def front_clearance(self):
        """Metres to the nearest solid thing in a +/-GUARD_ARC_DEG arc ahead.

        Returns None when there is no usable scan (never a number we might
        act on by mistake). Unlike v24's front_min this uses a WIDE arc, so
        chair legs and table bases are actually intersected instead of being
        straddled by a narrow cone, and it takes the GUARD_MIN_BEAMS-th
        smallest range so a single spurious short beam cannot brake us."""
        m = self.scan_msg
        if m is None or not m.ranges:
            return None
        arc = math.radians(GUARD_ARC_DEG)
        near = []
        for i, r in enumerate(m.ranges):
            ang = m.angle_min + i * m.angle_increment
            if _off_front(ang) <= arc and math.isfinite(r) and r > m.range_min:
                near.append(r)               # #65: arc centred on the FRONT
        if len(near) < GUARD_MIN_BEAMS:
            return None
        near.sort()
        return near[GUARD_MIN_BEAMS - 1]

    # ── #62: who is holding the wheel, and are we going forward? ────
    def agent_owns_cmd(self):
        """True when THIS node is the thing publishing to /cmd_vel.

        Mirrors publish_cmd() exactly -- if that method would publish
        self.cmd, then self.cmd is the real commanded velocity. Anything
        else means Nav2 is driving and self.cmd is a stale zero.
        """
        if self.mode in ("move", "locate"):
            return True                       # relative move / turn: ours
        if (self.mode in ("navigate", "find_more")
                and self.search_state in ("ROTATE", "ADVANCE")):
            return True                       # search spin / creep: also ours
        return False                          # Nav2 has the wheel

    def driving_forward(self):
        """True if the robot is being commanded FORWARD right now.

        A front-facing guard is only meaningful against forward motion.
        Returns True conservatively whenever we cannot tell -- an unknown
        velocity must be treated as dangerous, never as safe.
        """
        if not self.agent_owns_cmd():
            return True                       # Nav2 driving: we cannot see the
                                              # twist, so assume forward and
                                              # keep the guard live (#59)
        try:
            vx = float(self.cmd.linear.x)     # the twist WE last set
        except Exception:
            return True                       # unreadable: assume the worst
        return vx > GUARD_MIN_FWD_SPEED       # <=0 is a turn or a reverse

    # ── #59: the guard itself, on its own 10 Hz timer ──────────────
    def collision_guard(self):
        """Stop the robot if something solid is closer than MIN_FRONT_CLEAR.

        Runs whether the robot is being driven by Nav2, by a relative-move
        command, or by the search rotation. It is deliberately independent of
        every belief the agent holds about where objects are."""
        if self.manual_mode:
            return          # #60: the operator has the wheel, and can see
        if self.mode not in ("navigate", "explore", "find_more", "move"):
            return          # nothing of ours is driving
        # #62: replaces `if self.search_state == "ROTATE": return`. That only
        # covered the SEARCH rotation and left `turn right 90` (mode="move")
        # unprotected from its own guard. Ask what we actually commanded.
        if not self.driving_forward():
            return          # turning in place or reversing: a FRONT guard is
                            # meaningless, and blocking it traps the robot
        if time.monotonic() < self.guard_mute_until:
            return          # already stopped for this obstacle; don't re-fire
        d = self.front_clearance()
        if d is None or d >= MIN_FRONT_CLEAR:
            return

        # Something is genuinely too close. Kill the motion first, explain after.
        self.guard_mute_until = time.monotonic() + 5.0
        self.stop_base()
        self.mlog.log("GUARD", f"collision guard stopped the robot at {d:.2f} m "
                               f"(limit {MIN_FRONT_CLEAR} m, mode={self.mode})")

        if self.mode in ("navigate", "find_more") and self.target:
            # If we were driving at a target, being this close to something
            # solid means we have arrived at it -- OR that the believed
            # position was wrong and we nearly hit something else. Say which,
            # honestly, rather than claiming a clean arrival either way.
            believed = None
            xy = self.robot_xy()          # (x, y) in the map frame, or None
            if self.last_obj_xy is not None and xy is not None:
                believed = math.hypot(xy[0]-self.last_obj_xy[0],
                                      xy[1]-self.last_obj_xy[1])
            if believed is not None and believed > MIN_FRONT_CLEAR + 0.8:
                self.notify(f"Stopping — something solid is {d:.2f} m ahead, but I "
                            f"thought the {self.target} was still {believed:.1f} m "
                            f"away. My distance estimate was wrong, so I'm not "
                            f"driving any further.")
                self.mlog.log("WARN", f"belief/LiDAR mismatch: believed {believed:.2f} m, "
                                      f"measured {d:.2f} m")
            else:
                self.notify(f"Arrived at the {self.target} — stopped {d:.2f} m short of it.")
            self.end_navigate()
        else:
            self.notify(f"Stopping — an obstacle is {d:.2f} m ahead.")

    # ── stuck detection ──
    def check_stuck(self):
        driving = ((self.mode in ("navigate", "find_more")
                    and self.search_state in ("NAVIGATING", "ADVANCE"))
                   or (self.mode == "explore" and self.frontier_navigating))
        if not driving:
            self.last_pos = None; self.last_move_t = None; return
        rxy = self.robot_xy()
        if rxy is None: return
        now = time.monotonic()
        if self.last_pos is None:
            self.last_pos = rxy; self.last_move_t = now; return
        if math.hypot(rxy[0]-self.last_pos[0], rxy[1]-self.last_pos[1]) > STUCK_DIST:
            self.last_pos = rxy; self.last_move_t = now; return
        if now - self.last_move_t > STUCK_TIME:
            self.on_stuck()

    def on_stuck(self):
        self.last_pos = None; self.last_move_t = None
        self.mlog.log("STUCK", f"mode={self.mode} target={self.target!r}")   # #14
        if self.goal_handle is not None:
            try: self.goal_handle.cancel_goal_async()
            except Exception: pass
        self.goal_handle = None
        if self.mode == "explore":
            self.consecutive_stucks += 1
            self.blacklist.append(self.cur_goal)
            self.frontier_navigating = False
            if self.consecutive_stucks >= MAX_STUCKS:
                self.notify("I keep getting stuck while exploring. Please take MANUAL OVERRIDE "
                            "(drive me with teleop), then tell me to continue.")
                self.cancel_task()
            else:
                self.notify("I got stuck at a frontier — skipping it and trying another route.")
        else:
            self.notify(f"I'm stuck trying to reach the {self.target}. Please take MANUAL "
                        "OVERRIDE or give me a new command.")
            self.cancel_navigation()
