#!/usr/bin/env python3
# ─────────────────────────────────────────────────────────────────
#  vla_gui_v2.py  —  OPERATOR CONSOLE FOR THE VLA GROUND ROBOT
#
#  ARCHITECTURE (the viva answer)
#    The GUI is a pure CLIENT. It owns no robot state and makes no decisions:
#      publishes  /vla/command        (identical string to typing it)
#      publishes  /vla/voice/trigger  (push-to-talk for voice_command.py)
#      subscribes /vla/annotated/compressed, /vla/reply, /vla/status
#    Kill the GUI mid-mission and the robot carries on untouched.
#
#  WHAT CHANGED FROM v1
#    #G1  Thread-safe logging. The launcher thread was writing straight into
#         a Qt text box. Only the GUI thread may touch widgets -> random
#         crash on START ALL. Now every log line travels over a Qt signal.
#    #G2  start_new_session=True replaces preexec_fn=os.setsid. Same effect
#         (own process group so stopping a step kills its children) but
#         preexec_fn can deadlock in a program that has threads, and this
#         program has several.
#    #G3  One-shot steps. spawn_objects.py does its job and EXITS. v1 read
#         "process gone" as "failed". Now a step marked "oneshot" goes green
#         on exit code 0.
#    #G4  The log box no longer rebuilds 3x a second, so you can scroll up
#         and read the error that just flew past.
#    #G5  html.escape() replaces QTextDocumentFragment.toHtml(). The latter
#         is a document exporter, not an escaper - it injects its own block
#         markup into the conversation panel.
#    #G6  Checkbox states are read on the GUI thread and handed to the
#         launcher as plain data, instead of the launcher reaching into
#         widgets. Same root cause as #G1.
#
#  RUN   python3 vla_gui_v2.py      (inside the ubuntu22-gpu distrobox)
# ─────────────────────────────────────────────────────────────────

import html
# #G5: turns < > & into &lt; &gt; &amp; so robot text cannot break the HTML layout
import json
# reads/writes the launch config file and parses the /vla/status JSON packet
import os
# used for expanding ~ into /home/danyalaziz and for killing process groups
import shutil
# shutil.which() checks whether a program (distrobox-host-exec, gedit) exists
import signal
# supplies SIGINT / SIGTERM / SIGKILL, the three escalating "please stop" signals
import socket
# used to knock on a TCP port to see if a server (YOLO :5001, Ollama :11434) is up
import subprocess
# launches the other programs (Gazebo, the agent, the voice node) as child processes
import sys
# gives access to command-line arguments and lets us exit with a status code
import threading
# lets the launcher and the ROS spinner run in the background without freezing the window
import time
# timestamps for the conversation log and timeouts for the health probes

CONFIG_PATH = os.path.expanduser("~/.vla_gui.json")
# the launch config lives in your home folder, NOT inside this file, so you can edit it
GUI_VERSION = "vla_gui v2"
# printed in the window title so you can prove at a glance which version is running

# ── topics (must match vla_agent v9 #33) ────────────────────────────
CMD_TOPIC      = "/vla/command"
# we PUBLISH here; the agent treats it exactly like something you typed
REPLY_TOPIC    = "/vla/reply"
# we SUBSCRIBE here; everything the robot "says" arrives on this topic
STATUS_TOPIC   = "/vla/status"
# we SUBSCRIBE here; a small JSON state packet at 2 Hz drives the status bar
FEED_TOPIC     = "/vla/annotated/compressed"
# we SUBSCRIBE here; the JPEG camera frame with YOLO boxes already drawn on it
TRIGGER_TOPIC  = "/vla/voice/trigger"
# we PUBLISH here; True = start recording, False = stop, for the hold-to-talk button
VOICE_STATE    = "/vla/voice/state"
# we SUBSCRIBE here; idle / listening / recording / transcribing, shown in the status bar


# ─────────────────────────────────────────────────────────────────
#  DEFAULT LAUNCH STEPS — only used if ~/.vla_gui.json does not exist
# ─────────────────────────────────────────────────────────────────
DEFAULT_CONFIG = {
    "_comment": [
        "Edit the 'cmd' lines to match exactly what you type today.",
        "where: here | host | box:<distrobox-name>",
        "probe: port:<n> | topic:<n> | node:<n> | none",
        "oneshot: true  -> a script that finishes and exits (green on exit 0)",
    ],
    "sim_mode": True,
    # True shows the Gazebo steps, False shows the real-robot steps
    "steps": [
        {
            "name": "YOLO-World server",
            "where": "box:vla-box",
            "cmd": "source ~/yolo-env/bin/activate && python ~/yolo_server.py",
            "probe": "port:5001",
            "wait_s": 120,
            "enabled": True,
        },
        {
            "name": "Ollama (Qwen2.5-7B)",
            "where": "host",
            "cmd": "if pgrep -x ollama >/dev/null; then echo '[ok] already running'; "
                   "sleep infinity; else ollama serve; fi",
            "probe": "port:11434",
            "wait_s": 45,
            "enabled": True,
        },
        {
            "name": "Gazebo + SLAM + Nav2 + RViz",
            "where": "here",
            "cmd": "ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py "
                   "model:=lite slam:=true nav2:=true rviz:=true",
            "probe": "topic:/map",
            "wait_s": 300,
            "enabled": True,
            "sim_only": True,
        },
        {
            "name": "Spawn demo objects",
            "where": "here",
            "cmd": "python3 ~/spawn_objects.py",
            "probe": "none",
            "wait_s": 60,
            "enabled": True,
            "oneshot": True,
            "sim_only": True,
        },
        {
            "name": "VLA agent (v9)",
            "where": "here",
            "cmd": "cd ~ && python3 vla_agent_v9.py",
            "probe": "topic:/vla/status",
            "wait_s": 90,
            "enabled": True,
            "interactive": True,
        },
        {
            "name": "Voice input (Whisper)",
            "where": "here",
            "cmd": "cd ~ && python3 voice_command.py --mode ptt",
            "probe": "topic:/vla/voice/state",
            "wait_s": 240,
            "enabled": True,
            "interactive": True,
        },
    ],
}


def load_config():
    """Read ~/.vla_gui.json, creating it from the defaults on first run."""
    if not os.path.exists(CONFIG_PATH):
        # nothing saved yet -> this is the very first launch on this machine
        with open(CONFIG_PATH, "w") as f:
            # open the file for writing; it is created if it does not exist
            json.dump(DEFAULT_CONFIG, f, indent=2)
            # write the default steps out as readable, indented JSON
        print(f"[gui] wrote a starter launch config to {CONFIG_PATH}")
        # tell the operator where the file is so they can edit it
        return json.loads(json.dumps(DEFAULT_CONFIG))
        # dump-then-load makes a deep COPY, so edits in the GUI never corrupt the template
    try:
        with open(CONFIG_PATH) as f:
            # open the existing config for reading
            return json.load(f)
            # parse the JSON text into a Python dictionary
    except Exception as e:
        # a stray comma or missing quote should not stop the whole console from opening
        print(f"[gui] {CONFIG_PATH} is not valid JSON ({e}) — using defaults.")
        return json.loads(json.dumps(DEFAULT_CONFIG))


def wrap_command(cmd, where):
    """Turn a plain shell command into one that runs in the RIGHT place.

    This GUI lives inside ubuntu22-gpu. distrobox-host-exec is installed in
    every container and lets it reach back out to the Ubuntu 24.04 host —
    which is also how it reaches sideways into vla-box."""
    if where == "here":
        # "here" = this same container, so no wrapping at all is needed
        return ["bash", "-lc", cmd]
        # -l = login shell (loads your ~/.bashrc so ros2 and conda are on PATH), -c = run this string
    host_exec = shutil.which("distrobox-host-exec")
    # returns the full path to the helper, or None if we are already on the host
    if where == "host":
        if host_exec:
            # we are inside a container -> hop out to the host first
            return [host_exec, "bash", "-lc", cmd]
        return ["bash", "-lc", cmd]
        # already on the host, so just run it directly
    if where.startswith("box:"):
        # e.g. "box:vla-box" -> run this command inside the vla-box container
        box = where.split(":", 1)[1]
        # split once on the colon and keep the right-hand side: "vla-box"
        inner = f"distrobox enter {box} -- bash -lc {json.dumps(cmd)}"
        # json.dumps adds quotes and escapes, so a command containing && or spaces survives intact
        if host_exec:
            # hop to the host, then from the host enter the other container
            return [host_exec, "bash", "-lc", inner]
        return ["bash", "-lc", inner]
    return ["bash", "-lc", cmd]
    # unknown "where" value -> fall back to running it locally rather than crashing


def port_open(port, host="127.0.0.1", timeout=0.35):
    """Health probe: is anything accepting TCP connections on this port?"""
    try:
        with socket.create_connection((host, int(port)), timeout=timeout):
            # try to open a connection; the short timeout keeps the GUI responsive
            return True
            # the "with" block closes the socket immediately - we only wanted to knock
    except Exception:
        return False
        # refused, timed out, or unreachable -> the server is not up yet


# ─────────────────────────────────────────────────────────────────
#  ROS side — its own class and its own thread, so the window still
#  opens (and can still launch things) if rclpy is missing.
# ─────────────────────────────────────────────────────────────────
class RosBridge:
    def __init__(self, on_reply, on_status, on_frame, on_voice_state):
        self.ok = False
        # stays False if ROS could not be imported; every send() checks this first
        self.node = None
        self.on_reply = on_reply
        # callback functions handed in by the window, so this class knows nothing about Qt
        self.on_status = on_status
        self.on_frame = on_frame
        self.on_voice_state = on_voice_state
        self._topics = set()
        # cached list of live topic names, refreshed once a second
        self._nodes = set()
        # cached list of live node names
        try:
            import rclpy
            # the ROS 2 Python client library
            from rclpy.node import Node
            # the base class for anything that talks on the ROS network
            from rclpy.qos import (QoSProfile, QoSHistoryPolicy,
                                   QoSReliabilityPolicy)
            # QoS = Quality of Service, the delivery contract between publisher and subscriber
            from std_msgs.msg import String, Bool
            # the two simplest message types: a text string and a true/false flag
            from sensor_msgs.msg import CompressedImage
            # a JPEG-encoded camera frame
        except Exception as e:
            # no ROS on this machine -> degrade gracefully instead of refusing to start
            print(f"[gui] ROS not available ({e}). Launch panel still works; "
                  f"feed and commands are disabled.")
            return

        self._rclpy = rclpy
        # keep a reference so shutdown() can call rclpy.shutdown() later
        rclpy.init()
        # start the ROS client library for this process
        self.node = Node("vla_gui")
        # create our node; this name is what shows up in `ros2 node list`
        qos = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=1,
                         reliability=QoSReliabilityPolicy.RELIABLE)
        # KEEP_LAST depth 1 = only ever hold the newest frame, so a slow GUI drops
        # frames instead of building a backlog. RELIABLE matches the agent's publisher.
        self.String, self.Bool = String, Bool
        # store the message classes as attributes so send_command() can build messages
        self.cmd_pub = self.node.create_publisher(String, CMD_TOPIC, 10)
        # our outgoing command channel; 10 = queue depth
        self.trig_pub = self.node.create_publisher(Bool, TRIGGER_TOPIC, 10)
        # the push-to-talk trigger for the voice node
        self.node.create_subscription(String, REPLY_TOPIC,
                                      lambda m: self.on_reply(m.data), 20)
        # every robot reply is handed straight to the window's callback
        self.node.create_subscription(String, STATUS_TOPIC,
                                      lambda m: self.on_status(m.data), 5)
        # the 2 Hz JSON state packet
        self.node.create_subscription(String, VOICE_STATE,
                                      lambda m: self.on_voice_state(m.data), 5)
        # idle / recording / transcribing
        self.node.create_subscription(CompressedImage, FEED_TOPIC,
                                      lambda m: self.on_frame(bytes(m.data)), qos)
        # the annotated camera feed; bytes() copies the JPEG data out of the message
        self.node.create_timer(1.0, self._refresh_graph)
        # once a second, re-read which topics and nodes exist
        self.ok = True
        # from here on the send/publish methods are allowed to run
        threading.Thread(target=self._spin, daemon=True).start()
        # ROS must be "spun" to deliver callbacks; daemon=True lets the app exit cleanly

    def _spin(self):
        try:
            self._rclpy.spin(self.node)
            # blocks forever, dispatching incoming messages - hence its own thread
        except Exception:
            pass
            # on shutdown spin() raises; there is nothing useful to do about it

    def _refresh_graph(self):
        """Cache the ROS graph so the health probes are cheap and never block Qt."""
        try:
            self._topics = {n for n, _ in self.node.get_topic_names_and_types()}
            # returns (name, types) pairs; we only need the names
            self._nodes = {("/" + n.lstrip("/"))
                           for n in self.node.get_node_names()}
            # normalise every name to start with exactly one slash
        except Exception:
            pass
            # a transient discovery error must not kill the timer

    def has_topic(self, name):
        return name in self._topics
        # instant set lookup - no network call, so it is safe to call every 300 ms

    def has_node(self, name):
        return ("/" + name.lstrip("/")) in self._nodes
        # normalise the same way we did when caching, so "/x" and "x" both match

    def send_command(self, text):
        if not self.ok:
            return False
            # no ROS -> report failure so the window can tell the operator
        m = self.String(); m.data = text
        # build a std_msgs/String and put the command text in it
        self.cmd_pub.publish(m)
        # send it; the agent cannot tell this came from a GUI rather than a keyboard
        return True

    def set_voice_trigger(self, on):
        if not self.ok:
            return False
        m = self.Bool(); m.data = bool(on)
        # True = begin recording, False = stop and transcribe
        self.trig_pub.publish(m)
        return True

    def shutdown(self):
        try:
            if self.node:
                self.node.destroy_node()
                # cleanly remove the node from the ROS graph
            if self.ok:
                self._rclpy.shutdown()
                # shut the client library down
        except Exception:
            pass


# ─────────────────────────────────────────────────────────────────
#  Launch supervision
# ─────────────────────────────────────────────────────────────────
class Step:
    """One launchable process, with its output captured and a health probe."""

    def __init__(self, spec):
        self.spec = spec
        # the raw dictionary from the config file, kept for flags like "interactive"
        self.name = spec.get("name", "step")
        self.where = spec.get("where", "here")
        # here | host | box:<name>
        self.cmd = spec.get("cmd", "true")
        # "true" is a shell command that does nothing and succeeds - a safe default
        self.probe = spec.get("probe", "none")
        # how we decide this step is genuinely READY, not merely running
        self.wait_s = float(spec.get("wait_s", 60))
        # how long to wait for the probe before declaring failure
        self.oneshot = bool(spec.get("oneshot", False))
        # #G3: True means this script is SUPPOSED to finish and exit
        self.proc = None
        # the subprocess handle, or None if never started
        self.log = []
        # every line the process printed, so a red lamp can be explained
        self.state = "stopped"
        # stopped | starting | ready | failed  -> drives the lamp colour
        self.started_t = 0.0
        # when we launched it, used for the timeout maths

    def start(self):
        if self.proc is not None and self.proc.poll() is None:
            # poll() returns None while the process is still alive
            return
            # already running -> pressing start twice must not spawn a duplicate
        argv = wrap_command(self.cmd, self.where)
        # turn the plain command into one aimed at the right container/host
        self.log = [f"$ {' '.join(argv[:2])} ...", f"  ({self.where}) {self.cmd}", ""]
        # seed the log with what we are about to run, so the panel is never empty
        try:
            self.proc = subprocess.Popen(
                argv,
                # the command, already split into a list of arguments
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                # capture normal output AND errors down the same pipe, in the right order
                stdin=(subprocess.PIPE if self.spec.get("interactive")
                       else subprocess.DEVNULL),
                # interactive steps (agent, voice) call input(). A PIPE we never write to
                # makes that read BLOCK. DEVNULL would return EOF instantly and the agent
                # would read it as "operator typed quit" and die on startup.
                text=True, bufsize=1,
                # text=True gives us str not bytes; bufsize=1 = line buffered, so the log is live
                start_new_session=True)
                # #G2: put the child in its own process group. `ros2 launch` starts a whole
                # tree of processes; killing the group kills all of them. This replaces
                # preexec_fn=os.setsid, which can deadlock in a threaded program.
        except Exception as e:
            self.state = "failed"
            self.log.append(f"!! could not start: {e}")
            # e.g. bash missing, or a typo in "where" - show it rather than silently doing nothing
            return
        self.state = "starting"
        # orange lamp: the process exists but has not proven itself yet
        self.started_t = time.time()
        # start the stopwatch for the timeout
        threading.Thread(target=self._pump, daemon=True).start()
        # a reader thread, because reading a pipe blocks and must not freeze the window

    def _pump(self):
        """Continuously drain the child's output into self.log."""
        try:
            for line in self.proc.stdout:
                # iterating a pipe yields one line at a time, blocking in between
                self.log.append(line.rstrip("\n"))
                # strip the trailing newline; the text box adds its own line breaks
                if len(self.log) > 800:
                    del self.log[:300]
                    # cap memory: Gazebo is chatty and would otherwise grow forever
        except Exception:
            pass
            # the pipe closes when the process exits; that ends the loop normally

    def stop(self):
        """Escalating shutdown: ask nicely, then insist, then force."""
        if self.proc is None:
            self.state = "stopped"
            return
        try:
            os.killpg(os.getpgid(self.proc.pid), signal.SIGINT)
            # SIGINT = the same as Ctrl-C. killpg targets the whole GROUP, not just the parent,
            # so ros2 launch's children die with it.
            for _ in range(30):
                if self.proc.poll() is not None:
                    break
                    # it exited on its own - nothing more to do
                time.sleep(0.1)
                # give it up to 3 seconds total to shut down cleanly
            if self.proc.poll() is None:
                os.killpg(os.getpgid(self.proc.pid), signal.SIGTERM)
                # stronger "please terminate"
                time.sleep(0.5)
            if self.proc.poll() is None:
                os.killpg(os.getpgid(self.proc.pid), signal.SIGKILL)
                # unblockable. Last resort only, because it skips cleanup.
        except Exception:
            pass
            # the process may already be gone; that is success, not an error
        self.proc = None
        self.state = "stopped"

    def alive(self):
        return self.proc is not None and self.proc.poll() is None
        # True only if we started something AND it has not exited

    def check(self, ros):
        """Update state from the health probe.

        A lamp that turns green when a PROCESS starts is a lie — ros2 launch
        returns instantly and Gazebo takes a minute. These probes test the
        thing the NEXT step actually depends on."""
        if self.proc is None:
            self.state = "stopped"
            return
            # never started, or explicitly stopped -> grey lamp

        # #G3: one-shot steps are SUPPOSED to exit, so they are judged ONLY by
        # their exit code — never by a probe. Judging them by a probe would let
        # a "probe: none" script go green a millisecond after launch, before it
        # has actually done anything, which defeats the whole point.
        if self.oneshot:
            if self.proc.poll() is None:
                # still working - it has not told us whether it succeeded yet
                if time.time() - self.started_t > self.wait_s:
                    self.state = "failed"
                    # a one-shot script that never finishes is hung
                    if self.log and not self.log[-1].startswith("!! still running"):
                        self.log.append(f"!! still running after {self.wait_s:.0f}s "
                                        f"— expected this step to finish and exit")
                else:
                    self.state = "starting"
                return
            self.state = "ready" if self.proc.returncode == 0 else "failed"
            # exit code 0 is the universal Unix convention for success
            if self.state == "failed" and self.log and \
                    not self.log[-1].startswith("!! exited"):
                self.log.append(f"!! exited with code {self.proc.returncode}")
                # append once, not every 300 ms, so the log stays readable
            return

        if not self.alive():
            # a long-running step that died is a genuine failure
            self.state = "failed" if self.state != "stopped" else "stopped"
            return

        p = self.probe or "none"
        ready = False
        # assume not ready until a probe proves otherwise
        if p == "none":
            ready = True
            # no probe defined -> "alive" is the best evidence we have
        elif p.startswith("port:"):
            ready = port_open(p.split(":", 1)[1])
            # e.g. "port:5001" -> is the YOLO server accepting connections?
        elif p.startswith("topic:"):
            ready = bool(ros and ros.ok and ros.has_topic(p.split(":", 1)[1]))
            # e.g. "topic:/map" -> has SLAM actually produced a map yet?
        elif p.startswith("node:"):
            ready = bool(ros and ros.ok and ros.has_node(p.split(":", 1)[1]))
            # e.g. "node:/vla_agent" -> is that node registered on the ROS graph?

        if ready:
            self.state = "ready"
            # green lamp: the next step may safely start
        elif time.time() - self.started_t > self.wait_s:
            self.state = "failed"
            # running but never became useful - almost always a config or dependency problem
            if self.log and not self.log[-1].startswith("!! timed out"):
                self.log.append(f"!! timed out after {self.wait_s:.0f}s waiting "
                                f"for {p}  (process is still running)")
        else:
            self.state = "starting"
            # still inside its grace period - orange lamp, keep waiting


# ─────────────────────────────────────────────────────────────────
#  Qt window.  Built inside a function so that `import vla_gui_v2`
#  works on a machine with no PyQt5 (the tests rely on this).
# ─────────────────────────────────────────────────────────────────
def build_gui():
    from PyQt5 import QtCore, QtGui, QtWidgets
    # imported here, not at the top, so the module can be imported headless

    LAMP = {"stopped": "#6b7280", "starting": "#d97706",
            "ready": "#16a34a", "failed": "#dc2626"}
    # grey / orange / green / red - the only status vocabulary the operator needs

    class Console(QtWidgets.QMainWindow):

        # #G1: a Qt signal is the ONLY safe way to send data from a background
        # thread to the GUI thread. Qt automatically queues it and delivers it
        # on the GUI thread. (who, text)
        log_sig = QtCore.pyqtSignal(str, str)

        def __init__(self):
            super().__init__()
            # run QMainWindow's own setup first
            self.setWindowTitle(f"VLA Ground Robot — Operator Console  [{GUI_VERSION}]")
            self.resize(1500, 900)

            self.cfg = load_config()
            # read ~/.vla_gui.json
            self.sim_mode = bool(self.cfg.get("sim_mode", True))
            # simulation or real robot - decides which steps are shown
            self.steps = []
            self.frame_bytes = None
            # the most recent JPEG from the camera feed
            self.frame_t = 0.0
            # when that frame arrived, so we can say "no frame for 8s"
            self.status = {}
            # the decoded /vla/status packet
            self.voice_state = "idle"
            self.recording = False
            self._log_shown_step = None
            # #G4: which step's log is currently rendered
            self._log_shown_len = -1
            # #G4: how many lines were rendered, so we can skip pointless redraws

            self.log_sig.connect(self._log_line_gui)
            # #G1: wire the signal to the slot that actually touches the widget

            self.ros = RosBridge(self._ros_reply, self._ros_status,
                                 self._ros_frame, self._ros_voice_state)
            # start ROS and hand it our four callbacks

            self._build_layout()
            # create every widget
            self._rebuild_steps()
            # create one row per launch step

            self.timer = QtCore.QTimer(self)
            self.timer.timeout.connect(self._tick)
            self.timer.start(300)
            # refresh lamps, feed and status bar about 3 times a second

        # ── layout ──────────────────────────────────────────────
        def _build_layout(self):
            central = QtWidgets.QWidget(); self.setCentralWidget(central)
            # a QMainWindow needs one central widget to hold everything else
            root = QtWidgets.QHBoxLayout(central)
            # H = horizontal: launch panel on the left, everything else on the right
            root.setContentsMargins(10, 10, 10, 10)
            root.setSpacing(10)

            # ---------- LEFT: launch panel ----------
            left = QtWidgets.QWidget()
            left.setFixedWidth(370)
            # fixed so the video panel gets all the extra space when you resize
            lv = QtWidgets.QVBoxLayout(left)
            # V = vertical: title, mode, steps, buttons, log stacked downwards
            lv.setContentsMargins(0, 0, 0, 0)

            title = QtWidgets.QLabel("SYSTEM LAUNCH")
            title.setStyleSheet("font-size:14px;font-weight:600;letter-spacing:1px;")
            lv.addWidget(title)

            modebox = QtWidgets.QHBoxLayout()
            self.rb_sim = QtWidgets.QRadioButton("Simulation")
            self.rb_real = QtWidgets.QRadioButton("Real robot")
            # radio buttons are mutually exclusive by default - you cannot pick both
            (self.rb_sim if self.sim_mode else self.rb_real).setChecked(True)
            # tick whichever one the config says
            self.rb_sim.toggled.connect(self._mode_changed)
            # when it flips, rebuild the step list
            modebox.addWidget(self.rb_sim); modebox.addWidget(self.rb_real)
            modebox.addStretch()
            # push both buttons to the left
            lv.addLayout(modebox)

            self.step_area = QtWidgets.QVBoxLayout()
            self.step_area.setSpacing(4)
            holder = QtWidgets.QWidget(); holder.setLayout(self.step_area)
            scroll = QtWidgets.QScrollArea(); scroll.setWidgetResizable(True)
            # scrollable, so a long step list never overflows the window
            scroll.setWidget(holder); scroll.setFrameShape(QtWidgets.QFrame.NoFrame)
            lv.addWidget(scroll, 1)
            # the 1 is a stretch factor: this area absorbs spare vertical space

            btns = QtWidgets.QHBoxLayout()
            self.btn_start_all = QtWidgets.QPushButton("▶  START ALL")
            self.btn_start_all.setStyleSheet(
                "background:#16a34a;color:white;font-weight:700;padding:10px;")
            self.btn_start_all.clicked.connect(self._start_all)
            self.btn_stop_all = QtWidgets.QPushButton("■  STOP ALL")
            self.btn_stop_all.setStyleSheet(
                "background:#374151;color:white;font-weight:700;padding:10px;")
            self.btn_stop_all.clicked.connect(self._stop_all)
            btns.addWidget(self.btn_start_all, 2); btns.addWidget(self.btn_stop_all, 1)
            # START gets twice the width of STOP - it is the button you press most
            lv.addLayout(btns)

            cfgrow = QtWidgets.QHBoxLayout()
            b = QtWidgets.QPushButton("Edit launch config")
            b.clicked.connect(self._edit_config)
            cfgrow.addWidget(b)
            b2 = QtWidgets.QPushButton("Reload")
            b2.clicked.connect(self._reload_config)
            cfgrow.addWidget(b2)
            lv.addLayout(cfgrow)

            self.log_view = QtWidgets.QPlainTextEdit()
            # QPlainTextEdit is the fast one - QTextEdit would choke on Gazebo's output
            self.log_view.setReadOnly(True)
            self.log_view.setMaximumBlockCount(2000)
            # auto-discard the oldest lines past 2000 so memory stays flat
            self.log_view.setStyleSheet(
                "font-family:monospace;font-size:11px;background:#111827;color:#d1d5db;")
            lv.addWidget(QtWidgets.QLabel("Output of the selected step:"))
            lv.addWidget(self.log_view, 1)
            root.addWidget(left)
            left.hide()
            # LAUNCH PANEL HIDDEN. Its START ALL does not work -- handout v5 s6:
            # probes check topic existence rather than data, distrobox-host-exec
            # returns 127 so it cannot reach YOLO or Ollama, and it cannot
            # re-latch subscriptions. Worse, pressing it against a stack already
            # built by vla_sim.sh KILLS that stack. The GUI's real job is the
            # viewer half: feed, conversation, status, command box.
            #
            # addWidget STAYS, then hide(). The widget must keep its parent or
            # Python drops the last reference, Qt destroys it, and every child
            # (btn_start_all, log_view, the step lamps) becomes a dangling
            # pointer -- _tick() touches those lamps every 300 ms and would
            # raise "wrapped C/C++ object has been deleted".
            # A hidden widget is given zero space by the layout, so the feed
            # and conversation expand into the full window.
            # To restore the panel: delete the left.hide() line
            # ---------- RIGHT: feed + conversation ----------
            right = QtWidgets.QWidget()
            rv = QtWidgets.QVBoxLayout(right)
            rv.setContentsMargins(0, 0, 0, 0)

            self.status_bar = QtWidgets.QLabel("agent: not connected")
            self.status_bar.setStyleSheet(
                "background:#1f2937;color:#e5e7eb;padding:7px;font-family:monospace;")
            # monospace so the numbers do not jitter as they change
            rv.addWidget(self.status_bar)

            split = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
            # a draggable divider, so you can widen the video during a demo

            feedbox = QtWidgets.QWidget()
            fv = QtWidgets.QVBoxLayout(feedbox); fv.setContentsMargins(0, 0, 0, 0)
            cap = QtWidgets.QLabel("ROBOT VIEW  —  YOLO-World detections + OAK-D depth")
            cap.setStyleSheet("font-weight:600;padding:4px;")
            fv.addWidget(cap)
            self.feed_label = QtWidgets.QLabel("waiting for the camera feed…")
            self.feed_label.setAlignment(QtCore.Qt.AlignCenter)
            self.feed_label.setStyleSheet("background:#000;color:#6b7280;")
            self.feed_label.setMinimumSize(420, 320)
            # a QLabel can display a QPixmap - that is how the video is shown
            fv.addWidget(self.feed_label, 1)
            self.feed_note = QtWidgets.QLabel(f"subscribing {FEED_TOPIC}")
            self.feed_note.setStyleSheet("color:#6b7280;font-size:11px;padding:2px;")
            fv.addWidget(self.feed_note)
            split.addWidget(feedbox)

            convbox = QtWidgets.QWidget()
            cv = QtWidgets.QVBoxLayout(convbox); cv.setContentsMargins(0, 0, 0, 0)
            cvcap = QtWidgets.QLabel("CONVERSATION")
            cvcap.setStyleSheet("font-weight:600;padding:4px;")
            cv.addWidget(cvcap)
            self.conv = QtWidgets.QTextEdit(); self.conv.setReadOnly(True)
            # QTextEdit here (not Plain) because we want colour and bold labels
            self.conv.setStyleSheet("font-size:13px;background:#fbfbfd;")
            cv.addWidget(self.conv, 1)
            split.addWidget(convbox)
            split.setSizes([760, 640])
            # starting widths; the operator can drag from here
            rv.addWidget(split, 1)

            # ---------- command row ----------
            cmdrow = QtWidgets.QHBoxLayout()
            self.mode_combo = QtWidgets.QComboBox()
            self.mode_combo.addItems(["Text command", "Voice command"])
            self.mode_combo.currentIndexChanged.connect(self._input_mode_changed)
            self.mode_combo.setFixedWidth(150)
            cmdrow.addWidget(self.mode_combo)

            self.entry = QtWidgets.QLineEdit()
            self.entry.setPlaceholderText(
                "e.g.  go to the nearest chair   |   patrol the area   |   find a person")
            self.entry.returnPressed.connect(self._send_text)
            # pressing Enter sends, so you never have to reach for the mouse
            self.entry.setStyleSheet("padding:9px;font-size:14px;")
            cmdrow.addWidget(self.entry, 1)

            self.btn_send = QtWidgets.QPushButton("Send")
            self.btn_send.setStyleSheet(
                "background:#1F4E79;color:white;font-weight:700;padding:9px 18px;")
            # #1F4E79 is the NUST steel-blue, matching the presentation deck
            self.btn_send.clicked.connect(self._send_text)
            cmdrow.addWidget(self.btn_send)

            self.btn_mic = QtWidgets.QPushButton("🎤  HOLD TO TALK")
            self.btn_mic.setStyleSheet(
                "background:#C4A02C;color:#1a1a1a;font-weight:700;padding:9px 18px;")
            self.btn_mic.pressed.connect(self._mic_down)
            # "pressed" fires on mouse DOWN - recording starts the instant you press
            self.btn_mic.released.connect(self._mic_up)
            # "released" fires on mouse UP - that is what makes it push-to-talk
            self.btn_mic.hide()
            # hidden until the operator switches the dropdown to Voice
            cmdrow.addWidget(self.btn_mic)

            self.btn_stop = QtWidgets.QPushButton("■ STOP")
            self.btn_stop.setStyleSheet(
                "background:#dc2626;color:white;font-weight:800;padding:9px 18px;")
            self.btn_stop.clicked.connect(lambda: self._send("cancel", "operator"))
            # sends the word "cancel", exactly as if it had been typed or spoken
            cmdrow.addWidget(self.btn_stop)
            rv.addLayout(cmdrow)

            quick = QtWidgets.QHBoxLayout()
            for label, cmd in [("Patrol", "patrol the area"),
                               ("Find a person", "find a person"),
                               ("Go to nearest chair", "go to the nearest chair"),
                               ("Show feed", "show me the live feed"),
                               ("Undock", "undock"), ("Dock", "dock")]:
                qb = QtWidgets.QPushButton(label)
                qb.setStyleSheet("padding:6px;")
                qb.clicked.connect(lambda _, c=cmd: self._send(c, "operator"))
                # c=cmd captures the CURRENT value; without it every button would send the last one
                quick.addWidget(qb)
            quick.addStretch()
            rv.addLayout(quick)

            root.addWidget(right, 1)
            # stretch factor 1: the right side takes all the leftover width

        # ── launch steps ────────────────────────────────────────
        def _rebuild_steps(self):
            """Redraw the step list, e.g. after switching sim/real or reloading."""
            while self.step_area.count():
                w = self.step_area.takeAt(0).widget()
                # takeAt removes the item from the layout and returns it
                if w:
                    w.deleteLater()
                    # deleteLater is the safe way to destroy a Qt widget
            self.steps = []
            for spec in self.cfg.get("steps", []):
                if spec.get("sim_only") and not self.sim_mode:
                    continue
                    # skip Gazebo steps when we are on the real robot
                if spec.get("real_only") and self.sim_mode:
                    continue
                    # skip real-robot steps when we are in simulation
                st = Step(spec)
                row = QtWidgets.QWidget()
                h = QtWidgets.QHBoxLayout(row); h.setContentsMargins(2, 2, 2, 2)
                chk = QtWidgets.QCheckBox()
                chk.setChecked(bool(spec.get("enabled", True)))
                # unticking a step makes START ALL skip it - useful when debugging
                lamp = QtWidgets.QLabel("●")
                lamp.setStyleSheet(f"color:{LAMP['stopped']};font-size:18px;")
                name = QtWidgets.QPushButton(f"{st.name}")
                name.setFlat(True)
                # flat = looks like a label but is clickable
                name.setStyleSheet("text-align:left;padding:2px;")
                name.clicked.connect(lambda _, s=st: self._show_log(s, force=True))
                # clicking the name shows that step's output in the black box
                where = QtWidgets.QLabel(st.where)
                where.setStyleSheet("color:#6b7280;font-size:10px;")
                go = QtWidgets.QPushButton("▶"); go.setFixedWidth(30)
                go.clicked.connect(lambda _, s=st: s.start())
                sp = QtWidgets.QPushButton("■"); sp.setFixedWidth(30)
                sp.clicked.connect(lambda _, s=st: s.stop())
                h.addWidget(chk); h.addWidget(lamp); h.addWidget(name, 1)
                h.addWidget(where); h.addWidget(go); h.addWidget(sp)
                self.step_area.addWidget(row)
                st.ui = {"lamp": lamp, "chk": chk}
                # keep handles to the widgets this step owns
                self.steps.append(st)
            self.step_area.addStretch()
            # push all rows to the top
            self.selected = self.steps[0] if self.steps else None
            self._log_shown_len = -1
            # force the log box to redraw for the newly selected step

        def _mode_changed(self):
            self.sim_mode = self.rb_sim.isChecked()
            self.cfg["sim_mode"] = self.sim_mode
            self._rebuild_steps()
            # different mode -> a different set of steps

        def _start_all(self):
            """Start enabled steps IN ORDER, each waiting for its own probe.

            Starting them simultaneously is how you get an agent that comes up
            before Nav2 and sits there complaining there is no map."""
            enabled = [st for st in self.steps if st.ui["chk"].isChecked()]
            # #G6: read the checkboxes HERE, on the GUI thread, and hand the
            # launcher plain data. The worker thread must never touch a widget.
            self._log_line("system", "Starting the stack…")
            threading.Thread(target=self._start_all_worker,
                             args=(enabled,), daemon=True).start()
            # a background thread, otherwise the window would freeze for 5 minutes

        def _start_all_worker(self, enabled):
            """Runs on a BACKGROUND thread. Touches no widgets — only self.log_sig."""
            for st in enabled:
                if st.state == "ready":
                    continue
                    # already up (e.g. you pressed START ALL twice) - skip it
                st.start()
                self._log_line("system", f"→ {st.name} ({st.where}) …")
                t0 = time.time()
                while time.time() - t0 < st.wait_s:
                    st.check(self.ros)
                    # re-evaluate the health probe
                    if st.state == "ready":
                        self._log_line("system", f"✓ {st.name} ready "
                                                 f"({time.time() - t0:.0f}s)")
                        break
                        # move on to the next step
                    if st.state == "failed":
                        self._log_line("system", f"✗ {st.name} FAILED — click its "
                                                 f"name to read the output")
                        return
                        # stop the whole sequence: later steps depend on this one
                    time.sleep(0.4)
                    # poll gently rather than spinning the CPU
                else:
                    # this "else" belongs to the WHILE loop: it runs only if the loop
                    # ran out of time instead of hitting "break"
                    self._log_line("system", f"✗ {st.name} did not report ready "
                                             f"in {st.wait_s:.0f}s — stopping here")
                    return
            self._log_line("system", "All selected steps are up. The robot is ready.")

        def _stop_all(self):
            for st in reversed(self.steps):
                st.stop()
                # reversed = shut down in the opposite order to startup, so the agent
                # goes down before the Nav2 stack it depends on
            self._log_line("system", "All steps stopped.")

        def _show_log(self, st, force=False):
            """#G4: only redraw when the content actually changed, and keep the
            operator's scroll position if they have scrolled up to read."""
            if st is None:
                return
            self.selected = st
            n = len(st.log)
            if not force and st is self._log_shown_step and n == self._log_shown_len:
                return
                # nothing new since the last redraw - leave the box completely alone
            bar = self.log_view.verticalScrollBar()
            at_bottom = bar.value() >= bar.maximum() - 4
            # were we already following the tail? (the 4 is slack for rounding)
            keep = bar.value()
            # remember where the operator was reading
            self.log_view.setPlainText("\n".join(st.log[-600:]))
            # show the last 600 lines
            bar.setValue(bar.maximum() if at_bottom else keep)
            # auto-scroll only if they were already at the bottom
            self._log_shown_step = st
            self._log_shown_len = n

        def _edit_config(self):
            for ed in ("xdg-open", "gedit", "nano"):
                # try a graphical editor first, fall back to telling them the path
                if shutil.which(ed):
                    if ed == "nano":
                        QtWidgets.QMessageBox.information(
                            self, "Launch config",
                            f"Edit this file in a terminal, then press Reload:\n\n"
                            f"{CONFIG_PATH}")
                    else:
                        subprocess.Popen([ed, CONFIG_PATH])
                        # open the JSON in the system's default editor
                    return
            QtWidgets.QMessageBox.information(self, "Launch config", CONFIG_PATH)

        def _reload_config(self):
            self._stop_all()
            # never leave orphan processes behind when the step list changes
            self.cfg = load_config()
            self.sim_mode = bool(self.cfg.get("sim_mode", True))
            self._rebuild_steps()
            self._log_line("system", "Launch config reloaded.")

        # ── ROS callbacks (these arrive on the ROS thread) ──────
        def _ros_reply(self, text):
            self.log_sig.emit("robot", text)
            # #G1: emit only. Qt hands this to the GUI thread; we never touch the widget here.

        def _ros_status(self, js):
            try:
                self.status = json.loads(js)
                # plain attribute assignment is safe across threads; _tick reads it later
            except Exception:
                pass
                # a malformed packet must not kill the subscription

        def _ros_frame(self, data):
            self.frame_bytes = data
            # just store the JPEG bytes; decoding happens on the GUI thread in _tick
            self.frame_t = time.time()

        def _ros_voice_state(self, s):
            self.voice_state = s

        # ── logging ─────────────────────────────────────────────
        def _log_line(self, who, text):
            """Safe to call from ANY thread — it only emits a signal."""
            self.log_sig.emit(who, text)

        @QtCore.pyqtSlot(str, str)
        def _log_line_gui(self, who, text):
            """Runs on the GUI thread ONLY. This is the one place that writes
            into the conversation panel."""
            colours = {"robot": "#1F4E79", "operator": "#166534",
                       "system": "#6b7280"}
            label = {"robot": "ROBOT", "operator": "YOU", "system": "SYSTEM"}.get(
                who, "SYSTEM")
            ts = time.strftime("%H:%M:%S")
            # a timestamp on every line makes the log usable as demo evidence
            safe = html.escape(text)
            # #G5: escape < > & so a message containing "<" cannot break the layout.
            # v1 used QTextDocumentFragment.toHtml(), which is a document EXPORTER,
            # not an escaper, and injects its own block markup.
            self.conv.append(
                f'<div style="margin:3px 0"><span style="color:#9ca3af;'
                f'font-size:10px">{ts}</span> '
                f'<b style="color:{colours.get(who, "#6b7280")}">{label}</b> '
                f'<span>{safe}</span></div>')
            self.conv.verticalScrollBar().setValue(
                self.conv.verticalScrollBar().maximum())
            # always follow the newest message in the conversation panel

        # ── sending commands ────────────────────────────────────
        def _send_text(self):
            t = self.entry.text().strip()
            # strip removes stray spaces that would confuse the language model
            if not t:
                return
                # ignore an empty Enter press
            self.entry.clear()
            # clear immediately so the operator can type the next command
            self._send(t, "operator")

        def _send(self, text, who):
            self._log_line(who, text)
            # echo it in the conversation panel first, so there is always a record
            if not (self.ros and self.ros.ok):
                self._log_line("system", "ROS is not available — is the VLA agent "
                                         "running? (start it from the left panel)")
                return
            self.ros.send_command(text)
            # publish on /vla/command; the agent cannot tell this came from a GUI

        def _input_mode_changed(self, idx):
            voice = (idx == 1)
            # index 1 is "Voice command" in the dropdown
            self.btn_mic.setVisible(voice)
            self.entry.setEnabled(not voice)
            # grey out the text box in voice mode so the two paths cannot be confused
            self.btn_send.setVisible(not voice)
            if voice:
                self._log_line("system",
                               "Voice mode. Hold the gold button, speak, release. "
                               "(The 'Voice input' step must be running.)")

        def _mic_down(self):
            self.recording = True
            self.btn_mic.setText("● RECORDING — release to send")
            self.btn_mic.setStyleSheet(
                "background:#dc2626;color:white;font-weight:700;padding:9px 18px;")
            # turning the button red is the operator's proof the mic is actually live
            if not (self.ros and self.ros.ok and self.ros.set_voice_trigger(True)):
                self._log_line("system", "Cannot reach the voice node.")

        def _mic_up(self):
            self.recording = False
            self.btn_mic.setText("🎤  HOLD TO TALK")
            self.btn_mic.setStyleSheet(
                "background:#C4A02C;color:#1a1a1a;font-weight:700;padding:9px 18px;")
            if self.ros and self.ros.ok:
                self.ros.set_voice_trigger(False)
                # False tells the voice node to stop recording and transcribe
                self._log_line("system", "transcribing…")

        # ── periodic refresh ────────────────────────────────────
        def _tick(self):
            """Runs on the GUI thread every 300 ms."""
            for st in self.steps:
                st.check(self.ros)
                # re-evaluate every health probe
                st.ui["lamp"].setStyleSheet(
                    f"color:{LAMP.get(st.state, '#6b7280')};font-size:18px;")
                # repaint the lamp in the colour for its current state
            self._show_log(self.selected)
            # #G4: this now returns immediately unless the log actually grew

            # ---- live feed ----
            if self.frame_bytes is not None:
                if time.time() - self.frame_t < 3.0:
                    # a frame newer than 3 seconds counts as live video
                    pm = QtGui.QPixmap()
                    if pm.loadFromData(self.frame_bytes, "JPG"):
                        # decode the JPEG bytes into something Qt can draw
                        self.feed_label.setPixmap(pm.scaled(
                            self.feed_label.width(), self.feed_label.height(),
                            QtCore.Qt.KeepAspectRatio, QtCore.Qt.SmoothTransformation))
                        # KeepAspectRatio stops the picture being stretched
                    self.feed_note.setText(
                        f"live — {FEED_TOPIC}   (last frame "
                        f"{time.time() - self.frame_t:.1f}s ago)")
                else:
                    self.feed_note.setText(
                        f"⚠ no frame for {time.time() - self.frame_t:.0f}s")
                    # say so explicitly rather than showing a frozen picture

            # ---- status bar ----
            s = self.status
            if s:
                hop = f"  HOP×{s.get('hops', 0)}" if s.get("hop_mode") else ""
                # only shown while the agent is doing short-hop navigation (#31)
                d = s.get("target_dist")
                dtxt = f"  dist {d} m" if d is not None else ""
                self.status_bar.setText(
                    f"mode {s.get('mode', '?')}{hop}   target {s.get('target') or '-'}"
                    f"{dtxt}   map {'yes' if s.get('map') else 'no'}   "
                    f"camera {'yes' if s.get('camera') else 'no'}   "
                    f"docked {s.get('docked')}   feed {'on' if s.get('feed') else 'off'}"
                    f"   voice:{self.voice_state}   queued {s.get('queued', 0)}")
            elif self.ros and self.ros.ok:
                self.status_bar.setText("agent: waiting for /vla/status "
                                        "(start the VLA agent)")

        def closeEvent(self, ev):
            """Called when the operator closes the window."""
            self._stop_all()
            # kill every child process, so closing the GUI does not orphan Gazebo
            if self.ros:
                self.ros.shutdown()
            ev.accept()
            # accept = actually close

    return QtWidgets, Console


def main():
    try:
        from PyQt5 import QtWidgets  # noqa: F401
        # probe the import first so we can print a helpful message instead of a traceback
    except ImportError:
        print("PyQt5 is not installed:\n"
              "  sudo apt install -y python3-pyqt5")
        sys.exit(2)
    QtWidgets, Console = build_gui()
    app = QtWidgets.QApplication(sys.argv)
    # every Qt program needs exactly one QApplication
    app.setStyle("Fusion")
    # Fusion looks identical on every desktop, so demos are predictable
    w = Console()
    w.show()
    sys.exit(app.exec_())
    # exec_() runs the event loop and only returns when the window closes


if __name__ == "__main__":
    main()
    # only run main() when executed directly, not when imported by the test script