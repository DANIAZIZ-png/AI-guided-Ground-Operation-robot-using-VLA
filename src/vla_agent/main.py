"""Entry point: build the node, spin it, shut it down cleanly.

`main` is also the console-script entry point declared in pyproject.toml, so
after `pip install -e src` the agent starts with `vla-agent`.

Clean shutdown matters here and is not cosmetic: a ROS client that exits without
rclpy.shutdown() leaves a ghost in every other node's view of the graph until
the ROS daemon is restarted.
"""
from .core import *                 # noqa: F401,F403
from .core import print             # noqa: A001 - headless-safe console, fix #63
from .agent import VLAAgent


def main():
    rclpy.init()
    node = VLAAgent()

    # #33: mirror the terminal onto /vla/reply, so a GUI operator with zero
    # technical knowledge sees exactly what the engineer at the terminal sees.
    real_stdout = sys.stdout
    sys.stdout = ReplyTee(real_stdout, node.emit_reply)

    # #32b: MULTI-threaded executor. rclpy.spin() is single-threaded and every
    # timer sits in one mutually-exclusive group by default, so think() —
    # which blocks on an HTTP call to YOLO — used to stall publish_cmd() (the
    # 10 Hz velocity heartbeat) and annotate_think() (the live feed) with it.
    # That starvation is the "feed runs for a while then gets stuck" symptom.
    # Six threads: four timers + subscriptions + action callbacks, with slack.
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        node.run_input_loop()
    finally:
        node.shutdown = True
        node.cancel_task()
        node.mlog.log("RUN", "agent shutting down")
        try:
            executor.shutdown()
        except Exception:
            pass
        sys.stdout = real_stdout
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
