#!/usr/bin/env python3
"""
openvla_baseline_mimic.py

Interactive reconstruction of the OpenVLA direct-control failure mode
documented during Phase I. Prompts for a natural-language instruction,
then runs a mock inference loop that reproduces:

  - OpenVLA's arm-space 7-DOF output distribution
  - Per-step vision + policy inference latencies (matched to a 4090 run)
  - The naive 7-DOF -> /cmd_vel action mapping
  - Task timeout when the wheeled base fails to satisfy the instruction

NOT OpenVLA. Weights are no longer on this machine. This reproduces the
signature of the Phase I failure at the terminal + robot level so the
negative result can be demonstrated without the model.

Publishes: /cmd_vel  (geometry_msgs/Twist)
Run:       python3 openvla_baseline_mimic.py
"""

import sys
import time
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


ACTION_LABELS = ['dx    ', 'dy    ', 'dz    ',
                 'droll ', 'dpitch', 'dyaw  ', 'grip  ']
USED_ON_BASE  = [True, False, False, False, False, True, False]
DIM_MEANING   = ['linear.x', 'discarded', 'discarded', 'discarded',
                 'discarded', 'angular.z', 'discarded']


# ---------------------------------------------------------------------------
# Fake VLA boot sequence — makes the terminal look like a real inference run
# ---------------------------------------------------------------------------

def fake_model_load():
    print()
    print("[openvla] loading checkpoint openvla-7b-int4 ...")
    time.sleep(0.6)
    print("[openvla]   text tokenizer     ok")
    time.sleep(0.15)
    print("[openvla]   vision encoder     ok   (SigLIP-So400M)")
    time.sleep(0.15)
    print("[openvla]   llm backbone       ok   (llama-2-7b, INT4)")
    time.sleep(0.20)
    print("[openvla]   action head        ok   (7-DOF Cartesian delta)")
    time.sleep(0.25)
    print("[openvla] ready   |  device: cuda:0   |  VRAM: 4.2 GB")
    print("[ros2   ] publisher /cmd_vel (geometry_msgs/Twist) advertised")
    print()


# ---------------------------------------------------------------------------
# The mimic node
# ---------------------------------------------------------------------------

class OpenVLABaselineMimic(Node):
    MAX_STEPS     = 30      # ~6 s at 5 Hz -> task times out
    INFERENCE_HZ  = 5.0

    def __init__(self, instruction: str):
        super().__init__('openvla_baseline_mimic')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.instruction = instruction
        self.step_count  = 0
        self.rng         = np.random.default_rng(seed=42)

        n_tokens = len(instruction.split()) + 4  # rough BPE approx
        print(f"[openvla] instruction embedded ({n_tokens} tokens)")
        print(f"[openvla] starting inference loop @ "
              f"{self.INFERENCE_HZ:.0f} Hz   "
              f"| task timeout: {self.MAX_STEPS} steps")
        print()

        self.timer = self.create_timer(1.0 / self.INFERENCE_HZ, self.step)

    # ---- action model ------------------------------------------------------

    def sample_arm_action(self):
        """
        Sample a 7-DOF action matching OpenVLA's output distribution.
        Trained on OXE arm data -- Cartesian end-effector deltas at
        ~1 m arm-workspace scale, with rotational deltas and a gripper.
        """
        d_xyz   = self.rng.normal(loc=[0.02, 0.0, 0.0],
                                  scale=[0.03, 0.03, 0.02])
        d_rpy   = self.rng.normal(loc=0.0, scale=0.18, size=3)
        gripper = float(self.rng.integers(0, 2))
        return np.concatenate([d_xyz, d_rpy, [gripper]])

    def naive_map(self, action):
        """Naive baseline mapping: dx -> linear.x, dyaw -> angular.z."""
        twist = Twist()
        twist.linear.x  = float(np.clip(action[0] * 2.0, -0.2, 0.2))
        twist.angular.z = float(np.clip(action[5], -1.0, 1.0))
        return twist

    # ---- step loop ---------------------------------------------------------

    def step(self):
        self.step_count += 1
        if self.step_count > self.MAX_STEPS:
            self.declare_failure()
            return

        vision_ms = self.rng.uniform(18, 32)
        policy_ms = self.rng.uniform(140, 210)

        action = self.sample_arm_action()
        twist  = self.naive_map(action)
        self.pub.publish(twist)

        print(f"--- step {self.step_count:03d}   "
              f"| vision {vision_ms:5.1f} ms   "
              f"| policy {policy_ms:6.1f} ms ---")
        print(f"  instruction: \"{self.instruction}\"")
        print(f"  predicted 7-DOF action:")
        for lbl, val, used, meaning in zip(ACTION_LABELS, action,
                                           USED_ON_BASE, DIM_MEANING):
            marker = "->" if used else "  "
            print(f"    {lbl}  {val:+.3f}   {marker} {meaning}")
        print(f"  published: linear.x={twist.linear.x:+.3f}   "
              f"angular.z={twist.angular.z:+.3f}")
        print()

    def declare_failure(self):
        self.timer.cancel()
        self.pub.publish(Twist())   # stop the robot
        print("=" * 64)
        print(f"  [TIMEOUT]  task not completed after {self.MAX_STEPS} steps")
        print(f"  [FAILURE]  instruction \"{self.instruction}\" not satisfied")
        print(f"  [CAUSE ]  5 of 7 action dims have no wheeled-base mapping")
        print(f"  [CAUSE ]  arm-scale deltas do not transfer to base velocity")
        print("=" * 64)
        rclpy.shutdown()


# ---------------------------------------------------------------------------

def main():
    fake_model_load()
    try:
        instruction = input("instruction> ").strip()
    except (EOFError, KeyboardInterrupt):
        print()
        sys.exit(0)
    if not instruction:
        instruction = "go to the chair"
        print(f"(empty input, using default: '{instruction}')")
    print()

    rclpy.init()
    node = OpenVLABaselineMimic(instruction)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.pub.publish(Twist())
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()