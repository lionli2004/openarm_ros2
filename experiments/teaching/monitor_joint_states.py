#!/usr/bin/env python3
"""Read-only /joint_states monitor: prints name:position per line.

Usage:  python3 monitor_joint_states.py [--rate 2.0]
"""
import argparse

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class Monitor(Node):
    def __init__(self):
        super().__init__("joint_states_monitor")
        self._last = {}

        def cb(msg):
            line = " | ".join(
                f"{n}:{v:+.3f}" for n, v in zip(msg.name, msg.position))
            # only print when something changed by >0.0005 rad
            changed = any(
                abs(v - self._last.get(n, 1e9)) > 0.0005
                for n, v in zip(msg.name, msg.position))
            if changed:
                print(line, flush=True)
            self._last = dict(zip(msg.name, msg.position))

        self.sub = self.create_subscription(JointState, "/joint_states", cb, 10)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--rate", type=float, default=2.0)
    args = parser.parse_args()
    rclpy.init()
    node = Monitor()
    rate = node.create_rate(args.rate)
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.05)
        rate.sleep()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
