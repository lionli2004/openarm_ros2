#!/usr/bin/env python3
"""Calibration shared utilities (ROS2 layer).

Pure topic/action clients — no direct CAN. Requires the standard bimanual
launch running (trajectory controllers active). GravityModel comes from
common.py (pinocchio; run with PYTHONNOUSERSITE=1 to dodge the numpy clash).

Safety: CalibSafety checks window/velocity/stamp freshness and latches the
hardware estop (publishes -1.0 on both teaching-switch controllers) when
violated.
"""
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

ARM_JOINTS = {"right_": [f"openarm_right_joint{i}" for i in range(1, 8)],
              "left_": [f"openarm_left_joint{i}" for i in range(1, 8)]}
SWITCH_TOPICS = {"right_": "/right_teaching_switch/commands",
                 "left_": "/left_teaching_switch/commands"}
CALIB_DIR = Path.home() / "ros2_ws/experiments/teaching/data/calib"

# URDF limits per arm (rad), from the generated bimanual URDF
LIMITS = {
    "right_": ([-1.396, -0.175, -1.571, 0.0, -1.571, -0.785, -1.571],
               [3.491, 3.316, 1.571, 2.443, 1.571, 0.785, 1.571]),
    "left_": ([-3.491, -3.316, -1.571, 0.0, -1.571, -0.785, -1.571],
              [1.396, 0.175, 1.571, 2.443, 1.571, 0.785, 1.571]),
}


class TrajClient:
    """Blocking FollowJointTrajectory client for the given arm."""

    def __init__(self, node: Node, arm: str):
        self.arm = arm
        self.client = ActionClient(
            node, FollowJointTrajectory,
            f"/{arm}joint_trajectory_controller/follow_joint_trajectory")

    def wait(self, timeout=5.0):
        return self.client.wait_for_server(timeout_sec=timeout)

    def send(self, waypoint_lists, times):
        """waypoint_lists: list of 7-vecs; times: time_from_start per waypoint."""
        traj = JointTrajectory()
        traj.joint_names = ARM_JOINTS[self.arm]
        for wp, t in zip(waypoint_lists, times):
            pt = JointTrajectoryPoint()
            pt.positions = [float(v) for v in wp]
            pt.time_from_start = rclpy.duration.Duration(seconds=float(t)).to_msg()
            traj.points.append(pt)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.client._node, future, timeout_sec=10)
        if not future.done() or future.result() is None:
            return False, "goal not accepted"
        gh = future.result()
        result_future = gh.get_result_async()
        rclpy.spin_until_future_complete(self.client._node, result_future,
                                         timeout_sec=120)
        if not result_future.done():
            return False, "action timeout"
        res = result_future.result()
        if res.status != 4:  # STATUS_SUCCEEDED
            return False, f"action status {res.status}"
        return True, "ok"


class StateRecorder:
    """Subscribes /joint_states; buffers (q, dq, tau) samples with stamps."""

    def __init__(self, node: Node):
        self.node = node
        self._lock = __import__("threading").Lock()
        self.samples = []  # (stamp, q7, dq7, tau7)
        self._latest = None
        self.recording = False  # must exist before the first message arrives
        node.create_subscription(JointState, "/joint_states", self._cb, 10)

    def _cb(self, msg):
        name_to_idx = {n: i for i, n in enumerate(msg.name)}
        q = [0.0] * 7
        dq = [0.0] * 7
        tau = [0.0] * 7
        for i in range(7):
            for side in ("left_", "right_"):
                jn = f"openarm_{side}joint{i + 1}"
                if jn in name_to_idx:
                    k = name_to_idx[jn]
                    q[i] = msg.position[k]
                    dq[i] = msg.velocity[k]
                    tau[i] = msg.effort[k]
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        with self._lock:
            self._latest = (t, q, dq, tau)
            if self.recording:
                self.samples.append((t, list(q), list(dq), list(tau)))

    @property
    def latest(self):
        with self._lock:
            return self._latest

    def start(self):
        with self._lock:
            self.samples = []
            self.recording = True

    def stop(self):
        with self._lock:
            self.recording = False
            out = list(self.samples)
        return out


class CalibSafety:
    """Window / velocity / stamp checks; estop latch on violation."""

    def __init__(self, node: Node, arm: str, q_limits, max_dq=0.5,
                 estop_timeout_ms=300):
        self.node = node
        self.q_lower, self.q_upper = q_limits
        self.max_dq = max_dq
        self.tripped = False
        self.reason = ""
        self._pubs = {s: node.create_publisher(Float64MultiArray, t, 10)
                      for s, t in SWITCH_TOPICS.items()}

    def check(self, rec: StateRecorder):
        if self.tripped:
            return False
        latest = rec.latest
        if latest is None:
            return True  # no data yet
        t, q, dq, _ = latest
        now = time.time()
        if abs(now - t) > 2.0:  # stale joint_states
            self.trip("stale joint_states")
            return False
        if max(abs(v) for v in dq) > self.max_dq:
            self.trip(f"speed {max(abs(v) for v in dq):.2f} > {self.max_dq}")
            return False
        for i in range(7):
            if q[i] < self.q_lower[i] or q[i] > self.q_upper[i]:
                self.trip(f"joint{i+1} q={q[i]:.3f} outside limits")
                return False
        return True

    def trip(self, reason):
        if self.tripped:
            return
        self.tripped = True
        self.reason = reason
        self.node.get_logger().error(f"CALIB SAFETY TRIP: {reason} — estop")
        for side, pub in self._pubs.items():
            msg = Float64MultiArray()
            msg.data = [-1.0]
            pub.publish(msg)


def make_triangle(joint_idx, q_now, q_star, delta, v, hold):
    """Triangle waypoints through q*: now → q*-δ → q*+δ → q*-δ.

    Segment 1 starts at the current pose (only joint moves, others hold).
    Returns (waypoints, times). v = segment speed (rad/s), delta = half-span.
    """
    seg1_len = abs((q_star - delta) - q_now[joint_idx])
    seg = 2 * delta
    t1 = max(seg1_len, 0.01) / v
    tseg = seg / v
    wps = [list(q_now)]
    times = [0.0]
    for target in (q_star - delta, q_star + delta, q_star - delta):
        wp = list(hold)
        wp[joint_idx] = target
        wps.append(wp)
    times = [0.0, t1, t1 + tseg, t1 + 2 * tseg]
    return wps, times


def segment_waypoints(joint_idx, q_now, q_star, delta, v, hold, n=6):
    """Linear-interpolated waypoints for one straight segment to q_target."""
    target = list(hold)
    target[joint_idx] = q_star
    wps = []
    times = []
    for k in range(1, n + 1):
        f = k / n
        wp = [q_now[i] + f * (target[i] - q_now[i]) for i in range(7)]
        wps.append(wp)
        times.append(f * (abs(q_star - q_now[joint_idx]) / v))
    return wps, times
