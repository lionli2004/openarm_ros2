#!/usr/bin/env python3
"""OpenArm drag-teaching demo controller.

Orchestrates the record → replay demo:
  - set_mode:    teaching (drag) ↔ normal (replay) via the hardware-layer
                 runtime switch (gripper-effort interface, forward controllers)
  - estop:       latched emergency disable in the hardware layer
  - record:      capture /joint_states into a CSV trajectory
  - replay:      send the trajectory to both arm trajectory controllers

Services:
  ~/set_mode      std_srvs/SetBool   (true=teaching, false=normal)
  ~/estop         std_srvs/Trigger
  ~/record_start  std_srvs/Trigger
  ~/record_stop   std_srvs/Trigger   (returns CSV path in message)
  ~/replay        openarm_demo/srv/Replay
  ~/replay_stop   std_srvs/Trigger
  ~/status        openarm_demo/srv/Status
Topics:
  ~/status_text   std_msgs/String    (for the RViz panel)
"""
import csv
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import SetBool, Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from openarm_demo.srv import Replay, Status

ARM_JOINTS = {"left": [f"openarm_left_joint{i}" for i in range(1, 8)],
              "right": [f"openarm_right_joint{i}" for i in range(1, 8)]}
TRAJ_DIR = os.path.expanduser("~/ros2_ws/experiments/teaching/data/demo")


class DemoController(Node):
    def __init__(self):
        super().__init__("openarm_demo_controller")
        os.makedirs(TRAJ_DIR, exist_ok=True)

        # Hardware-layer mode switch (gripper-effort interface via forward
        # controllers): 1.0 → teaching, 0.0 → normal, -1.0 → estop
        self._switch_pubs = {
            "left": self.create_publisher(Float64MultiArray,
                                          "/left_teaching_switch/commands", 10),
            "right": self.create_publisher(Float64MultiArray,
                                           "/right_teaching_switch/commands", 10),
        }

        # Recording state (t is seconds since the first captured message)
        self._recording = False
        self._rec_start = None
        self._rec_samples = []  # (t, {joint_name: pos})

        # Replay state
        self._replaying = False
        self._traj_path = ""
        self._goal_handles = {}  # side -> goal handle (kept for cancel)
        self._replay_clients = {
            side: ActionClient(
                self, FollowJointTrajectory,
                f"/{side}_joint_trajectory_controller/follow_joint_trajectory")
            for side in ("left", "right")
        }

        # Services
        self.create_service(SetBool, "~/set_mode", self.cb_set_mode)
        self.create_service(Trigger, "~/estop", self.cb_estop)
        self.create_service(Trigger, "~/record_start", self.cb_record_start)
        self.create_service(Trigger, "~/record_stop", self.cb_record_stop)
        self.create_service(Replay, "~/replay", self.cb_replay)
        self.create_service(Trigger, "~/replay_stop", self.cb_replay_stop)
        self.create_service(Status, "~/status", self.cb_status)

        self._status_pub = self.create_publisher(String, "~/status_text", 10)
        self._js_sub = self.create_subscription(
            JointState, "/joint_states", self.cb_joint_states, 10)

        self.get_logger().info(
            "demo controller ready — trajectories in %s" % TRAJ_DIR)
        self.get_logger().warn(
            "NOTE: launch openarm.bimanual.launch.py with teaching_mode:=false "
            "for demo use — the panel controls teaching/normal. With "
            "teaching_mode:=true the hardware stays in teaching mode and "
            "replay is ignored.")

    # ------------------------------------------------------------- helpers
    def _switch(self, side, value):
        msg = Float64MultiArray()
        msg.data = [float(value)]
        self._switch_pubs[side].publish(msg)

    def _set_mode_both(self, teaching):
        for side in ("left", "right"):
            self._switch(side, 1.0 if teaching else 0.0)
        self.get_logger().info(
            "mode → %s" % ("teaching (drag)" if teaching else "normal (replay)"))

    def _publish_status(self, msg=""):
        s = self._status_pub
        s.publish(String(data=msg or self._status_line()))
        self.get_logger().info(self._status_line())

    def _status_line(self):
        return ("teaching=%s recording=%s replaying=%s traj=%s" %
                (self._teaching, self._recording, self._replaying,
                 self._traj_path or "-"))

    @property
    def _teaching(self):
        return getattr(self, "_teaching_mode", False)

    # ------------------------------------------------------------- callbacks
    def cb_joint_states(self, msg):
        if not self._recording:
            return
        st = msg.header.stamp
        if st.sec == 0 and st.nanosec == 0:
            return  # initial/fake messages may carry zero stamps — ignore
        t = st.sec + st.nanosec * 1e-9
        if self._rec_start is None:
            self._rec_start = t  # first valid message of this recording = t0
        self._rec_samples.append((t - self._rec_start,
                                  dict(zip(msg.name, msg.position))))

    def cb_set_mode(self, req, resp):
        teaching = bool(req.data)
        if teaching and self._replaying:
            # a running replay would be silently ignored by the hardware in
            # teaching mode — cancel it explicitly
            for side in ("left", "right"):
                h = self._goal_handles.get(side)
                if h is not None:
                    self._replay_clients[side]._cancel_goal_async(h)
            self._replaying = False
        self._teaching_mode = teaching
        self._set_mode_both(self._teaching_mode)
        resp.success = True
        resp.message = self._status_line()
        self._publish_status()
        return resp

    def cb_estop(self, req, resp):
        for side in ("left", "right"):
            self._switch(side, -1.0)
        resp.success = True
        resp.message = "estop sent to both arms (hardware latches)"
        self.get_logger().error(resp.message)
        return resp

    def cb_record_start(self, req, resp):
        if self._recording:
            resp.success = False
            resp.message = "already recording"
            return resp
        self._recording = True
        self._rec_start = None  # anchored to the first captured message
        self._rec_samples = []
        resp.success = True
        resp.message = "recording started — drag the arm now"
        self.get_logger().info(resp.message)
        self._publish_status()
        return resp

    def cb_record_stop(self, req, resp):
        if not self._recording:
            resp.success = False
            resp.message = "not recording"
            return resp
        self._recording = False
        if not self._rec_samples:
            resp.success = False
            resp.message = "no samples captured"
            return resp
        # save CSV with joint-name header
        ts = time.strftime("%Y%m%d-%H%M%S")
        path = os.path.join(TRAJ_DIR, f"traj_{ts}.csv")
        names = list(self._rec_samples[0][1].keys())
        with open(path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t"] + names)
            for t, s in self._rec_samples:
                w.writerow([round(t, 4)] + [round(s[n], 5) for n in names])
        self._traj_path = path
        resp.success = True
        resp.message = f"saved {len(self._rec_samples)} samples → {path}"
        self.get_logger().info(resp.message)
        self._publish_status()
        return resp

    def cb_replay(self, req, resp):
        if self._replaying:
            resp.success = False
            resp.message = "already replaying"
            return resp
        path = req.path or self._traj_path
        if not path or not os.path.exists(path):
            resp.success = False
            resp.message = f"trajectory not found: {path}"
            return resp
        # parse CSV
        with open(path) as f:
            reader = csv.reader(f)
            header = next(reader)
            rows = list(reader)
        speed = max(0.1, min(5.0, float(req.speed)))
        goals = {}
        for side, joints in ARM_JOINTS.items():
            traj = JointTrajectory()
            traj.joint_names = joints
            for row in rows:
                t = float(row[0]) / speed
                pt = JointTrajectoryPoint()
                pt.time_from_start = rclpy.duration.Duration(seconds=t).to_msg()
                try:
                    pt.positions = [float(row[header.index(j)]) for j in joints]
                except (ValueError, IndexError):
                    continue
                traj.points.append(pt)
            goal = FollowJointTrajectory.Goal()
            goal.trajectory = traj
            goals[side] = goal
        # send both goals (wait for action servers)
        for side, goal in goals.items():
            if not self._replay_clients[side].wait_for_server(timeout_sec=5.0):
                resp.success = False
                resp.message = f"{side} trajectory controller action not available"
                return resp
        self._replaying = True
        for side, goal in goals.items():
            future = self._replay_clients[side].send_goal_async(
                goal, feedback_callback=self._feedback(side))

            def _store(fut, s=side):
                if fut.result() is not None:
                    self._goal_handles[s] = fut.result()
            future.add_done_callback(_store)
        resp.success = True
        resp.message = (f"replaying {path} at {speed:.2f}x "
                        f"({len(rows)} pts, {rows[-1][0]} s)")
        self.get_logger().info(resp.message)
        self._publish_status()
        return resp

    def _feedback(self, side):
        def cb(fb):
            pass  # keep the action alive; status is enough for the demo
        return cb

    def cb_replay_stop(self, req, resp):
        for side in ("left", "right"):
            h = self._goal_handles.get(side)
            if h is not None:
                # Humble rclpy keeps the cancel API private
                self._replay_clients[side]._cancel_goal_async(h)
        self._replaying = False
        resp.success = True
        resp.message = "replay cancelled"
        self.get_logger().info(resp.message)
        self._publish_status()
        return resp

    def cb_status(self, req, resp):
        resp.teaching_mode = self._teaching
        resp.recording = self._recording
        resp.replaying = self._replaying
        resp.trajectory_path = self._traj_path
        resp.message = self._status_line()
        return resp


def main(args=None):
    rclpy.init(args=args)
    node = DemoController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass  # launch may already have torn down the context


if __name__ == "__main__":
    main()
