#!/usr/bin/env python3
"""Gravity calibration: slow triangle sweeps + sign-paired extraction.

For each joint j and pose q*: trajectory controller drives a triangle
[q*-δ] → [q*+δ] → [q*-δ] at v=0.08 rad/s while we record /joint_states.
Analysis (analyze_calib.py) pairs +/− direction samples at the same q:
    g_actual(q) = (τ_read,+ + τ_read,−)/2        (friction cancels exactly)
and regresses g_actual = k_j·g_model(q) + b0 per joint.

Usage:  PYTHONNOUSERSITE=1 python3 calib_gravity.py --arm right_ [--dry-run]
Requires the bimanual launch running (teaching_mode:=false). Keep clear of
the arm — it moves on its own.
"""
import argparse
import csv
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import rclpy
from rclpy.node import Node

from calib_common import (ARM_JOINTS, CALIB_DIR, LIMITS, CalibSafety,
                          StateRecorder, TrajClient, make_triangle)

V = 0.08      # rad/s sweep speed
DELTA = 0.05  # half-span around each pose
# Pose sets per joint (J3/J5 skipped: model gravity ≡ 0 there)
POSES = {
    "right_": {
        0: [-1.2, -0.6, 0.0, 0.6, 1.2],
        1: [-0.10, 0.30, 0.60, 0.90, 1.20],
        3: [0.15, 0.60, 1.10, 1.60, 2.00, 2.30],
        # J6 range reduced: the real mechanics hit the central column beyond
        # about ±0.2 rad (URDF limit ±0.785 is NOT achievable on the rig)
        5: [-0.20, -0.10, 0.00, 0.10, 0.20],
        6: [-1.2, -0.6, 0.0, 0.6, 1.2],
    },
    "left_": {
        0: [-1.2, -0.6, 0.0, 0.6, 1.2],
        1: [-1.20, -0.90, -0.60, -0.30, 0.10],
        3: [0.15, 0.60, 1.10, 1.60, 2.00, 2.30],
        5: [-0.20, -0.10, 0.00, 0.10, 0.20],
        6: [-1.2, -0.6, 0.0, 0.6, 1.2],
    },
}


def save_samples(path, samples):
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t"] + [f"q{i+1}" for i in range(7)] +
                   [f"dq{i+1}" for i in range(7)] +
                   [f"tau{i+1}" for i in range(7)])
        for t, q, dq, tau in samples:
            w.writerow([t] + q + dq + tau)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--arm", default="right_", choices=["right_", "left_"])
    parser.add_argument("--joints", default="",
                        help="comma list of joint indices to scan (e.g. 5 for J6); "
                             "empty = all")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()
    if args.joints:
        poses = {int(j) - 1: POSES[args.arm][int(j) - 1]
                 for j in args.joints.split(",") if int(j) - 1 in POSES[args.arm]}
    else:
        poses = POSES[args.arm]

    rclpy.init()
    node = Node(f"calib_gravity_{args.arm.rstrip('_')}")
    traj = TrajClient(node, args.arm)
    rec = StateRecorder(node)
    lo, hi = LIMITS[args.arm]
    safety = CalibSafety(node, args.arm, (lo, hi))
    run_dir = CALIB_DIR / time.strftime("%Y%m%d-%H%M%S") / "gravity"
    print(f"arm={args.arm}  speed={V} rad/s  dry_run={args.dry_run}")
    print("SAFETY: arm moves automatically — keep clear! Ctrl+C aborts.")
    if not args.dry_run:
        input("press Enter when the arm area is clear...")

    if not traj.wait():
        print("ERROR: trajectory controller action not available")
        return 1

    # current pose
    q_now = [0.0] * 7
    if not args.dry_run:
        for _ in range(20):
            rclpy.spin_once(node, timeout_sec=0.1)
            latest = rec.latest
            if latest is not None:
                q_now = list(latest[1])
                break

    def to_zero(q_now, v=0.08):
        """Single-waypoint trajectory to all zeros (controller interpolates)."""
        dist = max(abs(q) for q in q_now)
        t = max(dist / v, 0.5)
        return [[0.0] * 7], [t]

    try:
        for joint, joint_poses in poses.items():
            if args.dry_run:
                print(f"J{joint+1}: poses {joint_poses} (dry-run)")
                continue
            print(f"--- J{joint+1} ---")
            # return to zero first: residual poses in other joints couple into
            # this joint's gravity torque (analyze uses the recorded q7, but
            # cleaner data makes the regression more robust)
            if rec.latest is not None:
                q_now = list(rec.latest[1])
            wps, times = to_zero(q_now)
            rec.start()
            traj.send(wps, times)
            rec.stop()
            if rec.latest is not None:
                q_now = list(rec.latest[1])
            for qi, q_star in enumerate(joint_poses):
                rclpy.spin_once(node, timeout_sec=0.1)
                if not safety.check(rec):
                    print(f"SAFETY TRIP at J{joint+1} pose {q_star}: {safety.reason}")
                    return 1
                if rec.latest is not None:
                    q_now = list(rec.latest[1])
                wps, times = make_triangle(joint, q_now, q_star, DELTA, V, q_now)
                rec.start()
                ok, msg = traj.send(wps, times)
                samples = rec.stop()
                if not ok:
                    print(f"  q*={q_star:.2f}: traj {msg} — aborting")
                    return 1
                if not safety.check(rec):
                    print(f"SAFETY TRIP after J{joint+1} q*={q_star:.2f}")
                    return 1
                fname = f"{args.arm}J{joint+1}_p{qi}.csv"
                save_samples(run_dir / "raw" / fname, samples)
                print(f"  q*={q_star:+.2f} ok ({len(samples)} samples)")
    finally:
        node.destroy_node()
        rclpy.shutdown()
    print(f"\nDone. Data in {run_dir}")
    print("Next: PYTHONNOUSERSITE=1 python3 analyze_calib.py --arm "
          f"{args.arm} --dir {run_dir}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
