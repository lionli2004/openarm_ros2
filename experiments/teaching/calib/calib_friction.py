#!/usr/bin/env python3
"""Friction calibration: constant-speed triangles + breakaway (τ_s) ramps.

Per joint j at a fixed pose q*:
  - breakaway: inject a τ_ff ramp via the calib_tau controller (0.5 Nm/s),
    first motion |dq|>0.01 → τ_s = τ_read − k_j·g_model(q*)   [diagnostic]
  - constant-speed triangles [q*−δ]→[q*+δ]→[q*−δ] at speeds S[j],
    2 trials each; friction = τ_read − k_j·g_model(q)

Requires:
  - bimanual launch running, teaching_mode:=false
  - calib_tau controllers spawned:
      ros2 run controller_manager spawner left_calib_tau right_calib_tau -c /controller_manager
  - gravity calibration done (params_<arm>.txt with grav_k) — pass --params

Usage:  PYTHONNOUSERSITE=1 python3 calib_friction.py --arm right_ \
            --params <run_dir>/params_right_.txt [--dry-run]
"""
import argparse
import csv
import json
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from calib_common import (CALIB_DIR, CalibSafety, StateRecorder, TrajClient,
                          make_triangle)
from common import GravityModel

DELTA = 0.06
SPEEDS = {  # rad/s per joint group
    "fast": [0.05, 0.10, 0.20, 0.40],   # J1/J2 DM8009
    "slow": [0.02, 0.05, 0.10, 0.20],   # J3-J7
}
# calibration pose q* per joint (rest of the arm at zero)
# NOTE J6 (index 5): real mechanics hit the central column beyond ±0.2 rad,
# so its pose is 0.1 (URDF limit ±0.785 is not achievable on the rig)
CALIB_POSES = [0.3, 0.6, 0.4, 1.2, 0.4, 0.1, 0.4]
RAMP_RATE = 0.5   # Nm/s breakaway ramp
RAMP_CAP = 15.0   # Nm
FIRST_MOVE = 0.01  # rad/s


def load_grav_k(params_file):
    if not params_file:
        return [1.0] * 7
    k = [1.0] * 7
    for line in Path(params_file).read_text().splitlines():
        parts = line.split()
        if parts and parts[0] == "grav_k" and len(parts) == 8:
            k = [float(v) for v in parts[1:8]]
    return k


def save_csv(path, header, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(header)
        w.writerows(rows)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--arm", default="right_", choices=["right_", "left_"])
    parser.add_argument("--params", default="", help="params file with grav_k")
    parser.add_argument("--joints", default="",
                        help="comma list of joint indices (1-based, e.g. 6 for J6); "
                             "empty = all")
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()
    joints_to_scan = ([int(j) - 1 for j in args.joints.split(",")]
                      if args.joints else list(range(7)))

    rclpy.init()
    node = Node(f"calib_friction_{args.arm.rstrip('_')}")
    traj = TrajClient(node, args.arm)
    rec = StateRecorder(node)
    from calib_common import LIMITS  # shared limit table
    lo, hi = LIMITS[args.arm]
    safety = CalibSafety(node, args.arm, (lo, hi))
    gm = GravityModel(arm=args.arm)
    grav_k = load_grav_k(args.params)
    run_dir = CALIB_DIR / time.strftime("%Y%m%d-%H%M%S") / "friction"
    print(f"arm={args.arm}  grav_k={grav_k}")
    print("SAFETY: arm moves automatically — keep clear!")
    if not args.dry_run:
        input("press Enter when clear...")

    if not traj.wait():
        print("ERROR: trajectory controller action not available")
        return 1

    # current pose (for initial move)
    q_now = [0.0] * 7
    if not args.dry_run:
        for _ in range(20):
            rclpy.spin_once(node, timeout_sec=0.1)
            if rec.latest is not None:
                q_now = list(rec.latest[1])
                break

    # calib_tau publisher (effort injection)
    tau_pub = node.create_publisher(
        Float64MultiArray, f"/{args.arm}calib_tau/commands", 10)

    def publish_tau(values):
        msg = Float64MultiArray()
        msg.data = [float(v) for v in values]
        tau_pub.publish(msg)

    try:
        all_rows = []
        breakaway = {}
        for j in joints_to_scan:
            if args.dry_run:
                print(f"J{j+1}: speeds={SPEEDS['slow' if j >= 2 else 'fast']} "
                      f"q*={CALIB_POSES[j]}")
                continue
            speeds = SPEEDS["slow" if j >= 2 else "fast"]
            q_star = CALIB_POSES[j]
            print(f"--- J{j+1} (q*={q_star}, speeds={speeds}) ---")

            # ---- breakaway τ_s ----
            if not safety.check(rec):
                print(f"SAFETY TRIP: {safety.reason}"); return 1
            if rec.latest is not None:
                q_now = list(rec.latest[1])
            wps, times = make_triangle(j, q_now, q_star, 0.02, 0.05, q_now)
            ok, msg = traj.send(wps, times)
            if not ok:
                print(f"  move to q* failed: {msg}"); return 1
            tau_s = {}
            for direction in (1, -1):
                ramp = 0.0
                moved = False
                t0 = time.time()
                while ramp < RAMP_CAP and time.time() - t0 < 60:
                    vals = [0.0] * 7
                    vals[j] = direction * ramp
                    publish_tau(vals)
                    for _ in range(3):
                        rclpy.spin_once(node, timeout_sec=0.05)
                    latest = rec.latest
                    if latest is None:
                        continue
                    q, dq, tau = latest[1], latest[2], latest[3]
                    if abs(dq[j]) > FIRST_MOVE:
                        q7 = [0.0] * 7
                        q7[j] = q[j]
                        g = gm.compute(q7)[j]
                        tau_s[direction] = tau[j] - grav_k[j] * g
                        moved = True
                        break
                    ramp += RAMP_RATE * 0.15
                publish_tau([0.0] * 7)
                if not moved:
                    tau_s[direction] = None
                print(f"  τ_s({'+' if direction > 0 else '-'}) = "
                      f"{tau_s[direction] if tau_s[direction] is not None else '>cap'}")
                # pull back to q*
                if rec.latest is not None:
                    q_now = list(rec.latest[1])
                wps, times = make_triangle(j, q_now, q_star, 0.02, 0.05, q_now)
                traj.send(wps, times)
            breakaway[f"J{j+1}"] = tau_s

            # ---- constant-speed triangles ----
            for trial in range(2):
                for speed in speeds:
                    if not safety.check(rec):
                        print(f"SAFETY TRIP: {safety.reason}"); return 1
                    if rec.latest is not None:
                        q_now = list(rec.latest[1])
                    wps, times = make_triangle(j, q_now, q_star, DELTA, speed, q_now)
                    rec.start()
                    ok, msg = traj.send(wps, times)
                    samples = rec.stop()
                    if not ok:
                        print(f"  speed {speed}: {msg}"); return 1
                    for t, q, dq, tau in samples:
                        q7 = [0.0] * 7
                        q7[j] = q[j]
                        g = gm.compute(q7)[j]
                        all_rows.append([j + 1, speed, trial, t] + q + dq + tau +
                                        [grav_k[j] * g])
                    print(f"  speed {speed:.2f} trial {trial} ok "
                          f"({len(samples)} samples)")
        if args.dry_run:
            return 0
        save_csv(run_dir / f"friction_{args.arm}.csv",
                 ["joint", "speed", "trial", "t"] +
                 [f"q{i}" for i in range(1, 8)] +
                 [f"dq{i}" for i in range(1, 8)] +
                 [f"tau{i}" for i in range(1, 8)] + ["g_ff"],
                 all_rows)
        with open(run_dir / f"breakaway_{args.arm}.json", "w") as f:
            json.dump(breakaway, f, indent=2)
        print(f"\nData in {run_dir}")
        print("Next: PYTHONNOUSERSITE=1 python3 analyze_friction.py "
              f"--arm {args.arm} --dir {run_dir} --params {args.params}")
    finally:
        publish_tau([0.0] * 7)
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
