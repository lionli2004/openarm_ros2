#!/usr/bin/env python3
"""Experiment 2 — per-joint friction identification (low-speed torque ramp).

For each joint j (others held at 0 with teaching-level gain + τ_ff comp):
  - τ_ff(j) = g_py(j) + τ_ramp, ramp grows at 0.5 Nm/s from 0
  - static friction τ_s = first motion point (|dq| > 0.01 rad/s)
  - record net friction torque at |dq| ∈ {0.02, 0.05, 0.1, 0.2} rad/s
  - same downward (negative ramp), repeated twice per joint for repeatability

Output: data/exp2_friction/<ts>/*.csv + summary.json with
        τ_s±, τ_c±, B± per joint (output-side Nm).

Supports the hover criterion: hovering is stable where static friction
exceeds the gravity-model error.

Usage:  PYTHONNOUSERSITE=1 sudo python3 exp2_friction.py --iface can0 [--dry-run]
"""
import time

import numpy as np

from common import (Arm, GravityModel, Safety, TMAX, interactive_prompt,
                    make_run_dir, parse_args, save_csv, save_json)

# Per-joint hold gains for the non-scanning joints (same rationale as exp1)
HOLD_KP = [20.0, 100.0, 100.0, 100.0, 40.0, 40.0, 40.0]
HOLD_KD = [0.5, 2.5, 2.5, 2.5, 1.0, 1.0, 1.0]
RAMP_RATE = 0.5        # Nm/s
TARGET_DQS = [0.02, 0.05, 0.1, 0.2]
FIRST_MOVE_DQ = 0.01   # rad/s
LOOP_DT = 0.01         # 10 ms control loop (fast enough to avoid limit cycles)
WINDOW = 0.35          # rad, soft position window around scan start


def ramp_scan(arm, gm, safety, j, direction, run_dir, tag, loop_dt=LOOP_DT,
              max_duration_s=45.0):
    """Torque ramp on joint j in `direction` (±1). Returns (tau_s, points)."""
    cap = min(0.5 * TMAX[j], 15.0)  # ramp cap (output Nm)
    q, dq, _ = arm.refresh_recv()
    q_start = list(q)  # full 7-vector for the safety window check
    tau_net = 0.0
    moved = False
    tau_s = None
    speed_points = {}  # target dq -> tau_net
    rows = []
    t0 = time.monotonic()

    while True:
        if time.monotonic() - t0 > max_duration_s:
            print(f"    {tag} scan timeout after {max_duration_s:.0f} s")
            break
        g = gm.compute(q)
        tau_ff = [0.0] * 7
        for i in range(7):
            if i == j:
                tau_ff[i] = g[i] + tau_net
            else:
                tau_ff[i] = g[i]  # compensate others at zero pose
        # hold others at their start pose with low gain; ramp joint with Kp=0
        params = []
        for i in range(7):
            if i == j:
                params.append((0.0, 0.3, q[i], 0.0, tau_ff[i]))
            else:
                params.append((HOLD_KP[i], HOLD_KD[i], q[i], 0.0, tau_ff[i]))
        arm.mit_control_all(params)
        q, dq, tau_read = arm.refresh_recv()
        safety.heartbeat()
        ok, msg = safety.check_pose(q, dq, q_start)
        if not ok:
            break
        if abs(q[j] - q_start[j]) > WINDOW:
            print(f"    WINDOW exceeded J{j+1}: {q[j]-q_start[j]:+.3f} rad — stopping scan")
            break

        rows.append((time.monotonic(), q[j], dq[j], tau_read[j],
                     tau_net, g[j]))

        if not moved and abs(dq[j]) > FIRST_MOVE_DQ:
            moved = True
            tau_s = tau_net
            print(f"    {tag} first motion at τ_net={tau_net:+.3f} Nm (static friction)")
        for td in TARGET_DQS:
            if moved and td not in speed_points and abs(dq[j]) >= td:
                speed_points[td] = tau_net
                print(f"    {tag} |dq|={td:.2f} → τ_net={tau_net:+.3f} Nm")

        # advance ramp (loop_dt=0 in dry-run: step with nominal dt, no sleep)
        dt = loop_dt if loop_dt > 0 else LOOP_DT
        tau_net += direction * RAMP_RATE * dt
        if abs(tau_net) >= cap or (moved and len(speed_points) == len(TARGET_DQS)):
            break
        if loop_dt > 0:
            time.sleep(loop_dt)

    return tau_s, speed_points, rows


def main():
    args = parse_args("Experiment 2 — per-joint friction identification")
    print(f"Interface: {args.iface}  arm: {args.arm}  dry_run: {args.dry_run}")
    print("SAFETY: single-joint scans at low speed; keep clear of the joint; "
          "e-stop = power switch / CLI disable / 'q'.")

    arm = Arm(args.iface, dry_run=args.dry_run)
    gm = GravityModel(arm=args.arm)
    safety = Safety(arm, log=print)
    run_dir = make_run_dir("exp2_friction")

    def keepalive():
        """Hold current pose + heartbeat while waiting for operator input."""
        q, _, _ = arm.refresh_recv()
        arm.mit_control_all([(HOLD_KP[i], HOLD_KD[i], q[i], 0.0, 0.0)
                             for i in range(7)])
        arm.refresh_recv()
        safety.heartbeat()

    try:
        arm.enable()
        q, _, _ = arm.refresh_recv()
        print(f"current q = {[f'{v:+.3f}' for v in q]}")
        interactive_prompt("verify pose matches printed q, then Enter...",
                           keepalive, args.dry_run)

        summary = {}
        all_rows = []
        for j in range(7):
            print(f"\n=== Joint J{j+1} ===")
            # move to zero pose for this joint
            target = [0.0] * 7
            for i in range(7):
                target[i] = q[i]
            target[j] = 0.0
            # interpolate move
            q0, _, _ = arm.refresh_recv()
            for s in range(1, 201):
                t = s / 200
                qd = [q0[i] + t * (target[i] - q0[i]) for i in range(7)]
                g = gm.compute(qd)
                arm.mit_control_all([(HOLD_KP[i], HOLD_KD[i], qd[i], 0.0, g[i])
                                     for i in range(7)])
                arm.refresh_recv()
                safety.heartbeat()
                time.sleep(0.005)

            for rep in (1, 2):
                for direction, tag in ((1, "J+"), (-1, "J-")):
                    print(f"  ramp {tag} (rep {rep})...")
                    tau_s, sp, rows = ramp_scan(arm, gm, safety, j, direction,
                                                run_dir, tag,
                                                loop_dt=0.0 if args.dry_run else LOOP_DT)
                    for r in rows:
                        all_rows.append([j + 1, tag, rep] + list(r))
                    key = f"J{j+1}{tag}"
                    summary[key] = {"tau_s": tau_s,
                                    "speed_points": sp,
                                    "tau_cap": min(0.5 * TMAX[j], 15.0)}
                    if tau_s is None:
                        print(f"    {tag}: NO motion up to cap → not backdrivable "
                              f"within {min(0.5*TMAX[j],15.0):.1f} Nm")
                    safety.heartbeat()
                    # return joint to scan start (reverse, assisted by Kp hold)
                    qq, _, _ = arm.refresh_recv()
                    g = gm.compute(qq)
                    arm.mit_control_all([(HOLD_KP[i], HOLD_KD[i], qq[i], 0.0, g[i])
                                         for i in range(7)])
                    arm.refresh_recv()
                    time.sleep(0.3)

        save_csv(run_dir, "scan.csv", all_rows,
                 ["joint", "dir", "rep", "t_s", "q_rad", "dq_rads", "tau_read_Nm",
                  "tau_net_cmd_Nm", "g_py_Nm"])
        save_json(run_dir, "summary.json", summary)
        print(f"\nData written to {run_dir}")
        for k, v in summary.items():
            print(f"  {k}: τ_s={v['tau_s']}")
    finally:
        safety.stop()
        print("Motors disabled. Done.")


if __name__ == "__main__":
    main()
