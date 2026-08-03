#!/usr/bin/env python3
"""Experiment 1 — τ_ff unit & sign validation (GATE experiment).

Resolves two questions that gate everything else:
  (a) Is the Damiao MIT-frame torque in output-side Nm (so the C++ code's
      division by GEAR_RATIOS in the gravity compensation is wrong and must
      be removed), or motor-side Nm (division correct)?
  (b) Is the τ_ff sign consistent with the motor positive direction?

Method
  Phase A (arm supported by position loop, τ_ff=0): hold a set of poses
      (only one/two joints away from zero, J1 as g≈0 control), wait for
      convergence, then regress tau_readback = a·g_py(q) + b per joint.
      a≈1 → output-side (remove the /GEAR_RATIOS); a≈1/N → motor-side.
  Phase B (operator supports arm, Kp=0): at J2=45°, apply τ_ff ∈
      {0, +g, −g, +g/N, −g/N} for 5 s each and measure drift. The correct
      (unit, sign) combination drifts least.

Usage:  PYTHONNOUSERSITE=1 sudo python3 exp1_units.py --iface can0 [--dry-run]
"""
import time

import numpy as np

from common import (Arm, GravityModel, Safety, interactive_prompt,
                    make_run_dir, parse_args, save_csv, save_json)

D2R = np.deg2rad

# Phase A poses: {joint_index: target_rad} — one/two joints off zero
POSES = [
    ("J1+30", {0: D2R(30)}),
    ("J1-30", {0: D2R(-30)}),
    ("J2+30", {1: D2R(30)}),
    ("J2+60", {1: D2R(60)}),
    ("J3+20", {2: D2R(20)}),
    ("J3+40", {2: D2R(40)}),
    ("J4+15", {3: D2R(15)}),
    ("J4+30", {3: D2R(30)}),
]

# Per-joint hold gains for Phase A. With τ_ff=0 the arm must HOLD the pose:
#   - Kp=20 cannot hold J2 against gravity (observed live: 1 rad/s slide)
#   - but Kp=200 on J5-J7 (DM4310, TMAX 10 Nm) caused violent limit-cycle
#     oscillation at ~30 Hz script update rate (observed live: ±3.8 rad/s
#     buzzing). Production uses Kp=40 there; we stay conservative per joint:
#     J1 vertical axis needs little stiffness; J2-J4 carry gravity; J5-J7 low.
HOLD_KP = [20.0, 100.0, 100.0, 100.0, 40.0, 40.0, 40.0]
HOLD_KD = [0.5, 2.5, 2.5, 2.5, 1.0, 1.0, 1.0]
# Update rate matters: production runs at 100 Hz. At script rates below
# ~30 Hz the sampled position target + high stiffness limit-cycles (observed:
# J1 buzzed at Kp=50 with ~20 Hz updates; J5/J7 buzzed at Kp=200). Keep the
# loop at ~5 ms so the effective rate stays near 100 Hz even with CAN round
# trips (~4-5 ms per cycle on USB-CAN).
MOVE_STEPS, MOVE_DT = 200, 0.005  # 1 s interpolated moves
SETTLE_POLL_S = 0.005
# dq feedback quantization is 12-bit/45 rad/s ≈ 0.011 rad/s, so the settle
# threshold must be above it (0.02) or frictionless axes never "settle".
DQ_TOL = 0.02
SETTLE_S = 1.0
SETTLE_TIMEOUT_S = 20.0
HOLD_ERROR_RAD = 0.15  # > this ⇒ pose not held (recorded, still kept in data)


def move_to(arm, safety, target, kp=HOLD_KP, kd=HOLD_KD, tau_ff=None, n_steps=MOVE_STEPS):
    """Interpolated move from current pose to target (safe, no step jump)."""
    tau_ff = tau_ff or [0.0] * 7
    q0, _, _ = arm.refresh_recv()
    for s in range(1, n_steps + 1):
        t = s / n_steps
        qd = [q0[i] + t * (target[i] - q0[i]) for i in range(7)]
        arm.mit_control_all([(kp[i], kd[i], qd[i], 0.0, tau_ff[i])
                             for i in range(7)])
        q, dq, _ = arm.refresh_recv()
        safety.heartbeat()
        # window is vs the interpolated target (moving), not the start pose:
        # otherwise large moves would trip their own window
        ok, msg = safety.check_pose(q, dq, qd)
        if not ok:
            return False, msg
        time.sleep(MOVE_DT)
    return True, "ok"


def wait_settle(arm, safety, target, kp=HOLD_KP, kd=HOLD_KD, tau_ff=None,
                dq_tol=DQ_TOL, settle_s=SETTLE_S, timeout_s=SETTLE_TIMEOUT_S):
    """Hold target until |dq| < dq_tol for settle_s. Returns (ok, q, dq, tau, held)."""
    tau_ff = tau_ff or [0.0] * 7
    quiet_since = time.monotonic()
    t0 = time.monotonic()
    q = dq = tau = None
    while time.monotonic() - t0 < timeout_s:
        arm.mit_control_all([(kp[i], kd[i], target[i], 0.0, tau_ff[i])
                             for i in range(7)])
        q, dq, tau = arm.refresh_recv()
        safety.heartbeat()
        ok, msg = safety.check_pose(q, dq, target)
        if not ok:
            return False, q, dq, tau, False
        if max(abs(v) for v in dq) < dq_tol:
            if time.monotonic() - quiet_since >= settle_s:
                # check the pose was actually held (not sliding against Kp)
                held = max(abs(q[i] - target[i]) for i in range(7)) < HOLD_ERROR_RAD
                return True, q, dq, tau, held
        else:
            quiet_since = time.monotonic()
        time.sleep(SETTLE_POLL_S)
    held = q is not None and max(abs(q[i] - target[i]) for i in range(7)) < HOLD_ERROR_RAD
    return False, q, dq, tau, held


def phase_a(arm, gm, safety, run_dir):
    """Supported-arm multi-pose regression. Returns rows (pose, joint, g_py, tau_read)."""
    print(f"\n=== Phase A: supported poses, Kp={HOLD_KP}, Kd={HOLD_KD}, τ_ff=0 ===")
    rows = []
    for name, pose in POSES:
        target = [0.0] * 7
        for j, v in pose.items():
            target[j] = v
        ok, msg = move_to(arm, safety, target)
        if not ok:
            print(f"  {name}: move aborted ({msg}) — skipping")
            continue
        ok, q, dq, tau, held = wait_settle(arm, safety, target)
        if not ok:
            print(f"  {name}: did not settle in {SETTLE_TIMEOUT_S}s — skipping")
            continue
        if not held:
            print(f"  {name}: WARNING pose not held "
                  f"(|Δq|max={max(abs(q[i]-target[i]) for i in range(7)):.3f} rad)")
        g = gm.compute(q)
        print(f"  {name}: q=[{', '.join(f'{v:+.3f}' for v in q[:4])}...] "
              f"tau_read=[{', '.join(f'{v:+.2f}' for v in tau[:4])}...]")
        for j in range(7):
            rows.append({"pose": name, "joint": j + 1,
                         "g_py": float(g[j]), "tau_read": float(tau[j])})
        safety.heartbeat()
    return rows


def phase_b(arm, gm, safety, run_dir, args, keepalive):
    """Kp=0 manual support: 5 τ_ff hypotheses at J2=45°, measure drift."""
    print("\n=== Phase B: Kp=0, operator supports arm at J2=45° ===")
    target = [0.0] * 7
    target[1] = D2R(45)
    ok, msg = move_to(arm, safety, target)
    if not ok:
        print(f"  move aborted ({msg})")
        return []
    q, _, _ = arm.refresh_recv()
    g = gm.compute(q)
    g2 = g[1]  # J2 gravity torque at this pose
    N2 = 9.0  # J2 gear ratio
    hyp = {"0": 0.0, "+g": +g2, "-g": -g2, "+g/N": +g2 / N2, "-g/N": -g2 / N2}

    print(f"  J2 gravity torque (model) = {g2:+.2f} Nm")
    print("  >>> OPERATOR: support the arm at this pose, do NOT let it move.")
    interactive_prompt("  press Enter when holding...", keepalive, args.dry_run)

    results = []
    for name, tau_ff2 in hyp.items():
        print(f"  --- hypothesis τ_ff(J2) = {tau_ff2:+.2f} Nm ({name}) ---")
        tau_ff = [0.0] * 7
        tau_ff[1] = tau_ff2
        # watch drift for 5 s (operator releases momentarily on cue)
        print("  >>> OPERATOR: release the arm for 5 s...")
        t0 = time.monotonic()
        samples = []
        while time.monotonic() - t0 < 5.0:
            arm.mit_control_all([(0.0, 1.0, q[i], 0.0, tau_ff[i]) for i in range(7)])
            qq, dq, _ = arm.refresh_recv()
            safety.heartbeat()
            # drift is the MEASUREMENT here — use a wide manual window (±0.8
            # on J2) instead of the tight ±0.35 safety window
            if abs(qq[1] - q[1]) > 0.8:
                safety.trip(reason="phase_b drift window")
                break
            samples.append((time.monotonic() - t0, list(qq)))
            time.sleep(SETTLE_POLL_S)
        drift = [s[1][1] - q[1] for s in samples]  # J2 displacement
        last_drift = drift[-1] if drift else 0.0
        max_drift = max(abs(d) for d in drift) if drift else 0.0
        results.append({"hyp": name, "tau_ff": tau_ff2,
                        "drift_rad": float(last_drift),
                        "max_abs_drift": float(max_drift)})
        print(f"    drift = {last_drift:+.3f} rad ({np.rad2deg(last_drift):+.1f}°)")
        # hold again with Kp
        arm.mit_control_all([(HOLD_KP[i], HOLD_KD[i], q[i], 0.0, 0.0)
                             for i in range(7)])
        arm.refresh_recv()
        safety.heartbeat()
        interactive_prompt("  press Enter for next hypothesis...", keepalive,
                           args.dry_run)
    return results


def analyze(rows):
    """Per-joint regression tau_read = a*g_py + b."""
    print("\n=== Regression: tau_readback = a · g_py(q) + b ===")
    summary = {}
    for j in range(7):
        pts = [(r["g_py"], r["tau_read"]) for r in rows if r["joint"] == j + 1]
        if len(pts) < 2:
            print(f"  J{j+1}: insufficient data ({len(pts)} pts)")
            continue
        xs = np.array([p[0] for p in pts]); ys = np.array([p[1] for p in pts])
        a, b = np.polyfit(xs, ys, 1)
        summary[f"J{j+1}"] = {"slope_a": float(a), "intercept_b": float(b),
                              "n_pts": len(pts)}
        print(f"  J{j+1}: a={a:+.3f}  b={b:+.3f}  (n={len(pts)})")
    return summary


def main():
    args = parse_args("Experiment 1 — τ_ff unit/sign validation (gate)")
    print(f"Interface: {args.iface}  arm: {args.arm}  dry_run: {args.dry_run}")
    print("SAFETY: keep fingers clear; operator supports arm in Phase B; "
          "e-stop = power switch / CLI disable / script 'q'.")

    arm = Arm(args.iface, dry_run=args.dry_run)
    gm = GravityModel(arm=args.arm)
    safety = Safety(arm, log=print)
    run_dir = make_run_dir("exp1_units")

    def keepalive():
        """Hold current pose + heartbeat while waiting for operator input."""
        q, _, _ = arm.refresh_recv()
        arm.mit_control_all([(HOLD_KP[i], HOLD_KD[i], q[i], 0.0, 0.0)
                             for i in range(7)])
        arm.refresh_recv()
        safety.heartbeat()

    try:
        print("Enabling motors...")
        arm.enable()
        # first-motion verification: tiny 0.05 rad step on J1, low gain
        q, _, _ = arm.refresh_recv()
        print(f"  current q = {[f'{v:+.3f}' for v in q]}")
        print("  >>> verify pose matches the printed q before continuing!")
        interactive_prompt("  press Enter to do the 0.05 rad first-motion test on J1...",
                           keepalive, args.dry_run)
        t = list(q); t[0] += 0.05
        arm.mit_control_all([(5.0, 0.5, t[i], 0.0, 0.0) for i in range(7)])
        arm.refresh_recv(); safety.heartbeat()
        interactive_prompt("  press Enter if the tiny step behaved (no runaway)...",
                           keepalive, args.dry_run)

        rows = phase_a(arm, gm, safety, run_dir) if not args.dry_run else []
        b_rows = phase_b(arm, gm, safety, run_dir, args, keepalive) if not args.dry_run else []

        save_csv(run_dir, "phase_a.csv", [(r["pose"], r["joint"], r["g_py"],
                                           r["tau_read"]) for r in rows],
                 ["pose", "joint", "g_py_Nm", "tau_read_Nm"])
        save_csv(run_dir, "phase_b.csv", [(r["hyp"], r["tau_ff"], r["drift_rad"],
                                           r["max_abs_drift"]) for r in b_rows],
                 ["hyp", "tau_ff_Nm", "drift_rad", "max_abs_drift_rad"])
        summary = analyze(rows)
        save_json(run_dir, "summary.json", {"phase_a": summary,
                                            "phase_b": b_rows,
                                            "note": "a≈1 → output-side Nm, REMOVE /GEAR_RATIOS; "
                                                    "a≈1/N → motor-side, keep it"})
        print(f"\nData written to {run_dir}")
        print("Interpretation:")
        if summary:
            a_vals = [s["slope_a"] for s in summary.values()]
            print(f"  slopes: {[f'{a:+.2f}' for a in a_vals]}")
            print("  → slopes ≈ +1.0  : output-side Nm, REMOVE /GEAR_RATIOS in C++")
            print("  → slopes ≈ +1/N  : motor-side Nm, keep /GEAR_RATIOS")
            print("  → negative slopes: τ_ff sign flipped for those joints")
    finally:
        safety.stop()
        print("Motors disabled. Done.")


if __name__ == "__main__":
    main()
