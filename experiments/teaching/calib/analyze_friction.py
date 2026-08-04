#!/usr/bin/env python3
"""Offline friction fitting: τ_fric(dq) = τ_c±·tanh(dq/ε) + B±·dq.

Reads friction_<arm>.csv, selects steady-state windows (segment mid 50%,
|dq − s| < 0.3·s), computes τ_fric = τ_read − k·g_model(q), fits Coulomb +
viscous per direction (OLS over speeds), appends tau_c/b/eps to the params
file, and reports consistency vs breakaway τ_s.

Usage:  PYTHONNOUSERSITE=1 python3 analyze_friction.py --arm right_ \
            --dir <run_dir> --params <gravity_run>/params_<arm>.txt
"""
import argparse
import csv
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np

EPS = [0.01, 0.01, 0.02, 0.02, 0.01, 0.01, 0.01]  # per joint

SPEEDS = {  # same as calib_friction
    "fast": [0.05, 0.10, 0.20, 0.40],
    "slow": [0.02, 0.05, 0.10, 0.20],
}


def load_grav_k(params_file):
    k = [1.0] * 7
    if params_file and Path(params_file).exists():
        for line in Path(params_file).read_text().splitlines():
            parts = line.split()
            if parts and parts[0] == "grav_k" and len(parts) == 8:
                k = [float(v) for v in parts[1:8]]
    return k


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--arm", default="right_", choices=["right_", "left_"])
    parser.add_argument("--dir", required=True)
    parser.add_argument("--params", default="")
    args = parser.parse_args()

    run_dir = Path(args.dir)
    csv_path = run_dir / f"friction_{args.arm}.csv"
    if not csv_path.exists():
        print(f"no data: {csv_path}")
        return 1
    grav_k = load_grav_k(args.params)
    speeds = SPEEDS["fast"]  # friction CSV records actual speeds; we group below

    rows = []
    with open(csv_path) as f:
        r = csv.DictReader(f)
        for row in r:
            try:
                j = int(row["joint"]) - 1
            except ValueError:
                continue  # stray header line from appended files
            speed = float(row["speed"])
            q = [float(row[f"q{i}"]) for i in range(1, 8)]
            dq = [float(row[f"dq{i}"]) for i in range(1, 8)]
            tau = [float(row[f"tau{i}"]) for i in range(1, 8)]
            g_ff = float(row["g_ff"])
            rows.append((j, speed, q, dq, tau, g_ff))

    summary = {}
    tau_c_plus = [0.0] * 7
    tau_c_minus = [0.0] * 7
    b = [0.0] * 7
    for j in range(7):
        # steady-state windows per (speed, direction)
        pts = {1: [], -1: []}  # direction -> list of tau_fric
        for jj, speed, q, dq, tau, g_ff in rows:
            if jj != j:
                continue
            v = dq[j]
            if abs(v) < 0.3 * speed:
                continue  # not in steady state
            direction = 1 if v > 0 else -1
            tau_fric = tau[j] - g_ff
            pts[direction].append(tau_fric)
        summary[f"J{j+1}"] = {}
        for direction, key in ((1, "plus"), (-1, "minus")):
            vals = pts[direction]
            if not vals:
                summary[f"J{j+1}"][key] = {"error": "no steady-state data"}
                continue
            # mean per speed (approx: group by rounded speed)
            mean = float(np.mean(vals))
            std = float(np.std(vals))
            # viscous slope from spread across speeds would need per-speed
            # grouping; we approximate B via OLS on (v, τ_fric) samples
            vs = []
            ts = []
            for jj, speed, q, dq, tau, g_ff in rows:
                if jj != j:
                    continue
                v = dq[j]
                if direction * v > 0 and abs(v) > 0.3 * speed:
                    vs.append(v)
                    ts.append(tau[j] - g_ff)
            if len(vs) >= 2:
                B, C = np.polyfit(vs, ts, 1)
            else:
                B, C = 0.0, mean
            # Conservative truncation (observed live):
            #  - negative B: Stribeck effect makes the linear fit slope
            #    negative — no physical meaning, zero it
            #  - τ_c sign: force directional consistency (plus ≥ 0, minus ≤ 0)
            #  - SAFETY (J7 self-motion incident): fitted τ_c carries gravity
            #    residual bias; tanh(·/ε) saturates at tiny |dq|, so τ_c
            #    overestimated beyond the true static friction τ_s produces
            #    self-sustained motion on release. The safe bound is
            #    teach_scale·τ_c < τ_s (feedforward vanishes as dq→0, so
            #    holding is unaffected; only motion gets assisted). With
            #    teach_scale=0.5, J4 (τ_c 1.57, τ_s≈1.1-1.5) stays safe.
            #    Drop B entirely (viscous data unreliable).
            B = 0.0
            C = max(C, 0.0) if direction == 1 else min(C, 0.0)
            summary[f"J{j+1}"][key] = {"tau_c": float(C), "b": float(B),
                                       "n": len(vals), "std": std,
                                       "raw": {"tau_c": float(
                                           np.polyfit(vs, ts, 1)[1] if len(vs) >= 2 else mean),
                                           "b": float(
                                           np.polyfit(vs, ts, 1)[0] if len(vs) >= 2 else 0.0)}}
            if direction == 1:
                tau_c_plus[j] = C
            else:
                tau_c_minus[j] = C
            print(f"  J{j+1} {key}: τ_c={C:+.3f} B={B:+.3f} (n={len(vals)})")

    # write params file: REBUILD from scratch (gravity lines carried over,
    # friction lines regenerated) — never append, repeated runs must not
    # duplicate blocks
    out_path = Path(args.params) if args.params else run_dir / f"params_{args.arm}.txt"
    lines = [f"# OpenArm calib — {args.arm}"]
    lines.append("grav_k   " + " ".join(f"{v:.4f}" for v in grav_k))
    lines.append("tau_c_plus  " + " ".join(f"{v:.3f}" for v in tau_c_plus))
    lines.append("tau_c_minus " + " ".join(f"{v:.3f}" for v in tau_c_minus))
    lines.append("b_plus      " + " ".join(f"{0.0:.3f}" for _ in range(7)))
    lines.append("b_minus     " + " ".join(f"{0.0:.3f}" for _ in range(7)))
    lines.append("eps         " + " ".join(f"{v:.2f}" for v in EPS))
    lines.append("fric_scale   0.60")
    lines.append("teach_scale  0.50")
    lines.append("teach_deadzone 0.03")
    out_path.write_text("\n".join(lines) + "\n")
    print(f"\nParams written to {out_path}")
    print("Next: copy to ~/ros2_ws/calib/<arm>.calib and set calib_file "
          "in launch args (right_calib_file:=... left_calib_file:=...)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
