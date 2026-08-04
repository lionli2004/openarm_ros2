#!/usr/bin/env python3
"""Offline analysis of gravity calibration data.

Pairing: for each joint and pose file, bin samples by q (0.02 rad). For bins
with both + and − direction samples:
    g_actual(q) = (mean τ_read,+ + mean τ_read,−) / 2      (friction cancels)
g_model is evaluated at the FULL recorded q7 (bin-average), not at
"other joints = 0" — the calibration sweeps leave residual poses in other
joints, which couple into the gravity torque of the scanned joint.

Per joint: one pooled OLS over ALL poses: g_actual = k_j·g_model + b0_j,
then decision rules (accept k / offset / model-structure error).

Usage:  PYTHONNOUSERSITE=1 python3 analyze_calib.py --arm right_ --dir <run_dir>
Output: <run_dir>/summary.json, <run_dir>/params_<arm>.txt (grav_k line)
"""
import argparse
import csv
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np

from common import GravityModel

BIN = 0.02
MIN_SAMPLES = 5
G_MIN_FRAC = 0.3  # require |g_model| > 0.3·max|g| to compute point-slope k_i


def load_csv(path):
    rows = []
    with open(path) as f:
        r = csv.DictReader(f)
        for row in r:
            q = [float(row[f"q{i+1}"]) for i in range(7)]
            dq = [float(row[f"dq{i+1}"]) for i in range(7)]
            tau = [float(row[f"tau{i+1}"]) for i in range(7)]
            rows.append((q, dq, tau))
    return rows


def paired_extract(rows, joint):
    """Returns list of (q7_avg, g_actual) using ± pairing at the same q_j."""
    bins = {}
    for q, dq, tau in rows:
        if abs(dq[joint]) < 0.01:
            continue
        direction = 1 if dq[joint] > 0 else -1
        key = round(q[joint] / BIN)
        bins.setdefault(key, {1: [], -1: []})
        bins[key][direction].append((q, tau[joint]))
    out = []
    for key, dirs in bins.items():
        pos, neg = dirs[1], dirs[-1]
        if len(pos) < MIN_SAMPLES or len(neg) < MIN_SAMPLES:
            continue
        tau_pos = float(np.mean([t for _, t in pos]))
        tau_neg = float(np.mean([t for _, t in neg]))
        g_actual = (tau_pos + tau_neg) / 2.0
        # full-q average from both directions
        q7 = [0.0] * 7
        all_q = [q for q, _ in pos] + [q for q, _ in neg]
        for i in range(7):
            q7[i] = float(np.mean([q[i] for q in all_q]))
        out.append((q7, g_actual))
    return out


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--arm", default="right_", choices=["right_", "left_"])
    parser.add_argument("--dir", required=True, help="run dir (data/calib/<ts>/gravity)")
    args = parser.parse_args()

    run_dir = Path(args.dir)
    raw_dir = run_dir / "raw"
    gm = GravityModel(arm=args.arm)

    summary = {}
    grav_k = [1.0] * 7
    # collect (joint, g_model, g_actual) across ALL pose files
    pooled = {j: [] for j in range(7)}
    for path in sorted(raw_dir.glob(f"{args.arm}J*_p*.csv")):
        j = int(path.stem.split("J")[1][0]) - 1
        rows = load_csv(path)
        pts = paired_extract(rows, j)
        for q7, g_act in pts:
            g_model = gm.compute(q7)[j]
            pooled[j].append((g_model, g_act))
        print(f"  {path.name}: {len(pts)} paired points")

    for j in range(7):
        pts = pooled[j]
        if len(pts) < 3:
            print(f"  J{j+1}: insufficient pooled data ({len(pts)})")
            summary[f"J{j+1}"] = {"n": len(pts), "decision": "insufficient"}
            continue
        xs = np.array([p[0] for p in pts])
        ys = np.array([p[1] for p in pts])
        A = np.column_stack([xs, np.ones_like(xs)])
        k, b0 = np.linalg.lstsq(A, ys, rcond=None)[0]
        resid = ys - (k * xs + b0)
        # point-wise slope where model signal is significant
        mask = np.abs(xs) > G_MIN_FRAC * np.max(np.abs(xs))
        k_pts = (ys[mask] / xs[mask]) if mask.any() else np.array([])
        cv = float(np.std(k_pts) / np.mean(k_pts)) if len(k_pts) > 1 else 0.0
        grav_k[j] = float(k)
        s = {
            "n": len(pts),
            "k": float(k), "b0": float(b0),
            "resid_max": float(np.max(np.abs(resid))),
            "k_cv": cv,
            "k_range": ([float(np.min(k_pts)), float(np.max(k_pts))]
                        if len(k_pts) else None),
            "decision": "",
        }
        # decision rules
        if k < 0:
            s["decision"] = "STOP: k<0 — axis/sign error"
        elif cv < 0.05 and abs(k - 1) <= 0.15 and s["resid_max"] < 0.2:
            s["decision"] = "accept k (fast path)"
        elif cv < 0.15 and s["resid_max"] < 0.2:
            s["decision"] = "accept k with offset (|b0|<0.3 ignored)"
        elif cv >= 0.15:
            s["decision"] = "MODEL STRUCTURE ERROR — fix inertials.yaml CoM"
        else:
            s["decision"] = "high residual — check b0/sensor bias"
        summary[f"J{j+1}"] = s
        print(f"  J{j+1}: k={k:+.3f} b0={b0:+.3f} resid_max={s['resid_max']:.3f} "
              f"CV={cv*100:.1f}% (n={len(pts)})  → {s['decision']}")

    out = {
        "arm": args.arm,
        "grav_k": grav_k,
        "joints": summary,
        "note": "grav_k line ready for the calib file",
    }
    with open(run_dir / "summary.json", "w") as f:
        json.dump(out, f, indent=2)
    with open(run_dir / f"params_{args.arm}.txt", "w") as f:
        f.write(f"# OpenArm calib — {args.arm}  (gravity only)\n")
        f.write("grav_k   " + " ".join(f"{v:.4f}" for v in grav_k) + "\n")
    print(f"\nWritten {run_dir / 'summary.json'} and params_{args.arm}.txt")
    return 0


if __name__ == "__main__":
    sys.exit(main())
