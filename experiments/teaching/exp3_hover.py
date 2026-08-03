#!/usr/bin/env python3
"""Experiment 3 — hover & drag validation (teaching-equivalent, Python direct).

Interactive control loop (100 Hz):
  τ_cmd = (kp·scale, kd·scale, q_des, 0, τ_ff=g_py(q))

Keys (type in terminal, then Enter):
  t   drag mode: q_des tracks feedback — operator drags the arm freely
  h   hover test: freezes q_des at the current pose; operator releases;
      10 s of drift is recorded and reported per joint
  c   hover CONTROL: same but τ_ff=0 (proves compensation is what holds)
  q   quit (disables motors)

Output: data/exp3_hover/<ts>/hover_*.csv + summary.json (per-pose drift).

Usage:  PYTHONNOUSERSITE=1 sudo python3 exp3_hover.py --iface can0 \
            [--scale 0.1] [--dry-run]
"""
import threading
import time

import numpy as np

from common import (Arm, GravityModel, Safety, interactive_prompt,
                    make_run_dir, parse_args, save_csv, save_json)

KP_BASE = 200.0
KD_BASE = 5.0
HOVER_SECS = 10.0
LOOP_DT = 0.01


class KeyReader(threading.Thread):
    """Background stdin reader; sets self.key to the latest command."""

    def __init__(self):
        super().__init__(daemon=True)
        self.key = None
        self._lock = threading.Lock()

    def run(self):
        while True:
            line = input().strip().lower()
            with self._lock:
                self.key = line

    def take(self):
        with self._lock:
            k = self.key
            self.key = None
            return k


def main():
    args = parse_args("Experiment 3 — hover & drag validation")
    print(f"Interface: {args.iface}  arm: {args.arm}  scale: {args.scale}")
    print("SAFETY: operator supports the arm during hover tests; "
          "e-stop = power switch / CLI disable / 'q'.")

    arm = Arm(args.iface, dry_run=args.dry_run)
    gm = GravityModel(arm=args.arm)
    safety = Safety(arm, log=print)
    run_dir = make_run_dir("exp3_hover")

    def keepalive():
        """Hold current pose + heartbeat while waiting for operator input."""
        q, _, _ = arm.refresh_recv()
        arm.mit_control_all([(KP_BASE * 0.1, KD_BASE * 0.1, q[i], 0.0, 0.0)
                             for i in range(7)])
        arm.refresh_recv()
        safety.heartbeat()

    try:
        arm.enable()
        q, _, _ = arm.refresh_recv()
        print(f"current q = {[f'{v:+.3f}' for v in q]}")
        interactive_prompt("verify pose matches printed q, then Enter "
                           "(tiny first-motion test)...", keepalive, args.dry_run)
        t = list(q); t[0] += 0.05
        arm.mit_control_all([(5.0, 0.5, t[i], 0.0, 0.0) for i in range(7)])
        arm.refresh_recv(); safety.heartbeat()
        interactive_prompt("Enter if the tiny step behaved...", keepalive,
                           args.dry_run)

        if args.dry_run:
            print("dry-run: skipping interactive loop")
            return

        print("\nCommands: t=drag  h=hover test  c=hover control(τ_ff=0)  q=quit")
        kr = KeyReader()
        kr.start()

        mode = "drag"
        q_des = list(q)
        hover_rows = []
        hover_summary = []
        t_hover0 = None
        q_frozen = None
        tau_enabled = True

        while not safety.tripped:
            cmd = kr.take()
            if cmd == "q":
                print("quitting...")
                break
            elif cmd == "t":
                mode = "drag"
                print("[drag mode] drag the arm freely")
            elif cmd == "h":
                mode = "hover"
                q_frozen = list(q_des)
                tau_enabled = True
                t_hover0 = time.monotonic()
                print("[hover test] RELEASE the arm — recording 10 s drift...")
            elif cmd == "c":
                mode = "hover"
                q_frozen = list(q_des)
                tau_enabled = False
                t_hover0 = time.monotonic()
                print("[hover CONTROL τ_ff=0] RELEASE — recording 10 s drift...")

            if mode == "drag":
                q_des = list(q)
            elif mode == "hover":
                q_des = q_frozen
                if time.monotonic() - t_hover0 >= HOVER_SECS:
                    # summarize this hover run
                    drift = [q[i] - q_frozen[i] for i in range(7)]
                    print(f"[hover done] drift/10s: "
                          f"{[f'J{i+1}:{np.rad2deg(d):+.2f}°' for i, d in enumerate(drift)]}")
                    hover_summary.append({"mode": "comp" if tau_enabled else "ctrl",
                                          "q0": [float(v) for v in q_frozen],
                                          "drift_deg": [float(np.rad2deg(d)) for d in drift]})
                    mode = "drag"
                    tau_enabled = True

            g = gm.compute(q)
            tau_ff = g if tau_enabled else [0.0] * 7
            kp = KP_BASE * args.scale
            kd = KD_BASE * args.scale
            arm.mit_control_all([(kp, kd, q_des[i], 0.0, tau_ff[i])
                                 for i in range(7)])
            q, dq, tau_read = arm.refresh_recv()
            safety.heartbeat()
            hover_rows.append((time.monotonic(), mode, float(tau_enabled),
                               list(q), list(dq), list(tau_read), list(g)))
            if max(abs(v) for v in dq) > 1.0:
                print("  [speed alarm] drag velocity high — be careful")
            time.sleep(LOOP_DT)

        save_csv(run_dir, "hover.csv",
                 [(r[0], r[1], r[2]) + tuple(r[3]) + tuple(r[4]) + tuple(r[5]) + tuple(r[6])
                  for r in hover_rows],
                 ["t_s", "mode", "tau_enabled"] +
                 [f"q{i+1}" for i in range(7)] + [f"dq{i+1}" for i in range(7)] +
                 [f"tau_read{i+1}" for i in range(7)] + [f"g_py{i+1}" for i in range(7)])
        save_json(run_dir, "summary.json", {"hover_runs": hover_summary,
                                            "scale": args.scale})
        print(f"\nData written to {run_dir}")
        for h in hover_summary:
            m = "comp" if h["mode"] == "comp" else "control(τ_ff=0)"
            print(f"  {m}: {[f'J{i+1}:{d:+.2f}°' for i, d in enumerate(h['drift_deg'])]}")
    finally:
        safety.stop()
        print("Motors disabled. Done.")


if __name__ == "__main__":
    main()
