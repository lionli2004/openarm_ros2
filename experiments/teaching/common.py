#!/usr/bin/env python3
"""OpenArm V10 teaching-mode experiments — shared utilities.

Run experiments as:
    PYTHONNOUSERSITE=1 sudo python3 expX.py --iface can0 [--dry-run]

PYTHONNOUSERSITE=1 is required: a user-local numpy 2.x shadows the system
numpy 1.21.5 that pinocchio 4.0.0 was compiled against (crash otherwise).

Safety model (see plan):
  - 3-layer e-stop: script disable_all() (watchdog / 'q' / Ctrl+C)
    -> CLI `openarm-can-cli -i <iface> disable` -> physical 24V power switch
  - Watchdog thread disables all motors if no successful recv for >300 ms.
  - τ_ff is capped at 0.5 * tMax per joint; soft position window per joint.
  - Motors keep their last command when CAN times out (RID9=0), so a script
    crash does NOT make the arm run away — but always disable afterwards.
"""

import argparse
import json
import os
import signal
import sys
import threading
import time
from pathlib import Path

# --------------------------------------------------------------------------
# Sudo-environment compatibility (sudo clears PYTHONPATH and may drop
# PYTHONNOUSERSITE). Do this BEFORE importing numpy/pinocchio:
#   1) add ROS2 python path (pinocchio 4.0.0 lives there)
#   2) drop the user site-packages so the user-local numpy 2.x cannot
#      shadow the system numpy 1.21.5 that pinocchio 4.0 was built against
# --------------------------------------------------------------------------
for p in ("/opt/ros/humble/lib/python3.10/site-packages",
          "/usr/lib/python3/dist-packages"):
    if p not in sys.path:
        sys.path.insert(0, p)
for p in (str(Path.home() / ".local/lib/python3.10/site-packages"),
          "/home/lionli/.local/lib/python3.10/site-packages"):
    while p in sys.path:
        sys.path.remove(p)

import numpy as np

if np.__version__.startswith("2."):
    print("ERROR: numpy %s loaded — pinocchio 4.0.0 requires numpy 1.x."
          " Run with: PYTHONNOUSERSITE=1 sudo python3 ..." % np.__version__)
    sys.exit(1)

# --------------------------------------------------------------------------
# Hardware constants (right/left arms share CAN IDs, differ by interface)
# --------------------------------------------------------------------------
MOTOR_TYPES = [  # J1..J7
    "DM8009", "DM8009", "DM4340", "DM4340", "DM4310", "DM4310", "DM4310",
]
SEND_IDS = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07]
RECV_IDS = [0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17]
GEAR_RATIOS = [9.0, 9.0, 40.0, 40.0, 10.0, 10.0, 10.0]
# Output-side torque limits (Nm), used to cap τ_ff
TMAX = [54.0, 54.0, 28.0, 28.0, 10.0, 10.0, 10.0]
JOINT_NAMES = [f"J{i+1}" for i in range(7)]

URDF_PATH = "/home/lionli/ros2_ws/install/openarm_hardware/share/openarm_hardware/openarm.urdf"
NV_OFFSET = {"right_": 9, "left_": 0}  # bimanual 18-DOF layout
EXP_DIR = Path("/home/lionli/ros2_ws/experiments/teaching")


# --------------------------------------------------------------------------
# Arm wrapper
# --------------------------------------------------------------------------
class Arm:
    """Thin wrapper around openarm_can.OpenArm for the 7 arm motors (no gripper)."""

    def __init__(self, iface, enable_fd=True, dry_run=False):
        self.iface = iface
        self.dry_run = dry_run
        if dry_run:
            self.arm = None
            return
        from openarm_can import OpenArm  # deferred: only needed on real runs

        self.oa = OpenArm(iface, enable_fd=enable_fd)
        types = [getattr(__import__("openarm_can"), t) for t in MOTOR_TYPES]
        self.oa.init_arm_motors(types, SEND_IDS, RECV_IDS)
        self.arm = self.oa.get_arm()

    def enable(self):
        if self.dry_run:
            return
        self.oa.enable_all()
        time.sleep(0.1)
        self.oa.recv_all()

    def disable(self):
        if self.dry_run:
            return
        self.oa.disable_all()
        time.sleep(0.1)
        self.oa.recv_all()

    def refresh_recv(self, timeout_us=100):
        """refresh + recv, returns (q, dq, tau) lists or None on dry-run."""
        if self.dry_run:
            return [0.0] * 7, [0.0] * 7, [0.0] * 7
        self.oa.refresh_all()
        self.oa.recv_all(timeout_us)
        motors = self.arm.get_motors()
        return (
            [m.get_position() for m in motors],
            [m.get_velocity() for m in motors],
            [m.get_torque() for m in motors],
        )

    def mit_control_all(self, params):
        """params: list of 5-tuples (kp, kd, q, dq, tau)."""
        if self.dry_run:
            return
        from openarm_can import MITParam

        self.arm.mit_control_all([MITParam(*p) for p in params])


# --------------------------------------------------------------------------
# Gravity model (pinocchio on the 18-DOF bimanual URDF)
# --------------------------------------------------------------------------
class GravityModel:
    """Output-side gravity torques g(q) for one arm from the bimanual URDF.

    q input is a 7-vector of joint positions (rad, output side).
    Returns g (Nm, output side) — hypotheses: g itself vs g/GEAR_RATIOS
    are compared in the experiments to resolve the MIT-frame unit question.
    """

    def __init__(self, urdf_path=URDF_PATH, arm="right_"):
        import pinocchio as pin

        self.pin = pin
        self.model = pin.buildModelFromUrdf(urdf_path, False)  # pinocchio 4.x
        self.data = pin.Data(self.model)
        assert self.model.nv == 18, f"expected 18-DOF bimanual URDF, got nv={self.model.nv}"
        self.offset = NV_OFFSET[arm]
        # sanity: check joint names at the expected offsets
        names = self.model.names
        for i, jn in enumerate([f"openarm_{arm}joint{i+1}" for i in range(7)]):
            assert names[self.offset + i + 1] == jn, (
                f"URDF joint name mismatch at nv {self.offset+i}: "
                f"got '{names[self.offset+i+1]}', want '{jn}'")

    def compute(self, q7, arm="right_"):
        """q7: 7-vector of joint positions; returns 7-vector g (Nm, output side)."""
        q = np.zeros(self.model.nv)
        q[self.offset:self.offset + 7] = q7
        g = self.pin.computeGeneralizedGravity(self.model, self.data, q)
        return np.array(g[self.offset:self.offset + 7])


# --------------------------------------------------------------------------
# Safety: watchdog + signal handling + soft limits + torque cap
# --------------------------------------------------------------------------
class Safety:
    def __init__(self, arm: Arm, tau_cap_ratio=0.5, pos_window=0.35,
                 watchdog_timeout_ms=300, log=None):
        self.arm = arm
        self.tau_cap = [t * tau_cap_ratio for t in TMAX]
        self.pos_window = pos_window
        self.timeout_s = watchdog_timeout_ms / 1000.0
        self.log = log or (lambda *a, **k: None)
        self._last_heartbeat = time.monotonic()
        self._last_speed_log = 0.0
        self._tripped = False
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._watchdog = threading.Thread(target=self._run, daemon=True)
        self._watchdog.start()
        signal.signal(signal.SIGINT, self._sig)
        signal.signal(signal.SIGTERM, self._sig)

    def _run(self):
        while not self._stop.wait(0.1):
            if time.monotonic() - self._last_heartbeat > self.timeout_s:
                self.log("WATCHDOG: no comms for %.0f ms — DISABLING ALL" %
                         (self.timeout_s * 1000))
                self.trip(reason="watchdog")

    def _sig(self, signum, frame):
        self.log(f"signal {signum} — disabling all")
        self.trip(reason=f"signal {signum}")

    def heartbeat(self):
        self._last_heartbeat = time.monotonic()

    def trip(self, reason="manual"):
        with self._lock:
            if self._tripped:
                return
            self._tripped = True
        self.log(f"SAFETY TRIP ({reason})")
        self.arm.disable()
        self._stop.set()

    @property
    def tripped(self):
        return self._tripped

    def check_pose(self, q, dq, ref_q):
        """Soft limits: per-joint window around reference + τ_ff cap check.
        Returns (ok, msg)."""
        if self.tripped:
            return False, "already tripped"
        for i in range(7):
            if abs(q[i] - ref_q[i]) > self.pos_window:
                self.log(f"POS WINDOW violated J{i+1}: |q={q[i]:.3f} - ref={ref_q[i]:.3f}| > {self.pos_window}")
                self.trip(reason="pos window")
                return False, "pos window"
        if max(abs(v) for v in dq) > 1.0:
            now = time.monotonic()
            if now - self._last_speed_log > 1.0:  # rate-limit spam
                self._last_speed_log = now
                self.log(f"SPEED alarm: dq={[f'{v:.2f}' for v in dq]}")
            # informational only — operator drag can legitimately be fast
        return True, "ok"

    def clamp_tau(self, tau):
        """Clamp τ_ff to per-joint caps (output-side Nm)."""
        return [max(-c, min(c, t)) for t, c in zip(tau, self.tau_cap)]

    def stop(self):
        self._stop.set()
        self.arm.disable()


# --------------------------------------------------------------------------
# Data helpers
# --------------------------------------------------------------------------
def make_run_dir(exp_name):
    ts = time.strftime("%Y%m%d-%H%M%S")
    d = EXP_DIR / "data" / exp_name / ts
    d.mkdir(parents=True, exist_ok=True)
    return d


def save_csv(run_dir, filename, rows, header):
    import csv

    with open(run_dir / filename, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(header)
        w.writerows(rows)


def save_json(run_dir, filename, obj):
    with open(run_dir / filename, "w") as f:
        json.dump(obj, f, indent=2, ensure_ascii=False)


def parse_args(description):
    p = argparse.ArgumentParser(description=description)
    p.add_argument("--iface", default="can0", help="CAN interface (can0=right, can1=left)")
    p.add_argument("--arm", default="right_", choices=["right_", "left_"],
                   help="arm side (sets Pinocchio nv offset)")
    p.add_argument("--scale", type=float, default=0.1,
                   help="teaching gain scale (exp3)")
    p.add_argument("--dry-run", action="store_true",
                   help="simulate without hardware (for script testing)")
    return p.parse_args()


def prompt(msg, dry_run=False):
    """input() that auto-confirms in dry-run mode."""
    if dry_run:
        return ""
    return input(msg)


def interactive_prompt(msg, keepalive=None, dry_run=False, timeout=None):
    """Prompt that keeps the arm alive while waiting for input.

    Blocking input() would stall CAN traffic and trip the watchdog, so this
    uses a non-blocking select() loop and calls keepalive() periodically
    (keepalive should send a hold frame + safety.heartbeat()).
    """
    if dry_run:
        return ""
    import select

    print(msg, end="", flush=True)
    t0 = time.monotonic()
    while True:
        r, _, _ = select.select([sys.stdin], [], [], 0.1)
        if r:
            sys.stdin.readline()  # consume the newline
            return ""
        if keepalive is not None:
            keepalive()
        if timeout is not None and time.monotonic() - t0 > timeout:
            return None


def wait_key(prompt="...", timeout=None):
    """Blocking key read from stdin (None key on timeout)."""
    import select

    if timeout is None:
        return input(prompt).strip()
    r, _, _ = select.select([sys.stdin], [], [], timeout)
    if r:
        return sys.stdin.readline().strip()
    return None
