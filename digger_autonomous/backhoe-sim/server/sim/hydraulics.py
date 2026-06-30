"""Lumped hydraulic model: normalised valve command u in [-1,1] -> joint torque.

`u` is a normalised *torque/effort demand* produced by the computed-torque
controller (joint_ctrl) or by integrated teleop targets.  The model adds the
physics that make it feel hydraulic:

  * spool non-linearity (deadband + expo + saturation)
  * **valve-closed lock** — within the deadband the spool blocks flow and the
    near-incompressible oil holds the joint in place (this is why a real boom
    does NOT fall when the stick is centred).  Modelled as a stiff PD to the
    angle held at valve closure.  Solves static gravity hold without bypassing
    the valve abstraction.
  * pump flow sharing -> per-axis speed limit (simultaneous motion is slower)
  * relief valve -> torque saturation (stalls against rock / heavy load)

Fidelity levels (live-switchable):
  L0  static map, instant, no lag, no pump sharing      (clean debugging)
  L1  L0 + 1st-order torque lag + pump-shared speed cap  (default)
  L2  explicit chamber-pressure state + pose-dependent moment arm
"""
from __future__ import annotations

import math

import numpy as np

from config import JOINT_NAMES


def spool(u: float, deadband: float, expo: float) -> float:
    a = abs(u)
    if a < deadband:
        return 0.0
    s = min((a - deadband) / (1.0 - deadband), 1.0)
    return math.copysign(s ** expo, u)


def spool_inverse(s: float, deadband: float, expo: float) -> float:
    """Inverse of :func:`spool`: given a desired normalised output s in [-1,1],
    return the valve command u that produces it.  Lets the controller
    pre-compensate the spool non-linearity so feed-forward lands accurately."""
    a = min(abs(s), 1.0)
    if a < 1e-9:
        return 0.0
    u = deadband + (1.0 - deadband) * (a ** (1.0 / expo))
    return math.copysign(min(u, 1.0), s)


class Hydraulics:
    def __init__(self, cfg: dict):
        self.cfg = cfg
        n = len(JOINT_NAMES)
        self.tau_filt = np.zeros(n)
        self.q_hold = np.zeros(n)          # angle held while valve closed
        self._hold_valid = np.zeros(n, dtype=bool)
        self.P_a = np.full(n, 1.0e6)
        self.P_b = np.full(n, 1.0e6)
        # diagnostics
        self.last_tau = np.zeros(n)
        self.last_Q_frac = np.zeros(n)
        self.pump_util = 0.0
        self.last_pressure = np.zeros((n, 2))
        self.locked = np.zeros(n, dtype=bool)

    def _arm(self, name: str, q: float) -> float:
        h = self.cfg["hydraulics"]
        base = h["moment_arm"][name]
        if h["level"] == "L2" and name != "swing":
            return base * max(0.35, abs(math.sin(q + 0.6)))
        return base

    def step(self, u: np.ndarray, q: np.ndarray, qd: np.ndarray, dt: float) -> np.ndarray:
        h = self.cfg["hydraulics"]
        level = h["level"]
        n = len(JOINT_NAMES)
        A_a = h["A_a"]
        relief_F = h["P_relief"] * A_a

        # --- pump flow sharing: intended speed -> available speed per axis -----
        intended_v = np.zeros(n)     # rad/s the spool is asking for
        for i, name in enumerate(JOINT_NAMES):
            intended_v[i] = abs(spool(float(u[i]), h["deadband"], h["expo"])) \
                * self.cfg["teleop"]["max_rate"][name]
        Q_dem = np.array([intended_v[i] * self._arm(JOINT_NAMES[i], q[i]) * A_a
                          for i in range(n)])
        total = float(Q_dem.sum())
        self.pump_util = min(1.0, total / h["Q_pump"]) if h["Q_pump"] > 0 else 0.0
        share = 1.0
        if level in ("L1", "L2") and total > h["Q_pump"] and total > 0:
            share = h["Q_pump"] / total
        self.last_Q_frac = Q_dem / max(h["Q_pump"], 1e-9)
        v_avail = intended_v * share          # speed cap per axis (rad/s)

        tau = np.zeros(n)
        for i, name in enumerate(JOINT_NAMES):
            arm = self._arm(name, q[i])
            tau_lim = relief_F * arm
            s = spool(float(u[i]), h["deadband"], h["expo"])

            if abs(u[i]) < h["deadband"]:
                # valve closed -> hydraulic lock (hold last angle)
                if not self._hold_valid[i]:
                    self.q_hold[i] = q[i]
                    self._hold_valid[i] = True
                kp = h["servo_kv"][name]
                kd = h["lock_kd"][name]
                tau_cmd = -kp * (q[i] - self.q_hold[i]) - kd * qd[i]
                tau_cmd = float(np.clip(tau_cmd, -tau_lim, tau_lim))
                self.locked[i] = True
            else:
                self._hold_valid[i] = False
                self.locked[i] = False
                if level == "L2":
                    tau_cmd = self._step_pressure(i, name, s, qd[i], arm, dt)
                else:
                    tau_cmd = s * h["tau_max"][name]
                    # meter-out velocity cap: the metered flow bounds joint speed
                    # at v_avail in BOTH directions.  Above it, return-line back-
                    # pressure brakes the motion (this is what stops a boom from
                    # free-falling when lowered, and what makes simultaneous ops
                    # share the pump and slow down).  Smooth -> stable.
                    if level in ("L1", "L2"):
                        over = abs(qd[i]) - v_avail[i]
                        if over > 0.0:
                            tau_cmd -= math.copysign(h["k_meter"][name] * over, qd[i])
                    tau_cmd = float(np.clip(tau_cmd, -tau_lim, tau_lim))
                    if level == "L1":
                        a = dt / max(h["tau_response"], 1e-4)
                        self.tau_filt[i] += a * (tau_cmd - self.tau_filt[i])
                        tau_cmd = self.tau_filt[i]
            tau[i] = tau_cmd

        self.last_tau = tau.copy()
        self.last_pressure = np.stack([self.P_a, self.P_b], axis=1).copy()
        return tau

    def _step_pressure(self, i, name, s, qd, arm, dt) -> float:
        h = self.cfg["hydraulics"]
        A_a, A_b, beta = h["A_a"], h["A_b"], h["beta"]
        v_piston = qd * arm
        Q_in = s * self.cfg["teleop"]["max_rate"][name] * arm * A_a
        V_a = h["V_dead"] + A_a * 0.6 * h["stroke"]
        V_b = h["V_dead"] + A_b * 0.4 * h["stroke"]
        self.P_a[i] += dt * (beta / V_a) * (Q_in - A_a * v_piston)
        self.P_b[i] += dt * (beta / V_b) * (A_b * v_piston + min(Q_in, 0.0))
        self.P_a[i] = float(np.clip(self.P_a[i], 0.0, h["P_relief"]))
        self.P_b[i] = float(np.clip(self.P_b[i], 0.0, h["P_relief"]))
        F = self.P_a[i] * A_a - self.P_b[i] * A_b - self._friction(v_piston)
        return F * arm

    def _friction(self, v: float) -> float:
        h = self.cfg["hydraulics"]
        return h["visc_friction"] * v + h["coulomb_friction"] * math.tanh(50.0 * v)

    def reset(self) -> None:
        self.tau_filt[:] = 0.0
        self.P_a[:] = 1.0e6
        self.P_b[:] = 1.0e6
        self._hold_valid[:] = False
        self.locked[:] = False
