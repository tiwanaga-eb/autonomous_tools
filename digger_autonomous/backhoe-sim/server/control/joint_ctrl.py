"""Low-level joint controller (computed-torque + PID).

Outputs a normalised valve command u in [-1,1] per joint.  The control law is a
computed-torque design:

    tau_des = qfrc_bias (gravity + Coriolis feed-forward)  + Kp e + Ki integral - Kd qd
    u       = clip(tau_des / tau_max_axis, -1, 1)

Hydraulics.py then turns u back into the actually-deliverable torque (with spool
non-linearity, pump-shared speed limit, relief saturation and lag).  The gravity
feed-forward is what lets the machine hold a pose; the valve-closed lock in the
hydraulics handles the residual once u falls inside the deadband.

The control gains in CONFIG are interpreted in this torque-demand space.
"""
from __future__ import annotations

import numpy as np

from config import JOINT_NAMES
from sim.hydraulics import spool_inverse


class JointController:
    def __init__(self, cfg: dict):
        self.cfg = cfg
        self.n = len(JOINT_NAMES)
        self.i_err = np.zeros(self.n)

    def reset(self) -> None:
        self.i_err[:] = 0.0

    def u_from_targets(self, q_des: np.ndarray, q: np.ndarray, qd: np.ndarray,
                       dt: float, bias: np.ndarray | None = None) -> np.ndarray:
        """Computed-torque PID -> normalised valve command u in [-1,1].

        ``bias`` is the qfrc_bias gravity/Coriolis feed-forward (canonical order);
        pass None to disable gravity comp.
        """
        c = self.cfg["control"]
        u = np.zeros(self.n)
        use_bias = c["gravity_comp"] and bias is not None
        for i, name in enumerate(JOINT_NAMES):
            e = q_des[i] - q[i]
            self.i_err[i] += e * dt
            self.i_err[i] = float(np.clip(self.i_err[i], -c["i_clamp"], c["i_clamp"]))
            tau_des = (c["kp"][name] * c["tau_scale"] * e
                       + c["ki"][name] * c["tau_scale"] * self.i_err[i]
                       - c["kd"][name] * c["tau_scale"] * qd[i])
            if use_bias:
                tau_des += bias[i]
            tau_max = self.cfg["hydraulics"]["tau_max"][name]
            # pre-compensate the spool non-linearity so the delivered torque
            # matches tau_des (otherwise the expo curve eats the feed-forward).
            s = float(np.clip(tau_des / max(tau_max, 1e-6), -1.0, 1.0))
            h = self.cfg["hydraulics"]
            u[i] = spool_inverse(s, h["deadband"], h["expo"])
        return u


class Admittance:
    """Force-following helper for digging: yields tip depth when the measured
    cutting force exceeds the target (the bucket "gives" to keep force bounded).

    Returns a depth correction (m) to subtract from the nominal penetration so
    the trajectory keeps cutting force near target_dig_force.
    """

    def __init__(self, cfg: dict):
        self.cfg = cfg
        self.depth_offset = 0.0

    def reset(self) -> None:
        self.depth_offset = 0.0

    def update(self, measured_force: float, dt: float) -> float:
        a = self.cfg["control"]["admittance"]
        err = measured_force - a["target_dig_force"]
        # positive err (too much force) -> retract (reduce depth) ; clamp rate
        rate = np.clip(a["gain"] * err, -a["max_retract"], a["max_retract"])
        self.depth_offset += rate * dt
        self.depth_offset = max(0.0, self.depth_offset)   # never push deeper than nominal
        return self.depth_offset
