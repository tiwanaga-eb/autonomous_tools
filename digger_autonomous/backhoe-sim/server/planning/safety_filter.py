"""The single safety merge point for ALL motion sources.

joystick / FSM / assist  ->  v_nom  ->  [GeoFence barriers] -> [CBF-QP] -> v_safe

Whether the command originates from the autonomous FSM or from manual teleop, it
passes through here before reaching the joint controller / hydraulics.  This is
the design hinge the spec calls out: one place where design-surface and dump-
truck barriers are always enforced.
"""
from __future__ import annotations

import numpy as np

from config import JOINT_NAMES
from planning import cbf, geofence


class SafetyFilter:
    def __init__(self, cfg: dict):
        self.cfg = cfg
        self.last_info = {"geofence": False, "cbf": False, "clamped_axes": [],
                          "active": []}
        self.barriers = []

    def apply(self, machine, v_nom: np.ndarray, digging: bool) -> tuple[np.ndarray, dict]:
        """Filter a joint-velocity command (rad/s).  Returns (v_safe, status)."""
        s = self.cfg["safety"]
        n = len(JOINT_NAMES)
        # velocity bounds from per-axis max rate
        vmax = np.array([self.cfg["teleop"]["max_rate"][nm] for nm in JOINT_NAMES])
        if not s["enabled"]:
            self.barriers = []
            self.last_info = {"geofence": False, "cbf": False,
                              "clamped_axes": [], "active": []}
            return np.clip(v_nom, -vmax, vmax), self.last_info

        self.barriers = geofence.all_barriers(machine, self.cfg, digging)
        v_safe, info = cbf.filter_velocity(v_nom, self.barriers, -vmax, vmax,
                                           solver=s["qp"]["solver"])
        clamped = [JOINT_NAMES[i] for i in range(n)
                   if abs(v_safe[i] - np.clip(v_nom[i], -vmax[i], vmax[i])) > 1e-3]
        active = info["active"]
        status = {
            "geofence": any(a.startswith(("design", "truck")) for a in active),
            "cbf": info["modified"],
            "clamped_axes": clamped,
            "active": active,
            "infeasible": info["infeasible"],
            "min_h": min((b.h for b in self.barriers), default=float("inf")),
        }
        self.last_info = status
        return v_safe, status

    def barrier_report(self) -> list[dict]:
        return [{"kind": b.kind, "label": b.label, "h": round(float(b.h), 3)}
                for b in self.barriers]
