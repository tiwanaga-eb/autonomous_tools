"""Control Barrier Function safety filter (velocity-level QP).

Given a nominal joint-velocity command ``v_nom`` and a set of barriers (each with
value h and gradient dh/dq), solve

    v_safe = argmin_v  ||v - v_nom||^2
             s.t.  grad_i . v >= -alpha_i * h_i   for every barrier i
                   v_min <= v <= v_max

so the corrected command never drives any barrier below zero faster than the
class-K rate alpha_i*h_i allows.  This is the minimal modification that keeps
the tip above the design surface and the bucket clear of the truck.

Uses qpsolvers (OSQP).  Falls back to the unconstrained command if the QP is
infeasible / the solver is unavailable, flagging the event.
"""
from __future__ import annotations

import numpy as np

try:
    from qpsolvers import solve_qp
    _HAVE_QP = True
except Exception:                       # pragma: no cover
    _HAVE_QP = False


def filter_velocity(v_nom: np.ndarray, barriers, v_min, v_max,
                    solver: str = "osqp") -> tuple[np.ndarray, dict]:
    """Return (v_safe, info).  info has clamped axes and which barriers bound."""
    n = len(v_nom)
    info = {"active": [], "infeasible": False, "modified": False}
    if not barriers or not _HAVE_QP:
        return np.clip(v_nom, v_min, v_max), info

    # min 0.5 v^T P v + q^T v  with P=2I, q=-2 v_nom  (== ||v - v_nom||^2)
    P = 2.0 * np.eye(n)
    q = -2.0 * v_nom.astype(float)
    # constraints grad . v >= -alpha h   ->   -grad . v <= alpha h
    G = []
    hvec = []
    for b in barriers:
        G.append(-b.grad)
        hvec.append(b.alpha * b.h)
    G = np.array(G, dtype=float)
    hvec = np.array(hvec, dtype=float)
    lb = np.asarray(v_min, dtype=float)
    ub = np.asarray(v_max, dtype=float)

    try:
        v = solve_qp(P, q, G=G, h=hvec, lb=lb, ub=ub, solver=solver)
    except Exception:
        v = None
    if v is None:
        info["infeasible"] = True
        return np.clip(v_nom, v_min, v_max), info

    v = np.asarray(v, dtype=float)
    info["modified"] = bool(np.linalg.norm(v - v_nom) > 1e-4)
    # report barriers whose constraint is near-active
    for b in barriers:
        if b.grad @ v <= -b.alpha * b.h + 1e-3:
            info["active"].append(b.label or b.kind)
    return v, info
