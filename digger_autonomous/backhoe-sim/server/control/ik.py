"""Analytic inverse kinematics for the backhoe.

The swing joint selects the sagittal plane; within it the boom (L1) + arm (L2)
form a 2R chain solved in closed form for the bucket-pivot ("wrist") point, and
the bucket angle is specified independently to set the cutting orientation.

Joint sign convention (matches machine.fk_analytic): a hinge about +y by q maps
local +x to (cos q, -sin q) in the (radial, z) plane, i.e. +q pitches the link
DOWN.  We solve in standard CCW angles psi = -q and convert back.

A damped least-squares fallback (using the exact analytic FK) handles targets
the closed form misses (near-singular / slightly out of reach).
"""
from __future__ import annotations

import math

import numpy as np
from scipy.optimize import least_squares

from config import GEOM, HOME_POSE, JOINT_LIMITS, JOINT_NAMES

_LIM_LO = np.array([JOINT_LIMITS[n][0] for n in JOINT_NAMES])
_LIM_HI = np.array([JOINT_LIMITS[n][1] for n in JOINT_NAMES])
_HOME = np.array([HOME_POSE[n] for n in JOINT_NAMES])

# planar geometry constants
_PIV_R = GEOM["boom_pivot"][0]                      # boom pivot radial offset
_PIV_Z = GEOM["swing_pivot"][2] + GEOM["boom_pivot"][2]   # boom pivot world z
_L1 = GEOM["boom_len"]
_L2 = GEOM["arm_len"]
_TIP = np.array([GEOM["tip_local"][0], GEOM["tip_local"][2]])   # (x,z) in bucket frame
_L3 = float(np.linalg.norm(_TIP))
_TIP_OFF = math.atan2(-_TIP[1], _TIP[0])            # internal angle of tip link (R_y sense)


def _ry_plane(angle: float, vx: float, vz: float) -> tuple[float, float]:
    """Apply R_y(angle) to (vx,0,vz); return (radial, z) components."""
    c, s = math.cos(angle), math.sin(angle)
    return (c * vx + s * vz, -s * vx + c * vz)


def solve(target_world: np.ndarray, bucket_pitch: float,
          elbow_up: bool = True) -> tuple[np.ndarray | None, bool]:
    """Solve IK for a tip world position and absolute bucket pitch (R_y sense).

    Returns ``(q, ok)``.  ``q`` is clamped to joint limits; ``ok`` is False if
    the target is unreachable (closed form failed AND fallback didn't converge).
    """
    x, y, z = float(target_world[0]), float(target_world[1]), float(target_world[2])
    swing = math.atan2(y, x)
    radial = math.hypot(x, y)

    # tip link cumulative angle Q3 = bucket_pitch ; wrist = tip - tiplink
    Q3 = bucket_pitch
    tx, tz = _ry_plane(Q3, _TIP[0], _TIP[1])
    wrist_r = radial - tx
    wrist_z = z - tz

    # 2R from boom pivot to wrist, in standard CCW angles psi = -q.
    # Both elbow branches (±acos) are tried; the valid (in-limits) one is kept.
    # Which branch is "in range" depends on the joint-limit sign convention, so we
    # do NOT hard-code it (the old code assumed the +acos branch, which is wrong
    # for the backhoe convention where the arm range is positive).
    dr = wrist_r - _PIV_R
    dz = wrist_z - _PIV_Z
    D2 = dr * dr + dz * dz
    D = math.sqrt(D2)
    closed: list[np.ndarray] = []
    if abs(_L1 - _L2) - 1e-6 <= D <= (_L1 + _L2) - 1e-6:
        cos_psi2 = (D2 - _L1 * _L1 - _L2 * _L2) / (2 * _L1 * _L2)
        cos_psi2 = max(-1.0, min(1.0, cos_psi2))
        base = math.acos(cos_psi2)
        # order branches so the caller's elbow_up preference is tried first
        branches = (base, -base) if elbow_up else (-base, base)
        for psi2 in branches:
            psi1 = math.atan2(dz, dr) - math.atan2(_L2 * math.sin(psi2),
                                                   _L1 + _L2 * math.cos(psi2))
            q1, q2 = -psi1, -psi2
            q3 = Q3 - (q1 + q2)
            closed.append(np.array([swing, q1, q2, q3]))

    for q in closed:
        if not _violates(q):
            return np.clip(q, _LIM_LO, _LIM_HI), True

    # closed form missed (out of limits / out of reach): damped LSQ from several
    # in-range seeds, including the two closed-form branches and the home pose.
    seeds = [c for c in closed]
    seeds.append(np.array([swing, *_HOME[1:]]))
    seeds.append(np.clip(np.array([swing, 0.3, 1.2, 0.6]), _LIM_LO, _LIM_HI))
    q_fb, ok = _fallback(np.array([x, y, z]), bucket_pitch, seeds=seeds)
    if not ok:
        return (np.clip(q_fb, _LIM_LO, _LIM_HI) if q_fb is not None else None, False)
    return np.clip(q_fb, _LIM_LO, _LIM_HI), True


def _violates(q: np.ndarray) -> bool:
    return bool(np.any(q < _LIM_LO - 1e-3) or np.any(q > _LIM_HI + 1e-3))


# --- damped least-squares fallback using the exact analytic FK ---------------
def _fk(q: np.ndarray) -> tuple[np.ndarray, float]:
    """Tip position and absolute bucket pitch from joint angles (analytic)."""
    # replicate machine.fk_analytic without needing a Machine instance
    sw = q[0]
    csw, ssw = math.cos(sw), math.sin(sw)
    # planar position of tip in (radial, z)
    Q1, Q2, Q3 = q[1], q[1] + q[2], q[1] + q[2] + q[3]
    r, zz = _PIV_R, _PIV_Z
    dr, dz = _ry_plane(Q1, _L1, 0.0); r += dr; zz += dz
    dr, dz = _ry_plane(Q2, _L2, 0.0); r += dr; zz += dz
    dr, dz = _ry_plane(Q3, _TIP[0], _TIP[1]); r += dr; zz += dz
    pos = np.array([csw * r, ssw * r, zz])
    pitch = Q3
    return pos, pitch


def _fallback(target: np.ndarray, bucket_pitch: float, seeds=None):
    """Damped least-squares from one or more in-range seeds; return the first that
    converges (tip error < 5 cm)."""
    def resid(q):
        pos, pitch = _fk(q)
        return np.concatenate([pos - target, [0.3 * (pitch - bucket_pitch)]])
    if not seeds:
        seeds = [np.clip(np.array([math.atan2(target[1], target[0]), 0.3, 1.2, 0.6]),
                         _LIM_LO, _LIM_HI)]
    best = None
    for q0 in seeds:
        q0 = np.clip(np.asarray(q0, float), _LIM_LO, _LIM_HI)
        try:
            sol = least_squares(resid, q0, bounds=(_LIM_LO, _LIM_HI),
                                xtol=1e-8, ftol=1e-8, max_nfev=80)
        except Exception:
            continue
        pos, _ = _fk(sol.x)
        err = float(np.linalg.norm(pos - target))
        if err < 0.05:
            return sol.x, True
        if best is None or err < best[1]:
            best = (sol.x, err)
    return (best[0] if best else None), False
