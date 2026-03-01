from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List

from .geometry import wrap_angle
from .models import State
from .vehicle_config import VehicleConfig


@dataclass
class PrimitiveAction:
    v: float
    delta_dot: float
    target_gear: str


@dataclass
class PrimitiveRollout:
    states: List[State]
    end_x: float
    end_y: float
    end_yaw: float
    end_delta: float
    gear: str
    dist: float
    steer_effort: float


def generate_actions(
    vehicle: VehicleConfig,
    current_gear: str,
    switch_used: int,
    forward_only: bool,
    allow_reverse: bool,
    speed_levels: int,
    delta_dot_levels: int,
    v_fwd: float | None = None,
    v_rev: float | None = None,
) -> List[PrimitiveAction]:
    speed_levels = max(1, speed_levels)
    delta_dot_levels = max(1, delta_dot_levels)

    fwd_ref = abs(float(v_fwd if v_fwd is not None else vehicle.max_speed_fwd))
    rev_ref = abs(float(v_rev if v_rev is not None else vehicle.max_speed_rev))
    fwd_vals = [fwd_ref * (i + 1) / speed_levels for i in range(speed_levels)]
    rev_vals = [-rev_ref * (i + 1) / speed_levels for i in range(speed_levels)]

    if delta_dot_levels == 1:
        dots = [0.0]
    else:
        dots = []
        for i in range(delta_dot_levels):
            alpha = -1.0 + 2.0 * i / (delta_dot_levels - 1)
            dots.append(alpha * vehicle.max_steer_rate)

    actions: List[PrimitiveAction] = []

    if current_gear == "F":
        for v in fwd_vals:
            for d in dots:
                actions.append(PrimitiveAction(v=v, delta_dot=d, target_gear="F"))
        if not forward_only and allow_reverse and switch_used == 0:
            for v in rev_vals:
                for d in dots:
                    actions.append(PrimitiveAction(v=v, delta_dot=d, target_gear="R"))
    else:
        for v in rev_vals:
            for d in dots:
                actions.append(PrimitiveAction(v=v, delta_dot=d, target_gear="R"))

    return actions


def rollout_primitive(
    x: float,
    y: float,
    yaw: float,
    delta: float,
    t0: float,
    action: PrimitiveAction,
    vehicle: VehicleConfig,
    T: float,
    dt: float,
) -> PrimitiveRollout:
    """Simulate vehicle motion for time T under a constant action.

    Optimisations vs the original:
    - Pre-compute loop-invariant values (inv_wheelbase, step multiples) once.
    - Fast analytical path for delta_dot == 0 (constant-curvature arc or
      straight line): intermediate yaw and position are computed in closed form,
      so only a coarser state grid is needed for collision checking while
      maintaining the same end-point accuracy.
    - State objects are created after the integration loop rather than inside it,
      reducing repeated dataclass construction overhead per step.
    """
    dt = max(1e-3, dt)
    steps = max(1, int(math.ceil(T / dt)))
    step_dt = T / steps

    inv_wb = 1.0 / max(vehicle.wheel_base, 1e-6)
    gear = action.target_gear
    v = action.v
    ddot = action.delta_dot
    max_delta = vehicle.max_steer_angle
    spd = abs(v)
    v_step = v * step_dt
    ddot_step = ddot * step_dt

    cx, cy, cyaw, cdelta = x, y, yaw, delta
    t = t0
    dist = spd * T
    steer_effort = abs(ddot) * T

    # ── Fast path: constant curvature (delta_dot == 0) ───────────────────────
    # When steering rate is zero the curvature is constant throughout the
    # rollout.  Intermediate yaw and position are computed analytically
    # (closed-form per step) which is faster than the iterative Euler update.
    # All states are kept for collision checking — subsampling was removed
    # because it reduced the effective check density to a single state when
    # combined with collision_check_stride=3 in the planner.
    if ddot == 0.0:
        kappa = math.tan(cdelta) * inv_wb
        raw: List[tuple] = []
        if abs(kappa) < 1e-10:
            # Pure straight
            cos_y = math.cos(cyaw)
            sin_y = math.sin(cyaw)
            for i in range(1, steps + 1):
                s = i * step_dt
                raw.append((cx + v * cos_y * s, cy + v * sin_y * s, cyaw, kappa, t0 + s))
        else:
            # Constant-radius arc: closed-form per-step
            r = 1.0 / kappa
            for i in range(1, steps + 1):
                s = i * step_dt
                dyaw = v * kappa * s
                new_yaw = wrap_angle(cyaw + dyaw)
                new_x = cx + r * (math.sin(cyaw + dyaw) - math.sin(cyaw))
                new_y = cy - r * (math.cos(cyaw + dyaw) - math.cos(cyaw))
                raw.append((new_x, new_y, new_yaw, kappa, t0 + s))

        states = [
            State(x=r[0], y=r[1], yaw=r[2], curvature=r[3], gear=gear, v=v, t=r[4])
            for r in raw
        ]
        last = raw[-1]
        return PrimitiveRollout(
            states=states,
            end_x=last[0],
            end_y=last[1],
            end_yaw=last[2],
            end_delta=cdelta,
            gear=gear,
            dist=dist,
            steer_effort=steer_effort,
        )

    # ── General path: varying curvature (delta_dot != 0) ─────────────────────
    # Accumulate raw tuples first; build State objects only after the loop.
    raw_gen: List[tuple] = []
    for _ in range(steps):
        cdelta = max(-max_delta, min(max_delta, cdelta + ddot_step))
        kappa = math.tan(cdelta) * inv_wb
        cyaw = wrap_angle(cyaw + v_step * kappa)
        cx += v * math.cos(cyaw) * step_dt
        cy += v * math.sin(cyaw) * step_dt
        t += step_dt
        raw_gen.append((cx, cy, cyaw, kappa, t))

    states = [
        State(x=r[0], y=r[1], yaw=r[2], curvature=r[3], gear=gear, v=v, t=r[4])
        for r in raw_gen
    ]
    return PrimitiveRollout(
        states=states,
        end_x=cx,
        end_y=cy,
        end_yaw=cyaw,
        end_delta=cdelta,
        gear=gear,
        dist=dist,
        steer_effort=steer_effort,
    )
