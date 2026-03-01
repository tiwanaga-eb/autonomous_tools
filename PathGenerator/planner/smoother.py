from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List

from .models import Segment, State
from .vehicle_config import VehicleConfig


@dataclass
class SmoothingResult:
    success: bool
    segments: List[Segment]
    smoothing_applied: bool
    added_smoothing_length: float
    max_kappa_rate: float
    kappa_change_sum: float
    reason: str | None = None


def _segment_length(segment: Segment) -> float:
    out = 0.0
    for i in range(1, len(segment.states)):
        dx = segment.states[i].x - segment.states[i - 1].x
        dy = segment.states[i].y - segment.states[i - 1].y
        out += math.hypot(dx, dy)
    return out


def _copy_segment(seg: Segment) -> Segment:
    return Segment(
        gear=seg.gear,
        states=[
            State(
                x=s.x,
                y=s.y,
                yaw=s.yaw,
                curvature=s.curvature,
                gear=s.gear,
                v=s.v,
                t=s.t,
                is_ramp=s.is_ramp,
            )
            for s in seg.states
        ],
    )


def _transform_point(x: float, y: float, tx: float, ty: float, c: float, s: float) -> tuple[float, float]:
    return c * x - s * y + tx, s * x + c * y + ty


def _apply_rigid_transform(segments: List[Segment], start_idx: int, tx: float, ty: float, dtheta: float) -> None:
    c = math.cos(dtheta)
    s = math.sin(dtheta)
    for i in range(start_idx, len(segments)):
        for j, st in enumerate(segments[i].states):
            nx, ny = _transform_point(st.x, st.y, tx, ty, c, s)
            segments[i].states[j] = State(
                x=nx,
                y=ny,
                yaw=st.yaw + dtheta,
                curvature=st.curvature,
                gear=st.gear,
                v=st.v,
                t=st.t,
                is_ramp=st.is_ramp,
            )


def _integrate_ramp(start: State, k1: float, k2: float, kappa_rate_max: float, ds: float) -> List[State]:
    dk = k2 - k1
    s_ramp = abs(dk) / max(kappa_rate_max, 1e-9)
    sign = 1.0 if dk >= 0.0 else -1.0
    x = start.x
    y = start.y
    yaw = start.yaw
    s_acc = 0.0
    t = start.t
    out: List[State] = []

    while s_acc < s_ramp - 1e-9:
        step = min(ds, s_ramp - s_acc)
        kappa_mid = k1 + sign * kappa_rate_max * (s_acc + 0.5 * step)
        yaw_mid = yaw + 0.5 * kappa_mid * step
        x += math.cos(yaw_mid) * step
        y += math.sin(yaw_mid) * step
        yaw += kappa_mid * step
        s_acc += step
        t += step / max(abs(start.v), 1e-6)
        kappa_here = k1 + sign * kappa_rate_max * s_acc
        out.append(
            State(
                x=x,
                y=y,
                yaw=yaw,
                curvature=kappa_here,
                gear=start.gear,
                v=start.v,
                t=t,
                is_ramp=True,
            )
        )
    if out:
        last = out[-1]
        out[-1] = State(
            x=last.x,
            y=last.y,
            yaw=last.yaw,
            curvature=k2,
            gear=last.gear,
            v=last.v,
            t=last.t,
            is_ramp=True,
        )
    return out


def smooth_path_with_clothoid(
    segments: List[Segment],
    vehicle: VehicleConfig,
    enable_smoothing: bool,
) -> SmoothingResult:
    if not enable_smoothing:
        copied = [_copy_segment(s) for s in segments]
        return SmoothingResult(
            success=True,
            segments=copied,
            smoothing_applied=False,
            added_smoothing_length=0.0,
            max_kappa_rate=0.0,
            kappa_change_sum=0.0,
        )

    segs = [_copy_segment(s) for s in segments]
    smoothing_applied = False
    added_length = 0.0
    max_kappa_rate = 0.0
    kappa_change_sum = 0.0

    i = 0
    while i < len(segs) - 1:
        a = segs[i]
        b = segs[i + 1]
        if not a.states or not b.states or a.gear != b.gear:
            i += 1
            continue

        k1 = a.states[-1].curvature
        k2 = b.states[0].curvature
        dk = k2 - k1
        if abs(dk) < 1e-9:
            i += 1
            continue

        s_ramp = abs(dk) / max(vehicle.kappa_rate_max, 1e-9)
        if _segment_length(a) < 0.5 * s_ramp or _segment_length(b) < 0.5 * s_ramp:
            return SmoothingResult(
                success=False,
                segments=segs,
                smoothing_applied=smoothing_applied,
                added_smoothing_length=added_length,
                max_kappa_rate=max_kappa_rate,
                kappa_change_sum=kappa_change_sum,
                reason="insufficient_segment_length_for_smoothing",
            )

        anchor_a = a.states[-1]
        ramp_states = _integrate_ramp(
            start=anchor_a,
            k1=k1,
            k2=k2,
            kappa_rate_max=vehicle.kappa_rate_max,
            ds=max(vehicle.smoothing_step, 1e-3),
        )
        if not ramp_states:
            i += 1
            continue

        a.states.extend(ramp_states)
        smoothing_applied = True
        added_length += s_ramp
        kappa_change_sum += abs(dk)
        max_kappa_rate = max(max_kappa_rate, vehicle.kappa_rate_max)

        old_start = b.states[0]
        new_start = ramp_states[-1]
        dtheta = new_start.yaw - old_start.yaw
        c = math.cos(dtheta)
        s = math.sin(dtheta)
        rx, ry = _transform_point(old_start.x, old_start.y, 0.0, 0.0, c, s)
        tx = new_start.x - rx
        ty = new_start.y - ry
        _apply_rigid_transform(segs, i + 1, tx=tx, ty=ty, dtheta=dtheta)

        i += 1

    return SmoothingResult(
        success=True,
        segments=segs,
        smoothing_applied=smoothing_applied,
        added_smoothing_length=added_length,
        max_kappa_rate=max_kappa_rate,
        kappa_change_sum=kappa_change_sum,
    )
