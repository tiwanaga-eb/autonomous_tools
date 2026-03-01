from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Callable, Dict, List, Optional, Tuple

from .geometry import mod2pi, wrap_angle
from .models import Pose, Segment, State


@dataclass
class DubinsPath:
    mode: str
    params: Tuple[float, float, float]
    length: float


def _lsl(alpha: float, beta: float, d: float) -> Optional[Tuple[float, float, float]]:
    tmp = d + math.sin(alpha) - math.sin(beta)
    p2 = 2 + d * d - 2 * math.cos(alpha - beta) + 2 * d * (math.sin(alpha) - math.sin(beta))
    if p2 < 0:
        return None
    p = math.sqrt(p2)
    t = mod2pi(-alpha + math.atan2(math.cos(beta) - math.cos(alpha), tmp))
    q = mod2pi(beta - math.atan2(math.cos(beta) - math.cos(alpha), tmp))
    return t, p, q


def _rsr(alpha: float, beta: float, d: float) -> Optional[Tuple[float, float, float]]:
    tmp = d - math.sin(alpha) + math.sin(beta)
    p2 = 2 + d * d - 2 * math.cos(alpha - beta) + 2 * d * (-math.sin(alpha) + math.sin(beta))
    if p2 < 0:
        return None
    p = math.sqrt(p2)
    t = mod2pi(alpha - math.atan2(math.cos(alpha) - math.cos(beta), tmp))
    q = mod2pi(-beta + math.atan2(math.cos(alpha) - math.cos(beta), tmp))
    return t, p, q


def _lsr(alpha: float, beta: float, d: float) -> Optional[Tuple[float, float, float]]:
    p2 = -2 + d * d + 2 * math.cos(alpha - beta) + 2 * d * (math.sin(alpha) + math.sin(beta))
    if p2 < 0:
        return None
    p = math.sqrt(p2)
    tmp = math.atan2(-math.cos(alpha) - math.cos(beta), d + math.sin(alpha) + math.sin(beta)) - math.atan2(-2.0, p)
    t = mod2pi(-alpha + tmp)
    q = mod2pi(-mod2pi(beta) + tmp)
    return t, p, q


def _rsl(alpha: float, beta: float, d: float) -> Optional[Tuple[float, float, float]]:
    p2 = -2 + d * d + 2 * math.cos(alpha - beta) - 2 * d * (math.sin(alpha) + math.sin(beta))
    if p2 < 0:
        return None
    p = math.sqrt(p2)
    tmp = math.atan2(math.cos(alpha) + math.cos(beta), d - math.sin(alpha) - math.sin(beta)) - math.atan2(2.0, p)
    t = mod2pi(alpha - tmp)
    q = mod2pi(beta - tmp)
    return t, p, q


def _rlr(alpha: float, beta: float, d: float) -> Optional[Tuple[float, float, float]]:
    tmp = (6.0 - d * d + 2 * math.cos(alpha - beta) + 2 * d * (math.sin(alpha) - math.sin(beta))) / 8.0
    if abs(tmp) > 1.0:
        return None
    p = mod2pi(2 * math.pi - math.acos(tmp))
    t = mod2pi(alpha - math.atan2(math.cos(alpha) - math.cos(beta), d - math.sin(alpha) + math.sin(beta)) + p / 2.0)
    q = mod2pi(alpha - beta - t + p)
    return t, p, q


def _lrl(alpha: float, beta: float, d: float) -> Optional[Tuple[float, float, float]]:
    tmp = (6.0 - d * d + 2 * math.cos(alpha - beta) + 2 * d * (-math.sin(alpha) + math.sin(beta))) / 8.0
    if abs(tmp) > 1.0:
        return None
    p = mod2pi(2 * math.pi - math.acos(tmp))
    t = mod2pi(-alpha - math.atan2(math.cos(alpha) - math.cos(beta), d + math.sin(alpha) - math.sin(beta)) + p / 2.0)
    q = mod2pi(mod2pi(beta) - alpha - t + p)
    return t, p, q


_PATHS: Dict[str, Callable[[float, float, float], Optional[Tuple[float, float, float]]]] = {
    "LSL": _lsl,
    "RSR": _rsr,
    "LSR": _lsr,
    "RSL": _rsl,
    "RLR": _rlr,
    "LRL": _lrl,
}


class DubinsSolver:
    def __init__(self, min_turn_radius: float):
        self.min_turn_radius = min_turn_radius

    def shortest_path(self, start: Pose, goal: Pose) -> Optional[DubinsPath]:
        rho = self.min_turn_radius
        dx = goal.x - start.x
        dy = goal.y - start.y
        d = math.hypot(dx, dy) / rho
        theta = math.atan2(dy, dx)
        alpha = mod2pi(start.yaw - theta)
        beta = mod2pi(goal.yaw - theta)

        best: Optional[DubinsPath] = None
        for mode, fn in _PATHS.items():
            params = fn(alpha, beta, d)
            if params is None:
                continue
            length = rho * sum(params)
            if best is None or length < best.length:
                best = DubinsPath(mode=mode, params=params, length=length)
        return best

    def _step(self, x: float, y: float, yaw: float, ds: float, curvature: float) -> Tuple[float, float, float]:
        if abs(curvature) < 1e-10:
            return x + ds * math.cos(yaw), y + ds * math.sin(yaw), yaw
        delta = curvature * ds
        r = 1.0 / curvature
        nx = x + r * (math.sin(yaw + delta) - math.sin(yaw))
        ny = y - r * (math.cos(yaw + delta) - math.cos(yaw))
        return nx, ny, wrap_angle(yaw + delta)

    def sample_segment(self, start: Pose, goal: Pose, gear: str, speed: float, step_size: float) -> Optional[Segment]:
        path = self.shortest_path(start, goal)
        if path is None:
            return None

        rho = self.min_turn_radius
        states: List[State] = []
        x, y, yaw = start.x, start.y, start.yaw
        t = 0.0
        states.append(State(x=x, y=y, yaw=yaw, curvature=0.0, gear=gear, v=speed if gear == "F" else -speed, t=t))

        for primitive, p in zip(path.mode, path.params):
            if primitive == "S":
                curvature = 0.0
                seg_len = p * rho
            elif primitive == "L":
                curvature = 1.0 / rho
                seg_len = p * rho
            else:
                curvature = -1.0 / rho
                seg_len = p * rho

            traveled = 0.0
            while traveled + step_size < seg_len:
                ds = step_size
                x, y, yaw = self._step(x, y, yaw, ds, curvature)
                traveled += ds
                t += ds / max(speed, 1e-6)
                states.append(State(x=x, y=y, yaw=yaw, curvature=curvature, gear=gear, v=speed if gear == "F" else -speed, t=t))

            rem = max(0.0, seg_len - traveled)
            if rem > 1e-9:
                x, y, yaw = self._step(x, y, yaw, rem, curvature)
                t += rem / max(speed, 1e-6)
                states.append(State(x=x, y=y, yaw=yaw, curvature=curvature, gear=gear, v=speed if gear == "F" else -speed, t=t))

        if states:
            states[-1] = State(
                x=goal.x,
                y=goal.y,
                yaw=goal.yaw,
                curvature=states[-1].curvature,
                gear=gear,
                v=speed if gear == "F" else -speed,
                t=states[-1].t,
            )
        return Segment(gear=gear, states=states)

    def sample_reverse_segment(self, start: Pose, goal: Pose, speed: float, step_size: float) -> Optional[Segment]:
        start_fwd = Pose(start.x, start.y, wrap_angle(start.yaw + math.pi))
        goal_fwd = Pose(goal.x, goal.y, wrap_angle(goal.yaw + math.pi))
        fwd_seg = self.sample_segment(start_fwd, goal_fwd, gear="F", speed=speed, step_size=step_size)
        if fwd_seg is None:
            return None

        states: List[State] = []
        for s in fwd_seg.states:
            states.append(
                State(
                    x=s.x,
                    y=s.y,
                    yaw=wrap_angle(s.yaw - math.pi),
                    curvature=s.curvature,
                    gear="R",
                    v=-abs(speed),
                    t=s.t,
                )
            )
        if states:
            states[0] = State(
                x=start.x,
                y=start.y,
                yaw=start.yaw,
                curvature=states[0].curvature,
                gear="R",
                v=-abs(speed),
                t=0.0,
            )
            states[-1] = State(
                x=goal.x,
                y=goal.y,
                yaw=goal.yaw,
                curvature=states[-1].curvature,
                gear="R",
                v=-abs(speed),
                t=states[-1].t,
            )
        return Segment(gear="R", states=states)

    def sample_straight_segment(self, start: Pose, goal: Pose, gear: str, speed: float, step_size: float) -> Segment:
        dx = goal.x - start.x
        dy = goal.y - start.y
        length = math.hypot(dx, dy)
        if length < 1e-9:
            return Segment(
                gear=gear,
                states=[
                    State(
                        x=start.x,
                        y=start.y,
                        yaw=start.yaw,
                        curvature=0.0,
                        gear=gear,
                        v=speed if gear == "F" else -abs(speed),
                        t=0.0,
                    )
                ],
            )

        ux = dx / length
        uy = dy / length
        states: List[State] = []
        traveled = 0.0
        t = 0.0
        states.append(
            State(
                x=start.x,
                y=start.y,
                yaw=start.yaw,
                curvature=0.0,
                gear=gear,
                v=speed if gear == "F" else -abs(speed),
                t=t,
            )
        )

        step_size = max(step_size, 1e-3)
        while traveled + step_size < length:
            traveled += step_size
            t += step_size / max(speed, 1e-6)
            states.append(
                State(
                    x=start.x + ux * traveled,
                    y=start.y + uy * traveled,
                    yaw=start.yaw,
                    curvature=0.0,
                    gear=gear,
                    v=speed if gear == "F" else -abs(speed),
                    t=t,
                )
            )

        rem = length - traveled
        if rem > 1e-9:
            t += rem / max(speed, 1e-6)
            states.append(
                State(
                    x=goal.x,
                    y=goal.y,
                    yaw=goal.yaw,
                    curvature=0.0,
                    gear=gear,
                    v=speed if gear == "F" else -abs(speed),
                    t=t,
                )
            )
        else:
            states[-1] = State(
                x=goal.x,
                y=goal.y,
                yaw=goal.yaw,
                curvature=0.0,
                gear=gear,
                v=speed if gear == "F" else -abs(speed),
                t=states[-1].t,
            )
        return Segment(gear=gear, states=states)
