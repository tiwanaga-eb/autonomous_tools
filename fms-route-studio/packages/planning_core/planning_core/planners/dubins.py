"""Dubins 経路（最小旋回半径制約つきの解析的接続）。

由来: PathGenerator/planner/dubins.py（LSL/RSR/LSR/RSL/RLR/LRL の6語）を planning_core 用に整理。
用途（設計書 §9.1 B / §9.5）: 経由点ガイドで、各点の進入方位を尊重しつつ R_min を保証した
前進接続を作る。出力は (N,2) のポリライン（曲率は区間定数なので curvature_source="analytic"）。
"""
from __future__ import annotations

import math
from typing import Optional, Tuple

import numpy as np


def _mod2pi(a: float) -> float:
    return a % (2.0 * math.pi)


def _wrap(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def _lsl(a, b, d):
    tmp = d + math.sin(a) - math.sin(b)
    p2 = 2 + d * d - 2 * math.cos(a - b) + 2 * d * (math.sin(a) - math.sin(b))
    if p2 < 0:
        return None
    p = math.sqrt(p2)
    t = _mod2pi(-a + math.atan2(math.cos(b) - math.cos(a), tmp))
    q = _mod2pi(b - math.atan2(math.cos(b) - math.cos(a), tmp))
    return (t, p, q), "LSL"


def _rsr(a, b, d):
    tmp = d - math.sin(a) + math.sin(b)
    p2 = 2 + d * d - 2 * math.cos(a - b) + 2 * d * (-math.sin(a) + math.sin(b))
    if p2 < 0:
        return None
    p = math.sqrt(p2)
    t = _mod2pi(a - math.atan2(math.cos(a) - math.cos(b), tmp))
    q = _mod2pi(-b + math.atan2(math.cos(a) - math.cos(b), tmp))
    return (t, p, q), "RSR"


def _lsr(a, b, d):
    p2 = -2 + d * d + 2 * math.cos(a - b) + 2 * d * (math.sin(a) + math.sin(b))
    if p2 < 0:
        return None
    p = math.sqrt(p2)
    tmp = math.atan2(-math.cos(a) - math.cos(b), d + math.sin(a) + math.sin(b)) - math.atan2(-2.0, p)
    t = _mod2pi(-a + tmp)
    q = _mod2pi(-_mod2pi(b) + tmp)
    return (t, p, q), "LSR"


def _rsl(a, b, d):
    p2 = -2 + d * d + 2 * math.cos(a - b) - 2 * d * (math.sin(a) + math.sin(b))
    if p2 < 0:
        return None
    p = math.sqrt(p2)
    tmp = math.atan2(math.cos(a) + math.cos(b), d - math.sin(a) - math.sin(b)) - math.atan2(2.0, p)
    t = _mod2pi(a - tmp)
    q = _mod2pi(b - tmp)
    return (t, p, q), "RSL"


def _rlr(a, b, d):
    tmp = (6.0 - d * d + 2 * math.cos(a - b) + 2 * d * (math.sin(a) - math.sin(b))) / 8.0
    if abs(tmp) > 1.0:
        return None
    p = _mod2pi(2 * math.pi - math.acos(tmp))
    t = _mod2pi(a - math.atan2(math.cos(a) - math.cos(b), d - math.sin(a) + math.sin(b)) + p / 2.0)
    q = _mod2pi(a - b - t + p)
    return (t, p, q), "RLR"


def _lrl(a, b, d):
    tmp = (6.0 - d * d + 2 * math.cos(a - b) + 2 * d * (-math.sin(a) + math.sin(b))) / 8.0
    if abs(tmp) > 1.0:
        return None
    p = _mod2pi(2 * math.pi - math.acos(tmp))
    t = _mod2pi(-a - math.atan2(math.cos(a) - math.cos(b), d + math.sin(a) - math.sin(b)) + p / 2.0)
    q = _mod2pi(_mod2pi(b) - a - t + p)
    return (t, p, q), "LRL"


_WORDS = (_lsl, _rsr, _lsr, _rsl, _rlr, _lrl)


def shortest_dubins(
    start: Tuple[float, float, float],
    goal: Tuple[float, float, float],
    rho: float,
) -> Optional[Tuple[str, Tuple[float, float, float], float]]:
    """Return (mode, (t,p,q) in normalized units, length[m]) of the shortest Dubins word."""
    sx, sy, syaw = start
    gx, gy, gyaw = goal
    d = math.hypot(gx - sx, gy - sy) / rho
    theta = math.atan2(gy - sy, gx - sx)
    a = _mod2pi(syaw - theta)
    b = _mod2pi(gyaw - theta)
    best = None
    for fn in _WORDS:
        r = fn(a, b, d)
        if r is None:
            continue
        params, mode = r
        length = rho * sum(params)
        if best is None or length < best[2]:
            best = (mode, params, length)
    return best


def _step(x, y, yaw, ds, curv):
    if abs(curv) < 1e-10:
        return x + ds * math.cos(yaw), y + ds * math.sin(yaw), yaw
    r = 1.0 / curv
    nyaw = yaw + curv * ds
    return (
        x + r * (math.sin(nyaw) - math.sin(yaw)),
        y - r * (math.cos(nyaw) - math.cos(yaw)),
        _wrap(nyaw),
    )


def sample_dubins(
    start: Tuple[float, float, float],
    goal: Tuple[float, float, float],
    rho: float,
    step: float = 0.5,
) -> Optional[np.ndarray]:
    """Sample the shortest Dubins path as an (N,2) polyline at ~`step` spacing."""
    best = shortest_dubins(start, goal, rho)
    if best is None:
        return None
    mode, params, _ = best
    x, y, yaw = start
    pts = [(x, y)]
    for prim, p in zip(mode, params):
        curv = 0.0 if prim == "S" else (1.0 / rho if prim == "L" else -1.0 / rho)
        seg_len = p * rho
        traveled = 0.0
        while traveled + step < seg_len:
            x, y, yaw = _step(x, y, yaw, step, curv)
            traveled += step
            pts.append((x, y))
        rem = max(0.0, seg_len - traveled)
        if rem > 1e-9:
            x, y, yaw = _step(x, y, yaw, rem, curv)
            pts.append((x, y))
    pts[-1] = (goal[0], goal[1])
    return np.asarray(pts, float)


def reverse_dubins(
    start: Tuple[float, float, float],
    goal: Tuple[float, float, float],
    rho: float,
    step: float = 0.5,
) -> Optional[np.ndarray]:
    """後進（R）で start→goal を結ぶ最短曲線（R>=rho）。点列は start→goal 順（後進で辿る）。

    後進は「方位を反転した前進 Dubins」と幾何的に等価。点列はそのまま start→goal 順で、
    車体方位は接線+π（＝進行と逆向き）。寄り付きの「バックで差し込む」区間に使う。
    """
    sf = (start[0], start[1], _wrap(start[2] + math.pi))
    gf = (goal[0], goal[1], _wrap(goal[2] + math.pi))
    return sample_dubins(sf, gf, rho, step)


def _auto_headings(pts: np.ndarray) -> np.ndarray:
    """Per-waypoint heading [rad] from chord directions (+East/CCW)."""
    n = len(pts)
    h = np.zeros(n)
    if n < 2:
        return h
    d = np.diff(pts, axis=0)
    seg = np.arctan2(d[:, 1], d[:, 0])
    h[0] = seg[0]
    h[-1] = seg[-1]
    for i in range(1, n - 1):
        # 入射/出射方位の平均（角度の循環性を考慮）
        v = np.array([math.cos(seg[i - 1]) + math.cos(seg[i]), math.sin(seg[i - 1]) + math.sin(seg[i])])
        h[i] = math.atan2(v[1], v[0]) if np.hypot(*v) > 1e-9 else seg[i]
    return h


def plan_dubins(
    waypoints_xy,
    rho: float,
    step: float = 0.5,
    headings_deg: list[float | None] | None = None,
) -> np.ndarray:
    """Connect consecutive waypoints with shortest Dubins arcs (forward only, R>=rho).

    headings_deg: per-waypoint heading in degrees (+East/CCW); None entries are
    auto-derived from the chord directions.
    """
    pts = np.asarray(waypoints_xy, float)
    if len(pts) < 2:
        return pts.copy()
    auto = _auto_headings(pts)
    yaw = auto.copy()
    if headings_deg is not None:
        for i, hd in enumerate(headings_deg[: len(yaw)]):
            if hd is not None:
                yaw[i] = math.radians(hd)

    out: list[np.ndarray] = []
    for i in range(len(pts) - 1):
        seg = sample_dubins(
            (pts[i, 0], pts[i, 1], yaw[i]),
            (pts[i + 1, 0], pts[i + 1, 1], yaw[i + 1]),
            rho,
            step,
        )
        if seg is None:
            seg = np.vstack([pts[i], pts[i + 1]])
        out.append(seg if i == 0 else seg[1:])  # 重複端点を除去
    return np.vstack(out)
