"""Curvature analysis: kappa(s), dkappa/ds, minimum turning radius.

設計書 §10 / §21(M2): κ は3点外接円ベース（map_tool.py:201-223 由来）。
dκ/ds は弧長での数値微分（spline/grid_astar 由来は参考値、Trajectory.curvature_source で区別）。
"""
from __future__ import annotations

import numpy as np


def circumradius(p1, p2, p3) -> float:
    """Radius of the circle through three points (∞ for collinear)."""
    p1 = np.asarray(p1, float)
    p2 = np.asarray(p2, float)
    p3 = np.asarray(p3, float)
    a = float(np.linalg.norm(p2 - p1))
    b = float(np.linalg.norm(p3 - p2))
    c = float(np.linalg.norm(p3 - p1))
    s = (a + b + c) / 2.0
    area = max(np.sqrt(max(s * (s - a) * (s - b) * (s - c), 0.0)), 1e-12)
    return (a * b * c) / (4.0 * area)


def curvature_profile(pts) -> dict:
    """Return {s, kappa, dkappa_ds} arrays for a polyline (N, 2).

    kappa[i] = 1 / circumradius(pts[i-1], pts[i], pts[i+1]); endpoints copy neighbour.
    dkappa_ds = d(kappa)/d(arclength) via np.gradient.
    """
    pts = np.asarray(pts, float)
    n = len(pts)
    if n < 2:
        return {"s": np.zeros(n), "kappa": np.zeros(n), "dkappa_ds": np.zeros(n)}

    seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(seg)])

    kappa = np.zeros(n)
    for i in range(1, n - 1):
        if seg[i - 1] < 1e-9 or seg[i] < 1e-9:
            kappa[i] = kappa[i - 1]
            continue
        r = circumradius(pts[i - 1], pts[i], pts[i + 1])
        kappa[i] = (1.0 / r) if (np.isfinite(r) and r > 1e-9) else 0.0
    kappa[0] = kappa[1]
    kappa[-1] = kappa[-2]

    # 単調増加する s を仮定して数値微分（重複点は微小値で回避）
    s_safe = s.copy()
    for i in range(1, n):
        if s_safe[i] <= s_safe[i - 1]:
            s_safe[i] = s_safe[i - 1] + 1e-9
    dkappa_ds = np.gradient(kappa, s_safe)
    return {"s": s, "kappa": kappa, "dkappa_ds": dkappa_ds}


def cusp_mask(pts, cos_thresh: float = -0.7, window: int = 1) -> np.ndarray:
    """方向反転点(cusp/切り返し)とその近傍を True にする bool 配列。

    連続セグメントの向きが反転（cosθ < cos_thresh）する点は、3点外接円の曲率が見かけ上∞に
    化けるため、min_radius/操舵/曲率変化率の評価から除外する（設計: 切返点は R_min 評価対象外）。
    """
    pts = np.asarray(pts, float)
    n = len(pts)
    mask = np.zeros(n, dtype=bool)
    if n < 3:
        return mask
    for i in range(1, n - 1):
        v1 = pts[i] - pts[i - 1]
        v2 = pts[i + 1] - pts[i]
        n1 = float(np.hypot(v1[0], v1[1]))
        n2 = float(np.hypot(v2[0], v2[1]))
        if n1 < 1e-9 or n2 < 1e-9:
            continue
        if float(np.dot(v1, v2)) / (n1 * n2) < cos_thresh:
            for j in range(i - window, i + window + 1):
                if 0 <= j < n:
                    mask[j] = True
    return mask


def min_turning_radius(pts) -> float:
    """Minimum turning radius along a polyline (inf for a straight line)."""
    prof = curvature_profile(pts)
    kmax = float(np.max(np.abs(prof["kappa"]))) if len(prof["kappa"]) else 0.0
    return (1.0 / kmax) if kmax > 1e-9 else float("inf")
