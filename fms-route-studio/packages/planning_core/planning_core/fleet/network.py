"""道路ネットワーク／分岐（複数台制御 Phase A の基盤）。

既存経路（親）の途中から別経路を分岐させる「交差点(junction)」を扱う。分岐をピーキーにしないため、
分岐の起点姿勢は**親経路のその点の接線方向**にする（接線連続でなめらかに枝が出る）。実際の枝経路の
生成は既存の経路プランナ(plan_route / hybrid / spline)に、この起点姿勢を start として渡せばよい。

`junction_pose(points, s_frac)`: 親経路の弧長割合 s_frac(0..1) の (x, y, heading_deg) を返す。
"""
from __future__ import annotations

import math

import numpy as np


def _cum_s(pts: np.ndarray) -> np.ndarray:
    if len(pts) < 2:
        return np.zeros(len(pts))
    return np.concatenate([[0.0], np.cumsum(np.hypot(np.diff(pts[:, 0]), np.diff(pts[:, 1])))])


def junction_pose(points, s_frac: float) -> tuple[float, float, float]:
    """親経路 points の弧長割合 s_frac(0..1) における姿勢 (x, y, heading_deg) を返す。

    heading は進行方向の接線（+East/CCW[deg]）。分岐の start に使うと枝が接線方向へなめらかに出る。
    """
    pts = np.asarray(points, float)
    n = len(pts)
    if n == 0:
        return 0.0, 0.0, 0.0
    if n == 1:
        return float(pts[0, 0]), float(pts[0, 1]), 0.0
    s = _cum_s(pts)
    total = float(s[-1])
    target = max(0.0, min(1.0, float(s_frac))) * total
    i = int(np.searchsorted(s, target) - 1)
    i = max(0, min(i, n - 2))
    seg = max(s[i + 1] - s[i], 1e-9)
    f = (target - s[i]) / seg
    x = float(pts[i, 0] + f * (pts[i + 1, 0] - pts[i, 0]))
    y = float(pts[i, 1] + f * (pts[i + 1, 1] - pts[i, 1]))
    heading = math.degrees(math.atan2(pts[i + 1, 1] - pts[i, 1], pts[i + 1, 0] - pts[i, 0]))
    return x, y, heading
