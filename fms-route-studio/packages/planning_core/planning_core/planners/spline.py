"""Spline path generation (Phase 0).

由来: map_tool.py:178-289 (fit_spline / resample_by_spacing / fit_spline_with_min_radius)。
純関数 (numpy in/out)。最終的な Planner プロトコル統合は Phase 4。
"""
from __future__ import annotations

import numpy as np
from scipy.interpolate import splev, splprep

from ..analysis.curvature import min_turning_radius  # noqa: F401  (公開API互換のため保持)


def cumulative_lengths(pts) -> np.ndarray:
    pts = np.asarray(pts, float)
    if len(pts) < 2:
        return np.zeros(len(pts))
    seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    return np.concatenate([[0.0], np.cumsum(seg)])


def fit_spline(points_xy, s: float = 0.0, n: int = 2000, w=None) -> np.ndarray:
    """Fit a parametric B-spline through points; return n samples as (n, 2).

    w: 任意の点ごとの重み。端点を大きく重み付けすると、平滑化(s>0)でも端点をほぼ通る（始終点固定）。
    """
    pts = np.asarray(points_xy, float)
    m = len(pts)
    if m < 2:
        raise ValueError("need >= 2 points")
    k = min(3, m - 1)
    tck, _u = splprep([pts[:, 0], pts[:, 1]], s=s, k=k, w=w)
    u = np.linspace(0.0, 1.0, n)
    xs, ys = splev(u, tck)
    return np.column_stack([np.asarray(xs, float), np.asarray(ys, float)])


def resample_by_spacing(pts, spacing: float) -> np.ndarray:
    """Resample a polyline to ~uniform arc-length spacing."""
    pts = np.asarray(pts, float)
    if spacing is None or spacing <= 0:
        return pts.copy()
    s = cumulative_lengths(pts)
    total = float(s[-1])
    if total <= spacing:
        return np.vstack([pts[0], pts[-1]])
    # 均等割り（linspace）。arange+末尾追加だと末尾に極小セグメントが残り、
    # 3点外接円の曲率がスパイクするため避ける。
    nseg = max(1, int(round(total / spacing)))
    targets = np.linspace(0.0, total, nseg + 1)
    xs = np.interp(targets, s, pts[:, 0])
    ys = np.interp(targets, s, pts[:, 1])
    return np.column_stack([xs, ys])


def fit_spline_with_min_radius(
    points_xy,
    r_min: float | None,
    n: int = 2000,
    s_max: float = 1e6,
    iters: int = 18,
):
    """Increase smoothing s until the curve's min turning radius >= r_min.

    Returns (curve, used_s, warning, measured_min_radius).
    """
    if r_min is None or r_min <= 0:
        curve = fit_spline(points_xy, s=0.0, n=n)
        return curve, 0.0, None, min_turning_radius(curve)

    s = 0.0
    best = None
    best_s = 0.0
    best_r = -1.0
    for _ in range(iters):
        curve = fit_spline(points_xy, s=s, n=n)
        r = min_turning_radius(curve)
        if r > best_r:
            best_r, best, best_s = r, curve, s
        if r >= r_min:
            return curve, s, None, r
        s = 1.0 if s == 0.0 else s * 3.0
        if s > s_max:
            break
    warn = f"could not satisfy min_turn_radius={r_min:.2f}m (best={best_r:.2f}m, s={best_s})"
    return best, float(best_s), warn, float(best_r)


def _spline_with_curvature(points_xy, s: float, n: int, w=None):
    """Sample a B-spline and its analytic curvature κ(s) and arc length.

    返値: (curve(n,2), kappa(n), s_arc(n))。κ はスプライン1/2階微分から解析計算（3点法より滑らか）。
    w: 点ごとの重み（端点を重く→始終点固定）。
    """
    pts = np.asarray(points_xy, float)
    m = len(pts)
    k = min(3, m - 1)
    tck, _u = splprep([pts[:, 0], pts[:, 1]], s=s, k=k, w=w)
    u = np.linspace(0.0, 1.0, n)
    x, y = splev(u, tck)
    dx, dy = splev(u, tck, der=1)
    ddx, ddy = splev(u, tck, der=2)
    x = np.asarray(x, float); y = np.asarray(y, float)
    dx = np.asarray(dx, float); dy = np.asarray(dy, float)
    ddx = np.asarray(ddx, float); ddy = np.asarray(ddy, float)
    speed = np.hypot(dx, dy)
    kappa = (dx * ddy - dy * ddx) / (speed**3 + 1e-12)
    du = u[1] - u[0]
    s_arc = np.concatenate([[0.0], np.cumsum(0.5 * (speed[1:] + speed[:-1]) * du)])
    return np.column_stack([x, y]), kappa, s_arc


def fit_spline_curvature_limited(
    points_xy,
    r_min: float | None,
    kappa_rate_max: float | None = None,
    n: int = 2000,
    s_max: float = 1e7,
    iters: int = 26,
):
    """平滑化 s を増やし R_min と dκ/ds 上限の両方を満たすスプラインを返す（クロソイド近似）。

    Dubins/格子A* の「最小半径アーク＋曲率ステップ（瞬間操舵）」を、連続曲率・操舵レート制限つきの
    実車に則した経路へ整える（設計書 §9.5 クロソイド平滑化）。
    Returns (curve, used_s, warning, measured_min_radius, measured_max_kappa_rate)。
    """
    pts = np.asarray(points_xy, float)
    rmin = r_min if (r_min and r_min > 0) else 0.0

    # 疎な入力（数点）は平滑化の自由度が足りない → 密化してから当てる。
    # 重要: 弦上の線形補間で密化すると「直線＋頂点角」の折れ線を再現してしまう（κが0とスパイクに化ける）。
    # 平滑な補間スプライン上をサンプルして密化する。
    total = float(cumulative_lengths(pts)[-1]) if len(pts) >= 2 else 0.0
    if len(pts) < 60 and total > 1.0:
        pts = fit_spline(pts, s=0.0, n=200)

    # 端点（始点/終点）を強く重み付け → 平滑化(s>0)でも始終点をほぼ通る（置いたStart/Goalからのドリフト防止）。
    start_pt = pts[0].copy()
    goal_pt = pts[-1].copy()
    w = np.ones(len(pts))
    w[0] = w[-1] = 50.0

    def _anchor(curve):
        # 端点を厳密に Start/Goal へスナップ（重み付けで残差は微小なのでキンクは出ない）。
        curve = np.asarray(curve, float)
        curve[0] = start_pt
        curve[-1] = goal_pt
        return curve

    s = 0.0
    best = None
    for _ in range(iters):
        curve, kappa, s_arc = _spline_with_curvature(pts, s, n, w=w)
        kmax = float(np.max(np.abs(kappa))) if len(kappa) else 0.0
        rcur = (1.0 / kmax) if kmax > 1e-9 else float("inf")
        s_safe = s_arc.copy()
        for i in range(1, len(s_safe)):
            if s_safe[i] <= s_safe[i - 1]:
                s_safe[i] = s_safe[i - 1] + 1e-9
        dk = np.gradient(kappa, s_safe)
        dkmax = float(np.max(np.abs(dk))) if len(dk) else 0.0
        ok_r = rmin <= 0 or rcur >= rmin
        ok_dk = kappa_rate_max is None or dkmax <= kappa_rate_max
        if ok_r and ok_dk:
            return _anchor(curve), s, None, rcur, dkmax
        best = (curve, s, rcur, dkmax)
        s = 1.0 if s == 0.0 else s * 3.0
        if s > s_max:
            break
    if best is None:
        # 候補が1つも得られなかった（iters<=0 等の異常入力）。None のアンパックによる
        # 不可解な TypeError を避け、呼び出し側が扱える明示的なエラーにする。
        raise ValueError("spline fitting produced no candidate (invalid iters or degenerate input)")
    curve, s_used, rcur, dkmax = best
    warn = (
        f"could not fully satisfy R>={rmin:.1f}m / dκ/ds<={kappa_rate_max} "
        f"(R={rcur:.1f}m, dκ/ds={dkmax:.3g}, s={s_used:g})"
    )
    return _anchor(curve), float(s_used), warn, float(rcur), float(dkmax)
