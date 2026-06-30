"""Elastic Band 経路最適化（Quinlan-Khatib, 設計書 §9.4）。

経路を節点列とみなし、**収縮(平滑化)力＋走行可能領域境界からの反発(クリアランス)力**で
反復的に最適化する。grid_astar/hybrid/spline 等の初期経路を「コリドー中央へ寄せ、車体が
収まる余裕(desired_clearance)を確保する」洗練ポストプロセス。

- 走行可能 mask を**ハード制約**として扱う（節点は mask 内に留める。外へ出る移動は捨てる）。
- string-pull（幾何最短）とは別系統。こちらは**余裕重視**でフットプリント包含(§9.5)と相補的。
- TEB の時間項は持たない空間版（§13 動的再計画の基盤にもなる）。
"""
from __future__ import annotations

import numpy as np


def _clearance_sampler(mask: np.ndarray, transform):
    """走行可能 mask の距離変換から clearance(x,y)[m] を返すサンプラを作る。"""
    from scipy.ndimage import distance_transform_edt

    cell = float(abs(transform.a))
    dt = distance_transform_edt(mask > 0) * cell
    inv = ~transform
    ia, ib, ic = inv.a, inv.b, inv.c
    id_, ie, if_ = inv.d, inv.e, inv.f
    h, w = dt.shape

    def clearance(x: float, y: float) -> float:
        c = int(np.floor(ia * x + ib * y + ic))
        r = int(np.floor(id_ * x + ie * y + if_))
        if 0 <= r < h and 0 <= c < w:
            return float(dt[r, c])
        return 0.0

    return clearance, cell


def elastic_band(
    points,
    mask: np.ndarray,
    transform,
    *,
    desired_clearance: float = 0.0,
    iters: int = 120,
    step: float = 0.6,
    w_smooth: float = 0.35,
    w_clear: float = 0.6,
    fix_endpoints: bool = True,
):
    """初期経路 points(N,2) を mask 内で最適化した折れ線(N,2) を返す。

    desired_clearance[m] 未満の節点を、クリアランスが増える向き（距離変換の数値勾配）へ押し出し、
    同時に隣接点へ収縮（平滑化）させる。mask 外へ出る移動は採用しない（ハード制約）。
    """
    pts = np.asarray(points, float).copy()
    n = len(pts)
    if n < 3 or mask is None or transform is None:
        return pts
    clearance, cell = _clearance_sampler(mask, transform)
    h = max(cell, 0.5)  # 数値勾配のステップ[m]
    lo = 1 if fix_endpoints else 0
    hi = n - 1 if fix_endpoints else n

    for _ in range(iters):
        moved = 0.0
        newpts = pts.copy()
        for i in range(lo, hi):
            p = pts[i]
            prev = pts[i - 1] if i > 0 else p
            nxt = pts[i + 1] if i < n - 1 else p
            # 収縮(平滑化)力: 隣接中点へ（ラプラシアン）
            disp = w_smooth * (prev + nxt - 2.0 * p)
            # クリアランス力: 余裕が desired 未満なら、余裕が増える向き(数値勾配)へ
            c0 = clearance(p[0], p[1])
            if desired_clearance > 0.0 and c0 < desired_clearance:
                gx = clearance(p[0] + h, p[1]) - clearance(p[0] - h, p[1])
                gy = clearance(p[0], p[1] + h) - clearance(p[0], p[1] - h)
                g = np.array([gx, gy])
                ng = float(np.hypot(g[0], g[1]))
                if ng > 1e-9:
                    deficit = (desired_clearance - c0) / desired_clearance  # 0..1
                    disp = disp + w_clear * deficit * (g / ng) * cell
            cand = p + step * disp
            # ハード制約: 走行可能(clearance>0)に留める。外なら移動を捨てる。
            if clearance(cand[0], cand[1]) > 0.0:
                newpts[i] = cand
                moved = max(moved, float(np.hypot(cand[0] - p[0], cand[1] - p[1])))
        pts = newpts
        if moved < 0.01:
            break
    return pts
