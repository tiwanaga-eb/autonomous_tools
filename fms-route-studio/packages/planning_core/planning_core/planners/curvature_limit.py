"""最小旋回半径(R_min)を保証する曲率制限（設計書 §9.5）。

大域スプライン平滑化(s)はタイトコーナーで R_min を保証できないため、**違反区間だけ**を
局所ラプラシアン平滑化で開いて max|κ| <= 1/R_min に収束させる。解析と同じ3点外接円の曲率
(`curvature_profile`)で判定するので、出力の feasibility と一致する。端点は固定、走行可能 mask が
あれば領域内に留める（コリドー制約）。
"""
from __future__ import annotations

import numpy as np

from ..analysis.curvature import curvature_profile


def _inside_sampler(mask, transform):
    if mask is None or transform is None:
        return None
    inv = ~transform
    ia, ib, ic = inv.a, inv.b, inv.c
    id_, ie, if_ = inv.d, inv.e, inv.f
    h, w = mask.shape

    def inside(x: float, y: float) -> bool:
        c = int(np.floor(ia * x + ib * y + ic))
        r = int(np.floor(id_ * x + ie * y + if_))
        return 0 <= r < h and 0 <= c < w and mask[r, c] != 0

    return inside


def limit_curvature_polyline(
    points,
    r_min: float | None,
    *,
    iters: int = 1200,
    lam: float = 0.5,
    margin: float = 1.08,
    window: int = 2,
    mask=None,
    transform=None,
    fix_endpoints: bool = True,
):
    """折れ線の max|κ| を 1/r_min 以下に収める（違反箇所の近傍だけ局所平滑化）。

    違反点(κ>1/(r_min·margin))とその近傍(±window)を固定強度 lam でラプラシアン平滑化し、
    解析と同じ3点曲率で max|κ|<=1/r_min になるまで反復。compliant な区間は触らないので
    経路の大部分は配置点に忠実なまま、タイトコーナーだけが R_min まで開く。
    Returns (curve(N,2), measured_min_radius_m)。r_min<=0 なら無加工。
    """
    pts = np.asarray(points, float).copy()
    n = len(pts)
    if not r_min or r_min <= 0 or n < 3:
        prof = curvature_profile(pts)
        kmax = float(np.max(np.abs(prof["kappa"]))) if n else 0.0
        return pts, ((1.0 / kmax) if kmax > 1e-9 else float("inf"))

    k_cap = 1.0 / (r_min * margin)   # 近傍平滑化の対象しきい（合否より少し厳しめ＝安全側に開く）
    k_lim = 1.0 / r_min              # 合否ライン（解析と同じ）
    inside = _inside_sampler(mask, transform)
    lo, hi = (1, n - 1) if fix_endpoints else (0, n)

    measured = float("inf")
    for _ in range(iters):
        kappa = np.abs(curvature_profile(pts)["kappa"])
        kmax = float(kappa.max()) if n else 0.0
        measured = (1.0 / kmax) if kmax > 1e-9 else float("inf")
        if kmax <= k_lim:
            break
        viol = kappa > k_cap
        if not viol.any():
            break
        # 違反点の近傍(±window)を平滑化対象に
        sm = np.zeros(n, dtype=bool)
        for j in np.where(viol)[0]:
            sm[max(lo, j - window):min(hi, j + window + 1)] = True
        new = pts.copy()
        for i in range(lo, hi):
            if not sm[i]:
                continue
            target = 0.5 * (pts[i - 1] + pts[i + 1])
            cand = pts[i] + lam * (target - pts[i])
            if inside is None or inside(cand[0], cand[1]):
                new[i] = cand
        pts = new
    return pts, measured
