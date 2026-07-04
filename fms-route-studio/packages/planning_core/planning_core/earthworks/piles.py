"""排土（パイル）配置計画（earthworks）。

エリア多角形の中に、円錐パイルを格子状に均等配置する設計ツール。

パイル形状は**円錐**（安息角 φ で決まる法面勾配）:
    底面半径 r = h / tanφ,  体積 V = π r² h / 3 = π h³ / (3 tan²φ)
体積 V か高さ h のどちらかを与えれば他方が決まる。

配置モード:
- **間隔指定**: 縦横間隔 dx/dy（＋千鳥）でエリアの端（縁マージン）から詰めて格子配置。
- **撒き出し計算**: パイル体積 V と撒き出し厚 t から、1パイルが均せる面積 V/t → 推奨間隔
  d = √(V/t) と理論配置数 n = ⌊A·t/V⌋ を計算し、その間隔で配置する。

格子の向きはエリアの**最小外接矩形の主方向**（回転キャリパー）に合わせる＝斜め・細長い
エリアでも「端からいっぱいに」整列する。
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


@dataclass
class PileSpec:
    """円錐パイルの寸法。repose_deg=安息角[°]。"""

    height_m: float
    radius_m: float
    volume_m3: float
    repose_deg: float


def cone_from(volume_m3: float | None = None, height_m: float | None = None,
              repose_deg: float = 35.0) -> PileSpec:
    """体積 or 高さ（どちらか一方）＋安息角から円錐パイル寸法を求める。"""
    if not (0.0 < repose_deg < 89.0):
        raise ValueError("repose_deg must be in (0, 89)")
    t = math.tan(math.radians(repose_deg))
    if height_m is not None and height_m > 0:
        h = float(height_m)
        r = h / t
        v = math.pi * r * r * h / 3.0
    elif volume_m3 is not None and volume_m3 > 0:
        v = float(volume_m3)
        h = (3.0 * v * t * t / math.pi) ** (1.0 / 3.0)
        r = h / t
    else:
        raise ValueError("volume_m3 か height_m のどちらかを正の値で指定してください")
    return PileSpec(height_m=h, radius_m=r, volume_m3=v, repose_deg=float(repose_deg))


def _polygon_area(poly: np.ndarray) -> float:
    x, y = poly[:, 0], poly[:, 1]
    return 0.5 * abs(float(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1))))


def _convex_hull(pts: np.ndarray) -> np.ndarray:
    """Andrew's monotone chain。返値は反時計回りの凸包（閉じない）。"""
    p = np.unique(pts, axis=0)
    if len(p) <= 2:
        return p
    p = p[np.lexsort((p[:, 1], p[:, 0]))]

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower: list = []
    for q in p:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], q) <= 0:
            lower.pop()
        lower.append(q)
    upper: list = []
    for q in p[::-1]:
        while len(upper) >= 2 and cross(upper[-2], upper[-1], q) <= 0:
            upper.pop()
        upper.append(q)
    return np.asarray(lower[:-1] + upper[:-1], float)


def min_area_rect(poly) -> tuple[float, float, float, float, float]:
    """最小外接矩形（回転キャリパー）。返値 (angle_rad, cx, cy, w, h)。

    angle は矩形の「幅 w 方向」の向き。凸包の各辺方向を試す O(n²) 素朴版（エリア頂点数は小さい）。
    """
    pts = np.asarray(poly, float)
    hull = _convex_hull(pts)
    if len(hull) < 3:
        mn, mx = pts.min(0), pts.max(0)
        return 0.0, float((mn[0] + mx[0]) / 2), float((mn[1] + mx[1]) / 2), float(mx[0] - mn[0]), float(mx[1] - mn[1])
    best = None
    n = len(hull)
    for i in range(n):
        e = hull[(i + 1) % n] - hull[i]
        L = math.hypot(e[0], e[1])
        if L < 1e-9:
            continue
        a = math.atan2(e[1], e[0])
        c, s = math.cos(-a), math.sin(-a)
        rx = hull[:, 0] * c - hull[:, 1] * s
        ry = hull[:, 0] * s + hull[:, 1] * c
        w = float(rx.max() - rx.min())
        h = float(ry.max() - ry.min())
        area = w * h
        if best is None or area < best[0]:
            cx_r = (rx.min() + rx.max()) / 2
            cy_r = (ry.min() + ry.max()) / 2
            # 矩形中心を元座標へ戻す
            cc, ss = math.cos(a), math.sin(a)
            cx = cx_r * cc - cy_r * ss
            cy = cx_r * ss + cy_r * cc
            best = (area, a, float(cx), float(cy), w, h)
    _, a, cx, cy, w, h = best
    return a, cx, cy, w, h


def _point_in_poly(x: float, y: float, poly: np.ndarray) -> bool:
    inside = False
    n = len(poly)
    j = n - 1
    for i in range(n):
        xi, yi = poly[i]
        xj, yj = poly[j]
        if (yi > y) != (yj > y) and x < (xj - xi) * (y - yi) / ((yj - yi) or 1e-12) + xi:
            inside = not inside
        j = i
    return inside


def _dist_to_edges(x: float, y: float, poly: np.ndarray) -> float:
    """点から多角形の辺までの最短距離。"""
    n = len(poly)
    best = float("inf")
    for i in range(n):
        ax, ay = poly[i]
        bx, by = poly[(i + 1) % n]
        dx, dy = bx - ax, by - ay
        L2 = dx * dx + dy * dy
        t = 0.0 if L2 < 1e-12 else max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / L2))
        best = min(best, math.hypot(x - (ax + t * dx), y - (ay + t * dy)))
    return best


def place_grid(polygon, dx: float, dy: float, *, edge_margin: float = 0.0,
               stagger: bool = False, stagger_invert: bool = False,
               angle: float | None = None) -> np.ndarray:
    """多角形内に格子状の点（パイル中心）を配置する。返値 (N,2)。

    - 格子の向き angle[rad]。None はエリアの最小外接矩形の主方向。
    - 端からいっぱいに: 矩形の min 端 + edge_margin から dx/dy で敷き詰める。
    - edge_margin: 中心が多角形の辺からこの距離以上（パイル基部を内側に収めるなら基部半径）。
    - stagger: 千鳥（1行おきに dx/2 オフセット）。stagger_invert=True でオフセットする行を
      逆（偶数行←→奇数行）にする＝千鳥の斜め方向が反転する。
    """
    poly = np.asarray(polygon, float)
    if len(poly) < 3 or dx <= 0 or dy <= 0:
        return np.zeros((0, 2))
    a, cx, cy, w, h = min_area_rect(poly)
    if angle is not None:
        a = float(angle)
    c, s = math.cos(-a), math.sin(-a)
    rx = poly[:, 0] * c - poly[:, 1] * s
    ry = poly[:, 0] * s + poly[:, 1] * c
    x0, x1 = float(rx.min()) + edge_margin, float(rx.max()) - edge_margin
    y0, y1 = float(ry.min()) + edge_margin, float(ry.max()) - edge_margin
    out: list[tuple[float, float]] = []
    cc, ss = math.cos(a), math.sin(a)
    row = 0
    yy = y0
    off_parity = 0 if stagger_invert else 1
    while yy <= y1 + 1e-9:
        off = (dx / 2.0) if (stagger and row % 2 == off_parity) else 0.0
        xx = x0 + off
        while xx <= x1 + 1e-9:
            X = xx * cc - yy * ss
            Y = xx * ss + yy * cc
            if _point_in_poly(X, Y, poly) and (edge_margin <= 0 or _dist_to_edges(X, Y, poly) >= edge_margin - 1e-9):
                out.append((X, Y))
            xx += dx
        yy += dy
        row += 1
    return np.asarray(out, float) if out else np.zeros((0, 2))


def plan_piles(polygon, *, repose_deg: float = 35.0,
               volume_m3: float | None = None, height_m: float | None = None,
               dx: float | None = None, dy: float | None = None,
               spread_thickness_m: float | None = None,
               stagger: bool = False, stagger_invert: bool = False,
               edge_margin_m: float | None = None) -> dict:
    """エリア多角形へのパイル配置計画（間隔指定 or 撒き出し計算）。

    - 間隔指定: dx/dy を与える（dy 省略時は dx と同じ）。
    - 撒き出し計算: spread_thickness_m（撒き出し厚 t）を与えると、推奨間隔 d=√(V/t) と
      理論数 n=⌊A·t/V⌋ を計算して配置する（dx/dy 指定があればそれを優先）。
    - edge_margin_m: 縁からのマージン。None はパイル基部半径（基部がエリア内に収まる）。
    """
    poly = np.asarray(polygon, float)
    if len(poly) < 3:
        raise ValueError("polygon needs >= 3 vertices")
    spec = cone_from(volume_m3=volume_m3, height_m=height_m, repose_deg=repose_deg)
    area = _polygon_area(poly)

    n_theory = None
    d_suggest = None
    if spread_thickness_m and spread_thickness_m > 0:
        cover = spec.volume_m3 / float(spread_thickness_m)   # 1パイルが厚tで均せる面積[m²]
        d_suggest = math.sqrt(cover)
        n_theory = int(area / cover)
        if dx is None:
            dx = d_suggest
        if dy is None:
            dy = d_suggest
    if dx is None or dx <= 0:
        raise ValueError("間隔 dx（または撒き出し厚）を指定してください")
    if dy is None or dy <= 0:
        dy = dx

    margin = float(edge_margin_m) if edge_margin_m is not None else spec.radius_m
    centers = place_grid(poly, float(dx), float(dy), edge_margin=margin,
                         stagger=stagger, stagger_invert=stagger_invert)
    a, _cx, _cy, _w, _h = min_area_rect(poly)
    return {
        "pile": {
            "height_m": round(spec.height_m, 3),
            "radius_m": round(spec.radius_m, 3),
            "volume_m3": round(spec.volume_m3, 3),
            "repose_deg": spec.repose_deg,
        },
        "centers": [[round(float(x), 3), round(float(y), 3)] for x, y in centers],
        "count": int(len(centers)),
        "spacing": {"dx_m": round(float(dx), 3), "dy_m": round(float(dy), 3),
                    "stagger": bool(stagger), "stagger_invert": bool(stagger_invert)},
        "grid_angle_deg": round(math.degrees(a), 2),
        "edge_margin_m": round(margin, 3),
        "area_m2": round(area, 1),
        "total_volume_m3": round(spec.volume_m3 * len(centers), 1),
        "n_theory": n_theory,                 # 撒き出しモード時のみ（A·t/V）
        "suggested_spacing_m": (round(d_suggest, 3) if d_suggest else None),
    }
