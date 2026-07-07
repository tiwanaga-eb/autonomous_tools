"""車両フットプリント（車体矩形/多角形）の包含判定（設計書 §5 / §9）。

円近似(footprint_radius)ではなく、姿勢(x,y,yaw)に置いた実フットプリント多角形が
走行可能 mask 内に収まるかを**厳密に**判定する。用途は2つ:
  1) hybrid A* の衝突判定（運動学探索中の各姿勢で）。
  2) 生成済み軌跡の包含 violation 抽出（解析・UI 表示）。

座標規約: footprint_polygon は基準点(姿勢x,y,yaw)基準 [m]、x=前後(前+) / y=左右。yaw は +East/CCW[rad]。
（基準点は車種依存で **後輪車輪軸中心**（HM400=後輪2軸中心 / HD785・HD605=後輪軸中心）。これは
 rigid_bicycle/articulated の運動学基準＝Dubins/RS/hybrid が生成する経路点と一致する。多角形を
 そのまま姿勢へ剛体変換するので、基準点が車体中心でなくても判定は正しい。）
"""
from __future__ import annotations

import numpy as np

from .models.analysis import Violation


def vehicle_footprint(vehicle) -> np.ndarray:
    """車体中心基準のフットプリント多角形 (N,2)[m]。未定義なら overall_length×overall_width の矩形。"""
    poly = getattr(vehicle, "footprint_polygon", None)
    if poly:
        return np.asarray(poly, dtype=float)
    hl = float(vehicle.overall_length) / 2.0
    hw = float(vehicle.overall_width) / 2.0
    return np.array([[hl, hw], [hl, -hw], [-hl, -hw], [-hl, hw]], dtype=float)


def _point_in_poly(px: float, py: float, poly: np.ndarray) -> bool:
    """レイキャスティングによる点in多角形（境界は概ね内側扱い）。"""
    n = len(poly)
    inside = False
    j = n - 1
    for i in range(n):
        xi, yi = poly[i]
        xj, yj = poly[j]
        if ((yi > py) != (yj > py)) and (px < (xj - xi) * (py - yi) / (yj - yi + 1e-12) + xi):
            inside = not inside
        j = i
    return inside


def footprint_sample_points(poly: np.ndarray, spacing: float) -> np.ndarray:
    """多角形を覆うサンプル点 (N,2)[車体座標]。内部を spacing 間隔の格子で、境界(頂点)も含める。

    包含判定は「全サンプル点が mask 内」で行うため、内部の障害（穴）も spacing 解像度で検出できる。
    1回だけ前計算する想定（探索ループ内では呼ばない）。
    """
    poly = np.asarray(poly, float)
    spacing = max(float(spacing), 0.2)
    xmin, ymin = poly.min(axis=0)
    xmax, ymax = poly.max(axis=0)
    xs = np.arange(xmin, xmax + 1e-6, spacing)
    ys = np.arange(ymin, ymax + 1e-6, spacing)
    if xs.size == 0:
        xs = np.array([(xmin + xmax) / 2.0])
    if ys.size == 0:
        ys = np.array([(ymin + ymax) / 2.0])
    gx, gy = np.meshgrid(xs, ys)
    grid = np.column_stack([gx.ravel(), gy.ravel()])
    keep = np.array([_point_in_poly(p[0], p[1], poly) for p in grid], dtype=bool)
    inside = grid[keep] if keep.any() else grid
    # 頂点（角）は必ず含める（最外周のはみ出しを確実に拾う）
    return np.vstack([inside, poly])


def _inv_affine(transform):
    inv = ~transform
    return inv.a, inv.b, inv.c, inv.d, inv.e, inv.f


def place_world(pts_body: np.ndarray, x: float, y: float, yaw: float) -> np.ndarray:
    """車体座標のサンプル点群を姿勢(x,y,yaw)へ回転・並進してワールド座標 (N,2) に。"""
    c, s = np.cos(yaw), np.sin(yaw)
    rot = np.array([[c, -s], [s, c]])
    return pts_body @ rot.T + np.array([x, y])


def _rc(world_pts: np.ndarray, inv) -> tuple[np.ndarray, np.ndarray]:
    ia, ib, ic, id_, ie, if_ = inv
    xs = world_pts[:, 0]
    ys = world_pts[:, 1]
    cols = np.floor(ia * xs + ib * ys + ic).astype(int)
    rows = np.floor(id_ * xs + ie * ys + if_).astype(int)
    return rows, cols


def outside_count(
    world_pts: np.ndarray, mask: np.ndarray, inv, cost: np.ndarray | None = None, obstacle: float = 1e9
) -> int:
    """サンプル点のうち走行不可（mask=0 / 範囲外 / cost>=obstacle）の数。"""
    rows, cols = _rc(world_pts, inv)
    h, w = mask.shape
    oob = (rows < 0) | (rows >= h) | (cols < 0) | (cols >= w)
    bad = oob.copy()
    rr = np.clip(rows, 0, h - 1)
    cc = np.clip(cols, 0, w - 1)
    bad |= ~oob & (mask[rr, cc] == 0)
    if cost is not None:
        bad |= ~oob & (cost[rr, cc] >= obstacle)
    return int(bad.sum())


def path_min_clearance(points: np.ndarray, mask: np.ndarray, transform) -> tuple[float, int]:
    """経路に沿った最小離隔[m]（走行可能領域境界までの距離）と発生index。

    安全検証(設計 Stage 5)の「最小離隔」チェック用。mask の距離変換を経路点でサンプルする。
    """
    if mask is None or transform is None:
        return float("inf"), -1
    from scipy.ndimage import distance_transform_edt

    cell = float(abs(transform.a))
    dt = distance_transform_edt(mask > 0) * cell
    inv = _inv_affine(transform)
    pts = np.asarray(points, float)
    rows, cols = _rc(pts, inv)
    h, w = dt.shape
    oob = (rows < 0) | (rows >= h) | (cols < 0) | (cols >= w)
    rr = np.clip(rows, 0, h - 1)
    cc = np.clip(cols, 0, w - 1)
    vals = np.where(oob, 0.0, dt[rr, cc])
    i = int(np.argmin(vals)) if len(vals) else -1
    return (float(vals[i]) if i >= 0 else float("inf")), i


def footprint_clear(pts_body: np.ndarray, x: float, y: float, yaw: float, mask: np.ndarray, inv,
                    cost: np.ndarray | None = None, obstacle: float = 1e9) -> bool:
    """姿勢(x,y,yaw)のフットプリント全体が走行可能か（1点でも外なら False）。"""
    wp = place_world(pts_body, x, y, yaw)
    return outside_count(wp, mask, inv, cost, obstacle) == 0


def trajectory_footprint_violations(traj, vehicle, mask: np.ndarray, transform, spacing: float | None = None,
                                    ignore_ends_m: float = 0.0) -> list[Violation]:
    """生成済み軌跡の各点でフットプリント包含を判定し、はみ出す連続区間を Violation(kind='footprint') で返す。

    measured = 区間内の最大はみ出し率(0..1)、limit=0。mask/transform が無ければ空。
    ignore_ends_m: 始点/終点から弧長 ignore_ends_m 以内は判定から除外する。長尺車は配置した始終点の
    外側へ車体が必ずオーバーハングするが、それは計画で動かせない既定姿勢に由来し経路の不備ではないため。
    """
    if mask is None or transform is None or vehicle is None:
        return []
    pts = traj.points
    if len(pts) < 1:
        return []
    cell = float(abs(transform.a))
    spacing = spacing if spacing else max(cell, 0.5)
    poly = vehicle_footprint(vehicle)
    samp = footprint_sample_points(poly, spacing)
    total = max(1, len(samp))
    inv = _inv_affine(transform)

    s = np.array([p.s for p in pts], float)
    s_end = float(s[-1]) if len(s) else 0.0
    fracs = np.zeros(len(pts))
    for i, p in enumerate(pts):
        yaw = np.radians(p.heading_deg)
        wp = place_world(samp, p.x, p.y, yaw)
        fracs[i] = outside_count(wp, mask, inv) / total

    out: list[Violation] = []
    over = fracs > 0.0
    if ignore_ends_m > 0.0:
        over = over & (s >= ignore_ends_m) & (s <= s_end - ignore_ends_m)
    n = len(pts)
    i = 0
    while i < n:
        if over[i]:
            j = i
            peak = fracs[i]
            while j + 1 < n and over[j + 1]:
                j += 1
                peak = max(peak, fracs[j])
            out.append(Violation(kind="footprint", s_start=float(s[i]), s_end=float(s[j]),
                                  measured=float(peak), limit=0.0))
            i = j + 1
        else:
            i += 1
    return out
