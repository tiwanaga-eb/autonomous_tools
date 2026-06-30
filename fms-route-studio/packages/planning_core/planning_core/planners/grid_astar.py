"""コスト格子上の A*（大域コリドー抽出, 設計書 §9.2 / §9.5）。

由来: GlobalPlanner/terrain_route_planner/planner/astar.py + graph.py（8近傍・コスト重み）。
役割は「大域コリドー抽出」に限定（§9.5）。最終経路の R_min 保証は spline/dubins 平滑化が担う。

走行可能領域(drivable mask)を **ハード制約**（mask 外＝通行不可）、cost を **ソフトコスト**として併用。
ピクセル↔ワールドは rasterio affine transform で相互変換。
"""
from __future__ import annotations

import heapq

import numpy as np
from affine import Affine

_NEIGHBORS = (
    (-1, -1, 1.41421356),
    (-1, 0, 1.0),
    (-1, 1, 1.41421356),
    (0, -1, 1.0),
    (0, 1, 1.0),
    (1, -1, 1.41421356),
    (1, 0, 1.0),
    (1, 1, 1.41421356),
)


def world_to_rc(transform, x: float, y: float) -> tuple[int, int]:
    """ワールド(x,y) → 格子(row,col)。transform は rasterio Affine。

    floor で半開区間 [i, i+1) → index i に対応（セル中心 i+0.5 は確実に i へ。round だと .5 で隣へ飛ぶ）。
    """
    col, row = ~transform * (x, y)
    return int(np.floor(row)), int(np.floor(col))


def rc_to_world(transform, row: int, col: int) -> tuple[float, float]:
    """格子セル中心(row,col) → ワールド(x,y)。"""
    x, y = transform * (col + 0.5, row + 0.5)
    return float(x), float(y)


def _snap_to_free(
    passable: np.ndarray, row: int, col: int, max_cells: int | None = None
) -> tuple[int, int] | None:
    """最近傍の通行可能セルへスナップ（BFS）。max_cells を超える/見つからなければ None。

    max_cells を超える遠方スナップを禁止することで、断片化領域で始終点が遠くの別塊へ
    勝手に飛んで「偽の経路」が出るのを防ぐ（始終点が領域外なら正直に失敗させる）。
    """
    rows, cols = passable.shape
    row = min(max(row, 0), rows - 1)
    col = min(max(col, 0), cols - 1)
    if passable[row, col]:
        return row, col
    seen = np.zeros_like(passable, dtype=bool)
    heap = [(0, row, col)]
    while heap:
        dist, r, c = heapq.heappop(heap)
        if max_cells is not None and dist > max_cells:
            return None
        if seen[r, c]:
            continue
        seen[r, c] = True
        if passable[r, c]:
            return r, c
        for dr, dc, _ in _NEIGHBORS:
            nr, nc = r + dr, c + dc
            if 0 <= nr < rows and 0 <= nc < cols and not seen[nr, nc]:
                heapq.heappush(heap, (dist + 1, nr, nc))
    return None


def find_route_rc(
    cost: np.ndarray,
    passable: np.ndarray,
    start_rc: tuple[int, int],
    goal_rc: tuple[int, int],
    cell_size: float,
    max_snap_cells: int | None = None,
) -> list[tuple[int, int]]:
    """A*（8近傍・Euclidヒューリスティック）。passable 外は不可。返値は (row,col) 列。"""
    rows, cols = cost.shape
    s = _snap_to_free(passable, *start_rc, max_cells=max_snap_cells)
    g = _snap_to_free(passable, *goal_rc, max_cells=max_snap_cells)
    if s is None or g is None:
        return []
    gr, gc = g

    # soft cost は 1 以上に正規化（距離項が消えないように 1+cost_norm）
    cmax = float(cost[passable].max()) if passable.any() else 1.0
    cmax = cmax if cmax > 1e-9 else 1.0

    def w(r, c):
        return 1.0 + float(cost[r, c]) / cmax

    open_heap = [(0.0, 0.0, s[0], s[1])]
    g_score = {s: 0.0}
    came: dict = {s: None}
    visited = set()

    while open_heap:
        _, gv, r, c = heapq.heappop(open_heap)
        if (r, c) in visited:
            continue
        visited.add((r, c))
        if (r, c) == g:
            path = []
            node = g
            while node is not None:
                path.append(node)
                node = came[node]
            path.reverse()
            return path
        wc = w(r, c)
        for dr, dc, diag in _NEIGHBORS:
            nr, nc = r + dr, c + dc
            if not (0 <= nr < rows and 0 <= nc < cols):
                continue
            if not passable[nr, nc]:
                continue
            tentative = gv + diag * cell_size * 0.5 * (wc + w(nr, nc))
            nb = (nr, nc)
            if tentative < g_score.get(nb, float("inf")):
                g_score[nb] = tentative
                came[nb] = (r, c)
                h = cell_size * np.hypot(nr - gr, nc - gc)
                heapq.heappush(open_heap, (tentative + h, tentative, nr, nc))
    return []


def coarsen_grid(cost, drivable_mask, transform, factor: int, obstacle_value: float, passable_frac: float = 0.5):
    """fine 格子を factor×factor ブロックで粗格子化（大域コリドー抽出を高速・安定化）。

    cost は通行可セルの平均（無ければ obstacle）。passable はブロック内の自由率>=passable_frac。
    （収縮済みマスクは clearance 保証済みなので passable_frac<=0=「1セルでも有効なら通行可」が適切。
      未収縮なら 0.5=多数決で車体マージン近似。）new transform はブロック中心が rc_to_world で得られる。
    """
    factor = max(1, int(factor))
    if factor == 1:
        passable = (drivable_mask.astype(bool) if drivable_mask is not None else np.ones(cost.shape, bool))
        passable = passable & (cost < obstacle_value)
        return cost.astype(float), passable, transform

    h, w = cost.shape
    hc, wc = h // factor, w // factor
    if hc == 0 or wc == 0:
        passable = (cost < obstacle_value)
        if drivable_mask is not None:
            passable = passable & drivable_mask.astype(bool)
        return cost.astype(float), passable, transform

    cblk = cost[: hc * factor, : wc * factor].reshape(hc, factor, wc, factor)
    free = cblk < obstacle_value
    if drivable_mask is not None:
        mblk = (drivable_mask[: hc * factor, : wc * factor] > 0).reshape(hc, factor, wc, factor)
        free = free & mblk
    cnt = free.sum(axis=(1, 3))
    summ = np.where(free, cblk, 0.0).sum(axis=(1, 3))
    mean_cost = np.where(cnt > 0, summ / np.maximum(cnt, 1), obstacle_value)
    thresh = max(1, int(round(factor * factor * passable_frac)))
    passable = cnt >= thresh
    cost_c = np.where(passable, mean_cost, obstacle_value).astype(float)
    new_t = transform * Affine.scale(float(factor))
    return cost_c, passable, new_t


def plan_grid_astar(
    cost: np.ndarray,
    transform,
    start_xy: tuple[float, float],
    goal_xy: tuple[float, float],
    *,
    drivable_mask: np.ndarray | None = None,
    obstacle_value: float = 1e9,
    planner_cell_m: float = 1.0,
    passable_frac: float = 0.5,
    max_snap_m: float = 5.0,
) -> np.ndarray:
    """大域 A* コリドーをワールド座標 (N,2) で返す。経路が無ければ start/goal の2点。

    fine cost(0.1m等) は重いので planner_cell_m へ粗格子化してから A*（「大域コリドー抽出」§9.5）。
    drivable_mask(>0=走行可) をハード制約、cost をソフトコストに。
    passable_frac: 粗セルを通行可とみなすブロック内自由率（収縮済みマスクは 0=any が適切）。
    max_snap_m: 始終点を通行可能セルへスナップする最大距離[m]（超えたら経路なし＝偽経路防止）。
    """
    fine_cell = float(abs(transform.a))
    factor = max(1, int(round(planner_cell_m / fine_cell))) if fine_cell > 0 else 1
    cost_c, passable_c, t_c = coarsen_grid(
        cost, drivable_mask, transform, factor, obstacle_value, passable_frac=passable_frac
    )
    cell_c = float(abs(t_c.a))
    max_snap_cells = max(1, int(round(max_snap_m / cell_c)))

    s_rc = world_to_rc(t_c, *start_xy)
    g_rc = world_to_rc(t_c, *goal_xy)
    rc_path = find_route_rc(cost_c, passable_c, s_rc, g_rc, cell_c, max_snap_cells=max_snap_cells)
    if len(rc_path) < 2:
        return np.array([start_xy, goal_xy], float)

    pts = np.array([rc_to_world(t_c, r, c) for r, c in rc_path], float)
    pts[0] = start_xy
    pts[-1] = goal_xy
    return pts


def _segment_clear(a, b, mask: np.ndarray, transform, step: float) -> bool:
    """線分 a→b 上を ~step 間隔でサンプルし、全点が mask 内なら True（line-of-sight）。"""
    h, w = mask.shape
    dist = float(np.hypot(b[0] - a[0], b[1] - a[1]))
    n = max(2, int(dist / max(step, 1e-6)) + 1)
    for i in range(n + 1):
        t = i / n
        x = a[0] + (b[0] - a[0]) * t
        y = a[1] + (b[1] - a[1]) * t
        r, c = world_to_rc(transform, x, y)
        if not (0 <= r < h and 0 <= c < w) or mask[r, c] == 0:
            return False
    return True


def string_pull(pts, mask: np.ndarray, transform, step: float | None = None):
    """コリドー内で見通せる最遠点へ貪欲に短絡（funnel/糸引き）。階段状A*を taut 折れ線に。"""
    pts = np.asarray(pts, float)
    if mask is None or len(pts) < 3:
        return pts
    if step is None:
        # LOS チェックの刻みは fine cell より粗めに（0.1m×O(n^2)は重い。0.5m で十分）。
        step = max(float(abs(transform.a)), 0.5)
    out = [pts[0]]
    i = 0
    n = len(pts)
    while i < n - 1:
        j = n - 1
        while j > i + 1:
            if _segment_clear(pts[i], pts[j], mask, transform, step):
                break
            j -= 1
        out.append(pts[j])
        i = j
    return np.asarray(out, float)


def _insert_endpoint_headings(pts, mask, transform, head_start, head_goal, lead: float):
    """始終点の方位ベクトルに沿うガイド点を内側に挿入（コリドー内に入る場合のみ）。"""
    pts = [list(p) for p in np.asarray(pts, float)]
    if len(pts) < 2 or lead <= 0:
        return np.asarray(pts, float)
    h, w = (mask.shape if mask is not None else (0, 0))

    def in_mask(p):
        if mask is None:
            return True
        r, c = world_to_rc(transform, p[0], p[1])
        return 0 <= r < h and 0 <= c < w and mask[r, c] > 0

    out = [pts[0]]
    if head_start is not None:
        a = np.radians(head_start)
        gp = [pts[0][0] + lead * np.cos(a), pts[0][1] + lead * np.sin(a)]
        if in_mask(gp):
            out.append(gp)
    out += pts[1:-1]
    if head_goal is not None:
        a = np.radians(head_goal)
        gp = [pts[-1][0] - lead * np.cos(a), pts[-1][1] - lead * np.sin(a)]
        if in_mask(gp):
            out.append(gp)
    out.append(pts[-1])
    return np.asarray(out, float)


def _chaikin_open(pts):
    """開いた折れ線の Chaikin 角刈り1回（端点保持）。各辺を 1/4・3/4 で切り内側に丸める。"""
    pts = np.asarray(pts, float)
    if len(pts) < 3:
        return pts
    out = [pts[0]]
    for i in range(len(pts) - 1):
        a, b = pts[i], pts[i + 1]
        out.append(0.75 * a + 0.25 * b)
        out.append(0.25 * a + 0.75 * b)
    out.append(pts[-1])
    return np.asarray(out, float)


def smooth_polyline_in_corridor(
    pts,
    r_min: float | None,
    mask: np.ndarray | None,
    transform,
    n: int = 2000,
    frac_tol: float = 0.02,
    iters: int = 26,
    kappa_rate_max: float | None = None,
    head_start: float | None = None,
    head_goal: float | None = None,
):
    """A* 折れ線を「コリドー（drivable mask）内で最大限」平滑化（糸引き→方位ガイド→曲率/操舵レート制限）。

    手順: (1) 共線除去 (2) string-pull で階段を taut 化 (3) 始終点方位ガイド点を挿入
    (4) スプラインを s を上げて R_min と dκ/ds を満たすまで平滑化。ただし領域を逸脱したら止める
    （ハード制約＝領域包含を平滑性より優先）。満たせない場合は best-effort＋警告。
    Returns (curve, warning, measured_min_radius, out_of_corridor_fraction)。
    """
    from ..analysis.curvature import min_turning_radius
    from .spline import fit_spline_curvature_limited, resample_by_spacing

    cell0 = float(abs(transform.a))
    base = simplify_collinear(np.asarray(pts, float), tol=max(cell0 * 5, 0.5))  # 階段を事前間引き
    if mask is not None:
        base = string_pull(base, mask, transform)
    if head_start is not None or head_goal is not None:
        lead = max((r_min or 0.0) * 0.8, 4.0)
        base = _insert_endpoint_headings(base, mask, transform, head_start, head_goal, lead)
    base = np.asarray(base, float)

    if mask is None:
        curve, _s, warn, r, _dk = fit_spline_curvature_limited(base, r_min, kappa_rate_max, n)
        return curve, warn, float(r), 0.0

    cell = float(abs(transform.a))
    step = max(cell, 0.5)

    def densify(poly):
        return resample_by_spacing(poly, step) if len(poly) >= 2 else poly

    # Chaikin の角刈り（内側に丸める＝taut 経路から領域外へはみ出さない）で
    # コリドー内に留まる範囲で最大限丸める。spline interpolation の overshoot を回避。
    cur = base
    best_dense = densify(base)
    best_r = min_turning_radius(best_dense)
    best_frac = path_in_mask_fraction(best_dense, transform, mask)
    # Chaikin は反復ごとに点数が倍増するので少回数で十分（収束も速い）。
    for _ in range(min(iters, 6)):
        cand = _chaikin_open(cur)
        dense = densify(cand)
        frac = path_in_mask_fraction(dense, transform, mask)
        if frac > frac_tol:
            break  # これ以上の丸めは領域を逸脱
        cur = cand
        best_dense, best_r, best_frac = dense, min_turning_radius(dense), frac
        if r_min and best_r >= r_min:
            break  # 必要半径を満たした

    warn = None
    if r_min and best_r < r_min:
        warn = (
            f"コリドーが狭く R_min={r_min:.1f}m を満たせません"
            f"（達成 R={best_r:.1f}m）。走行可能領域の拡張・分割解消、または道幅の見直しが必要です。"
        )
    return best_dense, warn, float(best_r), float(best_frac)


def _smooth_run_in_corridor(seg, mask, transform, r_min, kappa_rate_max, frac_tol: float):
    """単一進行方向(gear)の区間をクロソイド平滑化（R_min と dκ/ds 上限を満たす連続曲率スプライン）。

    Hybrid A* の離散プリミティブ接合部は曲率が階段状（瞬間操舵）に変化するため「クネクネ」見える。
    dubins の瞬間操舵を連続化するのと同じ fit_spline_curvature_limited を区間に当てて滑らかにする。
    端点は厳密固定（cusp/始終点を保持）。コリドーを逸脱したら平滑化を捨てて生区間を返す（安全優先）。
    """
    from .spline import fit_spline_curvature_limited

    seg = np.asarray(seg, float)
    if len(seg) < 4:
        return seg
    try:
        curve, _s, _w, _r, _dk = fit_spline_curvature_limited(
            seg, r_min, kappa_rate_max=kappa_rate_max, n=max(2 * len(seg), 50)
        )
    except Exception:  # noqa: BLE001
        return seg
    if mask is not None and path_in_mask_fraction(curve, transform, mask) > frac_tol:
        return seg  # コリドー逸脱なら平滑化を捨てる
    return np.asarray(curve, float)


def smooth_kinematic_path(
    xy, gears, mask, transform, r_min: float | None = None,
    kappa_rate_max: float | None = None, frac_tol: float = 0.02,
):
    """運動学プランナ(hybrid A*/RRT*)の生経路を gear 区間ごとに平滑化して「クネクネ」を除く。

    cusp(進行方向の反転=gear変化)で区間分割し、各区間をクロソイド平滑化。cusp と始終点は保持。
    返値 (curve(N,2), gears(list[str]|None))。
    """
    xy = np.asarray(xy, float)
    if len(xy) < 4 or not gears or len(gears) != len(xy):
        return xy, gears
    # gear が変わる点（cusp）で区間分割。境界点は前区間に属し、後区間では重複を除く。
    bounds = [0]
    for i in range(1, len(xy)):
        if gears[i] != gears[i - 1]:
            bounds.append(i)
    bounds.append(len(xy) - 1)
    out_xy: list[np.ndarray] = []
    out_gears: list[str] = []
    for k in range(len(bounds) - 1):
        a, b = bounds[k], bounds[k + 1]
        sm = _smooth_run_in_corridor(xy[a : b + 1], mask, transform, r_min, kappa_rate_max, frac_tol)
        g = gears[a]
        if out_xy:  # 区間境界（cusp点）の重複を除く
            sm = sm[1:]
        out_xy.append(sm)
        out_gears.extend([g] * len(sm))
    curve = np.vstack(out_xy) if out_xy else xy
    return curve, out_gears


def path_in_mask_fraction(pts, transform, mask: np.ndarray) -> float:
    """ポリラインのうち drivable mask 外（または範囲外）に出る点の割合。コリドー逸脱の検査用。"""
    if mask is None or len(pts) == 0:
        return 0.0
    bad = 0
    h, w = mask.shape
    for x, y in pts:
        r, c = world_to_rc(transform, x, y)
        if 0 <= r < h and 0 <= c < w:
            if mask[r, c] == 0:
                bad += 1
        else:
            bad += 1
    return bad / len(pts)


def simplify_collinear(pts, tol: float = 1e-6) -> np.ndarray:
    """ほぼ共線な中間点を間引く（A* の階段状ポリラインを軽量化）。"""
    pts = np.asarray(pts, float)
    if len(pts) < 3:
        return pts.copy()
    keep = [0]
    for i in range(1, len(pts) - 1):
        a = pts[keep[-1]]
        b = pts[i]
        c = pts[i + 1]
        cross = (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
        base = np.hypot(*(c - a)) + 1e-9
        if abs(cross) / base > tol:
            keep.append(i)
    keep.append(len(pts) - 1)
    return pts[keep]
