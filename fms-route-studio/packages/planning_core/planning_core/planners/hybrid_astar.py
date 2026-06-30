"""Hybrid A*（運動学的A*, 設計書 §9.2 / 4a-2）。

由来: spotting_planner_ui/src/planner/hybridAstar.ts を Python へ移植・整理。
自転車モデルの曲率プリミティブ（±1/rho, 0）×(前進/後進)で状態(x,y,yaw)を離散探索し、
最小旋回半径 R>=rho をネイティブに保証、始終点の方位も尊重する。ゴール近傍では Dubins
解析接続で厳密に姿勢を合わせる。衝突は drivable mask（中心線包含）で判定。

grid_astar（位置のみの大域コリドー）と異なり、最初から運動学的に実行可能な経路を出す。
"""
from __future__ import annotations

import heapq
import math
from dataclasses import dataclass

import numpy as np

from ..footprint import _inv_affine, outside_count, place_world
from .dubins import sample_dubins
from .grid_astar import world_to_rc
from .reeds_shepp import reeds_shepp_paths


def _wrap(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def bicycle_step(x, y, yaw, curv, direction, ds):
    """定曲率アークで1ステップ進める（direction: +1前進/-1後進）。返値 (x,y,yaw)。"""
    sds = direction * ds
    if abs(curv) < 1e-9:
        return x + sds * math.cos(yaw), y + sds * math.sin(yaw), yaw
    r = 1.0 / curv
    nyaw = _wrap(yaw + sds * curv)
    cx = x - r * math.sin(yaw)
    cy = y + r * math.cos(yaw)
    return cx + r * math.sin(nyaw), cy - r * math.cos(nyaw), nyaw


@dataclass
class _Node:
    x: float
    y: float
    yaw: float
    curv: float
    direction: int
    g: float
    parent: int  # index into closed list (-1 for root)


def _heuristic(x, y, yaw, gx, gy, gyaw, rho):
    eucl = math.hypot(gx - x, gy - y)
    dh = _wrap(gyaw - yaw)
    turn = abs(dh) * rho
    return max(eucl, 0.5 * (eucl + turn))


def _holonomic_field(gx, gy, mask, cost, transform, xy_res, obstacle_value, max_cells: int = 1_500_000):
    """ゴールからの「障害物考慮・運動学無視」距離場（粗格子Dijkstra, [m]）。

    Hybrid A* のヒューリスティックをコリドー形状に沿わせ、狭い/非連結領域での到達性と
    探索効率を上げる（ユークリッドだけだと壁に向かって無駄に展開し max_iters で失敗しやすい）。
    返値 (dist(ch,cw)[m] or None, coarse_transform)。
    """
    from affine import Affine

    base = mask if mask is not None else cost
    if base is None or transform is None:
        return None, None
    cell = float(abs(transform.a))
    f = max(1, int(round(xy_res / cell)))
    sub_mask = mask[f // 2::f, f // 2::f] if mask is not None else None
    sub_cost = cost[f // 2::f, f // 2::f] if cost is not None else None
    ref = sub_mask if sub_mask is not None else sub_cost
    ch, cw = ref.shape
    if ch * cw > max_cells or ch == 0 or cw == 0:
        return None, None
    passable = np.ones((ch, cw), dtype=bool)
    if sub_mask is not None:
        passable &= sub_mask > 0
    if sub_cost is not None:
        passable &= sub_cost < obstacle_value
    coarse_T = transform * Affine.scale(f, f)

    gc_col, gr_row = (~coarse_T) * (gx, gy)
    gr, gc = int(math.floor(gr_row)), int(math.floor(gc_col))
    if not (0 <= gr < ch and 0 <= gc < cw) or not passable[gr, gc]:
        ys, xs = np.where(passable)
        if len(xs) == 0:
            return None, None
        j = int(np.argmin((ys - gr) ** 2 + (xs - gc) ** 2))
        gr, gc = int(ys[j]), int(xs[j])

    dist = np.full((ch, cw), np.inf)
    dist[gr, gc] = 0.0
    diag, orth = math.sqrt(2.0) * xy_res, float(xy_res)
    nbrs = ((-1, 0, orth), (1, 0, orth), (0, -1, orth), (0, 1, orth),
            (-1, -1, diag), (-1, 1, diag), (1, -1, diag), (1, 1, diag))
    pq = [(0.0, gr, gc)]
    while pq:
        d, r, c = heapq.heappop(pq)
        if d > dist[r, c]:
            continue
        for dr, dc, w in nbrs:
            nr, nc = r + dr, c + dc
            if 0 <= nr < ch and 0 <= nc < cw and passable[nr, nc]:
                nd = d + w
                if nd < dist[nr, nc]:
                    dist[nr, nc] = nd
                    heapq.heappush(pq, (nd, nr, nc))
    return dist, coarse_T


def hybrid_astar(
    start,
    goal,
    *,
    rho: float,
    mask: np.ndarray | None = None,
    transform=None,
    cost: np.ndarray | None = None,
    obstacle_value: float = 1e9,
    footprint: np.ndarray | None = None,
    xy_res: float = 1.0,
    yaw_res_deg: float = 15.0,
    step_len: float | None = None,
    allow_reverse: bool = False,
    max_snap_m: float = 0.0,
    pos_tol: float = 2.0,
    yaw_tol_deg: float = 20.0,
    reverse_penalty: float = 2.0,
    cusp_penalty: float = 6.0,
    steer_smooth: float = 0.5,
    soft_cost_weight: float = 1.0,
    analytic_radius: float = 12.0,
    max_iters: int = 120000,
):
    """start/goal は (x,y,yaw[rad])。返値 dict{points:[(x,y,yaw,gear)], length, n_cusps, status} or None。"""
    sx, sy, syaw = start
    gx, gy, gyaw = goal
    if step_len is None:
        step_len = max(xy_res * 1.5, 1.0)
    yaw_res = math.radians(yaw_res_deg)
    yaw_tol = math.radians(yaw_tol_deg)
    kmax = 1.0 / rho
    nyawbins = max(1, int(round(2 * math.pi / yaw_res)))

    cell = float(abs(transform.a)) if transform is not None else xy_res
    dt = None
    if mask is not None:
        from scipy.ndimage import distance_transform_edt

        dt = distance_transform_edt(mask > 0) * cell
    cmax = float(cost[cost < obstacle_value].max()) if (cost is not None and np.any(cost < obstacle_value)) else 1.0
    cmax = cmax if cmax > 1e-9 else 1.0

    # フットプリント（向き付き車体多角形）の厳密衝突判定（footprint+mask があれば有効）。
    use_fp = footprint is not None and mask is not None and transform is not None
    fp_inv = _inv_affine(transform) if use_fp else None

    def passable(x, y) -> bool:
        if mask is None and cost is None:
            return True
        r, c = world_to_rc(transform, x, y)
        if mask is not None:
            h, w = mask.shape
            if not (0 <= r < h and 0 <= c < w) or mask[r, c] == 0:
                return False
        if cost is not None:
            h, w = cost.shape
            if not (0 <= r < h and 0 <= c < w) or cost[r, c] >= obstacle_value:
                return False
        return True

    def pose_clear(x, y, yaw) -> bool:
        """姿勢(x,y,yaw)が通行可。footprint 有効時は車体全体、無効時は中心点で判定。"""
        if not use_fp:
            return passable(x, y)
        wp = place_world(footprint, x, y, yaw)
        return outside_count(wp, mask, fp_inv, cost, obstacle_value) == 0

    def soft(x, y) -> float:
        if cost is None:
            return 0.0
        r, c = world_to_rc(transform, x, y)
        h, w = cost.shape
        if 0 <= r < h and 0 <= c < w and cost[r, c] < obstacle_value:
            return float(cost[r, c]) / cmax
        return 0.0

    def clearance(x, y) -> float:
        if dt is None:
            return float("inf")
        r, c = world_to_rc(transform, x, y)
        h, w = dt.shape
        return float(dt[r, c]) if (0 <= r < h and 0 <= c < w) else 0.0

    # start/goal が走行不可なら近傍の通行可へスナップ（端点が僅かに領域外で即失敗するのを防ぐ）。
    def _snap(x, y, yaw):
        if max_snap_m <= 0 or pose_clear(x, y, yaw):
            return x, y
        rr = cell
        while rr <= max_snap_m + 1e-9:
            for deg in range(0, 360, 20):
                a = math.radians(deg)
                cx, cy = x + rr * math.cos(a), y + rr * math.sin(a)
                if pose_clear(cx, cy, yaw):
                    return cx, cy
            rr += cell
        return x, y

    sx, sy = _snap(sx, sy, syaw)
    gx, gy = _snap(gx, gy, gyaw)

    # 障害物考慮ヒューリスティック（ゴールからの粗格子Dijkstra距離場）。
    h_field, h_T = _holonomic_field(gx, gy, mask, cost, transform, xy_res, obstacle_value)

    def heuristic(x, y, yaw) -> float:
        nonholo = _heuristic(x, y, yaw, gx, gy, gyaw, rho)
        if h_field is not None:
            r, c = world_to_rc(h_T, x, y)
            if 0 <= r < h_field.shape[0] and 0 <= c < h_field.shape[1]:
                d = float(h_field[r, c])
                if math.isfinite(d):
                    return max(nonholo, d)
        return nonholo

    def arc_ok(x, y, yaw, curv, direction) -> bool:
        """プリミティブ円弧を細かくサンプルし全姿勢 pose_clear か（footprint 有効時は車体全体）。"""
        nsub = max(2, int(step_len / max(cell, 0.3)) + 1)
        for i in range(1, nsub + 1):
            d = step_len * i / nsub
            px, py, pyaw = bicycle_step(x, y, yaw, curv, direction, d)
            if not pose_clear(px, py, pyaw):
                return False
        return True

    def analytic_shot(x, y, yaw):
        """ゴールへ解析接続。前進(Dubins)を優先し、allow_reverse なら Reeds-Shepp（曲がりながらの
        後進＝角度付き切り返し）も試す。返値: [(x,y,yaw,gear), ...]（先頭は始姿勢）or None。"""
        # 前進 Dubins
        fwd = sample_dubins((x, y, yaw), (gx, gy, gyaw), rho, step=max(cell, 0.4))
        if fwd is not None:
            poses = []
            prev = (x, y, yaw)
            ok = True
            for kk, (px, py) in enumerate(fwd):
                pyaw = yaw if kk == 0 else math.atan2(py - prev[1], px - prev[0])
                if not pose_clear(px, py, pyaw):
                    ok = False
                    break
                poses.append((px, py, pyaw, "F"))
                prev = (px, py, pyaw)
            if ok:
                return poses
        # Reeds-Shepp（後進可）。曲がりながらの後進で cusp を角度付きに。最短が衝突しても
        # 次善のRS候補を順に試す（短い順に検証し、最初に全姿勢クリアな1本を採用）。
        if allow_reverse:
            for rs in reeds_shepp_paths((x, y, yaw), (gx, gy, gyaw), rho, step=max(cell, 0.4)):
                if all(pose_clear(px, py, pyaw) for (px, py, pyaw, _g) in rs):
                    return list(rs)
        return None

    # 中間曲率(±0.5κmax)も含め、滑らかな旋回・緩い曲がりの選択肢を増やす（半径2ρ≥ρ なのでR_min順守）。
    curvs = (-kmax, -0.5 * kmax, 0.0, 0.5 * kmax, kmax)
    prims = [(c, +1) for c in curvs] + ([(c, -1) for c in curvs] if allow_reverse else [])

    start_node = _Node(sx, sy, syaw, 0.0, 1, 0.0, -1)
    open_heap = [(heuristic(sx, sy, syaw), 0.0, 0)]
    nodes = [start_node]
    g_best: dict = {}
    visited: set = set()

    def key(x, y, yaw):
        return (int(round(x / xy_res)), int(round(y / xy_res)), int(round(_wrap(yaw) / yaw_res)) % nyawbins)

    g_best[key(sx, sy, syaw)] = 0.0
    iters = 0
    final_shot = None
    goal_idx = -1

    while open_heap and iters < max_iters:
        iters += 1
        _, gv, idx = heapq.heappop(open_heap)
        nd = nodes[idx]
        k = key(nd.x, nd.y, nd.yaw)
        if k in visited:
            continue
        visited.add(k)

        # ゴール判定 or 解析接続
        if math.hypot(gx - nd.x, gy - nd.y) <= max(analytic_radius, pos_tol):
            shot = analytic_shot(nd.x, nd.y, nd.yaw)
            if shot is not None:
                final_shot = shot
                goal_idx = idx
                break
        if math.hypot(gx - nd.x, gy - nd.y) <= pos_tol and abs(_wrap(gyaw - nd.yaw)) <= yaw_tol:
            goal_idx = idx
            break

        for curv, direction in prims:
            nx, ny, nyaw = bicycle_step(nd.x, nd.y, nd.yaw, curv, direction, step_len)
            if not passable(nx, ny) or not arc_ok(nd.x, nd.y, nd.yaw, curv, direction):
                continue
            nk = key(nx, ny, nyaw)
            edge = step_len * (1.0 + (reverse_penalty if direction < 0 else 0.0))
            edge += soft_cost_weight * step_len * soft(nx, ny)
            edge += steer_smooth * abs(curv - nd.curv)
            if direction != nd.direction:
                edge += cusp_penalty
            cl = clearance(nx, ny)
            if cl < 1.0:
                edge += 2.0 * (1.0 / max(cl, 0.05) - 1.0)
            ng = nd.g + edge
            if ng < g_best.get(nk, float("inf")):
                g_best[nk] = ng
                child = _Node(nx, ny, nyaw, curv, direction, ng, idx)
                nodes.append(child)
                ci = len(nodes) - 1
                h = heuristic(nx, ny, nyaw)
                heapq.heappush(open_heap, (ng + h, ng, ci))

    if goal_idx < 0:
        return None

    # 経路復元（root→goal）
    chain = []
    i = goal_idx
    while i != -1:
        chain.append(nodes[i])
        i = nodes[i].parent
    chain.reverse()

    pts: list[tuple] = []
    cusps = 0
    prev_dir = chain[0].direction
    for j, node in enumerate(chain):
        gear = "R" if node.direction < 0 else "F"
        if j == 0:
            pts.append((node.x, node.y, node.yaw, gear))
            continue
        if node.direction != prev_dir:
            cusps += 1
            prev_dir = node.direction
        # 親から本ノードまでのアークを補間
        par = chain[j - 1]
        nsub = max(1, int(step_len / max(cell, 0.4)))
        for s in range(1, nsub + 1):
            d = step_len * s / nsub
            px, py, pyaw = bicycle_step(par.x, par.y, par.yaw, node.curv, node.direction, d)
            pts.append((px, py, pyaw, gear))

    if final_shot is not None:
        prev_gear = pts[-1][3] if pts else "F"
        for k2 in range(1, len(final_shot)):  # 先頭は探索末端ノードと重複
            x2, y2, yaw2, g2 = final_shot[k2]
            if g2 != prev_gear:
                cusps += 1  # 解析接続内のギア変化（角度付き切り返し含む）
                prev_gear = g2
            pts.append((float(x2), float(y2), float(yaw2), g2))
        last_gear = pts[-1][3]
        pts[-1] = (gx, gy, gyaw, last_gear)

    length = sum(
        math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1]) for i in range(1, len(pts))
    )
    return {"points": pts, "length": length, "n_cusps": cusps, "status": "OK", "iters": iters}
