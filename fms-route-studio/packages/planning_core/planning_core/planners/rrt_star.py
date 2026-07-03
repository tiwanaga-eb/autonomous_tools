"""RRT*（サンプリングベース最適経路, 設計書 §9.5）。

ランダムに姿勢をサンプルし、Reeds-Shepp（R_min を守る前進＋後進曲線）をステアリング関数に
使って木を伸ばす。各ノード追加時に近傍内で親を貼り替え（rewire）、反復とともに経路長が
最適へ漸近する（RRT*）。狭所・複雑形状で hybrid A* の格子分解能に縛られない経路を探せ、
後進つき RS 接続のおかげで角度のついた切り返しも自然に出る。

衝突は hybrid A* と同じく drivable mask（footprint 有効時は車体多角形, 無効時は中心点）で判定。
最後にショートカット平滑化（非隣接姿勢を RS で直結できれば置換）で冗長な切り返しを削る。

乱数は seed 付き（既定 1）で**決定的**。式の不備に対しても RS 候補は終点検証済みのものだけが
使われるため、誤った（運動学的に実行不能な）経路を返さない。
"""
from __future__ import annotations

import math
import random
from dataclasses import dataclass, field

import numpy as np

from ..footprint import _inv_affine, outside_count, place_world
from .grid_astar import world_to_rc
from .reeds_shepp import reeds_shepp_paths


@dataclass
class _Node:
    x: float
    y: float
    yaw: float
    cost: float                       # root からの累積 RS 弧長
    parent: int                       # closed list の index（root は -1）
    seg: list = field(default_factory=list)  # 親→本ノードの (x,y,yaw,gear) 列（先頭=親姿勢）


def _path_len(pts) -> float:
    return sum(
        math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1]) for i in range(1, len(pts))
    )


def _truncate(pts, max_len: float):
    """点列を弧長 max_len で切り詰める（max_len<=0 はそのまま）。各点に yaw/gear があるので途中切りも有効。"""
    if max_len <= 0 or len(pts) < 2:
        return pts
    acc = 0.0
    out = [pts[0]]
    for i in range(1, len(pts)):
        d = math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1])
        if acc + d > max_len and len(out) >= 2:
            break
        acc += d
        out.append(pts[i])
    return out


def rrt_star(
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
    rs_step: float = 0.0,
    max_iters: int = 1500,
    goal_bias: float = 0.1,
    step_max: float = 0.0,
    connect_radius: float = 0.0,
    goal_tol: float = 0.8,
    goal_yaw_tol_deg: float = 12.0,
    extra_after_solution: int = 400,
    smooth_passes: int = 60,
    soft_cost_weight: float = 1.0,
    cusp_penalty: float = 4.0,
    seed: int = 1,
    bounds=None,
):
    """start/goal は (x,y,yaw[rad])。返値 dict{points:[(x,y,yaw,gear)], length, n_cusps, status} or None。

    - mask/transform/cost/footprint は hybrid_astar と同じ占有表現。
    - step_max>0 で 1 回の伸長を弧長制限（既定 0 = サンプル姿勢へ全接続）。
    - connect_radius>0 で rewire 近傍半径（既定 0 = シーン規模から自動）。
    """
    sx, sy, syaw = start
    gx, gy, gyaw = goal
    cell = float(abs(transform.a)) if transform is not None else xy_res
    if rs_step <= 0:
        rs_step = max(cell, 0.4)
    goal_yaw_tol = math.radians(goal_yaw_tol_deg)
    rng = random.Random(seed)
    w_yaw = 0.7 * rho  # 姿勢距離での yaw 重み（角度→弧長換算の目安）

    # ---- 占有判定（hybrid_astar と同じ規約）----
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
        if not use_fp:
            return passable(x, y)
        wp = place_world(footprint, x, y, yaw)
        return outside_count(wp, mask, fp_inv, cost, obstacle_value) == 0

    def seg_clear(pts) -> bool:
        return all(pose_clear(px, py, pyaw) for (px, py, pyaw, _g) in pts)

    # ---- サンプリング境界 ----
    if bounds is not None:
        xmin, ymin, xmax, ymax = bounds
    elif transform is not None and mask is not None:
        h, w = mask.shape
        xs, ys = [], []
        for cc, rr in ((0, 0), (w, 0), (0, h), (w, h)):
            wx, wy = transform * (cc, rr)
            xs.append(wx)
            ys.append(wy)
        xmin, xmax = min(xs), max(xs)
        ymin, ymax = min(ys), max(ys)
    else:
        pad = max(20.0, 0.6 * math.hypot(gx - sx, gy - sy))
        xmin, xmax = min(sx, gx) - pad, max(sx, gx) + pad
        ymin, ymax = min(sy, gy) - pad, max(sy, gy) + pad

    diag = math.hypot(xmax - xmin, ymax - ymin)
    if step_max <= 0:
        step_max = max(8.0, 0.18 * diag)
    if connect_radius <= 0:
        connect_radius = max(step_max * 1.5, 12.0)

    def pose_dist(ax, ay, ayaw, bx, by, byaw) -> float:
        d = math.hypot(bx - ax, by - ay)
        dy = abs((byaw - ayaw + math.pi) % (2.0 * math.pi) - math.pi)
        return d + w_yaw * dy

    # コスト(斜面/粗さ)積分と cusp ペナルティを加えた「最適化用コスト」。純幾何長だけだと
    # 高コスト域を平気で通り cusp も無料になるため、hybrid A* と同じ志向に揃える。
    cmax = float(cost[cost < obstacle_value].max()) if (cost is not None and np.any(cost < obstacle_value)) else 1.0
    cmax = cmax if cmax > 1e-9 else 1.0

    def soft_at(x, y) -> float:
        if cost is None or transform is None:
            return 0.0
        r, c = world_to_rc(transform, x, y)
        h, w = cost.shape
        if 0 <= r < h and 0 <= c < w and cost[r, c] < obstacle_value:
            return float(cost[r, c]) / cmax
        return 0.0

    def edge_cost(pts) -> float:
        geom = _path_len(pts)
        if cost is None and cusp_penalty <= 0:
            return geom
        soft_sum = 0.0
        cusps = 0
        for i in range(1, len(pts)):
            ds = math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1])
            if cost is not None:
                soft_sum += soft_at((pts[i][0] + pts[i - 1][0]) / 2, (pts[i][1] + pts[i - 1][1]) / 2) * ds
            if pts[i][3] != pts[i - 1][3]:
                cusps += 1
        return geom + soft_cost_weight * soft_sum + cusp_penalty * cusps

    def connect(a, b):
        """姿勢 a→b を RS で衝突なく結ぶ最短候補 (points, 最適化コスト) を返す。無ければ None。
        コストは弧長＋ソフトコスト積分＋cusp ペナルティ（純幾何長は report 時に別途算出）。"""
        for pts in reeds_shepp_paths(a, b, rho, rs_step):
            if seg_clear(pts):
                return pts, edge_cost(pts)
        return None

    # start が即不可なら失敗（呼び出し側でスナップ済みを想定）
    if not pose_clear(sx, sy, syaw):
        return None

    nodes = [_Node(sx, sy, syaw, 0.0, -1, [(sx, sy, syaw, "F")])]
    children: list[list[int]] = [[]]  # nodes と並走する子リスト（rewire のコスト伝播に必要）

    # 近傍クエリ用の並列座標バッファ。最近傍(1)/near集合(4) を Python の総当り
    # min()/内包で回すと O(n) のインタプリタコストが支配的になるため、numpy で一括計算する
    # （複雑度は同じ O(n²) だがベクトル化で実効 ~2桁高速。max_iters 規模では十分）。
    _cap = max_iters + 4
    _nx = np.empty(_cap)
    _ny = np.empty(_cap)
    _nyaw = np.empty(_cap)
    _nx[0], _ny[0], _nyaw[0] = sx, sy, syaw

    def _dists_to_all(bx: float, by: float, byaw: float, n: int) -> np.ndarray:
        d = np.hypot(_nx[:n] - bx, _ny[:n] - by)
        dy = np.abs((byaw - _nyaw[:n] + math.pi) % (2.0 * math.pi) - math.pi)
        return d + w_yaw * dy
    goal_node = -1
    best_cost = float("inf")
    solved_at = -1

    def add_node(node: _Node, parent: int) -> int:
        nodes.append(node)
        children.append([])
        nid = len(nodes) - 1
        if nid < _cap:  # 近傍クエリ用バッファへも登録
            _nx[nid], _ny[nid], _nyaw[nid] = node.x, node.y, node.yaw
        if parent >= 0:
            children[parent].append(nid)
        return nid

    def reparent(i: int, new_parent: int, new_seg, new_cost: float) -> None:
        """ノード i を new_parent へ貼り替え、差分コストを子孫へ伝播（RRT* のコスト不変条件を維持）。"""
        old = nodes[i].parent
        if old >= 0 and i in children[old]:
            children[old].remove(i)
        nodes[i].parent = new_parent
        nodes[i].seg = new_seg
        delta = new_cost - nodes[i].cost
        nodes[i].cost = new_cost
        children[new_parent].append(i)
        stack = list(children[i])
        while stack:
            k = stack.pop()
            nodes[k].cost += delta
            stack.extend(children[k])

    for it in range(max_iters):
        if solved_at >= 0 and it - solved_at > extra_after_solution:
            break
        # 1) サンプル（goal バイアス）
        if rng.random() < goal_bias:
            rx, ry, ryaw = gx, gy, gyaw
        else:
            rx = rng.uniform(xmin, xmax)
            ry = rng.uniform(ymin, ymax)
            ryaw = rng.uniform(-math.pi, math.pi)
            if not passable(rx, ry):
                continue

        # 2) 最近傍（numpy 一括距離計算）
        ni = int(np.argmin(_dists_to_all(rx, ry, ryaw, len(nodes))))
        near = nodes[ni]

        # 3) 伸長（step_max で切り詰め、その端点を新ノード姿勢に）
        conn = connect((near.x, near.y, near.yaw), (rx, ry, ryaw))
        if conn is None:
            continue
        full_pts, _flen = conn
        seg = _truncate(full_pts, step_max)
        if len(seg) < 2:
            continue
        nxx, nyy, nyaw = seg[-1][0], seg[-1][1], seg[-1][2]
        if not pose_clear(nxx, nyy, nyaw):
            continue

        # 4) 近傍集合の中で最小コスト親を選ぶ（choose-parent）
        near_ids = np.nonzero(_dists_to_all(nxx, nyy, nyaw, len(nodes)) <= connect_radius)[0].tolist()
        best_parent, best_seg, best_pcost = ni, seg, near.cost + edge_cost(seg)
        for i in near_ids:
            cand = connect((nodes[i].x, nodes[i].y, nodes[i].yaw), (nxx, nyy, nyaw))
            if cand is None:
                continue
            c_pts, c_cost = cand
            if nodes[i].cost + c_cost < best_pcost - 1e-6:
                best_parent, best_seg, best_pcost = i, c_pts, nodes[i].cost + c_cost

        new = _Node(nxx, nyy, nyaw, best_pcost, best_parent, best_seg)
        new_id = add_node(new, best_parent)

        # 5) rewire（新ノード経由で近傍が安くなれば貼り替え＋子孫へコスト伝播）
        for i in near_ids:
            if i == best_parent:
                continue
            cand = connect((nxx, nyy, nyaw), (nodes[i].x, nodes[i].y, nodes[i].yaw))
            if cand is None:
                continue
            c_pts, c_cost = cand
            if new.cost + c_cost < nodes[i].cost - 1e-6:
                reparent(i, new_id, c_pts, new.cost + c_cost)

        # 6) ゴール接続を試す（rewire で goal の祖先が安くなっている可能性 → best_cost を更新）
        if goal_node >= 0:
            best_cost = nodes[goal_node].cost
        gc = connect((nxx, nyy, nyaw), (gx, gy, gyaw))
        if gc is not None:
            g_pts, g_cost = gc
            total = new.cost + g_cost
            if total < best_cost - 1e-6:
                best_cost = total
                if goal_node < 0:
                    goal_node = add_node(_Node(gx, gy, gyaw, total, new_id, g_pts), new_id)
                else:
                    reparent(goal_node, new_id, g_pts, total)
                if solved_at < 0:
                    solved_at = it

    if goal_node < 0:
        # ゴール姿勢が許容圏内に来た任意ノードからの直接接続を最後に総当り（保険）
        for i in range(len(nodes)):
            if pose_dist(nodes[i].x, nodes[i].y, nodes[i].yaw, gx, gy, gyaw) > connect_radius:
                continue
            gc = connect((nodes[i].x, nodes[i].y, nodes[i].yaw), (gx, gy, gyaw))
            if gc is not None:
                g_pts, g_cost = gc
                if nodes[i].cost + g_cost < best_cost:
                    best_cost = nodes[i].cost + g_cost
                    goal_node = add_node(_Node(gx, gy, gyaw, best_cost, i, g_pts), i)
        if goal_node < 0:
            return None

    # ---- 経路復元（root→goal）----
    chain = []
    i = goal_node
    guard = 0
    while i != -1 and guard < len(nodes) + 5:
        chain.append(nodes[i])
        i = nodes[i].parent
        guard += 1
    chain.reverse()

    pts: list[tuple] = [chain[0].seg[0]]
    for node in chain:
        for p in node.seg[1:]:  # 先頭は親姿勢と重複
            pts.append((float(p[0]), float(p[1]), float(p[2]), p[3]))

    # ---- ショートカット平滑化（非隣接姿勢を RS 直結できれば置換し冗長 cusp を削る）----
    pts = _shortcut(pts, connect, rng, smooth_passes)

    # 終点スナップ
    if pts:
        pts[-1] = (gx, gy, gyaw, pts[-1][3])

    cusps = sum(1 for k in range(1, len(pts)) if pts[k][3] != pts[k - 1][3])
    return {
        "points": pts,
        "length": _path_len(pts),
        "n_cusps": cusps,
        "status": "OK",
        "n_nodes": len(nodes),
    }


def _shortcut(pts, connect, rng, passes: int):
    """ランダムに 2 点 i<j を選び、姿勢 i→j を RS で短く直結できれば区間を置換。"""
    if len(pts) < 4 or passes <= 0:
        return pts
    for _ in range(passes):
        n = len(pts)
        if n < 4:
            break
        i = rng.randint(0, n - 3)
        j = rng.randint(i + 2, n - 1)
        a = (pts[i][0], pts[i][1], pts[i][2])
        b = (pts[j][0], pts[j][1], pts[j][2])
        conn = connect(a, b)
        if conn is None:
            continue
        new_seg, _cost = conn
        new_len = _path_len(new_seg)  # 平滑化は幾何長で比較（コストは最適化用なので分離）
        old_len = _path_len(pts[i:j + 1])
        if new_len < old_len - 1e-3:
            pts = pts[:i] + [(float(p[0]), float(p[1]), float(p[2]), p[3]) for p in new_seg] + pts[j + 1:]
    return pts


def sample_rrt_star(start, goal, rho: float, **kw):
    """簡便ラッパ: 経路点列 (x,y,yaw,gear) を返す。失敗なら None。"""
    r = rrt_star(start, goal, rho=rho, **kw)
    return r["points"] if r else None
