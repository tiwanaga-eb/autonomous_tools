"""動的パスシミュレータ: エリア内の寄り付き（spotting / approach）計画（設計書 §13）。

ターゲット姿勢へ低速・高精度に寄り付くマニューバを生成する。切り返し回数を選択:
  max_switchbacks = 0   … 前進のみ
                  = 1   … 1回切り返し（前進→ステージ姿勢→後進で差し込み）

v2: 直線一辺倒を避けるため**多様な候補**（前進: 直進＋横オフセット中点経由 / 1切り返し: ステージ
距離×横オフセット×進入方位の格子）を生成し、**コスト関数**で最良を選ぶ。
  score = w_distance·全長 + w_time·所要時間 + w_reverse·後進距離 + w_switchback·切返回数
        + w_costmap·(コストマップ積分)  （+ クリアランス不足ペナルティ / 領域外は不可）
前進は Dubins、後進は reverse_dubins（方位反転 Dubins）で R>=rho を保証。
"""
from __future__ import annotations

import math
import os
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass, field

import numpy as np

from ..footprint import (
    _inv_affine,
    footprint_clear,
    footprint_sample_points,
    outside_count,
    place_world,
    vehicle_footprint,
)
from ..planners.dubins import plan_dubins, reverse_dubins, sample_dubins
from ..planners.reeds_shepp import reeds_shepp_paths
from .stationary import (
    headings as _headings,
    insert_endpoint_margin as _insert_endpoint_margin,
    resolve_no_stationary,
    segments_from_hybrid as _segments_from_hybrid,
    states_to_tuples as _states_to_tuples,
    straight_pts as _straight_pts,
)


@dataclass
class CostWeights:
    w_distance: float = 1.0      # 全長[m]
    w_time: float = 0.0          # 所要時間[s]
    w_reverse: float = 1.0       # 後進距離[m]（追加ペナルティ）
    w_switchback: float = 8.0    # 切り返し1回あたり
    w_costmap: float = 2.0       # コストマップ積分（正規化cost×m）
    w_turn: float = 6.0          # 総旋回量[rad]（蛇行=クネクネを抑制。大きいほど直線的）
    w_clearance: float = 0.0     # 境界余裕[m]への報酬（大=エリア中央寄りで頑健。score から減算）
    w_footprint: float = 200.0   # 車体はみ出し率(0..1)ペナルティ（best-effort選択で最小違反を選ぶ）


@dataclass
class SpottingResult:
    points: list[dict] = field(default_factory=list)  # {x,y,heading_deg,gear,s,t}
    switch_points: list[tuple] = field(default_factory=list)
    length_total: float = 0.0
    length_fwd: float = 0.0
    length_rev: float = 0.0
    time_total: float = 0.0
    n_switchbacks: int = 0
    min_clearance_m: float | None = None
    cost_integral: float = 0.0
    total_turn_rad: float = 0.0
    score: float = float("inf")
    approach_error_m: float = 0.0
    approach_error_deg: float | None = None  # 目標方位との差[°]（絶対値, 0..180。tyaw 供給時のみ）
    feasible: bool = False
    status: str = "NO_PATH"
    footprint_inside: bool | None = None   # 車体フットプリントが全姿勢でエリア内か（None=未評価）
    fp_max_frac: float = 0.0               # 最悪姿勢での車体はみ出し率(0..1)
    fp_worst_s: float | None = None        # はみ出し最悪点の弧長[m]（UI/診断用）
    allow_stationary: bool = True          # 据え切り(端点その場操舵)を許したか（解決後の実効値）
    endpoint_margin_start_m: float = 0.0   # 出発端に入れた直線リードイン長[m]（0=なし＝据え切り）
    endpoint_margin_goal_m: float = 0.0    # 到着端に入れた直線リードアウト長[m]（0=なし＝据え切り）
    min_cusp_margin_m: float | None = None  # 内部cusp(切り返し点)の実効直線マージン最小値[m]（None=内部cuspなし。~0=狭所で挿入不可＝据え切り必要）


def _polyline_len(pts: np.ndarray) -> float:
    if len(pts) < 2:
        return 0.0
    return float(np.sum(np.hypot(*np.diff(pts, axis=0).T)))


def _perp(yaw: float):
    return (-math.sin(yaw), math.cos(yaw))


def _adapt_margin(tip_dir, P, ux, uy, gear, margin, step, fp_ok):
    """cusp のオーバーシュート tip がエリア内に収まる最大マージンを選ぶ（狭窄エリア対応）。

    tip = P + tip_dir·m·(ux,uy)。fp_ok が無ければ margin をそのまま返す。tip 区間（P→tip）の
    数点で車体フットプリントを判定し、はみ出さない最大の m を採用。**どの長さでもはみ出す場合は
    0.0（挿入なし）**を返す — 包含はハード制約（安全ゲート）であり、直線マージンは追従性の
    ヒューリスティックなので、狭所では「マージンよりも収まること」を優先する（cusp では停止する
    ため、最悪でも停止中の据え切りで追従できる）。
    """
    if fp_ok is None or margin <= 0:
        return margin
    yaw = math.atan2(uy, ux) + (math.pi if gear == "R" else 0.0)
    for m in (margin, 0.66 * margin, 0.4 * margin, 0.2 * margin, 0.1 * margin):
        ok = True
        nchk = max(2, int(m / max(step, 1e-6)) + 1)
        for i in range(1, nchk + 1):
            d = tip_dir * m * i / nchk
            if not fp_ok(P[0] + d * ux, P[1] + d * uy, yaw):
                ok = False
                break
        if ok:
            return m
    return 0.0


def _apply_cusp_margins(segments, margin: float, step: float, fp_ok=None):
    """任意の segments=[(pts,gear),...] の各 F↔R 境界(cusp)に直線マージンを挿入する。

    cusp 点 P で「進行方向へ margin だけオーバーシュート→同地点へ後進で戻る」を挟むことで、
    P 前後を同一直線（ステア0°）にして追従可能にする。RS/hybrid など内部に cusp を持つ候補にも
    一律で適用できる（前進のみ候補は cusp が無いので不変）。返値 同形式。

    fp_ok(x,y,yaw)->bool を与えると、オーバーシュート tip がエリアをはみ出さない範囲まで
    マージンを自動短縮する（狭窄エリアで cusp 周りの車体はみ出しを防ぐ）。
    """
    if margin <= 0 or len(segments) < 2:
        return segments
    segs = [([tuple(map(float, p)) for p in np.asarray(pts, float)], gear) for pts, gear in segments]
    for k in range(1, len(segs)):
        pp, pg = segs[k - 1]
        cp, cg = segs[k]
        if pg == cg or len(pp) < 2 or len(cp) < 1:
            continue
        P, a = pp[-1], pp[-2]
        dx, dy = P[0] - a[0], P[1] - a[1]
        L = math.hypot(dx, dy)
        if L < 1e-6:
            continue
        ux, uy = dx / L, dy / L
        m = _adapt_margin(1.0, P, ux, uy, pg, margin, step, fp_ok)  # tip は前進方向(+)へ
        if m <= 1e-9:
            continue  # どの長さでもはみ出す → 挿入なし（cusp 停止中の据え切りで追従）
        Pp = (P[0] + m * ux, P[1] + m * uy)
        pp.extend(_straight_pts(P, Pp, step)[1:])           # 前セグ: P→Pp 直進オーバーシュート（同ギア）
        segs[k] = (_straight_pts(Pp, P, step) + cp[1:], cg)  # 現セグ: Pp→P 直進(後進)＋元の続き
        segs[k - 1] = (pp, pg)
    return segs


def _cusp_margins_of(points: list[dict]) -> list[float]:
    """内部 cusp（ギア反転点）ごとの実効直線マージン[m]を最終点列から実測する（診断用）。

    _adapt_margin は狭所でマージンを短縮・スキップ（=0）し得るが、その事実は挿入時に
    記録されない。そこで解決後の点列から cusp 前後の「方位ドリフト2°以内の弧長」を測り、
    短い側をその cusp の実効マージンとする。~0 は据え切りが必要な cusp を意味し、
    据え切り禁止時の UI 警告に使う。
    """
    out: list[float] = []
    n = len(points)
    for i in range(1, n):
        if points[i]["gear"] == points[i - 1]["gear"]:
            continue
        h0 = points[i - 1]["heading_deg"]

        def _run(idxs, base_s):
            r = 0.0
            for j in idxs:
                if abs((points[j]["heading_deg"] - h0 + 180.0) % 360.0 - 180.0) > 2.0:
                    break
                r = abs(points[j]["s"] - base_s)
            return r

        back = _run(range(i - 1, -1, -1), points[i - 1]["s"])
        fwd = _run(range(i, n), points[i]["s"])
        out.append(min(back, fwd))
    return out


def _stage_segments(start, S, Syaw, target, rho, step, margin, fp_ok=None,
                    start_lead: float = 0.0, goal_lead: float = 0.0):
    """前進(start→S)＋後進(S→target) を、**切り返し点 S に直線マージン**を挟んで構成する。

    切り返し点では実車はステアリングを 0°(直進)にしてから前後を反転する必要がある。Dubins の
    円弧が S で直接出会うと S で曲率≠0 となり追従不能。そこで S の手前 margin[m] を直線にして、
    前進は「Dubins→直線でSへ」、後進は「Sから直線で戻ってから旋回」。これで S 前後が同一直線
    （ステア0°）になり追従できる。返値 [(fwd_pts,"F"),(rev_pts,"R")] or None。

    fp_ok を与えると、手前マージン区間(S_pre→S)がエリアをはみ出さない範囲までマージンを短縮する。

    start_lead / goal_lead > 0（据え切り禁止）: **端点直線を生成時から織り込む**。
    出発は start から start_lead 直進した姿勢を Dubins の始点に、到着は target の手前
    goal_lead（車体方位の +方向。後進ドックでは通過順に target'→target）を終点にする。
    後付けのスプライス（Dubins 再接続）より確実で、狭所でも端点が曲がった候補が選ばれない。
    """
    sx0, sy0, syaw0 = float(start[0]), float(start[1]), float(start[2])
    tx0, ty0, tyaw0 = float(target[0]), float(target[1]), float(target[2])
    pre: list = []
    start_eff = (sx0, sy0, syaw0)
    if start_lead > 0:
        s1 = (sx0 + start_lead * math.cos(syaw0), sy0 + start_lead * math.sin(syaw0))
        pre = _straight_pts((sx0, sy0), s1, step)      # start→(直進)→start'
        start_eff = (s1[0], s1[1], syaw0)
    post: list = []
    target_eff = (tx0, ty0, tyaw0)
    if goal_lead > 0:
        t1 = (tx0 + goal_lead * math.cos(tyaw0), ty0 + goal_lead * math.sin(tyaw0))
        post = _straight_pts(t1, (tx0, ty0), step)     # target'→(直線後進)→target
        target_eff = (t1[0], t1[1], tyaw0)

    c, s = math.cos(Syaw), math.sin(Syaw)
    # 手前マージン区間 S_pre→S（前進ヘッディング=Syaw）がエリア内に収まるよう margin を適応短縮。
    if margin > 0:
        margin = _adapt_margin(-1.0, S, c, s, "F", margin, step, fp_ok)
    if margin <= 1e-9:  # マージン無し（狭所で挿入余地なし or 指定0）: S へ直接接続
        f = sample_dubins(start_eff, (S[0], S[1], Syaw), rho, step)
        r = reverse_dubins((S[0], S[1], Syaw), target_eff, rho, step)
        if f is None or r is None:
            return None
        return [(pre + list(f)[1:] if pre else list(f), "F"),
                (list(r) + post[1:] if post else list(r), "R")]
    S_pre = (S[0] - margin * c, S[1] - margin * s)  # S の手前（進入方位の逆へ margin）
    f_dub = sample_dubins(start_eff, (S_pre[0], S_pre[1], Syaw), rho, step)
    r_dub = reverse_dubins((S_pre[0], S_pre[1], Syaw), target_eff, rho, step)
    if f_dub is None or r_dub is None:
        return None
    fwd = (pre + list(f_dub)[1:] if pre else list(f_dub)) + _straight_pts(S_pre, S, step)[1:]  # …→S_pre→(直線)→S
    rev = _straight_pts(S, S_pre, step) + list(r_dub)[1:]          # S→(直線後進)→S_pre→…→target'
    if post:
        rev = rev + post[1:]                                        # →(直線後進)→target
    return [(fwd, "F"), (rev, "R")]


def _allow_stationary_default(vehicle) -> bool:
    """据え切り（端点その場操舵）の既定可否。履帯(その場旋回可)は許可、ホイール車は禁止。"""
    if vehicle is None:
        return False
    if getattr(vehicle, "kinematic_type", None) == "tracked_skid" or getattr(vehicle, "can_turn_in_place", False):
        return True
    return False


def _point_in_poly(x: float, y: float, poly) -> bool:
    """レイキャスティングで点(x,y)が多角形 poly=[(x,y),...] の内側か判定。"""
    n = len(poly)
    if n < 3:
        return False
    inside = False
    j = n - 1
    for i in range(n):
        xi, yi = poly[i][0], poly[i][1]
        xj, yj = poly[j][0], poly[j][1]
        if ((yi > y) != (yj > y)) and (x < (xj - xi) * (y - yi) / ((yj - yi) or 1e-12) + xi):
            inside = not inside
        j = i
    return inside


def _world_to_rc(transform, x, y):
    col, row = ~transform * (x, y)
    return int(math.floor(row)), int(math.floor(col))


def _rc_vec(inv, xs, ys):
    """逆アフィン inv=(a,b,c,d,e,f) で world(xs,ys)→(rows,cols) を一括変換（毎回逆行列を作らない）。"""
    ia, ib, ic, id_, ie, if_ = inv
    cols = np.floor(ia * xs + ib * ys + ic).astype(np.intp)
    rows = np.floor(id_ * xs + ie * ys + if_).astype(np.intp)
    return rows, cols


def _auto_workers(n_workers: int, n_items: int) -> int:
    """並列ワーカ数を決める。0=自動（CPU数-1, 上限8）。候補が少なければ直列（スレッド起動損を回避）。"""
    if n_items < 24:
        return 1
    w = n_workers if n_workers and n_workers > 0 else min((os.cpu_count() or 2) - 1, 8)
    return max(1, w)


def _pmap(fn, items, n_workers: int):
    """fn を items に適用。ベクトル化された数値処理は GIL を解放するためスレッドで実効並列化される。
    候補が少ない/ワーカ1なら直列（オーバーヘッド回避）。返値は items と同順のリスト。"""
    items = list(items)
    workers = _auto_workers(n_workers, len(items))
    if workers <= 1:
        return [fn(x) for x in items]
    with ThreadPoolExecutor(max_workers=workers) as ex:
        return list(ex.map(fn, items))


def _footprint_scan(xs, ys, head_deg, s_arr, fp_samp, fp_inv, mask, ignore_ends_m, scan_ds: float = 0.0):
    """各姿勢に車体フットプリントを置き、走行可能 mask 外に出るサンプル点を数える（**全姿勢を一括ベクトル化**）。

    返値 (inside, max_frac, worst_s)。inside=全姿勢でフットプリントがエリア内（端点除外区間を除く）。
    max_frac=最悪姿勢のはみ出し率(0..1)、worst_s=その弧長[m]。始点/終点の固定姿勢は車体が
    既定でオーバーハングしうるため ignore_ends_m[m] 以内を判定から除外する（経路の不備ではない）。

    入力は _build が持つ配列そのまま（dict 化前）— 40万dict/呼び出しの生成コストを避ける。
    scan_ds>0 なら判定姿勢を弧長 scan_ds 間隔に間引く（端点は常に含む）。車体は10m級・サンプル点
    間隔~1m なので 0.6m 間引きでの検出漏れは実質ない（経路ステップ0.3mの全点判定は過剰）。
    """
    n = len(xs)
    total = max(1, len(fp_samp))
    if n == 0:
        return True, 0.0, None
    if scan_ds > 0.0 and n > 3 and float(s_arr[-1]) > scan_ds:
        targets = np.arange(0.0, float(s_arr[-1]), scan_ds)
        idx = np.unique(np.concatenate([np.searchsorted(s_arr, targets), [n - 1]]))
        idx = np.clip(idx, 0, n - 1)
        xs, ys, head_deg, s_arr = xs[idx], ys[idx], head_deg[idx], s_arr[idx]
    yaws = np.radians(head_deg)
    c = np.cos(yaws)
    sn = np.sin(yaws)
    mx = fp_samp[:, 0]
    my = fp_samp[:, 1]
    # ワールド座標 (P,M): 各姿勢でサンプル点を回転・並進
    wx = xs[:, None] + np.outer(c, mx) - np.outer(sn, my)
    wy = ys[:, None] + np.outer(sn, mx) + np.outer(c, my)
    ia, ib, ic, id_, ie, if_ = fp_inv
    cols = np.floor(ia * wx + ib * wy + ic).astype(np.intp)
    rows = np.floor(id_ * wx + ie * wy + if_).astype(np.intp)
    h, w = mask.shape
    oob = (rows < 0) | (rows >= h) | (cols < 0) | (cols >= w)
    rr = np.clip(rows, 0, h - 1)
    cc = np.clip(cols, 0, w - 1)
    bad = oob | (~oob & (mask[rr, cc] == 0))
    frac = bad.sum(axis=1) / total          # (P,) 姿勢ごとのはみ出し率
    if ignore_ends_m > 0.0:
        s_end = float(s_arr[-1])
        consider = (s_arr >= ignore_ends_m) & (s_arr <= s_end - ignore_ends_m)
        frac = np.where(consider, frac, 0.0)
    i = int(np.argmax(frac))
    max_frac = float(frac[i])
    inside = max_frac <= 0.0
    worst_s = float(s_arr[i]) if max_frac > 0.0 else None
    return inside, max_frac, worst_s


def _build(segments, speed_fwd, speed_rev, transform, mask, dt, cost, obstacle_value, cmax, footprint, tx, ty,
           fp_samp=None, fp_inv=None, ignore_ends_m=0.0, tyaw=None, scan_ds: float = 0.0,
           light: bool = False):
    """segments=[(pts, gear), ...] → SpottingResult（メトリクス・コスト積分・クリアランス算出）。

    fp_samp/fp_inv（車体サンプル点＋逆アフィン）を与えると **向き付きフットプリントの包含**を
    厳密判定し、feasible=「全姿勢で車体がエリア内（切り返し点含む）」とする。未指定時は従来の
    中心線クリアランス(≥footprint)で判定（後方互換）。
    """
    # --- セグメントを1本に連結（重複端点を除く）。数値計算は連結後に一括ベクトル化する。---
    switch_points: list[tuple] = []
    total_turn = 0.0  # 総旋回量[rad]（蛇行指標。セグメント内のみ集計＝cuspの180°は除外）
    xs_parts, ys_parts, head_parts, gear_parts = [], [], [], []
    prev_gear = None
    first = True
    for pts, gear in segments:
        pts = np.asarray(pts, float)
        if len(pts) < 2:
            continue
        head = _headings(pts, gear)
        if len(head) >= 2:
            dh = np.diff(head)
            dh = (dh + math.pi) % (2 * math.pi) - math.pi  # (-π,π]
            total_turn += float(np.sum(np.abs(dh)))
        if prev_gear is not None and gear != prev_gear:
            switch_points.append((float(pts[0, 0]), float(pts[0, 1])))
        prev_gear = gear
        sl = slice(0, None) if first else slice(1, None)  # 連結時は重複端点(各seg先頭)を落とす
        xs_parts.append(pts[sl, 0])
        ys_parts.append(pts[sl, 1])
        head_parts.append(head[sl])
        gear_parts.append(np.full(pts[sl].shape[0], gear == "R"))
        first = False

    if not xs_parts:
        return SpottingResult(status="NO_PATH")

    X = np.concatenate(xs_parts)
    Y = np.concatenate(ys_parts)
    HEAD = np.concatenate(head_parts)
    is_rev = np.concatenate(gear_parts)  # 各点のギアが R か
    n = len(X)

    ds = np.empty(n)
    ds[0] = 0.0
    ds[1:] = np.hypot(np.diff(X), np.diff(Y))
    s_arr = np.cumsum(ds)
    spd = np.where(is_rev, speed_rev, speed_fwd)
    t_arr = np.cumsum(ds / np.maximum(spd, 1e-6))
    length_rev = float(ds[is_rev].sum())
    length_fwd = float(ds.sum() - length_rev)

    inv = _inv_affine(transform) if transform is not None else None

    # コスト積分（区間中点で評価）— ベクトル化
    cost_integral = 0.0
    if cost is not None and inv is not None and n >= 2:
        mxx = 0.5 * (X[1:] + X[:-1])
        myy = 0.5 * (Y[1:] + Y[:-1])
        rr, cc = _rc_vec(inv, mxx, myy)
        h, w = cost.shape
        ok = (rr >= 0) & (rr < h) & (cc >= 0) & (cc < w)
        cv = cost[np.clip(rr, 0, h - 1), np.clip(cc, 0, w - 1)]
        good = ok & (cv < obstacle_value)
        cost_integral = float(np.sum((cv[good] / cmax) * ds[1:][good]))

    # クリアランス（距離変換を経路点でサンプル）— ベクトル化
    cmin = float("inf") if mask is not None else None
    if mask is not None and dt is not None and inv is not None:
        rr, cc = _rc_vec(inv, X, Y)
        h, w = mask.shape
        oob = (rr < 0) | (rr >= h) | (cc < 0) | (cc >= w)
        cmin = -1.0 if bool(oob.any()) else float(dt[rr, cc].min())

    head_deg = np.degrees(HEAD)
    if light:
        # 軽量モード（候補の粗ランク付け用）: 点列 dict と診断（cusp マージン実測）を省く。
        # 数千候補 × 数百点の dict 生成が粗評価コストの大半を占めるため。points は空。
        states = []
    else:
        # dict 構築は tolist() 経由（np スカラーの float() 個別変換より数倍速い。
        # 候補2千件×数百点=数十万 dict を作るため、ここの定数倍が全体に効く）。
        xl, yl, hl = X.tolist(), Y.tolist(), head_deg.tolist()
        sl, tl = s_arr.tolist(), t_arr.tolist()
        gl = np.where(is_rev, "R", "F").tolist()
        states = [
            {"x": xl[i], "y": yl[i], "heading_deg": hl[i], "gear": gl[i], "s": sl[i], "t": tl[i]}
            for i in range(n)
        ]

    # 到達方位誤差[°]（目標 yaw が与えられた場合のみ。P-008「一発到達精度 ±0.5m/±5°」の方位側）
    err_deg = None
    if tyaw is not None and n:
        d = (float(head_deg[-1]) - math.degrees(tyaw)) % 360.0
        err_deg = float(min(d, 360.0 - d))
    # 内部 cusp の実効直線マージン（診断）。manual_switch_pose 等の early-return 経路も
    # 含め全結果に付くよう、最終選択時ではなくここで実測する。
    cms = _cusp_margins_of(states) if (switch_points and states) else []
    res = SpottingResult(
        points=states, switch_points=switch_points,
        length_total=length_fwd + length_rev, length_fwd=length_fwd, length_rev=length_rev,
        time_total=float(t_arr[-1]) if n else 0.0, n_switchbacks=max(0, len(switch_points)),
        min_clearance_m=(cmin if cmin is not None and math.isfinite(cmin) else cmin),
        cost_integral=cost_integral,
        total_turn_rad=total_turn,
        approach_error_m=float(np.hypot(X[-1] - tx, Y[-1] - ty)) if n else 1e9,
        approach_error_deg=err_deg,
        min_cusp_margin_m=(round(min(cms), 2) if cms else None),
    )
    # フットプリント包含（向き付き）が使えるならそれをハード制約に。狭窄エリアで切り返し点を
    # 含む全姿勢の車体がエリア内に収まるかを厳密判定する（ユーザー要件）。
    if fp_samp is not None and mask is not None and transform is not None and n:
        inside, max_frac, worst_s = _footprint_scan(X, Y, head_deg, s_arr, fp_samp, fp_inv, mask,
                                                    ignore_ends_m, scan_ds)
        res.footprint_inside = inside
        res.fp_max_frac = max_frac
        res.fp_worst_s = worst_s
        res.feasible = inside
        res.status = "OK" if inside else "FOOTPRINT_OUTSIDE"
    else:
        res.feasible = (cmin is None) or (cmin >= footprint)
        res.status = "OK" if res.feasible else "COLLISION"
    return res


def _seg_headings(pts):
    """点列の各点ヘッディング(接線)[rad]。footprint向き判定用（矩形なので前後は等価）。"""
    pts = np.asarray(pts, float)
    d = np.diff(pts, axis=0)
    ang = np.arctan2(d[:, 1], d[:, 0])
    return np.concatenate([ang[:1], ang]) if len(ang) else np.zeros(len(pts))


def _accept_curve(curve, accept):
    """平滑化後の曲線 curve(Nx2) の全点が accept(x,y,yaw)=エリア内かを判定。"""
    h = _seg_headings(curve)
    return all(accept(float(curve[i, 0]), float(curve[i, 1]), float(h[i])) for i in range(len(curve)))


def _seg_dk_p95(pts):
    """点列の |dκ/ds| の代表値（解析と同じく savgol 平滑後に微分＝ピーク dκ/ds と max|κ|）。

    返値 (peak_dk, max_kappa)。peak_dk は速度を律速する最大 dκ/ds（操舵レート制限の効く点）。
    解析(_robust_dkappa)と整合させ、量子化スパイクを savgol で均してから評価する。
    """
    from ..analysis.curvature import curvature_profile
    from ..analysis.trajectory import _robust_dkappa

    pr = curvature_profile(np.asarray(pts, float))
    k = np.abs(pr["kappa"])
    dk = np.abs(_robust_dkappa(pr["kappa"], pr["s"]))
    return (float(dk.max()) if len(dk) else 0.0, float(k.max()) if len(k) else 0.0)


def _smooth_segment(pts, accept, r_min, step, *, lock_start_m: float = 0.0, lock_end_m: float = 0.0):
    """gear区間1本(Nx2)を**平滑化スプライン**で滑らかにし、曲率の不連続(dκ/ds スパイク)と蛇行を抑える。

    RS の円弧-直線接合や hybrid のプリミティブ切替で κ が階段状に飛ぶと dκ/ds→∞ となり、操舵レート
    制限で速度が落ちる。C2連続なスプライン(平滑度 s>0)に置き換えると曲率が連続化して dκ/ds が下がり、
    s を上げると蛇行（曲率振動）も減る。スプラインは始終点が僅かにずれるので、**弧長に沿った線形補正**で
    始終点を厳密一致へ戻す。複数の s から「曲率上限を守りエリア内(accept)に収まる中で dκ/ds が最小」を選ぶ。

    lock_start_m / lock_end_m: 端から弧長この値以内を**ロック（不変）**にする。cusp 端では直進マージン
    （ステア0°で前後反転する区間）を保持しないと、前進終了と後進開始でヨー角が食い違って飛ぶため、
    cusp 側は margin 長をロックする。テーパでロック境界から滑らかに平滑化を立ち上げ、段差を作らない。
    """
    pts = np.asarray(pts, float)
    n = len(pts)
    if n < 6:
        return pts
    from ..planners.spline import fit_spline, resample_by_spacing

    L = float(np.sum(np.hypot(*np.diff(pts, axis=0).T)))
    if L < 1e-6:
        return pts
    # 元経路を弧長等間隔へ（スプライン目標とインデックス対応させてブレンドするため）。
    orig = np.asarray(resample_by_spacing(pts, max(step, 0.1)), float)
    orig[0] = pts[0]
    orig[-1] = pts[-1]
    npts = len(orig)
    if npts < 6:
        return pts
    seglen = np.hypot(*np.diff(orig, axis=0).T)
    s_arr = np.concatenate([[0.0], np.cumsum(seglen)])
    Ls = float(s_arr[-1])
    lo, hi = float(lock_start_m), Ls - float(lock_end_m)
    if hi <= lo:  # ロックで平滑化余地が無い（短いセグメント）→ 不変
        return pts
    taper = max(2.0 * step, 1.0)
    # 端ごとの重み: ロックがある端(cusp側)はロック域=0からテーパで立ち上げる。ロック0の端(path端)は
    # 端点直後からフル平滑化(=1)＝始点直後の曲率立ち上がりも平滑化できる（始終点位置は別途ピン留め）。
    ws = np.clip((s_arr - lo) / taper, 0.0, 1.0) if lock_start_m > 0 else np.ones_like(s_arr)
    we = np.clip((hi - s_arr) / taper, 0.0, 1.0) if lock_end_m > 0 else np.ones_like(s_arr)
    win = np.minimum(ws, we)
    win[(s_arr <= lo) | (s_arr >= hi)] = 0.0
    win = win[:, None]
    kmax = (1.0 / (r_min * 0.9)) if (r_min and r_min > 0) else 1e9  # R_min を僅かに超える丸めは許容
    base_peak, _ = _seg_dk_p95(pts)
    best = pts
    best_peak = base_peak
    fcol = np.linspace(0.0, 1.0, npts)[:, None]
    # 平滑度 s を弧長スケール基準で広めに（大 s ほど clothoid 的に κ 変化が緩む）。
    for s in (0.1 * L, 0.3 * L, 0.7 * L, 1.5 * L, 3.0 * L, 6.0 * L, 12.0 * L):
        try:
            curve = np.asarray(fit_spline(pts, s=s, n=npts), float)
        except Exception:  # noqa: BLE001
            continue
        # 端点ドリフトを弧長一次で補正して始終点を厳密一致（姿勢・到達精度を保つ）。
        curve = curve + (pts[0] - curve[0]) * (1.0 - fcol) + (pts[-1] - curve[-1]) * fcol
        # ロック＆テーパ適用した目標位置（ロック域では orig のまま＝cusp 直進マージンを保持）。
        tgt = orig + win * (curve - orig)
        # **各点クランプ・ブレンド**: 元位置→目標へ、車体がエリア内に収まる最大割合だけ寄せる。
        # はみ出す点は元位置を保つ（丸ごと棄却しない＝狭隘でも余裕のある所だけ滑らかになる）。
        h = _seg_headings(tgt)
        blended = orig.copy()
        for i in range(1, npts - 1):
            if win[i, 0] <= 0.0:  # ロック域は不変
                continue
            for fr in (1.0, 0.75, 0.5, 0.25):
                cand = orig[i] + fr * (tgt[i] - orig[i])
                if accept(float(cand[0]), float(cand[1]), float(h[i])):
                    blended[i] = cand
                    break
        peak, kmx = _seg_dk_p95(blended)
        if kmx > kmax:                       # R_min より急な曲率は作らない
            continue
        if peak < best_peak - 1e-9:          # ピーク dκ/ds（速度律速点）を最小化
            best, best_peak = blended, peak
    return best


def _hybrid_candidate(start, target, rho, cost, mask, transform, obstacle_value, allow_reverse, ev,
                      margin=0.0, step=0.3, fp_ok=None, footprint=None, wrap=None):
    """コストに沿って滑らかに曲がる cost-aware 候補（hybrid A*, 後進可）。失敗時 None。
    内部 cusp にも直線マージンを挿入して追従可能にする。

    footprint: 車体サンプル点（footprint_sample_points）。与えると**探索自体が向き付き車体の
    包含を制約**する＝狭いエリアでも「車体が収まる」N点ターンを hybrid が構築できる
    （従来は中心点判定で探索→ ev の footprint 評価で全滅し、狭所で解が出なかった）。

    1回目は粗い格子（速い）。不成立/はみ出しなら **細格子＋細ヨーで再試行**（狭所は格子が粗いと
    切り返しの置き場が量子化で消えるため）。細試行は max_iters でバウンドする。
    """
    from ..planners.hybrid_astar import hybrid_astar

    attempts = [
        dict(xy_res=max(rho * 0.3, 1.0), yaw_res_deg=15.0, pos_tol=max(rho * 0.3, 1.5), max_iters=50000),
    ]
    # 細格子リトライは「footprint 制約つきの狭所」専用（粗い格子では切り返しの置き場が量子化で
    # 消えるため）。footprint が無い不成立は幾何でなくクリアランス等が原因＝細格子でも解けないので
    # やらない。反復上限は自由セル数でスケールし、解が無いケースの全探索を防ぐ。
    if footprint is not None and mask is not None:
        free = int((mask > 0).sum())
        attempts.append(dict(xy_res=max(rho * 0.12, 0.7), yaw_res_deg=10.0,
                             pos_tol=max(rho * 0.15, 1.0),
                             max_iters=int(min(150000, max(30000, free * 10)))))
    best = None
    for a in attempts:
        res = hybrid_astar(
            start, target, rho=rho, mask=mask, transform=transform, cost=cost,
            obstacle_value=obstacle_value, allow_reverse=allow_reverse,
            footprint=footprint,
            analytic_radius=max(2.5 * rho, 8.0),
            soft_cost_weight=2.0, reverse_penalty=2.0, cusp_penalty=6.0, **a,
        )
        if res is None:
            continue
        segs = _segments_from_hybrid(res["points"])
        if not segs:
            continue
        if wrap is not None:
            # 据え切り禁止: start′/goal′ で計画した経路に、元の固定姿勢までの直線リードを接合する。
            # 直線の走行ギアは隣接セグメントと一致が条件（不一致=この変種は成立しない）。
            (p_start, s_gear, s_lead), (p_goal, t_gear, t_lead) = wrap
            if s_lead > 0:
                if segs[0][1] != s_gear:
                    continue
                pre = _straight_pts((p_start[0], p_start[1]),
                                    (float(segs[0][0][0][0]), float(segs[0][0][0][1])), step)
                segs[0] = (pre + [tuple(map(float, q)) for q in np.asarray(segs[0][0], float)[1:]], segs[0][1])
            if t_lead > 0:
                if segs[-1][1] != t_gear:
                    continue
                post = _straight_pts((float(segs[-1][0][-1][0]), float(segs[-1][0][-1][1])),
                                     (p_goal[0], p_goal[1]), step)
                segs[-1] = ([tuple(map(float, q)) for q in np.asarray(segs[-1][0], float)[:-1]] + post, segs[-1][1])
        c = ev(_apply_cusp_margins(segs, margin, step, fp_ok))
        if best is None or (c.feasible and not best.feasible) or \
           (c.feasible == best.feasible and c.fp_max_frac < best.fp_max_frac):
            best = c
        if best is not None and best.feasible:
            break  # 粗い試行で収まったら細試行は不要（速度優先）
    return best


def _score(res: SpottingResult, w: CostWeights) -> float:
    clear_reward = w.w_clearance * float(res.min_clearance_m) if (
        w.w_clearance and res.min_clearance_m is not None and math.isfinite(res.min_clearance_m)
    ) else 0.0
    return (
        w.w_distance * res.length_total
        + w.w_time * res.time_total
        + w.w_reverse * res.length_rev
        + w.w_switchback * res.n_switchbacks
        + w.w_costmap * res.cost_integral
        + w.w_turn * res.total_turn_rad
        + w.w_footprint * res.fp_max_frac   # 車体はみ出しを強く忌避（best-effort で最小違反を選ぶ）
        - clear_reward                      # 境界余裕が大きいほど低スコア（中央寄り＝頑健）
    )


def plan_spotting(
    start,
    target,
    rho: float,
    max_switchbacks: int | None = 1,
    *,
    step: float = 0.3,
    drivable_mask=None,
    transform=None,
    cost=None,
    obstacle_value: float = 1e9,
    footprint_radius: float = 0.0,
    speed_fwd: float = 3.0,
    speed_rev: float = 1.5,
    weights: CostWeights | None = None,
    require_switchback: bool = False,
    manual_switch_pose=None,
    switchback_zone=None,
    cusp_margin_m: float | None = None,
    vehicle=None,
    footprint_ignore_ends_m: float = 0.0,
    method: str = "auto",
    smooth_path: bool = True,
    smooth_iters: int = 60,
    n_workers: int = 0,
    allow_stationary_steer: bool | None = None,
) -> SpottingResult:
    """start 姿勢→target 姿勢への寄り付き経路（コスト関数で最良候補を選択）。

    method: 候補生成アルゴリズムの選択。
      "auto"        … 全手法(Dubins/RS/Hybrid)を生成しコスト関数で最良を選ぶ（既定）。
      "dubins"      … 前進Dubins＋ステージ姿勢の切り返し(Dubins)のみ。
      "reeds_shepp" … Reeds-Shepp 全体接続（複数半径ρを列挙＝可変半径近似）のみ。
      "hybrid_astar"… Hybrid A* のコスト考慮探索のみ（drivable_mask か cost が必要）。
    manual_switch_pose 指定時は method に依らず手動ステージ経路を生成する。

    require_switchback=True かつ切り返し可（max_switchbacks>=1）のとき、前進のみ候補を除外し
    **必ず後進を含む**（バックで差し込む）経路にする。

    vehicle を与え drivable_mask/transform もある場合、**向き付き車体フットプリント**で衝突判定し、
    「切り返し点を含む全姿勢で車体がエリア内に収まる」候補だけを feasible とする（円近似ではなく
    実車体矩形）。狭窄エリアでは cusp マージン短縮・ステージ姿勢の事前フィルタも自動で働く。
    footprint_ignore_ends_m: 始終点（固定姿勢）から弧長この値以内をフットプリント判定から除外。

    manual_switch_pose=(x,y,yaw) を与えると、その姿勢を**ステージ姿勢 S として固定**し、
    前進(start→S)＋後進(S→target) の1切り返し経路だけを生成する（自動候補探索はしない）。
    手動で切り返し点を指定したいとき用。

    switchback_zone=[(x,y),...] を与えると、**切り返し点（cusp）がその多角形内にある候補だけ**を
    採用する（前進のみ候補は制約対象外）。鉱山/土木で「ここでしか切り返せない」エリアを表現する。

    cusp_margin_m: 切り返し点に入れる直線マージン[m]。切り返し点ではステア0°(直進)で前後を反転
    する必要があり、円弧が直接出会うと追従不能。手前を直線にして追従可能にする（None=自動）。
    """
    start = (float(start[0]), float(start[1]), float(start[2]))
    target = (float(target[0]), float(target[1]), float(target[2]))
    sx, sy, syaw = start
    tx, ty, tyaw = target
    w = weights or CostWeights()
    # 切り返し直線マージン（既定: R_min の 0.4 倍か 3m の大きい方）。
    margin = float(cusp_margin_m) if (cusp_margin_m and cusp_margin_m > 0) else max(0.4 * rho, 3.0)
    # 据え切り（出発/到着の端点で停止中に操舵）の可否。None は車種既定（履帯=可, ホイール=不可）。
    allow_stationary = allow_stationary_steer if allow_stationary_steer is not None else _allow_stationary_default(vehicle)
    # 据え切り禁止時は端点直線リードを**候補の生成時から**織り込む（後付けスプライスより確実）。
    end_lead = 0.0 if allow_stationary else margin
    cmax = float(cost[cost < obstacle_value].max()) if (cost is not None and np.any(cost < obstacle_value)) else 1.0
    cmax = cmax if cmax > 1e-9 else 1.0

    dt = None
    if drivable_mask is not None and transform is not None:
        from scipy.ndimage import distance_transform_edt

        dt = distance_transform_edt(drivable_mask > 0) * float(abs(transform.a))

    # --- 向き付きフットプリント（実車体矩形）の包含判定をハード制約に。狭窄エリアで「切り返し点を
    #     含む全姿勢で車体がエリア外に出ない」ことを保証する。vehicle と領域が揃ったときのみ有効。---
    fp_samp = fp_inv = fp_search = None
    fp_ok = None
    if vehicle is not None and drivable_mask is not None and transform is not None:
        poly = vehicle_footprint(vehicle)
        fp_samp = footprint_sample_points(poly, max(abs(transform.a), 0.5))
        fp_inv = _inv_affine(transform)
        # hybrid A* の探索用フットプリント。粗すぎる（車体寸/6≈1.9m）と縁の細いはみ出しを
        # 取り零して ev()（セル精度）と食い違うため、車体寸/12 か セル の粗い方を使う。
        # 最終判定は fp_samp（セル精度）で ev() が再検査する。
        fp_search = footprint_sample_points(
            poly, max(abs(transform.a), max(vehicle.overall_width, vehicle.overall_length) / 12.0)
        )

        def fp_ok(x, y, yaw):  # noqa: ANN001
            return footprint_clear(fp_samp, x, y, yaw, drivable_mask, fp_inv)

    # フットプリント判定姿勢の弧長間隔。経路ステップ(0.3m)の全点判定は 10m級車体では過剰で、
    # 候補評価コストの大半を占めていた。0.6m 間引きで実質同じ判定（端点は常に含む）。
    scan_ds = max(step, 0.6)

    def ev(segments):
        return _build(segments, speed_fwd, speed_rev, transform, drivable_mask, dt, cost, obstacle_value,
                      cmax, footprint_radius, tx, ty, fp_samp=fp_samp, fp_inv=fp_inv,
                      ignore_ends_m=footprint_ignore_ends_m, tyaw=tyaw, scan_ds=scan_ds)

    cands: list[SpottingResult] = []
    allow_switch = (max_switchbacks is None) or (max_switchbacks >= 1)
    add_forward = (not require_switchback) or (not allow_switch)
    # アルゴリズム選択: どの候補生成器を使うか。auto は全部。
    method = (method or "auto").lower()
    use_dubins = method in ("auto", "dubins")
    use_rs = method in ("auto", "reeds_shepp")
    use_hybrid = method in ("auto", "hybrid_astar")

    # --- 手動切り返し点モード: 指定姿勢 S を固定し、前進(start→S)＋後進(S→target) のみ生成 ---
    if manual_switch_pose is not None:
        Sx, Sy, Syaw = float(manual_switch_pose[0]), float(manual_switch_pose[1]), float(manual_switch_pose[2])
        segs = _stage_segments(start, (Sx, Sy), Syaw, target, rho, step, margin, fp_ok,
                               start_lead=end_lead, goal_lead=end_lead)  # 切り返し点に直線マージン
        if segs is None:
            return SpottingResult(status="NO_PATH")
        res = ev(segs)
        res.score = _score(res, w)
        if not res.feasible:
            res.status = "INFEASIBLE_BEST_EFFORT"
        return res

    # --- 前進候補（0切り返し）: 直進＋横オフセット中点経由でカーブ選択肢も用意 ---
    chord = math.atan2(ty - sy, tx - sx)
    px, py = _perp(chord)
    midx, midy = (sx + tx) / 2, (sy + ty) / 2
    if add_forward and use_dubins:
        for lat in (0.0, rho, -rho, 2 * rho, -2 * rho):
            if lat == 0.0:
                path = sample_dubins(start, target, rho, step)
            else:
                mid = [midx + lat * px, midy + lat * py]
                path = plan_dubins(
                    [[sx, sy], mid, [tx, ty]], rho=rho, step=step,
                    headings_deg=[math.degrees(syaw), None, math.degrees(tyaw)],
                )
            if path is not None and len(path) > 1:
                cands.append(ev([(path, "F")]))

    # --- 1切り返し候補: ステージ姿勢S→後進で差し込み ---
    # 直線一辺倒（ドック正面で直進バック）を避けるため、進入方位 dh を 45/90度まで広げ、
    # さらにターゲット周囲のリング上（側方/斜め進入）の S も生成する。reverse_dubins が
    # 「曲がりながらの後進」になり、45/90度で差し込む候補が出る。各候補は ev() で衝突・
    # コスト・寄り付き誤差を評価し、最良がコスト関数で選ばれる。
    # 候補が「前進のみ/切り返し」の許可と整合するか（モードに合わない候補を弾く）。
    def _keep(c: SpottingResult) -> bool:
        if not allow_switch and c.n_switchbacks >= 1:
            return False  # 前進のみモードで切り返し候補は不可
        return add_forward or c.n_switchbacks >= 1  # 切り返し必須なら前進のみ候補を除外

    if allow_switch and use_dubins:
        tpx, tpy = _perp(tyaw)
        # ステージ姿勢 S を収集（fp_ok で領域外は事前に間引く＝1姿勢チェックは ev より格段に安い）。
        # (1) ドック前方の格子。**切り返し角度の多様性**を最大化するため進入方位 dh を ±12度刻みで
        #     ±156度まで、距離・横オフセットも密に刻む（切り返し後の進入角を広く探索）。
        # (2) ターゲット周囲リング: 方位角 bearing を20度刻みで一周、側方・斜め・背後からの差し込み。
        raw_poses: list[tuple] = []

        def _add_pose(Sx, Sy, Syaw):
            raw_poses.append((Sx, Sy, Syaw))

        dhs = tuple(math.radians(a) for a in range(-156, 157, 12))
        for d in np.linspace(max(0.4 * rho, 1.5), max(5.0 * rho, 16.0), 11):
            for lat in (0.0, 0.5 * rho, -0.5 * rho, 1.0 * rho, -1.0 * rho, 1.5 * rho, -1.5 * rho):
                for dh in dhs:
                    _add_pose(tx + d * math.cos(tyaw) + lat * tpx,
                              ty + d * math.sin(tyaw) + lat * tpy, tyaw + dh)
        # (1b) 遠方・横に離れた**粗いシェル**: 密格子（横±1.5ρ・距離5ρまで）の外側、
        #     横±2〜3ρ × 距離最大7ρ を方位24°刻みで覆う。近場が塞がったエリアや
        #     「少し離れた広場で切り返して戻る」型の解を候補に乗せる（fp_ok 事前間引きで
        #     エリア外は安価に落ちるため、広げても実評価数は増えにくい）。
        dhs_coarse = tuple(math.radians(a) for a in range(-144, 145, 24))
        for d in np.linspace(max(0.8 * rho, 3.0), max(7.0 * rho, 24.0), 6):
            for lat in (2.0 * rho, -2.0 * rho, 2.5 * rho, -2.5 * rho, 3.0 * rho, -3.0 * rho):
                for dh in dhs_coarse:
                    _add_pose(tx + d * math.cos(tyaw) + lat * tpx,
                              ty + d * math.sin(tyaw) + lat * tpy, tyaw + dh)
        for d in (max(1.2 * rho, 5.0), max(2.0 * rho, 8.0), max(3.0 * rho, 12.0), max(4.0 * rho, 16.0),
                  max(5.5 * rho, 20.0)):
            for bearing in (math.radians(a) for a in range(-160, 161, 20) if a != 0):
                bdir = tyaw + bearing
                for hfac in (0.3, 0.6, 1.0):
                    _add_pose(tx + d * math.cos(bdir), ty + d * math.sin(bdir), tyaw + hfac * bearing)

        # ステージ姿勢の事前フィルタ（車体がエリア内に置ける S だけ残す）。
        # 1姿勢ずつ footprint_clear を呼ぶと ~2,800回の小配列呼び出しになるため、
        # 全姿勢×全サンプル点を (P,M) にまとめて1回で判定する（_footprint_scan と同じ構造）。
        if fp_ok is not None and raw_poses:
            pa = np.asarray(raw_poses, float)
            c_ = np.cos(pa[:, 2])
            s_ = np.sin(pa[:, 2])
            wx = pa[:, 0][:, None] + np.outer(c_, fp_samp[:, 0]) - np.outer(s_, fp_samp[:, 1])
            wy = pa[:, 1][:, None] + np.outer(s_, fp_samp[:, 0]) + np.outer(c_, fp_samp[:, 1])
            ia, ib, ic_, id_, ie, if_ = fp_inv
            cols = np.floor(ia * wx + ib * wy + ic_).astype(np.intp)
            rows = np.floor(id_ * wx + ie * wy + if_).astype(np.intp)
            h_, w_ = drivable_mask.shape
            oob = (rows < 0) | (rows >= h_) | (cols < 0) | (cols >= w_)
            bad = oob | (~oob & (drivable_mask[np.clip(rows, 0, h_ - 1), np.clip(cols, 0, w_ - 1)] == 0))
            ok_mask = ~bad.any(axis=1)
            poses = [p for p, ok in zip(raw_poses, ok_mask) if ok]
        else:
            poses = raw_poses

        def _eval_stage(p):
            segs = _stage_segments(start, (p[0], p[1]), p[2], target, rho, step, margin, fp_ok,
                                   start_lead=end_lead, goal_lead=end_lead)
            return ev(segs) if segs is not None else None

        # --- 二段階評価: ~2,800姿勢を全部フル解像度で評価すると ev が支配的（数秒）になる。
        #     一次: 粗ステップ(2×step)＋粗スキャン間隔で全姿勢を採点 → best-effort と同じ
        #     優先キー（成立性 > はみ出し率 > スコア）で上位 K 姿勢だけをフル解像度で再評価。
        #     勝者の経路そのものはフル解像度で構築されるため出力品質は不変。切り返しゾーン
        #     指定時はゾーン内候補が長距離で不利になりがちなので、ゾーン内上位も別枠で残す。---
        step_c = step * 2.0

        def _eval_stage_coarse(p):
            segs = _stage_segments(start, (p[0], p[1]), p[2], target, rho, step_c, margin, fp_ok,
                                   start_lead=end_lead, goal_lead=end_lead)
            if segs is None:
                return None
            c = _build(segs, speed_fwd, speed_rev, transform, drivable_mask, dt, cost, obstacle_value,
                       cmax, footprint_radius, tx, ty, fp_samp=fp_samp, fp_inv=fp_inv,
                       ignore_ends_m=footprint_ignore_ends_m, tyaw=tyaw, scan_ds=scan_ds * 4.0,
                       light=True)
            c.score = _score(c, w)
            return c

        TOPK = 24
        coarse = [(c, p) for c, p in zip(_pmap(_eval_stage_coarse, poses, n_workers), poses)
                  if c is not None and _keep(c)]
        key = lambda cp: (not cp[0].feasible, round(cp[0].fp_max_frac, 4), cp[0].score)  # noqa: E731
        coarse.sort(key=key)
        selected = coarse[:TOPK]
        if switchback_zone is not None:
            def _in_zone(c):
                return c.switch_points and all(_point_in_poly(sx_, sy_, switchback_zone)
                                               for (sx_, sy_) in c.switch_points)
            zoned_c = [cp for cp in coarse if _in_zone(cp[0])]
            seen_p = {id(p) for _c, p in selected}
            selected += [cp for cp in zoned_c[:TOPK] if id(cp[1]) not in seen_p]
        # 切り返し点 S に直線マージンを入れて追従可能に（S 前後でステア0°）。**並列評価**。
        for c in _pmap(_eval_stage, [p for _c, p in selected], n_workers):
            if c is not None and _keep(c):
                cands.append(c)

    # --- Reeds-Shepp 全体接続候補（可変半径の近似）: start→target を1本のRSで結ぶ。
    #     全48語×複数半径ρを列挙し、各候補を ev() で衝突・コスト評価して最良を競わせる。固定最小半径
    #     では急ターンしか出ないが、半径を変えて試すことで「余裕がある所は緩く曲げ、cusp を減らす」
    #     可変半径RSの効果を近似する（狭窄では小半径、広い所では大半径が選ばれる）。---
    if use_rs:
        rs_inputs = []
        for rho_mult in (1.0, 1.3, 1.7, 2.2, 2.8):
            rr = rho * rho_mult
            for rspath in reeds_shepp_paths(start, target, rr, step)[:10]:  # 各半径で短い順 上位10本
                segs = _segments_from_hybrid(rspath)  # (x,y,yaw,gear) を gear連続区間に分割
                if segs:
                    rs_inputs.append(segs)

        def _eval_rs(segs):
            return ev(_apply_cusp_margins(segs, margin, step, fp_ok))  # 各cuspに直線マージン(適応)

        for c in _pmap(_eval_rs, rs_inputs, n_workers):
            if c is not None and _keep(c):
                cands.append(c)

    # --- cost-aware 候補: hybrid A*（コストに沿って滑らかに曲がる/後進可）。コストマップ or 領域がある時のみ ---
    if use_hybrid and (cost is not None or drivable_mask is not None):
        hc = _hybrid_candidate(start, target, rho, cost, drivable_mask, transform, obstacle_value, allow_switch, ev,
                               margin=margin, step=step, fp_ok=fp_ok, footprint=fp_search)
        if hc is not None and _keep(hc):
            cands.append(hc)
        if end_lead > 0:
            # 据え切り禁止: 端点直線リードを織り込んだ hybrid 変種（start′/goal′ から計画し、
            # 固定姿勢までの直線を接合）。狭所の K ターンはスプライス後付けでは端点を直線化
            # できないため、探索自体をリード込みで行う。ギア組合せ4変種を試す。
            sdx, sdy = math.cos(syaw), math.sin(syaw)
            tdx, tdy = math.cos(tyaw), math.sin(tyaw)
            for s_sign, s_gear in ((1.0, "F"), (-1.0, "R")):
                st2 = (sx + s_sign * end_lead * sdx, sy + s_sign * end_lead * sdy, syaw)
                for t_sign, t_gear in ((1.0, "R"), (-1.0, "F")):
                    tg2 = (tx + t_sign * end_lead * tdx, ty + t_sign * end_lead * tdy, tyaw)
                    hv = _hybrid_candidate(
                        st2, tg2, rho, cost, drivable_mask, transform, obstacle_value, allow_switch, ev,
                        margin=margin, step=step, fp_ok=fp_ok, footprint=fp_search,
                        wrap=((start, s_gear, end_lead), (target, t_gear, end_lead)),
                    )
                    if hv is not None and _keep(hv):
                        cands.append(hv)

    # 候補が1つも生成できなかった（例: method="hybrid_astar" なのに drivable/cost が無い、
    # 選択手法が指定姿勢に解を持たない等）。zone 制約由来ではないので NO_PATH を返す。
    if not cands:
        if method == "hybrid_astar" and cost is None and drivable_mask is None:
            st = "NO_HYBRID_MAP"
        elif method != "auto":
            st = "METHOD_NO_PATH"   # 選択手法が条件（切り返し必須等）を満たす解を出せなかった
        else:
            st = "NO_PATH"
        return SpottingResult(status=st)

    # --- 切り返し可能エリア制約: cusp（切り返し点）が全て zone 内にある候補だけ採用 ---
    #     前進のみ候補（switch_points 無し）は制約対象外。zone で全滅したら制約は無視せず
    #     「前進のみ候補」へフォールバック（切り返しできる場所が無い＝前進で寄るしかない）。
    if switchback_zone and len(switchback_zone) >= 3:
        def _switch_in_zone(c: SpottingResult) -> bool:
            return all(_point_in_poly(sx_, sy_, switchback_zone) for (sx_, sy_) in c.switch_points)
        zoned = [c for c in cands if _switch_in_zone(c)]
        cands = zoned if zoned else [c for c in cands if not c.switch_points]

    if not cands:
        return SpottingResult(status="NO_ZONE_PATH")

    for c in cands:
        c.score = _score(c, w)

    feas = [c for c in cands if c.feasible]
    if feas:
        # エリア内に完全に収まる候補があるなら、その中でコスト最小を選ぶ（はみ出し品は採らない）。
        best = min(feas, key=lambda c: c.score)
    else:
        # 収まる候補が無い best-effort: **はみ出し率を最優先で最小化**（同率ならコスト最小）。
        # こうしないと「短いが大きくはみ出す経路」が「長いが僅かにはみ出す経路」に勝ってしまう。
        best = min(cands, key=lambda c: (round(c.fp_max_frac, 4), c.score))
        best.status = "INFEASIBLE_BEST_EFFORT" if best.status not in ("FOOTPRINT_OUTSIDE",) else best.status

    # フットプリント/領域の包含判定（据え切りマージン挿入・平滑化で共用）。
    def _accept(x, y, yaw):
        if fp_ok is not None:
            return fp_ok(x, y, yaw)
        if drivable_mask is not None and transform is not None:
            r, c = _world_to_rc(transform, x, y)
            return 0 <= r < drivable_mask.shape[0] and 0 <= c < drivable_mask.shape[1] and drivable_mask[r, c] > 0
        return True

    # --- 据え切り禁止: 始端(start)・終端(goal)に直線マージンを挿入し端点曲率を0にする。
    #     停止点でステア0°→漸増/漸減。**平滑化の有無に依らず**適用（据え切り回避は要件）。
    #     ステージ候補は生成時からリード織り込み済み（既直線として検出）。RS/hybrid 等は後付け挿入。
    #     選ばれた best が直線化できない場合は、**直線化できる次善候補へフォールバック**する
    #     （旧: best のみ試し、失敗すると許可時と同一の経路が黙って返っていた）。
    #     挿入できた端はその長さ lead を記録し、後段の平滑化でロックして直線を保つ。---
    lead0 = leadN = 0.0
    if not allow_stationary and best.points:
        best, lead0, leadN = resolve_no_stationary(
            best, cands, start=start, target=target, margin=margin, rho=rho, step=step,
            accept=_accept, ev=ev, score=lambda c: _score(c, w),
        )

    # --- 平滑化ポストプロセス: 選択経路の曲率不連続(dκ/ds スパイク=速度低下)と蛇行を抑える。
    #     gear区間ごとに端点固定で平滑化。エリア外へ出る移動は拒否。悪化したら採用しない＝安全側。---
    if smooth_path and best.points and len(best.points) >= 5 and smooth_iters > 0:
        segs = _segments_from_hybrid(_states_to_tuples(best.points))
        if segs:
            # cusp 端は margin ロックでステア0°の前後反転を保持。path端は据え切り禁止で入れた lead をロック。
            kseg = len(segs)
            sm = []
            for k, (p, g) in enumerate(segs):
                ls = margin if k > 0 else lead0
                le = margin if k < kseg - 1 else leadN
                sm.append((_smooth_segment(p, _accept, rho, step, lock_start_m=ls, lock_end_m=le), g))
            cand = ev(sm)
            if cand.points:
                cand.score = _score(cand, w)
                ok = (cand.fp_max_frac <= best.fp_max_frac + 1e-6
                      and cand.approach_error_m <= best.approach_error_m + 0.05
                      and (best.feasible <= cand.feasible or not best.feasible))
                if ok:
                    cand.status = best.status if not cand.feasible else "OK"
                    best = cand
    # 診断: 解決後の据え切り可否と、実際に入れた端点直線長を結果に記録（UI 表示・切り分け用）。
    best.allow_stationary = bool(allow_stationary)
    best.endpoint_margin_start_m = round(float(lead0), 2)
    best.endpoint_margin_goal_m = round(float(leadN), 2)
    return best
