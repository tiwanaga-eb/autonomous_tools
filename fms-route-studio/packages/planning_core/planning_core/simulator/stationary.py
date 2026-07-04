"""据え切り回避（endpoint stationary-steer avoidance）。

寄り付き経路の端点（start/goal の停止姿勢）は、端点で曲率≠0 だと停止中の
その場操舵（据え切り）が必要になる。本モジュールは
  1) 経路端に固定姿勢へ接する直線リードを挿入して端点曲率を0にする
     （insert_endpoint_margin: Dubins 再接続・エリア逸脱ガード・ループ抑制つき）
  2) best が直線化できないとき、次善候補群から直線化できるものへ
     フォールバックして選び直す（resolve_no_stationary）
を提供する。spotting.py 本体から分離（候補生成・評価とは独立した後処理）。

併せて、両者が使う小さな経路ユーティリティ（headings/straight_pts/
segments_from_hybrid/states_to_tuples）もここに置く（spotting からも共用）。
"""
from __future__ import annotations

import math

import numpy as np

from ..planners.dubins import reverse_dubins, sample_dubins


def headings(pts: np.ndarray, gear: str) -> np.ndarray:
    """点列の車体方位[rad]（gear="R" は接線+π＝車体は進行方向の逆向き）。"""
    n = len(pts)
    h = np.zeros(n)
    if n < 2:
        return h
    d = np.diff(pts, axis=0)
    ang = np.arctan2(d[:, 1], d[:, 0])
    h[:-1] = ang
    h[-1] = ang[-1]
    if gear == "R":
        # 後進は車体が進行方向の逆向き → 接線に +π して車体方位へ。
        # arctan2 は (-π,π] なので (h + 2π) % 2π - π = wrap(h+π) が +π ラップになる。
        h = (h + 2 * math.pi) % (2 * math.pi) - math.pi
    return h


def straight_pts(a, b, step: float):
    """a→b の直線を step 間隔でサンプル（両端含む）。返値 [(x,y), ...]。"""
    d = math.hypot(b[0] - a[0], b[1] - a[1])
    n = max(1, int(round(d / max(step, 1e-6))))
    return [(a[0] + (b[0] - a[0]) * i / n, a[1] + (b[1] - a[1]) * i / n) for i in range(n + 1)]


def segments_from_hybrid(pts):
    """hybrid_astar の (x,y,yaw,gear) 列を gear 連続区間ごとの [(np点列, gear), ...] に分割。"""
    n = len(pts)
    segs = []
    i = 0
    while i < n - 1:
        g = pts[i + 1][3]  # i→i+1 の移動 gear
        j = i + 1
        while j < n - 1 and pts[j + 1][3] == g:
            j += 1
        segs.append((np.array([(pts[k][0], pts[k][1]) for k in range(i, j + 1)], float), g))
        i = j
    return segs


def states_to_tuples(points):
    """SpottingResult.points(dict列) を (x,y,yaw[rad],gear) タプル列へ（segment分割用）。"""
    return [(p["x"], p["y"], math.radians(p["heading_deg"]), p["gear"]) for p in points]


def insert_endpoint_margin(pts, gear, fixed_pose, e, rho, step, fp_ok, *, at_start):
    """経路端（停止する固定姿勢 start/target）に直線マージンを挿入し端点曲率を0にする（据え切り回避）。

    固定姿勢に接する直線 e[m] を作り、その先で元経路へ Dubins 再接続する。固定姿勢の位置・方位は
    厳密保持。区間が短ければ e を自動短縮（後進差し込みは区間が短くなりがち）。エリア(fp_ok)を
    逸脱する場合は挿入せず元の点列を返す（狭隘地での best-effort 後退）。
    返値 (新点列, 実際に入れた直線長[m])。0.0 は挿入なし。at_start=False は末尾固定(goal側)。
    """
    pts = np.asarray(pts, float)
    if e <= 0 or len(pts) < 4:
        return pts, 0.0
    rev = not at_start
    work = pts[::-1].copy() if rev else pts             # work[0] を固定端へ統一
    g = ("R" if gear == "F" else "F") if rev else gear  # 反転で進行方向が反転
    yaw = float(fixed_pose[2])
    seglen = np.hypot(*np.diff(work, axis=0).T)
    s = np.concatenate([[0.0], np.cumsum(seglen)])
    total = float(s[-1])
    hd = headings(work, g)

    travel = 1.0 if g == "F" else -1.0
    c, sn = math.cos(yaw), math.sin(yaw)
    p0 = (float(work[0, 0]), float(work[0, 1]))

    # 既に端点が直線（生成時にリードを織り込んだ候補など）なら挿入不要。**幾何判定**:
    # 固定姿勢の方位射線からの横偏差 ≤5cm ＋ 射線方向に単調前進、が続く長さを直線 run とする
    # （方位差だけの判定は緩いカーブ κ~0.06 でも数点は通ってしまい、誤って「直線」扱いになる）。
    dxs = work[:, 0] - p0[0]
    dys = work[:, 1] - p0[1]
    t_along = travel * (dxs * c + dys * sn)
    d_perp = np.abs(-sn * dxs + c * dys)
    run = 0.0
    for i in range(1, len(work)):
        if d_perp[i] > 0.05 or t_along[i] < t_along[i - 1] - 1e-6:
            break
        run = float(t_along[i])
    if run >= min(e, 0.45 * total) - 1e-6 and run >= 0.8:
        return pts, float(min(run, e))

    def _try(ee: float, sQ: float):
        """直線 ee ＋ 弧長 sQ の点への Dubins 再接続を試す。不成立/ループ/エリア逸脱は None。

        sQ >= total は「セグメント終端（cusp/他端の固定姿勢）へ接続」＝セグメント全体を
        引き直す最終手段（短いセグメントでは途中の Q が取れないため）。終端姿勢は厳密保持。
        """
        if sQ >= total - max(step, 0.5):
            iQ = len(work) - 1
        else:
            iQ = max(2, min(len(work) - 2, int(np.searchsorted(s, sQ))))
        Q = work[iQ]
        hQ = float(hd[iQ])
        core = (p0[0] + travel * ee * c, p0[1] + travel * ee * sn)
        straight = straight_pts(p0, core, step)        # 固定端→core の直線（ここで端点曲率0）
        recon = (sample_dubins((core[0], core[1], yaw), (Q[0], Q[1], hQ), rho, step) if g == "F"
                 else reverse_dubins((core[0], core[1], yaw), (Q[0], Q[1], hQ), rho, step))
        if recon is None or len(recon) < 2:
            return None
        # ループ抑制: 再接続長が直線距離＋1周弱(1.2πρ)を超える構成（ρループの大回り）は不採用。
        rec = np.asarray(recon, float)[:, :2]
        rl = float(np.sum(np.hypot(*np.diff(rec, axis=0).T)))
        if rl > math.hypot(Q[0] - core[0], Q[1] - core[1]) + 1.2 * math.pi * rho:
            return None
        new = np.asarray([list(p) for p in straight] + [list(p) for p in recon[1:]]
                         + [list(x) for x in work[iQ + 1:]], float)
        if fp_ok is not None:
            hh = headings(new, g)
            for i in range(len(new)):
                if not fp_ok(float(new[i, 0]), float(new[i, 1]), float(hh[i])):
                    return None                        # エリア逸脱 → この構成は不採用
        return new

    # 反対端の直線リード（挿入済み/生成時織り込み）を測る。再接続の置換はそこを侵食しない。
    far = 0.0
    hN = float(hd[-1])
    for i in range(len(work) - 2, -1, -1):
        if abs((float(hd[i]) - hN + math.pi) % (2.0 * math.pi) - math.pi) > math.radians(2.0):
            break
        far = total - float(s[i])
    smax = total - (far if far >= 0.8 else 0.0) - 0.5

    # マージン長 ee（狭所では短縮）× 再接続点 sQ のラダー。近すぎる Q（core の旋回円内）は
    # Dubins が必然的にρループ（大回り）するため _try のループガードが弾く。円外に出る最初の
    # 経路点（ユークリッド判定）も候補に加える。セグメント全置換（sQ=total）は**反対端に保護
    # すべきリードが無いときだけ**許可（旧実装はこれで反対端の直線を消していた）。
    for fe in (1.0, 0.66, 0.4):
        ee = min(e * fe, 0.45 * total)
        if ee < 0.8:
            continue                                   # 区間が極端に短く直線化の意味がない
        core = (p0[0] + travel * ee * c, p0[1] + travel * ee * sn)
        d2core = np.hypot(work[:, 0] - core[0], work[:, 1] - core[1])
        sqs = [ee + 0.8 * rho, ee + 1.6 * rho, ee + 2.5 * rho, ee + 3.5 * rho]
        first_outside = next((float(s[i]) for i in range(2, len(work) - 1)
                              if s[i] > ee + 0.5 and d2core[i] >= 2.05 * rho), None)
        if first_outside is not None:
            sqs.append(first_outside)
        sqs = [q for q in sqs if q <= smax]
        if far < 0.8:
            sqs.append(total)                          # 全置換（反対端のリードを消さない場合のみ）
        for sQ in sqs:
            new = _try(ee, sQ)
            if new is not None:
                return (new[::-1].copy() if rev else new), ee
    if run >= 0.8:
        return pts, float(run)                         # 挿入は不可だが端点は部分的に直線（その長さを報告）
    return pts, 0.0                                    # 挿入不可（据え切りは残るが安全側）


def resolve_no_stationary(best, cands, *, start, target, margin, rho, step, accept, ev, score):
    """据え切り禁止の解決: best と次善候補の両端に直線リードを挿入し、最良を選び直す。

    選ばれた best が直線化できない場合は、直線化できる次善候補へフォールバックする
    （旧: best のみ試し、失敗すると許可時と同一の経路が黙って返っていた）。
    ランキングは スコア上位 8 件 ＋「既に端点が直線」上位 6 件（生成時リード織り込みの
    ステージ候補等はスコアだけだと土俵に乗らない）。優先キーは
    成立性（エリア包含=安全ゲート）＞ 直線化できた端数 ＞ はみ出し率 ＞ コスト。

    引数: accept(x,y,yaw)->bool（エリア判定）、ev(segments)->result（再評価）、
    score(result)->float（コスト再計算）。返値 (best, lead0, leadN)。
    """
    def _straighten(c):
        """候補 c の両端に直線リードを付与。返値 (segs or None, lead0, leadN, 幾何変更あり)。"""
        segs0 = segments_from_hybrid(states_to_tuples(c.points))
        if not segs0:
            return None, 0.0, 0.0, False
        p0n, l0 = insert_endpoint_margin(segs0[0][0], segs0[0][1], start, margin, rho, step, accept, at_start=True)
        ch0 = p0n is not segs0[0][0]
        if l0 > 0:
            segs0[0] = (p0n, segs0[0][1])
        pNn, lN = insert_endpoint_margin(segs0[-1][0], segs0[-1][1], target, margin, rho, step, accept, at_start=False)
        chN = pNn is not segs0[-1][0]
        if lN > 0:
            segs0[-1] = (pNn, segs0[-1][1])
        return segs0, l0, lN, (ch0 or chN)

    def _straight_run(c, from_end: bool) -> float:
        """端点からの直線長[m]（方位ドリフト2°以内）。挿入せず既存点列だけで測る（安価）。"""
        p = c.points
        if len(p) < 3:
            return 0.0
        idxs = range(len(p) - 1, -1, -1) if from_end else range(len(p))
        it = iter(idxs)
        i0 = next(it)
        h0 = p[i0]["heading_deg"]
        base = p[i0]["s"]
        run = 0.0
        for i in it:
            if abs((p[i]["heading_deg"] - h0 + 180.0) % 360.0 - 180.0) > 2.0:
                break
            run = abs(p[i]["s"] - base)
        return run

    def _ends_of(c) -> int:
        return int(_straight_run(c, False) >= 0.8) + int(_straight_run(c, True) >= 0.8)

    others = [c for c in cands if c.points and c is not best]
    by_score = sorted(others, key=lambda c: (not c.feasible, round(c.fp_max_frac, 4), c.score))[:8]
    by_ends = sorted(others, key=lambda c: (not c.feasible, -_ends_of(c), round(c.fp_max_frac, 4), c.score))[:6]
    seen: set = {id(best)}
    ranked = [best]
    for c in by_score + by_ends:
        if id(c) not in seen:
            seen.add(id(c))
            ranked.append(c)
    pick = None  # (key, cand, l0, lN)
    for c in ranked:
        segs0, l0, lN, changed = _straighten(c)
        if segs0 is None:
            continue
        if changed:
            cand = ev(segs0)
            if not cand.points:
                continue
            cand.score = score(cand)
        else:
            cand = c
        key = (not cand.feasible, -(int(l0 > 0) + int(lN > 0)), round(cand.fp_max_frac, 4), cand.score)
        if pick is None or key < pick[0]:
            pick = (key, cand, l0, lN)
        if l0 > 0 and lN > 0 and cand.feasible:
            break  # 両端直線化＋成立 → これ以上は探さない
    if pick is not None:
        return pick[1], pick[2], pick[3]
    return best, 0.0, 0.0
