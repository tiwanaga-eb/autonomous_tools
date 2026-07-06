"""複数経路の重なり（競合）判定（複数台制御 Phase B）。

各経路を「中心線 ± 車幅/2 のコリドー」とみなし、2経路のコリドーが重なる区間を抽出する。
重なり区間は後段の区間Mutex（予約）対象になる。

実装は shapely 非依存: 共通グリッド上で各経路の中心線をラスタ化し、距離変換で「中心線までの距離場
[m]」を作る。コリドー重なり = (距離A<=半幅A) かつ (距離B<=半幅B)。各経路上の競合**弧長範囲**は、
その経路の点で「相手中心線までの距離 <= 半幅A+半幅B」になる連続区間として求める（セル→弧長の逆引き
不要で安定）。

座標は作業CRS(メートル)。経路 points は (N,2)[m]。
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np


@dataclass
class ConflictInterval:
    """競合する弧長区間（片方の経路上）。"""
    s_start: float
    s_end: float


@dataclass
class RouteConflict:
    """経路ペア (a,b) の競合。a_intervals/b_intervals は各経路上の競合弧長区間群。"""
    a: int
    b: int
    a_intervals: list[ConflictInterval] = field(default_factory=list)
    b_intervals: list[ConflictInterval] = field(default_factory=list)
    overlap_area_m2: float = 0.0
    kind: str = "crossing"            # "crossing"(交差) / "shared"(区間共有・併走/対向)
    bbox: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 0.0)  # (minx,miny,maxx,maxy)


def _cumulative_s(pts: np.ndarray) -> np.ndarray:
    if len(pts) < 2:
        return np.zeros(len(pts))
    d = np.hypot(np.diff(pts[:, 0]), np.diff(pts[:, 1]))
    return np.concatenate([[0.0], np.cumsum(d)])


def _common_grid(all_pts: list[np.ndarray], margin: float, cell: float):
    """全経路を覆う共通ラスタ (transform, shape) を作る（north-up）。"""
    from rasterio.transform import from_origin

    xs = np.concatenate([p[:, 0] for p in all_pts])
    ys = np.concatenate([p[:, 1] for p in all_pts])
    minx, maxx = float(xs.min()) - margin, float(xs.max()) + margin
    miny, maxy = float(ys.min()) - margin, float(ys.max()) + margin
    w = max(2, int(np.ceil((maxx - minx) / cell)))
    h = max(2, int(np.ceil((maxy - miny) / cell)))
    return from_origin(minx, maxy, cell, cell), (h, w)


def _line_distance(pts: np.ndarray, transform, shape) -> np.ndarray:
    """中心線(折れ線)をラスタ化し、各セルから中心線までの距離場 [m] を返す。"""
    import rasterio.features as rfeat
    from scipy.ndimage import distance_transform_edt

    cell = float(abs(transform.a))
    if len(pts) < 2:
        line = {"type": "Point", "coordinates": [float(pts[0, 0]), float(pts[0, 1])]} if len(pts) else None
    else:
        line = {"type": "LineString", "coordinates": [[float(x), float(y)] for x, y in pts]}
    burn = np.zeros(shape, dtype="uint8")
    if line is not None:
        burn = rfeat.rasterize([(line, 1)], out_shape=shape, transform=transform,
                               fill=0, all_touched=True, dtype="uint8")
    if not burn.any():
        return np.full(shape, np.inf)
    return distance_transform_edt(burn == 0) * cell


def _rc(transform, xs, ys):
    inv = ~transform
    ia, ib, ic, id_, ie, if_ = inv.a, inv.b, inv.c, inv.d, inv.e, inv.f
    cols = np.floor(ia * xs + ib * ys + ic).astype(np.intp)
    rows = np.floor(id_ * xs + ie * ys + if_).astype(np.intp)
    return rows, cols


def _labels_near_points(labels, transform, pts, dt_other, thr, hw_self, cell):
    """各点を「最寄りの重なりセルの連結成分ラベル」へ割当てる（in-conflict 点のみ、それ以外 0）。

    in-conflict = 相手中心線まで dt_other<=thr。点は自分の半幅 hw_self 内に重なりセルがあるので、
    点セルの近傍(半径 ~hw_self/cell)で最初に見つかる非ゼロラベルを採用する。
    """
    rows, cols = _rc(transform, pts[:, 0], pts[:, 1])
    h, w = labels.shape
    out = np.zeros(len(pts), dtype=int)
    if not np.any(labels):
        return out
    from scipy.ndimage import distance_transform_edt

    # 全セル→最寄りの非ゼロラベルセル（EDT の return_indices）を1回だけ前計算し、
    # 点ごとの (2rad+1)² 近傍走査（Python 二重ループ）を O(1) のテーブル参照に置き換える。
    dist, (ir, ic) = distance_transform_edt(labels == 0, return_indices=True)
    rad = float(np.ceil(hw_self / cell)) + 2.0
    rr = np.clip(rows, 0, h - 1)
    cc = np.clip(cols, 0, w - 1)
    in_bounds = (rows >= 0) & (rows < h) & (cols >= 0) & (cols < w)
    in_conflict = in_bounds & (dt_other[rr, cc] <= thr) & (dist[rr, cc] <= rad)
    nearest_label = labels[ir[rr, cc], ic[rr, cc]]
    out[in_conflict] = nearest_label[in_conflict]
    return out


def _interval_for_label(comp_of, s_arr, lab):
    """ラベル lab に割当てられた点の弧長範囲を1区間で返す（連続性は近似、min/max）。無ければ None。"""
    idx = np.where(comp_of == lab)[0]
    if len(idx) == 0:
        return None
    return ConflictInterval(float(s_arr[idx.min()]), float(s_arr[idx.max()]))


def detect_conflicts(routes: list[dict], *, cell: float = 0.5, clearance_m: float = 0.0) -> list[RouteConflict]:
    """複数経路の総当たりでコリドー重なりを検出する。**連結した重なり領域ごと**に別 RouteConflict を返す
    （交差2点や待避所で割れた領域を別ゾーンに分離＝区間Mutexが正しく機能する）。

    routes: [{"points": (N,2)[m], "half_width": float[m]}, ...]。half_width = 車幅/2(+任意マージン)。
    clearance_m: 競合とみなす追加余裕[m]（半幅に上乗せ。車車間の安全側）。
    """
    from scipy.ndimage import label as _cc_label

    pts = [np.asarray(r["points"], float) for r in routes]
    hw = [float(r.get("half_width", 1.0)) + float(clearance_m) for r in routes]
    valid = [i for i in range(len(pts)) if len(pts[i]) >= 1]
    if len(valid) < 2:
        return []
    margin = max(hw) + 2.0 * cell
    transform, shape = _common_grid([pts[i] for i in valid], margin, cell)
    dt = {i: _line_distance(pts[i], transform, shape) for i in valid}
    s = {i: _cumulative_s(pts[i]) for i in valid}

    conflicts: list[RouteConflict] = []
    for ai in range(len(valid)):
        for bi in range(ai + 1, len(valid)):
            a, b = valid[ai], valid[bi]
            ovl = (dt[a] <= hw[a]) & (dt[b] <= hw[b])
            if not ovl.any():
                continue
            thr = hw[a] + hw[b]
            labels, ncomp = _cc_label(ovl)  # 連結成分（=独立した重なり領域）
            comp_a = _labels_near_points(labels, transform, pts[a], dt[b], thr, hw[a], cell)
            comp_b = _labels_near_points(labels, transform, pts[b], dt[a], thr, hw[b], cell)
            for lab in range(1, ncomp + 1):
                a_iv = _interval_for_label(comp_a, s[a], lab)
                b_iv = _interval_for_label(comp_b, s[b], lab)
                if a_iv is None and b_iv is None:
                    continue
                comp_mask = labels == lab
                area = float(comp_mask.sum()) * cell * cell
                ys, xs = np.where(comp_mask)
                wx = transform.c + (xs + 0.5) * transform.a
                wy = transform.f + (ys + 0.5) * transform.e
                bbox = (float(wx.min()), float(wy.min()), float(wx.max()), float(wy.max()))
                a_len = (a_iv.s_end - a_iv.s_start) if a_iv else 0.0
                b_len = (b_iv.s_end - b_iv.s_start) if b_iv else 0.0
                kind = "shared" if min(a_len, b_len) > 3.0 * thr else "crossing"
                conflicts.append(RouteConflict(
                    a=a, b=b, a_intervals=[a_iv] if a_iv else [], b_intervals=[b_iv] if b_iv else [],
                    overlap_area_m2=area, kind=kind, bbox=bbox))
    return conflicts

