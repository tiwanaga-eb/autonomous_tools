"""鉱山・土木っぽい合成シナリオ群とチューニング・レポート生成（朝レビュー用）。

実データに依らず、運搬路・スイッチバック・ヘアピン・狭ベンチ・行き止まり・断片化など
典型的な現場形状を mask で合成し、車両×アルゴリズム×設定でプランナーを走らせて
feasible / 実測R / 最小離隔 / 反復数 を表にする。`write_report()` が Markdown を吐く。

注: API ルーター(planning.py)の全パイプラインではなく、到達性に効く中核(hybrid_astar+footprint,
grid_astar)を直接呼ぶ軽量評価。傾向把握とパラメータ当たり付け用。
"""
from __future__ import annotations

import math

import numpy as np
from affine import Affine

from .analysis import min_turning_radius
from .footprint import footprint_sample_points, path_min_clearance, vehicle_footprint
from .planners import hybrid_astar, plan_grid_astar
from .vehicle import load_builtin

CELL = 0.5


def _grid(w_m: float, h_m: float):
    """幅 w_m × 高さ h_m [m] の空 mask と transform（cell=0.5, 左上原点, 北上）。"""
    nx = int(round(w_m / CELL))
    ny = int(round(h_m / CELL))
    transform = Affine(CELL, 0.0, 0.0, 0.0, -CELL, float(h_m))
    return np.zeros((ny, nx), np.uint8), transform


def _xy(mask, transform):
    ny, nx = mask.shape
    cols = np.arange(nx)
    rows = np.arange(ny)
    X = transform.c + (cols + 0.5) * transform.a
    Y = transform.f + (rows + 0.5) * transform.e
    return np.meshgrid(X, Y)  # (Xgrid, Ygrid) shape (ny,nx)


def _band_x(mask, transform, y0, y1, x0=None, x1=None):
    X, Y = _xy(mask, transform)
    sel = (Y >= y0) & (Y <= y1)
    if x0 is not None:
        sel &= X >= x0
    if x1 is not None:
        sel &= X <= x1
    mask[sel] = 1


def _band_y(mask, transform, x0, x1, y0=None, y1=None):
    X, Y = _xy(mask, transform)
    sel = (X >= x0) & (X <= x1)
    if y0 is not None:
        sel &= Y >= y0
    if y1 is not None:
        sel &= Y <= y1
    mask[sel] = 1


def build_scenarios() -> list[dict]:
    scns: list[dict] = []

    # 1) 直線運搬路（幅14m）。長さは車体前方張り出し(HM400=後輪軸中心から前8.32m)が
    #    ゴールで端を超えないよう70mに（基準点=後輪軸中心のため余裕が要る）。
    m, t = _grid(70, 40)
    _band_x(m, t, 13, 27)  # y∈[13,27] width14
    scns.append(dict(name="haul_straight", desc="直線運搬路 幅14m",
                     mask=m, transform=t, start=(4, 20, 0.0), goal=(56, 20, 0.0)))

    # 2) スイッチバック（Z字, 幅12m）
    m, t = _grid(60, 60)
    _band_x(m, t, 6, 18, x0=4, x1=56)      # 下段
    _band_x(m, t, 42, 54, x0=4, x1=56)     # 上段
    _band_y(m, t, 44, 56, y0=6, y1=54)     # 右の縦連結
    scns.append(dict(name="switchback", desc="スイッチバック Z字 幅12m",
                     mask=m, transform=t, start=(8, 12, 0.0), goal=(8, 48, 0.0)))

    # 3) ヘアピン（U字, 幅12m, 内径やや小）
    m, t = _grid(50, 60)
    _band_x(m, t, 6, 18, x0=4, x1=40)      # 下アーム
    _band_x(m, t, 42, 54, x0=4, x1=40)     # 上アーム
    _band_y(m, t, 30, 42, y0=6, y1=54)     # 右の折り返し
    scns.append(dict(name="hairpin", desc="ヘアピン U字 幅12m",
                     mask=m, transform=t, start=(8, 12, 0.0), goal=(8, 48, 0.0)))

    # 4) 狭ベンチ（幅6m 直線; HD785=5.5幅はタイト, HM400=3.45はOK）。長さ70m（前方張り出し対応）。
    m, t = _grid(70, 30)
    _band_x(m, t, 12, 18)  # width6
    scns.append(dict(name="narrow_bench", desc="狭ベンチ 幅6m（HD785はタイト）",
                     mask=m, transform=t, start=(4, 15, 0.0), goal=(56, 15, 0.0)))

    # 5) 行き止まりベイ（袋小路。前進で入って後進退出が要る形）
    m, t = _grid(50, 40)
    _band_x(m, t, 16, 28, x0=4, x1=46)     # 進入路
    _band_y(m, t, 34, 46, y0=4, y1=28)     # 下に伸びるポケット
    scns.append(dict(name="dead_end_bay", desc="行き止まりベイ（後進退出向き）",
                     mask=m, transform=t, start=(6, 22, 0.0), goal=(40, 10, -math.pi / 2)))

    # 6) 断片化（2コリドーが3mギャップで分離; close/橋渡しが要る）
    m, t = _grid(60, 30)
    _band_x(m, t, 12, 18, x0=4, x1=28)     # 左
    _band_x(m, t, 12, 18, x0=31, x1=56)    # 右（3mギャップ）
    scns.append(dict(name="fragmented_gap", desc="断片化 3mギャップ（橋渡し要）",
                     mask=m, transform=t, start=(6, 15, 0.0), goal=(54, 15, 0.0)))

    # 7) T字交差（幹線＋分岐へ進入）
    m, t = _grid(60, 50)
    _band_x(m, t, 22, 34, x0=4, x1=56)     # 幹線（横）幅12
    _band_y(m, t, 24, 36, y0=4, y1=34)     # 分岐（縦・下方向）幅12
    scns.append(dict(name="t_junction", desc="T字交差 幅12m（幹線→分岐進入）",
                     mask=m, transform=t, start=(6, 28, 0.0), goal=(30, 10, -math.pi / 2)))

    # 8) 緩やかな大カーブ（広い掘削面を回り込む。R大きめで余裕）
    m, t = _grid(70, 60)
    X, Y = _xy(m, t)
    cx, cy, r_in, r_out = 35.0, 10.0, 18.0, 32.0
    rr = np.hypot(X - cx, Y - cy)
    m[(rr >= r_in) & (rr <= r_out) & (Y >= cy)] = 1   # 上半分の弧コリドー
    scns.append(dict(name="wide_curve", desc="緩い大カーブ 幅14m（回り込み）",
                     mask=m, transform=t, start=(cx - 25, cy + 7, math.pi / 2),
                     goal=(cx + 25, cy + 7, -math.pi / 2)))

    return scns


def run_hybrid(scn: dict, vehicle_id: str, *, allow_reverse: bool, enforce_footprint: bool,
               xy_res: float = 1.0) -> dict:
    """1シナリオを hybrid_astar で評価。feasible/R/clearance/iters を返す。"""
    veh = load_builtin(vehicle_id)
    mask, t = scn["mask"], scn["transform"]
    rho = veh.min_turning_radius if (veh.kinematic_type != "tracked_skid" and veh.min_turning_radius) else 3.0
    fp = None
    if enforce_footprint:
        fp = footprint_sample_points(vehicle_footprint(veh), max(xy_res, max(veh.overall_width, veh.overall_length) / 6.0))
    res = hybrid_astar(scn["start"], scn["goal"], rho=rho, mask=mask, transform=t,
                       footprint=fp, allow_reverse=allow_reverse, xy_res=xy_res,
                       max_snap_m=max(3.0, 2 * xy_res), pos_tol=max(2.0 * xy_res, 2.0),
                       analytic_radius=max(4.0 * rho, 12.0), max_iters=120000)
    if res is None:
        return {"ok": False, "reason": "no path", "R": None, "clearance": None, "iters": None, "cusps": None}
    pts = res["points"]
    curve = np.array([[p[0], p[1]] for p in pts], float)
    # 実測R は cusp(ギア変化点)を跨ぐと3点外接円が~0に化けるため、ギア連続区間ごとに計測して最小を採る。
    R = float("inf")
    run0 = 0
    for i in range(1, len(pts) + 1):
        if i == len(pts) or pts[i][3] != pts[run0][3]:
            seg = curve[run0:i]
            if len(seg) >= 3:
                R = min(R, float(min_turning_radius(seg)))
            run0 = i
    clr, _ = path_min_clearance(curve, mask, t)
    return {"ok": True, "reason": "", "R": (round(R, 1) if math.isfinite(R) else None),
            "clearance": round(clr, 1), "iters": res["iters"], "cusps": res["n_cusps"],
            "len": round(res["length"], 1)}


def write_report(path: str) -> str:
    """全シナリオ×車両×設定を評価して Markdown レポートを書き出す。返値=本文。"""
    scns = build_scenarios()
    vehicles = ["HD785", "HM400", "CD110R"]
    lines = ["# シナリオ・チューニングレポート（自動生成）", ""]
    lines.append("> 鉱山/土木っぽい合成シナリオで hybrid A* を評価。footprint=実車体包含ON、")
    lines.append("> reverse=切り返し許可。`feasible/実測R[m]/最小離隔[m]/反復/切返` を表示。")
    lines.append("> 「狭くて車体が入らない」は物理的に不可（アルゴリズムのバグではない）。")
    lines.append("> 実測R は cusp(切返点)を跨がずギア連続区間ごとに計測した最小値（切返の見かけ上の小Rを除外）。")
    lines.append("")
    for scn in scns:
        lines.append(f"## {scn['name']} — {scn['desc']}")
        lines.append("")
        lines.append("| 車両 | 設定 | 可否 | 実測R | 最小離隔 | 反復 | 切返 | 備考 |")
        lines.append("|---|---|---|---|---|---|---|---|")
        for vid in vehicles:
            for label, rev, fp in [("基本", False, True), ("+reverse", True, True), ("footprintOFF", False, False)]:
                r = run_hybrid(scn, vid, allow_reverse=rev, enforce_footprint=fp)
                ok = "✅" if r["ok"] else "❌"
                lines.append(
                    f"| {vid} | {label} | {ok} | {r['R'] if r['R'] is not None else '—'} | "
                    f"{r['clearance'] if r['clearance'] is not None else '—'} | "
                    f"{r['iters'] if r['iters'] is not None else '—'} | "
                    f"{r['cusps'] if r.get('cusps') is not None else '—'} | {r['reason']} |"
                )
        lines.append("")
    text = "\n".join(lines)
    with open(path, "w", encoding="utf-8") as f:
        f.write(text)
    return text
