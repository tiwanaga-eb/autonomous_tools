"""Assemble an analyzed Trajectory from a polyline, and summarize it.

設計書 §5.1 共通後処理 / §10。API・各プランナーが共通利用する単一ソース。
- numeric 源（spline/grid_astar）の dκ/ds は量子化ノイズが乗るため Savitzky-Golay で平滑化（参考値）。
- 車両プロファイルがあれば feasibility と violation 区間（kinematic_type で分岐）を算出。
"""
from __future__ import annotations

import numpy as np

from ..models.analysis import AnalysisResult, Violation
from ..models.route import Trajectory, TrajPoint
from ..models.vehicle import VehicleProfile
from .curvature import curvature_profile, cusp_mask

try:  # SciPy があれば Savitzky-Golay を使う（無くても動く）
    from scipy.signal import savgol_filter

    _HAS_SCIPY = True
except Exception:  # pragma: no cover
    _HAS_SCIPY = False


def _headings_deg(pts: np.ndarray, gears=None) -> np.ndarray:
    """Per-point heading [deg], +East / CCW (設計書の角度規約).

    gears を渡すと **ギア連続区間ごとの片側差分** で接線を求め、後進(R)区間は車体方位
    （＝進行方向の逆）へ 180° 反転する。切り返し点(cusp)では直線マージンの折返しで前後の
    隣接点が重複し、中心差分が (0,0) に退化して見かけ上 0°(真東) を向いてしまう。区間境界を
    またがない片側差分なら、cusp 点でも正しい車体方位（進入方位）を保てる。
    gears=None のときは従来どおり中心差分（前進のみ経路向け、滑らかな接線）。
    """
    n = len(pts)
    h = np.zeros(n)
    if n < 2:
        return h
    if gears is None:
        fwd = np.diff(pts, axis=0)
        ang_fwd = np.degrees(np.arctan2(fwd[:, 1], fwd[:, 0]))
        h[0] = ang_fwd[0]
        h[-1] = ang_fwd[-1]
        if n > 2:
            cen = pts[2:] - pts[:-2]
            h[1:-1] = np.degrees(np.arctan2(cen[:, 1], cen[:, 0]))
        return h
    # ギア連続区間ごとに forward 差分で接線を求め、R 区間は 180° 反転（simulator._headings と一致）。
    glen = len(gears)
    i = 0
    while i < n:
        j = i
        while j + 1 < n and (j + 1 >= glen or gears[j + 1] == gears[i]):
            j += 1
        run = np.asarray(pts[i:j + 1], float)
        if len(run) >= 2:
            d = np.diff(run, axis=0)
            ang = np.degrees(np.arctan2(d[:, 1], d[:, 0]))
            h[i:j] = ang
            h[j] = ang[-1]
        elif i > 0:                      # 単点区間: 直前点の接線を流用（退化回避）
            h[i] = h[i - 1]
        if i < glen and gears[i] == "R":
            h[i:j + 1] = ((h[i:j + 1] + 360.0) % 360.0) - 180.0
        i = j + 1
    return h


def _robust_dkappa(kappa: np.ndarray, s: np.ndarray) -> np.ndarray:
    """numeric 源向け: κ を Savitzky-Golay で平滑化してから弧長微分（量子化ノイズ低減）。"""
    n = len(kappa)
    s_safe = s.copy()
    for i in range(1, n):
        if s_safe[i] <= s_safe[i - 1]:
            s_safe[i] = s_safe[i - 1] + 1e-9
    k = kappa
    if _HAS_SCIPY and n >= 7:
        win = min(n if n % 2 == 1 else n - 1, 11)
        if win >= 5:
            k = savgol_filter(kappa, win, 3)
    return np.gradient(k, s_safe)


def _cusp_with_gears(pts, gears=None) -> np.ndarray:
    """cusp(切り返し)点のbool mask。幾何検出(cusp_mask)に加え、gear が与えられれば
    gear 変化点とその前後1点も cusp とみなす（直線マージンの折返し頂点とその近傍で曲率が
    退化スパイクするため、解析から除外する範囲を確実に覆う）。"""
    pts = np.asarray(pts, float)
    cusp = cusp_mask(pts)
    n = len(pts)
    if gears is not None and n:
        gc = np.zeros(n, dtype=bool)
        for i in range(1, min(n, len(gears))):
            if gears[i] != gears[i - 1]:
                gc[i] = True
        cusp = cusp | gc | np.r_[gc[1:], [False]] | np.r_[[False], gc[:-1]]  # ±1 近傍も含める
    return cusp


def build_trajectory(
    pts,
    curvature_source: str = "numeric",
    vehicle: VehicleProfile | None = None,
    grade_pct: np.ndarray | None = None,
    gears: list | None = None,
    min_speed_mps: float = 0.0,
    z: np.ndarray | None = None,
) -> Trajectory:
    """Build a Trajectory (per-point s, heading, kappa, dkappa/ds, steer, grade) from a polyline.

    gears: 各点のギア("F"/"R")。寄り付き等の前後進つき経路で渡すと、速度プロファイルが
    切り返し(ギア変化)点で停止する。未指定なら全点同一ギア扱い。
    z: 各点の標高[m]（点群由来 DSM のサンプル値。elevation_and_grade 参照）。NaN は None になる。
    """
    pts = np.asarray(pts, float)
    prof = curvature_profile(pts)
    s, kappa = prof["s"], prof["kappa"]
    head = _headings_deg(pts, gears)

    wheel_base = None
    if vehicle is not None and vehicle.supports_steering_angle():
        wheel_base = vehicle.wheel_base

    grades = None
    if grade_pct is not None:
        grades = np.asarray(grade_pct, float)
    zs = None
    if z is not None:
        zs = np.asarray(z, float)

    # 切り返し点（gear対応）。直線マージンの折返し頂点で曲率が退化スパイクするため、cusp 点の
    # 曲率を 0 にしてから dκ/ds・速度・各指標を算出する（生スパイクの隣接波及・グラフ汚染を防ぐ）。
    cusp = _cusp_with_gears(pts, gears)
    kappa = np.asarray(kappa, float).copy()
    kappa[cusp] = 0.0
    dk = prof["dkappa_ds"] if curvature_source == "analytic" else _robust_dkappa(kappa, s)
    # 注: 後進(R)区間の 180° 反転（車体方位＝進行方向の逆）と cusp 退化対策は
    # _headings_deg(pts, gears) 内で区間ごとに処理済み。

    points: list[TrajPoint] = []
    for i in range(len(pts)):
        steer = float(np.degrees(np.arctan(wheel_base * kappa[i]))) if (wheel_base and not cusp[i]) else None
        g = None
        if grades is not None and i < len(grades) and np.isfinite(grades[i]):
            g = float(grades[i])
        zv = None
        if zs is not None and i < len(zs) and np.isfinite(zs[i]):
            zv = float(zs[i])
        points.append(
            TrajPoint(
                s=float(s[i]),
                x=float(pts[i, 0]),
                y=float(pts[i, 1]),
                z=zv,
                heading_deg=float(head[i]),
                curvature=float(kappa[i]),
                curvature_rate=float(dk[i]),
                grade_pct=g,
                steer_deg=steer,
                gear=(gears[i] if gears is not None and i < len(gears) else None),
            )
        )

    # 速度プロファイル v(s)・時間 t（車両があれば; Stage 6）
    if vehicle is not None and len(points) >= 2:
        from .velocity import velocity_profile

        gears = [p.gear for p in points]
        grade_arr = grades if (grades is not None and len(grades) == len(points)) else None
        v, t = velocity_profile(s, kappa, gears, vehicle, grades=grade_arr, min_speed_mps=min_speed_mps)
        for i, p in enumerate(points):
            p.speed_mps = float(v[i])
            p.time_s = float(t[i])

    kappa_eval = np.abs(kappa).copy()
    kappa_eval[cusp] = 0.0  # cusp は曲率評価から除外
    kmax = float(np.max(kappa_eval)) if len(kappa_eval) else 0.0
    min_r = (1.0 / kmax) if kmax > 1e-9 else None
    return Trajectory(
        points=points,
        length_m=float(s[-1]) if len(s) else 0.0,
        min_radius_m=min_r,
        curvature_source="analytic" if curvature_source == "analytic" else "numeric",
    )


def _segments_over(s: np.ndarray, values: np.ndarray, limit: float) -> list[tuple[float, float, float]]:
    """|values| > limit の連続区間を [(s_start, s_end, max_measured), ...] で返す。"""
    out: list[tuple[float, float, float]] = []
    over = np.abs(values) > limit
    i = 0
    n = len(values)
    while i < n:
        if over[i]:
            j = i
            peak = abs(values[i])
            while j + 1 < n and over[j + 1]:
                j += 1
                peak = max(peak, abs(values[j]))
            out.append((float(s[i]), float(s[j]), float(peak)))
            i = j + 1
        else:
            i += 1
    return out


def summarize(
    traj: Trajectory,
    vehicle: VehicleProfile | None = None,
    *,
    drivable_mask=None,
    transform=None,
    tol: float = 0.05,
) -> AnalysisResult:
    """軌跡 → AnalysisResult（車両制約に対する違反区間つき）。

    drivable_mask + transform + vehicle が揃えば、車体フットプリント（矩形/多角形）が
    走行可能領域に収まるかを厳密判定し、はみ出し区間を violation(kind='footprint') として加える。

    tol: 違反判定の相対許容率（最小旋回半径/操舵角/曲率変化率）。離散化・端点・R_min ちょうど
    で生成した経路が境界で誤って違反扱いになるのを防ぐ（既定5%）。表示する limit は素の値のまま。
    """
    s = np.array([p.s for p in traj.points])
    kappa = np.array([p.curvature for p in traj.points])
    dk = np.array([p.curvature_rate for p in traj.points])
    steer = np.array([p.steer_deg if p.steer_deg is not None else np.nan for p in traj.points])
    grade = np.array([p.grade_pct if p.grade_pct is not None else np.nan for p in traj.points])

    # 切り返し点(cusp)は曲率/曲率変化率の見かけ上スパイクを評価から除外（設計: R_min評価対象外）。
    # gear 変化点とその前後も含める（直線マージン折返し頂点の退化スパイク対策, build_trajectory と統一）。
    pts_xy = np.array([[p.x, p.y] for p in traj.points], float) if traj.points else np.zeros((0, 2))
    gears = [p.gear for p in traj.points]
    cusp = _cusp_with_gears(pts_xy, gears)
    kappa = np.where(cusp, 0.0, kappa)
    dk = np.where(cusp, 0.0, dk)

    max_k = float(np.max(np.abs(kappa))) if len(kappa) else 0.0
    max_dk = float(np.max(np.abs(dk))) if len(dk) else 0.0
    has_grade = bool(np.isfinite(grade).any())
    max_grade = float(np.nanmax(np.abs(grade))) if has_grade else None

    not_applicable: list[str] = []
    violations: list[Violation] = []
    feasible = True

    skid = vehicle is not None and vehicle.kinematic_type == "tracked_skid"
    if skid:
        not_applicable += ["min_radius", "steer_rate"]

    # 最小旋回半径 → κ 上限（前進）
    if vehicle is not None and not skid:
        kappa_lim = vehicle.kappa_max_fwd
        if kappa_lim is None and vehicle.min_turning_radius:
            kappa_lim = 1.0 / vehicle.min_turning_radius
        if kappa_lim:
            for s0, s1, peak in _segments_over(s, kappa, kappa_lim * (1.0 + tol)):
                violations.append(
                    Violation(kind="min_radius", s_start=s0, s_end=s1, measured=peak, limit=kappa_lim)
                )

    # 曲率変化率
    if vehicle is not None and vehicle.kappa_rate_max:
        for s0, s1, peak in _segments_over(s, dk, vehicle.kappa_rate_max * (1.0 + tol)):
            violations.append(
                Violation(kind="kappa_rate", s_start=s0, s_end=s1, measured=peak, limit=vehicle.kappa_rate_max)
            )

    # 操舵角（rigid_bicycle のみ。max_steer_angle[rad] を deg に）
    max_steer_required = None
    if vehicle is not None and vehicle.supports_steering_angle() and np.isfinite(steer).any():
        max_steer_required = float(np.nanmax(np.abs(steer)))
        if vehicle.max_steer_angle:
            lim_deg = float(np.degrees(vehicle.max_steer_angle))
            steer_filled = np.where(np.isfinite(steer), steer, 0.0)
            for s0, s1, peak in _segments_over(s, steer_filled, lim_deg * (1.0 + tol)):
                violations.append(
                    Violation(kind="steer_rate", s_start=s0, s_end=s1, measured=peak, limit=lim_deg)
                )

    # 勾配（max_speed 制約は持たないので、DSM があれば情報として violation 化はしない閾値=なし）
    # 勾配の明示制約は車両に無いため、ここでは max_grade_pct を返すのみ（UI に表示）。

    # フットプリント包含（実車体矩形/多角形が走行可能領域に収まるか）。
    # 端点(始終点)は車体半長ぶんのオーバーハングが不可避＆計画で動かせないため判定から除外。
    if vehicle is not None and drivable_mask is not None and transform is not None:
        from ..footprint import trajectory_footprint_violations

        ends = float(getattr(vehicle, "overall_length", 0.0) or 0.0) / 2.0
        violations += trajectory_footprint_violations(traj, vehicle, drivable_mask, transform, ignore_ends_m=ends)

    # feasibility 総合判定（適用外を除いた違反があれば NG）
    if any(v.kind not in not_applicable for v in violations):
        feasible = False
    if (
        vehicle is not None
        and vehicle.min_turning_radius
        and traj.min_radius_m is not None
        and "min_radius" not in not_applicable
        and traj.min_radius_m < vehicle.min_turning_radius
    ):
        feasible = False

    # 速度プロファイル要約（Stage 6）
    speeds = [p.speed_mps for p in traj.points if p.speed_mps is not None]
    times = [p.time_s for p in traj.points if p.time_s is not None]
    max_speed = float(max(speeds)) if speeds else None
    time_total = float(max(times)) if times else None
    stop_d = None
    if max_speed is not None and vehicle is not None:
        from .velocity import stopping_distance

        stop_d = round(stopping_distance(max_speed, vehicle), 2)

    return AnalysisResult(
        min_radius_m=traj.min_radius_m,
        max_curvature=max_k,
        max_curvature_rate=max_dk,
        max_steer_rate_required=max_steer_required,
        max_grade_pct=max_grade,
        feasible=feasible,
        not_applicable=not_applicable,
        violations=violations,
        max_speed_mps=(round(max_speed, 2) if max_speed is not None else None),
        time_total_s=(round(time_total, 1) if time_total is not None else None),
        stopping_distance_m=stop_d,
    )
