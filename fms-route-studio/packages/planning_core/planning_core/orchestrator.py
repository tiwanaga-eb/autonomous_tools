"""経路生成オーケストレータ（設計 §9/§10。HTTP 非依存の単一入口）。

routers/planning.py::plan にあった業務ロジック本体（アルゴリズム選択 → 探索 →
Elastic Band 洗練 → リサンプル → R_min 保証 → 勾配/標高付与 → 解析 → 安全検証）。
API 層はリクエスト解釈とレイヤ IO（ラスタ読込）だけを行い、本モジュールへ委譲する。
これにより経路生成パイプライン全体が FastAPI 無しでテスト・再利用できる。

- 入力 `PlanSpec` はレイヤ ID ではなく読込済み ndarray＋transform を持つ。
- 失敗は `PlanError(status, message)`（status は HTTP 相当のヒント: 400=入力不備 /
  422=条件不成立）。API 層で HTTPException へ変換する。
- `analyze_polyline` は「polyline → 軌跡解析＋安全検証」の共通後処理で、
  /plan・/analyze・寄り付き(simulate) が共用する。
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from .analysis import build_trajectory, elevation_and_grade, min_turning_radius, summarize, verify_safety
from .footprint import footprint_sample_points, path_min_clearance, vehicle_footprint
from .models.analysis import AnalysisResult, SafetyReport
from .models.route import Trajectory
from .models.vehicle import VehicleProfile
from .planners import (
    elastic_band,
    fit_spline_curvature_limited,
    hybrid_astar,
    limit_curvature_polyline,
    plan_dubins,
    plan_grid_astar,
    resample_by_spacing,
    rrt_star,
    sample_reeds_shepp,
    smooth_kinematic_path,
    smooth_polyline_in_corridor,
)
from .rasters import MISREGISTERED_MSG, same_grid


class PlanError(Exception):
    """入力/条件起因の計画失敗。status は HTTP ステータス相当のヒント（400=入力不備, 422=条件不成立）。"""

    def __init__(self, status: int, message: str):
        super().__init__(message)
        self.status = status
        self.message = message


@dataclass
class PlanSpec:
    """plan_route の入力（HTTP・レイヤID 非依存。ラスタは呼び出し側で読込済みの配列を渡す）。"""

    waypoints: np.ndarray                       # (N,2) 経由点 [m, working CRS]
    headings_deg: list = field(default_factory=list)  # 各 waypoint の方位[°]（無い点は None）
    mode: str = "waypoint_guided"               # "auto" | "waypoint_guided"
    algorithm: str = "spline"                   # spline/dubins/grid_astar/hybrid_astar/reeds_shepp/rrt_star
    r_min: float | None = None                  # 最小旋回半径 [m]（None=制約なし）
    kappa_rate_max: float | None = None         # dκ/ds 上限（None=操舵レート平滑なし）
    spacing_m: float | None = 2.0
    samples: int = 2000
    planner_cell_m: float = 1.0
    corridor_width_m: float | None = None
    enforce_footprint: bool = True
    enforce_min_radius: bool = True
    allow_reverse: bool = False
    refine_elastic_band: bool = False
    vehicle: VehicleProfile | None = None
    cost: np.ndarray | None = None              # コストマップ（auto/A*/RRT* のソフトコスト）
    cost_transform: object | None = None
    obstacle_value: float = 1e9
    mask: np.ndarray | None = None              # 走行可能領域（ハード制約・EB/包含/安全検証にも使用）
    mask_transform: object | None = None
    dsm: np.ndarray | None = None               # 点群由来 DSM（勾配・標高 z の埋め込み）
    dsm_transform: object | None = None
    no_go_polygons: list | None = None          # 進入禁止領域（world 座標多角形のリスト）


@dataclass
class PlanOutcome:
    """plan_route の出力（API レスポンスと 1:1 対応）。"""

    trajectory: Trajectory
    analysis: AnalysisResult
    safety: SafetyReport
    algorithm: str
    used_smoothing_s: float = 0.0
    measured_min_radius_m: float | None = None
    min_clearance_m: float | None = None
    warning: str | None = None
    refined_elastic_band: bool = False


def rasterize_nogo(polygons, transform, shape) -> np.ndarray | None:
    """進入禁止(NoGoZone)多角形を burn(bool)にラスタ化。設計 §9.5「NoGoZone を制約に」。"""
    if not polygons or transform is None:
        return None
    import rasterio.features as rfeat

    geoms = []
    for ring in polygons:
        if not ring or len(ring) < 3:
            continue
        coords = [[float(p[0]), float(p[1])] for p in ring]
        coords.append(coords[0])
        geoms.append(({"type": "Polygon", "coordinates": [coords]}, 1))
    if not geoms:
        return None
    return rfeat.rasterize(geoms, out_shape=shape, transform=transform, fill=0, dtype="uint8").astype(bool)


def _erode_for_width(mask: np.ndarray, cell_m: float, width_m: float) -> np.ndarray:
    """道幅 width_m の車体中心線が領域内に収まるよう、mask を半幅ぶん収縮（距離変換）。

    収縮後の mask 内にある中心線は、最近傍の非走行可能セルまで >= width/2 を保証する。
    """
    half = width_m / 2.0
    if half <= 0:
        return (mask > 0).astype(np.uint8)
    from scipy.ndimage import distance_transform_edt

    dt = distance_transform_edt(mask > 0)  # [cells]
    return (dt * cell_m >= half).astype(np.uint8)


def _apply_endpoint_headings(pts: np.ndarray, head_start, head_goal, lead: float) -> np.ndarray:
    """始点/終点の方位ベクトルに沿うガイド点を内側に挿入し、spline の端接線を方位へ寄せる。"""
    pts = [list(p) for p in np.asarray(pts, float)]
    if len(pts) < 2 or lead <= 0:
        return np.asarray(pts, float)
    out = [pts[0]]
    if head_start is not None:
        a = np.radians(head_start)
        out.append([pts[0][0] + lead * np.cos(a), pts[0][1] + lead * np.sin(a)])
    out += pts[1:-1]
    if head_goal is not None:
        a = np.radians(head_goal)
        out.append([pts[-1][0] - lead * np.cos(a), pts[-1][1] - lead * np.sin(a)])
    out.append(pts[-1])
    return np.asarray(out, float)


def _nearest_gears(final_xy: np.ndarray, raw: list) -> list[str]:
    """最終経路の各点に、生プランナ点(x,y,gear)から最近傍の gear を割り当てる。
    resample/EB 後でも位置の最近傍で対応づくため、後進セグメントを保てる。"""
    rx = np.array([p[0] for p in raw], float)
    ry = np.array([p[1] for p in raw], float)
    rg = [p[2] for p in raw]
    out: list[str] = []
    for x, y in final_xy:
        j = int(np.argmin((rx - x) ** 2 + (ry - y) ** 2))
        out.append(rg[j] or "F")
    return out


def analyze_polyline(
    pts,
    *,
    vehicle: VehicleProfile | None = None,
    gears: list | None = None,
    curvature_source: str = "numeric",
    dsm: np.ndarray | None = None,
    dsm_transform=None,
    drivable_mask: np.ndarray | None = None,
    drivable_transform=None,
    min_speed_mps: float = 0.0,
    advisory_kinds: tuple[str, ...] = (),
    approach_error_m: float | None = None,
    approach_error_deg: float | None = None,
) -> tuple[Trajectory, AnalysisResult, SafetyReport, float | None, str | None]:
    """polyline → (Trajectory(z/grade付), AnalysisResult, SafetyReport, min_clearance_m, grade_warning)。

    /plan・/analyze・寄り付き(simulate) 共通の後処理。DSM があれば標高 z と縦断勾配を
    軌跡へ埋め込む。勾配/標高の**計算**失敗（範囲・形状起因の ValueError/IndexError）は
    grade_warning に載せて続行する（DSM ファイル読込の失敗は呼び出し側 IO 層の責務）。
    """
    pts = np.asarray(pts, float)
    grade = zprof = None
    grade_warning: str | None = None
    if dsm is not None and dsm_transform is not None and len(pts) >= 2:
        try:
            s = np.concatenate([[0.0], np.cumsum(np.hypot(np.diff(pts[:, 0]), np.diff(pts[:, 1])))])
            zprof, grade = elevation_and_grade(pts, s, dsm, dsm_transform)
        except (ValueError, IndexError) as e:
            grade = zprof = None
            grade_warning = f"勾配/標高の計算に失敗（DSM範囲/CRS不整合の可能性）: {e}"
    traj = build_trajectory(
        pts, curvature_source=curvature_source, vehicle=vehicle,
        grade_pct=grade, gears=gears, min_speed_mps=min_speed_mps, z=zprof,
    )
    result = summarize(traj, vehicle=vehicle, drivable_mask=drivable_mask, transform=drivable_transform)
    clearance_m = None
    if drivable_mask is not None and drivable_transform is not None:
        try:
            clearance_m, _ = path_min_clearance(pts, drivable_mask, drivable_transform)
        except Exception:  # noqa: BLE001 — クリアランスは診断値。失敗しても解析全体は返す
            clearance_m = None
    safety = verify_safety(
        traj, result, vehicle, clearance_m=clearance_m,
        advisory_kinds=advisory_kinds, footprint_evaluated=drivable_mask is not None,
        approach_error_m=approach_error_m, approach_error_deg=approach_error_deg,
    )
    return traj, result, safety, clearance_m, grade_warning


def plan_route(spec: PlanSpec) -> PlanOutcome:  # noqa: C901 — アルゴリズム分岐の単一入口
    """経路生成の単一入口。PlanSpec → PlanOutcome（失敗は PlanError）。"""
    pts = np.asarray(spec.waypoints, float)
    if len(pts) < 2:
        raise PlanError(400, "need >= 2 waypoints")
    heads = list(spec.headings_deg) if spec.headings_deg else [None] * len(pts)
    if len(heads) != len(pts):
        raise PlanError(400, "headings_deg の数が waypoints と一致しません")
    veh = spec.vehicle
    r_min = spec.r_min
    kappa_rate_max = spec.kappa_rate_max
    warn: str | None = None
    measured_r: float = float("inf")
    used_s = 0.0
    source = "numeric"
    # 後進つきプランナ(hybrid/RS/RRT*)の生 (x,y,gear)。最終経路に gear をマップして解析へ渡す
    # （後進セグメントの車体方位・cusp 評価を正しくする。前進のみアルゴは None）。
    raw_gear_pts: list | None = None

    # auto モードのアルゴリズム解決: 既定は hybrid_astar（運動学的・方位尊重）、半径不明なら grid_astar。
    algo = spec.algorithm
    if spec.mode == "auto" and algo not in ("grid_astar", "hybrid_astar"):
        algo = "hybrid_astar" if (r_min and r_min > 0) else "grid_astar"

    if algo in ("grid_astar", "hybrid_astar"):
        if spec.cost is None or spec.cost_transform is None:
            raise PlanError(400, "auto/grid_astar needs a costmap layer (costmap_layer_id or via drivable)")
        cost, transform = spec.cost, spec.cost_transform
        mask = spec.mask
        if mask is not None:
            # co-registration: 探索は1つの transform で cost/mask を併参照する。
            # shape だけでなく transform も照合し、別ゾーン/別コストマップ由来の誤結合を弾く。
            if not same_grid(transform, cost.shape, spec.mask_transform, mask.shape):
                raise PlanError(422, MISREGISTERED_MSG)
        obstacle = float(spec.obstacle_value)

        # 進入禁止領域(NoGoZone): cost を obstacle に、mask を 0 にカーブアウト（planner が回避）。
        burn = rasterize_nogo(spec.no_go_polygons, transform, cost.shape)
        if burn is not None:
            cost = np.where(burn, obstacle, cost)
            if mask is not None:
                mask = np.where(burn, 0, mask)

        # フットプリント包含: hybrid A* は車両があれば実車体多角形で衝突判定（向き付き矩形）。
        # その場合は半幅収縮(円近似)は使わない（footprint が正確に道幅を担保するため二重適用を避ける）。
        # footprint を使わない場合（車両なし等）は従来どおり corridor_width_m 指定時だけ半幅収縮する。
        cell_m = float(abs(transform.a))
        fp_search = None
        if algo == "hybrid_astar" and veh is not None and mask is not None and spec.enforce_footprint:
            poly = vehicle_footprint(veh)
            # 探索用は粗め（車体寸/6 か セル）でサンプルし速度を確保。解析(summarize)では cell 精度で再検査する。
            search_spacing = max(cell_m, max(veh.overall_width, veh.overall_length) / 6.0)
            fp_search = footprint_sample_points(poly, search_spacing)

        width = spec.corridor_width_m if (spec.corridor_width_m and spec.corridor_width_m > 0) else 0.0
        mask_eff = mask
        eroded = False
        if mask is not None and width and width > 0 and fp_search is None:
            mask_eff = _erode_for_width(mask, cell_m, width)
            eroded = True
            if int(mask_eff.sum()) == 0:
                raise PlanError(
                    422,
                    f"道幅 {width:.1f}m が走行可能領域に対して広すぎます（収縮後に通行可能セルが残りません）。"
                    f" 道幅を狭めるか走行可能領域を拡張してください。",
                )

        if algo == "hybrid_astar":
            if not r_min or r_min <= 0:
                raise PlanError(400, "hybrid_astar needs min_turn_radius（vehicle_id か min_turn_radius_m を指定）")
            chord = math.atan2(pts[-1, 1] - pts[0, 1], pts[-1, 0] - pts[0, 0])
            syaw = math.radians(heads[0]) if heads[0] is not None else chord
            gyaw = math.radians(heads[-1]) if heads[-1] is not None else chord
            hres = hybrid_astar(
                (pts[0, 0], pts[0, 1], syaw), (pts[-1, 0], pts[-1, 1], gyaw),
                rho=r_min, mask=mask_eff, transform=transform, cost=cost, obstacle_value=obstacle,
                footprint=fp_search,
                allow_reverse=spec.allow_reverse,
                max_snap_m=max(3.0, 2.0 * spec.planner_cell_m),
                xy_res=spec.planner_cell_m, pos_tol=max(2.0 * spec.planner_cell_m, 2.0),
                analytic_radius=max(4.0 * r_min, 12.0),
            )
            if hres is None:
                fp_hint = (
                    "（車体フットプリント包含が有効: 実車体が走行可能領域に収まる経路が必要です。"
                    f"車両{veh.id if veh else ''}は全長{veh.overall_length if veh else '?'}m×"
                    f"全幅{veh.overall_width if veh else '?'}m。"
                    "領域が車体に対し狭い/非連結の可能性。enforce_footprint を切るか領域を拡張してください）"
                    if fp_search is not None
                    else "（領域が狭い/非連結、または R_min に対しコリドーが狭い可能性。道幅・走行可能領域・始終点方位を見直してください）"
                )
                raise PlanError(422, f"hybrid A* で start→goal の運動学的経路が見つかりません{fp_hint}")
            curve = np.array([[p[0], p[1]] for p in hres["points"]], float)
            raw_gears = [p[3] for p in hres["points"]]
            # Hybrid A* は離散プリミティブの繋ぎで「クネクネ」しがち。cusp(進行方向反転)を保持したまま
            # コリドー内で角刈り平滑化して滑らかにする（R_min は Chaikin が増やす方向なので保たれる）。
            curve, sm_gears = smooth_kinematic_path(
                curve, raw_gears, mask_eff, transform, r_min=r_min, kappa_rate_max=kappa_rate_max
            )
            raw_gear_pts = [(float(x), float(y), g) for (x, y), g in zip(curve, sm_gears or raw_gears)]
            measured_r = float(min_turning_radius(curve))
            source = "numeric"
        else:
            # 大域コリドー抽出は粗格子で（fine 0.1m は重く脆い）。
            # 収縮済みは clearance 保証済みなので粗格子化は any-free（細いコリドーを消さない）。
            raw = plan_grid_astar(
                cost, transform, (pts[0, 0], pts[0, 1]), (pts[-1, 0], pts[-1, 1]),
                drivable_mask=mask_eff, obstacle_value=obstacle, planner_cell_m=spec.planner_cell_m,
                passable_frac=(0.0 if eroded else 0.5),
                max_snap_m=max(width / 2.0 + 2.0 * spec.planner_cell_m, 3.0),
            )
            if len(raw) <= 2 and (mask is not None or np.isfinite(obstacle)):
                raise PlanError(
                    422,
                    f"道幅 {width:.1f}m を確保した走行可能領域内に start→goal の経路が見つかりません"
                    "（領域が狭い/非連結、または始終点が領域外の可能性）。道幅か走行可能領域を見直してください。",
                )
            # 糸引きで階段を taut 化 → 始終点方位ガイド → 曲率/操舵レート制限で平滑化（領域内優先）。
            curve, warn, measured_r, _frac = smooth_polyline_in_corridor(
                raw, r_min, mask, transform, n=spec.samples,
                kappa_rate_max=kappa_rate_max,
                head_start=heads[0],
                head_goal=heads[-1],
            )
        source = "numeric"
    elif algo == "dubins":
        if r_min is None or r_min <= 0:
            raise PlanError(400, "dubins needs min_turn_radius (set vehicle_id or min_turn_radius_m)")
        step = max((spec.spacing_m or 1.0) * 0.5, 0.2)
        dub = plan_dubins(pts, rho=r_min, step=step, headings_deg=heads)
        if kappa_rate_max:
            # Dubins の曲率ステップ（瞬間操舵）をクロソイド近似で連続化＝実車に則す。
            dub_rs = resample_by_spacing(dub, max((spec.spacing_m or 1.0) * 0.5, 0.5))
            curve, used_s, warn, measured_r, _ = fit_spline_curvature_limited(
                dub_rs, r_min, kappa_rate_max=kappa_rate_max, n=spec.samples
            )
            source = "numeric"
        else:
            curve = dub
            measured_r = float(min_turning_radius(curve))
            source = "analytic"
    elif algo == "reeds_shepp":
        if r_min is None or r_min <= 0:
            raise PlanError(400, "reeds_shepp needs min_turn_radius（vehicle_id か min_turn_radius_m を指定）")
        step = max((spec.spacing_m or 1.0) * 0.5, 0.3)
        seg_pts: list[list[float]] = []
        seg_gears: list[tuple[float, float, str]] = []
        for i in range(len(pts) - 1):
            chord = math.atan2(pts[i + 1, 1] - pts[i, 1], pts[i + 1, 0] - pts[i, 0])
            syaw = math.radians(heads[i]) if heads[i] is not None else chord
            gyaw = math.radians(heads[i + 1]) if heads[i + 1] is not None else chord
            rs = sample_reeds_shepp((pts[i, 0], pts[i, 1], syaw), (pts[i + 1, 0], pts[i + 1, 1], gyaw), r_min, step=step)
            if rs is None:
                raise PlanError(422, f"reeds_shepp: 区間 {i}→{i + 1} を接続できません（R_min={r_min:.1f}m）。方位/配置を見直してください。")
            for k, p in enumerate(rs):
                if k == 0 and seg_pts:
                    continue  # 区間端の重複を除く
                seg_pts.append([p[0], p[1]])
                seg_gears.append((float(p[0]), float(p[1]), p[3]))
        curve = np.asarray(seg_pts, float)
        raw_gear_pts = seg_gears  # 後進セグメント保持
        measured_r = float(min_turning_radius(curve))
        source = "numeric"
    elif algo == "rrt_star":
        if r_min is None or r_min <= 0:
            raise PlanError(400, "rrt_star needs min_turn_radius（vehicle_id か min_turn_radius_m を指定）")
        # コスト/領域があれば利用（任意）。footprint は車両＋enforce 時に厳密衝突判定。
        rc, rtf = spec.cost, spec.cost_transform
        rm, mt = spec.mask, spec.mask_transform
        robstacle = float(spec.obstacle_value)
        if rm is not None:
            if rtf is None:
                rtf = mt
            elif rc is not None and not same_grid(rtf, rc.shape, mt, rm.shape):
                # rrt_star も 1 transform で cost/mask を併参照するため co-registration 必須
                raise PlanError(422, MISREGISTERED_MSG)
        if rtf is not None and rc is not None:
            burn = rasterize_nogo(spec.no_go_polygons, rtf, rc.shape)
            if burn is not None:
                rc = np.where(burn, robstacle, rc)
                if rm is not None:
                    rm = np.where(burn, 0, rm)
        fp_search = None
        if veh is not None and rm is not None and spec.enforce_footprint:
            cell_m = float(abs(rtf.a))
            fp_search = footprint_sample_points(
                vehicle_footprint(veh), max(cell_m, max(veh.overall_width, veh.overall_length) / 6.0)
            )
        chord = math.atan2(pts[-1, 1] - pts[0, 1], pts[-1, 0] - pts[0, 0])
        syaw = math.radians(heads[0]) if heads[0] is not None else chord
        gyaw = math.radians(heads[-1]) if heads[-1] is not None else chord
        rres = rrt_star(
            (pts[0, 0], pts[0, 1], syaw), (pts[-1, 0], pts[-1, 1], gyaw),
            rho=r_min, mask=rm, transform=rtf, cost=rc, obstacle_value=robstacle,
            footprint=fp_search, xy_res=spec.planner_cell_m,
        )
        if rres is None:
            raise PlanError(422, "rrt_star で start→goal の経路が見つかりません（反復上限内に未到達、または領域が狭い/非連結の可能性）。")
        curve = np.array([[p[0], p[1]] for p in rres["points"]], float)
        raw_gear_pts = [(float(p[0]), float(p[1]), p[3]) for p in rres["points"]]  # 後進セグメント保持
        measured_r = float(min_turning_radius(curve))
        source = "numeric"
    else:  # spline
        # 始終点の方位ベクトルがあれば端接線をそれへ寄せる（ガイド点挿入）。
        hs, hg = heads[0], heads[-1]
        spts = pts
        if hs is not None or hg is not None:
            lead = max((r_min or 0) * 0.6, (spec.spacing_m or 2.0) * 1.5, 4.0)
            spts = _apply_endpoint_headings(pts, hs, hg, lead)
        curve, used_s, warn, measured_r, _ = fit_spline_curvature_limited(
            spts, r_min, kappa_rate_max=kappa_rate_max, n=spec.samples
        )
        source = "numeric"

    # 走行可能 mask（EB 洗練・フットプリント包含チェックで共用）。NoGoZone もここで除外。
    dmask, dtransform = spec.mask, spec.mask_transform
    if dmask is not None:
        dburn = rasterize_nogo(spec.no_go_polygons, dtransform, dmask.shape)
        if dburn is not None:
            dmask = np.where(dburn, 0, dmask)

    # 4b Elastic Band 洗練（任意）: コリドー中央へ寄せ、車体が収まる余裕を確保。
    eb_applied = False
    if spec.refine_elastic_band and dmask is not None and len(curve) >= 3:
        half = 0.0
        if veh is not None:
            half = max(half, veh.overall_width / 2.0)
        if spec.corridor_width_m and spec.corridor_width_m > 0:
            half = max(half, spec.corridor_width_m / 2.0)
        desired = (half + 0.3) if half > 0 else 1.5
        node_sp = max(min(spec.spacing_m or 2.0, 2.0), 1.0)
        eb_in = resample_by_spacing(curve, node_sp)
        curve = elastic_band(eb_in, dmask, dtransform, desired_clearance=desired)
        eb_applied = True
        # EB で曲率が増え得るため、R_min が分かれば曲率/操舵レート制限で仕上げる。
        if r_min and r_min > 0:
            curve, used_s, w2, measured_r, _ = fit_spline_curvature_limited(
                curve, r_min, kappa_rate_max=kappa_rate_max, n=spec.samples
            )
            warn = w2 or warn
            source = "numeric"

    final = resample_by_spacing(curve, spec.spacing_m) if spec.spacing_m else curve

    # R_min 保証: 違反コーナーだけ局所平滑化して max|κ| <= 1/R_min に。
    # R_min を持たない車種（CD110R スキッド等）は r_min=None なのでスキップ。drivable があれば領域内に留める。
    # hybrid A* は運動学的に R_min を保証済み＆切り返し(cusp)を持ち得るため、曲率制限パスは適用しない
    # （cusp を平滑化で潰すと切り返しが壊れる）。spline/dubins/grid の非運動学経路にのみ適用。
    if spec.enforce_min_radius and r_min and r_min > 0 and len(final) >= 3 and algo not in ("hybrid_astar", "reeds_shepp", "rrt_star"):
        final, mr = limit_curvature_polyline(final, r_min, mask=dmask, transform=dtransform)
        if np.isfinite(mr):
            measured_r = mr
            if mr >= r_min * 0.99:
                warn = None  # R_min 達成（旧・大域平滑化の警告は解消）
            else:
                warn = (
                    f"R_min={r_min:.1f}m を完全には満たせません（最小 R={mr:.1f}m）。"
                    "配置点が車両に対しタイト/コリドーが狭い可能性。"
                )

    # 共通後処理: 勾配/標高付与 → 軌跡構築 → 解析 → 安全検証。
    # 後進つきプランナは生 gear を最終経路へマップ（後進の車体方位・cusp 評価を正しく）。
    gears = _nearest_gears(final, raw_gear_pts) if raw_gear_pts else None
    traj, result, safety, clearance_m, grade_warning = analyze_polyline(
        final, vehicle=veh, gears=gears, curvature_source=source,
        dsm=spec.dsm, dsm_transform=spec.dsm_transform,
        drivable_mask=dmask, drivable_transform=dtransform,
    )
    if grade_warning:
        warn = (warn + " / " if warn else "") + grade_warning
    # スキッドステア車は専用プリミティブ（差動旋回・その場旋回）を持たず Ackermann 系の
    # 経路で計画される。Ackermann 追従可能ならスキッド車も追従可能なので保守側だが、
    # その場旋回を活かした最短経路にはならない — 近似であることを明示する。
    if veh is not None and veh.kinematic_type == "tracked_skid":
        skid_note = "スキッドステア車は Ackermann 近似で計画（保守的・その場旋回は未活用）"
        warn = (warn + " / " if warn else "") + skid_note

    return PlanOutcome(
        trajectory=traj,
        analysis=result,
        safety=safety,
        algorithm=algo,
        used_smoothing_s=used_s,
        measured_min_radius_m=(float(measured_r) if np.isfinite(measured_r) else None),
        min_clearance_m=clearance_m,
        warning=warn,
        refined_elastic_band=eb_applied,
    )
