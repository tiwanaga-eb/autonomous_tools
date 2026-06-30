"""経路生成 API（設計書 §9 / §10）。

Phase 4a: 2モード（auto / waypoint_guided）× 複数アルゴリズム（spline / dubins / grid_astar）。
- 走行可能領域(drivable mask)を **ハード制約**、cost を **ソフトコスト** に（grid_astar）。
- 車両プロファイルで R_min を保証・操舵角/曲率の feasibility と violation 区間を算出。
- costmap レイヤの DSM があれば軌跡に沿った縦断勾配を付与。
"""
from __future__ import annotations

import math
from typing import Literal

import numpy as np
import rasterio
from fastapi import APIRouter, HTTPException
from pydantic import BaseModel, Field

from planning_core.analysis import build_trajectory, grade_profile, min_turning_radius, summarize, verify_safety
from planning_core.footprint import footprint_sample_points, path_min_clearance, vehicle_footprint
from planning_core.planners import (
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
from .. import store
from .. import vehicle_overrides

router = APIRouter(prefix="/api", tags=["planning"])

Algorithm = Literal["spline", "dubins", "grid_astar", "hybrid_astar", "reeds_shepp", "rrt_star"]
PlanMode = Literal["auto", "waypoint_guided"]


class XYIn(BaseModel):
    x: float
    y: float
    heading_deg: float | None = None


class PlanRequest(BaseModel):
    waypoints: list[XYIn]
    mode: PlanMode = "waypoint_guided"
    algorithm: Algorithm = "spline"
    min_turn_radius_m: float | None = Field(None, gt=0)   # 正のみ（0/負は無効）
    spacing_m: float | None = Field(2.0, gt=0)            # リサンプル間隔は正のみ
    samples: int = Field(2000, ge=50, le=20000)
    planner_cell_m: float = Field(1.0, ge=0.1, le=10.0)  # auto/A* の粗格子セル[m]
    limit_steer_rate: bool = True          # dκ/ds(操舵レート)を車両上限以下に平滑化
    enforce_footprint: bool = True         # hybrid A* で実車体フットプリント(向き付き矩形)を厳密な走行可能制約に
    enforce_min_radius: bool = True        # R_min を持つ車種で、違反コーナーを局所平滑化して最小旋回半径を保証
    allow_reverse: bool = False            # hybrid A* で後進プリミティブを許可（切り返しで狭所到達性UP）
    refine_elastic_band: bool = False      # 生成後に Elastic Band で洗練（コリドー中央へ寄せ余裕確保）
    corridor_width_m: float | None = Field(None, gt=0)  # A*で確保する道幅[m]（None=車両 overall_width）
    vehicle_id: str | None = None
    costmap_layer_id: str | None = None    # auto モードの cost / 勾配 DSM 供給元
    drivable_layer_id: str | None = None   # ハード制約（フットプリント包含）
    no_go_polygons: list[list[tuple[float, float]]] | None = None  # 進入禁止領域（world座標多角形）


class AnalyzeRequest(BaseModel):
    points: list[XYIn]
    vehicle_id: str | None = None
    costmap_layer_id: str | None = None


def _vehicle(vid: str | None):
    if not vid:
        return None
    try:
        return vehicle_overrides.resolve(vid)  # 既定＋車種別オーバーライド（チューニング値）
    except FileNotFoundError:
        raise HTTPException(404, f"unknown vehicle: {vid}")


def _read_raster(path: str):
    with rasterio.open(path) as ds:
        arr = ds.read(1)
        return arr, ds.transform


def _drivable_mask(drivable_layer_id: str | None):
    """drivable レイヤの mask と transform を返す（無ければ (None, None)）。EB/包含チェック共用。"""
    if not drivable_layer_id:
        return None, None
    dl = store.get_layer(drivable_layer_id)
    if dl and "mask_cog" in dl:
        return _read_raster(dl["mask_cog"])
    return None, None


def _rasterize_nogo(polygons, transform, shape) -> np.ndarray | None:
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


def _cost_layer_for(req_costmap_id: str | None, drivable_id: str | None) -> dict | None:
    """cost レイヤ meta を解決（明示 > drivable 経由）。"""
    if req_costmap_id:
        cl = store.get_layer(req_costmap_id)
        if not cl or cl.get("kind") != "cost":
            raise HTTPException(404, f"cost layer not found: {req_costmap_id}")
        return cl
    if drivable_id:
        dl = store.get_layer(drivable_id)
        if dl and dl.get("cost_layer_id"):
            return store.get_layer(dl["cost_layer_id"])
    return None


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


def _r_min(req: PlanRequest, veh) -> float | None:
    if req.min_turn_radius_m and req.min_turn_radius_m > 0:
        return req.min_turn_radius_m
    if veh is not None and veh.kinematic_type != "tracked_skid":
        return veh.min_turning_radius
    return None


def _nearest_gears(final_xy: np.ndarray, raw: list[tuple[float, float, str]]) -> list[str]:
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


@router.post("/plan")
def plan(req: PlanRequest):
    if len(req.waypoints) < 2:
        raise HTTPException(400, "need >= 2 waypoints")
    pts = np.array([[p.x, p.y] for p in req.waypoints], dtype=float)
    veh = _vehicle(req.vehicle_id)
    r_min = _r_min(req, veh)
    kappa_rate_max = (veh.kappa_rate_max if (veh and req.limit_steer_rate) else None)
    warn: str | None = None
    measured_r: float = float("inf")
    used_s = 0.0
    source = "numeric"
    # 後進つきプランナ(hybrid/RS/RRT*)の生 (x,y,gear)。最終経路に gear をマップして解析へ渡す
    # （後進セグメントの車体方位・cusp 評価を正しくする。前進のみアルゴは None）。
    raw_gear_pts: list[tuple[float, float, str]] | None = None

    # auto モードのアルゴリズム解決: 既定は hybrid_astar（運動学的・方位尊重）、半径不明なら grid_astar。
    algo = req.algorithm
    if req.mode == "auto" and algo not in ("grid_astar", "hybrid_astar"):
        algo = "hybrid_astar" if (r_min and r_min > 0) else "grid_astar"

    if algo in ("grid_astar", "hybrid_astar"):
        cl = _cost_layer_for(req.costmap_layer_id, req.drivable_layer_id)
        if not cl or "cost_cog" not in cl:
            raise HTTPException(400, "auto/grid_astar needs a costmap layer (costmap_layer_id or via drivable)")
        cost, transform = _read_raster(cl["cost_cog"])
        mask = None
        if req.drivable_layer_id:
            dl = store.get_layer(req.drivable_layer_id)
            if dl and "mask_cog" in dl:
                mask, _ = _read_raster(dl["mask_cog"])
        obstacle = float(cl.get("obstacle_value", 1e9))

        # 進入禁止領域(NoGoZone): cost を obstacle に、mask を 0 にカーブアウト（planner が回避）。
        burn = _rasterize_nogo(req.no_go_polygons, transform, cost.shape)
        if burn is not None:
            cost = np.where(burn, obstacle, cost)
            if mask is not None:
                mask = np.where(burn, 0, mask)

        # フットプリント包含: hybrid A* は車両があれば実車体多角形で衝突判定（向き付き矩形）。
        # その場合は半幅収縮(円近似)は使わない（footprint が正確に道幅を担保するため二重適用を避ける）。
        # footprint を使わない場合（車両なし等）は従来どおり corridor_width_m 指定時だけ半幅収縮する。
        cell_m = float(abs(transform.a))
        fp_search = None
        if algo == "hybrid_astar" and veh is not None and mask is not None and req.enforce_footprint:
            poly = vehicle_footprint(veh)
            # 探索用は粗め（車体寸/6 か セル）でサンプルし速度を確保。解析(summarize)では cell 精度で再検査する。
            search_spacing = max(cell_m, max(veh.overall_width, veh.overall_length) / 6.0)
            fp_search = footprint_sample_points(poly, search_spacing)

        width = req.corridor_width_m if (req.corridor_width_m and req.corridor_width_m > 0) else 0.0
        mask_eff = mask
        eroded = False
        if mask is not None and width and width > 0 and fp_search is None:
            mask_eff = _erode_for_width(mask, cell_m, width)
            eroded = True
            if int(mask_eff.sum()) == 0:
                raise HTTPException(
                    422,
                    f"道幅 {width:.1f}m が走行可能領域に対して広すぎます（収縮後に通行可能セルが残りません）。"
                    f" 道幅を狭めるか走行可能領域を拡張してください。",
                )

        if algo == "hybrid_astar":
            if not r_min or r_min <= 0:
                raise HTTPException(400, "hybrid_astar needs min_turn_radius（vehicle_id か min_turn_radius_m を指定）")
            chord = math.atan2(pts[-1, 1] - pts[0, 1], pts[-1, 0] - pts[0, 0])
            syaw = math.radians(req.waypoints[0].heading_deg) if req.waypoints[0].heading_deg is not None else chord
            gyaw = math.radians(req.waypoints[-1].heading_deg) if req.waypoints[-1].heading_deg is not None else chord
            hres = hybrid_astar(
                (pts[0, 0], pts[0, 1], syaw), (pts[-1, 0], pts[-1, 1], gyaw),
                rho=r_min, mask=mask_eff, transform=transform, cost=cost, obstacle_value=obstacle,
                footprint=fp_search,
                allow_reverse=req.allow_reverse,
                max_snap_m=max(3.0, 2.0 * req.planner_cell_m),
                xy_res=req.planner_cell_m, pos_tol=max(2.0 * req.planner_cell_m, 2.0),
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
                raise HTTPException(422, f"hybrid A* で start→goal の運動学的経路が見つかりません{fp_hint}")
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
                drivable_mask=mask_eff, obstacle_value=obstacle, planner_cell_m=req.planner_cell_m,
                passable_frac=(0.0 if eroded else 0.5),
                max_snap_m=max(width / 2.0 + 2.0 * req.planner_cell_m, 3.0),
            )
            if len(raw) <= 2 and (mask is not None or np.isfinite(obstacle)):
                raise HTTPException(
                    422,
                    f"道幅 {width:.1f}m を確保した走行可能領域内に start→goal の経路が見つかりません"
                    "（領域が狭い/非連結、または始終点が領域外の可能性）。道幅か走行可能領域を見直してください。",
                )
            # 糸引きで階段を taut 化 → 始終点方位ガイド → 曲率/操舵レート制限で平滑化（領域内優先）。
            curve, warn, measured_r, _frac = smooth_polyline_in_corridor(
                raw, r_min, mask, transform, n=req.samples,
                kappa_rate_max=kappa_rate_max,
                head_start=req.waypoints[0].heading_deg,
                head_goal=req.waypoints[-1].heading_deg,
            )
        source = "numeric"
    elif algo == "dubins":
        if r_min is None or r_min <= 0:
            raise HTTPException(400, "dubins needs min_turn_radius (set vehicle_id or min_turn_radius_m)")
        headings = [p.heading_deg for p in req.waypoints]
        step = max((req.spacing_m or 1.0) * 0.5, 0.2)
        dub = plan_dubins(pts, rho=r_min, step=step, headings_deg=headings)
        if kappa_rate_max:
            # Dubins の曲率ステップ（瞬間操舵）をクロソイド近似で連続化＝実車に則す。
            dub_rs = resample_by_spacing(dub, max((req.spacing_m or 1.0) * 0.5, 0.5))
            curve, used_s, warn, measured_r, _ = fit_spline_curvature_limited(
                dub_rs, r_min, kappa_rate_max=kappa_rate_max, n=req.samples
            )
            source = "numeric"
        else:
            curve = dub
            measured_r = float(min_turning_radius(curve))
            source = "analytic"
    elif algo == "reeds_shepp":
        if r_min is None or r_min <= 0:
            raise HTTPException(400, "reeds_shepp needs min_turn_radius（vehicle_id か min_turn_radius_m を指定）")
        step = max((req.spacing_m or 1.0) * 0.5, 0.3)
        headings = [p.heading_deg for p in req.waypoints]
        seg_pts: list[list[float]] = []
        seg_gears: list[tuple[float, float, str]] = []
        for i in range(len(pts) - 1):
            chord = math.atan2(pts[i + 1, 1] - pts[i, 1], pts[i + 1, 0] - pts[i, 0])
            syaw = math.radians(headings[i]) if headings[i] is not None else chord
            gyaw = math.radians(headings[i + 1]) if headings[i + 1] is not None else chord
            rs = sample_reeds_shepp((pts[i, 0], pts[i, 1], syaw), (pts[i + 1, 0], pts[i + 1, 1], gyaw), r_min, step=step)
            if rs is None:
                raise HTTPException(422, f"reeds_shepp: 区間 {i}→{i + 1} を接続できません（R_min={r_min:.1f}m）。方位/配置を見直してください。")
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
            raise HTTPException(400, "rrt_star needs min_turn_radius（vehicle_id か min_turn_radius_m を指定）")
        # コスト/領域があれば利用（任意）。footprint は車両＋enforce 時に厳密衝突判定。
        rc = rm = rtf = None
        robstacle = 1e9
        cl = _cost_layer_for(req.costmap_layer_id, req.drivable_layer_id)
        if cl and "cost_cog" in cl:
            rc, rtf = _read_raster(cl["cost_cog"])
            robstacle = float(cl.get("obstacle_value", 1e9))
        if req.drivable_layer_id:
            dl = store.get_layer(req.drivable_layer_id)
            if dl and "mask_cog" in dl:
                rm, mt = _read_raster(dl["mask_cog"])
                if rtf is None:
                    rtf = mt
        if rtf is not None and rc is not None:
            burn = _rasterize_nogo(req.no_go_polygons, rtf, rc.shape)
            if burn is not None:
                rc = np.where(burn, robstacle, rc)
                if rm is not None:
                    rm = np.where(burn, 0, rm)
        fp_search = None
        if veh is not None and rm is not None and req.enforce_footprint:
            cell_m = float(abs(rtf.a))
            fp_search = footprint_sample_points(
                vehicle_footprint(veh), max(cell_m, max(veh.overall_width, veh.overall_length) / 6.0)
            )
        chord = math.atan2(pts[-1, 1] - pts[0, 1], pts[-1, 0] - pts[0, 0])
        syaw = math.radians(req.waypoints[0].heading_deg) if req.waypoints[0].heading_deg is not None else chord
        gyaw = math.radians(req.waypoints[-1].heading_deg) if req.waypoints[-1].heading_deg is not None else chord
        rres = rrt_star(
            (pts[0, 0], pts[0, 1], syaw), (pts[-1, 0], pts[-1, 1], gyaw),
            rho=r_min, mask=rm, transform=rtf, cost=rc, obstacle_value=robstacle,
            footprint=fp_search, xy_res=req.planner_cell_m,
        )
        if rres is None:
            raise HTTPException(422, "rrt_star で start→goal の経路が見つかりません（反復上限内に未到達、または領域が狭い/非連結の可能性）。")
        curve = np.array([[p[0], p[1]] for p in rres["points"]], float)
        raw_gear_pts = [(float(p[0]), float(p[1]), p[3]) for p in rres["points"]]  # 後進セグメント保持
        measured_r = float(min_turning_radius(curve))
        source = "numeric"
    else:  # spline
        # 始終点の方位ベクトルがあれば端接線をそれへ寄せる（ガイド点挿入）。
        hs = req.waypoints[0].heading_deg
        hg = req.waypoints[-1].heading_deg
        spts = pts
        if hs is not None or hg is not None:
            lead = max((r_min or 0) * 0.6, (req.spacing_m or 2.0) * 1.5, 4.0)
            spts = _apply_endpoint_headings(pts, hs, hg, lead)
        curve, used_s, warn, measured_r, _ = fit_spline_curvature_limited(
            spts, r_min, kappa_rate_max=kappa_rate_max, n=req.samples
        )
        source = "numeric"

    # 走行可能 mask（EB 洗練・フットプリント包含チェックで共用）。NoGoZone もここで除外。
    dmask, dtransform = _drivable_mask(req.drivable_layer_id)
    if dmask is not None:
        dburn = _rasterize_nogo(req.no_go_polygons, dtransform, dmask.shape)
        if dburn is not None:
            dmask = np.where(dburn, 0, dmask)

    # 4b Elastic Band 洗練（任意）: コリドー中央へ寄せ、車体が収まる余裕を確保。
    eb_applied = False
    if req.refine_elastic_band and dmask is not None and len(curve) >= 3:
        half = 0.0
        if veh is not None:
            half = max(half, veh.overall_width / 2.0)
        if req.corridor_width_m and req.corridor_width_m > 0:
            half = max(half, req.corridor_width_m / 2.0)
        desired = (half + 0.3) if half > 0 else 1.5
        node_sp = max(min(req.spacing_m or 2.0, 2.0), 1.0)
        eb_in = resample_by_spacing(curve, node_sp)
        curve = elastic_band(eb_in, dmask, dtransform, desired_clearance=desired)
        eb_applied = True
        # EB で曲率が増え得るため、R_min が分かれば曲率/操舵レート制限で仕上げる。
        if r_min and r_min > 0:
            curve, used_s, w2, measured_r, _ = fit_spline_curvature_limited(
                curve, r_min, kappa_rate_max=kappa_rate_max, n=req.samples
            )
            warn = w2 or warn
            source = "numeric"

    final = resample_by_spacing(curve, req.spacing_m) if req.spacing_m else curve

    # R_min 保証: 違反コーナーだけ局所平滑化して max|κ| <= 1/R_min に。
    # R_min を持たない車種（CD110R スキッド等）は r_min=None なのでスキップ。drivable があれば領域内に留める。
    # hybrid A* は運動学的に R_min を保証済み＆切り返し(cusp)を持ち得るため、曲率制限パスは適用しない
    # （cusp を平滑化で潰すと切り返しが壊れる）。spline/dubins/grid の非運動学経路にのみ適用。
    if req.enforce_min_radius and r_min and r_min > 0 and len(final) >= 3 and algo not in ("hybrid_astar", "reeds_shepp", "rrt_star"):
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

    # 縦断勾配（DSM があれば）。失敗は黙って握り潰さず warning に出す（DSM不良とDSM無しを区別）。
    grade = None
    cl = _cost_layer_for(req.costmap_layer_id, req.drivable_layer_id)
    if cl and cl.get("dsm_cog"):
        try:
            dsm, dsm_t = _read_raster(cl["dsm_cog"])
            s = np.concatenate([[0.0], np.cumsum(np.hypot(np.diff(final[:, 0]), np.diff(final[:, 1])))])
            grade = grade_profile(final, s, dsm, dsm_t)
        except Exception as e:  # noqa: BLE001
            grade = None
            warn = (warn + " / " if warn else "") + f"勾配の計算に失敗（DSM読込/CRS不整合の可能性）: {e}"

    # フットプリント包含チェックは上で解決した走行可能 mask（dmask/dtransform）を共用。
    # 後進つきプランナは生 gear を最終経路へマップ（後進の車体方位・cusp 評価を正しく）。
    gears = _nearest_gears(final, raw_gear_pts) if raw_gear_pts else None
    traj = build_trajectory(final, curvature_source=source, vehicle=veh, grade_pct=grade, gears=gears)
    result = summarize(traj, vehicle=veh, drivable_mask=dmask, transform=dtransform)

    # 安全検証（設計 Stage 5）: 車両包絡線・旋回半径・操舵・勾配・最小離隔を統合し合否＋不可理由を返す。
    clearance_m = None
    if dmask is not None:
        clearance_m, _ = path_min_clearance(final, dmask, dtransform)
    safety = verify_safety(traj, result, veh, clearance_m=clearance_m)

    return {
        "trajectory": traj.model_dump(),
        "analysis": result.model_dump(),
        "safety": safety.model_dump(),
        "used_smoothing_s": used_s,
        "measured_min_radius_m": measured_r if np.isfinite(measured_r) else None,
        "min_clearance_m": clearance_m,
        "warning": warn,
        "refined_elastic_band": eb_applied,
    }


@router.post("/analyze")
def analyze(req: AnalyzeRequest):
    if len(req.points) < 2:
        raise HTTPException(400, "need >= 2 points")
    pts = np.array([[p.x, p.y] for p in req.points], dtype=float)
    veh = _vehicle(req.vehicle_id)
    grade = None
    cl = _cost_layer_for(req.costmap_layer_id, None)
    if cl and cl.get("dsm_cog"):
        try:
            dsm, dsm_t = _read_raster(cl["dsm_cog"])
            s = np.concatenate([[0.0], np.cumsum(np.hypot(np.diff(pts[:, 0]), np.diff(pts[:, 1])))])
            grade = grade_profile(pts, s, dsm, dsm_t)
        except Exception:
            grade = None
    traj = build_trajectory(pts, vehicle=veh, grade_pct=grade)
    result = summarize(traj, vehicle=veh)
    return {"trajectory": traj.model_dump(), "analysis": result.model_dump()}
