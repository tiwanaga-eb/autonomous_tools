"""動的パスシミュレータ API（寄り付き / 切り返し回数選択, 設計書 §13）。

ターゲット姿勢＋進入元＋切り返し回数(0/1)から寄り付き経路を計画し、軌跡・切り返し点・
メトリクス（所要時間/長さ/最小クリアランス/寄り付き誤差）を返す。drivable レイヤがあれば
その mask でクリアランスを評価する。
"""
from __future__ import annotations

import math
from typing import Literal

import numpy as np
import rasterio
from fastapi import APIRouter, HTTPException
from pydantic import BaseModel, Field

# 寄り付き候補生成アルゴリズム。auto=全手法をコスト比較 / 個別手法。
SpottingMethod = Literal["auto", "dubins", "reeds_shepp", "hybrid_astar"]

from planning_core.orchestrator import analyze_polyline
from planning_core.simulator import CostWeights, plan_spotting

from .. import store
from .. import vehicle_overrides
from ..rasters import same_grid

router = APIRouter(prefix="/api/simulate", tags=["simulate"])


class PoseIn(BaseModel):
    x: float
    y: float
    heading_deg: float = 0.0


class WeightsIn(BaseModel):
    w_distance: float = 1.0
    w_time: float = 0.0
    w_reverse: float = 1.0
    w_switchback: float = 8.0
    w_costmap: float = 2.0
    w_turn: float = 6.0          # 総旋回量ペナルティ（蛇行抑制）
    w_clearance: float = 0.0     # 境界余裕への報酬（大=エリア中央寄りで頑健）
    w_footprint: float = 200.0   # 車体はみ出し率ペナルティ（best-effortで最小違反を選ぶ）


class SpottingRequest(BaseModel):
    start: PoseIn
    target: PoseIn
    max_switchbacks: int | None = 1          # 0=前進のみ / 1=1切り返し / None=任意(当面1)
    require_switchback: bool = False          # True=切り返し必須（後進で差し込む）
    min_turn_radius_m: float | None = None
    vehicle_id: str | None = None
    drivable_layer_id: str | None = None     # クリアランス評価用 mask
    costmap_layer_id: str | None = None      # コスト関数用 cost マップ（未指定なら drivable 経由で解決）
    use_footprint: bool = True               # 車両 footprint_radius を要求クリアランスに
    road_width_m: float | None = None        # 道幅[m]（>0 で要求クリアランス=道幅/2、footprint_radius を上書き）
    no_go_polygons: list[list[tuple[float, float]]] | None = Field(None, max_length=128)  # 進入禁止領域（world座標多角形）
    with_exit: bool = False                  # 退出軌道も生成する（既定の行先は start）
    exit_goal: PoseIn | None = None          # 退出の行先 Goal 姿勢（指定時 target→exit_goal。未指定は target→start）
    manual_switch_pose: PoseIn | None = None  # 手動切り返し点（指定時は前進→S→後進のみ生成）
    switchback_zone: list[tuple[float, float]] | None = None  # 切り返し可能エリア（cuspをこの中に制約）
    containment_polygon: list[tuple[float, float]] | None = None  # 走行を収めるエリア。経路＋車体がこの多角形内に収まるよう制約（外は走行不可化）
    cusp_margin_m: float | None = None        # 切り返し点の直線マージン[m]（ステア0°で反転・追従可能に。None=自動）
    method: SpottingMethod = "auto"           # 候補生成アルゴリズム（auto/dubins/reeds_shepp/hybrid_astar）
    footprint_ignore_ends_m: float | None = None  # 端点(固定姿勢)のフットプリント判定除外[m]。None=車長×0.6で自動
    smooth_path: bool = True                   # 経路平滑化（曲率不連続=速度低下/蛇行を抑制）
    allow_stationary_steer: bool | None = None  # 据え切り(出発/到着の端点その場操舵)の可否。None=車種既定(履帯=可,ホイール=不可)
    min_speed_kmh: float = 0.0                  # 最低速度[km/h]（出発/到着/切返付近以外で下限。0=無効）
    weights: WeightsIn = WeightsIn()


def _vehicle(vid: str | None):
    if not vid:
        return None
    try:
        return vehicle_overrides.resolve(vid)  # 既定＋車種別オーバーライド（チューニング値）
    except FileNotFoundError:
        raise HTTPException(404, f"unknown vehicle: {vid}")


@router.post("/spotting")
def spotting(req: SpottingRequest):
    veh = _vehicle(req.vehicle_id)
    rho = req.min_turn_radius_m
    if (not rho or rho <= 0) and veh is not None:
        if veh.kinematic_type == "tracked_skid":
            # スキッドステアは小回り可（R_min≈0）。Dubins/後進は rho>0 が要るので小さめの値を使う。
            rho = max(veh.min_turning_radius or 0.0, 3.0)
        else:
            rho = veh.min_turning_radius
    if not rho or rho <= 0:
        raise HTTPException(400, "min_turn_radius が必要です（vehicle_id か min_turn_radius_m を指定）")

    import rasterio.features as rfeat
    from rasterio.transform import from_origin

    mask = None
    transform = None
    cl = None
    if req.drivable_layer_id:
        dl = store.get_layer(req.drivable_layer_id)
        if dl and "mask_cog" in dl:
            with rasterio.open(dl["mask_cog"]) as ds:
                mask = ds.read(1)
                transform = ds.transform
            if dl.get("cost_layer_id"):
                cl = store.get_layer(dl["cost_layer_id"])
    if req.costmap_layer_id:
        cl = store.get_layer(req.costmap_layer_id)

    # コスト関数用のコストマップ（あれば）。封じ込めの基準グリッドを cost に合わせるため先に読む。
    cost = None
    obstacle = 1e9
    ctransform = None
    if cl and cl.get("cost_cog"):
        with rasterio.open(cl["cost_cog"]) as ds:
            cost = ds.read(1)
            ctransform = ds.transform
        obstacle = float(cl.get("obstacle_value", 1e9))
        if transform is None:
            transform = ctransform

    has_contain = bool(req.containment_polygon and len(req.containment_polygon) >= 3)
    # 走行可能レイヤが無く封じ込めエリアが指定された場合: cost があればその**同一グリッド**上に
    # 「全面走行可」マスクを用意（後段の containment が多角形内へ絞る）。cost と整合し cost も活かせる。
    if has_contain and mask is None and cost is not None:
        mask = np.ones(cost.shape, dtype="uint8")
        transform = ctransform

    # 進入禁止領域(NoGoZone): mask からカーブアウト（寄り付き経路が回避/不可判定）。
    if mask is not None and transform is not None and req.no_go_polygons:
        geoms = []
        for ring in req.no_go_polygons:
            if ring and len(ring) >= 3:
                cs = [[float(p[0]), float(p[1])] for p in ring]
                cs.append(cs[0])
                geoms.append(({"type": "Polygon", "coordinates": [cs]}, 1))
        if geoms:
            burn = rfeat.rasterize(geoms, out_shape=mask.shape, transform=transform, fill=0, dtype="uint8").astype(bool)
            mask = np.where(burn, 0, mask)

    # 走行を収めるエリア(containment): 経路＋車体がこの多角形**内**に収まるよう、外側を走行不可化する。
    # mask があれば AND（drivable/cost グリッド上で多角形内のみ可）。無ければ多角形からローカルラスタを
    # 生成（cost も drivable も無い場合のみ）。
    if has_contain:
        ring = [[float(p[0]), float(p[1])] for p in req.containment_polygon]
        ring.append(ring[0])
        geom = {"type": "Polygon", "coordinates": [ring]}
        if mask is not None and transform is not None:
            keep = rfeat.rasterize([(geom, 1)], out_shape=mask.shape, transform=transform,
                                   fill=0, dtype="uint8").astype(bool)
            mask = np.where(keep, mask, 0)  # containment 外は走行不可に
        else:
            xs = [p[0] for p in req.containment_polygon]
            ys = [p[1] for p in req.containment_polygon]
            pad = float(veh.overall_length) * 1.5 if veh is not None else 12.0
            cellc = 0.5
            minx, maxx = min(xs) - pad, max(xs) + pad
            miny, maxy = min(ys) - pad, max(ys) + pad
            w = max(2, int(math.ceil((maxx - minx) / cellc)))
            h = max(2, int(math.ceil((maxy - miny) / cellc)))
            transform = from_origin(minx, maxy, cellc, cellc)  # north-up
            mask = rfeat.rasterize([(geom, 1)], out_shape=(h, w), transform=transform, fill=0, dtype="uint8")

    # 防御: mask と cost は同一グリッド前提（hybrid A* 等が同じ transform で両者を参照する）。
    # shape に加え transform も照合（同サイズでも別ゾーン/別範囲なら誤参照になる）。
    # 食い違う場合は cost を無効化して安全側で継続（封じ込め・mask 制約は維持）。
    if mask is not None and cost is not None and not same_grid(transform, mask.shape, ctransform, cost.shape):
        cost = None
        obstacle = 1e9

    # 縦断勾配 解析用 DSM（あれば）
    dsm = dsm_t = None
    if cl and cl.get("dsm_cog"):
        try:
            with rasterio.open(cl["dsm_cog"]) as ds:
                dsm = ds.read(1)
                dsm_t = ds.transform
        except Exception:  # noqa: BLE001
            dsm = dsm_t = None

    # 要求クリアランス: 道幅指定(>0)なら 道幅/2、無ければ車両 footprint_radius。
    footprint = 0.0
    if req.road_width_m and req.road_width_m > 0:
        footprint = float(req.road_width_m) / 2.0
    elif req.use_footprint and veh is not None:
        footprint = float(veh.footprint_radius or (veh.overall_width / 2.0))

    speed_fwd = float(veh.max_speed_fwd or 3.0) * 0.3 if veh else 3.0  # 寄り付きは低速
    speed_rev = float(veh.max_speed_rev or 1.5) * 0.5 if veh else 1.5

    common = dict(
        rho=rho, max_switchbacks=req.max_switchbacks, drivable_mask=mask, transform=transform,
        cost=cost, obstacle_value=obstacle, footprint_radius=footprint,
        speed_fwd=speed_fwd, speed_rev=speed_rev, weights=CostWeights(**req.weights.model_dump()),
        cusp_margin_m=req.cusp_margin_m,
        # 向き付き車体フットプリントでエリア包含をハード制約に（切り返し点含め車体がはみ出さない）。
        # use_footprint=False のときは円近似クリアランスのまま（後方互換）。
        vehicle=(veh if req.use_footprint else None),
        # 端点除外[m]: 未指定なら車長×0.6で自動。固定の始点/終点姿勢で車体が境界に少しかかるのは
        # 計画では動かせない既定姿勢に由来し経路の不備ではないため除外（cusp など中間は厳格に判定）。
        footprint_ignore_ends_m=(
            req.footprint_ignore_ends_m if req.footprint_ignore_ends_m is not None
            else (0.6 * float(veh.overall_length) if (veh is not None and req.use_footprint) else 0.0)
        ),
        method=req.method,
        smooth_path=req.smooth_path,
        allow_stationary_steer=req.allow_stationary_steer,
    )
    start_pose = (req.start.x, req.start.y, math.radians(req.start.heading_deg))
    target_pose = (req.target.x, req.target.y, math.radians(req.target.heading_deg))

    manual_pose = None
    if req.manual_switch_pose is not None:
        m = req.manual_switch_pose
        manual_pose = (m.x, m.y, math.radians(m.heading_deg))
    zone = [(float(p[0]), float(p[1])) for p in req.switchback_zone] if req.switchback_zone else None

    res = plan_spotting(
        start_pose, target_pose, require_switchback=req.require_switchback,
        manual_switch_pose=manual_pose, switchback_zone=zone, **common,
    )
    out = _result_dict(res, rho, footprint)
    out["method"] = req.method
    min_speed_mps = max(0.0, float(req.min_speed_kmh)) / 3.6
    _attach_analysis(out, res, veh, dsm, dsm_t, mask, transform, min_speed_mps)  # 経路と同じ軌跡解析＋安全検証

    # 退出軌道: ターゲット姿勢から行先(exit_goal、未指定なら start)へ離脱する経路を同条件で計画。
    if req.with_exit:
        if req.exit_goal is not None:
            eg = req.exit_goal
            exit_dest = (eg.x, eg.y, math.radians(eg.heading_deg))
        else:
            exit_dest = start_pose
        exit_res = plan_spotting(target_pose, exit_dest, require_switchback=False, **common)
        out["exit"] = _result_dict(exit_res, rho, footprint)
        _attach_analysis(out["exit"], exit_res, veh, dsm, dsm_t, mask, transform, min_speed_mps)
    return out


def _attach_analysis(out: dict, res, veh, dsm, dsm_t, dmask, dtransform, min_speed_mps: float = 0.0) -> None:
    """寄り付き経路にも経路と同じ軌跡解析(曲率/最小半径/操舵/勾配/標高/速度)＋安全検証を付与する。

    共通後処理は orchestrator.analyze_polyline（/plan と同一実装）を共用。寄り付きは前後進(cusp)を
    含むため gear を渡して速度プロファイルを切り返しで停止させ、cusp は曲率/操舵評価から自動除外
    される。低速マニューバのため dκ/ds(操舵レート)は参考扱い（合否に効かせない）。
    一発到達精度（P-008: 水平±0.5m・方位±5°）は合否チェックに含める。
    """
    pts = res.points
    if not pts or len(pts) < 2:
        return
    xy = np.array([[p["x"], p["y"]] for p in pts], float)
    gears = [p.get("gear") for p in pts]
    err_m = getattr(res, "approach_error_m", None)
    traj, result, safety, _clearance, _warn = analyze_polyline(
        xy, vehicle=veh, gears=gears,
        dsm=dsm, dsm_transform=dsm_t,
        drivable_mask=dmask, drivable_transform=dtransform,
        min_speed_mps=min_speed_mps, advisory_kinds=("kappa_rate",),
        approach_error_m=(err_m if err_m is not None and math.isfinite(err_m) else None),
        approach_error_deg=getattr(res, "approach_error_deg", None),
    )
    out["trajectory"] = traj.model_dump()
    out["analysis"] = result.model_dump()
    out["safety"] = safety.model_dump()


def _reason(res, footprint: float) -> str | None:
    """不可理由（日本語）。feasible かつ到達誤差小なら None。"""
    if res.status == "NO_ZONE_PATH":
        return "指定した切り返し可能エリア内に切り返し点を置けません。エリアを広げる/位置を見直すか、手動切り返し点を指定してください。"
    if res.status == "NO_PATH":
        return "経路が見つかりません。最小旋回半径・姿勢・切り返し許可を見直してください。"
    if res.status == "NO_HYBRID_MAP":
        return "Hybrid A* には走行可能領域かコストマップが必要です。レイヤを指定するか、アルゴリズムを「auto」に切り替えてください。"
    if res.status == "METHOD_NO_PATH":
        return "選択した寄り付きアルゴリズムでは条件を満たす経路が出ませんでした。「auto」に切り替えるか、切り返し必須/最小旋回半径/姿勢を見直してください。"
    if res.status == "COLLISION":
        mc = round(res.min_clearance_m, 2) if res.min_clearance_m is not None else "?"
        return f"クリアランス不足（最小 {mc}m < 要求 {round(footprint, 2)}m）。道幅を狭める/目標姿勢・エリアを見直してください。"
    if res.status == "FOOTPRINT_OUTSIDE":
        ws = f"（弧長 {round(res.fp_worst_s, 1)}m 付近）" if res.fp_worst_s is not None else ""
        return (f"車体が走行可能エリアをはみ出します{ws}。切り返し点を含め全姿勢で車体を収められませんでした。"
                "目標姿勢/最小旋回半径/エリア境界を見直すか、切り返し許可・道幅を調整してください。")
    if res.approach_error_m is not None and res.approach_error_m > 0.8:
        return f"目標姿勢に到達できません（寄り付き誤差 {round(res.approach_error_m, 2)}m）。切り返し許可/最小旋回半径/姿勢を見直してください。"
    if not res.feasible:
        return f"実現困難（{res.status}）。"
    return None


def _fin(v, ndigits: int):
    """inf/nan を JSON 安全に None へ（NO_PATH 等で score=inf になるのを防ぐ）。"""
    if v is None or not math.isfinite(v):
        return None
    return round(v, ndigits)


def _result_dict(res, rho: float, footprint: float) -> dict:
    return {
        "points": res.points,
        "switch_points": [{"x": x, "y": y} for x, y in res.switch_points],
        "metrics": {
            "length_total_m": _fin(res.length_total, 2),
            "length_fwd_m": _fin(res.length_fwd, 2),
            "length_rev_m": _fin(res.length_rev, 2),
            "time_total_s": _fin(res.time_total, 1),
            "n_switchbacks": res.n_switchbacks,
            "min_clearance_m": _fin(res.min_clearance_m, 2),
            "approach_error_m": _fin(res.approach_error_m, 3),
            "approach_error_deg": _fin(getattr(res, "approach_error_deg", None), 2),
            "cost_integral": _fin(res.cost_integral, 2),
            "score": _fin(res.score, 2),
            "footprint_inside": res.footprint_inside,
            "footprint_max_overhang_frac": _fin(res.fp_max_frac, 3),
        },
        "allow_stationary": getattr(res, "allow_stationary", True),
        "endpoint_margin_start_m": _fin(getattr(res, "endpoint_margin_start_m", 0.0), 2),
        "endpoint_margin_goal_m": _fin(getattr(res, "endpoint_margin_goal_m", 0.0), 2),
        "min_cusp_margin_m": _fin(getattr(res, "min_cusp_margin_m", None), 2),
        "feasible": res.feasible,
        "status": res.status,
        "reason": _reason(res, footprint),
        "rho_m": rho,
    }
