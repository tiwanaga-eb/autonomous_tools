"""経路生成 API（設計書 §9 / §10）。

このルーターは**リクエスト解釈とレイヤ IO（ラスタ読込）だけ**を行い、業務ロジック
（アルゴリズム選択 → 探索 → EB 洗練 → R_min 保証 → 勾配/標高 → 解析 → 安全検証）は
`planning_core.orchestrator.plan_route / analyze_polyline` へ委譲する（HTTP 非依存で
テスト可能・寄り付き simulate とも共通後処理を共有）。
"""
from __future__ import annotations

from typing import Literal

import numpy as np
import rasterio
from fastapi import APIRouter, HTTPException
from pydantic import BaseModel, Field

from planning_core.analysis import elevation_and_grade
from planning_core.orchestrator import PlanError, PlanSpec, analyze_polyline, plan_route

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


def _r_min(req: PlanRequest, veh) -> float | None:
    if req.min_turn_radius_m and req.min_turn_radius_m > 0:
        return req.min_turn_radius_m
    if veh is not None and veh.kinematic_type != "tracked_skid":
        return veh.min_turning_radius
    return None


@router.post("/plan")
def plan(req: PlanRequest):
    if len(req.waypoints) < 2:
        raise HTTPException(400, "need >= 2 waypoints")
    veh = _vehicle(req.vehicle_id)

    # リクエスト内キャッシュ: 同一ラスタ(cost/mask/dsm)を複数箇所で使うため1回だけ開く。
    _rasters: dict[str, tuple[np.ndarray, object]] = {}

    def _rd(path: str):
        if path not in _rasters:
            _rasters[path] = _read_raster(path)
        return _rasters[path]

    # レイヤ meta もリクエスト内で1回だけ解決（レジストリの多重パースを避ける）。
    cl = _cost_layer_for(req.costmap_layer_id, req.drivable_layer_id)
    dl = store.get_layer(req.drivable_layer_id) if req.drivable_layer_id else None

    cost = ct = mask = mt = dsm = dsmt = None
    obstacle = 1e9
    dsm_warn: str | None = None
    if cl and "cost_cog" in cl:
        cost, ct = _rd(cl["cost_cog"])
        obstacle = float(cl.get("obstacle_value", 1e9))
    if dl and "mask_cog" in dl:
        mask, mt = _rd(dl["mask_cog"])
    if cl and cl.get("dsm_cog"):
        try:
            dsm, dsmt = _rd(cl["dsm_cog"])
        except Exception as e:  # noqa: BLE001 — DSM 読込失敗は勾配なしで続行し warning に出す
            dsm = dsmt = None
            dsm_warn = f"勾配/標高の計算に失敗（DSM読込失敗の可能性）: {e}"

    spec = PlanSpec(
        waypoints=np.array([[p.x, p.y] for p in req.waypoints], dtype=float),
        headings_deg=[p.heading_deg for p in req.waypoints],
        mode=req.mode,
        algorithm=req.algorithm,
        r_min=_r_min(req, veh),
        kappa_rate_max=(veh.kappa_rate_max if (veh and req.limit_steer_rate) else None),
        spacing_m=req.spacing_m,
        samples=req.samples,
        planner_cell_m=req.planner_cell_m,
        corridor_width_m=req.corridor_width_m,
        enforce_footprint=req.enforce_footprint,
        enforce_min_radius=req.enforce_min_radius,
        allow_reverse=req.allow_reverse,
        refine_elastic_band=req.refine_elastic_band,
        vehicle=veh,
        cost=cost, cost_transform=ct, obstacle_value=obstacle,
        mask=mask, mask_transform=mt,
        dsm=dsm, dsm_transform=dsmt,
        no_go_polygons=req.no_go_polygons,
    )
    try:
        out = plan_route(spec)
    except PlanError as e:
        raise HTTPException(e.status, e.message)

    warning = out.warning
    if dsm_warn:
        warning = (warning + " / " if warning else "") + dsm_warn
    return {
        "trajectory": out.trajectory.model_dump(),
        "analysis": out.analysis.model_dump(),
        "safety": out.safety.model_dump(),
        "used_smoothing_s": out.used_smoothing_s,
        "measured_min_radius_m": out.measured_min_radius_m,
        "min_clearance_m": out.min_clearance_m,
        "warning": warning,
        "refined_elastic_band": out.refined_elastic_band,
    }


@router.post("/analyze")
def analyze(req: AnalyzeRequest):
    if len(req.points) < 2:
        raise HTTPException(400, "need >= 2 points")
    pts = np.array([[p.x, p.y] for p in req.points], dtype=float)
    veh = _vehicle(req.vehicle_id)
    dsm = dsmt = None
    cl = _cost_layer_for(req.costmap_layer_id, None)
    if cl and cl.get("dsm_cog"):
        try:
            dsm, dsmt = _read_raster(cl["dsm_cog"])
        except rasterio.errors.RasterioIOError:
            dsm = dsmt = None  # DSM 読込失敗は勾配なしで継続（想定外は伝播）
    traj, result, _safety, _clearance, _warn = analyze_polyline(pts, vehicle=veh, dsm=dsm, dsm_transform=dsmt)
    return {"trajectory": traj.model_dump(), "analysis": result.model_dump()}


class ElevationSampleRequest(BaseModel):
    points: list[XYIn]
    costmap_layer_id: str | None = None    # 省略時: DSM を持つ最新の cost レイヤ
    smooth_m: float = Field(8.0, ge=0.0, le=100.0)  # 勾配解析と同じ平滑化窓[m]。0=生サンプル


@router.post("/elevation/sample")
def elevation_sample(req: ElevationSampleRequest):
    """任意の点列（保存済みルート等）に、点群由来 DSM の標高 z[m] を後付けサンプリングする。

    経路生成時の自動埋め込み（/plan・寄り付き）と同じ双一次補間＋平滑化。DSM 範囲外/欠損は null。
    """
    if not req.points:
        raise HTTPException(400, "need >= 1 point")
    if req.costmap_layer_id:
        cl = store.get_layer(req.costmap_layer_id)
        if not cl or cl.get("kind") != "cost":
            raise HTTPException(404, f"cost layer not found: {req.costmap_layer_id}")
    else:
        cands = [l for l in store.list_layers() if l.get("kind") == "cost" and l.get("dsm_cog")]
        cl = cands[-1] if cands else None
    if not cl or not cl.get("dsm_cog"):
        raise HTTPException(404, "点群由来の DSM を持つコストマップレイヤがありません（先に LAS からコストマップを生成してください）")
    xy = np.array([[p.x, p.y] for p in req.points], float)
    dsm, dsm_t = _read_raster(cl["dsm_cog"])
    if len(xy) >= 2:
        s = np.concatenate([[0.0], np.cumsum(np.hypot(np.diff(xy[:, 0]), np.diff(xy[:, 1])))])
    else:
        s = np.zeros(len(xy))
    z, _ = elevation_and_grade(xy, s, dsm, dsm_t, smooth_m=req.smooth_m)
    zlist = [(float(v) if np.isfinite(v) else None) for v in z]
    return {
        "z": zlist,
        "layer_id": cl.get("id"),
        "n": len(zlist),
        "n_missing": sum(1 for v in zlist if v is None),
    }
