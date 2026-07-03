"""複数台制御 API（Phase B: 経路の重なり/競合判定）。

複数経路を受け取り、車幅コリドーの重なり（競合区間）を返す。後段で区間Mutex/簡易シミュレーションの
入力になる。半幅は vehicle_id（車幅/2）か half_width_m で指定。
"""
from __future__ import annotations

from fastapi import APIRouter
from pydantic import BaseModel, Field

from planning_core.fleet import (
    SimVehicle,
    auto_bay_center,
    detect_conflicts,
    junction_pose,
    lateral_detour,
    simulate_fleet,
    simulate_fleet_auto,
    simulate_fleet_sequential,
)

from .. import vehicle_overrides

router = APIRouter(prefix="/api/fleet", tags=["fleet"])


class RouteIn(BaseModel):
    name: str | None = None
    points: list[tuple[float, float]] = Field(..., max_length=200_000)  # 経路中心線 [(x,y), ...]（作業CRS・m）
    vehicle_id: str | None = None              # 車幅/2 を半幅に使う
    half_width_m: float | None = None          # 明示半幅[m]（vehicle_id より優先）


class ConflictRequest(BaseModel):
    routes: list[RouteIn] = Field(..., max_length=64)  # 同時判定する経路数の上限
    cell_m: float = 0.5                        # 判定ラスタ解像度[m]
    clearance_m: float = Field(0.0, ge=0.0)    # 車車間の追加余裕[m]（半幅に上乗せ）


def _half_width(r: RouteIn) -> float:
    if r.half_width_m and r.half_width_m > 0:
        return float(r.half_width_m)
    if r.vehicle_id:
        try:
            v = vehicle_overrides.resolve(r.vehicle_id)
            return float(v.overall_width) / 2.0
        except FileNotFoundError:
            pass
    return 1.7  # 既定（HD785相当の半幅）


@router.post("/conflicts")
def conflicts(req: ConflictRequest):
    routes = [{"points": r.points, "half_width": _half_width(r)} for r in req.routes]
    cs = detect_conflicts(routes, cell=req.cell_m, clearance_m=req.clearance_m)
    return {
        "conflicts": [
            {
                "a": c.a, "b": c.b, "kind": c.kind,
                "overlap_area_m2": round(c.overlap_area_m2, 2),
                "a_intervals": [{"s_start": round(i.s_start, 2), "s_end": round(i.s_end, 2)} for i in c.a_intervals],
                "b_intervals": [{"s_start": round(i.s_start, 2), "s_end": round(i.s_end, 2)} for i in c.b_intervals],
                "bbox": {"minx": c.bbox[0], "miny": c.bbox[1], "maxx": c.bbox[2], "maxy": c.bbox[3]},
            }
            for c in cs
        ],
        "n_routes": len(req.routes),
        "n_conflicts": len(cs),
    }


class BayIn(BaseModel):
    s_frac: float = 0.5               # 経路上の待避所中心（弧長の割合 0..1）
    offset_m: float = 6.0             # 横退避量[m]
    side: int = 1                     # +1=左 / -1=右
    ramp_m: float = 8.0               # 出入りの遷移長[m]
    hold_m: float = 12.0              # 退避平坦長[m]


class JunctionRequest(BaseModel):
    points: list[tuple[float, float]] = Field(..., max_length=200_000)  # 親経路中心線
    s_frac: float = 0.5                 # 分岐起点の弧長割合 0..1


@router.post("/junction")
def junction(req: JunctionRequest):
    """親経路上の分岐起点姿勢を返す（heading=接線。分岐をなめらかに出す start に使う）。"""
    x, y, h = junction_pose(req.points, req.s_frac)
    return {"x": x, "y": y, "heading_deg": h}


class SimRouteIn(RouteIn):
    priority: int = 0                 # 小さいほど高優先（競合時に先に通る）
    start_time_s: float = 0.0         # 出発時刻[s]
    v_max_mps: float | None = None    # 最大速度[m/s]（未指定は車両 max_speed_fwd）
    bay: BayIn | None = None          # すれ違い用の横退避（待避所）。譲る側に設定すると対向が捌ける


class SimRequest(BaseModel):
    routes: list[SimRouteIn] = Field(..., max_length=64)
    dt_s: float = Field(0.2, gt=0.0)
    gap_m: float = Field(2.0, ge=0.0)      # 占有区間手前の停止マージン[m]
    clearance_m: float = Field(0.0, ge=0.0)
    max_time_s: float = Field(600.0, gt=0.0)
    auto_passing: bool = False             # 接触時に低優先側へ自動で待避所を入れて解決を試みる
    dispatch: str = "simultaneous"         # "simultaneous"=全車同時 / "sequential"=逐次ローテーション
    loops: int = Field(1, ge=1, le=50)     # sequential 時の周回数（各車が経路をN周）


def _sim_vehicle(r: SimRouteIn) -> SimVehicle:
    v_max, accel, decel, hw, hl = 5.0, 0.5, 1.0, _half_width(r), 3.0
    if r.vehicle_id:
        try:
            v = vehicle_overrides.resolve(r.vehicle_id)
            v_max = float(v.max_speed_fwd or 5.0)
            accel = float(getattr(v, "max_accel", None) or 0.5)
            decel = float(getattr(v, "max_decel", None) or 1.0)
            hl = float(getattr(v, "overall_length", None) or 6.0) / 2.0
        except FileNotFoundError:
            pass
    pts = [(float(x), float(y)) for x, y in r.points]
    if r.bay is not None and len(pts) >= 3:
        import numpy as np

        arr = np.asarray(pts, float)
        det = lateral_detour(arr, s_center=auto_bay_center(arr, r.bay.s_frac),
                             offset=r.bay.offset_m, side=r.bay.side, ramp=r.bay.ramp_m, hold=r.bay.hold_m)
        pts = [(float(x), float(y)) for x, y in det]
    return SimVehicle(
        points=pts,
        v_max=float(r.v_max_mps) if r.v_max_mps else v_max,
        accel=accel, decel=decel, half_width=hw, half_length=hl,
        priority=r.priority, start_time=r.start_time_s, name=r.name or "",
    )


@router.post("/simulate")
def simulate(req: SimRequest):
    """複数台の簡易シミュレーション（区間予約Mutex＋優先度＋規定減速停止）。
    各車の時系列 trace（再生用）とイベント・デッドロック判定を返す。"""
    import numpy as np

    vehicles = [_sim_vehicle(r) for r in req.routes]
    for v in vehicles:
        v.points = np.asarray(v.points, float)
    if req.dispatch == "sequential":
        # 逐次ローテーション（1台ずつ・周回）。同時1台のため gap/clearance/auto_passing は無効。
        res = simulate_fleet_sequential(vehicles, loops=req.loops, dt=req.dt_s, max_time=req.max_time_s)
    else:
        runner = simulate_fleet_auto if req.auto_passing else simulate_fleet
        res = runner(vehicles, dt=req.dt_s, gap_m=req.gap_m,
                     clearance_m=req.clearance_m, max_time=req.max_time_s)
    return {
        "status": res.status,
        "deadlock": res.deadlock,
        "deadlock_time_s": res.deadlock_time,
        "collision": res.collision,
        "min_separation_m": res.min_separation_m,
        "min_sep_time_s": res.min_sep_time,
        "makespan_s": res.makespan,
        "total_wait_s": res.total_wait_s,
        "wait_time_s": res.wait_time_s,
        "travel_time_s": res.travel_time_s,
        "traces": res.traces,
        "events": res.events,
        "auto_bays": res.auto_bays,
        "names": [r.name or f"経路{i + 1}" for i, r in enumerate(req.routes)],
    }
