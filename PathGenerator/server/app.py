from __future__ import annotations

import asyncio
from pathlib import Path
from typing import Any, Dict, List, Optional

from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, Field

from planner.models import MapData, PlanRequest, PlannerParams, Pose, Tolerances
from planner.planner import plan_mining_dock
from planner.vehicle_config import load_vehicle_config


class PoseIn(BaseModel):
    x: float
    y: float
    yaw: float


class TolerancesIn(BaseModel):
    pos: float = 0.1
    yaw: float = 0.0873


class PlannerParamsIn(BaseModel):
    timeout_ms: int = 10000
    tolerances: TolerancesIn = Field(default_factory=TolerancesIn)
    weights: Dict[str, float] = Field(default_factory=dict)
    sampling_params: Dict[str, float] = Field(default_factory=dict)
    algorithm: str = "dubins"
    primitives_params: Dict[str, float] = Field(default_factory=dict)
    dubins_pp_params: Dict[str, Any] = Field(default_factory=dict)
    dubins_pp_behavioral_params: Dict[str, float] = Field(default_factory=dict)
    yaw_pref: Optional[float] = None
    seed: Optional[int] = None


class PlanInput(BaseModel):
    vehicle_id: str
    start_pose: PoseIn
    dock_pose: PoseIn
    exit_pose: Optional[PoseIn] = None
    drivable_polygon: List[List[float]]
    obstacles: List[List[List[float]]] = Field(default_factory=list)
    planner_params: PlannerParamsIn = Field(default_factory=PlannerParamsIn)
    safety_margin: float = 0.3


app = FastAPI(title="Mining Path Planner Web API")

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

STATIC_DIR = Path(__file__).parent / "static"
app.mount("/static", StaticFiles(directory=STATIC_DIR), name="static")


def _to_pose(p: PoseIn) -> Pose:
    return Pose(x=p.x, y=p.y, yaw=p.yaw)


def _normalize_polygon(raw: List[List[float]]) -> List[tuple[float, float]]:
    poly: List[tuple[float, float]] = []
    for point in raw:
        if len(point) != 2:
            raise HTTPException(status_code=422, detail="Polygon point must be [x, y]")
        poly.append((float(point[0]), float(point[1])))
    if len(poly) < 3:
        raise HTTPException(status_code=422, detail="Polygon needs at least 3 points")
    return poly


def _serialize_plan(resp) -> Dict[str, Any]:
    body: Dict[str, Any] = {
        "status": resp.status,
        "reason": resp.reason,
        "debug": resp.debug,
        "segments": [],
        "switch_pose": None,
        "metrics": None,
    }
    if resp.plan is None:
        return body

    body["switch_pose"] = {
        "x": resp.plan.switch_pose.x,
        "y": resp.plan.switch_pose.y,
        "yaw": resp.plan.switch_pose.yaw,
    }
    body["segments"] = [
        {
            "gear": seg.gear,
            "states": [
                {
                    "x": st.x,
                    "y": st.y,
                    "yaw": st.yaw,
                    "curvature": st.curvature,
                    "gear": st.gear,
                    "v": st.v,
                    "t": st.t,
                    "is_ramp": st.is_ramp,
                }
                for st in seg.states
            ],
        }
        for seg in resp.plan.segments
    ]

    m = resp.plan.metrics
    body["metrics"] = {
        "total_length": m.total_length,
        "total_time": m.total_time,
        "forward_length": m.forward_length,
        "reverse_length": m.reverse_length,
        "switch_yaw": m.switch_yaw,
        "dock_error_pos": m.dock_error_pos,
        "dock_error_yaw": m.dock_error_yaw,
        "collision_free": m.collision_free,
        "dynamic_feasible": m.dynamic_feasible,
        "max_steering_change": m.max_steering_change,
        "max_steering_rate_required": m.max_steering_rate_required,
        "max_kappa_rate": m.max_kappa_rate,
        "smoothing_applied": m.smoothing_applied,
        "added_smoothing_length": m.added_smoothing_length,
        "compute_time_ms": m.compute_time_ms,
    }
    return body


@app.get("/")
def index() -> FileResponse:
    return FileResponse(STATIC_DIR / "index.html")


@app.get("/api/vehicles")
def get_vehicles() -> Dict[str, List[str]]:
    cfg_dir = Path("vehicle_configs")
    vehicles = sorted(p.stem for p in cfg_dir.glob("*.yaml"))
    return {"vehicles": vehicles}


@app.post("/api/plan")
async def plan_path(payload: PlanInput) -> Dict[str, Any]:
    if len(payload.drivable_polygon) < 3:
        raise HTTPException(status_code=422, detail="drivable_polygon must have at least 3 points")

    drivable = _normalize_polygon(payload.drivable_polygon)
    obstacles = [_normalize_polygon(poly) for poly in payload.obstacles]

    req = PlanRequest(
        vehicle_id=payload.vehicle_id,
        start_pose=_to_pose(payload.start_pose),
        dock_pose=_to_pose(payload.dock_pose),
        exit_pose=_to_pose(payload.exit_pose) if payload.exit_pose else None,
        tolerances=Tolerances(
            pos=payload.planner_params.tolerances.pos,
            yaw=payload.planner_params.tolerances.yaw,
        ),
        map=MapData(
            drivable_polygons=[drivable],
            obstacle_polygons=obstacles,
            safety_margin=payload.safety_margin,
        ),
        planner_params=PlannerParams(
            timeout_ms=payload.planner_params.timeout_ms,
            weights=payload.planner_params.weights,
            sampling_params=payload.planner_params.sampling_params,
            algorithm=payload.planner_params.algorithm,
            primitives_params=payload.planner_params.primitives_params,
            dubins_pp_params=payload.planner_params.dubins_pp_params,
            dubins_pp_behavioral_params=payload.planner_params.dubins_pp_behavioral_params,
            yaw_pref=payload.planner_params.yaw_pref,
            seed=payload.planner_params.seed,
        ),
    )

    try:
        resp = await asyncio.to_thread(plan_mining_dock, req)
    except FileNotFoundError as e:
        raise HTTPException(status_code=404, detail=str(e)) from e
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e)) from e
    except Exception as e:  # pragma: no cover
        raise HTTPException(status_code=500, detail=f"planner crashed: {e}") from e

    if resp.status == "INVALID_INPUT":
        raise HTTPException(status_code=400, detail=resp.reason or "invalid input")

    body = _serialize_plan(resp)
    try:
        body["road_width"] = load_vehicle_config(payload.vehicle_id).road_width
    except Exception:
        body["road_width"] = 0.0
    return body
