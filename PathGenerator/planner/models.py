from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Literal, Optional, Tuple


Polygon = List[Tuple[float, float]]
Gear = Literal["F", "R"]
Status = Literal["OK", "NO_FEASIBLE_PATH", "TIMEOUT", "INVALID_INPUT"]


@dataclass
class Pose:
    x: float
    y: float
    yaw: float


@dataclass
class Tolerances:
    pos: float
    yaw: float


@dataclass
class MapData:
    drivable_polygons: List[Polygon]
    obstacle_polygons: List[Polygon]
    safety_margin: float


@dataclass
class PlannerParams:
    timeout_ms: int
    weights: Dict[str, float] = field(default_factory=dict)
    sampling_params: Dict[str, float] = field(default_factory=dict)
    algorithm: str = "dubins"
    primitives_params: Dict[str, float] = field(default_factory=dict)
    dubins_pp_params: Dict[str, object] = field(default_factory=dict)
    dubins_pp_behavioral_params: Dict[str, float] = field(default_factory=dict)
    yaw_pref: Optional[float] = None
    seed: Optional[int] = None


@dataclass
class PlanRequest:
    vehicle_id: str
    start_pose: Pose
    dock_pose: Pose
    exit_pose: Optional[Pose]
    tolerances: Tolerances
    map: MapData
    planner_params: PlannerParams


@dataclass
class State:
    x: float
    y: float
    yaw: float
    curvature: float
    gear: Gear
    v: float
    t: float
    is_ramp: bool = False


@dataclass
class Segment:
    gear: Gear
    states: List[State]


@dataclass
class PlanMetrics:
    total_length: float
    total_time: float
    forward_length: float
    reverse_length: float
    switch_yaw: float
    dock_error_pos: float
    dock_error_yaw: float
    collision_free: bool
    dynamic_feasible: bool
    max_steering_change: float
    max_steering_rate_required: float
    max_kappa_rate: float
    smoothing_applied: bool
    added_smoothing_length: float
    compute_time_ms: float


@dataclass
class TrajectoryPlan:
    segments: List[Segment]
    switch_pose: Pose
    metrics: PlanMetrics


@dataclass
class PlanResponse:
    status: Status
    plan: Optional[TrajectoryPlan]
    reason: Optional[str] = None
    debug: Optional[Dict[str, float]] = None
