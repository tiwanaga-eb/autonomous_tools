from __future__ import annotations

from typing import Literal

from pydantic import BaseModel

from .geo import XY

Gear = Literal["F", "R"]


class Waypoint(BaseModel):
    id: str
    role: Literal["start", "via", "goal"]
    xy: XY
    heading_deg: float | None = None
    gear: Gear | None = None


class TrajPoint(BaseModel):
    """A single sample of an analyzed trajectory (working CRS, metres)."""

    s: float                       # arc length [m]
    x: float
    y: float
    heading_deg: float             # +East / CCW
    curvature: float               # kappa [1/m]
    curvature_rate: float          # dkappa/ds [1/m^2]
    grade_pct: float | None = None       # longitudinal grade [%] (None if no DSM)
    steer_deg: float | None = None       # rigid_bicycle only
    gear: Gear | None = None
    speed_mps: float | None = None       # 速度プロファイル v(s) [m/s]（車両指定時）
    time_s: float | None = None          # 始点からの所要時間 [s]


class Trajectory(BaseModel):
    points: list[TrajPoint]
    length_m: float
    min_radius_m: float | None = None    # None for tracked_skid (turn-in-place)
    curvature_source: Literal["analytic", "numeric"] = "numeric"
