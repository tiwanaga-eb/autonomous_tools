from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Tuple

import yaml


@dataclass
class VehicleConfig:
    wheel_base: float
    max_steer_angle: float
    max_speed_fwd: float
    max_speed_rev: float
    footprint_polygon: List[Tuple[float, float]]
    gear_switch_time_penalty: float
    r_min: float
    kappa_max: float
    kappa_max_fwd: float
    kappa_max_rev: float
    bounding_radius: float
    max_steer_rate: float
    steer_ramp_length: float
    road_width: float
    kappa_rate_max: float
    smoothing_step: float
    enable_smoothing: bool
    overall_length: Optional[float] = None
    overall_width: Optional[float] = None
    overall_height: Optional[float] = None
    reverse_penalty_factor: Optional[float] = None


REQUIRED_KEYS = {
    "wheel_base",
    "max_steer_angle",
    "max_speed_fwd",
    "max_speed_rev",
    "gear_switch_time_penalty",
    "max_steer_rate",
    "steer_ramp_length",
    "road_width",
    "kappa_rate_max",
    "smoothing_step",
}


def load_vehicle_config(vehicle_id: str, base_dir: str = "vehicle_configs") -> VehicleConfig:
    path = Path(base_dir) / f"{vehicle_id}.yaml"
    if not path.exists():
        raise FileNotFoundError(f"vehicle config not found: {path}")

    data = yaml.safe_load(path.read_text())
    missing = REQUIRED_KEYS - set(data.keys())
    if missing:
        raise ValueError(f"missing keys in vehicle config: {sorted(missing)}")
    if "footprint_polygon" not in data and "footprint_radius" not in data:
        raise ValueError("missing keys in vehicle config: one of footprint_polygon or footprint_radius")

    wheel_base = float(data["wheel_base"])
    max_steer = float(data["max_steer_angle"])
    if "min_turning_radius" in data:
        r_min = float(data["min_turning_radius"])
    else:
        if abs(math.tan(max_steer)) < 1e-8:
            raise ValueError("max_steer_angle is too small")
        r_min = wheel_base / math.tan(max_steer)
    kappa_max = 1.0 / r_min
    kappa_max_fwd = float(data.get("kappa_max_fwd", data.get("kappa_max", kappa_max)))
    kappa_max_rev = float(data.get("kappa_max_rev", data.get("kappa_max", kappa_max)))

    if "footprint_polygon" in data:
        footprint = [(float(x), float(y)) for x, y in data["footprint_polygon"]]
    else:
        r = float(data["footprint_radius"])
        footprint = [(r, r), (r, -r), (-r, -r), (-r, r)]

    if "footprint_radius" in data:
        bounding_radius = float(data["footprint_radius"])
    else:
        bounding_radius = max(math.hypot(x, y) for x, y in footprint)

    return VehicleConfig(
        wheel_base=wheel_base,
        max_steer_angle=max_steer,
        max_speed_fwd=float(data["max_speed_fwd"]),
        max_speed_rev=float(data["max_speed_rev"]),
        footprint_polygon=footprint,
        gear_switch_time_penalty=float(data["gear_switch_time_penalty"]),
        r_min=r_min,
        kappa_max=kappa_max,
        kappa_max_fwd=kappa_max_fwd,
        kappa_max_rev=kappa_max_rev,
        bounding_radius=bounding_radius,
        max_steer_rate=float(data["max_steer_rate"]),
        steer_ramp_length=float(data["steer_ramp_length"]),
        road_width=float(data["road_width"]),
        kappa_rate_max=float(data["kappa_rate_max"]),
        smoothing_step=float(data["smoothing_step"]),
        enable_smoothing=bool(data.get("enable_smoothing", True)),
        overall_length=float(data["overall_length"]) if "overall_length" in data else None,
        overall_width=float(data["overall_width"]) if "overall_width" in data else None,
        overall_height=float(data["overall_height"]) if "overall_height" in data else None,
        reverse_penalty_factor=float(data["reverse_penalty_factor"]) if "reverse_penalty_factor" in data else None,
    )
