"""Vehicle profile loading (YAML -> VehicleProfile).

設計書 §8.1: HD785/HD605/HM400/CD110R を選択可能。HD785/HM400 は実値ベース、
HD605/CD110R は暫定(estimated)。configs/*.yaml に kinematic_type 付きで定義。
"""
from __future__ import annotations

from pathlib import Path

import yaml

from ..models.vehicle import VehicleProfile

CONFIG_DIR = Path(__file__).parent / "configs"
BUILTIN_IDS = ["HD785", "HD605", "HM400", "CD110R"]


def load_profile(path: str | Path) -> VehicleProfile:
    data = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    # YAML の footprint_polygon は [[x, y], ...] -> tuple へ正規化
    fp = data.get("footprint_polygon")
    if fp is not None:
        data["footprint_polygon"] = [tuple(pt) for pt in fp]
    return VehicleProfile(**data)


def load_builtin(vehicle_id: str) -> VehicleProfile:
    p = CONFIG_DIR / f"{vehicle_id}.yaml"
    if not p.exists():
        raise FileNotFoundError(f"No builtin vehicle config: {vehicle_id} ({p})")
    return load_profile(p)


def list_builtins() -> list[VehicleProfile]:
    return [load_builtin(v) for v in BUILTIN_IDS]
