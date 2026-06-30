"""車両パラメータのオーバーライド（車種ごとチューニング, 設計書 §8.1）。

実測YAML(planning_core/vehicle/configs)は「既定」として温存し、ユーザーが車種ごとに
入力した値を可逆オーバーライドとして JSON 保存する。経路生成・解析・寄り付き・フットプリントは
`resolve(vid)`（既定＋オーバーライド）を使うことで、チューニング値が即反映される。

寸法(overall_length/width)を変更し footprint_polygon を明示変更していない場合は、
新寸法から矩形フットプリントを再計算する（包含チェック/EB/3D表示と整合）。
"""
from __future__ import annotations

import json
import os
import threading

from planning_core.models.vehicle import VehicleProfile
from planning_core.vehicle import load_builtin

from .settings import VEHICLE_OVERRIDES_PATH

# read-modify-write をシリアライズ（並行編集での消失を防ぐ）。
_LOCK = threading.RLock()

# 編集可能フィールド（数値）。kinematic_type/id/name や spec_status は変更させない。
EDITABLE_FIELDS: list[str] = [
    "overall_length", "overall_width", "overall_height",
    "min_turning_radius", "kappa_max_fwd", "kappa_max_rev", "kappa_rate_max",
    "max_steer_angle", "max_steer_rate", "wheel_base",
    "max_articulation_angle", "track_width",
    "footprint_radius", "road_width",
    "max_speed_fwd", "max_speed_rev",
    "max_grade_pct", "min_clearance_m",
    "max_lateral_accel", "max_accel", "max_decel",
    # 詳細運動特性（HM400 実測表など）。steer_rate_profile は配列のため数値オーバーライド対象外。
    "accel_start", "decel_emergency", "lateral_accel_limit",
    "max_speed_downhill_empty", "max_speed_downhill_loaded",
]


def _load() -> dict:
    if VEHICLE_OVERRIDES_PATH.exists():
        return json.loads(VEHICLE_OVERRIDES_PATH.read_text(encoding="utf-8"))
    return {}


def _save(data: dict) -> None:
    VEHICLE_OVERRIDES_PATH.parent.mkdir(parents=True, exist_ok=True)
    tmp = VEHICLE_OVERRIDES_PATH.with_suffix(".json.tmp")
    tmp.write_text(json.dumps(data, indent=2, ensure_ascii=False), encoding="utf-8")
    os.replace(tmp, VEHICLE_OVERRIDES_PATH)


def get_override(vid: str) -> dict:
    return dict(_load().get(vid, {}))


def all_overrides() -> dict:
    return _load()


def _apply(base: VehicleProfile, ov: dict) -> VehicleProfile:
    """既定プロファイルにオーバーライドを適用して再検証した VehicleProfile を返す。"""
    data = base.model_dump()
    for k, v in ov.items():
        if k in EDITABLE_FIELDS and v is not None:
            data[k] = float(v) if not isinstance(v, bool) else v
    # 寸法変更時、footprint_polygon を明示変更していなければ新寸法から矩形を再計算
    if ("overall_length" in ov or "overall_width" in ov) and "footprint_polygon" not in ov:
        hl = float(data["overall_length"]) / 2.0
        hw = float(data["overall_width"]) / 2.0
        data["footprint_polygon"] = [(hl, hw), (hl, -hw), (-hl, -hw), (-hl, hw)]
    if data.get("footprint_polygon"):
        data["footprint_polygon"] = [tuple(p) for p in data["footprint_polygon"]]
    return VehicleProfile(**data)


def resolve(vid: str) -> VehicleProfile:
    """既定＋オーバーライドを適用した車両プロファイル。"""
    base = load_builtin(vid)
    ov = get_override(vid)
    if not ov:
        return base
    return _apply(base, ov)


def set_override(vid: str, fields: dict) -> VehicleProfile:
    """部分フィールドをマージして保存（検証してから）。保存後の resolved を返す。"""
    base = load_builtin(vid)  # 存在しなければ FileNotFoundError
    clean = {k: v for k, v in fields.items() if k in EDITABLE_FIELDS and v is not None}
    merged = {**get_override(vid), **clean}
    resolved = _apply(base, merged)  # 不正な組合せはここで ValidationError
    with _LOCK:
        data = _load()
        data[vid] = merged
        _save(data)
    return resolved


def reset_override(vid: str) -> VehicleProfile:
    with _LOCK:
        data = _load()
        if vid in data:
            del data[vid]
            _save(data)
    return load_builtin(vid)
