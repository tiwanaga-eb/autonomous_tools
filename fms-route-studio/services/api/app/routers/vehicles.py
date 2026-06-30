"""車両プロファイル API（設計書 §8.1）。HD785/HD605/HM400/CD110R を選択可能に。

車種ごとのパラメータチューニングをサポート: 実測YAMLは「既定」として温存し、ユーザー入力は
可逆オーバーライド(`app.vehicle_overrides`)として保存。一覧/詳細は既定＋オーバーライドを適用した
有効値を返す。経路生成・解析・寄り付きも resolve() を使う（planning/simulate 側）。
"""
from __future__ import annotations

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel, ValidationError

from planning_core.vehicle import BUILTIN_IDS, load_builtin

from .. import vehicle_overrides as vov

router = APIRouter(prefix="/api/vehicles", tags=["vehicles"])


class OverrideIn(BaseModel):
    fields: dict[str, float]


def _summary(v, overridden: bool) -> dict:
    return {
        "id": v.id,
        "name": v.name,
        "kinematic_type": v.kinematic_type,
        "min_turning_radius": v.min_turning_radius,
        "overall_width": v.overall_width,
        "overall_length": v.overall_length,
        "road_width": v.road_width,
        "spec_status": v.spec_status,
        "overridden": overridden,
    }


@router.get("")
def list_vehicles():
    overs = vov.all_overrides()
    return [_summary(vov.resolve(vid), bool(overs.get(vid))) for vid in BUILTIN_IDS]


@router.get("/{vehicle_id}")
def get_vehicle(vehicle_id: str):
    try:
        return vov.resolve(vehicle_id).model_dump()
    except FileNotFoundError:
        raise HTTPException(404, f"unknown vehicle: {vehicle_id}")


@router.get("/{vehicle_id}/detail")
def vehicle_detail(vehicle_id: str):
    """編集UI用: 有効値(effective)・既定(default)・オーバーライド(override)・編集可能フィールド。"""
    try:
        default = load_builtin(vehicle_id)
    except FileNotFoundError:
        raise HTTPException(404, f"unknown vehicle: {vehicle_id}")
    return {
        "effective": vov.resolve(vehicle_id).model_dump(),
        "default": default.model_dump(),
        "override": vov.get_override(vehicle_id),
        "editable_fields": vov.EDITABLE_FIELDS,
    }


@router.put("/{vehicle_id}")
def put_override(vehicle_id: str, body: OverrideIn):
    try:
        resolved = vov.set_override(vehicle_id, body.fields)
    except FileNotFoundError:
        raise HTTPException(404, f"unknown vehicle: {vehicle_id}")
    except ValidationError as e:
        raise HTTPException(400, f"パラメータが車種の運動学と矛盾します: {e.errors()[0].get('msg', str(e))}")
    return {"effective": resolved.model_dump(), "override": vov.get_override(vehicle_id)}


@router.delete("/{vehicle_id}/override")
def delete_override(vehicle_id: str):
    try:
        default = vov.reset_override(vehicle_id)
    except FileNotFoundError:
        raise HTTPException(404, f"unknown vehicle: {vehicle_id}")
    return {"effective": default.model_dump(), "override": {}}
