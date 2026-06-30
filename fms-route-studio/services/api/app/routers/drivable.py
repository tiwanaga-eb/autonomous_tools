"""走行可能領域 API（設計書 §12）。

cost レイヤの生cost → 閾値/形態学/クリアランス → 走行可能マスク。
人が include/exclude ポリゴンで微修正（非破壊: base + edits）。再生成しても編集は保持。
表示は半透明RGBA(範囲外透明)。version を上げて FE のタイルキャッシュを無効化。
"""
from __future__ import annotations

from typing import Literal

import numpy as np
import rasterio
from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
from rio_tiler.constants import WGS84_CRS
from rio_tiler.io import Reader

from planning_core.drivable import generate_drivable, mask_to_rgba, segment_drivable_base

from .. import store
from ..cogio import write_cog

router = APIRouter(prefix="/api/drivable", tags=["drivable"])


class GenParams(BaseModel):
    threshold: float = 150.0
    close_m: float = 1.0
    open_m: float = 0.5
    min_area_m2: float = 50.0
    clearance_m: float = 0.0
    smooth_m: float = 0.0        # メディアン平滑（境界ノイズ除去）半径[m]
    max_hole_m2: float = 0.0     # この面積以下の閉じ穴を埋める[m²]（0=埋めない）
    keep_largest: bool = False   # 最大の連結領域のみ残す
    method: Literal["threshold", "otsu", "adaptive"] = "threshold"  # 生成方法（otsu/adaptive=OpenCV自動）


class GenRequest(BaseModel):
    cost_layer_id: str
    params: GenParams = GenParams()


class EditRequest(BaseModel):
    op: Literal["include", "exclude"]
    polygon: list[tuple[float, float]]


def _read_cost(cost_layer_id: str):
    cl = store.get_layer(cost_layer_id)
    if not cl or "cost_cog" not in cl:
        raise HTTPException(404, "cost layer (with cost_cog) not found")
    with rasterio.open(cl["cost_cog"]) as ds:
        cost = ds.read(1)
        transform = ds.transform
        epsg = ds.crs.to_epsg() if ds.crs else cl.get("epsg")
    return cost, transform, int(epsg), float(cl.get("obstacle_value", 1e9))


def _build(layer_id: str, cost_layer_id: str, params: dict, edits: list, version: int) -> dict:
    cost, transform, epsg, obstacle = _read_cost(cost_layer_id)
    # 生成方法: threshold（従来）/ otsu・adaptive（OpenCV 自動セグメント＝base_override）。
    method = params.get("method", "threshold")
    base_override = None
    if method in ("otsu", "adaptive"):
        base_override = segment_drivable_base(cost, transform, obstacle_value=obstacle, method=method)
    mask, stats = generate_drivable(
        cost,
        transform,
        threshold=params["threshold"],
        obstacle_value=obstacle,
        close_m=params["close_m"],
        open_m=params["open_m"],
        min_area_m2=params["min_area_m2"],
        clearance_m=params["clearance_m"],
        smooth_m=params.get("smooth_m", 0.0),
        max_hole_m2=params.get("max_hole_m2", 0.0),
        keep_largest=params.get("keep_largest", False),
        edits=edits,
        base_override=base_override,
    )
    d = store.layer_dir(layer_id)
    d.mkdir(parents=True, exist_ok=True)
    disp = str(d / "cog.tif")
    maskp = str(d / "mask.tif")
    write_cog(disp, np.moveaxis(mask_to_rgba(mask), 2, 0), transform, epsg, nodata=None)
    write_cog(maskp, mask.astype("uint8"), transform, epsg, nodata=None)
    with Reader(disp) as r:
        gb = r.get_geographic_bounds(WGS84_CRS)
    meta = {
        "id": layer_id,
        "kind": "drivable",
        "version": version,
        "cog": disp,
        "mask_cog": maskp,
        "epsg": epsg,
        "cost_layer_id": cost_layer_id,
        "params": params,
        "edits": edits,
        "stats": stats,
        "geographic_bounds": [float(v) for v in gb],
    }
    store.add_layer(meta)
    return meta


def _require(layer_id: str) -> dict:
    m = store.get_layer(layer_id)
    if not m or m.get("kind") != "drivable":
        raise HTTPException(404, "drivable layer not found")
    return m


@router.post("")
def create(req: GenRequest):
    layer_id = store.new_layer_id("drivable")
    return _build(layer_id, req.cost_layer_id, req.params.model_dump(), [], 1)


@router.post("/{layer_id}/regenerate")
def regenerate(layer_id: str, params: GenParams):
    m = _require(layer_id)
    return _build(layer_id, m["cost_layer_id"], params.model_dump(), m.get("edits", []), m.get("version", 1) + 1)


@router.patch("/{layer_id}")
def edit(layer_id: str, req: EditRequest):
    m = _require(layer_id)
    edits = list(m.get("edits", [])) + [
        {"op": req.op, "polygon": [[float(p[0]), float(p[1])] for p in req.polygon]}
    ]
    return _build(layer_id, m["cost_layer_id"], m["params"], edits, m.get("version", 1) + 1)


@router.delete("/{layer_id}/edits/{index}")
def delete_edit(layer_id: str, index: int):
    """個別の include/exclude 編集を削除して再生成。"""
    m = _require(layer_id)
    edits = list(m.get("edits", []))
    if not (0 <= index < len(edits)):
        raise HTTPException(404, f"edit index out of range: {index}")
    edits.pop(index)
    return _build(layer_id, m["cost_layer_id"], m["params"], edits, m.get("version", 1) + 1)


@router.delete("/{layer_id}/edits")
def clear_edits(layer_id: str):
    """全編集をクリアして再生成（base のみに戻す）。"""
    m = _require(layer_id)
    return _build(layer_id, m["cost_layer_id"], m["params"], [], m.get("version", 1) + 1)


@router.get("/{layer_id}/analysis")
def analysis(layer_id: str):
    return _require(layer_id)["stats"]
