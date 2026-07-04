"""排土（パイル）配置 API（planning_core.earthworks への薄いラッパ）。

エリア多角形の中に円錐パイル（体積 or 高さ＋安息角）を格子配置する計画を返す。
- 間隔指定: dx_m / dy_m（＋stagger 千鳥）
- 撒き出し計算: spread_thickness_m（撒き出し厚）→ 推奨間隔 √(V/t)・理論数 ⌊A·t/V⌋
"""
from __future__ import annotations

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel, Field

from planning_core.earthworks import plan_piles

router = APIRouter(prefix="/api/earthworks", tags=["earthworks"])


class PilePlanRequest(BaseModel):
    polygon: list[tuple[float, float]] = Field(..., min_length=3, max_length=10_000)  # world座標 [m]
    repose_deg: float = Field(35.0, gt=0.0, lt=89.0)   # 安息角[°]
    volume_m3: float | None = Field(None, gt=0.0)      # パイル体積（volume か height の一方）
    height_m: float | None = Field(None, gt=0.0)       # パイル高さ
    dx_m: float | None = Field(None, gt=0.0)           # 配置間隔（横）
    dy_m: float | None = Field(None, gt=0.0)           # 配置間隔（縦）省略時 dx と同じ
    spread_thickness_m: float | None = Field(None, gt=0.0)  # 撒き出し厚 t（指定時は間隔を自動計算）
    stagger: bool = False                              # 千鳥配置
    stagger_invert: bool = False                       # 千鳥のオフセット行を逆に（斜め方向を反転）
    edge_margin_m: float | None = Field(None, ge=0.0)  # 縁マージン（None=パイル基部半径）


@router.post("/piles")
def piles(req: PilePlanRequest):
    try:
        return plan_piles(
            req.polygon,
            repose_deg=req.repose_deg,
            volume_m3=req.volume_m3,
            height_m=req.height_m,
            dx=req.dx_m,
            dy=req.dy_m,
            spread_thickness_m=req.spread_thickness_m,
            stagger=req.stagger,
            stagger_invert=req.stagger_invert,
            edge_margin_m=req.edge_margin_m,
        )
    except ValueError as e:
        raise HTTPException(400, str(e))
