"""Geometry value objects shared across the planning core.

角度規約: heading_deg は +East 起点・CCW（反時計回り）。core 内部計算は rad。
座標規約: working CRS のメートル系を既定とする（変換は geometry/projection.py に集約）。
"""
from __future__ import annotations

from pydantic import BaseModel


class XY(BaseModel):
    """A 2D point in some CRS (interpretation is contextual)."""

    x: float
    y: float


class GridRef(BaseModel):
    """Canonical reference raster grid (transform + CRS + size).

    設計書 §8.2: canonical は「プロジェクトの基準ラスタグリッド」。
    transform は rasterio Affine の (a, b, c, d, e, f)。north-up を不変条件とする(e<0)。
    """

    width: int
    height: int
    transform: tuple[float, float, float, float, float, float]
    epsg: int | None = None

    @property
    def is_north_up(self) -> bool:
        # transform[4] == e (y方向のピクセルサイズ)。north-up なら負。
        return self.transform[4] < 0
