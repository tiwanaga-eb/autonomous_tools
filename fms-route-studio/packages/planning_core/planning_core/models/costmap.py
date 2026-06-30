"""Cost-map domain types.

設計書 §8/§11/§21(B1): 生スカラー cost(float32, 単バンド) を「正」とする。
RGB は表示専用(colorize) で走行判定には使わない。DSM は同時生成する。
"""
from __future__ import annotations

from pydantic import BaseModel


class CostmapRef(BaseModel):
    """下流(plan/drivable)が参照する軽量ハンドル。version でレイヤ更新を検知。"""

    layer_id: str
    version: int


class CostmapParams(BaseModel):
    """生成パラメータ。物理スケールで安定化（slope[deg]/slope_limit、セル内高低差）。"""

    grid_size_m: float = 0.3
    w_slope: float = 500.0                 # 傾斜コストの重み
    w_rough: float = 100.0                 # 粗さ/植生/段差コストの重み
    rough_window_m: float = 1.0            # 粗さ評価窓[m]（DTM局所標準偏差）
    canopy_ref_m: float = 1.0              # 粗さ参照[m]（この高低差で粗さコスト最大）
    slope_limit_deg: float | None = 15.0   # 正規化基準。Noneでない場合これ超で不可侵
    obstacle_value: float = 1e9            # 不可侵の番兵値
    display_vmax: float | None = None      # 表示RGBの上限cost（None=自動: 有効costのp98）
    ground_percentile: float = 5.0         # 地表推定の分位点[%]（min-Zの外れ点対策。0で従来のmin）
    min_points_per_cell: int = 1           # この点数未満のセルは不信頼→補間対象（疎点群の偽地表を抑制）


class CostmapResult(BaseModel):
    """生成物（永続化される参照）。"""

    ref: CostmapRef
    cost_cog: str                          # 生スカラー cost (float32, 単バンド, canonical)
    rgb_cog: str                           # 表示専用 RGB(colorize)
    dsm_cog: str | None = None             # 同時生成した標高 DSM (float32, canonical)
    cost_unit: str = "weighted(slope_n*w_slope + rough_n*w_rough)"
    nodata: float = 1e9
    obstacle_value: float = 1e9
