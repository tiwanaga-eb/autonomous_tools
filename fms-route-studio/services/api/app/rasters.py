"""ラスタ整合ユーティリティ（planning_core.rasters の再エクスポート）。

実装は planning_core 側（オーケストレータが co-registration 検証に使うため）。
API 層からは従来どおり `from ..rasters import same_grid` で参照できる。
"""
from planning_core.rasters import MISREGISTERED_MSG, same_grid

__all__ = ["MISREGISTERED_MSG", "same_grid"]
