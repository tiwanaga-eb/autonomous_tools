"""Coordinate transforms — single source for canonical/working/WGS84 conversions.

設計書 §8.2 / §19: 変換をここに一元化し、CRS取り違えを型と単体テストで防ぐ。
由来: map_tool.py:159-169 (proj_xy / map_to_lonlat)。
"""
from __future__ import annotations

from functools import lru_cache

import numpy as np
import pyproj

WGS84 = 4326


@lru_cache(maxsize=128)
def _transformer(src_epsg: int, dst_epsg: int) -> pyproj.Transformer:
    return pyproj.Transformer.from_crs(
        f"epsg:{src_epsg}", f"epsg:{dst_epsg}", always_xy=True
    )


def project(xy, src_epsg: int, dst_epsg: int) -> np.ndarray:
    """Reproject an (N, 2) array of points from src_epsg to dst_epsg (x/y, lon/lat order)."""
    arr = np.asarray(xy, dtype=float)
    single = arr.ndim == 1
    if single:
        arr = arr.reshape(1, 2)
    if int(src_epsg) == int(dst_epsg):
        out = arr.copy()
    else:
        tr = _transformer(int(src_epsg), int(dst_epsg))
        X, Y = tr.transform(arr[:, 0], arr[:, 1])
        out = np.column_stack([np.asarray(X, float), np.asarray(Y, float)])
    return out[0] if single else out


def to_lonlat(xy, epsg_in: int) -> np.ndarray:
    """working/canonical -> WGS84 lon/lat."""
    return project(xy, epsg_in, WGS84)


def from_lonlat(lonlat, epsg_out: int) -> np.ndarray:
    """WGS84 lon/lat -> working/canonical."""
    return project(lonlat, WGS84, epsg_out)
