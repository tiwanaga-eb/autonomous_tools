"""配列 -> COG 書き出し（単バンド/マルチ/RGBA対応）。costmap・drivable で共用。"""
from __future__ import annotations

import os

import numpy as np
import rasterio
from rasterio.crs import CRS
from rasterio.enums import ColorInterp
from rio_cogeo.cogeo import cog_translate
from rio_cogeo.profiles import cog_profiles


def write_cog(path: str, array: np.ndarray, transform, epsg: int, nodata: float | None = None) -> None:
    data = array[np.newaxis, ...] if array.ndim == 2 else array
    count, h, w = data.shape
    tmp = path + ".tmp.tif"
    # temp は必ず tiled で書く。striped だと GDAL 3.12 で cog_translate の読み戻しが
    # "Strile size != expected size" で失敗するため（特に大きな単バンド配列）。
    profile = {
        "driver": "GTiff",
        "height": h,
        "width": w,
        "count": count,
        "dtype": str(data.dtype),
        "crs": CRS.from_epsg(int(epsg)),
        "transform": transform,
        "tiled": True,
        "blockxsize": 256,
        "blockysize": 256,
    }
    if nodata is not None:
        profile["nodata"] = nodata
    with rasterio.open(tmp, "w", **profile) as dst:
        for i in range(count):
            dst.write(data[i], i + 1)
        if count == 4:
            dst.colorinterp = [
                ColorInterp.red,
                ColorInterp.green,
                ColorInterp.blue,
                ColorInterp.alpha,
            ]
    cog_translate(tmp, path, cog_profiles.get("deflate"), quiet=True, add_mask=False)
    os.remove(tmp)
