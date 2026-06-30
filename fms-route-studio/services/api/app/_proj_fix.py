"""環境の古い PROJ(例: Anaconda の proj.db)を回避し、rasterio 同梱の PROJ データを使う。

重要: rasterio/GDAL を **import する前** に PROJ_DATA を設定しないと、GDAL が先に
悪い proj.db のパスをキャッシュしてしまう。そのため rasterio を import せず
importlib.util.find_spec でパスだけ特定して os.environ を設定する。

app/__init__.py の先頭で（rio_tiler/rasterio の import より前に）呼ぶこと。
"""
from __future__ import annotations

import importlib.util
import os


def apply_proj_fix() -> None:
    spec = importlib.util.find_spec("rasterio")  # rasterio を実行import せずに場所だけ取得
    if spec is None or not spec.origin:
        return
    base = os.path.dirname(spec.origin)
    proj_data = os.path.join(base, "proj_data")
    if os.path.exists(os.path.join(proj_data, "proj.db")):
        os.environ["PROJ_DATA"] = proj_data
        os.environ["PROJ_LIB"] = proj_data
