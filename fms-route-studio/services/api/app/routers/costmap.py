"""コストマップ生成（ツール埋め込み, 設計書 §11 / §21 B1,B3）。

LAS レイヤ → planning_core.costmap.build_costmap_arrays で
  - 生cost(float32, 単バンド)  … 走行可能領域・閾値判定の「正」
  - DSM(float32, 標高)         … 勾配解析の入力
  - 表示RGB(colorize)          … 地図オーバーレイ用
の3 COG を生成・登録し、cost レイヤ(meta に cost_cog/dsm_cog を保持)を返す。
"""
from __future__ import annotations

import os
import tempfile

import numpy as np
import rasterio
from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
from rasterio.crs import CRS
from rasterio.enums import ColorInterp
from rio_cogeo.cogeo import cog_translate
from rio_cogeo.profiles import cog_profiles
from rio_tiler.constants import WGS84_CRS
from rio_tiler.io import Reader

from planning_core.costmap import build_costmap_arrays, cost_to_rgba
from planning_core.geometry import project
from planning_core.io import read_las_xyz
from planning_core.models import CostmapParams

from .. import store
from ..settings import DEFAULT_EPSG

router = APIRouter(prefix="/api/costmap", tags=["costmap"])


class CostmapRequest(BaseModel):
    las_layer_id: str
    params: CostmapParams = CostmapParams()
    src_epsg: int | None = None      # LAS座標のEPSG（None=ヘッダ→無ければ target）
    target_epsg: int = DEFAULT_EPSG


def _write_cog(path: str, array: np.ndarray, transform, epsg: int, nodata: float | None) -> None:
    """単バンド(H,W) または マルチ(C,H,W) 配列を COG として書き出す。"""
    data = array[np.newaxis, ...] if array.ndim == 2 else array
    count, h, w = data.shape
    tmp = path + ".tmp.tif"
    # temp は tiled 必須（striped だと GDAL 3.12 で cog_translate 読み戻しが Strile エラー）。
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
    # add_mask=False: alpha をバンドとして保持（OL の RGBA レンダリング用）
    cog_translate(tmp, path, cog_profiles.get("deflate"), quiet=True, add_mask=False)
    os.remove(tmp)


@router.get("/{layer_id}/dsm_grid")
def dsm_grid(layer_id: str, max_size: int = 160):
    """3D表示用に DSM を粗いグリッド（最大 max_size 角）へダウンサンプルして返す。

    返値: {nx, ny, x0, y0, dx, dy, z(行優先, NaNはnull), zmin, zmax}（座標は layer の CRS, メートル）。
    """
    cl = store.get_layer(layer_id)
    if not cl or not cl.get("dsm_cog"):
        raise HTTPException(404, "DSM を持つ cost レイヤが見つかりません")
    with rasterio.open(cl["dsm_cog"]) as ds:
        h, w = ds.height, ds.width
        step = max(1, int(np.ceil(max(h, w) / max(8, max_size))))
        # 粗い出力サイズ
        oh = max(2, h // step)
        ow = max(2, w // step)
        arr = ds.read(1, out_shape=(oh, ow), resampling=rasterio.enums.Resampling.average).astype(float)
        t = ds.transform
        # 出力グリッドのワールド左上（セル中心）と間隔
        dx = t.a * step
        dy = t.e * step  # 通常負（north-up）
        x0 = t.c + t.a * (step / 2.0)
        y0 = t.f + t.e * (step / 2.0)
    nodata = cl.get("dsm_nodata")
    finite = np.isfinite(arr)
    if not finite.any():
        raise HTTPException(422, "DSM に有効値がありません")
    zmin = float(arr[finite].min())
    zmax = float(arr[finite].max())
    z = [[(float(v) if np.isfinite(v) else None) for v in row] for row in arr]
    return {
        "nx": ow, "ny": oh, "x0": x0, "y0": y0, "dx": dx, "dy": dy,
        "z": z, "zmin": zmin, "zmax": zmax, "epsg": cl.get("epsg"),
        "nodata": nodata,
    }


@router.post("")
def generate_costmap(req: CostmapRequest):
    las = store.get_layer(req.las_layer_id)
    if not las or las.get("kind") != "las":
        raise HTTPException(404, "LAS layer not found")

    x, y, z, header_epsg = read_las_xyz(las["source"])
    if x.size == 0:
        raise HTTPException(422, "LAS has no points")

    # LAS の CRS 解決: 明示指定 > ヘッダ > 経緯度ヒューリスティック > 作業ゾーン（そのまま）。
    # WGS84 の LAS はヘッダに CRS が無いことが多く、従来はメートル扱いで壊れていた。
    # 全点が |x|<=180 かつ |y|<=90 なら経緯度（WGS84, EPSG:4326）と推定して再投影する。
    src_epsg = req.src_epsg or header_epsg
    crs_note: str | None = None
    if src_epsg:
        las_crs_source = "specified" if req.src_epsg else "header"
    elif float(np.abs(x).max()) <= 180.0 and float(np.abs(y).max()) <= 90.0:
        src_epsg = 4326
        las_crs_source = "assumed_wgs84"
        crs_note = "LASヘッダにCRSが無いため経緯度(WGS84)と推定して再投影しました。違う場合は「LASのCRS」を指定して再生成してください。"
    else:
        src_epsg = req.target_epsg
        las_crs_source = "assumed_working"
    if int(src_epsg) != int(req.target_epsg):
        xy = project(np.column_stack([x, y]), int(src_epsg), int(req.target_epsg))
        x, y = xy[:, 0], xy[:, 1]

    # 点群密度チェック（グリッドが密度に対し細かすぎると cost が無意味になる）
    n_pts = int(x.size)
    area = max(float((x.max() - x.min()) * (y.max() - y.min())), 1e-9)
    density = n_pts / area
    grid = req.params.grid_size_m
    pts_per_cell = density * grid * grid
    recommended_grid = round(2.0 / density**0.5, 1) if density > 0 else None  # ~4 pts/cell
    warning = None
    if pts_per_cell < 1.0:
        warning = (
            f"点群が疎です: {density:.1f} pts/m² ・ {pts_per_cell:.2f} 点/セル(@{grid}m)。"
            f" 実用解像度は約 {recommended_grid}m。grid>={recommended_grid}m を推奨"
            f"（細かいgridでは cost が補間アーティファクトになり信頼できません）。"
        )
    if crs_note:
        warning = f"{warning} / {crs_note}" if warning else crs_note

    out = build_costmap_arrays(x, y, z, req.params)
    cost = out["cost"]
    dsm = out["dsm"]
    transform = out["transform"]
    nodata_mask = out["nodata_mask"]

    rgba = cost_to_rgba(
        cost,
        nodata_mask=nodata_mask,
        obstacle_value=req.params.obstacle_value,
        vmax=req.params.display_vmax,
    )
    rgba_bands = np.moveaxis(rgba, 2, 0)  # (4, H, W) RGBA（範囲外は alpha=0）

    layer_id = store.new_layer_id("cost")
    d = store.layer_dir(layer_id)
    d.mkdir(parents=True, exist_ok=True)
    rgb_cog = str(d / "cog.tif")        # 表示用RGBA（layer.cog）
    cost_cog = str(d / "cost.tif")      # 生cost float32
    dsm_cog = str(d / "dsm.tif")        # 標高 DSM

    _write_cog(rgb_cog, rgba_bands, transform, req.target_epsg, nodata=None)
    _write_cog(cost_cog, cost, transform, req.target_epsg, nodata=req.params.obstacle_value)
    _write_cog(dsm_cog, dsm, transform, req.target_epsg, nodata=float("nan"))

    with Reader(rgb_cog) as r:
        gb = r.get_geographic_bounds(WGS84_CRS)

    meta = {
        "id": layer_id,
        "kind": "cost",
        "version": 1,
        "cog": rgb_cog,
        "cost_cog": cost_cog,        # 走行可能領域/閾値判定の正（§12）
        "dsm_cog": dsm_cog,          # 勾配解析の入力（§10）
        "epsg": int(req.target_epsg),
        "crs_source": "computed",
        "las_src_epsg": int(src_epsg),        # LAS座標をどのCRSとして読んだか
        "las_crs_source": las_crs_source,     # specified / header / assumed_wgs84 / assumed_working
        "width": int(cost.shape[1]),
        "height": int(cost.shape[0]),
        "bands": 4,
        "geographic_bounds": [float(v) for v in gb],
        "source_las": req.las_layer_id,
        "cost_unit": "weighted(slope_n*w_slope + rough_n*w_rough)",
        "obstacle_value": req.params.obstacle_value,
        "density_pts_m2": round(density, 2),
        "pts_per_cell": round(pts_per_cell, 3),
        "recommended_grid_m": recommended_grid,
        "warning": warning,
    }
    store.add_layer(meta)
    return meta
