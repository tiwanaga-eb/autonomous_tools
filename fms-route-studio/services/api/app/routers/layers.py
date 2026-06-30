from __future__ import annotations

from pathlib import Path

import math
import os
import shutil

import numpy as np
import rasterio
from affine import Affine
from fastapi import APIRouter, File, HTTPException, UploadFile
from fastapi.responses import FileResponse
from rasterio.crs import CRS
from rasterio.enums import Resampling
from rasterio.vrt import WarpedVRT
from rio_cogeo.cogeo import cog_translate
from rio_cogeo.profiles import cog_profiles
from rio_tiler.constants import WGS84_CRS
from rio_tiler.io import Reader

from .. import store
from ..settings import ALL_KINDS, MAX_RASTER_DIM, MAX_UPLOAD_BYTES, RASTER_KINDS, get_working_epsg

router = APIRouter(prefix="/api/layers", tags=["layers"])


@router.get("")
def list_layers():
    return {"layers": store.list_layers()}


@router.get("/{layer_id}")
def get_layer(layer_id: str):
    meta = store.get_layer(layer_id)
    if not meta:
        raise HTTPException(404, "layer not found")
    return meta


@router.delete("/{layer_id}")
def delete_layer(layer_id: str):
    if not store.delete_layer(layer_id):
        raise HTTPException(404, "layer not found")
    return {"ok": True}


@router.get("/{layer_id}/points")
def las_points(layer_id: str, max_points: int = 200000):
    """3D表示用に LAS 点群を間引いて返す（XYZ＝作業CRS[m]、RGBがあれば0-255）。"""
    from planning_core.geometry import project
    from planning_core.io import read_las_points

    meta = store.get_layer(layer_id)
    if not meta or meta.get("kind") != "las" or not meta.get("source"):
        raise HTTPException(404, "LAS layer not found")
    xyz, rgb, epsg = read_las_points(meta["source"], max_points=max(1000, min(max_points, 600000)))
    if xyz.shape[0] == 0:
        raise HTTPException(422, "LAS has no points")
    # 作業CRS(6677)へ（ヘッダEPSGがあり異なる場合のみ再投影。無ければ既に6677想定）
    if epsg and int(epsg) != get_working_epsg():
        xy = project(xyz[:, :2], int(epsg), get_working_epsg())
        xyz = np.column_stack([xy, xyz[:, 2]])
    x = [round(float(v), 2) for v in xyz[:, 0]]
    y = [round(float(v), 2) for v in xyz[:, 1]]
    z = [round(float(v), 2) for v in xyz[:, 2]]
    out = {
        "n": int(xyz.shape[0]),
        "x": x, "y": y, "z": z,
        "has_rgb": rgb is not None,
        "rgb": rgb.tolist() if rgb is not None else None,
        "zmin": float(xyz[:, 2].min()), "zmax": float(xyz[:, 2].max()),
        "epsg": get_working_epsg(),
    }
    return out


@router.get("/{layer_id}/cog.tif")
def download_cog(layer_id: str):
    """COG をそのまま配信（FastAPI FileResponse は Range 対応）。

    FE は OpenLayers の ol/source/GeoTIFF でこれを直接読み、ネイティブ EPSG:6677 で
    表示する（Web Mercator 再投影を挟まないので座標は厳密＝3cm精度要件を満たす）。
    """
    meta = store.get_layer(layer_id)
    if not meta or "cog" not in meta:
        raise HTTPException(404, "raster layer not found")
    return FileResponse(meta["cog"], media_type="image/tiff", filename=f"{layer_id}.tif")


@router.post("/{kind}")
async def upload_layer(
    kind: str,
    file: UploadFile = File(...),
    assign_epsg: int | None = None,
):
    """GeoTIFF/LAS をアップロード→(ラスタは)COG化して登録。

    CRS 検出: 無い場合は再投影せず assign（既定 EPSG）。crs_source に記録（設計書 §6.1 / M3）。
    """
    if kind not in ALL_KINDS:
        raise HTTPException(400, f"unknown kind: {kind} (allowed: {sorted(ALL_KINDS)})")

    layer_id = store.new_layer_id(kind)
    d = store.layer_dir(layer_id)
    d.mkdir(parents=True, exist_ok=True)
    suffix = Path(file.filename or "").suffix or (".las" if kind == "las" else ".tif")
    src = d / f"source{suffix}"
    # 全体をメモリ展開せずチャンクでディスクへ（LAS/ortho は GB 級 → OOM 回避）。上限超過は 413。
    written = 0
    try:
        with src.open("wb") as out:
            while True:
                chunk = await file.read(1 << 20)  # 1 MiB
                if not chunk:
                    break
                written += len(chunk)
                if written > MAX_UPLOAD_BYTES:
                    raise HTTPException(413, f"file too large (> {MAX_UPLOAD_BYTES // (1 << 20)} MiB)")
                out.write(chunk)
    except HTTPException:
        shutil.rmtree(d, ignore_errors=True)  # 途中ファイル/レイヤディレクトリを掃除
        raise

    meta: dict = {
        "id": layer_id,
        "kind": kind,
        "version": 1,
        "filename": file.filename,
        "source": str(src),
    }

    if kind in RASTER_KINDS:
        with rasterio.open(src) as ds:
            detected = ds.crs
            width, height, count = ds.width, ds.height, ds.count

        crs_source = "detected"
        if detected is None:
            assigned = CRS.from_epsg(int(assign_epsg or get_working_epsg()))
            try:
                with rasterio.open(src, "r+") as ds:
                    ds.crs = assigned
            except Exception as e:  # noqa: BLE001
                raise HTTPException(422, f"failed to assign CRS: {e}")
            detected = assigned
            crs_source = "assigned"

        # 作業CRS(6677)へ正規化＋巨大ラスタは取り込み時にダウンサンプル。
        # （CRS不一致だとビューが ortho の投影になり、6677の経路/矢印が桁違いに見えるため。
        #  また巨大ortho の cog_translate はサーバをブロック/ディスク圧迫するため縮小する。）
        working = CRS.from_epsg(get_working_epsg())
        need_reproj = detected.to_epsg() != get_working_epsg()
        need_down = max(width, height) > MAX_RASTER_DIM
        cog = d / "cog.tif"
        downsampled_from = None
        reprojected_from = None

        if not need_reproj and not need_down:
            # 高速パス: そのまま COG 化
            cog_translate(str(src), str(cog), cog_profiles.get("deflate"), quiet=True)
            out_epsg = detected.to_epsg() or get_working_epsg()
        else:
            prepared = d / "prepared.tif"
            with rasterio.open(src) as ds:
                ctx = WarpedVRT(ds, crs=working, resampling=Resampling.bilinear) if need_reproj else ds
                srcr = ctx
                sw, sh = srcr.width, srcr.height
                factor = int(math.ceil(max(sw, sh) / MAX_RASTER_DIM)) if max(sw, sh) > MAX_RASTER_DIM else 1
                ow, oh = max(1, sw // factor), max(1, sh // factor)
                data = srcr.read(out_shape=(srcr.count, oh, ow), resampling=Resampling.average)
                tr = srcr.transform * Affine.scale(factor) if factor > 1 else srcr.transform
                bands = srcr.count
                dtype = srcr.dtypes[0]
                nod = srcr.nodata
                if need_reproj:
                    srcr.close()
            prof = dict(driver="GTiff", height=oh, width=ow, count=bands, dtype=dtype,
                        crs=working, transform=tr, tiled=True, blockxsize=512, blockysize=512, compress="deflate")
            if nod is not None:
                prof["nodata"] = nod
            with rasterio.open(prepared, "w", **prof) as dst:
                dst.write(data)
            cog_translate(str(prepared), str(cog), cog_profiles.get("deflate"), quiet=True)
            if os.path.exists(prepared):
                os.remove(prepared)
            out_epsg = get_working_epsg()
            count = bands
            if factor > 1:
                downsampled_from = [width, height]
            if need_reproj:
                reprojected_from = detected.to_epsg()
            width, height = ow, oh

        with Reader(str(cog)) as r:
            gb = r.get_geographic_bounds(WGS84_CRS)
            epsg = r.dataset.crs.to_epsg() if r.dataset.crs else None

        meta.update(
            {
                "cog": str(cog),
                "epsg": epsg or out_epsg,
                "crs_source": crs_source,
                "width": width,
                "height": height,
                "bands": count,
                "geographic_bounds": [float(v) for v in gb],
                "downsampled_from": downsampled_from,
                "reprojected_from": reprojected_from,
            }
        )

    store.add_layer(meta)
    return meta
