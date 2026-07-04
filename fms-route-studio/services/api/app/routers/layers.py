from __future__ import annotations

from pathlib import Path

import math
import os
import shutil

import numpy as np
import rasterio
from affine import Affine
from fastapi import APIRouter, File, HTTPException, Response, UploadFile
from fastapi.responses import FileResponse
from rasterio.crs import CRS
from rasterio.enums import Resampling
from rasterio.io import MemoryFile
from rasterio.vrt import WarpedVRT
from rio_cogeo.cogeo import cog_translate, cog_validate
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


def _load_las_points_working(meta: dict, max_points: int) -> tuple[np.ndarray, np.ndarray | None]:
    """LAS を間引き読みし作業CRSへ再投影して (xyz(N,3), rgb(N,3)uint8|None) を返す。

    CRS 解決は planning_core.io.resolve_las_epsg（ヘッダ > 経緯度ヒューリスティック > 作業CRS）
    に一元化（costmap 生成と同一規則）。
    """
    from planning_core.geometry import project
    from planning_core.io import read_las_points, resolve_las_epsg

    xyz, rgb, epsg = read_las_points(meta["source"], max_points=max_points)
    if xyz.shape[0] == 0:
        raise HTTPException(422, "LAS has no points")
    epsg, _src = resolve_las_epsg(epsg, xyz[:, 0], xyz[:, 1])
    if epsg and int(epsg) != get_working_epsg():
        xy = project(xyz[:, :2], int(epsg), get_working_epsg())
        xyz = np.column_stack([xy, xyz[:, 2]])
    return xyz, rgb


@router.get("/{layer_id}/points.bin")
def las_points_bin(layer_id: str, max_points: int = 1_000_000):
    """3D表示用の点群バイナリ（JSON の ~1/10 サイズ・大点数向け）。

    レイアウト（little-endian）:
      magic "FRSP"(4) | version u8=1 | has_rgb u8 | reserved u16 |
      n u32 | origin ox,oy,oz f64×3 | zmin,zmax f64×2 |
      rel_x f32×n | rel_y f32×n | rel_z f32×n | rgb u8×3n（has_rgb 時）
    座標は origin（点群中心）からの相対値 f32＝ミリ精度を保ったまま 12B/点。
    """
    import struct

    meta = store.get_layer(layer_id)
    if not meta or meta.get("kind") != "las" or not meta.get("source"):
        raise HTTPException(404, "LAS layer not found")
    # 上限16M点（15B/点 → 最大~240MB応答。ローカル運用＋バイナリ転送前提。ファイル総点数が上限）
    xyz, rgb = _load_las_points_working(meta, max_points=max(1000, min(max_points, 16_000_000)))
    ox = float(xyz[:, 0].min() + xyz[:, 0].max()) / 2.0
    oy = float(xyz[:, 1].min() + xyz[:, 1].max()) / 2.0
    zmin, zmax = float(xyz[:, 2].min()), float(xyz[:, 2].max())
    oz = (zmin + zmax) / 2.0
    n = int(xyz.shape[0])
    head = struct.pack("<4sBBHI5d", b"FRSP", 1, 1 if rgb is not None else 0, 0, n, ox, oy, oz, zmin, zmax)
    rel = (xyz - np.array([ox, oy, oz])).astype("<f4")
    parts = [head, rel[:, 0].tobytes(), rel[:, 1].tobytes(), rel[:, 2].tobytes()]
    if rgb is not None:
        parts.append(np.ascontiguousarray(rgb, dtype=np.uint8).tobytes())
    return Response(content=b"".join(parts), media_type="application/octet-stream")


@router.get("/{layer_id}/points")
def las_points(layer_id: str, max_points: int = 200000):
    """3D表示用に LAS 点群を間引いて返す（XYZ＝作業CRS[m]、RGBがあれば0-255）。"""
    meta = store.get_layer(layer_id)
    if not meta or meta.get("kind") != "las" or not meta.get("source"):
        raise HTTPException(404, "LAS layer not found")
    xyz, rgb = _load_las_points_working(meta, max_points=max(1000, min(max_points, 600000)))
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

    if kind == "las":
        # ヘッダの CRS を検出してメタに記録（コストマップ生成の既定 src・FE 表示用）。
        from planning_core.io import las_header_epsg

        det = las_header_epsg(src)
        if det:
            meta["epsg"] = int(det)
            meta["crs_source"] = "detected"

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

        gdal_cfg = {"GDAL_NUM_THREADS": "ALL_CPUS"}
        if not need_reproj and not need_down:
            # 高速パス: 既に妥当な COG ならコピーのみ、そうでなければ COG 化。
            try:
                is_cog, _errs, _warns = cog_validate(str(src), quiet=True)
            except Exception:  # noqa: BLE001
                is_cog = False
            if is_cog:
                shutil.copyfile(src, cog)
            else:
                cog_translate(str(src), str(cog), cog_profiles.get("deflate"), quiet=True, config=gdal_cfg)
            out_epsg = detected.to_epsg() or get_working_epsg()
        else:
            # 単一パス: 目標グリッド（作業CRS・≤MAX_RASTER_DIM）へ**直接**マルチスレッドでワープし、
            # メモリ経由で一度だけ COG エンコードする。
            # 旧実装は フル解像度warp読み → 中間tif(deflate) → cog_translate(再デコード+再deflate) の
            # 三重処理で、大型オルソの取り込みが数倍遅かった（実測 14.8s → 3.3s @ 42Mpx）。
            with rasterio.open(src) as ds0:
                probe = WarpedVRT(ds0, crs=working, resampling=Resampling.bilinear) if need_reproj else ds0
                sw, sh = probe.width, probe.height
                base_tr = probe.transform
                factor = int(math.ceil(max(sw, sh) / MAX_RASTER_DIM)) if max(sw, sh) > MAX_RASTER_DIM else 1
                ow, oh = max(1, sw // factor), max(1, sh // factor)
                tr = base_tr * Affine.scale(factor) if factor > 1 else base_tr
                if probe is not ds0:
                    probe.close()
                with WarpedVRT(
                    ds0, crs=working, transform=tr, width=ow, height=oh,
                    resampling=(Resampling.average if factor > 1 else Resampling.bilinear),
                    warp_mem_limit=512, num_threads=os.cpu_count() or 4,
                ) as vrt:
                    data = vrt.read()
                    bands, dtype, nod = vrt.count, vrt.dtypes[0], vrt.nodata
            prof = dict(driver="GTiff", height=oh, width=ow, count=bands, dtype=dtype,
                        crs=working, transform=tr, tiled=True, blockxsize=512, blockysize=512)
            if nod is not None:
                prof["nodata"] = nod
            # 表示用オルソ（8bit×3band）は JPEG(RGB, q90) COG＝エンコードが速くファイルも小さい
            # （YCbCr はブラウザ側 geotiff.js が色変換しないため使わない）。それ以外は可逆 deflate。
            if kind == "ortho" and bands == 3 and str(dtype) == "uint8":
                out_profile = dict(cog_profiles.get("jpeg"))
                out_profile["photometric"] = "RGB"
                out_profile["jpeg_quality"] = 90
            else:
                out_profile = cog_profiles.get("deflate")
            with MemoryFile() as mem:
                with mem.open(**prof) as tmp:
                    tmp.write(data)
                with mem.open() as tmp:
                    cog_translate(tmp, str(cog), out_profile, quiet=True, config=gdal_cfg)
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
