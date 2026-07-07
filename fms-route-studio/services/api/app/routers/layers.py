from __future__ import annotations

from pathlib import Path

import math
import os
import shutil

import numpy as np
import rasterio
from affine import Affine
from fastapi import APIRouter, File, HTTPException, Response, UploadFile
from fastapi.responses import FileResponse, StreamingResponse
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


@router.patch("/{layer_id}/epsg")
def set_layer_epsg(layer_id: str, epsg: int | None = None):
    """LAS レイヤの座標系(EPSG)を後付け指定する（ヘッダに CRS が無い現場 LAS 向け）。

    国内の測量 LAS はヘッダ CRS 欠落が常態で、メートル座標だと「作業CRSのまま」と
    みなされて誤った場所に配置される。ここで指定した EPSG は以後の costmap 生成・
    3D点群表示・オルソ生成すべてで最優先（ヘッダより強い）に使われる。
    epsg 省略（null）で指定を解除しヘッダ/推定へ戻す。
    """
    meta = store.get_layer(layer_id)
    if not meta or meta.get("kind") != "las":
        raise HTTPException(404, "LAS layer not found")
    if epsg is not None and not (1000 <= int(epsg) <= 999999):
        raise HTTPException(400, f"invalid epsg: {epsg}")
    patch = {"epsg": (int(epsg) if epsg is not None else None),
             "crs_source": ("user" if epsg is not None else None)}
    meta = store.update_layer(layer_id, patch)
    # 座標系が変わると DSM の配置も変わる → その場で再生成（オルソは「オルソ生成」で手動再生成）
    if meta:
        _generate_las_dsm(meta)
    return meta


def _load_las_points_working(meta: dict, max_points: int) -> tuple[np.ndarray, np.ndarray | None]:
    """LAS を間引き読みし作業CRSへ再投影して (xyz(N,3), rgb(N,3)uint8|None) を返す。

    CRS 解決は planning_core.io.resolve_las_epsg（メタ指定 > ヘッダ > 経緯度ヒューリスティック
    > 作業CRS）に一元化（costmap 生成と同一規則）。meta["epsg"] はアップロード時のヘッダ検出値
    または PATCH /{id}/epsg でのユーザー指定値。
    """
    from planning_core.geometry import project
    from planning_core.io import read_las_points, resolve_las_epsg

    xyz, rgb, epsg = read_las_points(meta["source"], max_points=max_points)
    if xyz.shape[0] == 0:
        raise HTTPException(422, "LAS has no points")
    epsg, _src = resolve_las_epsg(epsg, xyz[:, 0], xyz[:, 1], override=meta.get("epsg"))
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
    has_rgb = rgb is not None
    head = struct.pack("<4sBBHI5d", b"FRSP", 1, 1 if has_rgb else 0, 0, n, ox, oy, oz, zmin, zmax)
    rel = (xyz - np.array([ox, oy, oz])).astype("<f4")
    rgb8 = np.ascontiguousarray(rgb, dtype=np.uint8) if has_rgb else None

    # 16M点で本体 ~240MB。b"".join だと rel と結合バッファの二重持ちでピーク ~2倍になる
    # ため、列ごとに ~8MB のチャンクへ切ってストリーム送出する（Content-Length は既知）。
    chunk = 2_000_000  # f32 で 8MB/チャンク

    def _iter_body():
        yield head
        for col in range(3):
            a = rel[:, col]
            for i in range(0, n, chunk):
                yield a[i:i + chunk].tobytes()
        if rgb8 is not None:
            for i in range(0, n, chunk):
                yield rgb8[i:i + chunk].tobytes()

    total = len(head) + 12 * n + (3 * n if has_rgb else 0)
    return StreamingResponse(
        _iter_body(),
        media_type="application/octet-stream",
        headers={"Content-Length": str(total)},
    )


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


def _generate_las_dsm(las_meta: dict, x=None, y=None, z=None) -> str | None:
    """LAS から DSM(COG, float32, nodata=NaN) を生成しメタに dsm_cog を記録する。

    コストマップ未生成でも経路の標高(Z)埋め込み・勾配解析ができるようにするフォールバック
    （/plan・/analyze・/api/elevation/sample が cost レイヤの DSM が無いとき参照する）。
    x/y/z を渡せば点の再読込を省く（オルソ生成と同時実行用）。失敗は None（呼び出し側続行）。
    """
    from planning_core.ortho import auto_ortho_res, points_to_dsm

    try:
        if x is None:
            xyz, _rgb = _load_las_points_working(las_meta, max_points=12_000_000)
            x, y, z = xyz[:, 0], xyz[:, 1], xyz[:, 2]
        area = float(max(x.max() - x.min(), 1e-6) * max(y.max() - y.min(), 1e-6))
        # DSM は勾配解析用: 0.25m より細かくしない（点密度ノイズが勾配に乗る）
        r = max(auto_ortho_res(len(x), area, max_dim=MAX_RASTER_DIM), 0.25)
        dsm, tr = points_to_dsm(x, y, z, res=r)
        from .costmap import _write_cog

        d = store.layer_dir(las_meta["id"])
        d.mkdir(parents=True, exist_ok=True)
        dsm_cog = str(d / "dsm.tif")
        _write_cog(dsm_cog, dsm, tr, get_working_epsg(), nodata=float("nan"))
        store.update_layer(las_meta["id"], {"dsm_cog": dsm_cog, "dsm_res_m": round(r, 3)})
        las_meta["dsm_cog"] = dsm_cog
        return dsm_cog
    except Exception:  # noqa: BLE001 — DSM はフォールバック機能。失敗しても取込は成功扱い
        return None


def ensure_las_dsm(las_meta: dict) -> str | None:
    """LAS レイヤの DSM パスを返す（無ければその場で生成して永続化）。"""
    if las_meta.get("dsm_cog") and Path(las_meta["dsm_cog"]).exists():
        return las_meta["dsm_cog"]
    return _generate_las_dsm(las_meta)


def _generate_ortho_from_las(las_meta: dict, res: float | None = None) -> dict:
    """LAS 点群からオルソ（平均色ラスタ）を生成し ortho レイヤとして登録する。

    las2ortho（PDAL 版 dsm2ortho3.py）のネイティブ移植: RGB があれば平均色、無ければ
    Z の 2-98% 正規化グレー。CRS はレイヤの解決規則（メタ指定>ヘッダ>推定）に従い
    作業CRSへ再投影済みの点から作るため、点群/コストマップと必ず重なる。
    DSM（標高ラスタ）も同じ点群から併せて再生成する（点の読込を共有）。
    """
    from planning_core.ortho import auto_ortho_res, points_to_ortho

    xyz, rgb = _load_las_points_working(las_meta, max_points=24_000_000)
    x, y, z = xyz[:, 0], xyz[:, 1], xyz[:, 2]
    _generate_las_dsm(las_meta, x, y, z)  # 標高/勾配用 DSM を同時更新
    area = float(max(x.max() - x.min(), 1e-6) * max(y.max() - y.min(), 1e-6))
    r = float(res) if res and res > 0 else auto_ortho_res(len(x), area, max_dim=MAX_RASTER_DIM)
    img, tr = points_to_ortho(x, y, z, rgb, res=r)

    layer_id = store.new_layer_id("ortho")
    d = store.layer_dir(layer_id)
    d.mkdir(parents=True, exist_ok=True)
    cog = d / "cog.tif"
    prof = dict(driver="GTiff", height=int(img.shape[1]), width=int(img.shape[2]), count=3,
                dtype="uint8", crs=CRS.from_epsg(get_working_epsg()), transform=tr,
                tiled=True, blockxsize=512, blockysize=512, nodata=0)
    out_profile = dict(cog_profiles.get("jpeg"))
    out_profile["photometric"] = "RGB"
    out_profile["jpeg_quality"] = 90
    with MemoryFile() as mem:
        with mem.open(**prof) as tmp:
            tmp.write(img)
        with mem.open() as tmp:
            cog_translate(tmp, str(cog), out_profile, quiet=True,
                          config={"GDAL_NUM_THREADS": "ALL_CPUS"})
    with Reader(str(cog)) as rr:
        gb = rr.get_geographic_bounds(WGS84_CRS)

    stem = Path(las_meta.get("filename") or "las").stem
    meta = {
        "id": layer_id, "kind": "ortho", "version": 1,
        "filename": f"{stem}_ortho_{r:.2f}m.tif",
        "cog": str(cog),
        "source_las": las_meta["id"],
        "epsg": get_working_epsg(), "crs_source": "computed",
        "width": int(img.shape[2]), "height": int(img.shape[1]), "bands": 3,
        "res_m": round(r, 3),
        "geographic_bounds": [float(v) for v in gb],
    }
    store.add_layer(meta)
    return meta


@router.post("/{layer_id}/ortho")
def make_ortho_from_las(layer_id: str, res: float | None = None):
    """LAS レイヤからオルソを（再）生成する。res 省略時は点密度から自動決定。

    CRS を PATCH /{id}/epsg で指定し直した後の再生成にも使う。
    """
    meta = store.get_layer(layer_id)
    if not meta or meta.get("kind") != "las" or not meta.get("source"):
        raise HTTPException(404, "LAS layer not found")
    return _generate_ortho_from_las(meta, res)


@router.post("/{kind}")
async def upload_layer(
    kind: str,
    file: UploadFile = File(...),
    assign_epsg: int | None = None,
    src_epsg: int | None = None,
    make_ortho: bool = True,
):
    """GeoTIFF/LAS をアップロード→(ラスタは)COG化して登録。

    CRS 検出: 無い場合は再投影せず assign（既定 EPSG）。crs_source に記録（設計書 §6.1 / M3）。
    LAS: src_epsg で座標系を明示指定できる（ヘッダ CRS 欠落の現場 LAS 向け。後から
    PATCH /{id}/epsg でも変更可）。make_ortho=True（既定）なら取り込み後にオルソを自動生成。
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
        # 不正な LAS はここで早期拒否する（従来は登録が通り、後段の costmap 生成で 500 になっていた）。
        import laspy

        try:
            with laspy.open(str(src)):
                pass
        except Exception as e:  # noqa: BLE001
            shutil.rmtree(d, ignore_errors=True)
            raise HTTPException(422, f"LASファイルを読めません（壊れているか形式が不正）: {e}")

        # CRS をメタに記録: 明示指定(src_epsg) > ヘッダ検出。以後の costmap/3D点群/オルソが
        # resolve_las_epsg の override として最優先で使う（コストマップ生成の既定 src・FE 表示用）。
        from planning_core.io import las_header_epsg

        if src_epsg:
            meta["epsg"] = int(src_epsg)
            meta["crs_source"] = "user"
        else:
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

    # LAS 取り込み時のオルソ自動生成（las2ortho ワークフローの内蔵化）。DSM（標高/勾配用）は
    # ortho の有無に依らず常に生成する（コストマップ未生成でも Z 埋め込み・勾配解析を可能に）。
    # 失敗してもアップロード自体は成功として返す（オルソ/DSM は後から /ortho で再生成できる）。
    if kind == "las":
        if make_ortho:
            try:
                ortho_meta = _generate_ortho_from_las(meta)  # 内部で DSM も同時生成（点読込を共有）
                meta = store.update_layer(layer_id, {"auto_ortho_id": ortho_meta["id"]}) or meta
            except Exception as e:  # noqa: BLE001
                meta["ortho_error"] = f"オルソ自動生成に失敗: {e}"
        else:
            _generate_las_dsm(meta)
    return meta
