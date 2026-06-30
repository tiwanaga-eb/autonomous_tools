from __future__ import annotations

import os
from pathlib import Path

DATA_DIR = Path(os.environ.get("FRS_DATA_DIR", str(Path.home() / ".fms-route-studio" / "data")))
LAYERS_DIR = DATA_DIR / "layers"
REGISTRY_PATH = DATA_DIR / "registry.json"
PROJECTS_PATH = DATA_DIR / "projects.json"
VEHICLE_OVERRIDES_PATH = DATA_DIR / "vehicle_overrides.json"

DEFAULT_EPSG = int(os.environ.get("FRS_DEFAULT_EPSG", "6677"))

# 作業ゾーン（投影座標系）。起動時は FRS_DEFAULT_EPSG。UI から実行時に変更可（セッション内・メモリ保持）。
# アップロードの再投影先・/health が返す作業EPSG はこの値を参照する。
_working_epsg = DEFAULT_EPSG
# JGD2011 平面直角座標系 系I〜XIX（UIで選べる作業ゾーン）。
JGD2011_ZONES = tuple(range(6669, 6688))


def get_working_epsg() -> int:
    return _working_epsg


def set_working_epsg(epsg: int) -> int:
    global _working_epsg
    if int(epsg) not in JGD2011_ZONES:
        raise ValueError(f"EPSG:{epsg} は JGD2011 平面直角座標系(6669〜6687)ではありません")
    _working_epsg = int(epsg)
    return _working_epsg

# アップロードするラスタの最大辺[px]。これを超える画像は取り込み時にダウンサンプルしてからCOG化する
# （巨大ortho(数億px)の cog_translate がサーバをブロック/ディスクを圧迫するのを防ぐ）。
MAX_RASTER_DIM = int(os.environ.get("FRS_MAX_RASTER_DIM", "8192"))

# アップロード1ファイルのバイトサイズ上限[MiB]（LAS/orthoはGB級）。超過は 413。
# ※ これは「ファイルのバイト数」上限であり、点群の読込間引き点数(FRS_LAS_MAX_POINTS)とは別概念。
MAX_UPLOAD_MIB = int(os.environ.get("FRS_MAX_UPLOAD_MIB", "16384"))  # 既定16 GiB
MAX_UPLOAD_BYTES = MAX_UPLOAD_MIB * (1 << 20)

RASTER_KINDS = {"ortho", "cost", "dsm"}
ALL_KINDS = RASTER_KINDS | {"las"}
