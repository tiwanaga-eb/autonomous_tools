"""
GeoTIFF Downsampler
GeoTIFF画像を100MB以下にダウンサンプリングして出力するツール

依存ライブラリのインストール:
    pip install rasterio numpy tqdm
"""

import os
import sys
import math
import argparse
import numpy as np
from pathlib import Path
from tqdm import tqdm

try:
    import rasterio
    from rasterio.enums import Resampling
    from rasterio.transform import Affine
except ImportError:
    print("ERROR: rasterio がインストールされていません。")
    print("  pip install rasterio")
    sys.exit(1)


# ──────────────────────────────────────────────
# 定数
# ──────────────────────────────────────────────
TARGET_MB       = 100          # 目標ファイルサイズ (MB)
TARGET_BYTES    = TARGET_MB * 1024 * 1024
SAFETY_MARGIN   = 0.90         # 圧縮後のマージン (90%)
MAX_ITERATIONS  = 10           # 自動スケール調整の最大試行回数


# ──────────────────────────────────────────────
# ユーティリティ
# ──────────────────────────────────────────────
def file_size_mb(path: str) -> float:
    return os.path.getsize(path) / (1024 ** 2)


def estimate_scale_factor(src_path: str) -> float:
    """
    入力ファイルのサイズから、100MB に収めるための
    おおよそのスケールファクターを推定する。
    (圧縮なしの生ピクセル数ベースで計算)
    """
    with rasterio.open(src_path) as src:
        raw_bytes = src.width * src.height * src.count * (src.dtypes[0] in
                    ['uint16', 'int16', 'float16'] and 2 or
                    src.dtypes[0] in ['uint32', 'int32', 'float32'] and 4 or
                    src.dtypes[0] in ['float64', 'int64', 'uint64'] and 8 or 1)
    # 目標バイト数との比率から scale を計算
    ratio = (TARGET_BYTES * SAFETY_MARGIN) / raw_bytes
    scale = math.sqrt(ratio)
    return min(scale, 1.0)   # 拡大はしない


def dtype_itemsize(dtype_str: str) -> int:
    return np.dtype(dtype_str).itemsize


# ──────────────────────────────────────────────
# メイン処理
# ──────────────────────────────────────────────
def downsample_geotiff(
    input_path: str,
    output_path: str,
    scale: float | None = None,
    resampling_method: str = "lanczos",
    compress: str = "deflate",
    nodata: float | None = None,
    auto_adjust: bool = True,
) -> str:
    """
    GeoTIFF をダウンサンプリングして保存する。

    Parameters
    ----------
    input_path       : 入力 GeoTIFF パス
    output_path      : 出力 GeoTIFF パス
    scale            : リサイズ比率 (0 < scale <= 1)。None で自動推定
    resampling_method: リサンプリング手法 (lanczos / bilinear / nearest / average)
    compress         : 圧縮方式 (deflate / lzw / jpeg / none)
    nodata           : NoData 値 (None で元ファイルの値を引き継ぐ)
    auto_adjust      : 出力が 100MB を超えた場合に自動でスケールを下げる
    """
    input_path  = str(input_path)
    output_path = str(output_path)

    # ── リサンプリング手法マップ ──
    resample_map = {
        "lanczos" : Resampling.lanczos,
        "bilinear": Resampling.bilinear,
        "nearest" : Resampling.nearest,
        "average" : Resampling.average,
        "cubic"   : Resampling.cubic,
    }
    resample_alg = resample_map.get(resampling_method.lower(), Resampling.lanczos)

    # ── 圧縮オプション ──
    creation_options = {
        "TILED": True,
        "BLOCKXSIZE": 512,
        "BLOCKYSIZE": 512,
        "BIGTIFF": "IF_SAFER",
    }
    if compress.lower() != "none":
        creation_options["COMPRESS"] = compress.upper()
        if compress.lower() == "deflate":
            creation_options["ZLEVEL"] = 9
        elif compress.lower() == "jpeg":
            creation_options["JPEG_QUALITY"] = 85

    with rasterio.open(input_path) as src:
        src_width  = src.width
        src_height = src.height
        src_crs    = src.crs
        src_transform = src.transform
        src_nodata = src.nodata if nodata is None else nodata
        src_dtypes = src.dtypes
        band_count = src.count

        print(f"\n{'='*55}")
        print(f"  入力ファイル : {input_path}")
        print(f"  サイズ       : {src_width} x {src_height} px / {band_count} バンド")
        print(f"  データ型     : {src_dtypes[0]}")
        print(f"  CRS          : {src_crs}")
        print(f"  ファイルサイズ: {file_size_mb(input_path):.1f} MB")
        print(f"{'='*55}\n")

        # ── スケール自動推定 ──
        if scale is None:
            scale = estimate_scale_factor(input_path)
            print(f"  [自動推定] スケールファクター: {scale:.4f}")

        for attempt in range(1, MAX_ITERATIONS + 1):
            new_width  = max(1, int(src_width  * scale))
            new_height = max(1, int(src_height * scale))

            # 新しい GeoTransform
            new_transform = Affine(
                src_transform.a / scale, src_transform.b, src_transform.c,
                src_transform.d, src_transform.e / scale, src_transform.f,
            )

            profile = {
                "driver"   : "GTiff",
                "dtype"    : src_dtypes[0],
                "width"    : new_width,
                "height"   : new_height,
                "count"    : band_count,
                "crs"      : src_crs,
                "transform": new_transform,
                "nodata"   : src_nodata,
                **creation_options,
            }

            print(f"  試行 {attempt}: {new_width} x {new_height} px (scale={scale:.4f})")

            # ── 書き込み ──
            with rasterio.open(output_path, "w", **profile) as dst:
                for band_idx in tqdm(
                    range(1, band_count + 1),
                    desc=f"  バンド処理",
                    ncols=60,
                    leave=False,
                ):
                    data = src.read(
                        band_idx,
                        out_shape=(new_height, new_width),
                        resampling=resample_alg,
                    )
                    dst.write(data, band_idx)

            out_mb = file_size_mb(output_path)
            print(f"         → 出力サイズ: {out_mb:.1f} MB")

            if out_mb <= TARGET_MB:
                print(f"\n  ✅ 目標達成! {out_mb:.1f} MB <= {TARGET_MB} MB\n")
                break
            else:
                if not auto_adjust or attempt == MAX_ITERATIONS:
                    print(f"\n  ⚠️  {MAX_ITERATIONS} 回試行しましたが {TARGET_MB} MB を下回りませんでした。")
                    print(f"     最終出力: {out_mb:.1f} MB\n")
                    break
                # スケールを下げて再試行
                excess_ratio = TARGET_MB / out_mb
                scale *= math.sqrt(excess_ratio) * 0.97   # 少し余裕をもたせる
                print(f"         → スケール調整: {scale:.4f} で再試行...")

    return output_path


# ──────────────────────────────────────────────
# CLI
# ──────────────────────────────────────────────
def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="GeoTIFF を 100MB 以下にダウンサンプリングして出力します。",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("input",  help="入力 GeoTIFF ファイルパス")
    p.add_argument("output", nargs="?", help="出力ファイルパス (省略で input_resampled.tif)")
    p.add_argument(
        "--scale", "-s", type=float, default=None,
        help="リサイズ比率 (例: 0.5 で半分)。省略すると 100MB に収まるよう自動計算。",
    )
    p.add_argument(
        "--resampling", "-r",
        choices=["lanczos", "bilinear", "nearest", "average", "cubic"],
        default="lanczos",
        help="リサンプリング手法",
    )
    p.add_argument(
        "--compress", "-c",
        choices=["deflate", "lzw", "jpeg", "none"],
        default="deflate",
        help="圧縮方式 (jpeg は非可逆・RGB のみ推奨)",
    )
    p.add_argument(
        "--nodata", type=float, default=None,
        help="NoData 値 (省略で元ファイルの値を引き継ぐ)",
    )
    p.add_argument(
        "--no-auto-adjust", action="store_true",
        help="自動スケール調整を無効にする",
    )
    return p


def main():
    parser = build_parser()
    args   = parser.parse_args()

    input_path = args.input
    if not os.path.exists(input_path):
        print(f"ERROR: ファイルが見つかりません: {input_path}")
        sys.exit(1)

    if args.output:
        output_path = args.output
    else:
        p = Path(input_path)
        output_path = str(p.parent / f"{p.stem}_resampled{p.suffix}")

    downsample_geotiff(
        input_path      = input_path,
        output_path     = output_path,
        scale           = args.scale,
        resampling_method = args.resampling,
        compress        = args.compress,
        nodata          = args.nodata,
        auto_adjust     = not args.no_auto_adjust,
    )

    print(f"  出力先: {output_path}")
    print(f"  最終サイズ: {file_size_mb(output_path):.1f} MB")


if __name__ == "__main__":
    main()