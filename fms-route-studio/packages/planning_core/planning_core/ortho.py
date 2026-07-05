"""LAS 点群 → オルソ画像（真上から見た平均色ラスタ）。

参照実装 las2ortho/dsm2ortho3.py（PDAL writers.gdal）の planning_core ネイティブ版。
色ソースは RGB → Z(標高グレー) の順でフォールバックする（参照実装の RGB→Intensity→Z の
うち Intensity は io.las が読んでいないため v1 では省略）。

グリッド化はセル毎の平均（np.add.at のビンカウント）。点が無いセルは 0（nodata 扱い）。
出力は常に 3band uint8（グレーは複製）— 取り込み側の JPEG(RGB) COG パスに一本化するため。
"""
from __future__ import annotations

import math

import numpy as np
from affine import Affine


def auto_ortho_res(n_pts: int, area_m2: float, *, target_pts_per_cell: float = 2.0,
                   min_res: float = 0.05, max_res: float = 1.0, max_dim: int = 8192) -> float:
    """点密度から適切なオルソ解像度[m/px]を選ぶ。

    セルあたり ~target_pts_per_cell 点になる解像度を基本に [min_res, max_res] へクランプし、
    さらに辺長が max_dim px を超えない解像度まで粗くする。
    """
    if n_pts <= 0 or area_m2 <= 0:
        return max_res
    res = math.sqrt(area_m2 / n_pts * target_pts_per_cell)
    res = min(max(res, min_res), max_res)
    side = math.sqrt(area_m2)  # 正方形近似での辺長[m]
    if side / res > max_dim:
        res = side / max_dim
    return float(res)


def _mean_bin(vals: np.ndarray, idx: np.ndarray, n_cells: int) -> tuple[np.ndarray, np.ndarray]:
    """セル毎平均。返値 (mean(float64, n_cells), count(int64, n_cells))。"""
    s = np.zeros(n_cells, np.float64)
    c = np.zeros(n_cells, np.int64)
    np.add.at(s, idx, vals.astype(np.float64))
    np.add.at(c, idx, 1)
    mean = np.divide(s, c, out=np.zeros_like(s), where=c > 0)
    return mean, c


def _gray_to_u8(vals: np.ndarray, filled: np.ndarray) -> np.ndarray:
    """有効セルの 2-98 パーセンタイルで 0..255 に正規化（参照実装と同じ）。"""
    v = vals[filled]
    if v.size == 0:
        return np.zeros_like(vals, np.uint8)
    vmin = float(np.percentile(v, 2))
    vmax = float(np.percentile(v, 98))
    if vmax - vmin < 1e-9:  # 全セル同値（完全平坦）→ 中間グレー
        return np.where(filled, 128, 0).astype(np.uint8)
    out = ((vals - vmin) * (255.0 / (vmax - vmin))).clip(0, 255).astype(np.uint8)
    # nodata セルは 0 に固定（正規化で 0 以外になり得るため）
    out[~filled] = 0
    return out


def points_to_ortho(x: np.ndarray, y: np.ndarray, z: np.ndarray,
                    rgb: np.ndarray | None, *, res: float) -> tuple[np.ndarray, Affine]:
    """点群をオルソ画像へグリッド化する。

    x/y[m]（作業CRS）, z[m], rgb=(N,3)uint8|None。返値 (img(3,H,W)uint8, north-up Affine)。
    RGB が有れば平均色、無ければ Z の 2-98% 正規化グレーを 3band に複製。
    点の無いセルは (0,0,0)=nodata。
    """
    if x.size == 0:
        raise ValueError("no points")
    res = float(res)
    x0, x1 = float(x.min()), float(x.max())
    y0, y1 = float(y.min()), float(y.max())
    # floor+1: 端点（x=x1/y=y0）が自分のセルを持つように（ceil だと境界ちょうどの点が潰れる）
    w = int((x1 - x0) / res) + 1
    h = int((y1 - y0) / res) + 1
    col = np.clip(((x - x0) / res).astype(np.int64), 0, w - 1)
    row = np.clip(((y1 - y) / res).astype(np.int64), 0, h - 1)  # north-up: 上端が y1
    idx = row * w + col
    n_cells = h * w

    if rgb is not None and len(rgb) == len(x):
        bands = []
        counts = None
        for b in range(3):
            mean, c = _mean_bin(np.asarray(rgb[:, b], np.float64), idx, n_cells)
            counts = c
            bands.append(mean)
        filled = counts > 0
        img = np.stack([np.where(filled, bd, 0.0).clip(0, 255).astype(np.uint8).reshape(h, w)
                        for bd in bands])
        # 全点 (0,0,0) 等の実質無色 RGB は Z グレーへフォールバック
        if img.max() == 0:
            rgb = None
    if rgb is None or len(rgb) != len(x):
        mean, c = _mean_bin(np.asarray(z, np.float64), idx, n_cells)
        filled = c > 0
        g = _gray_to_u8(mean, filled).reshape(h, w)
        img = np.stack([g, g, g])

    transform = Affine(res, 0.0, x0, 0.0, -res, y1)
    return img, transform
