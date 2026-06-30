"""Raw scalar cost -> display RGB (表示専用)。走行判定には使わない。

設計書 §11/§21(B1)。表示は cost を [0, vmax] に正規化して 緑→黄→赤。
vmax 未指定なら **有効costのp98で自動スケール**（絶対スケールに依らず常にグラデが出る）。
不可侵/欠損(cost>=obstacle_value or NaN)はグレー。
"""
from __future__ import annotations

import numpy as np
from scipy.ndimage import binary_closing, binary_fill_holes, distance_transform_edt


def cost_to_rgb(
    cost: np.ndarray,
    vmax: float | None = None,
    nodata_mask: np.ndarray | None = None,
    obstacle_value: float = 1e9,
) -> np.ndarray:
    """Return an (H, W, 3) uint8 RGB image for display."""
    cost = np.asarray(cost, dtype=np.float32)
    h, w = cost.shape
    rgb = np.zeros((h, w, 3), dtype=np.uint8)

    finite = np.isfinite(cost) & (cost < obstacle_value)

    if vmax is None or vmax <= 0:
        vals = cost[finite]
        vmax = float(np.percentile(vals, 98)) if vals.size else 1.0
        if not np.isfinite(vmax) or vmax <= 0:
            vmax = 1.0

    t = np.zeros((h, w), dtype=np.float32)
    t[finite] = np.clip(cost[finite] / vmax, 0.0, 1.0)

    first = finite & (t <= 0.5)
    second = finite & (t > 0.5)
    # green(0,255,0) -> yellow(255,255,0)
    rgb[first, 0] = (255 * (t[first] / 0.5)).astype(np.uint8)
    rgb[first, 1] = 255
    rgb[first, 2] = 0
    # yellow -> red(255,0,0)
    rgb[second, 0] = 255
    rgb[second, 1] = (255 * (1.0 - (t[second] - 0.5) / 0.5)).astype(np.uint8)
    rgb[second, 2] = 0

    # 不可侵/範囲外（cost>=obstacle or NaN）はグレー
    rgb[~finite] = np.array([90, 90, 90], dtype=np.uint8)
    # 欠損は濃いめグレー
    if nodata_mask is not None and np.any(nodata_mask):
        rgb[nodata_mask] = np.array([60, 60, 60], dtype=np.uint8)
    return rgb


def cost_to_rgba(
    cost: np.ndarray,
    nodata_mask: np.ndarray,
    obstacle_value: float = 1e9,
    vmax: float | None = None,
) -> np.ndarray:
    """表示用 RGBA。データ範囲(footprint)内の穴を最近傍で埋め、範囲外は alpha=0（透明）。

    碁盤の目状の欠損を消し、データのある領域だけを連続塗りで重畳できるようにする。
    走行判定に使う生cost(穴=不可侵)とは別物（こちらは表示専用）。
    """
    cost = np.asarray(cost, dtype=np.float32)
    h, w = cost.shape
    valid_data = ~np.asarray(nodata_mask, dtype=bool)  # 点があったセル

    # footprint = データ領域を「面」として連結（スパースな点群でもドット化しないように
    # クロージングで点間を橋渡し → 内側の穴を塞ぐ）。橋渡し半径は点密度から推定。
    if valid_data.any():
        frac = float(valid_data.mean())
        radius = int(np.clip(np.ceil(1.0 / np.sqrt(max(frac, 1e-6))), 2, 30))
        closed = binary_closing(valid_data, structure=np.ones((3, 3), dtype=bool), iterations=radius)
        footprint = binary_fill_holes(closed)
    else:
        footprint = valid_data

    # 内側の穴を最近傍の有効値で埋める
    filled = cost
    if valid_data.any() and (~valid_data).any():
        idx = distance_transform_edt(~valid_data, return_distances=False, return_indices=True)
        filled = cost[tuple(idx)]

    finite = footprint & np.isfinite(filled) & (filled < obstacle_value)
    if vmax is None or vmax <= 0:
        vals = filled[finite]
        vmax = float(np.percentile(vals, 98)) if vals.size else 1.0
        if not np.isfinite(vmax) or vmax <= 0:
            vmax = 1.0

    rgba = np.zeros((h, w, 4), dtype=np.uint8)
    t = np.clip(filled / vmax, 0.0, 1.0)
    first = finite & (t <= 0.5)
    second = finite & (t > 0.5)
    rgba[first, 0] = (255 * (t[first] / 0.5)).astype(np.uint8)
    rgba[first, 1] = 255
    rgba[second, 0] = 255
    rgba[second, 1] = (255 * (1.0 - (t[second] - 0.5) / 0.5)).astype(np.uint8)
    # 不可侵（footprint内・走行不可）はグレー
    obst = footprint & ~finite
    rgba[obst, 0] = 110
    rgba[obst, 1] = 110
    rgba[obst, 2] = 110
    # alpha: footprint内=不透明 / 範囲外=透明
    rgba[..., 3] = np.where(footprint, 255, 0).astype(np.uint8)
    return rgba
